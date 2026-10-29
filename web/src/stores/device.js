import { defineStore } from 'pinia'
import { computed, ref } from 'vue'
import { getStatus } from '../api/index.js'

/**
 * 设备状态（目前后端 /api/status 一条接口全拿）。
 *
 * 2026-10-26 融合：这里以前读的是老后端的 /api/snapshot（at8811c / la10 / test
 * 三段结构）。新版后端的状态语义完全不同 —— 升降是双机数组、推杆/搅拌/天平/信号源
 * 各一段、加粉与标定单独各一段 —— 所以整个 store 换掉，字段名跟后端保持一致，
 * 页面不再做一层翻译（翻译层只留给还在用 /api/snapshot 的老代码）。
 */
export const useDeviceStore = defineStore('device', () => {
  const status = ref({
    ok: false,
    errors: [],
    lift: [],
    lift_link: [],
    sync: { dev: null, tol: 20, alarm: null },
    rod: {},
    stir: {},
    bal: {},
    sig: {},
    dispense: {},
    calib: {},
  })

  const error = ref('')
  const lastAt = ref(null)

  const bal = computed(() => status.value.bal || {})
  const rod = computed(() => status.value.rod || {})
  const stir = computed(() => status.value.stir || {})
  const sig = computed(() => status.value.sig || {})
  const lift = computed(() => status.value.lift || [])
  const sync = computed(() => status.value.sync || {})
  const errors = computed(() => status.value.errors || [])
  const dispense = computed(() => status.value.dispense || {})
  const calib = computed(() => status.value.calib || {})

  /** 天平：在线且读到了数 */
  const balanceOnline = computed(() => bal.value.online === true && bal.value.mg != null)
  /** 执行机构任意一个在线就算链路通 */
  const actuatorOnline = computed(() => (
    lift.value.length > 0 || rod.value.pos != null || stir.value.rpm != null
  ))

  const dispenseRunning = computed(() => dispense.value.running === true)
  const calibRunning = computed(() => calib.value.running === true)
  /** 搅拌在转（±1 rpm 以内当静止） */
  const stirRunning = computed(() => (
    stir.value.rpm != null && Math.abs(stir.value.rpm) > 1
  ))
  /** 加粉或标定运行中：除「停止」「急停」外的手动按钮统一锁 */
  const manualLocked = computed(() => dispenseRunning.value || calibRunning.value)
  /** 搅拌在转时禁止升降（后端硬拦，这里提前置灰） */
  const liftLocked = computed(() => stirRunning.value)

  const dispenseLive = computed(() => dispense.value.live || {})
  const dispenseResult = computed(() => dispense.value.result || null)
  const dispenseConfig = computed(() => dispense.value.config || {})

  let timer = null
  let inFlight = null
  let intervalMs = 500
  // 引用计数：布局层和页面都会 startPolling，谁先卸载都不能把别人的定时器停掉。
  // （没有这个计数时，从「设备监控」切到别的页会 stopPolling，布局层却不重新 mount，
  //   整套轮询就永久停了 —— 页面看着像"设备全离线"。）
  let refs = 0

  function fetchStatus() {
    if (inFlight) return inFlight
    inFlight = getStatus()
      .then((data) => {
        status.value = data
        error.value = ''
        lastAt.value = new Date()
      })
      .catch(() => {
        // 保留上一次的数据（停止按钮还要能点），但明确标出"不是实时的"
        error.value = '无法获取设备状态（后端 8765 未响应）'
      })
      .finally(() => { inFlight = null })
    return inFlight
  }

  function startPolling(ms = 500) {
    refs += 1
    if (timer) return            // 已经在跑：只加引用，不重置节奏
    intervalMs = ms
    fetchStatus()
    timer = setInterval(fetchStatus, intervalMs)
  }

  function stopPolling() {
    refs = Math.max(0, refs - 1)
    if (refs === 0 && timer) { clearInterval(timer); timer = null }
  }

  return {
    status, error, lastAt,
    bal, rod, stir, sig, lift, sync, errors, dispense, calib,
    balanceOnline, actuatorOnline,
    dispenseRunning, calibRunning, stirRunning, manualLocked, liftLocked,
    dispenseLive, dispenseResult, dispenseConfig,
    fetchStatus, startPolling, stopPolling,
  }
})
