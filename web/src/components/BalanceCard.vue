<template>
  <PanelSection title="天平（称重）">
    <template #header-actions>
      <el-tag size="small" effect="plain" type="info">AT8811C · COM11 · Modbus RTU 9600</el-tag>
      <el-tag size="small" :type="stableTag" effect="plain">{{ stableText }}</el-tag>
    </template>

    <p class="note">
      1 count = 1 mg（320 g / 1 mg 型）。稳定位与零点带取自状态字。
      「去皮归零」直接写天平实时重量寄存器为 0：振动/搅拌/推杆/升降只要有一项在动就会被后端拒绝，
      因为清零会把正在变化的量当成皮重扣掉。
    </p>

    <div class="weigh" :class="{ offline: !store.balanceOnline }">
      <div class="big">
        <strong>{{ gramText }}</strong><span class="unit">g</span>
      </div>
      <div class="sub">
        <b>{{ mgText }}</b> mg ·
        <span :class="stableTag === 'success' ? 'ok' : 'warn'">{{ stableText }}</span>
      </div>
    </div>

    <el-alert v-if="store.bal.error" type="error" :closable="false" show-icon class="alarm"
      :title="`天平读取异常：${store.bal.error}`" />

    <el-form label-width="88px" class="form">
      <el-form-item label="目标质量">
        <div class="inline">
          <el-input-number v-model="target" :min="0" :max="320000" :step="10" />
          <span class="lbl">mg</span>
          <span class="remain" :class="remainClass">{{ remainText }}</span>
        </div>
      </el-form-item>
    </el-form>

    <div class="actions">
      <el-button type="primary" :disabled="locked" :loading="busy" @click="tare">去皮归零</el-button>
      <el-button :disabled="locked" :loading="busy" @click="base">记录基准</el-button>
      <el-button :disabled="locked" :loading="busy" @click="clearBase">清除基准</el-button>
      <el-button type="success" plain :disabled="locked" :loading="busy" @click="stableSample">稳定采样</el-button>
      <el-button :disabled="busy" :loading="busy" @click="refresh">刷新</el-button>
    </div>

    <div class="metrics">
      <MetricCard label="原始计数" :value="fmt(store.bal.raw)" />
      <MetricCard label="基准 / Δ 增量" :value="baseText" />
      <MetricCard label="零点带" :value="zeroText" />
      <MetricCard label="刷新时间" :value="store.bal.at || '—'" />
    </div>
  </PanelSection>
</template>

<script setup>
import { computed, ref } from 'vue'
import { ElMessage, ElMessageBox } from 'element-plus'
import { useDeviceStore } from '../stores/device.js'
import {
  tareBalance, refreshBalance, setBalanceBase, sampleStableBalance, errorMessage,
} from '../api/index.js'
import PanelSection from './PanelSection.vue'
import MetricCard from './MetricCard.vue'

const store = useDeviceStore()
const target = ref(100)
const busy = ref(false)
const locked = computed(() => store.manualLocked)
const fmt = (v) => (v === null || v === undefined ? '—' : String(v))

const hasValue = computed(() => store.bal.mg != null)
const gramText = computed(() => (hasValue.value ? Number(store.bal.g).toFixed(3) : '—'))
const mgText = computed(() => (hasValue.value ? Number(store.bal.mg).toFixed(1) : '—'))
const stableText = computed(() => {
  if (store.bal.online === false) return '离线'
  return store.bal.stable ? '稳定' : '波动'
})
const stableTag = computed(() => {
  if (store.bal.online === false) return 'danger'
  return store.bal.stable ? 'success' : 'warning'
})
const zeroText = computed(() => {
  const z = store.bal.zero_band
  if (z === null || z === undefined) return '—'
  return z ? '在零点带' : '偏离零点'
})
const baseText = computed(() => {
  const base = store.bal.base
  const delta = store.bal.delta_mg
  if (!base || delta === null || delta === undefined) return '未设基准'
  return `${delta > 0 ? '+' : ''}${delta} / ${base.mg}`
})
const remainText = computed(() => {
  if (!hasValue.value || !target.value) return '—'
  const remain = target.value - store.bal.mg
  return remain >= 0 ? `还差 ${remain.toFixed(1)} mg` : `超出 ${(-remain).toFixed(1)} mg`
})
const remainClass = computed(() => {
  if (!hasValue.value || !target.value) return ''
  return target.value - store.bal.mg >= 0 ? 'need' : 'over'
})

async function run(fn, okText) {
  busy.value = true
  try {
    const result = await fn()
    if (okText) ElMessage.success(typeof okText === 'function' ? okText(result) : okText)
    await store.fetchStatus()
  } catch (error) {
    ElMessage.error(errorMessage(error))
  } finally {
    busy.value = false
  }
}

async function tare() {
  try {
    await ElMessageBox.confirm(
      '去皮归零会把天平当前重量写成 0。\n执行前请确认秤盘状态是你想要的皮重（空盘或容器），且所有机构已停止。',
      '去皮归零确认', { type: 'warning' })
  } catch { return }
  run(tareBalance, (r) => `天平已去皮归零：清零前 ${r.tare_before_mg} mg → 现在 ${r.mg} mg`)
}
const base = () => run(() => setBalanceBase(false), (r) => `已记录基准 ${r.base.mg} mg`)
const clearBase = () => run(() => setBalanceBase(true), '基准已清除')
const stableSample = () => run(
  () => sampleStableBalance({ timeout_s: 6, samples: 8 }),
  (r) => (r.stable
    ? `稳定采样完成：${r.mean_mg} mg（${r.samples} 个样本，标准差 ${r.sd_mg} mg，范围 ${r.min_mg}–${r.max_mg}）`
    : `未在超时内判稳：均值 ${r.mean_mg} mg、标准差 ${r.sd_mg} mg（${r.samples} 个样本），请等读数稳定再试`))
const refresh = () => run(refreshBalance, (r) => `天平已刷新：${r.g} g（${r.stable ? '稳定' : '波动'}）`)
</script>

<style scoped>
.note { margin: 0 0 14px; font-size: 12px; line-height: 1.75; color: #738196; }
.alarm { margin-bottom: 12px; }
.weigh { border: 1px solid #cde1fa; border-radius: 12px; padding: 20px 24px; background: linear-gradient(120deg, #edf5ff, #f8fbff); margin-bottom: 16px; }
.offline { background: #f5f7fa; border-color: #e0e6ed; }
.offline .big strong, .offline .sub { color: #97a4b4; }
.big { display: flex; align-items: baseline; gap: 10px; color: #1c66ba; font-variant-numeric: tabular-nums; }
.big strong { font-size: clamp(34px, 3.6vw, 48px); line-height: 1.1; letter-spacing: -1px; }
.unit { font-size: 17px; }
.sub { margin-top: 8px; font-size: 13px; color: #45688c; }
.sub b { font-size: 15px; font-variant-numeric: tabular-nums; }
.ok { color: #059669; }
.warn { color: #d97706; }
.form { margin-bottom: 4px; }
.inline { display: flex; align-items: center; gap: 10px; flex-wrap: wrap; }
.lbl { font-size: 13px; color: #606266; }
.remain { font-size: 13px; font-variant-numeric: tabular-nums; }
.need { color: #2563eb; }
.over { color: #d97706; }
.actions { display: flex; flex-wrap: wrap; gap: 8px; margin: 12px 0 18px; }
.actions :deep(.el-button + .el-button) { margin-left: 0; }
.metrics { display: grid; grid-template-columns: repeat(4, minmax(0, 1fr)); gap: 10px; }
.metrics :deep(.value-text) { font-size: 20px; }
@media (max-width: 1100px) { .metrics { grid-template-columns: repeat(2, minmax(0, 1fr)); } }
</style>
