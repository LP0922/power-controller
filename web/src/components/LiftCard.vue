<template>
  <PanelSection title="升降电机（双机同步）">
    <template #header-actions>
      <el-tag size="small" effect="plain" type="info">MD30 ×2 · COM10 · 广播 0x00</el-tag>
      <el-tag size="small" :type="moving ? 'warning' : 'success'" effect="plain">
        {{ moving ? '运动中' : '停止' }}
      </el-tag>
    </template>

    <p class="note">
      两台电缸必须同时运动：广播一帧同发 + 起跳监督 + 运行中偏差看门狗（超限自动急停）。
      行程 20–910（≈0–28mm），速度固定不可调。
      <strong>搅拌电机运行时禁止升降</strong>（后端硬拦，按钮同时置灰）。
    </p>

    <el-alert v-if="alarm" type="error" :closable="false" show-icon class="alarm">
      <template #title>
        【{{ alarm.time }}】{{ alarm.message }}
        <span v-if="alarm.positions">（当时位置 {{ alarm.positions.join('/') }}）</span>
      </template>
      <el-button size="small" @click="ack">确认恢复</el-button>
    </el-alert>

    <el-alert v-if="store.liftLocked" type="warning" :closable="false" show-icon class="alarm"
      title="搅拌正在运行，升降运动已锁定：请先停搅拌，等实时转速归零。" />

    <el-form label-width="88px" class="form">
      <el-form-item label="目标位置">
        <div class="target-row">
          <el-slider v-model="target" :min="20" :max="910" :step="1" :disabled="locked"
            class="slider" @input="syncFromSlider" />
          <el-input-number v-model="target" :min="20" :max="910" :step="1" :disabled="locked"
            class="num" />
          <span class="mm">≈{{ mm }}mm</span>
        </div>
      </el-form-item>
    </el-form>

    <div class="actions">
      <el-button type="primary" :disabled="locked" :loading="busy" @click="move">同步运动</el-button>
      <el-button type="success" plain :disabled="locked" :loading="busy" @click="home">回原位 (33)</el-button>
      <el-button :disabled="busy" @click="stop">停止</el-button>
      <el-button :disabled="locked" :loading="busy" @click="align">对齐双机</el-button>
    </div>

    <div class="metrics">
      <MetricCard label="从站1 位置" :value="pos(0)" />
      <MetricCard label="从站2 位置" :value="pos(1)" />
      <MetricCard label="双机偏差" :value="devText" :subtitle="`容差 ${store.sync.tol ?? '—'}`" />
      <MetricCard label="运动状态" :value="moving ? '运动中' : '停止'" />
    </div>
  </PanelSection>
</template>

<script setup>
import { computed, ref, watch } from 'vue'
import { ElMessage } from 'element-plus'
import { useDeviceStore } from '../stores/device.js'
import { moveLift, homeLift, stopLift, alignLift, ackSyncAlarm, errorMessage } from '../api/index.js'
import PanelSection from './PanelSection.vue'
import MetricCard from './MetricCard.vue'

const store = useDeviceStore()
const target = ref(33)
const busy = ref(false)

const locked = computed(() => store.manualLocked || store.liftLocked)
const mm = computed(() => ((target.value - 20) / 31.7).toFixed(1))
const moving = computed(() => store.lift.some((axis) => axis && axis.moving))
const alarm = computed(() => store.sync.alarm || null)
const devText = computed(() => {
  const dev = store.sync.dev
  return dev === null || dev === undefined ? '—' : String(dev)
})
const pos = (index) => {
  const axis = store.lift[index]
  return axis ? String(axis.pos) : '—'
}

// 只在"页面还没被手动改过"时跟着真机走，避免操作员正在拖滑块却被轮询抢回去
const touched = ref(false)
watch(() => store.lift[0]?.pos, (value) => {
  if (!touched.value && Number.isFinite(value)) target.value = value
})
function syncFromSlider() { touched.value = true }
watch(moving, (value) => { if (!value) touched.value = false })

async function run(fn, okText) {
  busy.value = true
  try {
    await fn()
    ElMessage.success(okText)
    await store.fetchStatus()
  } catch (error) {
    ElMessage.error(errorMessage(error))
  } finally {
    busy.value = false
  }
}

const move = () => { touched.value = true; run(() => moveLift(target.value), `升降指令已下发 → ${target.value}`) }
const home = () => run(homeLift, '升降回原位指令已下发 → 33')
const stop = () => run(stopLift, '升降停止指令已下发')
const align = () => run(alignLift, '双机已对齐')
const ack = () => run(ackSyncAlarm, '同步告警已确认恢复')
</script>

<style scoped>
.note { margin: 0 0 14px; font-size: 12px; line-height: 1.75; color: #738196; }
.alarm { margin-bottom: 12px; }
.form { margin-bottom: 4px; }
.target-row { display: flex; align-items: center; gap: 14px; width: 100%; }
.slider { flex: 1; min-width: 120px; }
.num { width: 110px; }
.mm { font-size: 13px; color: #52657d; font-variant-numeric: tabular-nums; min-width: 76px; }
.actions { display: flex; flex-wrap: wrap; gap: 8px; margin: 12px 0 18px; }
.actions :deep(.el-button + .el-button) { margin-left: 0; }
.metrics { display: grid; grid-template-columns: repeat(4, minmax(0, 1fr)); gap: 10px; }
.metrics :deep(.value-text) { font-size: 22px; }
@media (max-width: 1100px) { .metrics { grid-template-columns: repeat(2, minmax(0, 1fr)); } }
</style>
