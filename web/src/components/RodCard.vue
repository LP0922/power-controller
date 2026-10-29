<template>
  <PanelSection title="推杆">
    <template #header-actions>
      <el-tag size="small" effect="plain" type="info">因时 LA · COM10 · ID 3</el-tag>
    </template>

    <p class="note">
      LA 私有协议（55 AA 帧）。行程 0–2000，中程约 0.6s，落点 ±1。
      位置只在动作后 8s 内实时刷新，平时每 10s 兜底读一次（把 COM10 总线让给升降双机）；
      要看即时值点「刷新」。
    </p>

    <el-form label-width="88px" class="form">
      <el-form-item label="目标位置">
        <div class="target-row">
          <el-slider v-model="target" :min="0" :max="2000" :step="1" :disabled="locked" class="slider" />
          <el-input-number v-model="target" :min="0" :max="2000" :step="1" :disabled="locked" class="num" />
          <span class="pct">{{ Math.round(target / 20) }}%</span>
        </div>
      </el-form-item>
    </el-form>

    <div class="actions">
      <el-button type="primary" :disabled="locked" :loading="busy" @click="move">运动</el-button>
      <el-button type="success" plain :disabled="locked" :loading="busy" @click="home">回原位 (208)</el-button>
      <el-button type="danger" plain :disabled="busy" @click="stop">急停</el-button>
      <el-button :disabled="busy" @click="refresh">刷新</el-button>
    </div>

    <div class="metrics">
      <MetricCard label="当前位置" :value="fmt(store.rod.pos)" />
      <MetricCard label="目标位置" :value="fmt(store.rod.target)" />
      <MetricCard label="温度" :value="fmt(store.rod.temp)" unit="℃" />
      <MetricCard label="电流" :value="fmt(store.rod.current)" unit="mA" />
    </div>

    <p v-if="faultText" class="fault">⚠ {{ faultText }}</p>
  </PanelSection>
</template>

<script setup>
import { computed, ref } from 'vue'
import { ElMessage } from 'element-plus'
import { useDeviceStore } from '../stores/device.js'
import { moveRod, homeRod, stopRod, refreshRod, errorMessage } from '../api/index.js'

import PanelSection from './PanelSection.vue'
import MetricCard from './MetricCard.vue'

const store = useDeviceStore()
const target = ref(500)
const busy = ref(false)
const locked = computed(() => store.manualLocked)
const fmt = (v) => (v === null || v === undefined ? '—' : String(v))

// LA 的 error 字节只有低 4 位有定义；实测这台常报 4/8/12，按位提示不阻断动作
const faultText = computed(() => {
  const err = store.rod.error
  if (!err) return ''
  const bits = []
  if (err & 0x01) bits.push('bit0')
  if (err & 0x02) bits.push('bit1')
  if (err & 0x04) bits.push('bit2')
  if (err & 0x08) bits.push('bit3')
  return `推杆上报 error=${err}（${bits.join('+') || '未定义位'}）——设备原值，不影响下发指令`
})

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
const move = () => run(() => moveRod(target.value), `推杆指令已下发 → ${target.value}`)
const home = () => run(homeRod, '推杆回原位指令已下发 → 208')
const stop = () => run(stopRod, '推杆已急停（下次点运动会自动重新使能）')
const refresh = () => run(refreshRod, '推杆状态已强制刷新')
</script>

<style scoped>
.note { margin: 0 0 14px; font-size: 12px; line-height: 1.75; color: #738196; }
.form { margin-bottom: 4px; }
.target-row { display: flex; align-items: center; gap: 14px; width: 100%; }
.slider { flex: 1; min-width: 110px; }
.num { width: 110px; }
.pct { font-size: 13px; color: #52657d; min-width: 46px; font-variant-numeric: tabular-nums; }
.actions { display: flex; flex-wrap: wrap; gap: 8px; margin: 12px 0 18px; }
.actions :deep(.el-button + .el-button) { margin-left: 0; }
.metrics { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 10px; }
.metrics :deep(.value-text) { font-size: 22px; }
.fault { margin: 12px 0 0; font-size: 12px; color: #d97706; line-height: 1.6; }
</style>
