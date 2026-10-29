<template>
  <PanelSection title="旋转搅拌电机">
    <template #header-actions>
      <el-tag size="small" effect="plain" type="info">ZDT Emm42 · COM9 · 私有 0x6B</el-tag>
      <el-tag size="small" :type="spinning ? 'warning' : 'success'" effect="plain">
        {{ spinning ? '旋转中' : '静止' }}
      </el-tag>
    </template>

    <p class="note">
      速度模式连续旋转。0–1000 RPM 可直接启动；更高转速必须用曲线启动（加速度 ≥200），否则会失步。
      减速停 = 速度写 0 + 同一加速度曲线；<b>转速低于 50 RPM 时一律改成立即停</b>（低速没多少惯性，
      多花的时间只会让转子上的粉继续往下落）。堵转保护是锁存的，故障位消了也要点一次解除才接受新指令。
    </p>

    <el-form label-width="88px" class="form">
      <el-form-item label="转速 RPM">
        <div class="inline">
          <el-input-number v-model="rpm" :min="0" :max="3000" :step="10" :disabled="locked" />
          <span class="lbl">方向</span>
          <el-select v-model="direction" :disabled="locked" style="width: 110px">
            <el-option :value="0" label="顺时针" />
            <el-option :value="1" label="逆时针" />
          </el-select>
          <span class="lbl">加速度</span>
          <el-select v-model="acc" :disabled="locked" style="width: 210px">
            <el-option :value="0" label="0（直启，≤1000 RPM）" />
            <el-option :value="200" label="200（曲线，~357 RPM/s）" />
            <el-option :value="220" label="220（曲线，较快）" />
            <el-option :value="100" label="100（曲线，较缓）" />
          </el-select>
        </div>
      </el-form-item>
    </el-form>

    <div class="actions">
      <el-button type="primary" :disabled="locked" :loading="busy" @click="start">启动</el-button>
      <el-button :disabled="busy" @click="stopDecel">减速停止</el-button>
      <el-button :disabled="busy" @click="stopImm">立即停止</el-button>
      <el-button type="warning" plain :disabled="busy" @click="clearStall">解除堵转保护</el-button>
    </div>

    <div class="metrics">
      <MetricCard label="实时转速" :value="fmt(store.stir.rpm)" unit="RPM" />
      <MetricCard label="使能状态" :value="store.stir.enabled ? '已使能' : '未使能'" />
      <MetricCard label="到位标志" :value="flagText" />
      <MetricCard label="故障位" :value="errText" />
    </div>
  </PanelSection>
</template>

<script setup>
import { computed, ref } from 'vue'
import { ElMessage } from 'element-plus'
import { useDeviceStore } from '../stores/device.js'
import { startStir, stopStir, clearStirStall, errorMessage } from '../api/index.js'
import PanelSection from './PanelSection.vue'
import MetricCard from './MetricCard.vue'

const store = useDeviceStore()
const rpm = ref(300)
const direction = ref(0)
const acc = ref(200)
const busy = ref(false)
const locked = computed(() => store.manualLocked)
const spinning = computed(() => store.stirRunning)
const fmt = (v) => (v === null || v === undefined ? '—' : String(v))

// flags 低 2 位：bit0 使能、bit1 到位；bit2..3 是堵转/堵转保护
const FLAG_TEXT = { 0: '使能+到位', 1: '使能', 2: '到位', 3: '使能+到位' }
const flagText = computed(() => {
  const flags = store.stir.flags
  return flags === null || flags === undefined ? '—' : (FLAG_TEXT[flags] || String(flags))
})
const errText = computed(() => {
  const flags = store.stir.flags
  if (flags === null || flags === undefined) return '—'
  const err = (flags >> 2) & 0x03
  if (!err) return '无'
  return (err & 1 ? '堵转 ' : '') + (err & 2 ? '堵转保护' : '')
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
const start = () => run(
  () => startStir({ rpm: rpm.value, acc: acc.value, direction: direction.value }),
  `搅拌启动指令已下发 ${rpm.value} RPM`)
const stopDecel = () => run(
  () => stopStir({ mode: 'decel', acc: acc.value || 200 }),
  '搅拌停止指令已下发（转速 < 50 RPM 时按立即停处理）')
const stopImm = () => run(() => stopStir({ mode: 'immediate' }), '搅拌已立即停止')
const clearStall = () => run(clearStirStall, '堵转保护解除指令已下发（状态看下方故障位）')
</script>

<style scoped>
.note { margin: 0 0 14px; font-size: 12px; line-height: 1.75; color: #738196; }
.form { margin-bottom: 4px; }
.inline { display: flex; align-items: center; gap: 10px; flex-wrap: wrap; }
.lbl { font-size: 13px; color: #606266; }
.actions { display: flex; flex-wrap: wrap; gap: 8px; margin: 12px 0 18px; }
.actions :deep(.el-button + .el-button) { margin-left: 0; }
.metrics { display: grid; grid-template-columns: repeat(4, minmax(0, 1fr)); gap: 10px; }
.metrics :deep(.value-text) { font-size: 22px; }
@media (max-width: 1100px) { .metrics { grid-template-columns: repeat(2, minmax(0, 1fr)); } }
</style>
