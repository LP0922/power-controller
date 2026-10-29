<template>
  <PanelSection title="振动给料（信号发生器）">
    <template #header-actions>
      <el-tag size="small" effect="plain" type="info">QLS3600S · COM16 · 57600</el-tag>
      <el-tag size="small" :type="outputOn ? 'warning' : 'info'" effect="plain">
        {{ store.sig.output == null ? '未设置' : (outputOn ? '输出中' : '关闭') }}
      </el-tag>
    </template>

    <p class="note">
      OUT 输出驱动粉末站振动：幅度决定振动强度（≈喂粉速率），频率决定振动节拍。
      改动参数后需点「OUT 开」或「定时运行」才真正输出；定时到点自动关闭（异常/急停也保证关）。
      加粉/标定运行中，只有「OUT 关」「停止振动」可用 —— 振动由控制器接管。
    </p>

    <el-form label-width="80px" class="form">
      <el-form-item label="波形">
        <div class="waves">
          <el-button v-for="w in WAVES" :key="w.key"
            :type="waveKey === w.key ? 'primary' : 'default'"
            :disabled="locked" @click="setWave(w.key)">{{ w.label }}</el-button>
          <span class="hint">脉冲波 = 方波 + 占空比</span>
        </div>
      </el-form-item>
      <el-form-item label="参数">
        <div class="inline">
          <span class="lbl">频率</span>
          <el-input-number v-model="freq" :min="0.01" :max="5000000" :step="1" :disabled="locked" class="num" />
          <span class="unit">Hz</span>
          <span class="lbl">幅度</span>
          <el-input-number v-model="amp" :min="0.01" :max="20" :step="0.05" :precision="2" :disabled="locked" class="num" />
          <span class="unit">Vpp</span>
          <span class="lbl">占空</span>
          <el-input-number v-model="duty" :min="0.1" :max="99.9" :step="0.1" :precision="1" :disabled="locked" class="num" />
          <span class="unit">%</span>
          <el-button :disabled="locked" :loading="busy" @click="apply">应用参数</el-button>
        </div>
      </el-form-item>
      <el-form-item label="输出">
        <div class="inline">
          <el-button type="success" :disabled="locked" :loading="busy" @click="output(true)">OUT 开</el-button>
          <el-button :disabled="busy" @click="output(false)">OUT 关</el-button>
          <span class="lbl">定时</span>
          <el-input-number v-model="seconds" :min="1" :max="600" :step="1" :disabled="locked" class="num" />
          <span class="unit">秒</span>
          <el-button type="primary" plain :disabled="locked" :loading="busy" @click="runFor">定时运行</el-button>
          <el-button type="danger" plain :disabled="busy" @click="stopAll">停止振动</el-button>
        </div>
      </el-form-item>
    </el-form>

    <div class="metrics">
      <MetricCard label="当前波形" :value="store.sig.waveform_name || '未设置'" />
      <MetricCard label="频率 / 幅度" :value="faText" />
      <MetricCard label="OUT 输出" :value="store.sig.output == null ? '未设置' : (outputOn ? '输出中' : '关闭')" />
      <MetricCard label="定时剩余" :value="store.sig.remaining == null ? '—' : `${store.sig.remaining}s`" />
    </div>
  </PanelSection>
</template>

<script setup>
import { computed, ref, watch } from 'vue'
import { ElMessage } from 'element-plus'
import { useDeviceStore } from '../stores/device.js'
import {
  setSignalWaveform, setSignalFreq, setSignalAmp, setSignalDuty,
  setSignalOutput, runSignalFor, stopSignal, errorMessage,
} from '../api/index.js'
import PanelSection from './PanelSection.vue'
import MetricCard from './MetricCard.vue'

const WAVES = [
  { key: 'sine', label: '正弦波' },
  { key: 'square', label: '方波' },
  { key: 'pulse', label: '脉冲波' },
  { key: 'triangle', label: '三角波' },
]

const store = useDeviceStore()
const freq = ref(80)
const amp = ref(2)
const duty = ref(50)
const seconds = ref(5)
const busy = ref(false)
const waveKey = ref('square')
const locked = computed(() => store.manualLocked)
const outputOn = computed(() => store.sig.output === true)
const faText = computed(() => {
  const f = store.sig.freq
  const a = store.sig.amp
  if (f == null && a == null) return '—'
  return `${f ?? '—'} Hz / ${a ?? '—'} Vpp`
})

// 真机波形编号 → 页面按钮（0 正弦 / 1 三角 / 2 方波；脉冲波没有独立编号）
const WAVE_BY_ID = { 0: 'sine', 1: 'triangle', 2: 'square' }
watch(() => store.sig.waveform, (id) => {
  const key = WAVE_BY_ID[id]
  if (key) waveKey.value = key
}, { immediate: true })
// 参数跟着真机走（只在没被手动改过时同步，避免抢用户正在输的值）
const touched = ref(false)
watch(() => [store.sig.freq, store.sig.amp, store.sig.duty], ([f, a, d]) => {
  if (touched.value) return
  if (f != null) freq.value = f
  if (a != null) amp.value = a
  if (d != null) duty.value = d
}, { immediate: true })

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

const setWave = (key) => run(() => setSignalWaveform(key), `波形已切换为${
  WAVES.find((w) => w.key === key)?.label || key}（输出状态不变）`)

function apply() {
  touched.value = true
  return run(async () => {
    await setSignalFreq(freq.value)
    await setSignalAmp(amp.value)
    await setSignalDuty(duty.value)
    return { freq: freq.value, amp: amp.value, duty: duty.value }
  }, (r) => `参数已应用：${r.freq} Hz / ${r.amp} Vpp / 占空 ${r.duty}%（输出状态不变）`)
}

const output = (on) => run(() => setSignalOutput(on), on ? 'OUT 输出已开启' : 'OUT 输出已关闭')
const stopAll = () => run(stopSignal, '振动已停止（定时任务同时取消）')
const runFor = () => {
  touched.value = true
  return run(() => runSignalFor({
    seconds: seconds.value,
    frequency_hz: freq.value,
    amplitude_vpp: amp.value,
    waveform: waveKey.value,
  }), (r) => `振动已启动：${r.waveform_name || ''} ${r.freq}Hz ${r.amp}Vpp，运行 ${r.requested_s}s 后自动关闭`)
}
</script>

<style scoped>
.note { margin: 0 0 14px; font-size: 12px; line-height: 1.75; color: #738196; }
.form { margin-bottom: 4px; }
.waves { display: flex; align-items: center; gap: 8px; flex-wrap: wrap; }
.waves :deep(.el-button + .el-button) { margin-left: 0; }
.inline { display: flex; align-items: center; gap: 8px; flex-wrap: wrap; }
.inline :deep(.el-button + .el-button) { margin-left: 0; }
.lbl { font-size: 13px; color: #606266; }
.unit { font-size: 12px; color: #909399; }
.num { width: 118px; }
.hint { font-size: 12px; color: #909399; margin-left: 4px; }
.metrics { display: grid; grid-template-columns: repeat(4, minmax(0, 1fr)); gap: 10px; }
.metrics :deep(.value-text) { font-size: 20px; }
@media (max-width: 1100px) { .metrics { grid-template-columns: repeat(2, minmax(0, 1fr)); } }
</style>
