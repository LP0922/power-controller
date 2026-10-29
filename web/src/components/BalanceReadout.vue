<template>
  <div class="readout" :class="{ offline: !online }">
    <div class="head">
      <span class="title">天平实时读数</span>
      <el-tag size="small" :type="online ? 'success' : 'danger'" effect="dark">
        {{ online ? '在线' : '离线' }}
      </el-tag>
      <el-tag size="small" :type="stableTag" effect="plain">{{ stableText }}</el-tag>
      <span class="hint">AT8811C · COM11 · 1 count = 1 mg · 0.5s 刷新</span>
    </div>

    <div class="value">
      <strong class="gram">{{ gramText }}</strong><span class="unit">g</span>
      <span class="mg">{{ mgText }} mg</span>
      <span v-if="deltaText" class="delta" :class="deltaClass">{{ deltaText }}</span>
    </div>

    <div class="tail">
      <span class="at">{{ store.bal.at || '—' }}</span>
      <el-button size="small" type="primary" plain :disabled="locked || !online" :loading="busy"
        @click="tare">
        天平标零
      </el-button>
    </div>

    <el-alert v-if="store.bal.error" type="error" :closable="false" show-icon class="alarm"
      :title="`天平读取异常：${store.bal.error}`" />
  </div>
</template>

<script setup>
import { computed, ref } from 'vue'
import { ElMessage } from 'element-plus'
import { useDeviceStore } from '../stores/device.js'
import { tareBalance, errorMessage } from '../api/index.js'

/**
 * 天平实时读数条（只读取 /api/status 的 bal 段，标定、加粉页都能挂）。
 *
 * 为什么不自己开定时器：设备 store 已经在轮询 500ms（布局层 + 页面各持一份引用计数），
 * 这里再单独计时会出现两条节奏、数字跳动不一致。所以只做展示 + 一个标零按钮。
 */
const store = useDeviceStore()
const busy = ref(false)

const online = computed(() => store.balanceOnline)
const hasValue = computed(() => store.bal.mg != null)
const gramText = computed(() => (hasValue.value ? Number(store.bal.g).toFixed(3) : '—'))
const mgText = computed(() => (hasValue.value ? Number(store.bal.mg).toFixed(1) : '—'))
const stableText = computed(() => {
  if (store.bal.online === false) return '离线'
  if (!hasValue.value) return '无读数'
  return store.bal.stable ? '稳定' : '波动'
})
const stableTag = computed(() => {
  if (store.bal.online === false) return 'danger'
  return store.bal.stable ? 'success' : 'warning'
})

/** 相对基准的增量：标定时开了「记录基准」就有，没开就不显示 */
const deltaText = computed(() => {
  if (!hasValue.value) return ''
  const delta = store.bal.delta_mg
  if (delta === null || delta === undefined) return ''
  const d = Number(delta)
  return `${d > 0 ? '+' : ''}${d.toFixed(1)} mg`
})
const deltaClass = computed(() => (Number(store.bal.delta_mg) >= 0 ? 'up' : 'down'))

/** 标定/加粉运行中后端会拒绝清零（会把变化中的量当皮重扣掉），这里提前置灰 */
const locked = computed(() => store.manualLocked)

async function tare() {
  busy.value = true
  try {
    const r = await tareBalance()
    ElMessage.success(`天平已标零：清零前 ${r.tare_before_mg} mg → 现在 ${r.mg} mg`)
    await store.fetchStatus()
  } catch (error) {
    ElMessage.error(errorMessage(error))
  } finally {
    busy.value = false
  }
}
</script>

<style scoped>
.readout {
  display: flex;
  align-items: center;
  gap: 18px;
  flex-wrap: wrap;
  padding: 10px 18px;
  background: #fff;
  border: 1px solid #e4e7ed;
  border-left: 3px solid #409eff;
  border-radius: 8px;
  position: sticky;
  top: 0;
  z-index: 6;
}
.readout.offline { border-left-color: #dc2626; background: #fffafa; }
.head { display: flex; align-items: center; gap: 8px; flex-wrap: wrap; }
.title { font-size: 13px; font-weight: 600; color: #303133; }
.hint { font-size: 11.5px; color: #a3aebd; }
.value { display: flex; align-items: baseline; gap: 6px; margin-left: auto; }
.gram {
  font-size: 30px;
  font-weight: 700;
  line-height: 1;
  color: #303133;
  font-variant-numeric: tabular-nums;
  letter-spacing: 0.5px;
}
.offline .gram { color: #c0c4cc; }
.unit { font-size: 13px; color: #8b9aab; margin-right: 6px; }
.mg {
  font-size: 14px;
  color: #409eff;
  font-variant-numeric: tabular-nums;
  margin-right: 8px;
}
.delta { font-size: 13px; font-variant-numeric: tabular-nums; }
.delta.up { color: #dc2626; }
.delta.down { color: #059669; }
.tail { display: flex; align-items: center; gap: 10px; }
.at { font-size: 11.5px; color: #a3aebd; font-variant-numeric: tabular-nums; }
.alarm { width: 100%; margin-top: 4px; }
@media (max-width: 900px) {
  .value { margin-left: 0; }
  .gram { font-size: 24px; }
}
</style>
