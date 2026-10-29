<template>
  <div class="monitor-view">
    <!-- 顶部状态条：连接、设备错误、全部急停 -->
    <div class="status-bar">
      <div class="status-left">
        <span class="dot" :class="ok ? 'on' : 'off'" />
        <span class="status-text">
          {{ store.error || (ok ? `已连接 · ${clock}` : '未连接后端') }}
        </span>
        <el-tag v-if="store.manualLocked" type="warning" size="small" effect="dark">
          {{ store.dispenseRunning ? '加粉运行中' : '标定运行中' }} · 手动动作已锁定
        </el-tag>
        <el-tag v-if="store.stirRunning" type="warning" size="small" effect="plain">
          搅拌旋转中 · 升降已锁
        </el-tag>
      </div>
      <div class="status-right">
        <el-button type="danger" plain size="small" :loading="estopping" @click="estop">
          全部急停
        </el-button>
      </div>
    </div>

    <el-alert v-if="store.errors.length" type="error" :closable="false" show-icon class="errors"
      :title="store.errors.join('　|　')" />

    <div class="grid">
      <LiftCard />
      <RodCard />
    </div>
    <StirCard class="block" />
    <BalanceCard class="block" />
    <SignalCard class="block" />
  </div>
</template>

<script setup>
import { computed, onMounted, onUnmounted, ref } from 'vue'
import { ElMessage, ElMessageBox } from 'element-plus'
import { useDeviceStore } from '../stores/device.js'
import { emergencyStop, errorMessage } from '../api/index.js'
import LiftCard from '../components/LiftCard.vue'
import RodCard from '../components/RodCard.vue'
import StirCard from '../components/StirCard.vue'
import BalanceCard from '../components/BalanceCard.vue'
import SignalCard from '../components/SignalCard.vue'

const store = useDeviceStore()
const estopping = ref(false)
const clock = ref('')
let clockTimer = null

const ok = computed(() => store.status.ok === true && !store.error)

function updateClock() {
  clock.value = new Date().toLocaleTimeString('zh-CN', { hour12: false })
}

async function estop() {
  try {
    await ElMessageBox.confirm(
      '全部急停：搅拌立即停 + 推杆急停 + 升降停止 + 振动关闭。\n确认执行？',
      '全部急停', { type: 'warning' })
  } catch { return }
  estopping.value = true
  try {
    await emergencyStop()
    ElMessage.success('全部急停已执行')
    await store.fetchStatus()
  } catch (error) {
    ElMessage.error(errorMessage(error))
  } finally {
    estopping.value = false
  }
}

onMounted(() => {
  // 500ms 跟单页面板一致：再快也只是重复读同一批串口缓存
  store.startPolling(500)
  updateClock()
  clockTimer = setInterval(updateClock, 1000)
})
onUnmounted(() => {
  store.stopPolling()
  if (clockTimer) { clearInterval(clockTimer); clockTimer = null }
})
</script>

<style scoped>
.monitor-view { display: flex; flex-direction: column; gap: 16px; min-width: 0; }
.status-bar {
  display: flex; align-items: center; justify-content: space-between; gap: 16px;
  background: #fff; border: 1px solid #e4e7ed; border-radius: 8px; padding: 10px 16px;
}
.status-left { display: flex; align-items: center; gap: 12px; flex-wrap: wrap; }
.status-right { flex-shrink: 0; }
.dot { width: 8px; height: 8px; border-radius: 50%; background: #c0c4cc; flex-shrink: 0; }
.dot.on { background: #67c23a; box-shadow: 0 0 0 3px rgba(103, 194, 58, 0.16); }
.dot.off { background: #f56c6c; }
.status-text { font-size: 13px; color: #606266; }
.errors { margin: 0; }
.grid { display: grid; grid-template-columns: 1fr 1fr; gap: 16px; min-width: 0; }
.block { min-width: 0; }
@media (max-width: 1200px) {
  .grid { grid-template-columns: 1fr; }
}
</style>
