<template>
  <el-container class="admin-container">
    <!-- Sidebar -->
    <el-aside :width="isCollapse ? '64px' : '220px'" class="admin-sidebar">
      <div class="sidebar-header">
        <el-icon :size="22" class="sidebar-logo-icon"><Monitor /></el-icon>
        <span v-show="!isCollapse" class="sidebar-title">加粉设备监控</span>
      </div>
      <el-menu
        :default-active="activeMenu"
        :collapse="isCollapse"
        :collapse-transition="false"
        router
        background-color="#304156"
        text-color="#bfcbd9"
        active-text-color="#409eff"
      >
        <el-menu-item index="/monitor">
          <el-icon><Monitor /></el-icon>
          <template #title>设备监控</template>
        </el-menu-item>
        <el-menu-item index="/dispense">
          <el-icon><SetUp /></el-icon>
          <template #title>加粉控制</template>
        </el-menu-item>
        <el-menu-item index="/batch">
          <el-icon><List /></el-icon>
          <template #title>批量测试</template>
        </el-menu-item>
        <el-menu-item index="/powders">
          <el-icon><Collection /></el-icon>
          <template #title>粉末管理</template>
        </el-menu-item>
        <!-- 试验记录的数据源是 runs/*.json，目前后端还没有列表接口，先置灰 -->
        <el-menu-item index="/records" disabled title="试验记录依赖 runs 目录的列表接口，尚未接入">
          <el-icon><Document /></el-icon>
          <template #title>试验记录（待接入）</template>
        </el-menu-item>
      </el-menu>
    </el-aside>

    <!-- Main -->
    <el-container>
      <!-- Top Bar -->
      <el-header class="admin-header">
        <div class="header-left">
          <el-icon
            class="collapse-btn"
            :size="20"
            @click="isCollapse = !isCollapse"
          >
            <Fold v-if="!isCollapse" /><Expand v-else />
          </el-icon>
          <el-breadcrumb separator="/">
            <el-breadcrumb-item :to="{ path: '/' }">首页</el-breadcrumb-item>
            <el-breadcrumb-item>{{ currentTitle }}</el-breadcrumb-item>
          </el-breadcrumb>
        </div>
        <div class="header-right">
          <el-tag :type="balanceOnline ? 'success' : 'danger'" size="small" effect="dark">
            {{ balanceOnline ? '天平在线' : '天平离线' }}
          </el-tag>
          <el-tag :type="actuatorOnline ? 'success' : 'danger'" size="small" effect="dark">
            {{ actuatorOnline ? '执行器在线' : '执行器离线' }}
          </el-tag>
          <el-tag v-if="store.manualLocked" type="warning" size="small" effect="dark">
            {{ store.dispenseRunning ? '加粉运行中' : '标定运行中' }}
          </el-tag>
          <span class="header-time">{{ currentTime }}</span>
          <el-button :icon="Refresh" size="small" circle :loading="refreshing"
            @click="handleRefresh" />
          <el-button type="danger" size="small" :loading="estopping" @click="estop">
            全部急停
          </el-button>
        </div>
      </el-header>

      <!-- Content -->
      <el-main class="admin-main">
        <router-view />
      </el-main>
    </el-container>
  </el-container>
</template>

<script setup>
import { ref, computed, onMounted, onUnmounted } from 'vue'
import { useRoute } from 'vue-router'
import { Monitor, SetUp, List, Document, Collection, Fold, Expand, Refresh } from '@element-plus/icons-vue'
import { ElMessage, ElMessageBox } from 'element-plus'
import { useDeviceStore } from '../stores/device.js'
import { emergencyStop, errorMessage } from '../api/index.js'

const route = useRoute()
const store = useDeviceStore()
const isCollapse = ref(false)
const refreshing = ref(false)
const estopping = ref(false)
const currentTime = ref('')
let clockTimer = null

const activeMenu = computed(() => route.path)
const currentTitle = computed(() => route.meta?.title || '')
const balanceOnline = computed(() => !store.error && store.balanceOnline)
const actuatorOnline = computed(() => !store.error && store.actuatorOnline)

function updateClock() {
  const now = new Date()
  currentTime.value = now.toLocaleTimeString('zh-CN', { hour12: false })
}

async function handleRefresh() {
  refreshing.value = true
  await store.fetchStatus()
  refreshing.value = false
  if (store.error) ElMessage.error(store.error)
  else ElMessage.success('数据已刷新')
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
  // 布局层起来就开始轮询：每个页面都依赖同一份设备状态，不重复开定时器
  store.startPolling(500)
  updateClock()
  clockTimer = setInterval(updateClock, 1000)
})

onUnmounted(() => {
  store.stopPolling()
  if (clockTimer) {
    clearInterval(clockTimer)
    clockTimer = null
  }
})
</script>

<style scoped>
.admin-container {
  height: 100vh;
  overflow: hidden;
}

/* Sidebar */
.admin-sidebar {
  background-color: #304156;
  transition: width 0.3s;
  overflow-x: hidden;
}

.sidebar-header {
  display: flex;
  align-items: center;
  justify-content: center;
  height: 60px;
  color: #fff;
  border-bottom: 1px solid rgba(255, 255, 255, 0.1);
  gap: 8px;
  padding: 0 16px;
  white-space: nowrap;
}

.sidebar-logo-icon {
  color: #409eff;
  flex-shrink: 0;
}

.sidebar-title {
  font-size: 16px;
  font-weight: 600;
  overflow: hidden;
}

.el-menu {
  border-right: none;
}

/* Header */
.admin-header {
  display: flex;
  align-items: center;
  justify-content: space-between;
  background: #fff;
  border-bottom: 1px solid #e4e7ed;
  height: 50px;
  padding: 0 16px;
}

.header-left {
  display: flex;
  align-items: center;
  gap: 12px;
}

.collapse-btn {
  cursor: pointer;
  color: #606266;
}

.collapse-btn:hover {
  color: #409eff;
}

.header-right {
  display: flex;
  align-items: center;
  gap: 10px;
}

.header-time {
  font-size: 13px;
  color: #909399;
  font-family: 'Consolas', 'Courier New', monospace;
}

/* Main */
.admin-main {
  background: #f0f2f5;
  padding: 16px;
  overflow-y: auto;
  height: calc(100vh - 50px);
}
</style>
