import { createRouter, createWebHistory } from 'vue-router'

// 2026-10-26 融合：只保留目前后端真正支持的四页。
// 「网格实验」整页删除 —— 它依赖老后端的 /experiment/grid/*，新引擎里没有这套东西
// （网格扫描 = 固定频率/占空/窗口位置开环喷，新版没有"窗口位置"这个量）。
const routes = [
  {
    path: '/',
    component: () => import('../layouts/AdminLayout.vue'),
    redirect: '/monitor',
    children: [
      {
        path: 'monitor',
        name: 'DeviceMonitor',
        meta: { title: '设备监控', icon: 'Monitor' },
        component: () => import('../views/DeviceMonitor.vue'),
      },
      {
        path: 'dispense',
        name: 'DispenseControl',
        meta: { title: '加粉控制', icon: 'SetUp' },
        component: () => import('../views/DispenseControl.vue'),
      },
      {
        path: 'batch',
        name: 'BatchTest',
        meta: { title: '批量测试', icon: 'List' },
        component: () => import('../views/BatchTest.vue'),
      },
      {
        path: 'powders',
        name: 'PowderManagement',
        meta: { title: '粉末管理', icon: 'Collection' },
        component: () => import('../views/PowderManagement.vue'),
      },
      {
        // 试验记录还没接：数据源就是 runs/dispense_*.json 与 runs/batch_*.json，
        // 后端暂时没有列表接口，菜单里置灰。路由留着，以后接上直接可用。
        path: 'records',
        name: 'TestRecords',
        meta: { title: '试验记录', icon: 'Document' },
        component: () => import('../views/TestRecords.vue'),
      },
    ],
  },
  // 手输已删除的地址（比如旧的 /experiments）不要白屏，直接回设备监控
  { path: '/:pathMatch(.*)*', redirect: '/monitor' },
]

const router = createRouter({
  history: createWebHistory(),
  routes,
})

export default router
