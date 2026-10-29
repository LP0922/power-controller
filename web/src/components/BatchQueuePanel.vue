<template>
  <PanelSection title="批量加粉队列">
    <template #header-actions>
      <el-tag v-if="running" type="warning" size="small" effect="plain">
        批量运行中 {{ doneCount }}/{{ batch.set_count }}
      </el-tag>
      <el-tag v-else-if="batch.state === 'done'" type="success" size="small" effect="plain">
        上批完成 {{ batch.summary?.passed }}/{{ batch.summary?.done }}
      </el-tag>
      <el-tag v-else-if="batch.state === 'aborted'" type="danger" size="small" effect="plain">
        上批已停止
      </el-tag>
      <el-tag v-else type="info" size="small" effect="plain">
        {{ queueCount > 0 ? queueCount + ' 项待执行' : '空闲' }}
      </el-tag>
    </template>

    <p class="section-note">
      队列里的每一项都是一次正常加粉：参数与「加粉控制」页单点启动完全一致（三阶段配方、振幅基准、
      沉降门、超速熔断、缓补都一样），切点由后端按目标算。整批必须同一种粉末，项与项之间默认自动去皮。
      批量跑的时候手动启动会被后端 409 拦住 —— 别插队。
    </p>

    <!-- 运行进度 -->
    <div v-if="running" class="running-bar">
      <div class="running-head">
        <span>{{ batch.detail || '准备中' }}</span>
        <span>{{ doneCount }} / {{ batch.set_count }}</span>
      </div>
      <el-progress :percentage="runPercent" :stroke-width="8" :show-text="false" />
    </div>

    <!-- 队列（后端已有 items 但前端队列空了 = 页面刷新过，展示后端那份） -->
    <div v-if="visibleQueue.length > 0" class="queue-section">
      <el-table :data="visibleQueue" row-key="id" size="small" border stripe
        @selection-change="handleSelectionChange">
        <el-table-column v-if="!usingRemote" type="selection" width="42" align="center"
          :selectable="() => !locked" />
        <el-table-column label="#" width="45" align="center">
          <template #default="{ $index }">{{ $index + 1 }}</template>
        </el-table-column>
        <el-table-column label="粉末" min-width="110">
          <template #default="{ row }">{{ row.params.powder || '（不选粉）' }}</template>
        </el-table-column>
        <el-table-column label="分类" width="120" show-overflow-tooltip>
          <template #default="{ row }">{{ row.params.category || '默认' }}</template>
        </el-table-column>
        <el-table-column label="目标 mg" width="92" align="center">
          <template #default="{ row }">{{ row.params.target_mg }}</template>
        </el-table-column>
        <el-table-column label="重复" width="66" align="center">
          <template #default="{ row }">{{ row.params.repeat_count || 1 }}</template>
        </el-table-column>
        <el-table-column label="振幅" width="76" align="center">
          <template #default="{ row }">
            {{ row.params.amplitude_vpp != null ? row.params.amplitude_vpp + 'V' : '自动' }}
          </template>
        </el-table-column>
        <el-table-column label="沉降门" width="76" align="center">
          <template #default="{ row }">{{ fmtSwitch(row.params.settle_gate) }}</template>
        </el-table-column>
        <el-table-column label="熔断" width="86" align="center">
          <template #default="{ row }">
            {{ fmtSurge(row.params.surge_guard, row.params.surge_ratio) }}
          </template>
        </el-table-column>
        <el-table-column label="缓补" width="76" align="center">
          <template #default="{ row }">{{ fmtSwitch(row.params.trim_enabled) }}</template>
        </el-table-column>
        <el-table-column label="逐段覆盖" min-width="150" show-overflow-tooltip>
          <template #default="{ row }">{{ stageSummary(row.params.stages) }}</template>
        </el-table-column>
        <el-table-column v-if="!usingRemote" label="操作" width="66" align="center">
          <template #default="{ row }">
            <el-button type="danger" size="small" :icon="Delete" circle
              :disabled="locked" @click="removeFromQueue(row.id)" />
          </template>
        </el-table-column>
      </el-table>
      <p v-if="usingRemote" class="remote-note">
        这一批是后端正在执行的队列（页面刷新过，前端的待执行列表已清空）。停止后才能重新编辑。
      </p>
    </div>
    <div v-else class="empty-queue">
      <span class="muted">
        队列为空 —— 到「加粉控制」页选好粉末与目标质量，点「加入批量队列」。
      </span>
    </div>

    <div class="batch-actions">
      <el-button type="primary" :disabled="queueCount === 0 || locked"
        :loading="starting" @click="runItems(batchQueue)">
        运行全部 ({{ queueCount }})
      </el-button>
      <el-button type="success" :disabled="selectedItems.length === 0 || locked"
        :loading="starting" @click="runItems(selectedItems)">
        运行选中 ({{ selectedItems.length }})
      </el-button>
      <el-button type="danger" :disabled="!running" @click="stopBatch">
        停止批量
      </el-button>
      <el-button :disabled="queueCount === 0 || locked || running" @click="clearQueue">
        清空队列
      </el-button>
      <el-checkbox v-model="autoTareBetweenGroups" :disabled="locked" class="tare-checkbox">
        组间自动去皮
      </el-checkbox>
    </div>

    <!-- 结果 -->
    <div v-if="results.length > 0" class="results-section">
      <h4 class="section-title">
        批量结果
        <span class="sub">{{ batch.run_id }}</span>
      </h4>
      <el-table :data="results" size="small" border stripe>
        <el-table-column label="#" width="52" align="center">
          <template #default="{ $index }">{{ $index + 1 }}</template>
        </el-table-column>
        <el-table-column label="粉末" min-width="100">
          <template #default="{ row }">{{ row.powder_name || row.powder || '—' }}</template>
        </el-table-column>
        <el-table-column label="目标" width="82" align="center">
          <template #default="{ row }">{{ row.target_mg }} mg</template>
        </el-table-column>
        <el-table-column label="实得" width="92" align="center">
          <template #default="{ row }">{{ fmt(row.achieved_mg) }} mg</template>
        </el-table-column>
        <el-table-column label="偏差" width="82" align="center">
          <template #default="{ row }">
            <span :class="deltaClass(row.delta_mg)">{{ fmtSigned(row.delta_mg) }}</span>
          </template>
        </el-table-column>
        <el-table-column label="判定" width="82" align="center">
          <template #default="{ row }">
            <el-tag v-if="row.passed" type="success" size="small">合格</el-tag>
            <el-tag v-else type="danger" size="small">{{ row.verdict || '不合格' }}</el-tag>
          </template>
        </el-table-column>
        <el-table-column label="用时" width="76" align="center">
          <template #default="{ row }">{{ fmt(row.duration_s) }} s</template>
        </el-table-column>
        <el-table-column label="振幅" width="76" align="center">
          <template #default="{ row }">{{ fmt(row.amplitude_vpp, 2) }} V</template>
        </el-table-column>
        <el-table-column label="结束原因" min-width="140" show-overflow-tooltip>
          <template #default="{ row }">{{ row.stop_reason || '—' }}</template>
        </el-table-column>
      </el-table>

      <div class="summary-line">
        <span>通过率: {{ batch.summary?.pass_rate ?? passRate }}%</span>
        <el-divider direction="vertical" />
        <span>合格 {{ passCount }} / 不合格 {{ failCount }}</span>
        <el-divider direction="vertical" />
        <span>共 {{ results.length }} 项</span>
        <span v-if="batch.started_at" class="muted">· {{ batch.started_at }}
          <template v-if="batch.finished_at"> → {{ batch.finished_at }}</template>
        </span>
      </div>
      <p class="log-hint">
        本次结果记录在 <b>runs/batch_{{ batch.run_id || '*' }}.json</b>，
        每一项的逐拍曲线可以从对应 <b>runs/dispense_*.json</b> 里查（结果行里带 log_path）。
      </p>
    </div>
    <div v-else-if="running" class="results-section">
      <span class="muted">等待第一项结果…</span>
    </div>

    <el-alert v-if="error" :title="error" type="error" :closable="false" class="message" />
  </PanelSection>
</template>

<script setup>
import { computed, onMounted, onUnmounted, ref } from 'vue'
import { Delete } from '@element-plus/icons-vue'
import { ElMessage, ElMessageBox } from 'element-plus'
import { useDeviceStore } from '../stores/device.js'
import {
  startDispenseBatch, cancelDispenseBatch, getDispenseBatchStatus, errorMessage,
} from '../api/index.js'
import { stageSummary, validateBatchQueue } from '../utils/dispense.js'
import { useBatchQueue } from '../composables/useBatchQueue.js'
import PanelSection from './PanelSection.vue'

const store = useDeviceStore()
const { batchQueue, autoTareBetweenGroups, removeFromQueue, clearQueue } = useBatchQueue()

const batch = ref({ state: 'idle', running: false, results: [], items: [], summary: {} })
const error = ref('')
const starting = ref(false)
const selectedIds = ref([])

// 批量状态全部来自后端（页面刷新也不丢），前端不自己累加。
// 单独轮询 /dispense/batch/status，而不是塞进 /status —— 批量状态只在跑的时候才需要细看。
let timer = null
async function pollBatch() {
  try {
    batch.value = await getDispenseBatchStatus()
  } catch {
    // 后端短暂无响应时保留上一帧，不把"运行中"误判成结束
  }
}
onMounted(() => {
  store.startPolling(500)
  pollBatch()
  timer = setInterval(pollBatch, 800)
})
onUnmounted(() => {
  store.stopPolling()
  if (timer) { clearInterval(timer); timer = null }
})

const running = computed(() => batch.value.running === true)
const results = computed(() => batch.value.results || [])
const locked = computed(() => running.value || store.dispenseRunning || starting.value)
const queueCount = computed(() => batchQueue.value.length)
const selectedItems = computed(() => (
  batchQueue.value.filter((i) => selectedIds.value.includes(i.id))
))

// 前端队列为空、后端有 items → 说明页面刷新过，把后端那份摊出来看
const usingRemote = computed(() => queueCount.value === 0 && (batch.value.items || []).length > 0)
const visibleQueue = computed(() => (usingRemote.value
  ? (batch.value.items || []).map((it, i) => ({
    id: `remote-${i}`,
    params: {
      powder: it.powder, category: it.category, target_mg: it.target_mg,
      repeat_count: it.repeat_count, amplitude_vpp: it.amplitude_vpp,
      settle_gate: it.settle_gate, surge_guard: it.surge_guard,
      surge_ratio: it.surge_ratio, trim_enabled: it.trim_enabled, stages: it.stages,
    },
  }))
  : batchQueue.value))

const doneCount = computed(() => {
  if (running.value && batch.value.current_index >= 0) {
    // 正在跑第 N 项：已完成 N-1 项 + 当前项按 live 进度算
    return Math.min(results.value.length + 1, batch.value.set_count || 0)
  }
  return results.value.length
})
const runPercent = computed(() => {
  const total = batch.value.set_count || 0
  return total ? Math.min(100, Math.round((100 * results.value.length) / total)) : 0
})
const passCount = computed(() => results.value.filter((r) => r.passed).length)
const failCount = computed(() => results.value.filter((r) => !r.passed).length)
const passRate = computed(() => (results.value.length
  ? ((passCount.value / results.value.length) * 100).toFixed(1) : 0))

function handleSelectionChange(rows) { selectedIds.value = rows.map((r) => r.id) }

async function runItems(items) {
  if (!items.length) return
  starting.value = true
  error.value = ''
  try {
    validateBatchQueue(items)
    const payload = {
      powder: items[0].params.powder || null,
      tare_between_sets: autoTareBetweenGroups.value,
      sets: items.map((item) => {
        const p = item.params
        const set = { target_mg: p.target_mg, repeat_count: p.repeat_count || 1 }
        // 只带真有值的键：后端按"没传就用默认"处理，传 null 会被当成显式覆盖
        for (const key of ['category', 'amplitude_vpp', 'tail_mass_mg',
          'settle_gate', 'settle_gate_rate_mg_s', 'taper_factor',
          'surge_guard', 'surge_ratio', 'trim_enabled', 'trim_enter_remain_mg', 'stages']) {
          if (p[key] != null) set[key] = p[key]
        }
        return set
      }),
    }
    const total = payload.sets.reduce((sum, s) => sum + (s.repeat_count || 1), 0)
    await ElMessageBox.confirm(
      `确认用「${payload.powder || '不选粉'}」执行 ${payload.sets.length} 行`
      + `（含重复共 ${total} 次加粉）？\n`
      + `组间去皮：${autoTareBetweenGroups.value ? '开' : '关'} · `
      + '每项参数与「加粉控制」页单点启动相同。',
      '批量启动确认', { type: 'warning' })
    const response = await startDispenseBatch(payload)
    ElMessage.success(`已启动批量加粉（${response?.set_count ?? total} 项）`)
    await pollBatch()
  } catch (e) {
    if (e !== 'cancel' && e !== 'close') {
      error.value = errorMessage(e)
      ElMessage.error(error.value)
    }
  } finally {
    starting.value = false
  }
}

async function stopBatch() {
  try {
    await cancelDispenseBatch()
    ElMessage.info('已请求停止批量（等当前这一项收尾）')
    await pollBatch()
  } catch (e) {
    ElMessage.error(errorMessage(e))
  }
}

function fmtSwitch(value) {
  if (value == null) return '默认'
  return value ? '开' : '关'
}
function fmtSurge(guard, ratio) {
  if (guard === false) return '关'
  if (ratio === 0) return '关'
  if (ratio == null) return guard == null ? '默认' : '开 2.5×'
  return `开 ${ratio}×`
}
function deltaClass(delta) {
  if (delta == null) return ''
  if (delta > 10 || delta < -10) return 'delta-bad'
  return 'delta-ok'
}
function fmt(value, digits) {
  if (value === undefined || value === null || value === '') return '--'
  if (typeof value === 'number') {
    return Number.isInteger(value) ? String(value) : value.toFixed(digits ?? 2)
  }
  return String(value)
}
function fmtSigned(value) {
  const text = fmt(value)
  if (text === '--') return text
  return Number(value) > 0 ? `+${text}` : text
}
</script>

<style scoped>
.section-note { font-size: 12px; line-height: 1.7; color: #738196; margin: 0 0 14px; }
.running-bar { margin-bottom: 14px; padding: 10px 14px; background: #fdf6ec; border: 1px solid #faecd8; border-radius: 8px; }
.running-head { display: flex; justify-content: space-between; gap: 8px; font-size: 12px; color: #8a6d3b; margin-bottom: 8px; }
.queue-section { margin-bottom: 14px; }
.remote-note { margin: 8px 0 0; font-size: 11.5px; color: #b08d57; }
.empty-queue { padding: 20px 0; text-align: center; border: 1px dashed #dcdfe6; border-radius: 4px; margin-bottom: 14px; }
.muted { color: #c0c4cc; font-size: 13px; }
.batch-actions { display: flex; align-items: center; gap: 8px; margin-bottom: 16px; flex-wrap: wrap; }
.batch-actions :deep(.el-button + .el-button) { margin-left: 0; }
.tare-checkbox { margin-left: 4px; }
.results-section { margin-top: 8px; }
.section-title { font-size: 14px; font-weight: 600; color: #303133; margin: 0 0 8px 0; padding-bottom: 6px; border-bottom: 1px solid #ebeef5; }
.section-title .sub { font-weight: 400; font-size: 11px; color: #b3bcc9; margin-left: 8px; }
.summary-line { margin-top: 14px; padding: 10px 14px; background: #f5f7fa; border-radius: 6px; font-size: 13px; color: #606266; display: flex; align-items: center; flex-wrap: wrap; gap: 4px; }
.log-hint { margin: 8px 0 0; font-size: 11.5px; color: #9aa7b6; line-height: 1.7; }
.delta-ok { color: #67c23a; }
.delta-bad { color: #f56c6c; font-weight: 600; }
.message { margin-top: 14px; }
</style>
