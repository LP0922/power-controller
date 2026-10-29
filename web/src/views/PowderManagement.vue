<template>
  <div class="page">
    <!-- 天平实时读数：标定要盯秤看有没有出粉，这里直接给数字（数据来自设备 store 的 500ms 轮询） -->
    <BalanceReadout />

    <!-- ======================= 02 粉末标定 ======================= -->
    <PanelSection title="粉末标定（开环前馈）">
      <template #header-actions>
        <el-tag size="small" effect="plain" type="info">速率表 → 粉体库 feedforward_v2</el-tag>
        <el-tag size="small" :type="calib.running ? 'warning' : 'info'" effect="plain">
          {{ calibStateText }}
        </el-tag>
      </template>

      <p class="lead">
        选粉末 → 设定<b>搅拌转速</b>和<b>推杆开度</b> → 开环喷 T 秒，用<b>振动期间的天平斜率</b>
        算出这种粉在这个工况下能出多少粉。结果写进 <b>data/powder_library/&lt;粉名&gt;.json</b>，
        闭环加粉时按阶段目标速率去这张表里<b>挑工作点</b> —— 挑中哪个点，
        <b>搅拌转速和推杆开度都照抄它</b>，推杆整段锁死、速率只靠振幅调。
        <b>频率和占空比按粉记住</b>：选已标定的粉会自动填成它当初那组。
        <b>标定会真实出粉</b>：默认 1 点、每点 20 秒，含沉降约 45 秒。
      </p>

      <el-form label-width="96px" :disabled="locked" class="form">
        <el-form-item label="粉末">
          <div class="row">
            <el-select v-model="calibPowder" filterable class="grow" @change="onCalibPowderChange">
              <el-option v-for="p in powders" :key="p.powder_name" :value="p.powder_name"
                :label="powderLabel(p)" />
            </el-select>
            <el-button @click="reloadPowders">刷新列表</el-button>
            <span class="unit">{{ calibPowderInfo }}</span>
          </div>
        </el-form-item>

        <el-form-item label="新建粉末">
          <div class="row">
            <el-input v-model="newName" placeholder="输入新粉末名称" class="grow" />
            <el-select v-model="newCategory" clearable placeholder="分类（可选）" style="width: 200px">
              <el-option v-for="c in CATEGORIES" :key="c.value" :value="c.value" :label="c.label" />
            </el-select>
            <el-button @click="createPowder">添加档案</el-button>
            <span class="unit">只建空档案，标定数据另算</span>
          </div>
        </el-form-item>

        <el-form-item label="工作点">
          <div class="row">
            <span class="unit">搅拌</span>
            <el-input v-model="stirInput" placeholder="100" style="width: 150px" />
            <span class="unit">RPM（0=不搅，逗号分隔可扫多点）</span>
          </div>
        </el-form-item>
        <el-form-item label="推杆开度">
          <div class="row">
            <el-input v-model="rodInput" placeholder="600" style="width: 150px" />
            <span class="unit">units（逗号分隔可扫多点）</span>
            <el-tag :type="gridError ? 'danger' : 'info'" size="small" effect="plain">
              {{ gridError || `共 ${gridPoints} 点` }}
            </el-tag>
          </div>
        </el-form-item>

        <el-form-item label="振动参数">
          <div class="row">
            <span class="unit">频率</span>
            <el-input-number v-model="calibFreq" :min="10" :max="80" :step="1" class="num" />
            <span class="unit">Hz</span>
            <span class="unit">占空</span>
            <el-input-number v-model="calibDuty" :min="5" :max="50" :step="1" class="num" />
            <span class="unit">%</span>
            <span class="unit">振幅</span>
            <el-input-number v-model="calibAmp" :min="0.01" :max="20" :step="0.05" :precision="2" class="num" />
            <span class="unit">Vpp</span>
            <span class="unit">每点</span>
            <el-input-number v-model="calibDur" :min="2" :max="30" :step="1" class="num" />
            <span class="unit">秒</span>
          </div>
        </el-form-item>
      </el-form>

      <div class="actions">
        <el-button type="primary" :disabled="locked || !calibPowder" :loading="busy" @click="startCalib">
          开始标定
        </el-button>
        <el-button type="danger" plain :disabled="!calib.running" @click="stopCalib">停止标定</el-button>
        <span class="unit">{{ calib.detail || '空闲' }}</span>
      </div>

      <el-progress :percentage="calibPct" :stroke-width="10" class="progress" />
      <div class="metrics">
        <MetricCard label="当前点" :value="curPoint" />
        <MetricCard label="阶段" :value="calib.current?.phase || '—'" />
        <MetricCard label="本点用时" :value="calib.current?.elapsed_s != null ? `${calib.current.elapsed_s}s` : '—'" />
        <MetricCard label="最近速率" :value="lastRate" unit="mg/s" />
      </div>

      <el-table v-if="calibPoints.length" :data="calibPoints" border stripe size="small" class="table">
        <el-table-column label="搅拌 RPM" width="110">
          <template #default="{ row }">{{ fmt(row.stir_rpm, 0) }}</template>
        </el-table-column>
        <el-table-column label="推杆开度" width="100">
          <template #default="{ row }">{{ row.rod_units }}</template>
        </el-table-column>
        <el-table-column label="振动参数" min-width="180">
          <template #default="{ row }">{{ signalText(row.signal) }}</template>
        </el-table-column>
        <el-table-column label="速率 mg/s" width="120">
          <template #default="{ row }"><b>{{ fmt(row.rate_mg_s) }}</b></template>
        </el-table-column>
        <el-table-column label="CV %" width="92">
          <template #default="{ row }">{{ fmt(row.cv_pct, 1) }}</template>
        </el-table-column>
        <el-table-column label="本点出粉 mg" width="130">
          <template #default="{ row }">{{ fmt(row.delta_mg, 0) }}</template>
        </el-table-column>
        <el-table-column label="启动延迟 s" width="120">
          <template #default="{ row }">{{ fmt(row.startup_delay_s, 2) }}</template>
        </el-table-column>
        <el-table-column label="判定" min-width="150">
          <template #default="{ row }">
            <span :class="isBad(row) ? 'bad' : 'good'">{{ isBad(row) ? '不可信' : '可用' }}</span>
          </template>
        </el-table-column>
      </el-table>
      <p v-else class="empty">尚未标定。选好粉末、确认秤上清空后点「开始标定」。</p>
    </PanelSection>

    <!-- ======================= 03 标定管理 ======================= -->
    <PanelSection title="标定管理">
      <template #header-actions>
        <el-tag size="small" effect="plain" type="info">删无效标定点 · 立即重算前馈表</el-tag>
        <el-tag v-if="badCount" size="small" type="danger" effect="dark">{{ badCount }} 点不可信</el-tag>
      </template>

      <p class="lead">
        标定表里有的点根本不能用：料斗搭桥/没料测出来的<b>速率 0</b>、曲线抖动过大的 <b>CV &gt; 40%</b>、
        天平被碰导致的负速率 —— 这些点留在表里会把「挑工作点」带歪。
        <b>这里按点删</b>，删完马上重算三阶段搅拌转速与速率区间（档案里其它历史字段不动）。
        <b>runs/calib_*.json 里的历史归档不会跟着删</b>，那是原始记录。
      </p>

      <div class="row mgmt-head">
        <el-select v-model="mgmtPowder" filterable style="width: 320px" @change="loadDetail">
          <el-option v-for="p in powders" :key="p.powder_name" :value="p.powder_name"
            :label="powderLabel(p)" />
        </el-select>
        <el-button :disabled="locked" @click="reloadDetail">刷新</el-button>
        <span class="unit">{{ mgmtInfo }}</span>
      </div>

      <div class="actions">
        <el-button :disabled="locked || !detail" @click="selectBad">勾选全部不可信</el-button>
        <el-button :disabled="locked" @click="clearSelection">取消勾选</el-button>
        <el-button type="primary" :disabled="locked || !selection.length" @click="deleteSelected">
          删除勾选点
        </el-button>
        <el-button type="warning" plain :disabled="locked || !badCount" @click="deleteAllBad">
          一键删全部不可信
        </el-button>
        <el-button type="danger" plain :disabled="locked || !mgmtPowder" @click="wipe">
          清空该粉全部标定
        </el-button>
        <span class="unit">已选 {{ selection.length }} 点</span>
      </div>

      <el-table v-if="detailPoints.length" ref="mgmtTable" :data="detailPoints" border stripe size="small"
        class="table" row-key="index" @selection-change="onSelectionChange">
        <el-table-column type="selection" width="46" :selectable="() => !locked" />
        <el-table-column label="标定时间" min-width="160">
          <template #default="{ row }">{{ row.at || '—' }}</template>
        </el-table-column>
        <el-table-column label="搅拌 RPM" width="110">
          <template #default="{ row }">{{ fmt(row.stir_rpm, 0) }}</template>
        </el-table-column>
        <el-table-column label="推杆开度" width="100">
          <template #default="{ row }">{{ row.rod_units }}</template>
        </el-table-column>
        <el-table-column label="振动参数" min-width="180">
          <template #default="{ row }">{{ signalText(row.signal) }}</template>
        </el-table-column>
        <el-table-column label="速率 mg/s" width="120">
          <template #default="{ row }"><b>{{ fmt(row.rate_mg_s) }}</b></template>
        </el-table-column>
        <el-table-column label="CV %" width="92">
          <template #default="{ row }">{{ fmt(row.cv_pct, 1) }}</template>
        </el-table-column>
        <el-table-column label="出粉 mg" width="110">
          <template #default="{ row }">{{ fmt(row.delta_mg, 0) }}</template>
        </el-table-column>
        <el-table-column label="判定" min-width="190">
          <template #default="{ row }">
            <span v-if="!row.issues?.length" class="good">可用</span>
            <span v-else class="bad">不可信：{{ row.issues.join('、') }}</span>
          </template>
        </el-table-column>
      </el-table>
      <p v-else class="empty">该粉还没有标定数据（加粉会退回纯闭环）。</p>

      <!-- 加粉标记：粉末级的特殊行为开关，存档案 dispense_flags 段（目前只有封堵剂2 有） -->
      <div v-if="detail?.dispense_flags" class="recipe">
        <h4 class="recipe-title">加粉标记（已存入档案 dispense_flags）</h4>
        <ul class="flag-list">
          <li v-if="detail.dispense_flags.zero_rate_pulse === false">
            <span class="bad">不许在加粉速率为 0 时脉冲</span>：别的粉零速率时靠踢一脚
            （0.5s 高幅脉冲）震开料桥，这个粉不踢——它起振即成团爆发，脉冲会甩出几十上百 mg。
            顶上还不出料就让它顶着振幅带继续震，实在不出按断料中止。
          </li>
          <li v-else>
            <span class="good">允许零速率脉冲</span>：卡粉/搭桥时按 0.5s 高幅脉冲救济（踢一脚）。
          </li>
          <li v-if="detail.dispense_flags.slow_amp_span_vpp">
            慢加振幅带 <b>基准 ±{{ fmt(detail.dispense_flags.slow_amp_span_vpp, 2) }}V</b>
            （快加/粗加仍是基准 ±0.50V）——这是"不许零速率脉冲"的补偿，靠更大的连续振幅余量把料逼出来。
          </li>
        </ul>
        <p v-if="detail.dispense_flags.note" class="unit recipe-notes">
          {{ detail.dispense_flags.note }}
        </p>
      </div>

      <!-- 三阶段配方：操作者在加粉页定下的那张表，存进档案的 stage_recipe_v1 段 -->
      <div v-if="detail?.stage_recipe" class="recipe">
        <h4 class="recipe-title">三阶段配方（已存入档案 stage_recipe_v1）</h4>
        <p class="unit">{{ recipeHead }}</p>
        <el-table :data="detail.stage_recipe.stages || []" border stripe size="small" class="table">
          <el-table-column label="阶段" width="90">
            <template #default="{ row }">{{ row.stage_text || row.stage }}</template>
          </el-table-column>
          <el-table-column label="搅拌 RPM" width="105">
            <template #default="{ row }">{{ fmt(row.stir_rpm, 0) }}</template>
          </el-table-column>
          <el-table-column label="推杆开度" width="100">
            <template #default="{ row }">{{ row.rod_units ?? '—' }}</template>
          </el-table-column>
          <el-table-column label="振幅 Vpp" width="105">
            <template #default="{ row }"><b>{{ fmt(row.amplitude_vpp, 2) }}</b></template>
          </el-table-column>
          <el-table-column label="目标速率 mg/s" width="130">
            <template #default="{ row }">{{ fmt(row.rate_mg_s, 1) }}</template>
          </el-table-column>
          <el-table-column label="占空比 %" width="105">
            <template #default="{ row }">{{ fmt(row.duty_percent, 0) }}</template>
          </el-table-column>
          <el-table-column label="加到 mg">
            <template #default="{ row }">{{ row.switch_mass_mg ?? '—' }}</template>
          </el-table-column>
        </el-table>
        <p v-if="detail.stage_recipe.notes?.length" class="unit recipe-notes">
          {{ detail.stage_recipe.notes.join('；') }}
        </p>
      </div>

      <p class="foot">
        删除只动这张标定表（<b>{{ detail?.file || '—' }}</b> 的 feedforward_v2 段），
        删完立即重算三阶段搅拌转速与速率区间，下次加粉就按新表算。
      </p>
    </PanelSection>
  </div>
</template>

<script setup>
import { computed, onMounted, onUnmounted, ref } from 'vue'
import { ElMessage, ElMessageBox } from 'element-plus'
import { useDeviceStore } from '../stores/device.js'
import {
  listPowders, createPowder as createPowderProfile, getPowderDetail, deletePowderPoints,
  forgetPowderCalibration, getCalibOptions, startCalibration, stopCalibration,
  errorMessage,
} from '../api/index.js'
import PanelSection from '../components/PanelSection.vue'
import MetricCard from '../components/MetricCard.vue'
import BalanceReadout from '../components/BalanceReadout.vue'

const CATEGORIES = [
  { value: 'Low-flow', label: 'Low-flow（低流量）' },
  { value: 'Medium-flow', label: 'Medium-flow（中流量）' },
  { value: 'High-flow', label: 'High-flow（高流量）' },
]

const store = useDeviceStore()
const powders = ref([])
const busy = ref(false)

// ---------------- 02 标定表单
const calibPowder = ref('')
const newName = ref('')
const newCategory = ref('')
const stirInput = ref('100')
const rodInput = ref('600')
const calibFreq = ref(80)
const calibDuty = ref(50)
const calibAmp = ref(2)
const calibDur = ref(20)
const limits = ref({ stir_min: 0, stir_max: 1500, rod_min: 20, rod_max: 1200 })
let lastFilledPowder = ''

const calib = computed(() => store.calib)
const locked = computed(() => store.manualLocked || busy.value)
const calibStateText = computed(() => ({
  idle: '空闲', preparing: '准备中', running: '标定中', done: '已完成',
  aborted: '已中止', failed: '失败',
}[calib.value.state] || '空闲'))
const calibPoints = computed(() => calib.value.points || [])
const calibPct = computed(() => {
  const total = calib.value.total || 0
  return total ? Math.min(100, Math.round((100 * (calib.value.done || 0)) / total)) : 0
})
const curPoint = computed(() => {
  const cur = calib.value.current || {}
  if (cur.stir_rpm == null) return '—'
  return `${cur.stir_rpm} RPM / ${cur.rod_units}`
})
const lastRate = computed(() => {
  const pts = calibPoints.value
  return pts.length ? fmt(pts[pts.length - 1].rate_mg_s) : '—'
})
const calibPowderInfo = computed(() => {
  const p = powders.value.find((x) => x.powder_name === calibPowder.value)
  if (!p) return '—'
  if (!p.calibrated) return `${p.category || '未分类'} · 尚无标定数据（加粉会退回纯闭环）`
  const st = p.stage_stir_rpm || {}
  return `${p.category || '未分类'} · ${p.points} 点 · 粗加 ${st.coarse ?? '–'} / `
    + `精加 ${st.fine ?? '–'} / 精修 ${st.precision ?? '–'} RPM`
})

function parseList(text) {
  return String(text || '').split(/[,，\s]+/).map((s) => s.trim())
    .filter((s) => s !== '').map(Number)
}
const grid = computed(() => {
  const lim = limits.value || {}
  const stirs = parseList(stirInput.value)
  const rods = parseList(rodInput.value)
  if (!stirs.length) return { error: '请填搅拌转速（不想搅就填 0）' }
  if (!rods.length) return { error: '请填推杆开度' }
  for (const v of stirs) {
    if (!Number.isFinite(v) || v < (lim.stir_min ?? 0) || v > (lim.stir_max ?? 1500)) {
      return { error: `搅拌 ${v} 超出 ${lim.stir_min ?? 0}–${lim.stir_max ?? 1500} RPM` }
    }
  }
  for (const v of rods) {
    if (!Number.isFinite(v) || v < (lim.rod_min ?? 20) || v > (lim.rod_max ?? 1200)) {
      return { error: `推杆开度 ${v} 超出 ${lim.rod_min ?? 20}–${lim.rod_max ?? 1200}` }
    }
  }
  return { stirs, rods, points: stirs.length * rods.length }
})
const gridError = computed(() => grid.value.error || '')
const gridPoints = computed(() => grid.value.points || 0)

// ---------------- 03 管理
const mgmtPowder = ref('')
const detail = ref(null)
const selection = ref([])
const mgmtTable = ref(null)
const detailPoints = computed(() => detail.value?.points || [])
const badCount = computed(() => detailPoints.value.filter((p) => p.unreliable).length)
const mgmtInfo = computed(() => {
  if (!detail.value) return '—'
  const pts = detailPoints.value
  if (!pts.length) return '该粉还没有标定数据'
  const st = detail.value.stage_stir_rpm || {}
  return `${pts.length} 点 · 不可信 ${badCount.value} 点 · 三阶段搅拌 `
    + `${Math.round(st.coarse || 0)} / ${Math.round(st.fine || 0)} / ${Math.round(st.precision || 0)} RPM`
})
// 三阶段配方的摘要行：振动参数 / 目标 / 录入时间 / 来源
const recipeHead = computed(() => {
  const r = detail.value?.stage_recipe
  if (!r) return ''
  const sig = r.signal || {}
  const parts = []
  if (sig.frequency_hz != null) parts.push(`振动 ${sig.frequency_hz}Hz/${sig.duty_percent ?? 50}%`)
  if (r.target_mg != null) parts.push(`目标 ${r.target_mg} mg`)
  if (r.updated_at) parts.push(`录入 ${r.updated_at}`)
  return parts.join(' · ') + (r.source ? ` · ${r.source}` : '')
})

// ---------------- 通用
function fmt(v, digits = 2) {
  if (v === null || v === undefined || v === '') return '—'
  const n = Number(v)
  return Number.isFinite(n) ? n.toFixed(digits) : String(v)
}
const isBad = (row) => (row.cv_pct > 40 || !row.rate_mg_s)
const signalText = (sig) => {
  if (!sig || sig.frequency_hz === undefined || sig.frequency_hz === null) return '—'
  return `${fmt(sig.frequency_hz, 0)}Hz · ${fmt(sig.duty_percent, 0)}% · ${fmt(sig.amplitude_vpp, 2)}Vpp`
}
function powderLabel(p) {
  const bad = p.unreliable_points || 0
  return `${p.powder_name} · ${p.calibrated ? `${p.points} 点${bad ? `（不可信 ${bad}）` : ''}` : '未标定'}`
}
function fail(error) {
  if (error === 'cancel' || error === 'close') return
  ElMessage.error(errorMessage(error))
}

async function reloadPowders() {
  try {
    const data = await listPowders()
    const errs = data?.errors || []
    powders.value = data?.powders || []
    if (errs.length) {
      ElMessage.warning(`粉体库 ${errs.length} 个档案读不出来：`
        + errs.slice(0, 3).map((e) => e.file).join('、'))
    }
    const keepCalib = powders.value.some((p) => p.powder_name === calibPowder.value)
    if (!keepCalib) calibPowder.value = (powders.value[0] || {}).powder_name || ''
    const keepMgmt = powders.value.some((p) => p.powder_name === mgmtPowder.value)
    if (!keepMgmt) {
      const first = powders.value.find((p) => p.calibrated) || powders.value[0]
      mgmtPowder.value = first ? first.powder_name : ''
      await loadDetail()
    }
    await onCalibPowderChange(true)
  } catch (error) { fail(error) }
}

// 换粉时把振动参数填成「这粉当初标定用的那一组」，重标沿用才不会把新旧点混成两套工况
async function onCalibPowderChange(force = false) {
  const p = powders.value.find((x) => x.powder_name === calibPowder.value)
  if (!p) return
  if (force !== true && p.powder_name === lastFilledPowder) return
  lastFilledPowder = p.powder_name
  if (p.calibrated && p.signal) {
    if (p.signal.frequency_hz != null) calibFreq.value = p.signal.frequency_hz
    if (p.signal.duty_percent != null) calibDuty.value = p.signal.duty_percent
    if (p.signal.amplitude_vpp != null) calibAmp.value = p.signal.amplitude_vpp
  }
}

async function createPowder() {
  const name = newName.value.trim()
  if (!name) { ElMessage.error('请先输入粉末名称'); return }
  busy.value = true
  try {
    const r = await createPowderProfile(name, newCategory.value || null)
    await reloadPowders()
    calibPowder.value = r.name
    await onCalibPowderChange(true)
    newName.value = ''
    ElMessage.success(`已新建粉末「${r.name}」（档案 ${r.file}），可以直接标定它`)
  } catch (error) { fail(error) } finally { busy.value = false }
}

async function startCalib() {
  const p = powders.value.find((x) => x.powder_name === calibPowder.value)
  if (!p) { ElMessage.error('请先选择粉末'); return }
  if (gridError.value) { ElMessage.error(`工作点填得不对：${gridError.value}`); return }
  const g = grid.value
  const dur = calibDur.value || 20
  const overwrite = p.calibrated
    ? `\n注意：该粉已有 ${p.points} 个标定点，同工况的新数据会覆盖旧值。` : ''
  try {
    await ElMessageBox.confirm(
      `开始标定「${p.powder_name}」\n`
      + `搅拌 ${g.stirs.join(' / ')} RPM × 推杆开度 ${g.rods.join(' / ')} = ${g.points} 点，每点 ${dur} 秒\n`
      + `预计 ${((g.points * (dur + 8)) / 60).toFixed(1)} 分钟，期间会真实出粉。\n`
      + `请确认：秤已清空、料斗有粉、机构周围无人。${overwrite}`,
      '开始标定', { type: 'warning', customClass: 'pre-wrap' })
  } catch { return }
  busy.value = true
  try {
    const r = await startCalibration({
      name: p.powder_name,
      category: p.category || null,
      stir_levels: g.stirs,
      rod_points: g.rods,
      frequency_hz: calibFreq.value,
      duty_percent: calibDuty.value,
      amplitude_vpp: calibAmp.value,
      duration_s: dur,
    })
    ElMessage.success(`标定已启动：${r.total || g.points} 个点（进度见下方）`)
    await store.fetchStatus()
  } catch (error) { fail(error) } finally { busy.value = false }
}

async function stopCalib() {
  try {
    const r = await stopCalibration()
    ElMessage.info(r?.stopped ? '已请求停止标定（正在停振并落盘已测点）' : '当前没有运行中的标定任务')
  } catch (error) { fail(error) }
}

async function loadDetail() {
  if (!mgmtPowder.value) { detail.value = null; return }
  try {
    detail.value = await getPowderDetail(mgmtPowder.value)
    selection.value = []
  } catch (error) { fail(error) }
}
async function reloadDetail() {
  await reloadPowders()
  await loadDetail()
}
function onSelectionChange(rows) { selection.value = rows || [] }
function selectBad() {
  const table = mgmtTable.value
  if (!table) return
  table.clearSelection()
  for (const row of detailPoints.value) {
    if (row.unreliable) table.toggleRowSelection(row, true)
  }
}
function clearSelection() { mgmtTable.value?.clearSelection() }

async function removePoints(payload, label) {
  if (!mgmtPowder.value) { ElMessage.error('请先选择粉末'); return }
  busy.value = true
  try {
    const r = await deletePowderPoints(mgmtPowder.value, payload)
    await reloadPowders()
    await loadDetail()
    ElMessage.success(r.removed
      ? `${label}：已删除 ${r.removed} 点，剩余 ${r.remaining} 点`
        + (r.cleared ? '（已删空，该粉变回未标定）' : '')
      : `${label}：没有可删的点`)
  } catch (error) { fail(error) } finally { busy.value = false }
}

async function deleteSelected() {
  const indices = selection.value.map((r) => r.index).sort((a, b) => a - b)
  if (!indices.length) { ElMessage.error('先在表里勾选要删的标定点'); return }
  try {
    await ElMessageBox.confirm(
      `删除「${mgmtPowder.value}」的 ${indices.length} 个标定点？\n`
      + '删除后立即按剩下的点重算三阶段搅拌转速与速率区间，不可撤销。\n'
      + '（runs/ 下的历史归档文件不会删）',
      '删除确认', { type: 'warning' })
  } catch { return }
  removePoints({ indices }, '删除勾选点')
}

async function deleteAllBad() {
  if (!badCount.value) { ElMessage.error('这张表里没有判为不可信的点'); return }
  try {
    await ElMessageBox.confirm(
      `一键删除「${mgmtPowder.value}」中全部 ${badCount.value} 个不可信点？\n`
      + '判定口径：速率为 0 / 出粉量为 0 / CV > 40% / 字段缺失。\n'
      + '删除后立即重算前馈表，不可撤销（runs/ 归档文件保留）。',
      '清理确认', { type: 'warning' })
  } catch { return }
  removePoints({ mode: 'invalid' }, '清理不可信点')
}

async function wipe() {
  try {
    await ElMessageBox.confirm(
      `⚠ 清空「${mgmtPowder.value}」的全部标定数据？\n`
      + '这会摘掉档案里的 feedforward_v2 整段（三阶段搅拌转速、前馈表、修正系数全部失效），\n'
      + '该粉之后加粉会退回纯闭环。档案文件本身和它的历史字段不会被删。',
      '清空确认', { type: 'warning' })
  } catch { return }
  busy.value = true
  try {
    const r = await forgetPowderCalibration(mgmtPowder.value)
    await reloadPowders()
    await loadDetail()
    ElMessage.success(r.removed ? `已清空「${r.name}」的标定数据` : `没清掉：${r.reason || '档案不存在'}`)
  } catch (error) { fail(error) } finally { busy.value = false }
}

onMounted(async () => {
  store.startPolling(500)
  try {
    const opts = await getCalibOptions()
    limits.value = opts.limits || limits.value
    stirInput.value = String((opts.stir_levels || [100])[0])
    rodInput.value = String((opts.rod_points || [600])[0])
    calibFreq.value = opts.frequency_hz ?? 80
    calibDuty.value = opts.duty_percent ?? 50
    calibAmp.value = opts.amplitude_vpp ?? 2
    calibDur.value = opts.duration_s ?? 20
  } catch (error) { fail(error) }
  // 先拿默认参数、再读粉体库：顺序反了会被「这粉当初那组」参数盖回去
  await reloadPowders()
})
onUnmounted(() => store.stopPolling())
</script>

<style scoped>
.page { display: flex; flex-direction: column; gap: 16px; min-width: 0; }
.lead { margin: 0 0 16px; font-size: 12px; line-height: 1.8; color: #6b7c91; }
.lead b { color: #3d5573; }
.form :deep(.el-form-item) { margin-bottom: 12px; }
.row { display: flex; align-items: center; gap: 8px; flex-wrap: wrap; width: 100%; }
.row :deep(.el-button + .el-button) { margin-left: 0; }
.grow { flex: 1; min-width: 160px; }
.num { width: 104px; }
.unit { font-size: 12px; color: #8b9aab; }
.actions { display: flex; align-items: center; gap: 8px; flex-wrap: wrap; margin: 14px 0; }
.actions :deep(.el-button + .el-button) { margin-left: 0; }
.progress { margin: 6px 0 16px; }
.metrics { display: grid; grid-template-columns: repeat(4, minmax(0, 1fr)); gap: 10px; margin-bottom: 16px; }
.metrics :deep(.value-text) { font-size: 20px; }
.table { margin-top: 4px; }
.good { color: #059669; }
.bad { color: #dc2626; }
.empty { margin: 8px 0 0; font-size: 12px; color: #8b9aab; }
.mgmt-head { margin-bottom: 4px; }
.recipe { margin-top: 18px; padding-top: 12px; border-top: 1px dashed #dbe3ec; }
.recipe-title { margin: 0 0 4px; font-size: 13px; font-weight: 600; color: #3d5573; }
.recipe-notes { margin: 6px 0 0; line-height: 1.7; }
.flag-list { margin: 4px 0 0; padding-left: 18px; font-size: 12px; line-height: 1.85; color: #4a5b70; }
.flag-list li { margin-bottom: 2px; }
.foot { margin: 10px 0 0; font-size: 11.5px; color: #8b9aab; line-height: 1.7; }
@media (max-width: 1100px) { .metrics { grid-template-columns: repeat(2, minmax(0, 1fr)); } }
</style>
