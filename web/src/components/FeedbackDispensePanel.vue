<template>
  <PanelSection title="闭环加粉">
    <template #header-actions>
      <el-tag :type="running ? 'warning' : 'info'" effect="plain">{{ statusLabel }}</el-tag>
      <el-tag v-if="store.dispense.state && !running" size="small" effect="plain" type="info">
        {{ stateText }}
      </el-tag>
    </template>

    <p class="lead">
      天平质量 → 运动观测器 → <b>振幅闭环</b> + 控制器（只做预测停车判定）。
      三阶段逐段配置：快加（冲量）→ 粗加（跟得上又不毛躁）→ 慢加（低幅收尾）。
      <b>推杆只在阶段入口开到该段开度，整段锁死不动，收尾也不复位</b>；
      <b>速率只由振幅调</b>（基准 ±0.5V 内小步，单步 0.05–0.15V）。
      目标 100–20000 mg，验收 ±10 mg。
    </p>

    <!-- ================= 第一块：粉末 / 目标 / 保护开关 ================= -->
    <div class="setup-grid">
      <div class="setup-col">
        <h4 class="col-title">粉末与目标</h4>
        <el-form label-width="92px" :disabled="locked" class="form">
          <el-form-item label="粉末">
            <div class="row">
              <el-select v-model="powderName" filterable clearable class="grow"
                placeholder="不用标定 · 纯闭环" @change="onPowderChange">
                <el-option v-for="p in powders" :key="p.powder_name" :value="p.powder_name"
                  :label="`${p.powder_name} · ${p.points} 点`" />
              </el-select>
              <el-button @click="loadPowders">刷新</el-button>
            </div>
          </el-form-item>
          <el-form-item label="粉体分类">
            <el-select v-model="category" class="grow" @change="schedulePreview(true)">
              <el-option v-for="c in CATEGORIES" :key="c.value" :value="c.value" :label="c.label" />
            </el-select>
          </el-form-item>
          <el-form-item label="目标质量">
            <div class="row">
              <el-button v-for="t in [200, 500, 1000, 2000]" :key="t"
                :type="target === t ? 'primary' : 'default'" :plain="target === t"
                size="small" @click="target = t">{{ t }}</el-button>
              <el-input-number v-model="target" :min="100" :max="20000" :step="10" class="grow" />
              <span class="unit">mg</span>
            </div>
          </el-form-item>
          <el-form-item label="起始振幅">
            <div class="row">
              <el-input-number v-model="ampOverride" :min="0.01" :max="4" :step="0.05" :precision="2"
                placeholder="自动" class="grow" />
              <span class="unit">Vpp（留空=按该粉标定）</span>
            </div>
          </el-form-item>
          <el-form-item label="尾部余量">
            <div class="row">
              <el-input-number v-model="tailMass" :min="0" :max="50" :step="0.5" :precision="1"
                placeholder="自动" class="grow" />
              <span class="unit">mg（留空=算法默认）</span>
            </div>
          </el-form-item>
        </el-form>
        <p class="powder-info">{{ powderInfo }}</p>
      </div>

      <div class="setup-col">
        <h4 class="col-title">保护开关</h4>
        <el-form label-width="112px" :disabled="locked" class="form">
          <el-form-item label="切段沉降门">
            <div class="row">
              <el-switch v-model="settleGate" active-text="开" inactive-text="关" />
              <span class="unit">切段前停振等余料落完（重粉必开）</span>
            </div>
          </el-form-item>
          <el-form-item label="触发速率">
            <div class="row">
              <el-input-number v-model="gateRate" :min="0.5" :max="300" :step="0.5"
                placeholder="1.5×" class="grow" :disabled="!settleGate" />
              <span class="unit">mg/s（留空=本段规定速率×1.5）</span>
            </div>
          </el-form-item>
          <el-form-item label="提前降幅">
            <div class="row">
              <el-input-number v-model="taperFactor" :min="0" :max="3" :step="0.05" :precision="2"
                class="grow" :disabled="!settleGate" />
              <span class="unit">×当前速率（0=不提前降）</span>
            </div>
          </el-form-item>
          <el-form-item label="超速熔断">
            <div class="row">
              <el-switch v-model="surgeGuard" active-text="开" inactive-text="关" />
              <span class="unit">段内落料过猛立刻停振（防塌料冲过头）</span>
            </div>
          </el-form-item>
          <el-form-item label="熔断倍率">
            <div class="row">
              <el-input-number v-model="surgeRatio" :min="1" :max="10" :step="0.1" :precision="1"
                class="grow" :disabled="!surgeGuard" />
              <span class="unit">×本段规定速率（粗加 20→50mg/s）</span>
            </div>
          </el-form-item>
          <el-form-item label="缓补脉冲">
            <div class="row">
              <el-switch v-model="trimEnabled" active-text="开" inactive-text="关" />
              <span class="unit">开：慢加剩 ≤触发量就切真脉冲补料；关：不切脉冲，但收尾偏轻仍按慢加行振幅补差</span>
            </div>
          </el-form-item>
          <el-form-item label="缓补触发量">
            <div class="row">
              <el-input-number v-model="trimEnter" :min="5" :max="200" :step="1"
                placeholder="40" class="grow" :disabled="!trimEnabled" />
              <span class="unit">mg（留空=40）· 每发固定 0.40s，只调振幅</span>
            </div>
          </el-form-item>
        </el-form>
      </div>
    </div>

    <!-- ================= 第二块：三阶段配方表 ================= -->
    <div class="stage-block">
      <div class="stage-head">
        <h4 class="col-title">三阶段配方</h4>
        <el-button size="small" :disabled="locked" @click="refreshPreview(true)">按标定表重算</el-button>
        <span class="plan-info" :class="{ bad: planError }">{{ planText }}</span>
      </div>

      <table class="stage-table">
        <thead>
          <tr>
            <th class="c-stage">阶段</th>
            <th class="c-en">启用</th>
            <th>搅拌 RPM</th>
            <th>推杆开度</th>
            <th>振幅 Vpp</th>
            <th>目标速率 mg/s</th>
            <th>占空比 %</th>
            <th>加到 mg</th>
            <th>沉降等 s</th>
          </tr>
        </thead>
        <tbody>
          <tr v-for="row in stageRows" :key="row.name">
            <td class="c-stage">{{ STAGE_TEXT[row.name] }}</td>
            <td class="c-en">
              <el-checkbox v-model="row.enabled" :disabled="row.name === 'slow' || locked" />
            </td>
            <td><el-input-number v-model="row.stir_rpm" :min="0" :max="1500" :step="10"
              :controls="false" size="small" :disabled="locked || !row.enabled" /></td>
            <td><el-input-number v-model="row.rod_units" :min="20" :max="1200" :step="5"
              :controls="false" size="small" :disabled="locked || !row.enabled" /></td>
            <td><el-input-number v-model="row.amplitude_vpp" :min="0.01" :max="4" :step="0.05"
              :precision="2" :controls="false" size="small" :disabled="locked || !row.enabled" /></td>
            <td><el-input-number v-model="row.rate_mg_s" :min="0.1" :max="300" :step="0.1"
              :precision="1" :controls="false" size="small" :disabled="locked || !row.enabled" /></td>
            <td><el-input-number v-model="row.duty_percent" :min="5" :max="50" :step="1"
              :controls="false" size="small" :disabled="locked || !row.enabled" /></td>
            <td><el-input-number v-model="row.switch_mass_mg" :min="1" :step="1"
              :controls="false" size="small" :disabled="locked || !row.enabled || row.name === 'slow'" /></td>
            <td><el-input-number v-model="row.settle_wait_s" :min="0" :max="5" :step="0.1"
              :precision="1" :controls="false" size="small"
              :disabled="locked || !row.enabled || row.name === 'slow'" /></td>
          </tr>
        </tbody>
      </table>

      <p class="stage-hint">
        切点规则：目标 &gt; 5000 时快加冲到「剩余 800mg」切粗加、粗加到剩余 100mg；1000–5000 时快加到 75%、
        粗加到 95%；≤ 1000 只用粗加+慢加；&lt; 300 时粗加到「目标−40mg」。
        目标速率留空 = 快加 75 / 粗加 20 / 慢加 5 mg/s —— 这三个数只是「从标定表挑工作点」的靶子，
        挑中哪个点就用它的搅拌转速和推杆开度。<b>振幅三行默认 3.0 / 2.5 / 2.0 Vpp</b>，
        该粉档案里存了三阶段配方时按配方填，闭环在基准 ±0.5V 内加减。
      </p>
      <p v-if="recipeHint" class="recipe-hint">{{ recipeHint }}</p>
      <p v-if="ampBandHint" class="recipe-hint">{{ ampBandHint }}</p>
      <p class="points">{{ pointsText }}</p>
    </div>

    <!-- ================= 第三块：启停 ================= -->
    <div class="run-bar">
      <el-button type="primary" size="large" :disabled="!ready" :loading="submitting" @click="start">
        开始加粉
      </el-button>
      <el-button type="danger" plain size="large" :disabled="!running" @click="stop">停止加粉</el-button>
      <el-button :disabled="locked" @click="tare">天平标零</el-button>
      <el-button type="primary" plain :disabled="!ready" @click="enqueue">加入批量队列</el-button>
      <el-input-number v-model="repeat" :min="1" :max="20" :size="small" class="repeat"
        :disabled="locked" />
      <span class="unit">重复次数（批量里跑几遍）</span>
      <span class="run-state">{{ runText }}</span>
    </div>

    <!-- ================= 第四块：实时 + 结果 ================= -->
    <div class="monitor-grid">
      <div class="monitor-col">
        <el-progress :percentage="progressPct" :stroke-width="12" :status="progressStatus" />
        <div class="live-grid">
          <MetricCard label="当前质量" :value="massValue" :unit="`/ ${displayTarget} mg`" />
          <MetricCard label="流量" :value="rateValue" unit="mg/s" />
          <MetricCard label="振幅" :value="ampValue" unit="Vpp" />
          <MetricCard label="阶段" :value="stageValue" />
        </div>
        <p class="live-note">{{ liveNote }}</p>
      </div>

      <div class="result-col">
        <div v-if="!result" class="result-empty">
          本次还没跑过加粉。跑完之后这里显示判定、偏差、逐段明细和曲线。
        </div>
        <div v-else class="result-box">
          <div class="result-head">
            <b :class="result.verdict === '合格' ? 'good' : 'bad'">{{ result.verdict }}</b>
            <span>目标 <b>{{ result.target_mg }}</b> mg</span>
            <span>实得 <b>{{ result.achieved_mg }}</b> mg</span>
            <span :class="result.delta_mg > 0 ? 'bad' : 'good'">
              偏差 {{ result.delta_mg > 0 ? '+' : '' }}{{ result.delta_mg }} mg
            </span>
            <span class="muted">验收 {{ result.acceptance_mg?.[0] }}–{{ result.acceptance_mg?.[1] }} mg · {{ result.duration_s }}s</span>
          </div>
          <p class="result-stop">
            停止原因：{{ result.stop_reason }}
            <span v-if="result.estimated"> · ⚠ 天平读数中断，终值为最后一次已知值</span>
            <span v-if="result.log_path"> · 记录 {{ fileName(result.log_path) }}</span>
          </p>
          <div v-if="result.stages?.length" class="result-stages">
            <div v-for="s in result.stages" :key="s.stage" class="stage-line">
              <b>{{ s.stage_text }}</b>：{{ s.start_mass_mg }}→{{ s.end_mass_mg ?? '—' }} mg
              <span class="muted">
                （目标 {{ s.rate_target_mg_s }} / 实际 {{ s.actual_rate_mg_s ?? '—' }} mg/s，
                幅 {{ s.amp_min_vpp ?? s.amp_start_vpp ?? '—' }}–{{ s.amp_max_vpp ?? s.amp_end_vpp ?? '—' }}V
                （末 {{ s.amp_end_vpp ?? '—' }}，调 {{ s.amp_changes ?? '—' }} 次），
                杆 {{ s.rod_ff }}→{{ s.rod_final ?? '—' }}
                <template v-if="s.settle_gain_mg !== undefined">
                  ，切段沉降 +{{ s.settle_gain_mg }}mg/{{ s.settle_wait_s }}s{{ s.settle_stable ? '' : '（未判稳）' }}
                </template>
                ）
              </span>
            </div>
          </div>
          <div class="curve" v-html="curveSvg" />
        </div>
      </div>
    </div>
  </PanelSection>
</template>

<script setup>
import { computed, onMounted, onUnmounted, reactive, ref, watch } from 'vue'
import { ElMessage, ElMessageBox } from 'element-plus'
import { useDeviceStore } from '../stores/device.js'
import {
  getDispensePowders, getDispenseConfig, startDispense, cancelDispense,
  tareBalance, errorMessage,
} from '../api/index.js'
import { dispenseStageQueueItem } from '../utils/dispense.js'
import { useBatchQueue } from '../composables/useBatchQueue.js'
import PanelSection from './PanelSection.vue'
import MetricCard from './MetricCard.vue'

const STAGE_TEXT = { fast: '快加', coarse: '粗加', slow: '慢加' }
const STAGE_ORDER = ['fast', 'coarse', 'slow']
// 三阶段振幅默认（用户 2026-10-23 18:36 定）：快加 3.0 / 粗加 2.5 / 慢加 2.0 Vpp
const STAGE_AMP_DEFAULT = { fast: 3.0, coarse: 2.5, slow: 2.0 }
const CATEGORIES = [
  { value: 'Medium-flow', label: 'Medium-flow（中流量）' },
  { value: 'Low-flow', label: 'Low-flow（低流量）' },
  { value: 'High-flow', label: 'High-flow（高流量）' },
]

const store = useDeviceStore()
const { batchQueue, addToQueue } = useBatchQueue()

const powders = ref([])
const powderName = ref('')
const category = ref('Medium-flow')
const target = ref(500)
const ampOverride = ref(null)
const tailMass = ref(null)

const settleGate = ref(true)
const gateRate = ref(null)
const taperFactor = ref(0.6)
const surgeGuard = ref(true)
const surgeRatio = ref(2.5)
const trimEnabled = ref(false)
const trimEnter = ref(null)

const plan = ref(null)
const planError = ref('')
const submitting = ref(false)
const repeat = ref(1)

// 三行表格：由后端计划回填，用户可以逐格改；提交时就按这张表发
const stageRows = reactive(STAGE_ORDER.map((name) => ({
  name,
  enabled: name === 'slow',
  stir_rpm: null,
  rod_units: null,
  amplitude_vpp: STAGE_AMP_DEFAULT[name],
  rate_mg_s: null,
  duty_percent: null,
  switch_mass_mg: null,
  settle_wait_s: null,
})))

// 表里这套切点，是按哪个目标算出来的（防「改了目标没等重算就点开始」）
let filledTarget = null

const dispense = computed(() => store.dispense)
const live = computed(() => store.dispenseLive)
const result = computed(() => store.dispenseResult)
const running = computed(() => store.dispenseRunning)
const locked = computed(() => running.value || store.calibRunning || submitting.value)
const ready = computed(() => !locked.value && Number.isInteger(target.value) && target.value >= 100)

const displayTarget = computed(() => store.dispenseConfig.target_mg ?? target.value)
const statusLabel = computed(() => (running.value ? '运行中' : '待命'))
const stateText = computed(() => ({
  done: '上次已完成', aborted: '上次中止', cancelled: '上次已停止',
  idle: '空闲', running: '加粉中', settling: '结算中',
}[store.dispense.state] || '空闲'))
const runText = computed(() => store.dispense.detail || '空闲')

const massValue = computed(() => {
  const mass = live.value.mass_mg
  if (mass != null) return Number(mass).toFixed(1)
  const final = result.value?.achieved_mg
  return final != null ? Number(final).toFixed(1) : '--'
})
const rateValue = computed(() => (running.value && live.value.rate_mg_s != null
  ? Number(live.value.rate_mg_s).toFixed(1) : '0.0'))
const ampValue = computed(() => (running.value && live.value.amp_vpp != null
  ? Number(live.value.amp_vpp).toFixed(2) : '0.00'))
const stageValue = computed(() => {
  if (!running.value) return result.value ? '已结束' : '--'
  return live.value.stage_text || STAGE_TEXT[live.value.stage] || live.value.stage || '准备中'
})
const progressPct = computed(() => {
  const done = live.value.mass_mg ?? result.value?.achieved_mg ?? 0
  const goal = displayTarget.value || 1
  return Math.min(100, Math.max(0, Math.round((100 * done) / goal)))
})
const progressStatus = computed(() => {
  if (running.value) return ''
  if (!result.value) return ''
  return result.value.verdict === '合格' ? 'success' : 'exception'
})
const liveNote = computed(() => {
  if (running.value) {
    const remain = live.value.remaining_mg
    const elapsed = live.value.elapsed_s
    return `已用 ${elapsed ?? '—'}s · 还差 ${remain != null ? Math.max(0, remain).toFixed(0) : '—'} mg`
  }
  if (result.value) return `上次结束：${result.value.stop_reason}`
  return '等待启动加粉'
})

const powderInfo = computed(() => {
  if (!powderName.value) return '未选粉：搅拌全程 0，纯闭环对照，振动按默认 80Hz/50%'
  const p = powders.value.find((x) => x.powder_name === powderName.value)
  if (!p) return '—'
  const st = p.stage_stir_rpm || {}
  const sig = p.signal || {}
  const vib = sig.frequency_hz !== undefined
    ? ` · 振动 ${sig.frequency_hz}Hz/${sig.duty_percent}%/${sig.amplitude_vpp}Vpp`
    : ' · 无标定信号（按默认 80Hz/50%）'
  return `${p.points} 点 · 快加 ${st.coarse ?? '–'} / 粗加 ${st.fine ?? '–'} / 慢加 ${st.precision ?? '–'} RPM${vib}`
})

const planText = computed(() => {
  if (planError.value) return planError.value
  if (!plan.value) return '—'
  const p = plan.value
  const gate = p.settle_gate || {}
  const gateText = gate.enabled === false
    ? '沉降门关'
    : (gate.rate_mg_s == null
      ? `沉降门开·按段 ${gate.rate_ratio ?? 1.5}×`
      : `沉降门开≥${gate.rate_mg_s}mg/s`)
  const parts = (p.plan || []).map((s) => {
    const sw = s.switch_mass_mg == null ? p.target_mg : s.switch_mass_mg
    return `${s.stage_text} ${sw}mg`
  })
  return `${p.calibrated ? '按标定表' : '无标定，按预设'} · `
    + `振动 ${p.frequency_hz}Hz/${p.duty_percent}%（${p.signal_source === 'powder' ? '该粉标定' : '默认'}） · `
    + `${gateText} · 预计 ${p.estimated_s}s（超时上限 ${p.max_run_s}s） · ${parts.join(' → ')}`
})

const pointsText = computed(() => {
  const pts = plan.value?.calibration_points || []
  if (!pts.length) return '标定点：无（未选粉 / 该粉没标定，将用预设种子跑纯闭环）'
  return '标定点（可直接抄进上表）：' + pts.map((p) =>
    `${Math.round(p.stir_rpm)}rpm/${p.rod_units}→${p.rate_mg_s}mg/s`
    + (p.cv_pct > 40 ? `(CV${p.cv_pct}%不可信)` : '')).join('　')
})

// 档案配方（stage_recipe_v1）提示：让操作者知道这三行整行是哪来的、手改后按谁的跑
const recipeHint = computed(() => {
  const items = (plan.value?.stage_recipe?.stages || [])
    .filter((s) => STAGE_ORDER.includes(s?.stage))
  if (!items.length) return ''
  const brief = items.map((s) => {
    const bits = []
    if (s.stir_rpm != null) bits.push(`${Math.round(Number(s.stir_rpm))} RPM`)
    if (s.amplitude_vpp != null) bits.push(`${Number(s.amplitude_vpp).toFixed(2)} V`)
    if (s.rate_mg_s != null) bits.push(`${Number(s.rate_mg_s)} mg/s`)
    return `${STAGE_TEXT[s.stage]} ${bits.join('·')}`
  }).join(' → ')
  const edited = items.filter((s) => {
    const row = stageRows.find((r) => r.name === s.stage)
    if (!row) return false
    const ampDiff = s.amplitude_vpp == null || row.amplitude_vpp == null
      || Math.abs(Number(row.amplitude_vpp) - Number(s.amplitude_vpp)) > 0.005
    const stirDiff = s.stir_rpm != null && row.stir_rpm != null
      && Math.abs(Number(row.stir_rpm) - Number(s.stir_rpm)) > 0.5
    const rateDiff = s.rate_mg_s != null && row.rate_mg_s != null
      && Math.abs(Number(row.rate_mg_s) - Number(s.rate_mg_s)) > 0.05
    return ampDiff || stirDiff || rateDiff
  })
  if (edited.length) {
    return `档案配方 stage_recipe_v1：${brief} —— 表格已手改`
      + `（${edited.map((s) => STAGE_TEXT[s.stage]).join('、')}），按表格现值为准`
  }
  return `三行整套取自档案配方 stage_recipe_v1：${brief}`
    + `（搅拌/推杆/振幅/速率/切点都按档案，可手改，改了按表格现值为准；刷新不会被重算冲掉）`
})

// 振幅闭环带 + 零速率脉冲许可：都来自后端 plan.stages（StagePlan.to_dict），
// 也就是运行时真正生效的那套 cap/floor —— 页面不再自己猜"基准±0.5V"。
// 封堵剂2 这类粉在档案 dispense_flags 里把慢加放宽到 ±1.0V、并关掉零速率脉冲，
// 操作者从这一行就能直接看出"这一段振幅会在哪个区间走、顶上会不会踢一脚"。
const ampBandHint = computed(() => {
  const items = (plan.value?.stages || [])
    .filter((s) => STAGE_ORDER.includes(s?.stage) && Array.isArray(s?.amplitude_range_vpp))
  if (!items.length) return ''
  const parts = items.map((s) => {
    const lo = Number(s.amplitude_range_vpp[0]).toFixed(2)
    const hi = Number(s.amplitude_range_vpp[1]).toFixed(2)
    const span = s.amplitude_span_vpp
    const tag = span ? `，放宽到 ±${Number(span).toFixed(2)}V` : ''
    return `${STAGE_TEXT[s.stage]} ${lo}–${hi}V${tag}`
  })
  const noPulse = items.some((s) => s.zero_rate_pulse === false)
  return `振幅闭环带：${parts.join('　')}`
    + (noPulse ? '｜该粉标记：不许在加粉速率为 0 时脉冲（顶上不踢脚）' : '')
})

const curveSvg = computed(() => {
  const res = result.value
  if (!res || !res.curve || res.curve.length < 2) return ''
  const W = 560, H = 90, pad = 4
  const curve = res.curve
  const maxT = curve[curve.length - 1][0] || 1
  const maxM = Math.max(res.target_mg * 1.15, ...curve.map((p) => p[1]), 1)
  const x = (t) => pad + (W - 2 * pad) * (t / maxT)
  const y = (m) => H - pad - (H - 2 * pad) * (m / maxM)
  const points = curve.map((p) => `${x(p[0]).toFixed(1)},${y(p[1]).toFixed(1)}`).join(' ')
  const ty = y(res.target_mg).toFixed(1)
  return `<svg viewBox="0 0 ${W} ${H}" style="width:100%;height:${H}px">
    <line x1="${pad}" y1="${ty}" x2="${W - pad}" y2="${ty}" stroke="#94a3b8" stroke-dasharray="4 4" stroke-width="1"/>
    <polyline points="${points}" fill="none" stroke="#2563eb" stroke-width="1.8" stroke-linejoin="round"/>
  </svg>
  <div style="font-size:11px;color:#7a8ba3">虚线 = 目标 ${res.target_mg} mg，横轴 ${maxT.toFixed(0)}s</div>`
})

const fileName = (path) => String(path).split(/[\\/]/).pop()

function fail(error) {
  if (error === 'cancel' || error === 'close') return
  ElMessage.error(errorMessage(error))
}

async function loadPowders() {
  try {
    const data = await getDispensePowders()
    // 加粉只列已标定的粉：没标定的选了也拿不到前馈，等同纯闭环，列进来只会误导
    powders.value = (data?.powders || []).filter((p) => p.calibrated)
    if (powderName.value && !powders.value.some((p) => p.powder_name === powderName.value)) {
      powderName.value = ''
    }
    await refreshPreview(true)
  } catch (error) { fail(error) }
}

// 换粉时把「档案里那组」带出来：缓补开关、起始振幅、分类
function onPowderChange() {
  const p = powders.value.find((x) => x.powder_name === powderName.value)
  if (p) {
    trimEnabled.value = !!p.trim_pulse
    if (p.category) category.value = p.category
    const amp = p.signal?.amplitude_vpp
    if (amp != null && amp >= 0.01 && amp <= 4) ampOverride.value = amp
  }
  refreshPreview(true)
}

// 只把"用户改过的格子"发上去：跟后端默认一致的不传，避免把默认值固化成覆盖
function collectStages() {
  const rows = stageRows
    .filter((r) => r.enabled)
    .map((r) => {
      const item = { name: r.name }
      if (r.stir_rpm != null) item.stir_rpm = r.stir_rpm
      if (r.rod_units != null) item.rod_units = r.rod_units
      if (r.amplitude_vpp != null) item.amplitude_vpp = r.amplitude_vpp
      if (r.rate_mg_s != null) item.rate_mg_s = r.rate_mg_s
      if (r.duty_percent != null) item.duty_percent = r.duty_percent
      if (r.switch_mass_mg != null) item.switch_mass_mg = r.switch_mass_mg
      if (r.settle_wait_s != null) item.settle_wait_s = r.settle_wait_s
      return item
    })
  return rows.length ? rows : undefined
}

function fillStages(planRows, recipe) {
  // 档案里有阶段配方（stage_recipe_v1）→ 三行**整套**按档案填：搅拌 / 推杆 / 振幅 /
  // 目标速率 / 占空 / 切点。
  // 2026-10-27 14:06 用户："为什么我刷新了页面就不是了"——旧版只回填了振幅，
  // 搅拌/速率/切点仍取后端按标定表算的值；刷新会走 loadPowders → refreshPreview(true)，
  // 于是快加被打回 50RPM / 2.9mg·s⁻¹ / 1466.8mg。
  // 没有配方的粉（失水剂1、封堵剂1…）仍按"从标定表挑工作点"回填，行为不变。
  const recipeMap = {}
  for (const item of (recipe?.stages || [])) {
    if (STAGE_ORDER.includes(item?.stage)) recipeMap[item.stage] = item
  }
  const recipeTarget = Number(recipe?.target_mg) || 0
  // 切点跟档案目标绑定：目标改了按比例缩放；慢加那格留空
  // （后端要求切点 < 目标，填 2000 会被拒 1–1999）
  const scaledSwitch = (mg, name) => {
    if (name === 'slow' || mg == null) return null
    const ratio = recipeTarget > 0 ? target.value / recipeTarget : 1
    const v = Math.round(Number(mg) * ratio)
    if (!Number.isFinite(v) || v < 1) return null
    return Math.min(v, Math.max(1, target.value - 1))
  }
  for (const row of stageRows) {
    const rec = recipeMap[row.name]
    row.enabled = !!rec
    row.stir_rpm = rec?.stir_rpm != null ? Math.round(Number(rec.stir_rpm)) : null
    row.rod_units = rec?.rod_units != null ? Math.round(Number(rec.rod_units)) : null
    const amp = Number(rec?.amplitude_vpp)
    row.amplitude_vpp = Number.isFinite(amp) && amp >= 0.01 && amp <= 4
      ? amp
      : STAGE_AMP_DEFAULT[row.name]
    row.rate_mg_s = rec?.rate_mg_s ?? null
    row.duty_percent = rec?.duty_percent ?? null
    row.switch_mass_mg = scaledSwitch(rec?.switch_mass_mg, row.name)
    row.settle_wait_s = rec?.settle_wait_s ?? null
  }
  for (const item of planRows || []) {
    const row = stageRows.find((r) => r.name === item.stage)
    // 档案里有这一段的配方 → 以档案为准，不让算出来的工作点盖掉
    if (!row || recipeMap[row.name]) continue
    row.enabled = true
    row.stir_rpm = item.stir_rpm == null ? null : Math.round(item.stir_rpm)
    row.rod_units = item.rod_ff ?? null
    row.amplitude_vpp = STAGE_AMP_DEFAULT[row.name]
    row.rate_mg_s = item.rate_mg_s ?? null
    row.duty_percent = item.duty_seed_pct ?? null
    row.switch_mass_mg = item.switch_mass_mg ?? null
    row.settle_wait_s = item.settle_wait_s ?? null
  }
}

let previewTimer = null
// 请求序号：防抖只是减少调用次数，慢响应回来晚了仍可能盖掉新结果（改目标时最明显）
let planSeq = 0
function schedulePreview(fill) {
  clearTimeout(previewTimer)
  previewTimer = setTimeout(() => refreshPreview(fill), 350)
}

/**
 * 算计划。
 * @param {boolean} fill true = 用标定表重算并回填表格；false = 沿用表格里现有值只刷新摘要
 */
async function refreshPreview(fill) {
  const seq = ++planSeq
  planError.value = ''
  if (!Number.isInteger(target.value) || target.value < 100 || target.value > 20000) {
    planError.value = '目标质量需在 100–20000 mg'
    return
  }
  try {
    const data = await getDispenseConfig(powderName.value, target.value, {
      category: category.value || null,
      amplitude_vpp: ampOverride.value ?? null,
      tail_mass_mg: tailMass.value ?? null,
      // fill=true：不带 stages，让后端按标定表从头算；false：沿用表格里的值
      stages: fill ? null : collectStages(),
      settle_gate: settleGate.value,
      settle_gate_rate_mg_s: settleGate.value ? (gateRate.value ?? null) : null,
      taper_factor: taperFactor.value ?? null,
      surge_guard: surgeGuard.value,
      surge_ratio: surgeGuard.value ? (surgeRatio.value ?? null) : 0,
      trim_enabled: trimEnabled.value,
      trim_enter_remain_mg: trimEnabled.value ? (trimEnter.value ?? null) : null,
    })
    if (seq !== planSeq) return
    if (fill) {
      fillStages(data.plan, data.stage_recipe)
      filledTarget = target.value
      // 档案配方填进去的是**另一套工作点**（快加 100RPM / 50mg·s⁻¹ / 切点 1450），
      // 上面这次 data 是按标定表算的（561.4s、切点 1466.8），摘要会和表格对不上。
      // 所以按表格现值再算一次（fill=false，只刷新摘要，不再回填）。
      if (data.stage_recipe?.stages?.length) {
        plan.value = data
        schedulePreview(false)
        return
      }
    }
    plan.value = data
  } catch (error) {
    if (seq === planSeq) planError.value = `计划计算失败：${errorMessage(error)}`
  }
}

function buildPayload() {
  return {
    target_mg: target.value,
    powder: powderName.value || null,
    category: category.value || null,
    amplitude_vpp: ampOverride.value ?? null,
    tail_mass_mg: tailMass.value ?? null,
    stages: collectStages(),
    settle_gate: settleGate.value,
    settle_gate_rate_mg_s: settleGate.value ? (gateRate.value ?? null) : null,
    taper_factor: taperFactor.value ?? null,
    surge_guard: surgeGuard.value,
    surge_ratio: surgeGuard.value ? (surgeRatio.value ?? null) : 0,
    trim_enabled: trimEnabled.value,
    trim_enter_remain_mg: trimEnabled.value ? (trimEnter.value ?? null) : null,
    trim_pulse_target_mg: null,
  }
}

async function start() {
  // 目标改过、表格还是上一套切点 → 先按新目标重算再提交
  if (filledTarget !== target.value) {
    await refreshPreview(true)
    if (filledTarget !== target.value) {
      ElMessage.error('三阶段配方没跟上目标质量，自动重算失败——请点「按标定表重算」再开始')
      return
    }
    ElMessage.info(`已按新目标 ${target.value} mg 重算三阶段切点`)
  }
  const enabled = stageRows.filter((r) => r.enabled)
  if (!enabled.length) { ElMessage.error('三阶段至少要启用一段'); return }

  const switches = enabled.map((r) => r.switch_mass_mg)
    .filter((v) => v != null && Number.isFinite(v))
  const lastSwitch = switches.length ? Math.max(...switches) : 0
  const slowRemain = Math.max(0, target.value - lastSwitch)
  const slowWarn = slowRemain > Math.max(0.3 * target.value, 150)
    ? `\n\n⚠ 慢加段要补 ${slowRemain.toFixed(0)} mg（占目标 `
      + `${(slowRemain / Math.max(1, target.value) * 100).toFixed(0)}%）——`
      + '正常收尾只该 5~15%，切点可能没按新目标重算'
    : ''

  const brief = enabled.map((r) =>
    `${STAGE_TEXT[r.name]}：搅 ${r.stir_rpm ?? '自动'} / 杆 ${r.rod_units ?? '自动'} / `
    + `幅 ${r.amplitude_vpp ?? '自动'}V / ${r.rate_mg_s ?? '自动'} mg/s / `
    + `加到 ${r.switch_mass_mg ?? '收尾'} mg`).join('\n')

  const gateText = settleGate.value
    ? `切段沉降门：开（切出速率 ≥ ${gateRate.value ?? '本段规定速率×1.5'}，`
      + `质量+速率×1s 够到切点就提前停振；提前 ${taperFactor.value ?? 0.6}×速率降幅）\n`
    : '切段沉降门：关（达标即刻切段）\n'
  const surgeText = surgeGuard.value
    ? `超速熔断：开（段内 0.7s 速率 ≥ 本段规定速率×${surgeRatio.value ?? 2.5}，`
      + '或剩余量不够这一瞬间的落料 → 立刻停振等余料）\n'
    : '超速熔断：关（塌料只能靠超重保护）\n'
  const trimText = trimEnabled.value
    ? `缓补：开（慢加剩 ≤${trimEnter.value ?? 40}mg 停连续振动，改 0.40s 真脉冲，只调振幅）\n`
    : '缓补：关（慢加直接判停收尾；收尾偏轻时仍按慢加行振幅定幅补差，补进验收带就停）\n'

  try {
    await ElMessageBox.confirm(
      `开始闭环加粉：目标 ${target.value} mg\n`
      + `粉末：${powderName.value || '（不选粉，纯闭环）'}\n${brief}\n`
      + `${gateText}${surgeText}${trimText}\n`
      + '推杆整段锁死、收尾不复位。运行中请勿手动操作振动/推杆/升降/搅拌。'
      + slowWarn + '\n\n确认机构周围安全后继续？',
      '开始加粉', { type: 'warning', customClass: 'pre-wrap' })
  } catch { return }

  submitting.value = true
  try {
    const response = await startDispense(buildPayload())
    ElMessage.success('加粉已启动 → 目标 ' + target.value + ' mg'
      + (response?.detail ? '：' + response.detail : ''))
    await store.fetchStatus()
  } catch (error) { fail(error) } finally { submitting.value = false }
}

async function stop() {
  try {
    const r = await cancelDispense()
    ElMessage.info(r?.stopped ? '已请求停止（正在停振并结算）' : '当前没有运行中的加粉任务')
  } catch (error) { fail(error) }
}

async function tare() {
  try { await tareBalance(); ElMessage.success('天平已标零') } catch (error) { fail(error) }
}

function enqueue() {
  try {
    const item = dispenseStageQueueItem({ ...buildPayload(), repeat_count: repeat.value })
    const other = batchQueue.value.find(
      (q) => (q.params?.powder || '') !== (item.params.powder || ''))
    if (other) {
      throw new Error(`队列里已有「${other.params.powder || '不选粉'}」的任务：`
        + '同一批只能用一种粉末，请先到「批量测试」页清空队列')
    }
    addToQueue(item)
    ElMessage.success(`已加入批量队列：${target.value} mg × ${repeat.value}`
      + `（当前队列 ${batchQueue.value.length} 行）`)
  } catch (error) { fail(error) }
}

watch(target, () => schedulePreview(true))
watch([settleGate, gateRate, taperFactor, surgeGuard, surgeRatio, trimEnabled, trimEnter],
  () => schedulePreview(false))
onUnmounted(() => { clearTimeout(previewTimer) })
onMounted(() => { loadPowders(); store.startPolling(500) })
</script>

<style scoped>
.lead { margin: 0 0 18px; font-size: 12px; line-height: 1.8; color: #6b7c91; }
.lead b { color: #3d5573; }
.setup-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 28px; }
.setup-col { min-width: 0; }
.col-title { margin: 0 0 12px; font-size: 13px; font-weight: 600; color: #3a5273;
  padding-left: 9px; border-left: 3px solid #409eff; line-height: 1.2; }
.form :deep(.el-form-item) { margin-bottom: 12px; }
.row { display: flex; align-items: center; gap: 8px; flex-wrap: wrap; width: 100%; }
.row :deep(.el-button + .el-button) { margin-left: 0; }
.grow { flex: 1; min-width: 120px; }
.unit { font-size: 12px; color: #8b9aab; white-space: nowrap; }
.powder-info { margin: 4px 0 0 92px; font-size: 12px; color: #8294a8; line-height: 1.6; }

.stage-block { margin-top: 24px; padding-top: 20px; border-top: 1px solid #eef1f6; }
.stage-head { display: flex; align-items: center; gap: 12px; flex-wrap: wrap; margin-bottom: 12px; }
.stage-head .col-title { margin: 0; }
.plan-info { font-size: 12px; color: #7b8b9f; flex: 1; min-width: 200px; line-height: 1.6; }
.plan-info.bad { color: #dc2626; }
.stage-table { width: 100%; border-collapse: collapse; font-size: 12px; }
.stage-table th {
  text-align: left; font-weight: 600; color: #7b8b9f; padding: 6px 6px;
  border-bottom: 1px solid #e6ecf5; white-space: nowrap;
}
.stage-table td { padding: 4px 6px; border-bottom: 1px solid #f2f5fa; }
.stage-table .c-stage { width: 56px; color: #33495f; font-weight: 600; white-space: nowrap; }
.stage-table .c-en { width: 52px; }
.stage-table :deep(.el-input-number) { width: 100%; }
.stage-hint { margin: 12px 0 6px; font-size: 11.5px; line-height: 1.75; color: #8b9aab; }
.stage-hint b { color: #6d7f93; }
.recipe-hint {
  margin: 6px 0; padding: 5px 9px; font-size: 11.5px; line-height: 1.7;
  color: #1d4ed8; background: #eff6ff; border-left: 3px solid #60a5fa; border-radius: 2px;
}
.points { margin: 0; font-size: 12px; color: #7b8b9f; line-height: 1.7; }

.run-bar {
  display: flex; align-items: center; gap: 10px; flex-wrap: wrap;
  margin: 22px 0 18px; padding-top: 18px; border-top: 1px solid #eef1f6;
}
.run-bar :deep(.el-button + .el-button) { margin-left: 0; }
.repeat { width: 96px; }
.run-state { font-size: 12px; color: #7b8b9f; }

.monitor-grid { display: grid; grid-template-columns: minmax(0, 1fr) minmax(0, 1.15fr); gap: 24px; }
.monitor-col, .result-col { min-width: 0; }
.live-grid { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 10px; margin: 16px 0 10px; }
.live-grid :deep(.value-text) { font-size: 22px; }
.live-note { margin: 0; font-size: 12px; color: #7b8b9f; line-height: 1.7; }

.result-empty {
  border: 1px dashed #dce3ec; border-radius: 10px; padding: 26px 18px;
  font-size: 12px; color: #8b9aab; line-height: 1.8; background: #fafbfd;
}
.result-box { border: 1px solid #e6ecf5; border-radius: 10px; padding: 14px 16px; background: #fbfcfe; }
.result-head { display: flex; align-items: baseline; gap: 14px; flex-wrap: wrap; font-size: 13px; color: #46596f; }
.result-head b { font-size: 15px; }
.result-head b.good, .good { color: #059669; }
.result-head b.bad, .bad { color: #dc2626; }
.result-stop { margin: 6px 0 0; font-size: 12px; color: #7b8b9f; line-height: 1.7; }
.result-stages { margin-top: 10px; font-size: 12px; color: #5b6e85; line-height: 1.9; }
.stage-line b { color: #33495f; }
.muted { color: #93a1b3; }
.curve { margin-top: 10px; }

@media (max-width: 1200px) {
  .setup-grid { grid-template-columns: 1fr; gap: 8px; }
  .monitor-grid { grid-template-columns: 1fr; }
}
</style>
