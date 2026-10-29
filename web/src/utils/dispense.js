// 加粉参数的新模型（2026-10-26 融合）：
// 老版是「窗口位置 + 频率/占空 + 粗/细/精速率」，新版是「三阶段 + 振幅闭环」。
// 老那套换算函数（validateInitial / dispensePayload / largeDispensePayload /
// dispenseQueueItem / expandGrid / gridPayload / batchResults）没有对应接口了，
// 整段删掉，免得又有人照着调出一个 404。

export const TARGET_MIN_MG = 100
export const TARGET_MAX_MG = 20000

export const isValidTarget = (value) => (
  Number.isInteger(value) && value >= TARGET_MIN_MG && value <= TARGET_MAX_MG
)

function integer(value, min, max, label) {
  if (!Number.isInteger(value) || value < min || value > max) {
    throw new Error(`${label}必须是 ${min}–${max} 之间的整数`)
  }
}

/** 试验记录页还在用的查询参数构造（该页尚未接入目前后端，保留以免 build 断） */
export function recordQuery(filters) {
  const query = {}
  if (filters.type) query.test_type = filters.type
  if (filters.dateFrom) query.date_from = filters.dateFrom.replaceAll('-', '')
  if (filters.dateTo) query.date_to = filters.dateTo.replaceAll('-', '')
  if (filters.result) query.result = filters.result
  return query
}

/**
 * 三阶段模型的批量项：加粉面板点「加入批量队列」时用它拼。
 * 与「开始加粉」提交的 payload 完全同构 —— 批量跑出来的和单点跑出来的必须是同一套参数，
 * 否则两边结果不可比。只带引擎真会消费的键。
 */
export function dispenseStageQueueItem({
  target_mg, powder, category, amplitude_vpp, tail_mass_mg,
  stages, settle_gate, settle_gate_rate_mg_s, taper_factor,
  surge_guard, surge_ratio, trim_enabled, trim_enter_remain_mg,
  repeat_count = 1,
}) {
  const target = Number(target_mg)
  if (!isValidTarget(target)) {
    throw new Error(`目标质量需为 ${TARGET_MIN_MG}–${TARGET_MAX_MG} mg 的整数`)
  }
  integer(repeat_count, 1, 20, '重复次数')
  const params = {
    target_mg: target,
    powder: powder || null,
    repeat_count,
    category: category || null,
    amplitude_vpp: amplitude_vpp ?? null,
    tail_mass_mg: tail_mass_mg ?? null,
    settle_gate: settle_gate ?? null,
    settle_gate_rate_mg_s: settle_gate_rate_mg_s ?? null,
    taper_factor: taper_factor ?? null,
    surge_guard: surge_guard ?? null,
    surge_ratio: surge_ratio ?? null,
    trim_enabled: trim_enabled ?? null,
    trim_enter_remain_mg: trim_enter_remain_mg ?? null,
  }
  if (stages && stages.length) params.stages = stages
  return { type: 'dispense', typeLabel: '加粉', params }
}

export function validateBatchQueue(queue) {
  if (!queue.length || queue.length > 20) throw new Error('批量队列需为 1–20 组')
  const types = new Set(queue.map((item) => item.type))
  if (types.size !== 1) throw new Error('不同任务类型请分批执行，不能混合运行')
  if (queue[0].type !== 'dispense') {
    throw new Error('当前只支持「加粉」批量：定速引擎已下线，加粉统一走三阶段振幅闭环')
  }
  for (const item of queue) {
    const target = item.params?.target_mg
    if (!isValidTarget(target)) {
      throw new Error(`每一项目标质量需为 ${TARGET_MIN_MG}–${TARGET_MAX_MG} mg 的整数`)
    }
    integer(item.params?.repeat_count ?? 1, 1, 20, '重复次数')
  }
  if (new Set(queue.map((item) => item.params?.powder || '')).size !== 1) {
    throw new Error('同一批量队列只能使用一种粉末，请先清空现有加粉任务')
  }
  const total = queue.reduce((sum, item) => sum + (item.params?.repeat_count ?? 1), 0)
  if (total > 40) throw new Error(`展开后共 ${total} 项，超过后端上限 40 项`)
}

/** 批量队列里一行参数的摘要文字（表格里显示用） */
export function stageSummary(stages) {
  if (!Array.isArray(stages) || !stages.length) return '—'
  const text = { fast: '快', coarse: '粗', slow: '慢' }
  return stages.map((s) => {
    const bits = []
    if (s.amplitude_vpp != null) bits.push(`${s.amplitude_vpp}V`)
    if (s.rod_units != null) bits.push(`杆${s.rod_units}`)
    if (s.stir_rpm != null) bits.push(`${s.stir_rpm}rpm`)
    return `${text[s.name] || s.name}:${bits.join('/') || '默认'}`
  }).join('  ')
}
