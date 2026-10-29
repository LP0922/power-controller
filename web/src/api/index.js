import axios from 'axios'

// 目前只有一个后端（8765，v7 三阶段引擎），所有接口都直接对着它。
// 老版 device_control_server.py 里那套「窗口位置 + 频率/占空 + 粗/细/精速率」
// 的接口（/window/*、/constant-rate/*、/test-records、/experiment/grid/*、
// /powder/fingerprints…）在这个后端里根本不存在，2026-10-26 融合时整段删掉了，
// 留着只会点出 404。需要什么就往下加，不要从老后端抄名字。
const api = axios.create({
  baseURL: '/api',
  timeout: 30000,
  headers: { 'Content-Type': 'application/json' },
})

// ---------------------------------------------------------------- 状态
/** 设备总状态：lift/rod/stir/bal/sig/dispense/calib/errors/sync 一次全拿到 */
export async function getStatus() {
  const { data } = await api.get('/status', { timeout: 8000 })
  return data
}

// 老版兼容快照（at8811c / la10 / test 三段结构），只有老组件还在用
export async function getSnapshot() {
  const { data } = await api.get('/snapshot', { timeout: 8000 })
  return data
}

export async function sendLog(message, level = 'info') {
  const { data } = await api.post('/log', { message, level })
  return data
}

// ---------------------------------------------------------------- 升降（双 MD30 广播同步）
export const moveLift = async (target) => (await api.post('/lift/move', { target })).data
export const homeLift = async () => (await api.post('/lift/home')).data
export const stopLift = async () => (await api.post('/lift/stop')).data
export const alignLift = async () => (await api.post('/lift/align')).data
export const ackSyncAlarm = async () => (await api.post('/lift/ack')).data

// ---------------------------------------------------------------- 推杆（因时 LA）
export const moveRod = async (target) => (await api.post('/rod/move', { target })).data
export const homeRod = async () => (await api.post('/rod/home')).data
export const stopRod = async () => (await api.post('/rod/stop')).data
export const refreshRod = async () => (await api.post('/rod/refresh')).data

// ---------------------------------------------------------------- 旋转搅拌（ZDT Emm42）
export const startStir = async ({ rpm, acc = 200, direction = 0 }) =>
  (await api.post('/stir/start', { rpm, acc, direction })).data
export const stopStir = async ({ mode = 'decel', acc = 200 } = {}) =>
  (await api.post('/stir/stop', { mode, acc })).data
export const clearStirStall = async () => (await api.post('/stir/clear-stall')).data
export const setStirEnable = async (on) => (await api.post('/stir/enable', { on })).data

// ---------------------------------------------------------------- 天平（AT8811C）
export const tareBalance = async () => (await api.post('/bal/tare')).data
export const refreshBalance = async () => (await api.post('/bal/refresh')).data
export const setBalanceBase = async (clear = false) => (await api.post('/bal/base', { clear })).data
export const sampleStableBalance = async ({ timeout_s = 6, samples = 8 } = {}) =>
  (await api.post('/bal/stable', { timeout_s, samples })).data

// ---------------------------------------------------------------- 振动给料（QLS3600S 信号发生器）
export const setSignalWaveform = async (waveform) =>
  (await api.post('/sig/waveform', { waveform })).data
export const setSignalFreq = async (hz) => (await api.post('/sig/freq', { hz })).data
export const setSignalAmp = async (vpp) => (await api.post('/sig/amp', { vpp })).data
export const setSignalDuty = async (percent) => (await api.post('/sig/duty', { percent })).data
export const setSignalOutput = async (on) => (await api.post('/sig/output', { on })).data
export const runSignalFor = async (payload) => (await api.post('/sig/run', payload)).data
export const stopSignal = async () => (await api.post('/sig/stop')).data

// ---------------------------------------------------------------- 闭环加粉（三阶段）
export const getDispensePowders = async () => (await api.get('/dispense/powders')).data

/**
 * 只算计划、不动硬件。
 * @param {object} payload target_mg / powder / category / amplitude_vpp / tail_mass_mg /
 *                         stages / settle_gate / settle_gate_rate_mg_s / taper_factor /
 *                         surge_guard / surge_ratio / trim_enabled / trim_enter_remain_mg
 * GET 版只带 powder/category/target，POST 版能带全套参数（含 stages 覆盖）。
 */
export async function getDispenseConfig(powder, targetMg, extra = {}) {
  const body = { target_mg: targetMg, powder: powder || null, ...extra }
  const { data } = await api.post('/dispense/plan', body)
  return data
}

/** 用 GET 快速探一个计划（不带 stages 覆盖），做预警用 */
export async function getDispensePlanQuick(powder, targetMg) {
  const { data } = await api.get('/dispense/plan', {
    params: { target_mg: targetMg, powder: powder || undefined },
  })
  return data
}

export const startDispense = async (payload) => (await api.post('/dispense/start', payload, { timeout: 60000 })).data
export const cancelDispense = async () => (await api.post('/dispense/stop')).data

// ---------------------------------------------------------------- 批量（串行排队，每项仍是一次正常加粉）
export const startDispenseBatch = async (payload) =>
  (await api.post('/dispense/batch', payload, { timeout: 60000 })).data
export const getDispenseBatchStatus = async () => (await api.get('/dispense/batch/status')).data
export const cancelDispenseBatch = async () => (await api.post('/dispense/batch/stop')).data

// ---------------------------------------------------------------- 粉末库 + 标定
export const listPowders = async () => (await api.post('/powder/list')).data
export const createPowder = async (name, category = null) =>
  (await api.post('/powder/create', { name, category })).data
export const getPowderDetail = async (name) => (await api.post('/powder/detail', { name })).data
export const deletePowderPoints = async (name, payload) =>
  (await api.post('/powder/points/delete', { name, ...payload })).data
export const forgetPowderCalibration = async (name) =>
  (await api.post('/powder/forget', { name })).data

export const getCalibOptions = async () => (await api.post('/calib/options')).data
export const startCalibration = async (payload) => (await api.post('/calib/start', payload)).data
export const stopCalibration = async () => (await api.post('/calib/stop')).data

// ---------------------------------------------------------------- 全局
export const emergencyStop = async () => (await api.post('/estop')).data

// 试验记录：目前后端还没实现（老后端的 /test-records 没搬过来），
// 「试验记录」页也还是待接入状态。这里保留导出只是为了让那个页面能编译，
// 真要接的时候数据源是 runs/dispense_*.json 与 runs/batch_*.json。
export async function getTestRecords(params = {}) {
  const { data } = await api.get('/test-records', { params })
  return data
}

export function errorMessage(error) {
  return error?.response?.data?.error || error?.message || '请求失败'
}

export default api
