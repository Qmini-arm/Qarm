import { useEffect, useMemo, useState } from 'react'
import * as api from './api'

type Tab = 'control' | 'status' | 'config' | 'programs'

type Joint = {
  name: string
  id: number
  angle: number
  velocity: number
  torque: number
  temperature: number
  min: number
  max: number
}

const initialJoints: Joint[] = [
  { name: 'J1', id: 0, angle: -0.473, velocity: 0.006, torque: -0.012, temperature: 31, min: -2.967, max: 2.967 },
  { name: 'J2', id: 1, angle: 0.883, velocity: -0.012, torque: -1.31, temperature: 33, min: -1.57, max: 1.57 },
  { name: 'J3', id: 2, angle: -1.662, velocity: 0.008, torque: 0.42, temperature: 32, min: -2.007, max: 2.007 },
  { name: 'J4', id: 3, angle: -1.268, velocity: 0.004, torque: 0.021, temperature: 30, min: -2, max: 2 },
  { name: 'J5', id: 4, angle: -0.02, velocity: 0.003, torque: -0.001, temperature: 29, min: -2.007, max: 2.007 },
  { name: 'J6', id: 5, angle: -0.495, velocity: -0.003, torque: 0.002, temperature: 29, min: -1.5, max: 1.5 },
]

const reference = [0, 1.748, 0.155, -0.02, 0, -1.57]

function clamp(value: number, min: number, max: number) {
  return Math.min(max, Math.max(min, value))
}

function format(value: number, digits = 3) {
  return value.toFixed(digits)
}

function StatusPill({ connected, enabled }: { connected: boolean; enabled: boolean }) {
  return (
    <div className="status-pill">
      <span className={`status-dot ${connected ? 'green' : 'gray'}`} />
      <span>{connected ? '已连接' : '未连接'}</span>
      <span className="status-divider" />
      <span className={`status-dot ${enabled ? 'orange' : 'gray'}`} />
      <span>{enabled ? '已使能' : '已掉使能'}</span>
    </div>
  )
}

function ArmViewport({ joints, viserUrl }: { joints: Joint[]; viserUrl: string }) {
  const [useViser, setUseViser] = useState(false)
  const q = joints.map((joint) => joint.angle)
  const points = [
    [150, 215],
    [150 + 62 * Math.cos(q[1]), 215 - 62 * Math.sin(q[1])],
    [150 + 62 * Math.cos(q[1]) + 76 * Math.cos(q[1] + q[2]), 215 - 62 * Math.sin(q[1]) - 76 * Math.sin(q[1] + q[2])],
    [150 + 62 * Math.cos(q[1]) + 76 * Math.cos(q[1] + q[2]) + 50 * Math.cos(q[1] + q[2] + q[3]), 215 - 62 * Math.sin(q[1]) - 76 * Math.sin(q[1] + q[2]) - 50 * Math.sin(q[1] + q[2] + q[3])],
  ]
  return (
    <div className="viewport">
      <div className="viewport-toolbar"><span>{useViser ? 'Viser 3D 实时视图' : '本地姿态预览'}</span><span className="view-chip">BASE_LINK</span><button className={useViser ? 'viser-toggle active' : 'viser-toggle'} onClick={() => setUseViser((value) => !value)}>{useViser ? '本地预览' : '打开 Viser'}</button><button aria-label="重置视角">↺</button></div>
      {useViser ? <div className="viser-frame"><iframe title="Qarm Viser 3D 视图" src={viserUrl} /><div className="viser-hint">Viser 地址：{viserUrl}</div></div> : <svg viewBox="0 0 420 320" role="img" aria-label="机械臂姿态视图">
        <defs>
          <linearGradient id="armGradient" x1="0" x2="1">
            <stop offset="0" stopColor="#45d4b5" /><stop offset="1" stopColor="#38a6ff" />
          </linearGradient>
          <pattern id="grid" width="24" height="24" patternUnits="userSpaceOnUse">
            <path d="M 24 0 L 0 0 0 24" fill="none" stroke="#dce5ee" strokeWidth="1" />
          </pattern>
        </defs>
        <rect x="0" y="0" width="420" height="320" fill="url(#grid)" opacity=".55" />
        <line x1="30" y1="260" x2="390" y2="260" stroke="#b8c7d6" strokeWidth="2" />
        <line x1="150" y1="40" x2="150" y2="260" stroke="#d8e3eb" strokeDasharray="5 5" />
        <line x1="90" y1="215" x2="210" y2="215" stroke="#ff7b61" strokeWidth="2" opacity=".7" />
        <polyline points={points.map(([x, y]) => `${x},${y}`).join(' ')} fill="none" stroke="url(#armGradient)" strokeWidth="22" strokeLinecap="round" strokeLinejoin="round" />
        {points.map(([x, y], index) => <circle key={index} cx={x} cy={y} r={index === 0 ? 13 : 9} fill="#0d2333" stroke="#fff" strokeWidth="3" />)}
        <circle cx={points[3][0]} cy={points[3][1]} r="5" fill="#f59e0b" />
        <text x="314" y="30" fill="#718296" fontSize="11">x</text><text x="164" y="45" fill="#718296" fontSize="11">z</text>
        <text x="164" y="281" fill="#718296" fontSize="11">桌面支撑参考平面</text>
      </svg>}
      <div className="viewport-footer"><span><i className="legend-line cyan" />实时姿态</span><span><i className="legend-line coral" />标定支撑平面</span><span>末端工具：tool0</span></div>
    </div>
  )
}

function App() {
  const [tab, setTab] = useState<Tab>('control')
  const [connected, setConnected] = useState(true)
  const [enabled, setEnabled] = useState(false)
  const [estop, setEstop] = useState(false)
  const [gravity, setGravity] = useState(false)
  const [joints, setJoints] = useState(initialJoints)
  const [speed, setSpeed] = useState(0.25)
  const [acceleration, setAcceleration] = useState(0.5)
  const [notice, setNotice] = useState('系统就绪，等待动作')
  const [lastAction, setLastAction] = useState('')
  const [apiOnline, setApiOnline] = useState(false)
  const [apiError, setApiError] = useState('')
  const viserUrl = import.meta.env.VITE_VISER_URL || 'http://127.0.0.1:8080'

  useEffect(() => {
    let disposed = false
    const sync = async () => {
      try {
        const status = await api.getStatus()
        if (disposed) return
        setApiOnline(true)
        setApiError('')
        setConnected(status.connected)
        setEnabled(status.enabled)
        setEstop(status.estop)
        setGravity(status.gravity)
        setLastAction(status.last_action || '')
        setNotice(status.notice || '后端已连接')
        setJoints((current) => status.joints.map((item, index) => ({
          ...(current.find((joint) => joint.id === item.id) || current[index]),
          name: item.name || current[index]?.name || `J${index + 1}`,
          id: item.id,
          angle: Number(item.angle),
          velocity: Number(item.velocity),
          torque: Number(item.torque),
          temperature: Number(item.temperature),
          error: item.error,
        })))
      } catch (error) {
        if (disposed) return
        setApiOnline(false)
        setApiError(error instanceof Error ? error.message : '控制服务不可达')
      }
    }
    void sync()
    const statusTimer = window.setInterval(() => void sync(), 1000)
    return () => { disposed = true; window.clearInterval(statusTimer) }
  }, [])

  useEffect(() => {
    if (apiOnline || !connected || estop) return
    const timer = window.setInterval(() => {
      setJoints((current) => current.map((joint) => ({
        ...joint,
        velocity: Number((joint.velocity * 0.86 + (Math.random() - 0.5) * 0.004).toFixed(4)),
        temperature: Number((joint.temperature + (enabled ? 0.002 : -0.004)).toFixed(1)),
      })))
    }, 800)
    return () => window.clearInterval(timer)
  }, [apiOnline, connected, enabled, estop])

  const averageTemp = useMemo(() => joints.reduce((sum, joint) => sum + joint.temperature, 0) / joints.length, [joints])

  function setJointAngle(index: number, value: number) {
    setJoints((current) => current.map((joint, item) => item === index ? { ...joint, angle: clamp(value, joint.min, joint.max) } : joint))
  }

  async function runJointMotion() {
    if (!enabled) { setNotice('请先使能机械臂'); return }
    if (estop) { setNotice('急停状态已锁定运动'); return }
    try {
      const status = await api.movej(joints.map((joint) => joint.angle), speed, acceleration)
      setApiOnline(true); setApiError(''); setLastAction(status.last_action || 'MOVEJ'); setNotice(status.notice)
    } catch (error) {
      setApiOnline(false)
      setApiError(error instanceof Error ? error.message : 'MOVEJ 请求失败')
      setLastAction('MOVEJ (SIM)')
      setNotice(`本地仿真：MOVEJ 已完成，速度 ${speed.toFixed(2)} rad/s，加速度 ${acceleration.toFixed(2)} rad/s²`)
    }
  }

  async function returnCalibrationPose() {
    if (!enabled) { setNotice('请先使能机械臂'); return }
    try {
      const status = await api.returnHome(import.meta.env.VITE_HOME_TRAJECTORY || 'build/calibration_home.csv')
      setApiOnline(true); setApiError(''); setLastAction(status.last_action || 'RETURN_CALIBRATION'); setNotice(status.notice)
    } catch (error) {
      setApiOnline(false)
      setApiError(error instanceof Error ? error.message : '回零请求失败')
      setJoints((current) => current.map((joint, index) => ({ ...joint, angle: reference[index] })))
      setLastAction('RETURN_CALIBRATION (SIM)')
      setNotice('本地仿真：已回到桌面支撑标定位；真机需配置有效轨迹并人工确认')
    }
  }

  async function emergencyStop() {
    try {
      const status = await api.setEstop()
      setApiOnline(true); setApiError(''); setEstop(status.estop); setEnabled(status.enabled); setGravity(status.gravity); setNotice(status.notice)
    } catch (error) {
      setApiOnline(false); setApiError(error instanceof Error ? error.message : '急停请求失败')
      setEstop(true); setEnabled(false); setGravity(false); setLastAction('ESTOP'); setNotice('本地仿真：急停已触发，所有动作已停止')
    }
  }

  async function clearEmergencyStop() {
    try {
      const status = await api.clearEstop()
      setApiOnline(true); setApiError(''); setEstop(status.estop); setNotice(status.notice)
    } catch (error) {
      setApiOnline(false); setApiError(error instanceof Error ? error.message : '解除急停请求失败')
      setEstop(false); setNotice('本地仿真：急停已解除，请重新使能')
    }
  }

  async function toggleConnection() {
    try {
      const status = connected ? await api.disconnect() : await api.connect()
      setApiOnline(true); setApiError(''); setConnected(status.connected); setEnabled(status.enabled); setGravity(status.gravity); setNotice(status.notice)
    } catch (error) {
      setApiOnline(false); setApiError(error instanceof Error ? error.message : '连接请求失败')
      setConnected((value) => !value); setNotice('本地仿真：连接状态已切换')
    }
  }

  async function toggleEnabled() {
    if (estop) return
    try {
      const status = await api.setEnabled(!enabled)
      setApiOnline(true); setApiError(''); setEnabled(status.enabled); setConnected(status.connected); setNotice(status.notice)
    } catch (error) {
      setApiOnline(false); setApiError(error instanceof Error ? error.message : '使能请求失败')
      setEnabled((value) => !value); setNotice(`本地仿真：${enabled ? '机械臂已掉使能' : '机械臂已使能'}`)
    }
  }

  async function toggleGravity() {
    if (!enabled || estop) return
    try {
      const status = await api.setGravity(!gravity)
      setApiOnline(true); setApiError(''); setGravity(status.gravity); setNotice(status.notice)
    } catch (error) {
      setApiOnline(false); setApiError(error instanceof Error ? error.message : '重力补偿请求失败')
      setGravity((value) => !value); setNotice(`本地仿真：${gravity ? '重力补偿已关闭' : '100% 重力补偿已开启'}`)
    }
  }

  return (
    <div className="app-shell">
      <header className="topbar">
        <div className="brand"><div className="brand-mark">Q</div><div><strong>Qarm Control</strong><span>六轴机械臂控制平台</span></div></div>
        <div className="topbar-actions">
          <label className="select-label">工具坐标系<select defaultValue="tool0"><option value="tool0">tool0</option><option value="gripper">gripper</option></select></label>
          <StatusPill connected={connected} enabled={enabled} />
          <button className="ghost-button" onClick={toggleConnection}>{connected ? '断开连接' : '重新连接'}</button>
          <button className="estop-button" onClick={estop ? clearEmergencyStop : emergencyStop}>{estop ? '解除急停' : '急停'}</button>
        </div>
      </header>
      <div className="layout">
        <aside className="sidebar">
          <div className="side-caption">控制台</div>
          <button className={tab === 'control' ? 'side-item active' : 'side-item'} onClick={() => setTab('control')}><span>⌁</span>机械臂控制</button>
          <button className={tab === 'status' ? 'side-item active' : 'side-item'} onClick={() => setTab('status')}><span>◌</span>状态监控</button>
          <button className={tab === 'config' ? 'side-item active' : 'side-item'} onClick={() => setTab('config')}><span>⚙</span>机械臂配置</button>
          <button className={tab === 'programs' ? 'side-item active' : 'side-item'} onClick={() => setTab('programs')}><span>▦</span>在线编程</button>
          <div className="sidebar-spacer" />
          <div className="connection-card"><span className={`status-dot ${connected ? 'green' : 'gray'}`} /><div><strong>{connected ? (apiOnline ? '控制器在线' : '本地仿真在线') : '控制器离线'}</strong><small>{apiOnline ? '192.168.10.102 · M8010' : '后端不可达 · simulation'}</small></div></div>
          <div className="version">Qarm Platform v0.1.0</div>
        </aside>
        <main className="main-content">
          <div className="page-heading"><div><div className="eyebrow">机器人控制台 / {tab === 'control' ? '运动控制' : tab === 'status' ? '状态监控' : tab === 'config' ? '机械臂配置' : '在线编程'}</div><h1>{tab === 'control' ? '机械臂控制' : tab === 'status' ? '状态监控' : tab === 'config' ? '机械臂配置' : '在线编程'}</h1></div><div className="heading-meta"><span className="live-indicator"><i /> LIVE</span><span>刷新 100 Hz</span></div></div>
          {tab === 'control' && <>
            <section className="control-grid">
              <div className="panel motion-panel"><div className="panel-heading"><div><span className="panel-kicker">运动控制</span><h2>关节控制</h2></div><span className="mode-toggle"><button className="selected">关节</button><button>姿态</button></span></div>
                <div className="joint-inputs">{joints.map((joint, index) => <label className="joint-input" key={joint.name}><span>{joint.name}<em>ID {joint.id}</em></span><input type="number" value={joint.angle} step="0.001" onChange={(event) => setJointAngle(index, Number(event.target.value))} /><small>rad</small></label>)}</div>
                <div className="motion-settings"><label>速度<input type="number" value={speed} step="0.01" onChange={(event) => setSpeed(Number(event.target.value))} /><small>rad/s</small></label><label>加速度<input type="number" value={acceleration} step="0.1" onChange={(event) => setAcceleration(Number(event.target.value))} /><small>rad/s²</small></label><label className="check-label"><input type="checkbox" defaultChecked />阻塞</label></div>
                <button className="primary-action" disabled={!enabled || estop || !connected} onClick={runJointMotion}>执行关节运动 <span>→</span></button>
                <div className="action-row"><button className="outline-action" disabled={!enabled || estop || !connected} onClick={returnCalibrationPose}>回到标零姿态</button><button className="outline-action" disabled={estop} onClick={() => { setJoints(initialJoints); setNotice('已恢复当前反馈姿态') }}>复制当前角度</button></div>
              </div>
              <div className="panel feedback-panel"><div className="panel-heading"><div><span className="panel-kicker">实时反馈</span><h2>关节状态</h2></div><span className="tiny-live"><i /> 6 / 6</span></div><div className="feedback-list">{joints.map((joint) => <div className="feedback-row" key={joint.name}><div className="feedback-name"><strong>{joint.name}</strong><span className="motor-id">M{joint.id}</span></div><strong>{format(joint.angle)}</strong><span>{format(joint.velocity, 3)}</span><span>{format(joint.torque)}</span><span className="temp">{joint.temperature.toFixed(1)}°</span></div>)}</div><div className="feedback-header"><span>关节</span><span>角度 rad</span><span>速度</span><span>力矩 Nm</span><span>温度</span></div></div>
            </section>
            <section className="lower-grid"><div className="panel safety-panel"><div className="panel-heading"><div><span className="panel-kicker">安全控制</span><h2>动力与保护</h2></div><span className={`state-label ${estop ? 'danger' : enabled ? 'ok' : ''}`}>{estop ? '急停' : enabled ? '运行中' : '待机'}</span></div><div className="safety-actions"><button className={enabled ? 'safety-button enabled' : 'safety-button'} disabled={!connected || estop} onClick={toggleEnabled}>{enabled ? '掉使能机械臂' : '使能机械臂'}</button><button className={gravity ? 'safety-button gravity enabled' : 'safety-button gravity'} disabled={!enabled || estop} onClick={toggleGravity}>{gravity ? '关闭重力补偿' : '开启重力补偿'}</button></div><div className="notice-box"><span className="notice-icon">i</span><span>{apiError ? `${notice}（${apiError}）` : `${notice}${apiOnline ? '' : ' · 本地仿真 fallback'}`}</span></div></div><div className="panel pose-panel"><div className="panel-heading"><div><span className="panel-kicker">末端位姿</span><h2>tool0</h2></div><button className="mini-button">复制</button></div><div className="pose-values"><div><span>X</span><strong>0.107</strong><small>m</small></div><div><span>Y</span><strong>-0.460</strong><small>m</small></div><div><span>Z</span><strong>0.289</strong><small>m</small></div><div><span>RX</span><strong>1.571</strong><small>rad</small></div><div><span>RY</span><strong>0.284</strong><small>rad</small></div><div><span>RZ</span><strong>-1.204</strong><small>rad</small></div></div></div></section>
            <ArmViewport joints={joints} viserUrl={viserUrl} />
          </>}
          {tab === 'status' && <section className="status-page"><div className="metrics-grid"><div className="metric-card"><span>控制器状态</span><strong>{connected ? '在线' : '离线'}</strong><small>192.168.10.102</small></div><div className="metric-card"><span>动力状态</span><strong>{enabled ? '已使能' : '已掉使能'}</strong><small>{gravity ? '100% 重力补偿' : '无重力补偿'}</small></div><div className="metric-card"><span>平均温度</span><strong>{averageTemp.toFixed(1)}°C</strong><small>阈值 55°C</small></div><div className="metric-card"><span>最后动作</span><strong>{lastAction || '—'}</strong><small>{notice}</small></div></div><div className="panel table-panel"><div className="panel-heading"><div><span className="panel-kicker">设备监控</span><h2>六轴运行数据</h2></div><button className="mini-button">导出 CSV</button></div><div className="wide-table"><div className="wide-row header"><span>关节</span><span>电机 ID</span><span>位置 rad</span><span>速度 rad/s</span><span>力矩 Nm</span><span>温度</span><span>状态</span></div>{joints.map((joint) => <div className="wide-row" key={joint.name}><span><strong>{joint.name}</strong></span><span>{joint.id}</span><span>{format(joint.angle)}</span><span>{format(joint.velocity)}</span><span>{format(joint.torque)}</span><span>{joint.temperature.toFixed(1)}°C</span><span><i className="status-dot green" /> 正常</span></div>)}</div></div></section>}
          {tab === 'config' && <section className="config-page"><div className="panel config-panel"><div className="panel-heading"><div><span className="panel-kicker">机械臂配置</span><h2>软限位与工具坐标系</h2></div><button className="primary-small">保存配置</button></div><div className="config-section"><h3>关节软限位</h3><p>限制普通运动范围；桌面支撑标定位由回标定位流程单独管理。</p><div className="limit-table"><div className="limit-row header"><span>关节</span><span>最小 rad</span><span>最大 rad</span><span>当前值</span></div>{joints.map((joint) => <div className="limit-row" key={joint.name}><span><strong>{joint.name}</strong><small>M{joint.id}</small></span><input value={joint.min} readOnly /><input value={joint.max} readOnly /><span className="current-limit">{format(joint.angle)}</span></div>)}</div></div><div className="config-section tool-section"><h3>工具坐标系</h3><div className="tool-fields"><label>名称<input defaultValue="tool0" /></label><label>X<input defaultValue="0.000" /></label><label>Y<input defaultValue="0.000" /></label><label>Z<input defaultValue="0.000" /></label><label>重量 kg<input defaultValue="0.000" /></label></div></div></div></section>}
          {tab === 'programs' && <section className="program-page"><div className="panel program-panel"><div className="panel-heading"><div><span className="panel-kicker">在线编程</span><h2>动作流程</h2></div><button className="primary-small">新建流程</button></div><div className="program-layout"><div className="block-palette"><span>指令模块</span><button>＋ 运动 MOVEJ</button><button>＋ 等待</button><button>＋ 判断</button><button>＋ 计算</button><button>＋ 弹窗</button></div><div className="flow-canvas"><div className="flow-block start">开始</div><div className="flow-line" /><div className="flow-block motion"><span>运动</span><strong>MOVEJ</strong><small>速度 0.25 rad/s</small></div><div className="flow-line" /><div className="flow-block wait"><span>等待</span><strong>1.0 s</strong></div><div className="flow-line" /><div className="flow-block end">结束</div></div><div className="program-inspector"><span>选中模块</span><strong>MOVEJ</strong><label>J1<input defaultValue="0.0" /></label><label>J2<input defaultValue="1.748" /></label><label>J3<input defaultValue="0.155" /></label><button className="outline-action">保存模块</button></div></div></div></section>}
        </main>
      </div>
    </div>
  )
}

export default App
