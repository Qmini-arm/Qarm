import { useEffect, useRef, useState } from 'react'
import * as api from './api'

type Tab = 'control' | 'status' | 'config' | 'programs'
const tabs: { id: Tab; label: string; symbol: string }[] = [
  { id: 'control', label: '机械臂控制', symbol: '⌁' },
  { id: 'status', label: '状态监控', symbol: '◌' },
  { id: 'config', label: '机械臂配置', symbol: '⚙' },
  { id: 'programs', label: '在线编程', symbol: '▦' },
]
const format = (value: number, digits = 3) =>
  Number.isFinite(value) ? value.toFixed(digits) : '--'

function download(name: string, data: string, type: string) {
  const url = URL.createObjectURL(new Blob([data], { type }))
  const link = document.createElement('a')
  link.href = url
  link.download = name
  link.click()
  URL.revokeObjectURL(url)
}

function ArmViewport({
  joints,
  calibrated,
}: {
  joints: api.ApiJoint[]
  calibrated: boolean
}) {
  const [useViser, setUseViser] = useState(false)
  const viserUrl = import.meta.env.VITE_VISER_URL || 'http://127.0.0.1:8080'
  return (
    <div className="viewport">
      <div className="viewport-toolbar">
        <span>{useViser ? 'Viser 模型视图' : '关节角度'}</span>
        <span className="view-chip">{calibrated ? 'URDF' : 'MOTOR'}</span>
        <button className="viser-toggle" onClick={() => setUseViser(!useViser)}>
          {useViser ? '关节角度' : '打开 Viser'}
        </button>
      </div>
      {useViser ? (
        <div className="viser-frame">
          <iframe title="Qarm Viser 3D 视图" src={viserUrl} />
        </div>
      ) : (
        <div className="joint-gauges">
          {joints.map((joint) => (
            <div className="joint-gauge" key={joint.id}>
              <svg
                viewBox="0 0 140 120"
                role="img"
                aria-label={`${joint.name} ${format(joint.angle)} rad`}
              >
                <circle
                  cx="70"
                  cy="62"
                  r="44"
                  fill="none"
                  stroke="#d5e3e8"
                  strokeWidth="9"
                />
                <line
                  x1="70"
                  y1="62"
                  x2={70 + 39 * Math.sin(joint.angle)}
                  y2={62 - 39 * Math.cos(joint.angle)}
                  stroke="#178f87"
                  strokeWidth="5"
                  strokeLinecap="round"
                />
                <circle cx="70" cy="62" r="7" fill="#173344" />
                <text
                  x="70"
                  y="20"
                  textAnchor="middle"
                  fill="#5c7687"
                  fontSize="10"
                >
                  0
                </text>
              </svg>
              <strong>
                {joint.name} <small>M{joint.id}</small>
              </strong>
              <span>{format(joint.angle)} rad</span>
            </div>
          ))}
        </div>
      )}
      <div className="viewport-footer">
        <span>{joints.length} DOF</span>
        <span>{calibrated ? '关节空间' : '电机输出角 · 未标定'}</span>
        <span>tool0</span>
      </div>
    </div>
  )
}

function App() {
  const [tab, setTab] = useState<Tab>('control')
  const [status, setStatus] = useState<api.ApiStatus | null>(null)
  const [online, setOnline] = useState(false)
  const [busy, setBusy] = useState(false)
  const [error, setError] = useState('')
  const [serviceError, setServiceError] = useState('')
  const [notice, setNotice] = useState('正在连接控制服务')
  const [targets, setTargets] = useState<number[]>([])
  const [speed, setSpeed] = useState(0.25)
  const [acceleration, setAcceleration] = useState(0.5)
  const [trajectory, setTrajectory] = useState(
    import.meta.env.VITE_HOME_TRAJECTORY || 'build/calibration_home.csv',
  )
  const [planConfirmed, setPlanConfirmed] = useState(false)
  const [program, setProgram] = useState<api.Program | null>(null)
  const [selectedNode, setSelectedNode] = useState(1)
  const [programNotice, setProgramNotice] = useState('')
  const initializedModel = useRef('')
  const fileInput = useRef<HTMLInputElement>(null)
  const joints = status?.joints ?? []
  const connected = Boolean(online && status?.connected)
  const enabled = Boolean(status?.enabled)
  const estop = Boolean(status?.estop)
  const ready = connected && enabled && !estop && !busy
  const averageTemp = joints.length
    ? joints.reduce((sum, joint) => sum + joint.temperature, 0) / joints.length
    : 0

  function applyStatus(next: api.ApiStatus) {
    setStatus(next)
    setOnline(true)
    setServiceError('')
    setNotice(next.notice)
    const identity = JSON.stringify(next.joint_names)
    if (identity !== initializedModel.current) {
      initializedModel.current = identity
      setTargets(next.joints.map((joint) => joint.angle))
      setProgram({
        version: 1,
        joint_names: next.joint_names,
        nodes: [
          { type: 'start' },
          { type: 'movej', joints: next.joints.map((joint) => joint.angle) },
          { type: 'end' },
        ],
      })
      setSelectedNode(1)
    }
  }

  useEffect(() => {
    let disposed = false
    let pending = false
    const sync = async () => {
      if (pending) return
      pending = true
      try {
        const next = await api.getStatus()
        if (!disposed) applyStatus(next)
      } catch (cause) {
        if (!disposed) {
          setOnline(false)
          setServiceError(
            cause instanceof Error ? cause.message : '控制服务不可达',
          )
        }
      } finally {
        pending = false
      }
    }
    void sync()
    const timer = window.setInterval(() => void sync(), 1000)
    return () => {
      disposed = true
      window.clearInterval(timer)
    }
  }, [])

  async function command(action: () => Promise<api.ApiStatus>) {
    setBusy(true)
    setError('')
    try {
      applyStatus(await action())
    } catch (cause) {
      if (cause instanceof api.ApiError && cause.status)
        applyStatus(cause.status)
      setError(cause instanceof Error ? cause.message : '请求失败')
    } finally {
      setBusy(false)
    }
  }

  function editTarget(index: number, value: number) {
    setTargets((current) =>
      current.map((target, item) => (item === index ? value : target)),
    )
  }

  const targetValid =
    targets.length === joints.length &&
    targets.every(
      (value, index) =>
        Number.isFinite(value) &&
        value >= joints[index].min &&
        value <= joints[index].max,
    ) &&
    speed > 0 &&
    acceleration > 0

  function newProgram() {
    setProgram({
      version: 1,
      joint_names: status?.joint_names ?? [],
      nodes: [{ type: 'start' }, { type: 'end' }],
    })
    setSelectedNode(0)
    setProgramNotice('')
  }

  function addNode(type: 'movej' | 'wait') {
    if (!program) return
    const node: api.ProgramNode =
      type === 'movej'
        ? { type, joints: [...targets] }
        : { type, duration_s: 1 }
    setProgram({
      ...program,
      nodes: [...program.nodes.slice(0, -1), node, { type: 'end' }],
    })
    setSelectedNode(program.nodes.length - 1)
    setProgramNotice('')
  }

  function updateNode(node: api.ProgramNode) {
    if (!program) return
    setProgram({
      ...program,
      nodes: program.nodes.map((current, index) =>
        index === selectedNode ? node : current,
      ),
    })
    setProgramNotice('')
  }

  async function validate(exportFile = false) {
    if (!program) return
    try {
      await api.validateProgram(program)
      setProgramNotice(
        `校验通过 · ${program.joint_names.length} 轴 · ${program.nodes.length} 个节点`,
      )
      if (exportFile)
        download(
          'qarm-program.json',
          JSON.stringify(program, null, 2),
          'application/json',
        )
    } catch (cause) {
      setProgramNotice(cause instanceof Error ? cause.message : '校验失败')
    }
  }

  async function importProgram(file: File) {
    try {
      const imported = JSON.parse(await file.text()) as api.Program
      await api.validateProgram(imported)
      setProgram(imported)
      setSelectedNode(0)
      setProgramNotice('流程已导入，校验通过')
    } catch (cause) {
      setProgramNotice(cause instanceof Error ? cause.message : '导入失败')
    }
  }

  const node = program?.nodes[selectedNode]
  return (
    <div className="app-shell">
      <header className="topbar">
        <div className="brand">
          <div className="brand-mark">Q</div>
          <div>
            <strong>Qarm Control</strong>
            <span>
              {status ? `${status.dof} 自由度机械臂控制平台` : '机械臂控制平台'}
            </span>
          </div>
        </div>
        <div className="topbar-actions">
          <div className="status-pill">
            <span className={`status-dot ${connected ? 'green' : 'gray'}`} />
            <span>{connected ? '已连接' : '未连接'}</span>
            <span className="status-divider" />
            <span>{enabled ? '已使能' : '未使能'}</span>
          </div>
          <button
            className="ghost-button"
            disabled={!online || busy}
            onClick={() =>
              void command(() =>
                status?.connected ? api.disconnect() : api.connect(),
              )
            }
          >
            {status?.connected ? '断开连接' : '重新连接'}
          </button>
          <button
            className="estop-button"
            disabled={!online || busy}
            onClick={() => void command(estop ? api.clearEstop : api.setEstop)}
          >
            {estop ? '解除急停' : '急停'}
          </button>
        </div>
      </header>
      <div className="layout">
        <aside className="sidebar">
          <div className="side-caption">控制台</div>
          {tabs.map((item) => (
            <button
              key={item.id}
              className={`side-item ${tab === item.id ? 'active' : ''}`}
              onClick={() => setTab(item.id)}
            >
              <span>{item.symbol}</span>
              {item.label}
            </button>
          ))}
          <div className="sidebar-spacer" />
          <div className="connection-card">
            <span className={`status-dot ${online ? 'green' : 'gray'}`} />
            <div>
              <strong>
                {online
                  ? status?.mode === 'simulation'
                    ? '仿真服务在线'
                    : '硬件服务在线'
                  : '控制服务离线'}
              </strong>
              <small>
                {status?.mode === 'hardware' ? 'M8010 · BRAKE' : 'SIMULATION'}
              </small>
            </div>
          </div>
        </aside>
        <main className="main-content">
          <div className="page-heading">
            <div>
              <div className="eyebrow">
                QARM / {status?.mode === 'hardware' ? 'HARDWARE' : 'SIMULATION'}
              </div>
              <h1>{tabs.find((item) => item.id === tab)?.label}</h1>
            </div>
            <div className="heading-meta">
              <span className="live-indicator">
                <i />
                {online ? 'ONLINE' : 'OFFLINE'}
              </span>
              <span>{status?.dof ?? '--'} DOF</span>
            </div>
          </div>
          <div
            className={`service-notice ${error || serviceError ? 'error' : ''}`}
            role="status"
          >
            {serviceError || error || notice}
          </div>
          {tab === 'control' && (
            <>
              <section className="control-grid">
                <div className="panel motion-panel">
                  <div className="panel-heading">
                    <div>
                      <span className="panel-kicker">运动控制</span>
                      <h2>关节目标</h2>
                    </div>
                    <span className="state-label">rad</span>
                  </div>
                  <div className="joint-inputs">
                    {joints.map((joint, index) => (
                      <label className="joint-input" key={joint.id}>
                        <span>
                          {joint.name}
                          <em>ID {joint.id}</em>
                        </span>
                        <input
                          aria-label={`${joint.name} 目标角度`}
                          type="number"
                          value={
                            Number.isFinite(targets[index])
                              ? targets[index]
                              : ''
                          }
                          min={joint.min}
                          max={joint.max}
                          step="0.001"
                          onChange={(event) =>
                            editTarget(index, event.target.valueAsNumber)
                          }
                        />
                        <small>rad</small>
                      </label>
                    ))}
                  </div>
                  <div className="motion-settings">
                    <label>
                      速度
                      <input
                        type="number"
                        value={Number.isFinite(speed) ? speed : ''}
                        min="0.01"
                        step="0.01"
                        onChange={(event) =>
                          setSpeed(event.target.valueAsNumber)
                        }
                      />
                      <small>rad/s</small>
                    </label>
                    <label>
                      加速度
                      <input
                        type="number"
                        value={
                          Number.isFinite(acceleration) ? acceleration : ''
                        }
                        min="0.01"
                        step="0.1"
                        onChange={(event) =>
                          setAcceleration(event.target.valueAsNumber)
                        }
                      />
                      <small>rad/s²</small>
                    </label>
                  </div>
                  <button
                    className="primary-action"
                    disabled={
                      !ready || !status?.capabilities.movej || !targetValid
                    }
                    onClick={() =>
                      void command(() =>
                        api.movej(targets, speed, acceleration),
                      )
                    }
                  >
                    执行关节运动 <span>→</span>
                  </button>
                  <div className="action-row">
                    <button
                      className="outline-action"
                      disabled={!online || !joints.length}
                      onClick={() =>
                        setTargets(joints.map((joint) => joint.angle))
                      }
                    >
                      复制当前角度
                    </button>
                  </div>
                </div>
                <div className="panel feedback-panel">
                  <div className="panel-heading">
                    <div>
                      <span className="panel-kicker">实时反馈</span>
                      <h2>关节状态</h2>
                    </div>
                    <span className="tiny-live">
                      <i />
                      {connected ? joints.length : 0} / {status?.dof ?? '--'}
                    </span>
                  </div>
                  <div className="feedback-header">
                    <span>关节</span>
                    <span>角度 rad</span>
                    <span>速度</span>
                    <span>力矩 Nm</span>
                    <span>温度</span>
                  </div>
                  <div className="feedback-list">
                    {joints.map((joint) => (
                      <div className="feedback-row" key={joint.id}>
                        <div className="feedback-name">
                          <strong>{joint.name}</strong>
                          <span className="motor-id">M{joint.id}</span>
                        </div>
                        <strong>{format(joint.angle)}</strong>
                        <span>{format(joint.velocity)}</span>
                        <span>{format(joint.torque)}</span>
                        <span className="temp">
                          {format(joint.temperature, 1)}°
                        </span>
                      </div>
                    ))}
                  </div>
                </div>
              </section>
              <section className="lower-grid">
                <div className="panel safety-panel">
                  <div className="panel-heading">
                    <div>
                      <span className="panel-kicker">安全控制</span>
                      <h2>动力与保护</h2>
                    </div>
                    <span
                      className={`state-label ${estop ? 'danger' : enabled ? 'ok' : ''}`}
                    >
                      {estop ? '急停' : enabled ? '运行中' : '待机'}
                    </span>
                  </div>
                  <div className="safety-actions">
                    <button
                      className={`safety-button ${enabled ? 'enabled' : ''}`}
                      disabled={
                        !connected ||
                        estop ||
                        busy ||
                        (!enabled && !status?.capabilities.enable)
                      }
                      onClick={() =>
                        void command(() => api.setEnabled(!enabled))
                      }
                    >
                      {enabled ? '掉使能机械臂' : '使能机械臂'}
                    </button>
                    <button
                      className={`safety-button gravity ${status?.gravity ? 'enabled' : ''}`}
                      disabled={!ready || !status?.capabilities.gravity}
                      onClick={() =>
                        void command(() => api.setGravity(!status?.gravity))
                      }
                    >
                      {status?.gravity ? '关闭重力补偿' : '开启重力补偿'}
                    </button>
                  </div>
                  <div className="notice-box">
                    {status?.mode === 'hardware'
                      ? '硬件反馈模式 · 运动适配器未接入'
                      : '仿真控制'}
                  </div>
                </div>
                <div className="panel pose-panel">
                  <div className="panel-heading">
                    <div>
                      <span className="panel-kicker">当前模型</span>
                      <h2>{status?.dof ?? '--'} 自由度 · tool0</h2>
                    </div>
                    <span
                      className={`state-label ${status?.calibrated ? 'ok' : ''}`}
                    >
                      {status?.calibrated ? '已标定' : '待重新标定'}
                    </span>
                  </div>
                  <div className="model-values">
                    <span>关节链</span>
                    <strong>{status?.joint_names.join(' / ') || '--'}</strong>
                    <span>角度坐标</span>
                    <strong>
                      {status?.angle_space === 'uncalibrated_motor_output'
                        ? '未标定电机输出角'
                        : 'URDF 关节角'}
                    </strong>
                    <span>回标定位</span>
                    <strong>
                      {status?.calibration_pose_validated
                        ? '参考姿态有效'
                        : '参考姿态未验证'}
                    </strong>
                  </div>
                </div>
              </section>
              <ArmViewport
                joints={joints}
                calibrated={status?.angle_space !== 'uncalibrated_motor_output'}
              />
            </>
          )}
          {tab === 'status' && (
            <section className="status-page">
              <div className="metrics-grid">
                <div className="metric-card">
                  <span>控制器状态</span>
                  <strong>{connected ? '在线' : '离线'}</strong>
                  <small>{status?.lifecycle || '--'}</small>
                </div>
                <div className="metric-card">
                  <span>动力状态</span>
                  <strong>{enabled ? '已使能' : '未使能'}</strong>
                  <small>
                    {status?.gravity ? '重力补偿开启' : '重力补偿关闭'}
                  </small>
                </div>
                <div className="metric-card">
                  <span>平均温度</span>
                  <strong>
                    {joints.length ? format(averageTemp, 1) : '--'}°C
                  </strong>
                  <small>{joints.length} 个电机</small>
                </div>
                <div className="metric-card">
                  <span>最后动作</span>
                  <strong>{status?.last_action || '--'}</strong>
                  <small>{status?.notice}</small>
                </div>
              </div>
              <div className="panel table-panel">
                <div className="panel-heading">
                  <div>
                    <span className="panel-kicker">设备监控</span>
                    <h2>{status?.dof ?? '--'} 轴运行数据</h2>
                  </div>
                  <button
                    className="mini-button"
                    disabled={!joints.length}
                    onClick={() =>
                      download(
                        'qarm-joints.csv',
                        [
                          'joint,motor_id,position_rad,velocity_rad_s,torque_nm,temperature_c,error',
                          ...joints.map((joint) =>
                            [
                              joint.joint_name,
                              joint.id,
                              joint.angle,
                              joint.velocity,
                              joint.torque,
                              joint.temperature,
                              joint.error ?? 0,
                            ].join(','),
                          ),
                        ].join('\n'),
                        'text/csv',
                      )
                    }
                  >
                    导出 CSV
                  </button>
                </div>
                <div className="wide-table">
                  <div className="wide-row header">
                    <span>关节</span>
                    <span>电机 ID</span>
                    <span>位置 rad</span>
                    <span>速度 rad/s</span>
                    <span>力矩 Nm</span>
                    <span>温度</span>
                    <span>状态</span>
                  </div>
                  {joints.map((joint) => (
                    <div className="wide-row" key={joint.id}>
                      <strong>{joint.name}</strong>
                      <span>{joint.id}</span>
                      <span>{format(joint.angle)}</span>
                      <span>{format(joint.velocity)}</span>
                      <span>{format(joint.torque)}</span>
                      <span>{format(joint.temperature, 1)}°C</span>
                      <span>
                        {!connected
                          ? '离线'
                          : joint.error
                            ? `错误 ${joint.error}`
                            : '正常'}
                      </span>
                    </div>
                  ))}
                </div>
              </div>
            </section>
          )}
          {tab === 'config' && (
            <section className="config-page">
              <div className="panel config-panel">
                <div className="panel-heading">
                  <div>
                    <span className="panel-kicker">机械臂配置</span>
                    <h2>模型限位与标定</h2>
                  </div>
                  <span className="state-label">URDF</span>
                </div>
                <div className="config-section">
                  <h3>关节软限位</h3>
                  <div className="limit-table">
                    <div className="limit-row header">
                      <span>关节</span>
                      <span>最小 rad</span>
                      <span>最大 rad</span>
                      <span>当前 rad</span>
                    </div>
                    {joints.map((joint) => (
                      <div className="limit-row" key={joint.id}>
                        <span>
                          <strong>{joint.name}</strong>
                          <small>M{joint.id}</small>
                        </span>
                        <span>{format(joint.min)}</span>
                        <span>{format(joint.max)}</span>
                        <span className="current-limit">
                          {format(joint.angle)}
                        </span>
                      </div>
                    ))}
                  </div>
                </div>
                <div className="config-section">
                  <h3>回到标定姿态</h3>
                  <div className="notice-box">
                    {status?.calibration_pose_validated
                      ? '参考姿态已验证'
                      : '当前结构的参考姿态尚未验证'}
                  </div>
                  <label className="trajectory-field">
                    轨迹路径
                    <input
                      value={trajectory}
                      onChange={(event) => setTrajectory(event.target.value)}
                    />
                  </label>
                  <label className="plan-confirmation">
                    <input
                      type="checkbox"
                      checked={planConfirmed}
                      onChange={(event) =>
                        setPlanConfirmed(event.target.checked)
                      }
                    />
                    已检查轨迹碰撞与当前电机供电周期
                  </label>
                  <button
                    className="outline-action"
                    disabled={
                      !ready ||
                      !status?.capabilities.return_home ||
                      !planConfirmed
                    }
                    onClick={() =>
                      void command(() => api.returnHome(trajectory))
                    }
                  >
                    回到标定姿态
                  </button>
                </div>
              </div>
            </section>
          )}
          {tab === 'programs' && (
            <section className="program-page">
              <div className="panel program-panel">
                <div className="panel-heading">
                  <div>
                    <span className="panel-kicker">在线编程</span>
                    <h2>动作流程</h2>
                  </div>
                  <div className="program-actions">
                    <button
                      className="mini-button"
                      disabled={!online}
                      onClick={() => fileInput.current?.click()}
                    >
                      导入
                    </button>
                    <button
                      className="mini-button"
                      disabled={!online || !program}
                      onClick={() => void validate(true)}
                    >
                      导出
                    </button>
                    <button
                      className="primary-small"
                      disabled={!status}
                      onClick={newProgram}
                    >
                      新建流程
                    </button>
                    <input
                      ref={fileInput}
                      type="file"
                      accept="application/json,.json"
                      hidden
                      onChange={(event) => {
                        const file = event.target.files?.[0]
                        if (file) void importProgram(file)
                        event.target.value = ''
                      }}
                    />
                  </div>
                </div>
                <div className="program-layout">
                  <div className="block-palette">
                    <span>指令模块</span>
                    <button
                      disabled={!program}
                      onClick={() => addNode('movej')}
                    >
                      ＋ MOVEJ
                    </button>
                    <button disabled={!program} onClick={() => addNode('wait')}>
                      ＋ 等待
                    </button>
                    <button
                      disabled={!online || !program}
                      onClick={() => void validate()}
                    >
                      校验流程
                    </button>
                    <div className="program-result" role="status">
                      {programNotice}
                    </div>
                  </div>
                  <div className="flow-canvas">
                    {program?.nodes.map((item, index) => (
                      <div key={index} className="flow-node">
                        <button
                          onClick={() => setSelectedNode(index)}
                          className={`flow-block ${item.type === 'movej' ? 'motion' : item.type} ${index === selectedNode ? 'selected' : ''}`}
                        >
                          <strong>
                            {item.type === 'start'
                              ? '开始'
                              : item.type === 'end'
                                ? '结束'
                                : item.type === 'movej'
                                  ? 'MOVEJ'
                                  : `${format(item.duration_s, 1)} s`}
                          </strong>
                          {item.type === 'movej' && (
                            <small>{item.joints.length} 轴 · rad</small>
                          )}
                        </button>
                        {index < program.nodes.length - 1 && (
                          <div className="flow-line" />
                        )}
                      </div>
                    ))}
                  </div>
                  <div className="program-inspector">
                    <span>选中模块</span>
                    <strong>{node?.type.toUpperCase() || '--'}</strong>
                    {node?.type === 'movej' &&
                      joints.map((joint, index) => (
                        <label key={joint.id}>
                          {joint.name}
                          <input
                            aria-label={`${joint.name} 模块角度`}
                            type="number"
                            min={joint.min}
                            max={joint.max}
                            step="0.01"
                            value={
                              Number.isFinite(node.joints[index])
                                ? node.joints[index]
                                : ''
                            }
                            onChange={(event) =>
                              updateNode({
                                ...node,
                                joints: node.joints.map((value, i) =>
                                  i === index
                                    ? event.target.valueAsNumber
                                    : value,
                                ),
                              })
                            }
                          />
                        </label>
                      ))}
                    {node?.type === 'wait' && (
                      <label>
                        等待秒数
                        <input
                          type="number"
                          min="0"
                          max="3600"
                          step="0.1"
                          value={
                            Number.isFinite(node.duration_s)
                              ? node.duration_s
                              : ''
                          }
                          onChange={(event) =>
                            updateNode({
                              ...node,
                              duration_s: event.target.valueAsNumber,
                            })
                          }
                        />
                      </label>
                    )}
                    {program &&
                      node &&
                      node.type !== 'start' &&
                      node.type !== 'end' && (
                        <button
                          className="outline-action"
                          onClick={() => {
                            setProgram({
                              ...program,
                              nodes: program.nodes.filter(
                                (_, index) => index !== selectedNode,
                              ),
                            })
                            setSelectedNode(0)
                            setProgramNotice('')
                          }}
                        >
                          删除模块
                        </button>
                      )}
                  </div>
                </div>
              </div>
            </section>
          )}
        </main>
      </div>
    </div>
  )
}

export default App
