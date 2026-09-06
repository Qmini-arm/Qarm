export type ApiJoint = {
  name: string
  joint_name: string
  id: number
  angle: number
  velocity: number
  torque: number
  temperature: number
  min: number
  max: number
  error?: number
}

export type ApiStatus = {
  connected: boolean
  enabled: boolean
  estop: boolean
  gravity: boolean
  mode: string
  lifecycle: string
  joints: ApiJoint[]
  dof: number
  joint_names: string[]
  calibrated: boolean
  calibration_pose_validated: boolean
  angle_space: 'urdf' | 'uncalibrated_motor_output'
  capabilities: {
    feedback: boolean
    enable: boolean
    gravity: boolean
    movej: boolean
    return_home: boolean
    physical_estop: boolean
  }
  last_action: string
  notice: string
  hardware_io_enabled?: boolean
  reader?: string | null
}

export type ApiOptions = { baseUrl?: string; timeoutMs?: number }

export type ProgramNode =
  | { type: 'start' }
  | { type: 'end' }
  | { type: 'movej'; joints: number[] }
  | { type: 'wait'; duration_s: number }
export type Program = {
  version: 1
  joint_names: string[]
  nodes: ProgramNode[]
}

export class ApiError extends Error {
  constructor(
    message: string,
    public status?: ApiStatus,
  ) {
    super(message)
  }
}

const baseUrl = (import.meta.env.VITE_API_URL || '').replace(/\/$/, '')
const timeoutMs = Number(import.meta.env.VITE_API_TIMEOUT_MS || 2500)

async function request<T>(
  path: string,
  init?: RequestInit,
  options: ApiOptions = {},
): Promise<T> {
  const controller = new AbortController()
  const timer = window.setTimeout(
    () => controller.abort(),
    options.timeoutMs ?? timeoutMs,
  )
  try {
    const response = await fetch(`${options.baseUrl ?? baseUrl}${path}`, {
      ...init,
      headers: { 'Content-Type': 'application/json', ...(init?.headers || {}) },
      signal: controller.signal,
    })
    const payload = (await response.json().catch(() => ({}))) as T & {
      error?: string
      status?: ApiStatus
      errors?: string[]
    }
    if (!response.ok)
      throw new ApiError(
        payload.error ||
          payload.errors?.join('; ') ||
          `后端请求失败 (${response.status})`,
        payload.status,
      )
    return payload
  } finally {
    window.clearTimeout(timer)
  }
}

export async function getStatus(options?: ApiOptions) {
  const status = await request<ApiStatus>('/api/status', undefined, options)
  if (
    !status.capabilities ||
    !Array.isArray(status.joint_names) ||
    status.dof !== status.joints?.length ||
    status.dof !== status.joint_names.length ||
    status.joints.some(
      (joint, index) =>
        joint.joint_name !== status.joint_names[index] ||
        !Number.isFinite(joint.min) ||
        !Number.isFinite(joint.max),
    )
  ) {
    throw new ApiError('控制服务模型不兼容，请更新控制服务')
  }
  return status
}

export function connect(options?: ApiOptions) {
  return request<ApiStatus>(
    '/api/connect',
    { method: 'POST', body: '{}' },
    options,
  )
}

export function disconnect(options?: ApiOptions) {
  return request<ApiStatus>(
    '/api/disconnect',
    { method: 'POST', body: '{}' },
    options,
  )
}

export function setEnabled(enabled: boolean, options?: ApiOptions) {
  return request<ApiStatus>(
    '/api/enable',
    { method: 'POST', body: JSON.stringify({ enabled }) },
    options,
  )
}

export function setEstop(options?: ApiOptions) {
  return request<ApiStatus>(
    '/api/estop',
    { method: 'POST', body: '{}' },
    options,
  )
}

export function clearEstop(options?: ApiOptions) {
  return request<ApiStatus>(
    '/api/clear-estop',
    { method: 'POST', body: '{}' },
    options,
  )
}

export function setGravity(enabled: boolean, options?: ApiOptions) {
  return request<ApiStatus>(
    '/api/gravity',
    { method: 'POST', body: JSON.stringify({ enabled }) },
    options,
  )
}

export function movej(
  joints: number[],
  speed: number,
  acceleration: number,
  options?: ApiOptions,
) {
  return request<ApiStatus>(
    '/api/movej',
    {
      method: 'POST',
      body: JSON.stringify({ joints, speed, acceleration }),
    },
    options,
  )
}

export function returnHome(trajectoryPath: string, options?: ApiOptions) {
  return request<ApiStatus>(
    '/api/return-home',
    {
      method: 'POST',
      body: JSON.stringify({
        trajectory_path: trajectoryPath,
        confirm_collision_checked_plan: true,
      }),
    },
    options,
  )
}

export function validateProgram(program: Program) {
  return request<{ valid: boolean; errors: string[] }>(
    '/api/program/validate',
    {
      method: 'POST',
      body: JSON.stringify(program),
    },
  )
}
