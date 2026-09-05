export type ApiJoint = {
  name?: string
  id: number
  angle: number
  velocity: number
  torque: number
  temperature: number
  min?: number
  max?: number
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
  last_action: string
  notice: string
  hardware_io_enabled?: boolean
  reader?: string | null
}

export type ApiOptions = { baseUrl?: string; timeoutMs?: number }

const baseUrl = (import.meta.env.VITE_API_URL || '').replace(/\/$/, '')
const timeoutMs = Number(import.meta.env.VITE_API_TIMEOUT_MS || 2500)

async function request<T>(path: string, init?: RequestInit, options: ApiOptions = {}): Promise<T> {
  const controller = new AbortController()
  const timer = window.setTimeout(() => controller.abort(), options.timeoutMs ?? timeoutMs)
  try {
    const response = await fetch(`${options.baseUrl ?? baseUrl}${path}`, {
      ...init,
      headers: { 'Content-Type': 'application/json', ...(init?.headers || {}) },
      signal: controller.signal,
    })
    const payload = await response.json().catch(() => ({})) as T & { error?: string }
    if (!response.ok) throw new Error(payload.error || `后端请求失败 (${response.status})`)
    return payload
  } finally {
    window.clearTimeout(timer)
  }
}

export function getStatus(options?: ApiOptions) {
  return request<ApiStatus>('/api/status', undefined, options)
}

export function connect(options?: ApiOptions) {
  return request<ApiStatus>('/api/connect', { method: 'POST', body: '{}' }, options)
}

export function disconnect(options?: ApiOptions) {
  return request<ApiStatus>('/api/disconnect', { method: 'POST', body: '{}' }, options)
}

export function setEnabled(enabled: boolean, options?: ApiOptions) {
  return request<ApiStatus>('/api/enable', { method: 'POST', body: JSON.stringify({ enabled }) }, options)
}

export function setEstop(options?: ApiOptions) {
  return request<ApiStatus>('/api/estop', { method: 'POST', body: '{}' }, options)
}

export function clearEstop(options?: ApiOptions) {
  return request<ApiStatus>('/api/clear-estop', { method: 'POST', body: '{}' }, options)
}

export function setGravity(enabled: boolean, options?: ApiOptions) {
  return request<ApiStatus>('/api/gravity', { method: 'POST', body: JSON.stringify({ enabled }) }, options)
}

export function movej(joints: number[], speed: number, acceleration: number, options?: ApiOptions) {
  return request<ApiStatus>('/api/movej', {
    method: 'POST',
    body: JSON.stringify({ joints, speed, acceleration }),
  }, options)
}

export function returnHome(trajectoryPath: string, options?: ApiOptions) {
  return request<ApiStatus>('/api/return-home', {
    method: 'POST',
    body: JSON.stringify({ trajectory_path: trajectoryPath, confirm_collision_checked_plan: true }),
  }, options)
}
