// REST + WebSocket client for the HopeJR Desk backend.
// URLs are same-origin (Vite proxies /api and /ws to :8000).

async function jpost(path, body) {
  const r = await fetch(path, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: body ? JSON.stringify(body) : undefined,
  })
  if (!r.ok) throw new Error(`${path}: ${r.status}`)
  return r.json()
}
async function jget(path) {
  const r = await fetch(path)
  if (!r.ok) throw new Error(`${path}: ${r.status}`)
  return r.json()
}

export const api = {
  // control
  enable: () => jpost('/api/control/servo/enable'),
  disable: () => jpost('/api/control/servo/disable'),
  estop: () => jpost('/api/control/estop'),
  recover: () => jpost('/api/control/recover'),
  home: () => jpost('/api/control/home'),
  reset: () => jpost('/api/control/reset'),
  commandJoint: (name, position) => jpost('/api/control/joint', { name, position }),
  rebootServo: (name) => jpost(`/api/control/servo/${name}/reboot`),
  scan: () => jget('/api/control/scan'),
  // hand
  fingers: () => jget('/api/hand/fingers'),
  commandFinger: (index, aperture) => jpost('/api/hand/finger', { index, aperture }),
  commandHandMotor: (name, value) => jpost('/api/hand/motor', { name, value }),
  grip: (fingers, aperture) => jpost('/api/hand/grip', { fingers, aperture }),
  contactThreshold: (current_ma) => jpost('/api/hand/contact-threshold', { current_ma }),
  // hand tendon kinematics (motor value -> joint angle)
  handKinematics: () => jget('/api/hand/kinematics'),
  handKinReference: (pose) => jpost('/api/hand/kinematics/reference', { pose }),
  commandPose: (pose) => jpost('/api/hand/pose', { pose }),
  trackHand: (landmarks, handedness, w_apex) =>
    jpost('/api/hand/track', { landmarks, handedness, w_apex }),
  handKinOffsets: (offsets) => jpost('/api/hand/kinematics/offsets', { offsets }),
  handKinReset: () => jpost('/api/hand/kinematics/reset'),
  handKinMomentArm: (moment_arm) => jpost('/api/hand/kinematics/moment-arm', { moment_arm }),
  handKinSlack: (slack) => jpost('/api/hand/kinematics/slack', { slack }),
  handKinDeadzone: (pct) => jpost('/api/hand/kinematics/deadzone', { pct }),
  handKinMcpGain: (body) => jpost('/api/hand/kinematics/mcp-gain', body),
  handPinchCapture: (finger) => jpost('/api/hand/kinematics/pinch/capture', { finger }),
  handPinchClear: (finger) => jpost('/api/hand/kinematics/pinch/clear', { finger }),
  // experimental contact-sample log for building the four-finger MCP model
  contactSample: (finger) => jpost('/api/hand/contact/sample', { finger }),
  contactSamples: () => jget('/api/hand/contact/samples'),
  contactClear: () => jpost('/api/hand/contact/clear'),
  // calibration
  calibStatus: () => jget('/api/calibration/status'),
  calibStart: (home = false) => jpost('/api/calibration/start', { home }),
  calibReset: () => jpost('/api/calibration/reset'),
  calibStop: () => jpost('/api/calibration/stop'),
  calibSave: () => jpost('/api/calibration/save'),
  calibDirection: (name, drive_mode) => jpost('/api/calibration/direction', { name, drive_mode }),
  // teaching (hand-guiding)
  teachStatus: () => jget('/api/teaching/status'),
  teachStart: () => jpost('/api/teaching/start'),
  teachStop: () => jpost('/api/teaching/stop'),
  teachSave: (name) => jpost('/api/teaching/save', { name }),
  teachReplay: (name) => jpost('/api/teaching/replay', { name }),
  teachDelete: (name) => jpost('/api/teaching/delete', { name }),
  // config + logs
  getConfig: () => jget('/api/config'),
  updateConfig: (cfg) => jpost('/api/config', cfg),
  getLinks: () => jget('/api/links'),
  setLink: (name, mass, com) => jpost('/api/links', { name, mass, com }),
  resetLinks: () => jpost('/api/links/reset'),
  getLogs: (limit = 200) => jget(`/api/logs?limit=${limit}`),
}

// Auto-reconnecting telemetry websocket.
export function connectTelemetry(onSnapshot, onStatus) {
  let ws, closed = false, timer
  const url = `${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws/telemetry`
  function open() {
    ws = new WebSocket(url)
    ws.onopen = () => onStatus && onStatus('connected')
    ws.onmessage = (e) => { try { onSnapshot(JSON.parse(e.data)) } catch {} }
    ws.onclose = () => {
      onStatus && onStatus('disconnected')
      if (!closed) timer = setTimeout(open, 1000)
    }
    ws.onerror = () => ws.close()
  }
  open()
  return () => { closed = true; clearTimeout(timer); ws && ws.close() }
}
