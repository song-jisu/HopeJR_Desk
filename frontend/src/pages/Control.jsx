import React, { useState } from 'react'
import { Card } from '../components.jsx'
import { api } from '../api.js'

export default function Control({ snap }) {
  const [busy, setBusy] = useState(false)
  const status = snap?.status
  const arm = (snap?.motors || []).filter(m => m.unit === 'arm')

  const call = (fn) => async () => { setBusy(true); try { await fn() } finally { setBusy(false) } }

  return (
    <>
      <Card title="Robot Control" style={{ marginBottom: 14 }}>
        <div className="btn-row">
          <button className="btn green" disabled={busy || status?.estop} onClick={call(api.enable)}>Servo Enable</button>
          <button className="btn" disabled={busy} onClick={call(api.disable)}>Servo Disable</button>
          <button className="btn primary" disabled={busy || status?.estop} onClick={call(api.home)}>Home Position</button>
          <button className="btn" disabled={busy || status?.estop} onClick={call(api.reset)}>Joint Reset</button>
          {status?.estop
            ? <button className="btn warn estop-big" disabled={busy} onClick={call(api.recover)}>Recover</button>
            : <button className="btn danger estop-big" disabled={busy} onClick={call(api.estop)}>■ EMERGENCY STOP</button>}
        </div>
        <div className="help" style={{ marginTop: 8 }}>
          <b>Home Position</b> drives each servo to the encoder midpoint (raw ~2048), computed from
          the current calibration and clamped to the safe range. On <b>Servo Enable</b> the arm holds
          its current pose (it does not jump to home).
        </div>
        <div className="help" style={{ marginTop: 6 }}>
          Servo: <b>{status?.servo_enabled ? 'ON' : 'OFF'}</b> · E-Stop: <b style={{ color: status?.estop ? 'var(--danger)' : undefined }}>{status?.estop ? 'ENGAGED' : 'Clear'}</b>
        </div>
      </Card>

      <Card title="Arm Joint Jog (normalized -100..100)">
        {arm.map(m => (
          <div className="slider-row" key={m.name}>
            <label>{m.name}</label>
            <input type="range" min={-100} max={100} step={1} value={Math.round(m.command)}
              disabled={!status?.servo_enabled || status?.estop}
              onChange={(e) => api.commandJoint(m.name, Number(e.target.value))} />
            <span className="val">{m.command.toFixed(0)}</span>
          </div>
        ))}
        {!status?.servo_enabled && <div className="help">Enable servo to jog joints.</div>}
      </Card>
    </>
  )
}
