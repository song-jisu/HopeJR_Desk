import React from 'react'
import { Card, Stat, Bar } from '../components.jsx'

export default function Dashboard({ snap }) {
  if (!snap) return <p className="page-sub">Waiting for telemetry…</p>
  const { status, motors, fingers } = snap
  const maxTemp = Math.max(0, ...motors.map(m => m.temperature))
  const maxCur = Math.max(0, ...motors.map(m => m.current))
  const arm = motors.filter(m => m.unit === 'arm')

  return (
    <>
      <div className="grid cols-4" style={{ marginBottom: 14 }}>
        <Card><Stat label="Robot" value={status.connected ? 'Connected' : 'Offline'} /></Card>
        <Card><Stat label="Servo" value={status.servo_enabled ? 'ON' : 'OFF'} /></Card>
        <Card><Stat label="E-Stop" value={status.estop ? 'ENGAGED' : 'Clear'} /></Card>
        <Card><Stat label="Current Task" value={status.current_task || '—'} /></Card>
      </div>

      <div className="grid cols-4" style={{ marginBottom: 14 }}>
        <Card><Stat label="Arm servos online" value={`${status.arm_online}/7`} /></Card>
        <Card><Stat label="Hand servos online" value={`${status.hand_online}/16`} /></Card>
        <Card><Stat label="Max temperature" value={maxTemp.toFixed(1)} unit="°C" /></Card>
        <Card><Stat label="Max current" value={maxCur.toFixed(0)} unit="mA" /></Card>
      </div>

      <div className="grid cols-2">
        <Card title="Arm — Joint Position / Velocity / Current">
          <table>
            <thead><tr><th>Joint</th><th>ID</th><th>Pos</th><th>Vel</th><th>Current</th><th>Temp</th></tr></thead>
            <tbody>
              {arm.map(m => (
                <tr key={m.name}>
                  <td>{m.name}</td>
                  <td className="num">{m.servo_id}</td>
                  <td className="num">{m.position.toFixed(1)}</td>
                  <td className="num">{m.velocity.toFixed(1)}</td>
                  <td className="num">{m.current.toFixed(0)} mA</td>
                  <td className="num" style={{ color: m.temperature > 60 ? 'var(--warn)' : undefined }}>
                    {m.temperature.toFixed(1)}°</td>
                </tr>
              ))}
            </tbody>
          </table>
        </Card>

        <Card title="Hand — Finger State">
          <div className="grid cols-3" style={{ gap: 10 }}>
            {fingers.map(f => (
              <div key={f.index} className="finger">
                <div className="name">{f.index}. {f.name}</div>
                <span className={`badge ${f.state}`}>{f.state}</span>
                <div style={{ width: '100%' }}>
                  <Bar value={f.aperture} kind={f.contact ? 'warn' : ''} />
                </div>
                <div className="help">{f.aperture.toFixed(0)}% · {f.max_current.toFixed(0)} mA</div>
              </div>
            ))}
          </div>
          <div className="help" style={{ marginTop: 12 }}>
            Comm delay {status.comm_delay_ms.toFixed(1)} ms · packet loss {(status.packet_loss * 100).toFixed(1)}% · backend “{status.mode}”
          </div>
        </Card>
      </div>
    </>
  )
}
