import React, { useEffect, useRef, useState } from 'react'
import { Card } from '../components.jsx'
import { api } from '../api.js'
import { HandSetup } from './HandSetup.jsx'

// Web version of lerobot's RangeFinderGUI: torque off, backdrive each joint
// through its full range, record raw min/max, then save to the calibration JSON.
export default function Calibration({ snap }) {
  const [st, setSt] = useState(null)
  const [msg, setMsg] = useState('')
  const [unit, setUnit] = useState('arm')
  const [home, setHome] = useState(false)
  const timer = useRef()

  const poll = () => api.calibStatus().then(setSt).catch(() => {})
  useEffect(() => {
    poll()
    timer.current = setInterval(poll, 400)
    return () => clearInterval(timer.current)
  }, [])

  const note = (m) => { setMsg(m); setTimeout(() => setMsg(''), 2500) }
  const start = async () => { await api.calibStart(home); note('Calibration started — torque OFF. Move each joint through its full range by hand.') }
  const reset = async () => { await api.calibReset(); note('Recorded ranges cleared — sweep again.') }
  const stop = async () => { await api.calibStop(); note('Stopped.') }
  const save = async () => { const r = await api.calibSave(); note(r.message || 'saved') }
  const flip = async (m) => { await api.calibDirection(m.name, m.drive_mode ? 0 : 1); note(`${m.name} direction flipped`) }

  if (!st) return <p className="page-sub">Loading…</p>
  if (!st.supported) {
    return <Card title="Calibration">
      <div className="help">Calibration is only available with the <b>serial</b> backend (real robot). Current backend does not support it.</div>
    </Card>
  }

  const motors = (st.motors || []).filter(m => m.unit === unit)
  const active = st.active

  return (
    <>
      <Card title="Range-Finder Calibration" style={{ marginBottom: 14 }}>
        <ol className="help" style={{ marginBottom: 10, paddingLeft: 18, lineHeight: 1.7 }}>
          <li>Click <b>Start</b> — torque turns OFF. <b>Rec min/max start empty</b> and fill from your
              hand-movement; on Save they <b>overwrite</b> the old range (not intersect it).</li>
          <li>Move every joint <b>by hand</b> through its full range (both extremes). Never drag the
              sliders now — they still use the old (wrong) range and could damage the robot.</li>
          <li>If a joint's <b>Span jumps near 4096</b> (a comm glitch, or it crossed the encoder wrap),
              click <b>Reset ranges</b> and sweep again. If it genuinely wraps, re-Start with the
              "Center pose" option below.</li>
          <li>Click <b>Save</b> (writes the lerobot JSON + reloads live; keeps a <code>.bak</code>),
              then <b>Stop</b>.</li>
        </ol>
        <div className="btn-row">
          {!active
            ? <button className="btn green" onClick={start}>Start Calibration (torque off)</button>
            : <><button className="btn warn" onClick={stop}>Stop</button>
              <button className="btn" onClick={reset}>Reset ranges</button></>}
          <button className="btn primary" onClick={save} disabled={!active && !st.motors.some(m => m.rec_span > 0)}>Save to file</button>
          <span style={{ flex: 1 }} />
          {['arm', 'hand'].map(u => (
            <button key={u} className={`btn ${unit === u ? 'primary' : ''}`} onClick={() => setUnit(u)}>{u}</button>
          ))}
        </div>
        {!active && (
          <label className="help" style={{ display: 'flex', alignItems: 'center', gap: 6, marginTop: 8 }}>
            <input type="checkbox" checked={home} onChange={e => setHome(e.target.checked)} />
            Center pose first (half-turn homing) — only if a joint's range crosses the encoder wrap; writes Homing_Offset to EEPROM. Put the arm at a neutral pose if you enable this.
          </label>
        )}
        {active && <div className="help" style={{ marginTop: 8, color: 'var(--warn)' }}>
          ⚠ Torque off — heavy joints (arm) may droop. Support the arm while sweeping each joint.
        </div>}
        {msg && <div className="help" style={{ marginTop: 8, color: 'var(--ok)' }}>{msg}</div>}
      </Card>

      <Card title={`${unit} joints — raw min/max`}>
        <table>
          <thead><tr>
            <th>Motor</th><th>ID</th><th>Raw now</th><th>Rec min</th><th>Rec max</th><th>Span</th>
            <th>Saved range (old)</th><th>Dir</th><th></th>
          </tr></thead>
          <tbody>
            {motors.map(m => (
              <tr key={m.name} style={{ opacity: m.online ? 1 : 0.4 }}>
                <td>{m.name}</td>
                <td className="num">{m.servo_id}</td>
                <td className="num">{m.raw ?? '—'}</td>
                <td className="num">{m.rec_min ?? '—'}</td>
                <td className="num">{m.rec_max ?? '—'}</td>
                <td className="num" style={{ color: m.rec_span > 100 ? 'var(--ok)' : 'var(--muted)' }}>{m.rec_span || 0}</td>
                <td className="num help">{m.cur_min}–{m.cur_max}</td>
                <td>{m.drive_mode ? '⟲ inv' : '→ norm'}</td>
                <td><button className="btn" onClick={() => flip(m)}>Flip dir</button></td>
              </tr>
            ))}
          </tbody>
        </table>
        <div className="help" style={{ marginTop: 10 }}>
          <b>Span</b> should grow as you sweep a joint (green when &gt;100 counts). <b>Flip dir</b> inverts
          that motor's direction (drive_mode) live — use it if a joint moves the opposite way from the slider.
          Not-online joints are dimmed.
        </div>
      </Card>

      {/* Hand tendon-model setup: dead zones, home pose, per-joint tuning, pinch
          poses. Only relevant to the hand unit. */}
      {unit === 'hand' && <div style={{ marginTop: 14 }}><HandSetup snap={snap} /></div>}
    </>
  )
}
