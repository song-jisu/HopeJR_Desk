import React, { useEffect, useRef, useState } from 'react'
import { Card } from '../components.jsx'
import { api } from '../api.js'
import CameraHand from '../CameraHand.jsx'

// Hand-guiding (PLAN §5): the arm goes compliant so you move it by hand while
// positions record; then save / replay motions.
export default function Teaching() {
  const [st, setSt] = useState(null)
  const [name, setName] = useState('')
  const [msg, setMsg] = useState('')
  const timer = useRef()

  const poll = () => api.teachStatus().then(setSt).catch(() => {})
  useEffect(() => { poll(); timer.current = setInterval(poll, 500); return () => clearInterval(timer.current) }, [])

  const note = (m) => { setMsg(m); setTimeout(() => setMsg(''), 2500) }
  const start = async () => { await api.teachStart(); note('Arm torque off — move it by hand. Recording…') }
  const stop = async () => { const r = await api.teachStop(); note(r.message) }
  const save = async () => { if (!name.trim()) return note('enter a name'); const r = await api.teachSave(name); note(r.message); setName('') }
  const replay = async (n) => { const r = await api.teachReplay(n); note(r.message) }
  const del = async (n) => { await api.teachDelete(n); note('deleted ' + n) }

  if (!st) return <p className="page-sub">Loading…</p>
  if (!st.supported) return <Card title="Motion Teaching">
    <div className="help">Hand-guiding needs the <b>serial</b> backend (real robot). The mock backend records the pose but there is no physical arm to move.</div>
  </Card>

  return (
    <>
      <Card title="Hand-Guiding" style={{ marginBottom: 14 }}>
        <div className="help" style={{ marginBottom: 10 }}>
          Start cuts <b>arm</b> torque so you can move it freely by hand; positions record live, then Save.
          The <b>hand</b> stays powered — it is a tendon mechanism and cannot be backdriven — so pose it
          with the camera below (or the Hand page sliders). Both halves land in the same recording.
          Heavy joints sag once torque is off, so support the arm before you press Start.
        </div>
        <div className="btn-row">
          {!st.active
            ? <button className="btn green" onClick={start}>Start Hand-Guiding</button>
            : <button className="btn danger" onClick={stop}>Stop &amp; Keep Recording</button>}
          <span style={{ flex: 1 }} />
          {st.active && <span className="pill"><span className="dot on" />REC · {st.points} pts · {st.duration}s</span>}
        </div>
        <div className="chip-input" style={{ marginTop: 12 }}>
          <span className="help">Save recorded motion as:</span>
          <input value={name} onChange={e => setName(e.target.value)} placeholder="motion name" />
          <button className="btn primary" onClick={save} disabled={st.active || !st.points}>Save</button>
        </div>
        {!st.active && st.points > 0 && <div className="help" style={{ marginTop: 6 }}>{st.points} points recorded (unsaved). Save to keep.</div>}
        {msg && <div className="help" style={{ marginTop: 8, color: 'var(--ok)' }}>{msg}</div>}
      </Card>

      <Card title="Hand — camera control" style={{ marginBottom: 14 }}>
        <div className="help" style={{ marginBottom: 10 }}>
          The hand copies your real hand while you guide the arm. It only drives the
          servos while a recording is running, so you can frame the shot first.
        </div>
        <CameraHand active driving={!!st.active}
          driveHint="Press “Start Hand-Guiding” above — the hand only moves while recording." />
      </Card>

      <Card title="Saved Motions">
        {!st.motions.length && <div className="help">No motions yet. Record one above.</div>}
        {st.motions.length > 0 && (
          <table>
            <thead><tr><th>Name</th><th>Points</th><th>Duration</th><th>Calibration</th><th></th></tr></thead>
            <tbody>
              {st.motions.map(m => (
                <tr key={m.name}>
                  <td>{m.name}</td>
                  <td className="num">{m.points}</td>
                  <td className="num">{m.duration}s</td>
                  {/* Recorded positions are percentages of each joint's calibrated
                      range, so a changed range moves them. Replay remaps them via
                      raw encoder counts — unless the motion predates stamping. */}
                  <td>
                    {m.calib_unknown
                      ? <span style={{ color: 'var(--warn)' }} title="Recorded before calibration was stamped — cannot be remapped">unstamped</span>
                      : m.calib_drift
                        ? <span style={{ color: 'var(--warn)' }} title="Calibration changed since recording; replay remaps via raw encoder counts">{m.calib_drift} remapped</span>
                        : <span className="help">match</span>}
                  </td>
                  <td>
                    <div className="btn-row">
                      <button className="btn" onClick={() => replay(m.name)} disabled={st.active}>▶ Replay</button>
                      <button className="btn" onClick={() => del(m.name)}>Delete</button>
                    </div>
                  </td>
                </tr>
              ))}
            </tbody>
          </table>
        )}
        <div className="help" style={{ marginTop: 10 }}>
          ⚠ Replay drives the arm to the recorded positions — keep the workspace clear and a hand near E-Stop.
        </div>
      </Card>
    </>
  )
}
