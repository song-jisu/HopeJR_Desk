import React, { useEffect, useState } from 'react'
import { Card } from '../components.jsx'
import { api } from '../api.js'

// One physical DOF per entry: [key, label, min, max] in degrees. The URDF splits
// each into a driven `*_support` joint plus a mimic, so one angle covers both.
export const JOINTS = {
  thumb: [['cmc', 'CMC (rotation)', -172, -3], ['mcp', 'MCP', 0, 56],
          ['pip', 'PIP', 0, 56], ['dip', 'DIP', 0, 56]],
  other: [['mcp_flex', 'MCP flex (α)', 0, 90], ['mcp_spread', 'MCP spread (β)', -45, 45],
          ['pip', 'PIP+DIP', 0, 68]],
}
export const FINGER_MOTORS = [
  ['thumb', ['thumb_cmc', 'thumb_mcp', 'thumb_pip', 'thumb_dip']],
  ['index', ['index_radial_flexor', 'index_ulnar_flexor', 'index_pip_dip']],
  ['middle', ['middle_radial_flexor', 'middle_ulnar_flexor', 'middle_pip_dip']],
  ['ring', ['ring_radial_flexor', 'ring_ulnar_flexor', 'ring_pip_dip']],
  ['pinky', ['pinky_radial_flexor', 'pinky_ulnar_flexor', 'pinky_pip_dip']],
]
export const keysFor = (f) => JOINTS[f === 'thumb' ? 'thumb' : 'other']

// Thumb tendon slack (percent of each motor's travel that doesn't move the
// joint). Depends on how the hand was assembled/strung, so it lives in setup.
const DEADZONE_MOTORS = ['thumb_mcp', 'thumb_pip', 'thumb_dip']

function DeadZone({ snap }) {
  const kin = snap?.hand_joints || {}
  const dz = kin.deadzone_pct || {}
  return (
    <Card title="Tendon Slack (assembly) — thumb dead zones" style={{ marginBottom: 14 }}>
      <div className="help" style={{ marginBottom: 10 }}>
        The first part of a thumb motor's travel only takes up tendon slack — the joint
        stays still. This depends on how you strung the hand. Enter the percent of travel
        that is dead before the joint starts to move (defaults 20 / 50 / 25).
      </div>
      <div className="chip-input">
        {DEADZONE_MOTORS.map(m => (
          <span key={m} className="pill" style={{ gap: 4 }}>
            <span className="help">{m.replace('thumb_', '')}</span>
            <input type="number" step="1" min="0" max="95" style={{ width: 70 }}
              defaultValue={dz[m] ?? 0}
              onBlur={e => api.handKinDeadzone({ [m]: Number(e.target.value) })} />
            <span className="help">%</span>
          </span>
        ))}
      </div>
    </Card>
  )
}

export function HomePose({ snap }) {
  const [pose, setPose] = useState(null)      // the reference pose being edited
  const [saved, setSaved] = useState(null)    // what the backend last confirmed
  const [msg, setMsg] = useState('')
  const kin = snap?.hand_joints || {}
  const live = kin.fingers || {}
  const homeAt = kin.reference_at || {}

  // seed the editor from whatever reference is already stored (zeros if none)
  useEffect(() => {
    api.handKinematics()
      .then(k => { setPose(k.reference_pose || null); setSaved(k.configured) })
      .catch(() => {})
  }, [])

  // Push on every change. It only rewrites a small JSON and recomputes offsets,
  // so the measured column updates live as the slider moves.
  const edit = (f, k, v) => {
    const next = { ...pose, [f]: { ...pose[f], [k]: v } }
    setPose(next)
    api.handKinReference(next).then(() => { setSaved(true); setMsg('') })
      .catch(e => setMsg(String(e)))
  }
  const reset = async () => {
    await api.handKinReset()
    const k = await api.handKinematics()
    setPose(k.reference_pose || null); setSaved(false); setMsg('offsets cleared')
  }

  return (
    <Card title="Home Pose — what the hand looks like at startup" style={{ marginBottom: 14 }}>
      <div className="help" style={{ marginBottom: 12 }}>
        At startup every hand motor sits at a calibration end-stop (min or max), and the
        fingers are <b>not</b> straight — but the tendon model assumes straight at its own
        zero. Drag each slider to the angle that joint is <b>actually</b> at in that home
        pose. Each change is solved joint → motor and stored as the motor's θ offset, so
        the readings on the right update immediately.
        {saved === false && <> <b style={{ color: 'var(--warn)' }}>
          No reference saved yet — angles below are meaningless until you set one.</b></>}
      </div>

      {!pose && <div className="help">loading…</div>}
      {pose && (
        <div className="grid cols-3" style={{ gap: 12 }}>
          {FINGER_MOTORS.map(([f, names]) => (
            <div key={f} className="finger" style={{ alignItems: 'stretch' }}>
              <div style={{ display: 'flex', justifyContent: 'space-between' }}>
                <span className="name" style={{ textTransform: 'capitalize' }}>{f}</span>
                <span className="help" title="motor values at home">
                  {names.map(n => (homeAt[n] ?? 0).toFixed(0)).join(' / ')}%
                </span>
              </div>
              {keysFor(f).map(([k, label, lo, hi]) => (
                <div key={k} style={{ marginTop: 8 }}>
                  <div className="help" style={{ display: 'flex', justifyContent: 'space-between' }}>
                    <span>{label}</span>
                    <span>
                      <span className="val">{(pose[f]?.[k] ?? 0).toFixed(0)}°</span>
                      <span style={{ opacity: 0.55 }}>
                        {' '}→ now {live[f]?.[k] !== undefined ? `${live[f][k].toFixed(1)}°` : '–'}
                      </span>
                    </span>
                  </div>
                  <input type="range" min={lo} max={hi} step={1}
                    value={Math.round(pose[f]?.[k] ?? 0)}
                    onChange={e => edit(f, k, Number(e.target.value))} />
                </div>
              ))}
            </div>
          ))}
        </div>
      )}

      <div className="btn-row" style={{ marginTop: 12 }}>
        <button className="btn" onClick={reset}>Clear offsets (back to model zero)</button>
        {msg && <span className="help" style={{ color: 'var(--warn)' }}>{msg}</span>}
      </div>
      <div className="help" style={{ marginTop: 8 }}>
        Anchored to the <b>home</b> motor values shown per finger, not to wherever the hand
        is now — so moving the hand afterwards does not disturb the setting.
      </div>
    </Card>
  )
}

export function JointTuning({ snap }) {
  const [detail, setDetail] = useState(false)
  const kin = snap?.hand_joints || {}
  const live = kin.fingers || {}

  return (
    <Card title="Finger Joint Angles — tendon model" style={{ marginBottom: 14 }}>
      <table style={{ marginBottom: 8 }}>
        <thead>
          <tr><th>Finger</th><th colSpan={4}>joint angles now (deg)</th></tr>
        </thead>
        <tbody>
          {FINGER_MOTORS.map(([f]) => {
            const cols = keysFor(f)
            return (
              <tr key={f}>
                <td style={{ textTransform: 'capitalize' }}>{f}</td>
                {[0, 1, 2, 3].map(i => {
                  const c = cols[i]
                  return <td key={i} className="num">
                    {c && live[f]?.[c[0]] !== undefined
                      ? <span title={c[1]}>{c[1].split(' ')[0]} {live[f][c[0]].toFixed(1)}°</span>
                      : <span className="help">–</span>}
                  </td>
                })}
              </tr>
            )
          })}
        </tbody>
      </table>

      <div style={{ marginTop: 10 }}>
        <button className="btn" onClick={() => setDetail(d => !d)}>
          {detail ? 'Hide' : 'Show'} moment-arm tuning
        </button>
      </div>
      {detail && (
        <>
          <div className="help" style={{ margin: '8px 0' }}>
            <b>Moment arm</b> (mm): flexion per motor count — 3D bends more than real →
            increase. <b>Slack</b> (rad): dead zone at the start — if the 3D joint bends
            while the real one is still straight at low travel, increase slack.
          </div>
          <table style={{ marginTop: 4 }}>
            <thead><tr><th>Motor</th><th>θ (rad)</th><th>moment arm</th><th>slack</th></tr></thead>
            <tbody>
              {Object.entries(kin.theta || {}).map(([n, th]) => (
                <tr key={n}>
                  <td>{n}</td>
                  <td className="num">{th.toFixed(3)}</td>
                  <td className="num">
                    <input type="number" step="0.5" min="0.5"
                      defaultValue={kin.moment_arm?.[n] ?? 7.6}
                      style={{ width: 80 }}
                      onBlur={e => api.handKinMomentArm({ [n]: Number(e.target.value) })} />
                  </td>
                  <td className="num">
                    <input type="number" step="0.05" min="0"
                      defaultValue={kin.flex_slack?.[n] ?? 0}
                      style={{ width: 80 }}
                      onBlur={e => api.handKinSlack({ [n]: Number(e.target.value) })} />
                  </td>
                </tr>
              ))}
            </tbody>
          </table>

          <div className="help" style={{ margin: '10px 0 4px' }}>
            MCP (four fingers). <b>Radial/Ulnar arm</b> = each tendon's flexion moment arm
            — raise the side that over-flexes. Balance them so a full curl has <b>zero</b>
            spread. <b>Spread</b> = tilt per imbalance (negative flips the side).
          </div>
          <table>
            <thead><tr><th>Finger</th><th>radial arm</th><th>ulnar arm</th><th>spread</th></tr></thead>
            <tbody>
              {['index', 'middle', 'ring', 'pinky'].map(f => (
                <tr key={f}>
                  <td style={{ textTransform: 'capitalize' }}>{f}</td>
                  <td className="num">
                    <input type="number" step="0.05" min="0.05"
                      defaultValue={kin.mcp_flex_arm_radial?.[f] ?? 1}
                      style={{ width: 80 }}
                      onBlur={e => api.handKinMcpGain({ radial: { [f]: Number(e.target.value) } })} />
                  </td>
                  <td className="num">
                    <input type="number" step="0.05" min="0.05"
                      defaultValue={kin.mcp_flex_arm_ulnar?.[f] ?? 1}
                      style={{ width: 80 }}
                      onBlur={e => api.handKinMcpGain({ ulnar: { [f]: Number(e.target.value) } })} />
                  </td>
                  <td className="num">
                    <input type="number" step="0.05"
                      defaultValue={kin.mcp_spread_gain?.[f] ?? -0.6}
                      style={{ width: 80 }}
                      onBlur={e => api.handKinMcpGain({ spread: { [f]: Number(e.target.value) } })} />
                  </td>
                </tr>
              ))}
            </tbody>
          </table>
        </>
      )}

      <div style={{ marginTop: 12, paddingTop: 10, borderTop: '1px solid var(--border)' }}>
        <div className="help" style={{ marginBottom: 6 }}>
          <b>Pinch poses</b> — the camera can't make the robot's thumb tip meet a finger on
          its own. Drive the sliders until the tips actually touch, then Capture. Camera
          tracking then blends to it whenever it sees you pinch that finger.
        </div>
        <div className="btn-row">
          {['index', 'middle', 'ring', 'pinky'].map(f => {
            const has = (kin.pinch_captured || []).includes(f)
            return (
              <span key={f} className="pill" style={{ gap: 4 }}>
                <button className="btn" onClick={() => api.handPinchCapture(f)}>
                  {has ? '✓ ' : ''}Capture thumb+{f}
                </button>
                {has && <button className="btn" title="clear"
                  onClick={() => api.handPinchClear(f)}>✕</button>}
              </span>
            )
          })}
        </div>
      </div>

      <div className="help" style={{ marginTop: 10 }}>
        PIP and DIP share one motor on the four fingers, so that split is only meaningful
        in free motion — on contact the load, not the geometry, decides it.
      </div>
    </Card>
  )
}

// EXPERIMENTAL data collection: log which finger is touching the thumb at the
// current motor state, storing only the thumb + that finger's motors. Used to
// fit the four-finger MCP model to real contact poses. Remove if it does not
// prove reusable across HopeJR hands.
function ContactLog() {
  const [counts, setCounts] = useState({})
  const [total, setTotal] = useState(0)
  const [msg, setMsg] = useState('')

  const refresh = () => api.contactSamples()
    .then(d => { setCounts(d.counts || {}); setTotal(d.total || 0) }).catch(() => {})
  useEffect(() => { refresh() }, [])

  const record = async (f) => {
    const r = await api.contactSample(f)
    setMsg(r.message || ''); refresh()
  }
  const clear = async () => { await api.contactClear(); setMsg('cleared'); refresh() }
  const download = async () => {
    const d = await api.contactSamples()
    const blob = new Blob([JSON.stringify(d.samples, null, 2)], { type: 'application/json' })
    const a = document.createElement('a')
    a.href = URL.createObjectURL(blob); a.download = 'contact_samples.json'; a.click()
  }

  return (
    <Card title="Contact Samples — thumb ↔ finger (data collection)" style={{ marginBottom: 14 }}>
      <div className="help" style={{ marginBottom: 10 }}>
        Move the thumb and one finger until their tips <b>touch</b>, then click that finger
        to log the current motor state (only the thumb + that finger's motors are stored).
        Take many samples with the thumb in <b>different</b> poses. This is raw data for
        fitting the four-finger MCP model — collect a spread, then download.
      </div>
      <div className="btn-row" style={{ marginBottom: 8 }}>
        {['index', 'middle', 'ring', 'pinky'].map(f => (
          <button key={f} className="btn primary" onClick={() => record(f)}>
            thumb+{f} <span style={{ opacity: 0.7 }}>({counts[f] || 0})</span>
          </button>
        ))}
      </div>
      <div className="btn-row">
        <span className="help">{total} sample{total === 1 ? '' : 's'} total</span>
        <button className="btn" disabled={!total} onClick={download}>Download JSON</button>
        <button className="btn" disabled={!total} onClick={clear}>Clear all</button>
        {msg && <span className="help" style={{ color: 'var(--ok)' }}>{msg}</span>}
      </div>
    </Card>
  )
}

// Everything hand-model related, shown on the Calibration page.
export function HandSetup({ snap }) {
  return (
    <>
      <DeadZone snap={snap} />
      <HomePose snap={snap} />
      <JointTuning snap={snap} />
      <ContactLog />
    </>
  )
}
