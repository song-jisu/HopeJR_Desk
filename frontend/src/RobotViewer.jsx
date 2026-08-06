import React, { useEffect, useRef, useState } from 'react'
import * as THREE from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js'
import { ColladaLoader } from 'three/examples/jsm/loaders/ColladaLoader.js'
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js'
import URDFLoader from 'urdf-loader'
import { api } from './api.js'

const URDF = '/assets/hopejr_right_arm_description/urdf/hopejr_right_arm.urdf'
const ARM = ['shoulder_pitch', 'shoulder_yaw', 'shoulder_roll', 'elbow_flex',
  'wrist_roll', 'wrist_yaw', 'wrist_pitch']
const DEG = Math.PI / 180
const LS_KEY = 'hopejr_viewer_tuning_v3'   // bumped: old saved tuning is discarded

// --- hand ---------------------------------------------------------------------
// The arm URDF embeds the whole hand under a `hand_` prefix. Unlike the arm we do
// NOT map normalized 0..100 onto the joint limits — the tendon transmission is
// nonlinear, so that would show a visibly wrong pose. Instead the backend's
// tendon model (hand_joints in telemetry) hands us real angles in degrees.
//
// Every DOF is split into a free `*_support` joint plus a mimic of it, wired in
// series on the same axis with multiplier 1 — so the link's total rotation is
// TWICE the support value, and we drive support = angle / 2. Distal joints that
// mimic another joint (index_dip_2 etc.) follow on their own.
// [urdfJoint, divisor, sign]. divisor 2 = the support/mimic split above; 1 for a
// joint that has no mimic. sign corrects the URDF axis against the model's own
// convention — beta is negated because the PDF measures spread the opposite way
// round from the URDF's +Z on `*_pip_support` (fingers deviated the wrong way).
const handMap = (f) => ({
  mcp_flex: [`hand_${f}_mcp_2_support`, 2, 1],    // alpha
  // beta (URDF calls this joint "pip"). +1 now that the MCP model's spread gain
  // was fitted to real contact data and its sign matches the URDF spread axis.
  mcp_spread: [`hand_${f}_pip_support`, 2, 1],
  pip: [`hand_${f}_dip_1_support`, 2, 1],         // PIP; DIP mimics it at 0.8
})
const HAND_JOINTS = {
  thumb: {
    cmc: ['hand_thumb_mcp_2', 1, 1],              // direct linkage, no mimic
    mcp: ['hand_thumb_pip_1_support', 2, 1],
    pip: ['hand_thumb_pip_2_support', 2, 1],
    dip: ['hand_thumb_dip_support', 2, 1],
  },
  index: handMap('index'), middle: handMap('middle'),
  ring: handMap('ring'), pinky: handMap('pinky'),
}

// Baked-in mounting correction (URDF axis/zero vs the physical robot). ALWAYS
// applied — the user never has to toggle these.
const BASE_OFFSET_DEG = { shoulder_pitch: 0 }
const BASE_INVERT = new Set(['shoulder_pitch', 'shoulder_yaw', 'shoulder_roll'])

// User fine-tuning on TOP of the base correction (panel), starts clean.
const DEFAULT_TUNING = Object.fromEntries(ARM.map(m => [m, { off: 0, inv: false }]))

function loadTuning() {
  try {
    const t = JSON.parse(localStorage.getItem(LS_KEY))
    if (t) return { ...structuredClone(DEFAULT_TUNING), ...t }
  } catch {}
  return structuredClone(DEFAULT_TUNING)
}

export default function RobotViewer({ snapRef }) {
  const mountRef = useRef(null)
  const robotRef = useRef(null)
  const jointMapRef = useRef({})
  const anglesRef = useRef({})
  const linksRef = useRef({})        // {linkName: {mass, com}}
  const downstreamRef = useRef({})   // motor -> [{obj, mass, com}]
  const tuningRef = useRef(loadTuning())
  const [tuning, setTuning] = useState(tuningRef.current)
  const [status, setStatus] = useState('loading URDF…')
  const [showTune, setShowTune] = useState(false)
  const [showMass, setShowMass] = useState(false)
  const [showGrav, setShowGrav] = useState(false)
  const [angles, setAngles] = useState({})
  const [grav, setGrav] = useState({})
  const [cur, setCur] = useState({})
  const [masses, setMasses] = useState({})   // {armLinkName: mass}
  const [simMode, setSimMode] = useState(false)   // drive URDF from sliders, no robot
  const [simVals, setSimVals] = useState(() => Object.fromEntries(ARM.map(m => [m, 0])))
  const simModeRef = useRef(false)
  const simValsRef = useRef(simVals)

  const setMass = (name, val) => {
    setMasses(m => ({ ...m, [name]: val }))
    if (linksRef.current[name]) linksRef.current[name].mass = Number(val) || 0
    api.setLink(name, Number(val) || 0).catch(() => {})
  }

  // keep the ref + localStorage in sync with the tuning state
  useEffect(() => {
    tuningRef.current = tuning
    try { localStorage.setItem(LS_KEY, JSON.stringify(tuning)) } catch {}
  }, [tuning])

  const setJoint = (m, key, val) =>
    setTuning(t => ({ ...t, [m]: { ...t[m], [key]: val } }))

  useEffect(() => { simModeRef.current = simMode }, [simMode])
  useEffect(() => { simValsRef.current = simVals }, [simVals])

  useEffect(() => {
    const mount = mountRef.current
    if (!mount) return
    const W = () => mount.clientWidth || 400
    const H = () => mount.clientHeight || 400

    const scene = new THREE.Scene()
    scene.background = new THREE.Color(0x0e1116)
    const camera = new THREE.PerspectiveCamera(45, W() / H(), 0.01, 100)
    camera.position.set(0.6, 0.5, 0.6)

    let renderer
    try { renderer = new THREE.WebGLRenderer({ antialias: true }) }
    catch { setStatus('WebGL not available'); return }
    renderer.setPixelRatio(window.devicePixelRatio)
    renderer.setSize(W(), H())
    mount.appendChild(renderer.domElement)

    scene.add(new THREE.HemisphereLight(0xffffff, 0x223344, 1.2))
    const dl = new THREE.DirectionalLight(0xffffff, 1.1); dl.position.set(1, 2, 1); scene.add(dl)
    scene.add(new THREE.GridHelper(2, 20, 0x2a3340, 0x1c232d))

    const controls = new OrbitControls(camera, renderer.domElement)
    controls.enableDamping = true

    const world = new THREE.Group()
    world.rotation.x = -Math.PI / 2   // URDF Z-up → Three.js Y-up
    scene.add(world)

    const mat = new THREE.MeshStandardMaterial({ color: 0x9aa7b4, metalness: 0.2, roughness: 0.7 })
    let meshOk = 0, meshErr = 0
    const manager = new THREE.LoadingManager()
    const loader = new URDFLoader(manager)
    loader.packages = {
      hopejr_arm_description: '/assets/hopejr_arm_description',
      hopejr_hand_description: '/assets/hopejr_hand_description',
      hopejr_right_arm_description: '/assets/hopejr_right_arm_description',
    }
    const applyMat = (o) => { o.traverse && o.traverse(n => { if (n.isMesh) n.material = mat }); return o }
    loader.loadMeshCb = (path, mgr, done) => {
      const p = path.replace(/\.dae$/i, '.obj')   // visual .obj: matches URDF scale, reliable
      const ext = p.split('.').pop().toLowerCase()
      const ok = (o) => { meshOk++; done(applyMat(o)) }
      const fail = (e) => { meshErr++; console.warn('[RobotViewer] mesh fail', p, e); done(null) }
      try {
        if (ext === 'obj') new OBJLoader(mgr).load(p, ok, undefined, fail)
        else if (ext === 'stl') new STLLoader(mgr).load(p, g => ok(new THREE.Mesh(g, mat)), undefined, fail)
        else if (ext === 'dae') new ColladaLoader(mgr).load(p, c => ok(c.scene), undefined, fail)
        else fail('ext')
      } catch (e) { fail(e) }
    }

    manager.onLoad = () => {
      const robot = robotRef.current
      if (!robot) return
      world.updateMatrixWorld(true)
      const box = new THREE.Box3().setFromObject(robot)
      if (box.isEmpty()) { setStatus(`no meshes (ok:${meshOk} fail:${meshErr})`); return }
      const c = box.getCenter(new THREE.Vector3())
      const s = box.getSize(new THREE.Vector3()).length() || 0.5
      camera.near = Math.max(1e-4, s / 1000); camera.far = s * 100; camera.updateProjectionMatrix()
      controls.target.copy(c)
      camera.position.set(c.x + s * 0.7, c.y + s * 0.5, c.z + s * 0.7)
      controls.update()
      setStatus(meshErr ? `${meshErr} meshes missing` : '')
    }

    // per-arm-joint downstream links (with mass) for gravity torque
    const buildDownstream = () => {
      const robot = robotRef.current
      if (!robot || !Object.keys(linksRef.current).length) return
      const ds = {}
      for (const [motor, jn] of Object.entries(jointMapRef.current)) {
        const joint = robot.joints[jn]; const list = []
        joint && joint.traverse(o => {
          const md = linksRef.current[o.name]
          if (md && md.mass > 0) list.push({ obj: o, mass: md.mass, com: md.com || [0, 0, 0] })
        })
        ds[motor] = list
      }
      downstreamRef.current = ds
      // arm links = everything downstream of the first joint; seed the mass panel
      const armLinks = {}
      for (const L of (ds.shoulder_pitch || [])) armLinks[L.obj.name] = L.mass
      if (Object.keys(armLinks).length) setMasses(armLinks)
    }
    fetch('/api/links').then(r => r.json()).then(d => { linksRef.current = d.links || {}; buildDownstream() }).catch(() => {})

    loader.load(URDF, (robot) => {
      robotRef.current = robot
      const map = {}
      for (const m of ARM) {
        const jn = robot.joints[m] ? m : (robot.joints['hand_' + m] ? 'hand_' + m : null)
        if (jn) { map[m] = jn; robot.joints[jn].ignoreLimits = true }   // offsets may exceed limits
      }
      jointMapRef.current = map
      world.add(robot)
      buildDownstream()
      setStatus('loading meshes…')
    }, undefined, (err) => {
      console.warn('[RobotViewer] URDF load error', err)
      setStatus('failed to load URDF — backend up & /assets proxied? (restart vite)')
    })

    let raf
    const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v))
    const animate = () => {
      raf = requestAnimationFrame(animate)
      const robot = robotRef.current, snap = snapRef.current, tune = tuningRef.current
      const simOn = simModeRef.current
      if (robot && (snap || simOn)) {
        const pos = {}
        if (simOn) { for (const m of ARM) pos[m] = simValsRef.current[m] ?? 0 }
        else { for (const mo of snap.motors) pos[mo.name] = mo.position }
        for (const [motor, joint] of Object.entries(jointMapRef.current)) {
          const j = robot.joints[joint]; let v = pos[motor]; const t = tune[motor] || {}
          if (j && v != null && j.limit) {
            // effective invert = baked-in XOR user; effective offset = base + user
            if (BASE_INVERT.has(motor) !== !!t.inv) v = -v
            const off = ((BASE_OFFSET_DEG[motor] || 0) + (t.off || 0)) * DEG
            const lo = Number(j.limit.lower), hi = Number(j.limit.upper)
            if (isFinite(lo) && isFinite(hi) && hi > lo) {
              const angle = lo + ((clamp(v, -100, 100) + 100) / 200) * (hi - lo) + off
              robot.setJointValue(joint, angle)
              anglesRef.current[motor] = angle / DEG
            }
          } else { anglesRef.current[motor] = j ? null : 'no joint' }
        }

        // hand: real joint angles straight from the tendon model, no remapping.
        // Joint limits are left ON here (the arm disables them) so an unset
        // reference pose shows as a clamped hand rather than a folded-inside-out
        // one.
        const hj = snap && snap.hand_joints && snap.hand_joints.fingers
        for (const finger in (hj || {})) {
          const map = HAND_JOINTS[finger]
          if (!map) continue
          for (const key in map) {
            const [jn, div, sign] = map[key]
            const deg = hj[finger][key]
            if (deg == null || !robot.joints[jn]) continue
            robot.setJointValue(jn, sign * deg * DEG / div)
          }
        }
      }
      controls.update(); renderer.render(scene, camera)
    }
    animate()

    const ro = new ResizeObserver(() => {
      camera.aspect = W() / H(); camera.updateProjectionMatrix(); renderer.setSize(W(), H())
    })
    ro.observe(mount)
    const G = new THREE.Vector3(0, -9.81, 0)   // down in the Three.js Y-up scene
    const dbgTimer = setInterval(() => {
      setAngles({ ...anglesRef.current })
      const snap = snapRef.current
      if (snap) {
        const c = {}
        for (const mo of snap.motors) if (mo.unit === 'arm') c[mo.name] = mo.current
        setCur(c)
      }
      // τg is now computed in the backend (app/kinematics.py) and streamed in
      // the telemetry snapshot — single source of truth. Prefer it; fall back to
      // the in-browser calc only if the backend field is absent (old backend).
      if (snap && snap.gravity && Object.keys(snap.gravity).length) {
        setGrav({ ...snap.gravity })
        return
      }
      const robot = robotRef.current
      if (!robot) return
      robot.updateMatrixWorld(true)
      const g = {}
      for (const [motor, jn] of Object.entries(jointMapRef.current)) {
        const joint = robot.joints[jn]
        if (!joint) continue
        const jpos = joint.getWorldPosition(new THREE.Vector3())
        const axis = joint.axis.clone().transformDirection(joint.matrixWorld).normalize()
        let tau = 0
        for (const L of (downstreamRef.current[motor] || [])) {
          const com = L.obj.localToWorld(new THREE.Vector3(L.com[0], L.com[1], L.com[2]))
          const r = com.sub(jpos)
          tau += r.cross(G.clone().multiplyScalar(L.mass)).dot(axis)
        }
        g[motor] = tau
      }
      setGrav(g)
    }, 250)

    return () => {
      cancelAnimationFrame(raf); clearInterval(dbgTimer); ro.disconnect(); controls.dispose(); renderer.dispose()
      if (renderer.domElement.parentNode) renderer.domElement.parentNode.removeChild(renderer.domElement)
    }
  }, [snapRef])

  const box = {
    position: 'absolute', background: '#161b22e6', border: '1px solid #2a3340',
    borderRadius: 6, fontSize: 11, color: '#8b98a5',
  }
  return (
    <div ref={mountRef} style={{ width: '100%', height: '100%', position: 'relative' }}>
      {status && <div style={{ ...box, top: 8, left: 8, right: 8, padding: '6px 10px', pointerEvents: 'none' }}>{status}</div>}
      <button onClick={() => { setSimMode(s => !s); setShowGrav(false); setShowMass(false); setShowTune(false) }}
        style={{ ...box, top: 8, right: 186, padding: '4px 8px', cursor: 'pointer', color: simMode ? '#4c8dff' : '#e6edf3' }}>
        🎚 sim
      </button>
      <button onClick={() => { setShowGrav(s => !s); setShowMass(false); setShowTune(false); setSimMode(false) }}
        style={{ ...box, top: 8, right: 124, padding: '4px 8px', cursor: 'pointer', color: '#e6edf3' }}>
        🖐 guide
      </button>
      <button onClick={() => { setShowMass(s => !s); setShowTune(false); setShowGrav(false) }}
        style={{ ...box, top: 8, right: 66, padding: '4px 8px', cursor: 'pointer', color: '#e6edf3' }}>
        ⚖ mass
      </button>
      <button onClick={() => { setShowTune(s => !s); setShowMass(false); setShowGrav(false) }}
        style={{ ...box, top: 8, right: 8, padding: '4px 8px', cursor: 'pointer', color: '#e6edf3' }}>
        ⚙ tune
      </button>
      {showMass && (
        <div style={{ ...box, top: 40, right: 8, padding: 10, width: 300, maxHeight: '80%', overflow: 'auto' }}>
          <div style={{ marginBottom: 6 }}><b>Gravity torque vs measured current</b></div>
          <div style={{ display: 'grid', gridTemplateColumns: '1fr 52px 52px', gap: 3, alignItems: 'center' }}>
            <span className="help">joint</span><span className="help" style={{ textAlign: 'right' }}>τg N·m</span><span className="help" style={{ textAlign: 'right' }}>cur</span>
            {ARM.map(m => (
              <React.Fragment key={m}>
                <span>{m.replace('shoulder_', 'sh_').replace('wrist_', 'wr_')}</span>
                <span style={{ textAlign: 'right', color: '#4c8dff', fontVariantNumeric: 'tabular-nums' }}>{grav[m] == null ? '—' : grav[m].toFixed(2)}</span>
                <span style={{ textAlign: 'right', fontVariantNumeric: 'tabular-nums' }}>{cur[m] == null ? '—' : Math.round(cur[m])}</span>
              </React.Fragment>
            ))}
          </div>
          <div style={{ marginTop: 10, marginBottom: 4 }}><b>Link mass (kg)</b></div>
          {Object.keys(masses).length === 0 && <div className="help">loading…</div>}
          {Object.entries(masses).map(([name, mass]) => (
            <div key={name} style={{ display: 'grid', gridTemplateColumns: '1fr 90px 46px', gap: 4, alignItems: 'center', margin: '3px 0' }}>
              <span style={{ fontSize: 10 }}>{name.replace('_link', '')}</span>
              <input type="range" min={0} max={3} step={0.01} value={mass}
                onChange={e => setMass(name, Number(e.target.value))} />
              <input type="number" step={0.05} value={mass}
                onChange={e => setMass(name, Number(e.target.value))}
                style={{ width: 44, background: '#0e1116', color: '#e6edf3', border: '1px solid #2a3340', borderRadius: 4 }} />
            </div>
          ))}
          <div className="help" style={{ marginTop: 8, fontSize: 10 }}>
            Move the arm to a pose, hold still, and adjust masses until τg tracks the measured current across
            poses. Saved live to the backend.
          </div>
        </div>
      )}
      {showGrav && (
        <div style={{ ...box, top: 40, right: 8, padding: 10, width: 300, maxHeight: '80%', overflow: 'auto' }}>
          <div style={{ marginBottom: 6 }}><b>Hand-guiding = free-drive</b></div>
          <div className="help" style={{ fontSize: 10, marginBottom: 10 }}>
            When you start teaching, the arm's motor torque is <b>released</b> so you can move it freely by hand and
            trace a trajectory — positions are recorded the whole time. Gearbox friction holds it between moves;
            heavy joints (shoulder) may drift, so support the arm. On stop, it re-holds where you left it.
          </div>
          <div className="help" style={{ fontSize: 10, marginBottom: 10 }}>
            True gravity-float isn't possible on these servos (no torque sensing), so we free the arm instead of
            trying to compensate gravity.
          </div>
          <div style={{ marginBottom: 4 }}><b style={{ fontSize: 12 }}>Gravity torque τg (N·m)</b> <span className="help">info only</span></div>
          <div style={{ display: 'grid', gridTemplateColumns: '1fr 60px', gap: 2, alignItems: 'center' }}>
            {ARM.map(m => (
              <React.Fragment key={m}>
                <span style={{ fontSize: 10 }}>{m.replace('shoulder_', 'sh_').replace('wrist_', 'wr_')}</span>
                <span style={{ textAlign: 'right', fontSize: 10, fontVariantNumeric: 'tabular-nums', color: '#4c8dff' }}>{grav[m] == null ? '—' : grav[m].toFixed(2)}</span>
              </React.Fragment>
            ))}
          </div>
        </div>
      )}
      {showTune && (
        <div style={{ ...box, top: 40, right: 8, padding: 10, width: 300, maxHeight: '75%', overflow: 'auto' }}>
          <div style={{ display: 'grid', gridTemplateColumns: '1fr 48px 24px 36px 48px', gap: 4, alignItems: 'center' }}>
            <b>joint</b><b>offset°</b><b>inv</b><b>now°</b><b title="gravity torque (N·m)">τg</b>
            {ARM.map(m => (
              <React.Fragment key={m}>
                <span>{m.replace('shoulder_', 'sh_').replace('wrist_', 'wr_')}</span>
                <input type="number" step={5} value={tuning[m].off}
                  onChange={e => setJoint(m, 'off', Number(e.target.value))}
                  style={{ width: 44, background: '#0e1116', color: '#e6edf3', border: '1px solid #2a3340', borderRadius: 4 }} />
                <input type="checkbox" checked={tuning[m].inv} onChange={e => setJoint(m, 'inv', e.target.checked)} />
                <span style={{ textAlign: 'right', fontVariantNumeric: 'tabular-nums' }}>
                  {angles[m] == null ? '—' : (typeof angles[m] === 'string' ? '✗' : Math.round(angles[m]))}
                </span>
                <span style={{ textAlign: 'right', fontVariantNumeric: 'tabular-nums', color: '#4c8dff' }}>
                  {grav[m] == null ? '—' : grav[m].toFixed(2)}
                </span>
              </React.Fragment>
            ))}
          </div>
          <div className="help" style={{ marginTop: 6, fontSize: 10 }}>
            τg = gravity torque each joint holds at the current pose (from URDF link masses). Edit masses in Configuration.
          </div>
          <button onClick={() => setTuning(structuredClone(DEFAULT_TUNING))}
            style={{ ...box, marginTop: 8, padding: '4px 8px', cursor: 'pointer', position: 'static', color: '#e6edf3' }}>
            reset defaults
          </button>
        </div>
      )}
      {simMode && (
        <div style={{ ...box, top: 40, right: 8, padding: 10, width: 300, maxHeight: '80%', overflow: 'auto' }}>
          <div style={{ marginBottom: 6 }}><b>Sim drive</b> <span className="help">no robot — motor % → URDF</span></div>
          {ARM.map(m => (
            <div key={m} style={{ display: 'grid', gridTemplateColumns: '64px 1fr 42px', gap: 6, alignItems: 'center', margin: '3px 0' }}>
              <span style={{ fontSize: 10 }}>{m.replace('shoulder_', 'sh_').replace('wrist_', 'wr_')}</span>
              <input type="range" min={-100} max={100} step={1} value={simVals[m]}
                onChange={e => setSimVals(v => ({ ...v, [m]: Number(e.target.value) }))} />
              <span style={{ textAlign: 'right', fontVariantNumeric: 'tabular-nums', fontSize: 10 }}>
                {angles[m] == null ? '—' : (typeof angles[m] === 'string' ? '✗' : Math.round(angles[m]) + '°')}
              </span>
            </div>
          ))}
          <button onClick={() => setSimVals(Object.fromEntries(ARM.map(m => [m, 0])))}
            style={{ ...box, marginTop: 8, padding: '4px 8px', cursor: 'pointer', position: 'static', color: '#e6edf3' }}>
            all → 0
          </button>
          <div className="help" style={{ marginTop: 6, fontSize: 10 }}>
            Drives each motor's normalized value (−100…100) through the real mapping (limits / offset / invert),
            so you can check the URDF with the robot disconnected. Right column = resulting joint angle.
          </div>
        </div>
      )}
    </div>
  )
}
