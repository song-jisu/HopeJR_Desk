import React, { useEffect, useRef, useState } from 'react'
import { FilesetResolver, HandLandmarker } from '@mediapipe/tasks-vision'
import { landmarksToPose, smoothPose } from './handTracking.js'
import { api } from './api.js'

// WASM + model are served from /public/mediapipe, not a CDN, so this works with
// no internet (and the backend is on WSL where a CDN fetch is flaky anyway).
const WASM = '/mediapipe/wasm'
const MODEL = '/mediapipe/hand_landmarker.task'
const SEND_HZ = 15          // servo command rate; landmark detection runs faster

// finger connections, for the overlay skeleton
const BONES = [[0, 1], [1, 2], [2, 3], [3, 4], [0, 5], [5, 6], [6, 7], [7, 8],
  [0, 9], [9, 10], [10, 11], [11, 12], [0, 13], [13, 14], [14, 15], [15, 16],
  [0, 17], [17, 18], [18, 19], [19, 20], [5, 9], [9, 13], [13, 17]]

export default function CameraHand({ active, driving, driveHint }) {
  const videoRef = useRef(null)
  const canvasRef = useRef(null)
  const landmarkerRef = useRef(null)
  const streamRef = useRef(null)
  const rafRef = useRef(0)
  const poseRef = useRef(null)
  const lastSendRef = useRef(0)
  const inFlightRef = useRef(false)   // backpressure: at most one command in flight
  const dropRef = useRef(0)
  const drivingRef = useRef(driving)
  const optsRef = useRef({ invertSpread: false, mirror: true, retarget: true, wApex: 1, smooth: 0.25 })

  const [devices, setDevices] = useState([])
  const [deviceId, setDeviceId] = useState('')
  const [status, setStatus] = useState('idle')
  const [pose, setPose] = useState(null)
  const [fps, setFps] = useState(0)
  const [invertSpread, setInvertSpread] = useState(false)
  const [mirror, setMirror] = useState(true)
  const [hint, setHint] = useState('')
  const [diag, setDiag] = useState(null)
  const [record, setRecord] = useState(false)
  const [clips, setClips] = useState([])       // {name, url, size} from this session
  const [sendInfo, setSendInfo] = useState({ n: 0, err: '' })
  const [recording, setRecording] = useState(false)
  const [retarget, setRetarget] = useState(true)   // placement matching vs angle copy
  const [wApex, setWApex] = useState(1)
  const [smooth, setSmooth] = useState(0.25)   // lower = steadier, higher = snappier
  const recorderRef = useRef(null)
  const chunksRef = useRef([])

  // Records the raw camera stream (not the landmark overlay) for the whole
  // Start..Stop window, so a teaching take has matching footage.
  const startRecording = (stream) => {
    const type = ['video/webm;codecs=vp9', 'video/webm;codecs=vp8', 'video/webm']
      .find(t => MediaRecorder.isTypeSupported(t))
    if (!type) { setHint('This browser cannot record webm.'); return }
    chunksRef.current = []
    const rec = new MediaRecorder(stream, { mimeType: type })
    rec.ondataavailable = e => { if (e.data.size) chunksRef.current.push(e.data) }
    rec.onstop = () => {
      const blob = new Blob(chunksRef.current, { type })
      chunksRef.current = []
      if (!blob.size) return
      const name = `hand-${new Date().toISOString().replace(/[:.]/g, '-')}.webm`
      setClips(c => [{ name, url: URL.createObjectURL(blob), size: blob.size }, ...c])
    }
    rec.start(1000)          // flush every second so a crash loses at most 1 s
    recorderRef.current = rec
    setRecording(true)
  }
  const stopRecording = () => {
    const r = recorderRef.current
    recorderRef.current = null
    setRecording(false)
    if (r && r.state !== 'inactive') r.stop()
  }

  // "no camera" has several unrelated causes (insecure origin, denied
  // permission, device held by another app, enumerate returning stubs). Ask the
  // browser directly rather than guessing.
  const runDiag = async () => {
    const d = {
      'page origin': location.origin,
      'secure context': String(window.isSecureContext),
      'navigator.mediaDevices': String(!!navigator.mediaDevices),
      'getUserMedia': String(!!navigator.mediaDevices?.getUserMedia),
    }
    try {
      const p = await navigator.permissions?.query({ name: 'camera' })
      d['permission'] = p ? p.state : '(not queryable)'
    } catch (e) { d['permission'] = `(query failed: ${e.name})` }
    try {
      const all = await navigator.mediaDevices.enumerateDevices()
      d['devices total'] = String(all.length)
      d['video inputs'] = String(all.filter(x => x.kind === 'videoinput').length)
      d['kinds'] = all.map(x => x.kind).join(', ') || '(empty list)'
    } catch (e) { d['enumerateDevices'] = `${e.name}: ${e.message}` }
    try {
      const s = await navigator.mediaDevices.getUserMedia({ video: true })
      d['getUserMedia test'] = 'OK — ' + s.getVideoTracks().map(t => t.label).join(', ')
      s.getTracks().forEach(t => t.stop())
    } catch (e) { d['getUserMedia test'] = `${e.name}: ${e.message}` }
    setDiag(d)
    listDevices()
  }

  useEffect(() => { drivingRef.current = driving }, [driving])
  useEffect(() => { optsRef.current = { invertSpread, mirror, retarget, wApex, smooth } },
    [invertSpread, mirror, retarget, wApex, smooth])

  // Browsers hide deviceIds AND labels until the page has been granted camera
  // access at least once — before that enumerateDevices() reports nothing
  // useful, which looks exactly like "no camera plugged in". `prompt` opens a
  // throwaway stream purely to trigger the permission dialog.
  const listDevices = async ({ prompt = false } = {}) => {
    if (!navigator.mediaDevices?.getUserMedia) {
      setHint('This page is not a secure context, so the browser blocks camera access. '
        + 'Open the UI at http://localhost:… (not the WSL IP) and reload.')
      return []
    }
    try {
      if (prompt) {
        const tmp = await navigator.mediaDevices.getUserMedia({ video: true })
        tmp.getTracks().forEach(t => t.stop())
      }
      const cams = (await navigator.mediaDevices.enumerateDevices())
        .filter(d => d.kind === 'videoinput')
      setDevices(cams)
      setDeviceId(id => (cams.some(c => c.deviceId === id) ? id : cams[0]?.deviceId || ''))
      setHint(cams.length
        ? (cams[0].label ? '' : 'Press “Allow camera” to reveal device names.')
        : 'The browser reports no video input. Check that the camera is not held by '
          + 'another app (RealSense Viewer, Teams, …), then Allow camera again.')
      return cams
    } catch (e) {
      setHint(e.name === 'NotAllowedError'
        ? 'Camera permission was denied — allow it in the address-bar icon and retry.'
        : `${e.name}: ${e.message}`)
      return []
    }
  }
  useEffect(() => {
    listDevices()
    const md = navigator.mediaDevices
    if (!md?.addEventListener) return
    const onChange = () => listDevices()
    md.addEventListener('devicechange', onChange)
    return () => md.removeEventListener('devicechange', onChange)
  }, [])

  const stop = () => {
    cancelAnimationFrame(rafRef.current)
    stopRecording()
    streamRef.current?.getTracks().forEach(t => t.stop())
    streamRef.current = null
    if (videoRef.current) videoRef.current.srcObject = null
    inFlightRef.current = false
    setStatus('idle'); setPose(null); poseRef.current = null
  }

  const start = async () => {
    try {
      setStatus('opening camera…')
      const stream = await navigator.mediaDevices.getUserMedia({
        video: deviceId ? { deviceId: { exact: deviceId } } : { facingMode: 'user' },
      })
      streamRef.current = stream
      videoRef.current.srcObject = stream
      await videoRef.current.play()
      await listDevices()

      if (!landmarkerRef.current) {
        setStatus('loading model…')
        const fileset = await FilesetResolver.forVisionTasks(WASM)
        const make = (delegate) => HandLandmarker.createFromOptions(fileset, {
          baseOptions: { modelAssetPath: MODEL, delegate },
          runningMode: 'VIDEO',
          numHands: 1,
        })
        // GPU needs WebGL2; fall back rather than dying on machines without it
        landmarkerRef.current = await make('GPU').catch(() => make('CPU'))
      }
      if (record) startRecording(stream)
      setStatus('tracking')
      loop()
    } catch (e) {
      setStatus(`failed: ${e.message}`)
      stop()
    }
  }

  const loop = () => {
    let last = performance.now(), frames = 0, fpsT = last
    const tick = () => {
      rafRef.current = requestAnimationFrame(tick)
      const v = videoRef.current, lmk = landmarkerRef.current
      if (!v || !lmk || v.readyState < 2) return
      const now = performance.now()
      if (now === last) return
      last = now

      let res
      try { res = lmk.detectForVideo(v, now) } catch { return }
      const marks = res?.landmarks?.[0]
      draw(marks)

      if (++frames >= 15) { setFps(Math.round(frames * 1000 / (now - fpsT))); frames = 0; fpsT = now }
      if (!marks) return

      // worldLandmarks are metric and rotation-normalized — much better for
      // angles than the image-space ones, which are perspective-distorted.
      const src = res.worldLandmarks?.[0] || marks
      const next = landmarksToPose(src, optsRef.current)
      if (!next) return
      poseRef.current = smoothPose(poseRef.current, next, optsRef.current.smooth)
      setPose(poseRef.current)

      // Backpressure: never have more than one command in flight. Firing at a
      // fixed rate regardless of how fast the robot answers queues requests up,
      // and a queue is worse than useless here — the arriving poses are stale,
      // and pressing Stop leaves the backlog still draining into the servos.
      // Dropping the frame instead keeps the robot on the LATEST pose.
      if (drivingRef.current && !inFlightRef.current
          && now - lastSendRef.current > 1000 / SEND_HZ) {
        lastSendRef.current = now
        inFlightRef.current = true
        const t0 = now
        const ok = r => setSendInfo(s => ({
          ...s, n: s.n + 1, err: r.ok ? '' : (r.message || 'rejected'),
          tip: r.tip_error_mm, ms: Math.round(performance.now() - t0),
          solveMs: r.solve_ms, serialHz: r.serial_hz,
        }))
        const bad = e => setSendInfo(s => ({ ...s, err: String(e.message || e) }))
        const done = () => { inFlightRef.current = false }
        if (optsRef.current.retarget) {
          // send raw landmarks; the backend does FK/IK placement matching
          const hand = res.handedness?.[0]?.[0]?.categoryName || 'Right'
          api.trackHand(src.map(p => [p.x, p.y, p.z]), hand, optsRef.current.wApex)
            .then(ok).catch(bad).finally(done)
        } else {
          api.commandPose(poseRef.current).then(ok).catch(bad).finally(done)
        }
      } else if (drivingRef.current && inFlightRef.current) {
        dropRef.current++
      }
    }
    tick()
  }

  const draw = (marks) => {
    const c = canvasRef.current, v = videoRef.current
    if (!c || !v) return
    const w = v.videoWidth || 320, h = v.videoHeight || 240
    if (c.width !== w) { c.width = w; c.height = h }
    const g = c.getContext('2d')
    g.clearRect(0, 0, w, h)
    if (!marks) return
    g.strokeStyle = '#2f81f7'; g.lineWidth = Math.max(2, w / 250)
    for (const [a, b] of BONES) {
      g.beginPath()
      g.moveTo(marks[a].x * w, marks[a].y * h)
      g.lineTo(marks[b].x * w, marks[b].y * h)
      g.stroke()
    }
    g.fillStyle = '#3fb950'
    for (const p of marks) {
      g.beginPath(); g.arc(p.x * w, p.y * h, Math.max(2, w / 200), 0, 7); g.fill()
    }
  }

  useEffect(() => { if (!active) stop() }, [active])
  useEffect(() => () => { stop(); landmarkerRef.current?.close?.() }, [])
  // re-open on camera change while running
  useEffect(() => {
    if (status === 'tracking' && deviceId) { stop(); start() }

  }, [deviceId])

  const running = status === 'tracking'
  return (
    <div>
      <div className="chip-input" style={{ marginBottom: 10 }}>
        <span className="help">Camera</span>
        <select value={deviceId} onChange={e => setDeviceId(e.target.value)}
          style={{ maxWidth: 320 }}>
          {!devices.length && <option value="">(no camera yet)</option>}
          {devices.map((d, i) => (
            <option key={d.deviceId || i} value={d.deviceId}>
              {d.label || `Camera ${i + 1}`}
            </option>
          ))}
        </select>
        <button className="btn" onClick={() => listDevices({ prompt: true })}>
          Allow camera
        </button>
        <button className="btn" onClick={runDiag}>Diagnose</button>
        <label className="help" title="Set this before pressing Start">
          <input type="checkbox" checked={record} disabled={running}
            onChange={e => setRecord(e.target.checked)} /> Record video
        </label>
        {!running
          ? <button className="btn primary" onClick={start}>
              {record ? 'Start tracking + recording' : 'Start tracking'}
            </button>
          : <button className="btn warn" onClick={stop}>
              {recording ? 'Stop & save recording' : 'Stop'}
            </button>}
        <span className="help">{status}{running ? ` · ${fps} fps` : ''}</span>
        {recording &&
          <span className="pill"><span className="dot on" />REC</span>}
      </div>
      {/* The robot's fingers are rolling-contact linkages with 3 DOF where a
          human has 4, so copying joint angles is only an approximation. Matching
          placement (fingertip + the peak of the finger's arc) is the faithful
          option; the angle copy is kept as a fallback. */}
      <div className="chip-input" style={{ marginBottom: 10 }}>
        <span className="help">Mapping</span>
        <button className={`btn ${retarget ? 'primary' : ''}`} onClick={() => setRetarget(true)}>
          Placement (tip + arc peak)
        </button>
        <button className={`btn ${!retarget ? 'primary' : ''}`} onClick={() => setRetarget(false)}>
          Copy joint angles
        </button>
        {retarget && <>
          <span className="help">arc-peak weight</span>
          <input type="range" min={0} max={2} step={0.1} value={wApex} style={{ width: 120 }}
            onChange={e => setWApex(Number(e.target.value))} />
          <span className="val">{wApex.toFixed(1)}</span>
          <span className="help" style={{ opacity: 0.6 }}>0 = fingertip only</span>
        </>}
      </div>
      <div className="chip-input" style={{ marginBottom: 10 }}>
        {/* Landmark jitter -> servo hunting (the buzz). Lower = steadier hand but
            more lag; the backend deadband handles the rest. */}
        <span className="help">Smoothing</span>
        <input type="range" min={0.05} max={1} step={0.05} value={smooth} style={{ width: 160 }}
          onChange={e => setSmooth(Number(e.target.value))} />
        <span className="val">{smooth.toFixed(2)}</span>
        <span className="help" style={{ opacity: 0.6 }}>
          lower = steadier (less jitter), higher = snappier
        </span>
      </div>
      {hint && <div className="help" style={{ color: 'var(--warn)', marginBottom: 10 }}>{hint}</div>}
      {diag && (
        <table style={{ marginBottom: 10 }}>
          <tbody>
            {Object.entries(diag).map(([k, v]) => (
              <tr key={k}>
                <td className="help">{k}</td>
                <td style={{ color: /error|fail|false|empty|^0$/i.test(v) ? 'var(--danger)' : 'inherit' }}>
                  {v}
                </td>
              </tr>
            ))}
          </tbody>
        </table>
      )}
      {devices.length > 1 && (
        <div className="help" style={{ marginBottom: 10 }}>
          A depth camera (RealSense) publishes several video inputs — the RGB one plus
          depth/IR streams. Pick the <b>RGB</b> entry; the IR streams are greyscale and
          MediaPipe tracks them poorly.
        </div>
      )}

      <div className="chip-input" style={{ marginBottom: 10 }}>
        <label className="help"><input type="checkbox" checked={mirror}
          onChange={e => setMirror(e.target.checked)} /> Mirror preview</label>
        <label className="help"><input type="checkbox" checked={invertSpread}
          onChange={e => setInvertSpread(e.target.checked)} /> Invert spread
          <span style={{ opacity: 0.6 }}> (angle-copy mode only)</span></label>
      </div>

      {(record || clips.length > 0) && (
        <div className="help" style={{ marginBottom: 10, padding: '8px 10px',
             border: '1px solid var(--border)', borderRadius: 6 }}>
          <b>Recordings</b> — one clip per Start→Stop. They live in this tab only,
          so click a name to download before closing it.
          {clips.length === 0
            ? <div style={{ marginTop: 4, opacity: 0.7 }}>
                (none yet — a clip appears here after you press Stop)
              </div>
            : <ul style={{ margin: '4px 0 0 16px' }}>
                {clips.map(c => (
                  <li key={c.name}>
                    <a href={c.url} download={c.name}>{c.name}</a>
                    {' '}· {(c.size / 1e6).toFixed(1)} MB
                  </li>
                ))}
              </ul>}
        </div>
      )}

      <div style={{ display: 'flex', gap: 14, flexWrap: 'wrap' }}>
        <div style={{ position: 'relative', width: 360, maxWidth: '100%' }}>
          <video ref={videoRef} playsInline muted
            style={{ width: '100%', borderRadius: 6, background: '#000',
                     transform: mirror ? 'scaleX(-1)' : 'none' }} />
          <canvas ref={canvasRef}
            style={{ position: 'absolute', inset: 0, width: '100%', height: '100%',
                     transform: mirror ? 'scaleX(-1)' : 'none' }} />
        </div>
        <div style={{ flex: 1, minWidth: 260 }}>
          <table>
            <thead><tr><th>Finger</th><th>flex</th><th>spread</th><th>PIP</th></tr></thead>
            <tbody>
              {pose ? Object.entries(pose).map(([f, j]) => (
                <tr key={f}>
                  <td style={{ textTransform: 'capitalize' }}>{f}</td>
                  <td className="num">{(j.mcp_flex ?? j.mcp)?.toFixed(0)}°</td>
                  <td className="num">{j.mcp_spread !== undefined
                    ? `${j.mcp_spread.toFixed(0)}°` : `cmc ${j.cmc?.toFixed(0)}°`}</td>
                  <td className="num">{j.pip?.toFixed(0)}°</td>
                </tr>
              )) : <tr><td colSpan={4} className="help">no hand detected</td></tr>}
            </tbody>
          </table>
          {/* Exactly why the robot is or is not moving — each failure mode used
              to look identical from the outside ("it just doesn't move"). */}
          <div style={{ marginTop: 8 }}>
            {!running && <div className="help">Stopped — press Start tracking.</div>}
            {running && !pose && <div className="help" style={{ color: 'var(--warn)' }}>
              Camera running but <b>no hand detected</b> — hold your hand in frame.
            </div>}
            {running && pose && !driving && <div className="help" style={{ color: 'var(--warn)' }}>
              Tracking, but <b>not driving the robot</b>. {driveHint || 'Enable the servo first.'}
            </div>}
            {running && pose && driving && (
              sendInfo.err
                ? <div className="help" style={{ color: 'var(--danger)' }}>
                    Send failed: {sendInfo.err}
                  </div>
                : <div className="help" style={{ color: 'var(--ok)' }}>
                    Driving · {sendInfo.n} sent
                    {/* three separate costs — solve, network, and how fast the
                        serial loop can actually push it to the servos */}
                    {sendInfo.ms != null && <> · round trip <b>{sendInfo.ms} ms</b></>}
                    {sendInfo.solveMs != null && <> (solve {sendInfo.solveMs} ms)</>}
                    {sendInfo.serialHz != null && <> · serial loop <b>{sendInfo.serialHz} Hz</b>
                      {sendInfo.serialHz < SEND_HZ &&
                        <span style={{ color: 'var(--warn)' }}> — slower than the
                          {' '}{SEND_HZ} Hz command rate, this is the bottleneck</span>}</>}
                    {sendInfo.tip && <> · tip error {Object.entries(sendInfo.tip)
                      .map(([f, v]) => `${f.slice(0, 2)} ${v}`).join(', ')} mm</>}
                  </div>
            )}
          </div>
        </div>
      </div>
    </div>
  )
}
