import React, { useEffect, useRef, useState } from 'react'
import { connectTelemetry } from './api.js'
import { Pill } from './components.jsx'
import RobotViewer from './RobotViewer.jsx'
import Dashboard from './pages/Dashboard.jsx'
import Control from './pages/Control.jsx'
import Hand from './pages/Hand.jsx'
import Config from './pages/Config.jsx'
import Calibration from './pages/Calibration.jsx'
import Teaching from './pages/Teaching.jsx'
import Logs from './pages/Logs.jsx'

const NAV = [
  { id: 'dashboard', icon: '▤', label: 'Dashboard' },
  { id: 'control', icon: '⛭', label: 'Robot Control' },
  { id: 'hand', icon: '✋', label: 'Hand Control' },
  { id: 'teaching', icon: '🖐', label: 'Teaching' },
  { id: 'calibration', icon: '📏', label: 'Calibration' },
  { id: 'config', icon: '⚙', label: 'Configuration' },
  { id: 'logs', icon: '☰', label: 'Logs' },
]

const SUBTITLE = {
  dashboard: 'Real-time robot state monitoring',
  control: 'Servo, homing, jog & emergency stop',
  hand: 'Finger control, grip patterns & contact detection',
  teaching: 'Hand-guiding: move the arm by hand, record & replay',
  calibration: 'Range-finder: backdrive joints to record min/max',
  config: 'Per-servo limits and home offsets',
  logs: 'Event & error history',
}

export default function App() {
  const [page, setPage] = useState('dashboard')
  const [snap, setSnap] = useState(null)
  const [link, setLink] = useState('connecting')
  const snapRef = useRef(null)   // latest snapshot for the 3D viewer (no re-render)

  useEffect(() => connectTelemetry((s) => { snapRef.current = s; setSnap(s) }, setLink), [])

  const st = snap?.status
  return (
    <div className="app">
      <div className="topbar">
        <div className="brand">Hope<span>JR</span> Desk</div>
        <div className="spacer" />
        <Pill on={link === 'connected'}>{link === 'connected' ? 'Backend live' : 'Reconnecting'}</Pill>
        <Pill on={st?.connected}>{st?.connected ? 'Robot' : 'No robot'}</Pill>
        <Pill on={st?.servo_enabled}>Servo {st?.servo_enabled ? 'ON' : 'OFF'}</Pill>
        <Pill on={st?.estop ? 'warn' : true}>{st?.estop ? 'E-STOP' : 'Ready'}</Pill>
        <span className="pill">{st?.mode || '—'}</span>
      </div>

      <div className="sidebar">
        {NAV.map(n => (
          <button key={n.id} className={`nav-item ${page === n.id ? 'active' : ''}`} onClick={() => setPage(n.id)}>
            <span className="nav-icon">{n.icon}</span>{n.label}
          </button>
        ))}
      </div>

      <div className="content">
        <h1 className="page-title">{NAV.find(n => n.id === page).label}</h1>
        <p className="page-sub">{SUBTITLE[page]}</p>
        {page === 'dashboard' && <Dashboard snap={snap} />}
        {page === 'control' && <Control snap={snap} />}
        {page === 'hand' && <Hand snap={snap} />}
        {page === 'teaching' && <Teaching />}
        {page === 'calibration' && <Calibration snap={snap} />}
        {page === 'config' && <Config />}
        {page === 'logs' && <Logs />}
      </div>

      <div className="rightpanel">
        <div className="rp-title">Live 3D · {st?.mode || '—'}</div>
        <div className="rp-viewer"><RobotViewer snapRef={snapRef} /></div>
        <div className="rp-foot help">
          Arm joints follow telemetry (normalized → URDF limits). Drag to orbit · scroll to zoom.
        </div>
      </div>
    </div>
  )
}
