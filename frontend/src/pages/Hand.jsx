import React, { useState } from 'react'
import { Card, Bar } from '../components.jsx'
import { api } from '../api.js'
import CameraHand from '../CameraHand.jsx'
import { FINGER_MOTORS } from './HandSetup.jsx'

function MotorSliders({ snap, disabled, reason }) {
  const motors = snap?.motors || []
  const fingers = snap?.fingers || []
  const byName = Object.fromEntries(motors.map(m => [m.name, m]))
  const enabled = snap?.status?.servo_enabled && !snap?.status?.estop && !disabled

  return (
    <Card title="Hand Motors — 16 individual servos" style={{ marginBottom: 14 }}>
      {disabled && <div className="help" style={{ marginBottom: 10 }}>{reason}</div>}
      <div className="grid cols-3" style={{ gap: 12 }}>
        {FINGER_MOTORS.map(([f, names], i) => {
          const st = fingers.find(x => x.name === f)
          return (
            <div key={f} className="finger" style={{ alignItems: 'stretch' }}>
              <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
                <span className="name" style={{ textTransform: 'capitalize' }}>{i + 1}. {f}</span>
                {st && <span className={`badge ${st.state}`}>{st.state}</span>}
              </div>
              {names.map(n => {
                const m = byName[n]
                if (!m) return null
                const short = n.replace(`${f}_`, '').replace('_flexor', '')
                return (
                  <div key={n} style={{ marginTop: 8 }}>
                    <div className="help" style={{ display: 'flex', justifyContent: 'space-between' }}>
                      <span>{short} <span style={{ opacity: 0.6 }}>#{m.servo_id}</span></span>
                      <span className="val">{m.position.toFixed(0)}%</span>
                    </div>
                    <Bar value={m.position} kind={st?.contact ? 'warn' : ''} />
                    <input type="range" min={0} max={100} value={Math.round(m.command)}
                      disabled={!enabled}
                      onChange={e => api.commandHandMotor(n, Number(e.target.value))} />
                  </div>
                )
              })}
            </div>
          )
        })}
      </div>
      {!enabled && <div className="help" style={{ marginTop: 10 }}>
        Enable servo (Robot Control) to move motors.
      </div>}
    </Card>
  )
}

export default function Hand({ snap }) {
  const [thr, setThr] = useState(150)
  const [mode, setMode] = useState('slider')          // 'slider' | 'camera'
  const servoOn = snap?.status?.servo_enabled && !snap?.status?.estop

  return (
    <>
      <Card title="Hand Control Mode" style={{ marginBottom: 14 }}>
        <div className="btn-row">
          {[['slider', 'Sliders — drive each servo directly'],
            ['camera', 'Camera — copy a real hand (MediaPipe)']].map(([m, label]) => (
            <button key={m} className={`btn ${mode === m ? 'primary' : ''}`}
              onClick={() => setMode(m)}>{label}</button>
          ))}
        </div>
        {mode === 'camera' && (
          <div style={{ marginTop: 12 }}>
            {/* Landmarks -> joint angles happens in the browser; the backend runs
                the same joint -> motor model the display uses, so the 3D hand and
                the real hand get the identical pose. Set up the hand model
                (home pose, tuning, dead zones, pinch) on the Calibration page. */}
            <CameraHand active={mode === 'camera'} driving={!!servoOn}
              driveHint="Enable the servo on the Robot Control page." />
          </div>
        )}
      </Card>

      <MotorSliders snap={snap} disabled={mode === 'camera'}
        reason="Camera mode is driving the hand — switch to Sliders to move servos by hand." />

      <Card title="Contact Detection Threshold">
        <div className="chip-input">
          <span className="help">Servo current threshold (contact when finger current exceeds this):</span>
          <input type="range" min={0} max={600} value={thr} style={{ width: 220 }}
            onChange={e => setThr(Number(e.target.value))} />
          <span className="val">{thr} mA</span>
          <button className="btn" onClick={() => api.contactThreshold(thr)}>Apply</button>
        </div>
      </Card>
    </>
  )
}
