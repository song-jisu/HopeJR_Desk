import React, { useEffect, useRef, useState } from 'react'
import { Card } from '../components.jsx'
import { api } from '../api.js'

export default function Logs() {
  const [logs, setLogs] = useState([])
  const timer = useRef()

  useEffect(() => {
    const tick = () => api.getLogs(200).then(d => setLogs(d.logs)).catch(() => {})
    tick(); timer.current = setInterval(tick, 1500)
    return () => clearInterval(timer.current)
  }, [])

  const fmt = (ts) => new Date(ts * 1000).toLocaleTimeString()

  return (
    <Card title="Logs (PLAN §11)">
      <div style={{ maxHeight: '70vh', overflow: 'auto' }}>
        {logs.slice().reverse().map((l, i) => (
          <div className="log-line" key={i}>
            <span className={`lvl ${l.level}`}>{l.level.toUpperCase()}</span>
            <span className="help">{fmt(l.ts)} </span>
            {l.message}
          </div>
        ))}
        {!logs.length && <div className="help">No logs yet.</div>}
      </div>
    </Card>
  )
}
