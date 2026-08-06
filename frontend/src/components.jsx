import React from 'react'

export function Card({ title, children, style }) {
  return (
    <div className="card" style={style}>
      {title && <h3>{title}</h3>}
      {children}
    </div>
  )
}

export function Stat({ label, value, unit }) {
  return (
    <div className="stat">
      <div className="value">{value}<span className="unit"> {unit}</span></div>
      <div className="label">{label}</div>
    </div>
  )
}

export function Bar({ value, max = 100, kind }) {
  const pct = Math.max(0, Math.min(100, (value / max) * 100))
  return <div className={`bar ${kind || ''}`}><span style={{ width: `${pct}%` }} /></div>
}

export function Dot({ state }) {
  // state: true(on)/false(off)/'warn'
  const cls = state === 'warn' ? 'warn' : state ? 'on' : 'off'
  return <span className={`dot ${cls}`} />
}

export function Pill({ on, children }) {
  return <span className="pill"><Dot state={on} />{children}</span>
}
