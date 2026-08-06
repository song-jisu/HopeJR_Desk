import React, { useEffect, useState } from 'react'
import { Card } from '../components.jsx'
import { api } from '../api.js'

export default function Config() {
  const [rows, setRows] = useState([])
  const [links, setLinks] = useState({})
  const [msg, setMsg] = useState('')

  const load = () => api.getConfig().then(d => setRows(d.motors))
  const loadLinks = () => api.getLinks().then(d => setLinks(d.links || {}))
  useEffect(() => { load(); loadLinks() }, [])

  const edit = (name, key, val) =>
    setRows(rs => rs.map(r => r.name === name ? { ...r, [key]: val } : r))

  const note = (m) => { setMsg(m); setTimeout(() => setMsg(''), 1500) }
  const save = async (r) => {
    await api.updateConfig({
      name: r.name, cmd_min: Number(r.cmd_min), cmd_max: Number(r.cmd_max), home: Number(r.home),
    })
    note(`saved ${r.name}`)
  }
  const editLink = (name, val) => setLinks(l => ({ ...l, [name]: { ...l[name], mass: val } }))
  const saveLink = async (name) => { await api.setLink(name, Number(links[name].mass), links[name].com); note(`saved ${name}`) }
  const resetLinks = async () => { await api.resetLinks(); loadLinks(); note('links reset to URDF') }

  return (
    <>
      {msg && <div className="help" style={{ color: 'var(--ok)', marginBottom: 8 }}>{msg}</div>}
    <Card title="Robot Configuration — Per-Servo Limits & Home (PLAN §8)">
      <div className="help" style={{ marginBottom: 10 }}>
        Command range and home offset per servo. Applied live to command clamping & homing.
        {msg && <b style={{ color: 'var(--ok)', marginLeft: 8 }}>{msg}</b>}
      </div>
      <table>
        <thead><tr><th>Motor</th><th>ID</th><th>Bus</th><th>Model</th><th>cmd_min</th><th>cmd_max</th><th>home</th><th></th></tr></thead>
        <tbody>
          {rows.map(r => (
            <tr key={r.name}>
              <td>{r.name}</td>
              <td className="num">{r.servo_id}</td>
              <td>{r.unit}</td>
              <td>{r.model}</td>
              {['cmd_min', 'cmd_max', 'home'].map(k => (
                <td key={k}><input style={{ width: 70 }} className="chip-input" value={r[k]}
                  onChange={e => edit(r.name, k, e.target.value)} /></td>
              ))}
              <td><button className="btn" onClick={() => save(r)}>Save</button></td>
            </tr>
          ))}
        </tbody>
      </table>
    </Card>

    <Card title="Link Parameters — Mass & CoM (PLAN §8)" style={{ marginTop: 14 }}>
      <div className="help" style={{ marginBottom: 10 }}>
        Per-link mass (kg) and centre of mass, from the URDF (editable). Used by the 3D viewer to compute
        per-joint gravity torque (τg) for gravity-compensated hand-guiding.
        <button className="btn" style={{ marginLeft: 10 }} onClick={resetLinks}>Reset to URDF</button>
      </div>
      <table>
        <thead><tr><th>Link</th><th>Mass (kg)</th><th>CoM (x, y, z) m</th><th></th></tr></thead>
        <tbody>
          {Object.entries(links).filter(([, v]) => (v.mass || 0) > 0).map(([name, v]) => (
            <tr key={name}>
              <td>{name}</td>
              <td><input style={{ width: 90 }} className="chip-input" value={v.mass}
                onChange={e => editLink(name, e.target.value)} /></td>
              <td className="num help">{(v.com || [0, 0, 0]).map(x => x.toFixed(3)).join(', ')}</td>
              <td><button className="btn" onClick={() => saveLink(name)}>Save</button></td>
            </tr>
          ))}
        </tbody>
      </table>
    </Card>
    </>
  )
}
