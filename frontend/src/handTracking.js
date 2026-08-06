// MediaPipe hand landmarks -> HopeJR joint angles (degrees).
//
// The output keys match the backend's tendon model exactly, so the result can
// go straight to POST /api/hand/pose (drive) or be compared against what
// /api/hand/kinematics reports (verify).
//
// MediaPipe gives 21 landmarks:
//   0 wrist
//   1..4  thumb   CMC, MCP, IP, TIP
//   5..8  index   MCP, PIP, DIP, TIP
//   9..12 middle
//  13..16 ring
//  17..20 pinky

const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1],
                         a[2] * b[0] - a[0] * b[2],
                         a[0] * b[1] - a[1] * b[0]]
const len = (a) => Math.hypot(a[0], a[1], a[2])
const unit = (a) => { const n = len(a); return n > 1e-9 ? [a[0] / n, a[1] / n, a[2] / n] : [0, 0, 0] }
const DEG = 180 / Math.PI

const at = (lm, i) => [lm[i].x, lm[i].y, lm[i].z]

/** Bend angle at b, in degrees. 0 = the three points are collinear (straight). */
function bend(lm, a, b, c) {
  const u = unit(sub(at(lm, b), at(lm, a)))
  const v = unit(sub(at(lm, c), at(lm, b)))
  return Math.acos(Math.max(-1, Math.min(1, dot(u, v)))) * DEG
}

/** Signed in-plane deviation of the proximal phalanx from its metacarpal. */
function spread(lm, mcp, pip, palmN) {
  const meta = unit(sub(at(lm, mcp), at(lm, 0)))
  const prox = unit(sub(at(lm, pip), at(lm, mcp)))
  // drop the out-of-plane (flexion) part, then take the signed angle about the
  // palm normal — flexion must not leak into the spread reading
  const flat = (v) => unit(sub(v, palmN.map(c => c * dot(v, palmN))))
  const a = flat(meta), b = flat(prox)
  const ang = Math.acos(Math.max(-1, Math.min(1, dot(a, b)))) * DEG
  return dot(cross(a, b), palmN) < 0 ? -ang : ang
}

const FINGERS = {
  index: [5, 6, 7], middle: [9, 10, 11], ring: [13, 14, 15], pinky: [17, 18, 19],
}

// The thumb CMC is a direct linkage whose joint value IS the servo angle
// (-172°..-3°). We only get the thumb's abduction from the camera, so map the
// measured thumb-to-index metacarpal angle across that range: tucked in -> -172,
// spread out -> -3. Rough by construction — tune with CMC_SPAN if the thumb
// over/under-travels.
const CMC_SPAN = [10, 60]      // measured degrees that map to the full range
const CMC_RANGE = [-172, -3]

/**
 * @param landmarks  21 MediaPipe landmarks (worldLandmarks preferred — metric)
 * @param opts.invertSpread  flip the abduction sign (camera/handedness mirror)
 * @returns {thumb:{cmc,mcp,pip,dip}, index:{mcp_flex,mcp_spread,pip}, ...}
 */
export function landmarksToPose(landmarks, opts = {}) {
  if (!landmarks || landmarks.length < 21) return null
  const lm = landmarks
  const s = opts.invertSpread ? -1 : 1

  // Palm frame: normal from the two edge metacarpals, so spread is measured in
  // the plane of the palm regardless of how the hand is held.
  const palmN = unit(cross(sub(at(lm, 5), at(lm, 0)), sub(at(lm, 17), at(lm, 0))))

  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v))
  const pose = {}

  for (const [name, [mcp, pip, dip]] of Object.entries(FINGERS)) {
    pose[name] = {
      // flexion as the bend angle at the joint — unsigned, and immune to the
      // palm-normal sign ambiguity that trips up a projected measurement
      mcp_flex: clamp(bend(lm, 0, mcp, pip), 0, 90),
      mcp_spread: clamp(s * spread(lm, mcp, pip, palmN), -45, 45),
      pip: clamp(bend(lm, mcp, pip, dip), 0, 68),
    }
  }

  // thumb: only 3 segments, so PIP and DIP share the distal bend
  const thumbAbd = Math.acos(Math.max(-1, Math.min(1,
    dot(unit(sub(at(lm, 2), at(lm, 1))), unit(sub(at(lm, 5), at(lm, 0))))))) * DEG
  const t = clamp((thumbAbd - CMC_SPAN[0]) / (CMC_SPAN[1] - CMC_SPAN[0]), 0, 1)
  const distal = clamp(bend(lm, 2, 3, 4), 0, 56)
  pose.thumb = {
    cmc: CMC_RANGE[0] + t * (CMC_RANGE[1] - CMC_RANGE[0]),
    mcp: clamp(bend(lm, 1, 2, 3), 0, 56),
    pip: distal,
    dip: distal,
  }
  return pose
}

/** Exponential smoothing so servo commands don't chatter on landmark jitter. */
export function smoothPose(prev, next, alpha = 0.4) {
  if (!prev) return next
  const out = {}
  for (const f in next) {
    out[f] = {}
    for (const k in next[f]) {
      const p = prev[f]?.[k]
      out[f][k] = p == null ? next[f][k] : p + alpha * (next[f][k] - p)
    }
  }
  return out
}
