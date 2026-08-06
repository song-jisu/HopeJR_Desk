"""Posable HopeJR hand mesh model from the URDF, for depth fitting."""
import math, os, xml.etree.ElementTree as ET
import numpy as np, trimesh

BASE = "/mnt/c/Users/user/Documents/HopeJR_Desk/ros2_ws/src/hopejr_hand_description"
URDF = BASE + "/urdf/hopejr_hand.urdf"
MDIR = BASE + "/meshes/collision/"


def _f3(s): return np.array([float(x) for x in (s or "0 0 0").split()], float)
def _rot(ax, t):
    ax = np.asarray(ax, float); n = np.linalg.norm(ax)
    if n < 1e-9 or abs(t) < 1e-15: return np.eye(3)
    ax = ax/n; K = np.array([[0,-ax[2],ax[1]],[ax[2],0,-ax[0]],[-ax[1],ax[0],0]])
    return np.eye(3)+math.sin(t)*K+(1-math.cos(t))*K@K
def _rpy(r): return _rot([0,0,1],r[2])@_rot([0,1,0],r[1])@_rot([1,0,0],r[0])
def _T(xyz, rpy, R2=None):
    R = _rpy(rpy); R = R@R2 if R2 is not None else R
    M = np.eye(4); M[:3,:3]=R; M[:3,3]=xyz; return M


class HandModel:
    def __init__(self):
        root = ET.parse(URDF).getroot()
        self.joints = []
        for j in root.findall('joint'):
            o = j.find('origin'); ax = j.find('axis'); m = j.find('mimic')
            self.joints.append(dict(
                name=j.get('name'), type=j.get('type'),
                parent=j.find('parent').get('link'), child=j.find('child').get('link'),
                xyz=_f3(o.get('xyz') if o is not None else None),
                rpy=_f3(o.get('rpy') if o is not None else None),
                axis=_f3(ax.get('xyz')) if ax is not None else np.array([0,0,1.]),
                mimic=(m.get('joint') if m is not None else None),
                mult=float(m.get('multiplier', 1)) if m is not None else 1.0,
                offset=float(m.get('offset', 0)) if m is not None else 0.0))
        self.child_joint = {j['child']: j for j in self.joints}
        # meshed links (scale 0.1 uniform)
        self.meshes = {}
        for L in root.findall('link'):
            g = L.find('collision/geometry/mesh')
            if g is None: continue
            f = MDIR + g.get('filename').split('/')[-1]
            if os.path.exists(f):
                mesh = trimesh.load(f, force='mesh'); mesh.apply_scale(0.0001)  # STL is mm, URDF origins are m -> to metres
                self.meshes[L.get('name')] = mesh
        self.root = (set(j['parent'] for j in self.joints)
                     - set(j['child'] for j in self.joints)).pop()

    def _val(self, jvals, j):
        if j['type'] == 'fixed': return 0.0
        if j['mimic']: return jvals.get(j['mimic'], 0.0)*j['mult']+j['offset']
        return jvals.get(j['name'], 0.0)

    def link_tf(self, jvals):
        """World transform of every link."""
        tf = {self.root: np.eye(4)}
        # links are ordered parent-before-child in this URDF; iterate to fixpoint
        pend = list(self.joints)
        while pend:
            nxt = []
            for j in pend:
                if j['parent'] in tf:
                    R2 = _rot(j['axis'], self._val(jvals, j)) if j['type'] != 'fixed' else None
                    tf[j['child']] = tf[j['parent']] @ _T(j['xyz'], j['rpy'], R2)
                else:
                    nxt.append(j)
            if len(nxt) == len(pend): break
            pend = nxt
        return tf

    def vertices(self, jvals, links=None):
        """Transformed mesh VERTICES (deterministic — for optimization, where
        random surface sampling would make the numeric Jacobian pure noise)."""
        tf = self.link_tf(jvals)
        want = links or list(self.meshes)
        out = []
        for k in want:
            if k not in tf: continue
            v = np.asarray(self.meshes[k].vertices)
            T = tf[k]; out.append((T[:3,:3]@v.T).T + T[:3,3])
        return np.vstack(out)

    def sample(self, jvals, n=6000, links=None):
        """Point cloud sampled on the posed hand surface (metres, hand frame)."""
        tf = self.link_tf(jvals)
        want = links or list(self.meshes)
        areas = {k: self.meshes[k].area for k in want if k in tf}
        tot = sum(areas.values()); out = []
        for k, a in areas.items():
            m = int(max(30, n*a/tot))
            p, _ = trimesh.sample.sample_surface(self.meshes[k], m)
            T = tf[k]; out.append((T[:3,:3]@p.T).T + T[:3,3])
        return np.vstack(out)


# --- motor angles -> free URDF joint values ---------------------------------
def free_joints_from_angles(fingers, thumb, spread_sign=1.0):
    """fingers[f] = (mcp_flex_deg, mcp_spread_deg, pip_deg); thumb = (cmc,mcp,pip,dip) deg.
    Support joint = total/2 (support+mimic). thumb_mcp_2 (cmc) is a lone free joint."""
    d = math.radians; jv = {"wrist_pitch": 0.0}
    for f, (a, b, p) in fingers.items():
        jv[f"{f}_mcp_2_support"] = d(a)/2
        jv[f"{f}_pip_support"] = spread_sign*d(b)/2
        jv[f"{f}_dip_1_support"] = d(p)/2
    c, m, p, dp = thumb
    jv["thumb_mcp_2"] = d(c)
    jv["thumb_pip_1_support"] = d(m)/2
    jv["thumb_pip_2_support"] = d(p)/2
    jv["thumb_dip_support"] = d(dp)/2
    return jv
