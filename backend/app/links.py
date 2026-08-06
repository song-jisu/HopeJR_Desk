"""Link inertial parameters (PLAN.md §8 Link Parameter Management).

Parses per-link mass + centre-of-mass from the URDF as defaults, and holds
user overrides (persisted). Used by the 3D viewer to compute per-joint gravity
torque for gravity-compensated hand-guiding.
"""
from __future__ import annotations

import json
import os
import xml.etree.ElementTree as ET

_URDF = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "..", "..", "ros2_ws", "src",
    "hopejr_right_arm_description", "urdf", "hopejr_right_arm.urdf"))
_OVERRIDES = os.environ.get(
    "HOPEJR_LINKS_FILE",
    os.path.join(os.path.dirname(os.path.dirname(__file__)), "link_params.json"))


def _parse_urdf() -> dict:
    """{link: {mass, com:[x,y,z]}} from the URDF <inertial> blocks."""
    out: dict[str, dict] = {}
    try:
        root = ET.parse(_URDF).getroot()
    except Exception:
        return out
    for link in root.findall("link"):
        name = link.get("name")
        inertial = link.find("inertial")
        if name is None or inertial is None:
            continue
        m = inertial.find("mass")
        o = inertial.find("origin")
        mass = float(m.get("value", 0.0)) if m is not None else 0.0
        com = [0.0, 0.0, 0.0]
        if o is not None and o.get("xyz"):
            try:
                com = [float(x) for x in o.get("xyz").split()][:3]
            except Exception:
                pass
        out[name] = {"mass": round(mass, 6), "com": com}
    return out


def _load_overrides() -> dict:
    try:
        with open(_OVERRIDES) as f:
            return json.load(f)
    except Exception:
        return {}


def _save_overrides(ov: dict) -> None:
    with open(_OVERRIDES, "w") as f:
        json.dump(ov, f, indent=2)


def get_links() -> dict:
    """Defaults from URDF merged with user overrides."""
    base = _parse_urdf()
    ov = _load_overrides()
    for name, vals in ov.items():
        if name in base:
            base[name] = {**base[name], **vals}
        else:
            base[name] = vals
    return base


def set_link(name: str, mass=None, com=None) -> dict:
    ov = _load_overrides()
    entry = ov.get(name, {})
    if mass is not None:
        entry["mass"] = float(mass)
    if com is not None:
        entry["com"] = [float(x) for x in com][:3]
    ov[name] = entry
    _save_overrides(ov)
    return get_links()


def reset_links() -> dict:
    if os.path.exists(_OVERRIDES):
        os.remove(_OVERRIDES)
    return get_links()
