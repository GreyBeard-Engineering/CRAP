"""Turn a built gearbox into a JSON payload for the three.js viewer."""
import base64
import numpy as np
from build123d import Location, Part
from part_generation_fcns.base_step import base_mounting_holes
from part_generation_fcns.envelope import poser
from part_generation_fcns.gearbox import gearbox

# main custom parts get their own color; everything else is keyed by name
_COLORS = {
    "base": "#4f7cac",
    "top": "#5e9c76",
    "top_cap": "#7fbf95",
    "cycloid": "#d98c3f",
    "cycloid_cap": "#e8b070",
    "eccentric_insert": "#c0504d",
    "eccentric_insert_stacked": "#c0504d",
    "output": "#8064a2",
    "output_cap": "#a58bc4",
}
_HARDWARE = (("bolt", "#3a3d42"), ("pin", "#6b6f75"), ("spacer", "#9aa0a6"),
             ("bearing", "#c8ccd2"), ("roller", "#c8ccd2"))


def _color(name: str) -> str:
    if name in _COLORS:
        return _COLORS[name]
    for key, color in _HARDWARE:
        if key in name:
            return color
    return "#999999"


def _b64(a: np.ndarray) -> str:
    return base64.b64encode(np.ascontiguousarray(a).tobytes()).decode("ascii")


def tessellate(part: Part, tol: float = 0.05) -> dict:
    verts, tris = part.tessellate(tol, angular_tolerance=0.2)
    v = np.array([(p.X, p.Y, p.Z) for p in verts], dtype=np.float32)
    t = np.array(tris, dtype=np.uint32)
    return {"verts": _b64(v), "tris": _b64(t), "n_verts": len(v), "n_tris": len(t)}


def _part_name(label: str, names) -> str:
    return label if label in names else label.rsplit("_", 1)[0]


def collision_meshes(gb: gearbox, collisions, limit: int = 12) -> list[dict]:
    """The overlapping volume of the worst collision of each kind of part pair, tessellated in
    assembly coordinates with the input turned to its angle. These are exact booleans, so
    they're only drawn for a few representative hits."""
    posed, names, seen = poser(gb), set(gb.parts), set()
    out = []
    for a, b, depth, theta in collisions:      # worst first
        kind = tuple(sorted((_part_name(a, names), _part_name(b, names))))
        if kind in seen:
            continue
        seen.add(kind)
        if len(seen) > limit:
            break
        try:
            common = posed(a, theta) & posed(b, theta)
        except Exception:
            common = None
        if common is None or common.volume <= 0:
            continue
        out.append({"a": a, "b": b, "depth": depth, "theta": theta, **tessellate(common, 0.02)})
    return out


def matrix(loc: Location) -> list[float]:
    """4x4 transform, column-major (three.js Matrix4.fromArray order)."""
    tr = loc.wrapped.Transformation()
    m = [[tr.Value(r, c) for c in range(1, 5)] for r in range(1, 4)] + [[0.0, 0.0, 0.0, 1.0]]
    return [m[r][c] for c in range(4) for r in range(4)]


def gearbox_payload(gb: gearbox) -> dict:
    groups, kinds, stock = gb.motion_groups(), gb.part_kinds(), gb.stock_types()
    parts, instances = {}, {}
    for name, part in gb.parts.items():
        parts[name] = {**tessellate(part), "group": groups[name], "color": _color(name),
                       "kind": kinds[name], "stock": stock[name]}
    for name, locs in gb.locations().items():
        instances[name] = [matrix(l) for l in locs]
    z = gb.stack()
    return {
        "parts": parts,
        "instances": instances,
        "motion": {
            "e": gb.cycloid.e,
            "ratio": gb.cycloid.ratio,
            "num_disks": gb.num_disks,
            # the assembly is posed at input angle 0, with the cycloid offset by +e along X
            "cycloid_center": [gb.cycloid.e, 0.0],
            "height": z["top_cap"] + gb.top.cap.flange.thickness,
            "radius": base_mounting_holes(gb.base).R + gb.base.mounting_holes.r + gb.base.mounting_hole_offset,
        },
    }
