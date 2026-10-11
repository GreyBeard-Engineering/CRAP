"""Fast clearance check on simple part envelopes.

Every part is described, in its own frame, by Z-aligned primitives:
  cylinders  (x, y, r, r_in, z0, z1)   solid (r_in = 0) or annular material
  voids      (x, y, r, z0, z1)          round holes cut through that material
  prisms     (polygon, z0, z1)          extruded outlines that aren't circles (the cycloid)

Cylinder pairs are tested exactly (centre distance against radii). A prism is tested as points:
its outline and the outlines of its part's holes, sampled every ~0.25 mm, against the other
part's material, plus one interior point of each of the other part's cylinders against the
prism's material (that catches a cylinder buried entirely inside it).

The check runs over a sweep of input angles, posing each rigid body the way the GUI animates it,
so it covers the whole motion rather than just the assembled pose."""
import math
from dataclasses import dataclass, field

import numpy as np
from build123d import Location

EPS = 1e-3          # contact tolerance: touching (screw in a tight hole, roller on the profile) is fine
SPACING = 0.25      # outline sampling for the point tests (mm)


@dataclass
class Envelope:
    cyls: list = field(default_factory=list)      # (x, y, r, r_in, z0, z1)
    voids: list = field(default_factory=list)     # (x, y, r, z0, z1)
    prisms: list = field(default_factory=list)    # (polygon (n, 2), z0, z1)


def contains(env: Envelope, p) -> np.ndarray:
    """Which local-frame points (n, 3) lie in the envelope's material (for testing envelopes
    against the CAD they describe)."""
    p = np.asarray(p, float)
    xy, z = p[:, :2], p[:, 2]
    solid = np.zeros(len(p), bool)
    for x, y, r, rin, z0, z1 in env.cyls:
        d = np.hypot(xy[:, 0] - x, xy[:, 1] - y)
        solid |= (z > z0) & (z < z1) & (d < r) & (d > rin)
    for poly, z0, z1 in env.prisms:
        solid |= (z > z0) & (z < z1) & _Poly(poly).inside(xy)
    for x, y, r, z0, z1 in env.voids:
        solid &= ~((z > z0) & (z < z1) & (np.hypot(xy[:, 0] - x, xy[:, 1] - y) < r))
    return solid


def cyl(r: float, z0: float, z1: float, r_in: float = 0.0, x: float = 0.0, y: float = 0.0) -> tuple:
    return (x, y, r, max(0.0, r_in), min(z0, z1), max(z0, z1))


def void(r: float, z0: float, z1: float, x: float = 0.0, y: float = 0.0) -> tuple:
    return (x, y, r, min(z0, z1), max(z0, z1))


def pattern_voids(h, z0: float, z1: float, r: float | None = None) -> list:
    """Voids for a patterned_hole (same placement as PolarLocations(h.R, h.N, h.start_angle))."""
    out = []
    for k in range(h.N):
        a = math.radians(h.start_angle + 360.0 * k / h.N)
        out.append(void(h.r if r is None else r, z0, z1, h.R * math.cos(a), h.R * math.sin(a)))
    return out


# ---------- geometry helpers ----------

def _rot(a):
    """Rotation matrices for an array of angles: (..., 2, 2)."""
    c, s = np.cos(a), np.sin(a)
    return np.stack([np.stack([c, -s], -1), np.stack([s, c], -1)], -2)


def _circle_points(x, y, r):
    n = max(12, int(math.ceil(2 * math.pi * r / SPACING)))
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    return np.column_stack([x + r * np.cos(a), y + r * np.sin(a)])


def _resample(poly):
    """Closed polyline with no gap longer than SPACING."""
    q = np.vstack([poly, poly[:1]])
    out = []
    for p0, p1 in zip(q[:-1], q[1:]):
        n = max(1, int(math.ceil(np.hypot(*(p1 - p0)) / SPACING)))
        t = np.arange(n)[:, None] / n
        out.append(p0 + t * (p1 - p0))
    return np.vstack(out)


class _Poly:
    """Point-in-polygon for one prism outline (local frame). Star-shaped outlines (the cycloid
    about its centre) use a radius-by-angle lookup; anything else uses even-odd crossings."""

    def __init__(self, poly):
        self.poly = np.asarray(poly, float)
        a = np.unwrap(np.arctan2(self.poly[:, 1], self.poly[:, 0]))
        d = np.diff(a)
        self.star = (np.all(d > 0) or np.all(d < 0)) and abs(abs(a[-1] - a[0]) - 2 * np.pi) < 0.1
        if self.star:
            self.ang = np.arctan2(self.poly[:, 1], self.poly[:, 0])
            self.rad = np.hypot(self.poly[:, 0], self.poly[:, 1])

    def inside(self, p):
        if len(p) == 0:
            return np.zeros(0, bool)
        if self.star:
            r = np.interp(np.arctan2(p[:, 1], p[:, 0]), self.ang, self.rad, period=2 * np.pi)
            return np.hypot(p[:, 0], p[:, 1]) < r - EPS
        x, y = p[:, :1], p[:, 1:]
        x1, y1 = self.poly[:, 0], self.poly[:, 1]
        x2, y2 = np.roll(x1, -1), np.roll(y1, -1)
        cross = (y1 > y) != (y2 > y)
        with np.errstate(divide="ignore", invalid="ignore"):
            xi = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
        return ((cross & (x < xi)).sum(1) % 2) == 1


# ---------- placed instances ----------

@dataclass
class _Inst:
    name: str
    label: str
    body: int
    A: np.ndarray       # 2x2 (rotation, or a reflection for a flipped part)
    t: np.ndarray       # 2
    sz: float           # +1, or -1 for a part flipped upside down
    tz: float
    env: Envelope
    zlo: float = 0.0
    zhi: float = 0.0
    polys: list = field(default_factory=list)    # (_Poly, z0, z1), local xy, world z

    def xy(self, p):
        return np.asarray(p, float) @ self.A.T + self.t

    def z(self, z0, z1):
        a, b = self.sz * z0 + self.tz, self.sz * z1 + self.tz
        return min(a, b), max(a, b)


def _placement(loc: Location):
    tr = loc.wrapped.Transformation()
    m = np.array([[tr.Value(r, c) for c in range(1, 5)] for r in range(1, 4)])
    return m[:2, :2], m[:2, 3], float(np.sign(m[2, 2]) or 1.0), m[2, 3]


BODIES_FIXED = ("housing", "input", "output")


def _instances(gb, envs: dict) -> tuple[list[_Inst], list[str]]:
    groups, N = gb.motion_groups(), gb.num_disks
    bodies = list(BODIES_FIXED) + [f"cycloid_{k}" for k in range(N)]
    out = []
    for name, locs in gb.locations().items():
        if name not in envs:
            raise KeyError(f"part {name!r} has no envelope")
        chunk = max(1, len(locs) // N)
        for i, loc in enumerate(locs):
            g = groups[name]
            body = f"cycloid_{min(i // chunk, N - 1)}" if g == "cycloid" else g
            A, t, sz, tz = _placement(loc)
            inst = _Inst(name, name if len(locs) == 1 else f"{name}_{i + 1}", bodies.index(body),
                         A, t, sz, tz, envs[name])
            zs = [inst.z(c[4], c[5]) for c in inst.env.cyls] + [inst.z(p[1], p[2]) for p in inst.env.prisms]
            inst.zlo, inst.zhi = min(z[0] for z in zs), max(z[1] for z in zs)
            inst.polys = [(_Poly(p[0]), *inst.z(p[1], p[2])) for p in inst.env.prisms]
            out.append(inst)
    return out, bodies


def body_poses(gb, bodies: list[str], thetas: np.ndarray):
    """(angle (nb, K), translation (nb, K, 2)) of each rigid body at each input angle, matching
    the GUI's motion: input turns by theta, the output (and each disc's spin) by -theta/ratio,
    and disc k orbits at phase k*360/N."""
    e, ratio, N = gb.cycloid.e, gb.cycloid.ratio, gb.num_disks
    out = -thetas / ratio
    ang = np.zeros((len(bodies), len(thetas)))
    tr = np.zeros((len(bodies), len(thetas), 2))
    for i, b in enumerate(bodies):
        if b == "input":
            ang[i] = thetas
        elif b == "output":
            ang[i] = out
        elif b.startswith("cycloid_"):
            phi = 2 * np.pi * int(b.split("_")[1]) / N
            ang[i] = out
            home = _rot(out) @ np.array([e * np.cos(phi), e * np.sin(phi)])
            tr[i] = np.column_stack([e * np.cos(thetas + phi), e * np.sin(thetas + phi)]) - home
    return ang, tr


def poser(gb):
    """posed(label, theta_deg): an instance's CAD with the input turned by theta_deg (for
    drawing overlaps)."""
    insts, bodies = _instances(gb, gb.envelopes())
    where, count = {}, {}
    for inst in insts:
        where[inst.label] = (inst, count.get(inst.name, 0))
        count[inst.name] = count.get(inst.name, 0) + 1
    locations = gb.locations()

    def posed(label: str, theta_deg: float):
        inst, i = where[label]
        ang, tr = body_poses(gb, bodies, np.array([math.radians(theta_deg)]))
        b = inst.body
        pose = Location((tr[b, 0, 0], tr[b, 0, 1], 0), (0, 0, math.degrees(ang[b, 0])))
        return gb.parts[inst.name].moved(pose * locations[inst.name][i])
    return posed


# ---------- material tests (in an instance's assembled-pose world frame) ----------

def _breaks(inst: _Inst):
    zs = {inst.zlo, inst.zhi}
    for c in inst.env.cyls:
        zs.update(inst.z(c[4], c[5]))
    for v in inst.env.voids:
        zs.update(inst.z(v[3], v[4]))
    for _, z0, z1 in inst.polys:
        zs.update((z0, z1))
    zs = sorted(zs)
    return [(a, b) for a, b in zip(zs[:-1], zs[1:]) if b - a > EPS]


def _solid_at(inst: _Inst, p, zm, closed=False):
    """Inside the instance's solids at height zm. closed=True counts the boundary too."""
    tol = -EPS if closed else EPS
    hit = np.zeros(len(p), bool)
    for x, y, r, rin, z0, z1 in inst.env.cyls:
        za, zb = inst.z(z0, z1)
        if za < zm < zb:
            d = np.hypot(*(p - inst.xy([x, y])).T)
            hit |= (d < r - tol) & ((rin <= 0) | (d > rin + tol))
    if inst.polys:
        local = (p - inst.t) @ inst.A        # A is orthogonal: inverse = transpose
        for poly, za, zb in inst.polys:
            if za < zm < zb:
                hit |= poly.inside(local)
    return hit


def _void_at(inst: _Inst, p, zm, strict=False):
    """Inside one of the instance's holes at height zm (strict: clearly inside, not on its edge)."""
    tol = -EPS if strict else EPS
    hit = np.zeros(len(p), bool)
    for x, y, r, z0, z1 in inst.env.voids:
        za, zb = inst.z(z0, z1)
        if za < zm < zb:
            hit |= np.hypot(*(p - inst.xy([x, y])).T) < r + tol
    return hit


def _material(inst: _Inst, p, zint):
    """Points p (assembled-pose world frame), each over a Z interval, that lie in the instance's
    material somewhere in that interval (boundary contact doesn't count)."""
    hit = np.zeros(len(p), bool)
    for a, b in _breaks(inst):
        sel = (zint[:, 0] < b - EPS) & (zint[:, 1] > a + EPS)
        if sel.any():
            zm = (a + b) / 2
            q = p[sel]
            hit[sel] |= _solid_at(inst, q, zm) & ~_void_at(inst, q, zm)
    return hit


def _outline_points(inst: _Inst):
    """Points on the instance's material boundary - its prism outlines and the edges of its
    holes - in the assembled-pose world frame, each with the Z interval it bounds material over."""
    src = []
    for poly, z0, z1 in inst.polys:
        src.append((inst.xy(_resample(poly.poly)), z0, z1, False))
    for x, y, r, z0, z1 in inst.env.voids:
        src.append((inst.xy(_circle_points(x, y, r)), *inst.z(z0, z1), True))
    pts, zint = [], []
    for a, b in _breaks(inst):
        zm = (a + b) / 2
        for p, z0, z1, is_hole in src:
            if not (z0 < zm < z1):
                continue
            keep = ~_void_at(inst, p, zm, strict=True)
            if is_hole:
                keep &= _solid_at(inst, p, zm, closed=True)
            pts.append(p[keep])
            zint.append(np.tile([a, b], (int(keep.sum()), 1)))
    if not pts:
        return np.zeros((0, 2)), np.zeros((0, 2))
    return np.vstack(pts), np.vstack(zint)


# ---------- the check ----------

def clearance(gb, angles: int = 360) -> list[tuple[str, str, float, float]]:
    """Interference over one input turn: [(label_a, label_b, depth_mm, theta_deg)], the worst
    angle for each colliding pair. Parts on the same rigid body are checked once (they don't
    move relative to each other); parts on different bodies at every angle."""
    from scipy.spatial import cKDTree

    insts, bodies = _instances(gb, gb.envelopes())
    thetas = 2 * np.pi * np.arange(angles) / angles
    ang, tr = body_poses(gb, bodies, thetas)
    R = _rot(ang)                                                   # (nb, K, 2, 2)
    worst: dict[tuple[int, int], tuple[float, int]] = {}

    def record(i, j, depth, k):
        key = (min(i, j), max(i, j))
        if depth > worst.get(key, (0.0, 0))[0]:
            worst[key] = (float(depth), int(k))

    def world(body, p0):
        """p0 (n, 2) in the assembled pose -> (n, K, 2) at every angle."""
        return np.einsum("kab,nb->nka", R[body], p0) + tr[body][None]

    # cylinder and void tables
    C = [(ii, *inst.xy(c[:2]), c[2], c[3], *inst.z(c[4], c[5]))
         for ii, inst in enumerate(insts) for c in inst.env.cyls]
    V = [(ii, *inst.xy(v[:2]), v[2], *inst.z(v[3], v[4]))
         for ii, inst in enumerate(insts) for v in inst.env.voids]
    C = np.array(C, float).reshape(-1, 7)
    V = np.array(V, float).reshape(-1, 6)
    c_inst, v_inst = C[:, 0].astype(int), V[:, 0].astype(int)
    body_of = np.array([inst.body for inst in insts])
    Cw = np.zeros((len(C), angles, 2))
    Vw = np.zeros((len(V), angles, 2))
    for b in range(len(bodies)):
        m = body_of[c_inst] == b
        if m.any():
            Cw[m] = world(b, C[m, 1:3])
        m = body_of[v_inst] == b
        if m.any():
            Vw[m] = world(b, V[m, 1:3])
    voids_of = {ii: np.flatnonzero(v_inst == ii) for ii in range(len(insts))}

    # 1. cylinder against cylinder
    i, j = np.triu_indices(len(C), 1)
    zlo, zhi = np.maximum(C[i, 5], C[j, 5]), np.minimum(C[i, 6], C[j, 6])
    # every body turns about the centre axis and the discs also orbit by e, so a pair can only
    # meet if their distances from the axis come within reach of each other
    rad = np.hypot(C[:, 1], C[:, 2])
    reach = C[i, 3] + C[j, 3] + 4 * gb.cycloid.e + EPS
    keep = (c_inst[i] != c_inst[j]) & (zhi - zlo > EPS) & (np.abs(rad[i] - rad[j]) < reach)
    i, j, zlo, zhi = i[keep], j[keep], zlo[keep], zhi[keep]
    same = body_of[c_inst[i]] == body_of[c_inst[j]]
    d = np.linalg.norm(Cw[i] - Cw[j], axis=-1)                   # (P, K)
    ri, rj, ini, inj = C[i, 3:4], C[j, 3:4], C[i, 4:5], C[j, 4:5]
    hit = (d < ri + rj - EPS) & ((ini <= 0) | (d + rj > ini + EPS)) & ((inj <= 0) | (d + ri > inj + EPS))
    hit[same, 1:] = False
    for p in np.flatnonzero(hit.any(1)):
        ks = np.flatnonzero(hit[p])
        a, b = i[p], j[p]
        ok = np.zeros(len(ks), bool)
        # cancelled where one cylinder sits wholly inside a hole of the other part
        for x, y in ((a, b), (b, a)):
            for v in voids_of[c_inst[x]]:
                if V[v, 4] <= zlo[p] + EPS and V[v, 5] >= zhi[p] - EPS:
                    dv = np.linalg.norm(Vw[v, ks] - Cw[y, ks], axis=-1)
                    ok |= dv + C[y, 3] <= V[v, 3] + EPS
        depth = np.minimum(ri[p] + rj[p] - d[p, ks],
                           np.minimum(np.where(ini[p] > 0, d[p, ks] + rj[p] - ini[p], np.inf),
                                      np.where(inj[p] > 0, d[p, ks] + ri[p] - inj[p], np.inf)))
        depth = np.minimum(depth, zhi[p] - zlo[p])          # an axial overlap is a penetration too
        for k, dep in zip(ks[~ok], depth[~ok]):
            record(c_inst[a], c_inst[b], dep, k)

    # 2. prisms (the cycloid): outline points against the other parts' material, and the other
    # parts' cylinders buried inside the prism
    for pi, P in enumerate(insts):
        if not P.polys:
            continue
        pts, zint = _outline_points(P)
        if not len(pts):
            continue
        tree = cKDTree(pts)
        lo, hi = pts.min(0), pts.max(0)
        Rp_inv = np.swapaxes(R[P.body], -1, -2)                    # world -> P's assembled frame
        others = [qi for qi, Q in enumerate(insts)
                  if qi != pi and Q.zhi > P.zlo + EPS and Q.zlo < P.zhi - EPS]
        for qi in others:
            Q = insts[qi]
            same_body = Q.body == P.body
            ks = np.arange(1) if same_body else np.arange(angles)
            for c in np.flatnonzero(c_inst == qi):
                if not (C[c, 6] > P.zlo + EPS and C[c, 5] < P.zhi - EPS):
                    continue
                rel = np.einsum("kab,kb->ka", Rp_inv[ks], Cw[c, ks] - tr[P.body][ks])
                kc = ks
                near = np.all((rel > lo - C[c, 3]) & (rel < hi + C[c, 3]), axis=1)
                if not near.any():
                    continue
                kc, rel = kc[near], rel[near]
                # (a) P's outline points inside this cylinder (and not in one of Q's holes)
                for kk, cand in enumerate(tree.query_ball_point(rel, C[c, 3] - EPS)):
                    if not cand:
                        continue
                    cand = np.asarray(cand)
                    zi = zint[cand]
                    dd = np.hypot(*(pts[cand] - rel[kk]).T)
                    zov = np.minimum(zi[:, 1], C[c, 6]) - np.maximum(zi[:, 0], C[c, 5])
                    m = zov > EPS
                    if C[c, 4] > 0:
                        m &= dd > C[c, 4] + EPS
                    if m.any() and len(voids_of[qi]):
                        k = kc[kk]
                        vrel = (Vw[voids_of[qi], k] - tr[P.body][k]) @ R[P.body][k]
                        for v, vp in zip(voids_of[qi], vrel):
                            inz = (V[v, 4] <= np.maximum(zi[:, 0], C[c, 5]) + EPS) & \
                                  (V[v, 5] >= np.minimum(zi[:, 1], C[c, 6]) - EPS)
                            m &= ~(inz & (np.hypot(*(pts[cand] - vp).T) < V[v, 3] + EPS))
                    if m.any():
                        record(pi, qi, float(np.minimum(C[c, 3] - dd[m], zov[m]).max()), kc[kk])
                # (b) the cylinder buried in P: an interior point of it inside P's material
                off = np.array([(C[c, 3] + C[c, 4]) / 2 if C[c, 4] > 0 else 0.0, 0.0])
                rep = rel + off @ np.eye(2)
                inside = _material(P, rep, np.tile(C[c, 5:7], (len(rep), 1)))
                if inside.any():
                    zov = min(C[c, 6], P.zhi) - max(C[c, 5], P.zlo)
                    record(pi, qi, float(min(C[c, 3], zov)), kc[np.argmax(inside)])
            # (c) another prism's outline inside P
            if Q.polys:
                qpts, qz = _outline_points(Q)
                for k in ks:
                    w = (qpts @ R[Q.body][k].T + tr[Q.body][k] - tr[P.body][k]) @ R[P.body][k]
                    if _material(P, w, qz).any():
                        record(pi, qi, EPS, k)
                        break

    out = [(insts[a].label, insts[b].label, round(dep, 4), round(math.degrees(thetas[k]), 1))
           for (a, b), (dep, k) in worst.items()]
    return sorted(out, key=lambda h: -h[2])
