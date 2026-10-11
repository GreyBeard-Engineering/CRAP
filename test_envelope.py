"""Checks that the fast clearance check's part envelopes still match the CAD.

Run:  python test_envelope.py

1. Every unique part: random points in its bounding box must be inside the envelope exactly
   when they're inside the CAD solid (allowing ~1% for fillets and screw sockets, which the
   envelopes leave out). A generator change without a matching envelope change fails here.
2. The fast check against the exact (OCC) check in the assembled pose, on clean and
   deliberately broken designs: both must report the same colliding pairs.
"""
import time

import numpy as np
from build123d import Vector

from part_generation_fcns.envelope import contains
from part_generation_fcns.gearbox import EXAMPLE_KWARGS, gearbox

DESIGNS = {
    "example": {},
    "2 discs": {"num_disks": 2},
    "3 discs": {"num_disks": 3},
}
BROKEN = {
    "cap screws into the output": {"cap_output_spacing": 1.0},
    "needle screws into the base": {"base_cycloid_spacing": 0.2},
    "ring screws into the output": {"output_top_spacing": 0.0, "output_plate_edge_offset": 20.0},
}


def check_envelopes(gb: gearbox, tag: str, n: int = 3000, limit: float = 0.01):
    rng = np.random.default_rng(0)
    envs = gb.envelopes()
    for name, part in gb.parts.items():
        bb = part.bounding_box()
        lo, hi = np.array([bb.min.X, bb.min.Y, bb.min.Z]), np.array([bb.max.X, bb.max.Y, bb.max.Z])
        pad = 0.05 * (hi - lo) + 0.1
        pts = rng.uniform(lo - pad, hi + pad, size=(n, 3))
        cad = np.array([part.is_inside(Vector(*p)) for p in pts])
        env = contains(envs[name], pts)
        miss = float(np.mean(cad != env))
        assert miss <= limit, f"{tag}: {name} envelope disagrees with its CAD at {miss:.1%} of points"
    print(f"  {tag}: {len(gb.parts)} part envelopes match their CAD")


def pairs(hits) -> set:
    return {tuple(sorted(h[:2])) for h in hits}


def check_against_exact(tag: str, kw: dict, expect_hits: bool):
    gb = gearbox(**{**EXAMPLE_KWARGS, **kw}, assembly_path=None, strict=False)
    t = time.time()
    fast = gb.collisions(mode="fast", angles=1)        # the assembled pose only, like exact
    t_fast = time.time() - t
    t = time.time()
    exact = gb.collisions(mode="exact")
    t_exact = time.time() - t
    sweep_t = time.time()
    swept = gb.collisions()
    t_sweep = time.time() - sweep_t
    assert pairs(fast) == pairs(exact), (f"{tag}: fast and exact disagree\n  fast only: "
                                         f"{pairs(fast) - pairs(exact)}\n  exact only: {pairs(exact) - pairs(fast)}")
    assert bool(exact) == expect_hits, f"{tag}: expected {'collisions' if expect_hits else 'none'}, got {exact}"
    assert pairs(fast) <= pairs(swept), f"{tag}: the sweep missed pairs found at theta 0"
    print(f"  {tag}: {len(exact)} colliding pairs, fast and exact agree "
          f"(fast {t_fast:.2f} s, full-turn sweep {t_sweep:.2f} s, exact {t_exact:.1f} s)")
    return gb


if __name__ == "__main__":
    print("envelopes against the CAD")
    for tag, kw in DESIGNS.items():
        check_envelopes(gearbox(**{**EXAMPLE_KWARGS, **kw}, assembly_path=None, strict=False), tag)
    print("fast check against the exact check")
    check_against_exact("example", {}, expect_hits=False)
    for tag, kw in BROKEN.items():
        check_against_exact(tag, kw, expect_hits=True)
    print("all checks passed")
