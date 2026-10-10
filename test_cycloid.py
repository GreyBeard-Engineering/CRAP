"""Define a test cycloid disc, sanity-check its profile, and generate disc + cap STEP files.

Run:  python test_cycloid.py [output.step] [cap.step]
"""
import sys
import numpy as np
from part_generation_fcns.classdefs import *
from part_generation_fcns.cycloid_step import cycloid_profile, cycloid_thickness, make_cycloid_step, output_holes


def make_test_cycloid() -> cycloid:
    return cycloid(
        roller_bearing=bearing(ID=2, OD=5, th=2.5),       # MR52-ish
        roller_R=40.0,
        input_bearing=bearing(ID=10, OD=19, th=5),         # 6800-ish
        output_needle_roller=bearing(ID=3, OD=6, th=2.5),  # MR63-ish
        output_holes_R=24.0,
        output_holes_N=6,
        e=1.2,
        ratio=23,
        cap=cap(
            flange=flange(thickness=2.0, offset=1.5),
            bearing_retainer_holes=patterned_hole(r=1.0, R=12.0, N=4),
            r=15.0,
        ),
    )


def check_profile(c: cycloid, pts_per_lobe: int = 60):
    pts = cycloid_profile(c, pts_per_lobe)
    roller_r = c.roller_bearing.OD / 2
    r = np.hypot(pts[:, 0], pts[:, 1])

    assert pts.shape == (c.ratio * pts_per_lobe, 2), pts.shape
    assert np.all(np.isfinite(pts)), "profile has NaN/inf points"

    # Lobe tips/valleys sit at roller_R - roller_r +/- e
    r_max_expected = c.roller_R - roller_r + c.e
    r_min_expected = c.roller_R - roller_r - c.e
    assert abs(r.max() - r_max_expected) < 0.05, (r.max(), r_max_expected)
    assert abs(r.min() - r_min_expected) < 0.05, (r.min(), r_min_expected)

    # One radius peak per lobe
    peaks = np.sum((r > np.roll(r, 1)) & (r > np.roll(r, -1)))
    assert peaks == c.ratio, f"expected {c.ratio} lobes, found {peaks}"

    # Features must fit inside the disc's minimum radius
    oh = output_holes(c)
    assert abs(oh.r - (c.output_needle_roller.OD / 2 + c.e)) < 1e-9
    holes_outer = oh.R + oh.r
    holes_inner = oh.R - oh.r
    assert holes_outer < r_min_expected, "output holes break through the lobes"
    assert c.input_bearing.OD / 2 < holes_inner, "bearing seat overlaps output holes"

    print(f"profile ok: {len(pts)} pts, {peaks} lobes, r in [{r.min():.3f}, {r.max():.3f}]")
    return r_max_expected


def check_part(c: cycloid, path: str, cap_path: str, r_max: float):
    part, cap_part = make_cycloid_step(c, path, cap_path)
    bb = part.bounding_box()
    t = cycloid_thickness(c)

    assert part.is_valid, "generated solid is not valid"
    assert part.volume > 0
    assert abs(bb.size.Z - t) < 1e-6, f"thickness {bb.size.Z} != {t}"
    assert bb.size.X <= 2 * r_max + 0.1 and bb.size.Y <= 2 * r_max + 0.1, bb.size
    print(f"part ok: volume={part.volume:.1f} mm^3, bbox={bb.size}, wrote {path}")

    cbb = cap_part.bounding_box()
    assert cap_part.is_valid, "generated cap is not valid"
    assert abs(cbb.size.Z - c.cap.flange.thickness) < 1e-6, cbb.size
    assert abs(cbb.size.X - 2 * c.cap.r) < 1e-3, cbb.size
    print(f"cap ok: volume={cap_part.volume:.1f} mm^3, bbox={cbb.size}, wrote {cap_path}")


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "test_cycloid.step"
    cap_out = sys.argv[2] if len(sys.argv) > 2 else "test_cycloid_cap.step"
    c = make_test_cycloid()
    r_max = check_profile(c)
    check_part(c, out, cap_out, r_max)
    print("all checks passed")
