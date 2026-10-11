import numpy as np
from build123d import *
from part_generation_fcns.classdefs import *
from part_generation_fcns.errors import GeometryError, geometry_issue
from part_generation_fcns.paths import ensure_parent
from part_generation_fcns.cap_step import cap_radii, check_cap, make_cap
from part_generation_fcns.tolerancing import NOMINAL, tolerancing

def cycloid_profile(c: cycloid, pts_per_lobe: int = 60, clearance: float = 0.0) -> np.ndarray:
    """Points on the disc outline (pin-offset epitrochoid). The profile is the trochoid
    offset by the roller radius, so clearance just adds to that offset."""
    roller_r = c.roller_bearing.OD / 2 + clearance
    n = c.ratio + 1  # number of ring rollers
    th = np.linspace(0, 2 * np.pi, c.ratio * pts_per_lobe, endpoint=False)
    psi = np.arctan2(np.sin((1 - n) * th),
                     c.roller_R / (c.e * n) - np.cos((1 - n) * th))
    x = c.roller_R * np.cos(th) - roller_r * np.cos(th + psi) - c.e * np.cos(n * th)
    y = -c.roller_R * np.sin(th) + roller_r * np.sin(th + psi) + c.e * np.sin(n * th)
    return np.column_stack([x, y])


def profile_undercuts(c: cycloid, pts_per_lobe: int = 400) -> bool:
    """True when the roller is too big for the trochoid's tightest curve: the offset profile
    then doubles back on itself (its direction reverses) and the disc outline is invalid."""
    p = cycloid_profile(c, pts_per_lobe)
    d = np.diff(np.vstack([p, p[:1]]), axis=0)
    return bool(np.any(np.sum(d * np.roll(d, -1, axis=0), axis=1) < 0))


def output_holes(c: cycloid) -> patterned_hole:
    """Output holes sized so each needle roller orbits its hole by e."""
    return patterned_hole(r=c.output_needle_roller.OD / 2 + c.e, R=c.output_holes_R, N=c.output_holes_N)


def cap_reaches_output_holes(c: cycloid) -> bool:
    """True when the cycloid cap extends over the output holes, so it needs the holes too."""
    oh = output_holes(c)
    return cap_radii(c.cap, c.input_bearing)[1] > oh.R - oh.r


def cap_screw_hole_gap(c: cycloid) -> float:
    """Smallest edge-to-edge distance between a cap screw hole and an output hole."""
    oh, rh = output_holes(c), c.cap.bearing_retainer_holes
    a = np.radians(oh.start_angle + np.arange(oh.N) * 360 / oh.N)
    b = np.radians(rh.start_angle + np.arange(rh.N) * 360 / rh.N)
    d = np.hypot(oh.R * np.cos(a)[:, None] - rh.R * np.cos(b)[None, :],
                 oh.R * np.sin(a)[:, None] - rh.R * np.sin(b)[None, :])
    return float(d.min() - oh.r - rh.r)


def cycloid_thickness(c: cycloid) -> float:
    return c.cap.flange.thickness + c.input_bearing.th


def make_cycloid(c: cycloid, tol: tolerancing = NOMINAL) -> Part:
    n = c.ratio + 1
    if c.roller_R / (c.e * n) <= 1:
        geometry_issue("cycloid.cusps", "roller_R / (e * (ratio+1)) must be > 1 (profile cusps)")
    if profile_undercuts(c):
        geometry_issue("cycloid.undercut", "the roller is too large for this e / ratio / roller R: "
                            "the cycloid profile undercuts (reduce e or the roller OD, or increase roller R)")
    check_cap(c.cap, c.input_bearing, name="cycloid_cap")
    oh = output_holes(c)
    if cap_reaches_output_holes(c):
        # the cap gets the output holes too; its screws must sit clear of them, and it must
        # stay inside the disc
        if cap_screw_hole_gap(c) <= 0:
            geometry_issue("cycloid.cap_screws_hit_output_holes",
                                "cycloid cap screw holes run into the output holes")
        root = c.roller_R - c.roller_bearing.OD / 2 - c.e
        if cap_radii(c.cap, c.input_bearing)[1] >= root:
            geometry_issue("cycloid.cap_past_disc_edge", "cycloid cap reaches past the cycloid's edge")

    tf = c.cap.flange.thickness
    rb = c.input_bearing.OD / 2 + tol.delta("cycloid.bearing_seat")
    pts = [tuple(p) for p in cycloid_profile(c, clearance=tol.clearance)]

    with BuildPart() as disc:
        with BuildSketch():
            with BuildLine():
                Spline(*pts, periodic=True)
            make_face()
            Circle(c.input_bearing.OD / 2 - c.cap.flange.offset, mode=Mode.SUBTRACT)  # flange lip bore
            with PolarLocations(oh.R, oh.N):
                Circle(oh.r + tol.clearance, mode=Mode.SUBTRACT)
            with PolarLocations(c.cap.bearing_retainer_holes.R, c.cap.bearing_retainer_holes.N, start_angle=c.cap.bearing_retainer_holes.start_angle):
                Circle(c.cap.bearing_retainer_holes.r + tol.delta("cycloid.retainer_holes"), mode=Mode.SUBTRACT)
        extrude(amount=cycloid_thickness(c))
        # bearing seat above the flange lip
        with BuildSketch(Plane.XY.offset(tf)):
            Circle(rb)
        extrude(amount=c.input_bearing.th, mode=Mode.SUBTRACT)
    return disc.part


def make_cycloid_step(c: cycloid, path: str = "steps/cycloid.step",
                      cap_path: str = "steps/cycloid_cap.step") -> tuple[Part, Part]:
    """Export cycloid disc and its cap to separate STEP files, each sitting on z=0."""
    disc = make_cycloid(c)
    cap_part = make_cap(c.cap, c.input_bearing)
    disc.label, cap_part.label = "cycloid", "cycloid_cap"
    export_step(disc, ensure_parent(path))
    export_step(cap_part, ensure_parent(cap_path))
    return disc, cap_part


if __name__ == "__main__":
    c = cycloid(
        roller_bearing=bearing(ID=2, OD=5, th=2.5),      # MR52-ish
        roller_R=40.0,
        input_bearing=bearing(ID=10, OD=19, th=5),        # 6800-ish
        output_needle_roller=bearing(ID=3, OD=6, th=2.5),  # MR63-ish
        output_holes_R=24.0,
        output_holes_N=6,
        e=1.2,
        ratio=23,
        cap=cap(
            flange=flange(thickness=2.0, offset=1.5),
            bearing_retainer_holes=patterned_hole(r=1.0, R=12.0, N=4),  # M2 pilot
            edge_offset=2.0,                                      # 15 mm radius
        ),
    )
    disc, cap_part = make_cycloid_step(c)
    for p in (disc, cap_part):
        print(p.label, "volume", p.volume, "valid", p.is_valid)
