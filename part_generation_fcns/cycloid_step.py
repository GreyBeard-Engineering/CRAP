import numpy as np
from build123d import *
from part_generation_fcns.classdefs import *
from part_generation_fcns.cap_step import check_cap, make_cap
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


def output_holes(c: cycloid) -> patterned_hole:
    """Output holes sized so each needle roller orbits its hole by e."""
    return patterned_hole(r=c.output_needle_roller.OD / 2 + c.e, R=c.output_holes_R, N=c.output_holes_N)


def cycloid_thickness(c: cycloid) -> float:
    return c.cap.flange.thickness + c.input_bearing.th


def make_cycloid(c: cycloid, tol: tolerancing = NOMINAL) -> Part:
    n = c.ratio + 1
    if c.roller_R / (c.e * n) <= 1:
        raise ValueError("roller_R / (e * (ratio+1)) must be > 1 (profile cusps)")
    check_cap(c.cap, c.input_bearing)
    oh = output_holes(c)
    if c.cap.r >= oh.R - oh.r:
        raise ValueError("cap would cover the output holes")

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
            with PolarLocations(c.cap.bearing_retainer_holes.R, c.cap.bearing_retainer_holes.N):
                Circle(c.cap.bearing_retainer_holes.r + tol.delta("cycloid.retainer_holes"), mode=Mode.SUBTRACT)
        extrude(amount=cycloid_thickness(c))
        # bearing seat above the flange lip
        with BuildSketch(Plane.XY.offset(tf)):
            Circle(rb)
        extrude(amount=c.input_bearing.th, mode=Mode.SUBTRACT)
    return disc.part


def make_cycloid_step(c: cycloid, path: str = "cycloid.step",
                      cap_path: str = "cycloid_cap.step") -> tuple[Part, Part]:
    """Export cycloid disc and its cap to separate STEP files, each sitting on z=0."""
    disc = make_cycloid(c)
    cap_part = make_cap(c.cap, c.input_bearing)
    disc.label, cap_part.label = "cycloid", "cycloid_cap"
    export_step(disc, path)
    export_step(cap_part, cap_path)
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
            r=15.0,
        ),
    )
    disc, cap_part = make_cycloid_step(c)
    for p in (disc, cap_part):
        print(p.label, "volume", p.volume, "valid", p.is_valid)
