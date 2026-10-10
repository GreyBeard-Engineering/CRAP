import numpy as np
from build123d import *
from part_generation_fcns.classdefs import *


def _check_base(b: base):
    center_r = b.drive_motor.shaft_r + b.shaft_hole_offset
    motor_h = b.drive_motor.mounting_holes
    roller_hole_r = b.roller_bearing.ID / 2
    ring_r = b.mounting_holes.r + b.mounting_hole_offset

    if center_r >= motor_h.R - motor_h.r:
        raise ValueError("center hole overlaps motor mounting holes")
    if motor_h.R + motor_h.r >= b.roller_holes_R - roller_hole_r:
        raise ValueError("motor mounting holes overlap roller holes")
    if b.roller_holes_R + roller_hole_r >= b.outer_r:
        raise ValueError("roller holes break through outer edge")
    if b.mounting_holes.R <= b.outer_r:
        raise ValueError("mounting_holes.R must exceed outer_r")
    if ring_r >= b.outer_r:
        raise ValueError("mounting tab wider than the base")


def make_base(b: base) -> Part:
    _check_base(b)
    ring_r = b.mounting_holes.r + b.mounting_hole_offset
    tab_angles = np.radians(np.arange(b.mounting_holes.N) * 360 / b.mounting_holes.N)

    def is_tab_corner(v: Vertex) -> bool:
        """Concave corner where a tab's straight side meets the outer circle."""
        x, y = v.X, v.Y
        if abs(np.hypot(x, y) - b.outer_r) > 1e-3:
            return False
        # perpendicular distance from each tab's centerline
        perp = np.abs(-x * np.sin(tab_angles) + y * np.cos(tab_angles))
        return bool(np.any(np.abs(perp - ring_r) < 1e-3))

    with BuildPart() as part:
        with BuildSketch() as sk:
            Circle(b.outer_r)
            # tabs: strip from center out to the hole, capped by the donut ring
            with PolarLocations(0, b.mounting_holes.N):
                Rectangle(b.mounting_holes.R, 2 * ring_r, align=(Align.MIN, Align.CENTER))
            with PolarLocations(b.mounting_holes.R, b.mounting_holes.N):
                Circle(ring_r)
            if b.mounting_hole_fillet_r > 0:
                fillet([v for v in sk.vertices() if is_tab_corner(v)], b.mounting_hole_fillet_r)

            Circle(b.drive_motor.shaft_r + b.shaft_hole_offset, mode=Mode.SUBTRACT)
            with PolarLocations(b.drive_motor.mounting_holes.R, b.drive_motor.mounting_holes.N):
                Circle(b.drive_motor.mounting_holes.r, mode=Mode.SUBTRACT)
            with PolarLocations(b.roller_holes_R, b.roller_holes_N):
                Circle(b.roller_bearing.ID / 2, mode=Mode.SUBTRACT)
            with PolarLocations(b.mounting_holes.R, b.mounting_holes.N):
                Circle(b.mounting_holes.r, mode=Mode.SUBTRACT)
        extrude(amount=b.t)
    return part.part


def make_base_step(b: base, path: str = "base.step") -> Part:
    part = make_base(b)
    part.label = "base"
    export_step(part, path)
    return part


if __name__ == "__main__":
    b = base(
        drive_motor=motor(
            shaft_r=2.5,                                           # NEMA17
            mounting_holes=patterned_hole(r=1.6, R=21.92, N=4),    # M3 on 31 mm square
        ),
        shaft_hole_offset=9.0,                                     # clears 22 mm boss
        roller_bearing=bearing(ID=2, OD=5, th=2.5),                # MR52-ish, matches cycloid
        roller_holes_R=40.0,
        roller_holes_N=24,                                         # ratio + 1 rollers
        outer_r=48.0,
        t=5.0,
        mounting_holes=patterned_hole(r=2.2, R=55.0, N=2),         # M4
        mounting_hole_offset=3.0,
        mounting_hole_fillet_r=3.0,
    )
    part = make_base_step(b)
    print("volume", part.volume, "valid", part.is_valid)
