import numpy as np
from build123d import *
from part_generation_fcns.classdefs import *
from part_generation_fcns.errors import GeometryError, geometry_issue
from part_generation_fcns.paths import ensure_parent
from part_generation_fcns.tolerancing import NOMINAL, tolerancing


def base_outer_r(b: base) -> float:
    """Disc radius: edge_offset beyond the roller holes' outer edge."""
    return b.roller_holes_R + b.roller_bearing.ID / 2 + b.edge_offset


def base_mounting_holes(b: base) -> patterned_hole:
    """Mounting tab holes with R resolved to an absolute radius (stored R is beyond the base edge)."""
    h = b.mounting_holes
    return patterned_hole(r=h.r, R=base_outer_r(b) + h.R, N=h.N, start_angle=h.start_angle)


def _check_base(b: base):
    center_r = b.drive_motor.shaft_r + b.shaft_hole_offset
    motor_h = b.drive_motor.mounting_holes
    roller_hole_r = b.roller_bearing.ID / 2
    ring_r = b.mounting_holes.r + b.mounting_hole_offset

    if center_r >= motor_h.R - motor_h.r:
        geometry_issue("base.center_hole_hits_motor_holes", "center hole overlaps motor mounting holes")
    if motor_h.R + motor_h.r >= b.roller_holes_R - roller_hole_r:
        geometry_issue("base.motor_holes_hit_roller_holes", "motor mounting holes overlap roller holes")
    if b.edge_offset <= 0:
        geometry_issue("base.edge_offset", "base edge_offset must be > 0 (roller holes would break through the edge)")
    if b.mounting_holes.R <= 0:
        geometry_issue("base.mounting_holes_R", "mounting_holes.R (distance beyond the base edge) must be > 0")
    if ring_r >= base_outer_r(b):
        geometry_issue("base.tab_too_wide", "mounting tab wider than the base")


def make_base(b: base, tol: tolerancing = NOMINAL) -> Part:
    _check_base(b)
    ring_r = b.mounting_holes.r + b.mounting_hole_offset
    outer_r = base_outer_r(b)
    mh = base_mounting_holes(b)
    tab_angles = np.radians(mh.start_angle + np.arange(mh.N) * 360 / mh.N)

    def is_tab_corner(v: Vertex) -> bool:
        """Concave corner where a tab's straight side meets the outer circle."""
        x, y = v.X, v.Y
        if abs(np.hypot(x, y) - outer_r) > 1e-3:
            return False
        # perpendicular distance from each tab's centerline
        perp = np.abs(-x * np.sin(tab_angles) + y * np.cos(tab_angles))
        return bool(np.any(np.abs(perp - ring_r) < 1e-3))

    with BuildPart() as part:
        with BuildSketch() as sk:
            Circle(outer_r)
            # tabs: strip from center out to the hole, capped by the donut ring
            with PolarLocations(0, mh.N, start_angle=mh.start_angle):
                Rectangle(mh.R, 2 * ring_r, align=(Align.MIN, Align.CENTER))
            with PolarLocations(mh.R, mh.N, start_angle=mh.start_angle):
                Circle(ring_r)
            if b.mounting_hole_fillet_r > 0:
                fillet([v for v in sk.vertices() if is_tab_corner(v)], b.mounting_hole_fillet_r)

            Circle(b.drive_motor.shaft_r + b.shaft_hole_offset, mode=Mode.SUBTRACT)
            with PolarLocations(b.drive_motor.mounting_holes.R, b.drive_motor.mounting_holes.N,
                                start_angle=b.drive_motor.mounting_holes.start_angle):
                Circle(b.drive_motor.mounting_holes.r + tol.delta("base.motor_holes"), mode=Mode.SUBTRACT)
            with PolarLocations(b.roller_holes_R, b.roller_holes_N):
                Circle(b.roller_bearing.ID / 2 + tol.delta("base.roller_holes"), mode=Mode.SUBTRACT)
            with PolarLocations(mh.R, mh.N, start_angle=mh.start_angle):
                Circle(b.mounting_holes.r + tol.delta("base.mounting_holes"), mode=Mode.SUBTRACT)
        extrude(amount=b.t)
    return part.part


def make_base_step(b: base, path: str = "steps/base.step") -> Part:
    part = make_base(b)
    part.label = "base"
    export_step(part, ensure_parent(path))
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
        edge_offset=7.0,                                           # 48 mm radius
        t=5.0,
        mounting_holes=patterned_hole(r=2.2, R=7.0, N=2),          # M4, centers 7 mm beyond the edge (R 55)
        mounting_hole_offset=3.0,
        mounting_hole_fillet_r=3.0,
    )
    part = make_base_step(b)
    print("volume", part.volume, "valid", part.is_valid)
