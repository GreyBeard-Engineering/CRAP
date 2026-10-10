from build123d import *
from part_generation_fcns.classdefs import *
from part_generation_fcns.tolerancing import NOMINAL, tolerancing
from part_generation_fcns.cap_step import check_cap, make_cap


def _radii(tp: top):
    rb = tp.output_bearing.OD / 2                         # bearing seat
    ri = rb - tp.cap.flange.offset        # flange lip inner radius
    rw = rb + tp.output_bearing_offset                    # tower outer wall
    H = tp.cap.flange.thickness + tp.output_bearing.th  # tower height
    return rb, ri, rw, H


def _check_top(tp: top):
    rb, ri, rw, H = _radii(tp)
    roller_hole_r = tp.roller_bearing.ID / 2
    rh = tp.cap.bearing_retainer_holes

    check_cap(tp.cap, tp.output_bearing)
    if rh.R + rh.r >= rw:
        raise ValueError("bearing retainer holes must sit within the tower wall")
    if tp.roller_holes_R - roller_hole_r <= rw:
        raise ValueError("roller holes overlap the tower wall")
    if tp.roller_holes_R + roller_hole_r >= tp.outer_r:
        raise ValueError("roller holes break through outer edge")
    if tp.base_thickness >= H:
        raise ValueError("base_thickness must be less than the tower height")


def make_top(tp: top, tol: tolerancing = NOMINAL) -> Part:
    _check_top(tp)
    rb, ri, rw, H = _radii(tp)
    rb += tol.delta("top.bearing_seat")
    tf, tb = tp.cap.flange.thickness, tp.base_thickness

    with BuildPart() as part:
        # (r, z) cross-section, revolved about Z
        with BuildSketch(Plane.XZ):
            with BuildLine():
                Polyline((ri, 0), (tp.outer_r, 0), (tp.outer_r, tb), (rw, tb),
                         (rw, H), (rb, H), (rb, tf), (ri, tf), close=True)
            make_face()
        revolve(axis=Axis.Z)

        with BuildSketch():
            with PolarLocations(tp.roller_holes_R, tp.roller_holes_N):
                Circle(tp.roller_bearing.ID / 2 + tol.delta("top.roller_holes"))
        extrude(amount=tb, mode=Mode.SUBTRACT)
        with BuildSketch():
            with PolarLocations(tp.cap.bearing_retainer_holes.R, tp.cap.bearing_retainer_holes.N):
                Circle(tp.cap.bearing_retainer_holes.r + tol.delta("top.retainer_holes"))
        extrude(amount=H, mode=Mode.SUBTRACT)
    return part.part


def make_top_step(tp: top, top_path: str = "top.step", cap_path: str = "top_cap.step") -> tuple[Part, Part]:
    """Export top and its cap to separate STEP files, each sitting on z=0."""
    top_part = make_top(tp)
    cap_part = make_cap(tp.cap, tp.output_bearing)
    top_part.label, cap_part.label = "top", "top_cap"
    export_step(top_part, top_path)
    export_step(cap_part, cap_path)
    return top_part, cap_part


if __name__ == "__main__":
    tp = top(
        output_bearing=bearing(ID=25, OD=37, th=7),               # 6805-ish
        output_bearing_offset=6.0,
        base_thickness=4.0,
        roller_bearing=bearing(ID=2, OD=5, th=2.5),               # matches cycloid
        roller_holes_R=40.0,
        roller_holes_N=24,
        cap=cap(
            flange=flange(thickness=2.0, offset=2.0),
            bearing_retainer_holes=patterned_hole(r=1.25, R=21.5, N=6),  # M2.5 pilot
            r=24.5,                                               # flush with tower wall
        ),
        outer_r=48.0,
    )
    top_part, cap_part = make_top_step(tp)
    for p in (top_part, cap_part):
        print(p.label, "volume", p.volume, "valid", p.is_valid)
