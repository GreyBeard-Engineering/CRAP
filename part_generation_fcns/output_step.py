from build123d import *
from part_generation_fcns.classdefs import *
from part_generation_fcns.cap_step import check_cap, make_cap
from part_generation_fcns.tolerancing import NOMINAL, tolerancing


def output_height(o: output) -> float:
    return o.plate_t + o.flange.thickness + o.output_bearing.th


def _check_output(o: output):
    rj = o.output_bearing.ID / 2
    rs = rj + o.flange.offset
    pin_r = o.output_needle_roller.ID / 2

    if rs >= o.output_bearing.OD / 2:
        print("warning: output shoulder reaches the output bearing outer race")
    if o.output_holes_R - pin_r <= rs:
        raise ValueError("output pin holes overlap the shoulder")
    if o.output_holes_R + pin_r >= o.plate_r:
        raise ValueError("output pin holes break through the plate edge")
    if o.bore_r >= rj:
        raise ValueError("bore breaks through the journal")
    check_cap(o.cap, o.output_bearing, race="inner")
    if o.cap.r < o.bore_r:
        print("warning: output cap inner radius is smaller than the bore")


def make_output(o: output, tol: tolerancing = NOMINAL) -> Part:
    """Revolved profile: plate carrying the output pins, shoulder supporting the
    output bearing inner race, journal through the bearing."""
    _check_output(o)
    rj = o.output_bearing.ID / 2
    rs = rj + o.flange.offset
    tp = o.plate_t
    ts = tp + o.flange.thickness
    H = output_height(o)
    rjt = rj + tol.delta("output.journal")

    with BuildPart() as part:
        # (r, z) cross-section, revolved about Z
        with BuildSketch(Plane.XZ):
            with BuildLine():
                Polyline((0, 0), (o.plate_r, 0), (o.plate_r, tp), (rs, tp),
                         (rs, ts), (rjt, ts), (rjt, H), (0, H), close=True)
            make_face()
        revolve(axis=Axis.Z)

        with BuildSketch():
            with PolarLocations(o.output_holes_R, o.output_holes_N):
                Circle(o.output_needle_roller.ID / 2 + tol.delta("output.needle_holes"))
        extrude(amount=tp, mode=Mode.SUBTRACT)
        with BuildSketch():
            if o.bore_r > 0:
                Circle(o.bore_r)
            with PolarLocations(o.cap.bearing_retainer_holes.R, o.cap.bearing_retainer_holes.N):
                Circle(o.cap.bearing_retainer_holes.r + tol.delta("output.retainer_holes"))
        extrude(amount=H, mode=Mode.SUBTRACT)
    return part.part


def make_output_step(o: output, path: str = "output.step",
                     cap_path: str = "output_cap.step") -> tuple[Part, Part]:
    """Export output and its inner-race cap to separate STEP files, each sitting on z=0."""
    part = make_output(o)
    cap_part = make_cap(o.cap, o.output_bearing, race="inner")
    part.label, cap_part.label = "output", "output_cap"
    export_step(part, path)
    export_step(cap_part, cap_path)
    return part, cap_part


if __name__ == "__main__":
    o = output(
        output_bearing=bearing(ID=25, OD=37, th=7),               # 6805-ish
        flange=flange(thickness=3.0, offset=2.0),
        plate_r=34.0,
        plate_t=4.0,
        output_needle_roller=bearing(ID=3, OD=6, th=2.5),         # MR63-ish
        output_holes_R=24.0,
        output_holes_N=6,
        bore_r=6.0,
        cap=cap(
            flange=flange(thickness=2.0, offset=1.5),
            bearing_retainer_holes=patterned_hole(r=1.0, R=9.5, N=4),    # M2 pilot
            r=6.0,                                                # matches bore
        ),
    )
    for p in make_output_step(o):
        print(p.label, "volume", p.volume, "valid", p.is_valid)
