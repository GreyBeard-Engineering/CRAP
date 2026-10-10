from build123d import *
from part_generation_fcns.classdefs import *


def check_cap(cp: cap, b: bearing):
    """Validate a cap against the bearing it retains."""
    ri = b.OD / 2 - cp.bearing_support_flange_offset
    rh = cp.bearing_retainer_holes

    if ri <= b.ID / 2:
        print("warning: support flange / cap reaches the bearing inner race")
    if rh.R - rh.r <= b.OD / 2:
        raise ValueError("bearing retainer holes overlap the bearing seat")
    if rh.R + rh.r >= cp.r:
        raise ValueError("bearing retainer holes break through the cap edge")


def make_cap(cp: cap, b: bearing) -> Part:
    """Ring clamping the bearing outer race against the support flange."""
    t = cp.bearing_support_flange_thickness

    with BuildPart() as part:
        with BuildSketch():
            Circle(cp.r)
            Circle(b.OD / 2 - cp.bearing_support_flange_offset, mode=Mode.SUBTRACT)
            with PolarLocations(cp.bearing_retainer_holes.R, cp.bearing_retainer_holes.N):
                Circle(cp.bearing_retainer_holes.r, mode=Mode.SUBTRACT)
        extrude(amount=t)
    return part.part
