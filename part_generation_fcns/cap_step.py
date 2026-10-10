from build123d import *
from part_generation_fcns.classdefs import *


def cap_radii(cp: cap, b: bearing, race: str = "outer") -> tuple[float, float]:
    """(inner, outer) radius of the cap ring.
    race="outer": clamps the outer race; inner = OD/2 - offset, outer = cp.r
    race="inner": clamps the inner race; inner = cp.r, outer = ID/2 + offset"""
    if race == "outer":
        return b.OD / 2 - cp.flange.offset, cp.r
    if race == "inner":
        return cp.r, b.ID / 2 + cp.flange.offset
    raise ValueError(f"race must be 'outer' or 'inner', got {race!r}")


def check_cap(cp: cap, b: bearing, race: str = "outer"):
    """Validate a cap against the bearing it retains."""
    ri, ro = cap_radii(cp, b, race)
    rh = cp.bearing_retainer_holes

    if race == "outer":
        if ri <= b.ID / 2:
            print("warning: support flange / cap reaches the bearing inner race")
        if rh.R - rh.r <= b.OD / 2:
            raise ValueError("bearing retainer holes overlap the bearing seat")
        if rh.R + rh.r >= ro:
            raise ValueError("bearing retainer holes break through the cap edge")
    else:
        if ro >= b.OD / 2:
            print("warning: support flange / cap reaches the bearing outer race")
        if rh.R + rh.r >= b.ID / 2:
            raise ValueError("bearing retainer holes break through the journal")
        if rh.R - rh.r <= ri:
            raise ValueError("bearing retainer holes break into the cap bore")


def make_cap(cp: cap, b: bearing, race: str = "outer", hole_delta: float = 0.0) -> Part:
    """Ring clamping a bearing race against its support flange. hole_delta grows (or, if
    negative, shrinks) the screw holes."""
    ri, ro = cap_radii(cp, b, race)

    with BuildPart() as part:
        with BuildSketch():
            Circle(ro)
            if ri > 0:
                Circle(ri, mode=Mode.SUBTRACT)
            with PolarLocations(cp.bearing_retainer_holes.R, cp.bearing_retainer_holes.N):
                Circle(cp.bearing_retainer_holes.r + hole_delta, mode=Mode.SUBTRACT)
        extrude(amount=cp.flange.thickness)
    return part.part
