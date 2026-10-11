from build123d import *
from part_generation_fcns.classdefs import *
from part_generation_fcns.errors import GeometryError


def cap_radii(cp: cap, b: bearing, race: str = "outer") -> tuple[float, float]:
    """(inner, outer) radius of the cap ring. The flange offset sets the edge bearing on the
    race; the free edge sits edge_offset beyond the screw holes.
    race="outer": clamps the outer race; inner = OD/2 - flange offset, outer = holes + edge_offset
    race="inner": clamps the inner race; inner = holes - edge_offset, outer = ID/2 + flange offset"""
    rh = cp.bearing_retainer_holes
    if race == "outer":
        return b.OD / 2 - cp.flange.offset, rh.R + rh.r + cp.edge_offset
    if race == "inner":
        return rh.R - rh.r - cp.edge_offset, b.ID / 2 + cp.flange.offset
    raise ValueError(f"race must be 'outer' or 'inner', got {race!r}")


def check_cap(cp: cap, b: bearing, race: str = "outer", name: str = "cap"):
    """Validate a cap against the bearing it retains; name prefixes the error codes."""
    ri, ro = cap_radii(cp, b, race)
    rh = cp.bearing_retainer_holes

    if race == "outer":
        if ri <= b.ID / 2:
            print("warning: support flange / cap reaches the bearing inner race")
        if rh.R - rh.r <= b.OD / 2:
            raise GeometryError(f"{name}.holes_overlap_seat", "bearing retainer holes overlap the bearing seat")
        if cp.edge_offset <= 0:
            raise GeometryError(f"{name}.edge_offset", "cap edge_offset must be > 0 (screw holes would break through the edge)")
    else:
        if ro >= b.OD / 2:
            print("warning: support flange / cap reaches the bearing outer race")
        if rh.R + rh.r >= b.ID / 2:
            raise GeometryError(f"{name}.holes_break_journal", "bearing retainer holes break through the journal")
        if cp.edge_offset <= 0:
            raise GeometryError(f"{name}.edge_offset", "cap edge_offset must be > 0 (screw holes would break into the bore)")
        if ri < 0:
            raise GeometryError(f"{name}.no_bore", "inner-race cap edge_offset leaves no bore (holes - edge_offset < 0)")


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
