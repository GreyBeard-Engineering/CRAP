from build123d import *
from part_generation_fcns.classdefs import *

# ISO 4762 socket head cap screw proportions: d -> (head_d, head_h, socket_af)
_SHCS = {
    2.0: (3.8, 2.0, 1.5),
    2.5: (4.5, 2.5, 2.0),
    3.0: (5.5, 3.0, 2.5),
    4.0: (7.0, 4.0, 3.0),
    5.0: (8.5, 5.0, 4.0),
    6.0: (10.0, 6.0, 5.0),
}


def bored_cylinder(r_outer: float, r_inner: float, h: float) -> Part:
    """Tube from z=0 to z=h. r_inner=0 gives a solid cylinder."""
    with BuildPart() as part:
        with BuildSketch():
            Circle(r_outer)
            if r_inner > 0:
                Circle(r_inner, mode=Mode.SUBTRACT)
        extrude(amount=h)
    return part.part


def socket_head_bolt(d: float, length: float) -> bolt:
    """Socket head cap screw of shank diameter d; ISO 4762 head where tabulated."""
    head_d, head_h, af = _SHCS.get(round(d, 2), (1.6 * d + 0.6, d, 0.8 * d))
    return bolt(d=d, length=length, head_d=head_d, head_h=head_h, socket_af=af)


def bolt_name(b: bolt) -> str:
    """'M3x13.5' for metric sizes; 'D3.175x14.5' for anything else. Length to 0.1 mm."""
    size = f"M{b.d:g}" if round(b.d, 2) in _SHCS else f"D{b.d:g}"
    return f"{size}x{round(b.length, 1):g}"


def make_bolt(b: bolt) -> Part:
    """Head underside at z=0, head up to z=head_h, shank down to z=-length."""
    with BuildPart() as part:
        with BuildSketch():
            Circle(b.head_d / 2)
            RegularPolygon(b.socket_af / 2, 6, major_radius=False, mode=Mode.SUBTRACT)
        extrude(amount=b.head_h)
        with BuildSketch():
            Circle(b.head_d / 2)
        extrude(amount=b.head_h - b.d / 2)           # fill socket below its depth
        with BuildSketch():
            Circle(b.d / 2)
        extrude(amount=-b.length)
    return part.part


def make_bearing(b: bearing) -> Part:
    return bored_cylinder(b.OD / 2, b.ID / 2, b.th)


def spacer_r(b: bearing) -> float:
    """Spacer OD that bears on the inner race only: midway between ID and OD."""
    return (b.ID + b.OD) / 4


def make_spacer(b: bearing, h: float, bore_delta: float = 0.0) -> Part:
    """Spacer riding on a bearing's pin, contacting only its inner race."""
    return bored_cylinder(spacer_r(b), b.ID / 2 + bore_delta, h)
