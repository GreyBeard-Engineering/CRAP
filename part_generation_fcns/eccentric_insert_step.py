from build123d import *
from part_generation_fcns.classdefs import *
from part_generation_fcns.errors import GeometryError, geometry_issue
from part_generation_fcns.paths import ensure_parent
from part_generation_fcns.tolerancing import NOMINAL, tolerancing


def eccentric_insert_thickness(ei: eccentric_insert) -> float:
    return ei.flange.thickness + ei.input_bearing.th


def make_eccentric_insert(ei: eccentric_insert, tol: tolerancing = NOMINAL) -> Part:
    """Revolved L: flange supporting the bearing inner race, journal through the bearing,
    and a motor shaft hole offset from the journal axis by e."""
    rj = ei.input_bearing.ID / 2           # journal
    rf = rj + ei.flange.offset             # flange
    tf, H = ei.flange.thickness, eccentric_insert_thickness(ei)
    rjt = rj + tol.delta("eccentric_insert.journal")

    if ei.motor.shaft_r + ei.e >= rj:
        geometry_issue("insert.shaft_hole_breaks_journal", "motor shaft hole breaks through the journal (shaft_r + e >= input_bearing.ID/2)")
    if rf >= ei.input_bearing.OD / 2:
        print("warning: insert flange reaches the input bearing outer race")

    with BuildPart() as part:
        # (r, z) cross-section, revolved about the journal axis
        with BuildSketch(Plane.XZ):
            with BuildLine():
                Polyline((0, 0), (rf, 0), (rf, tf), (rjt, tf), (rjt, H), (0, H), close=True)
            make_face()
        revolve(axis=Axis.Z)

        with BuildSketch():
            with Locations((ei.e, 0)):
                Circle(ei.motor.shaft_r + tol.delta("eccentric_insert.shaft_hole"))
        extrude(amount=H, mode=Mode.SUBTRACT)
    return part.part


def make_eccentric_insert_step(ei: eccentric_insert, path: str = "steps/eccentric_insert.step") -> Part:
    part = make_eccentric_insert(ei)
    part.label = "eccentric_insert"
    export_step(part, ensure_parent(path))
    return part


if __name__ == "__main__":
    ei = eccentric_insert(
        input_bearing=bearing(ID=10, OD=19, th=5),                 # 6800-ish
        flange=flange(thickness=12.0, offset=1.5),
        motor=motor(shaft_r=2.5, mounting_holes=patterned_hole(r=1.6, R=21.92, N=4)),  # NEMA17
        e=1.2,
    )
    part = make_eccentric_insert_step(ei)
    print("volume", part.volume, "valid", part.is_valid)
