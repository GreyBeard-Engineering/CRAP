import math
import os
from build123d import Compound, Location, Part, export_step, export_stl
from part_generation_fcns.classdefs import *
from part_generation_fcns.base_step import make_base
from part_generation_fcns.top_step import make_top, _radii as top_radii
from part_generation_fcns.cycloid_step import make_cycloid, cycloid_thickness
from part_generation_fcns.eccentric_insert_step import make_eccentric_insert
from part_generation_fcns.output_step import make_output, output_height
from part_generation_fcns.cap_step import cap_radii, make_cap
from part_generation_fcns.hardware import bored_cylinder, bolt_name, make_bearing, make_bolt, make_spacer, socket_head_bolt
from part_generation_fcns.tolerancing import NOMINAL, tolerancing


def _polar(R: float, N: int):
    """(x, y, angle_deg) of N points on radius R, first on +X (matches PolarLocations)."""
    return [(R * math.cos(math.radians(a)), R * math.sin(math.radians(a)), a)
            for a in (360 * k / N for k in range(N))]


class gearbox:
    """Cycloidal gearbox. Dimensions shared between parts (roller bearing, roller ring
    radius and count, housing radius) are given once and propagated to each part.

    Axial stack: base | gap | cycloid + cycloid cap | gap | output plate | gap | top,
    every gap = plate_spacing.

    Constructing a gearbox builds every part and, unless assembly_path is None,
    writes them all, in assembled position, to one STEP file."""

    def __init__(
        self, *,
        # shared
        ratio: int,
        roller_bearing: bearing,
        roller_R: float,
        outer_r: float,
        plate_spacing: float, # axial gap base-cycloid, cycloid cap-output, output-top
        # cycloid
        e: float,
        input_bearing: bearing,
        output_needle_roller: bearing,
        output_holes_R: float,
        output_holes_N: int,
        cycloid_cap: cap,
        # base
        drive_motor: motor,
        shaft_hole_offset: float,
        base_t: float,
        mounting_holes: patterned_hole,
        mounting_hole_offset: float,
        mounting_hole_fillet_r: float,
        # top
        output_bearing: bearing,
        output_bearing_offset: float,
        top_base_thickness: float,
        top_cap: cap,
        # eccentric insert
        eccentric_insert_flange_offset: float,
        # output
        output_flange_offset: float,
        output_plate_r: float,
        output_plate_t: float,
        output_bore_r: float,
        output_cap: cap, # inner-race cap; r is its inner radius
        # export
        assembly_path: str | None = "gearbox.step",
    ):
        self.plate_spacing = plate_spacing
        self.cycloid = cycloid(
            roller_bearing=roller_bearing,
            roller_R=roller_R,
            input_bearing=input_bearing,
            output_needle_roller=output_needle_roller,
            output_holes_R=output_holes_R,
            output_holes_N=output_holes_N,
            e=e,
            ratio=ratio,
            cap=cycloid_cap,
        )
        self.base = base(
            drive_motor=drive_motor,
            shaft_hole_offset=shaft_hole_offset,
            roller_bearing=roller_bearing,
            roller_holes_R=roller_R,
            roller_holes_N=ratio + 1,
            outer_r=outer_r,
            t=base_t,
            mounting_holes=mounting_holes,
            mounting_hole_offset=mounting_hole_offset,
            mounting_hole_fillet_r=mounting_hole_fillet_r,
        )
        self.top = top(
            output_bearing=output_bearing,
            output_bearing_offset=output_bearing_offset,
            base_thickness=top_base_thickness,
            roller_bearing=roller_bearing,
            roller_holes_R=roller_R,
            roller_holes_N=ratio + 1,
            cap=top_cap,
            outer_r=outer_r,
        )
        # insert flange spans a gap below the base, the base, the base-cycloid gap,
        # and the cycloid's support lip
        self.eccentric_insert = eccentric_insert(
            input_bearing=input_bearing,
            flange=flange(
                thickness=plate_spacing + base_t + plate_spacing + cycloid_cap.flange.thickness,
                offset=eccentric_insert_flange_offset,
            ),
            motor=drive_motor,
            e=e,
        )
        # output shoulder spans the output-top gap and the top's support lip,
        # so the output bearing lands on both the shoulder and the lip
        self.output = output(
            output_bearing=output_bearing,
            flange=flange(
                thickness=plate_spacing + top_cap.flange.thickness,
                offset=output_flange_offset,
            ),
            plate_r=output_plate_r,
            plate_t=output_plate_t,
            output_needle_roller=output_needle_roller,
            output_holes_R=output_holes_R,
            output_holes_N=output_holes_N,
            bore_r=output_bore_r,
            cap=output_cap,
        )
        self._check_assembly()
        self._layout_cache = self._layout()
        self.parts = self.build()
        if assembly_path is not None:
            self.export_assembly(assembly_path)

    def _check_assembly(self):
        insert_r = self.eccentric_insert.input_bearing.ID / 2 + self.eccentric_insert.flange.offset
        base_hole_r = self.base.drive_motor.shaft_r + self.base.shaft_hole_offset
        cycloid_lip_r = self.cycloid.input_bearing.OD / 2 - self.cycloid.cap.flange.offset
        if insert_r + self.eccentric_insert.e >= base_hole_r:
            raise ValueError("eccentric insert flange hits the base center hole as it orbits")
        if insert_r >= cycloid_lip_r:
            raise ValueError("eccentric insert flange hits the cycloid's bearing support lip")

        shoulder_r = self.output.output_bearing.ID / 2 + self.output.flange.offset
        top_lip_r = self.top.output_bearing.OD / 2 - self.top.cap.flange.offset
        if shoulder_r >= top_lip_r:
            raise ValueError("output shoulder hits the top's bearing support lip")
        if self.output.plate_r >= self.cycloid.roller_R - self.cycloid.roller_bearing.OD / 2:
            raise ValueError("output plate hits the ring rollers")
        # output cap and top cap sit in the same plane (journal top = tower top)
        if cap_radii(self.output.cap, self.output.output_bearing, "inner")[1] >= \
           cap_radii(self.top.cap, self.top.output_bearing, "outer")[0]:
            raise ValueError("output cap hits the top cap")

    def stack(self) -> dict[str, float]:
        """Z of the underside of each plate-like part. z=0 is the underside of the base."""
        sp = self.plate_spacing
        z = {"base": 0.0, "eccentric_insert": -sp}  # insert hangs plate_spacing below the base
        z["cycloid"] = self.base.t + sp
        z["cycloid_cap"] = z["cycloid"] + cycloid_thickness(self.cycloid)
        z["output"] = z["cycloid_cap"] + self.cycloid.cap.flange.thickness + sp
        z["top"] = z["output"] + self.output.plate_t + sp
        z["top_cap"] = z["top"] + top_radii(self.top)[3]
        z["output_cap"] = z["output"] + output_height(self.output)
        z["cycloid_mid"] = z["cycloid"] + cycloid_thickness(self.cycloid) / 2
        return z

    def _layout(self, tol: tolerancing = NOMINAL) -> dict[str, tuple[Part, list[Location]]]:
        """Every unique part, built at its own origin, with the location of each instance.
        The motor axis is Z; the cycloid sits offset by +e along X, meshing with the roller at +X.
        tol applies export fits; the default builds the nominal model."""
        z, e = self.stack(), self.cycloid.e
        c, tp, o = self.cycloid, self.top, self.output
        L = {}

        # custom parts
        L["base"] = (make_base(self.base, tol), [Location((0, 0, z["base"]))])
        # insert's shaft hole is at local +e; turn it 180 deg so the hole lands on the motor axis
        L["eccentric_insert"] = (make_eccentric_insert(self.eccentric_insert, tol),
                                 [Location((e, 0, z["eccentric_insert"]), (0, 0, 180))])
        L["cycloid"] = (make_cycloid(c, tol), [Location((e, 0, z["cycloid"]))])
        L["cycloid_cap"] = (make_cap(c.cap, c.input_bearing, hole_delta=tol.delta("cycloid_cap.holes")), [Location((e, 0, z["cycloid_cap"]))])
        L["output"] = (make_output(o, tol), [Location((0, 0, z["output"]))])
        L["output_cap"] = (make_cap(o.cap, o.output_bearing, race="inner", hole_delta=tol.delta("output_cap.holes")), [Location((0, 0, z["output_cap"]))])
        L["top"] = (make_top(tp, tol), [Location((0, 0, z["top"]))])
        L["top_cap"] = (make_cap(tp.cap, tp.output_bearing, hole_delta=tol.delta("top_cap.holes")), [Location((0, 0, z["top_cap"]))])

        # main bearings, seated on their support lips
        L["input_bearing"] = (make_bearing(c.input_bearing),
                              [Location((e, 0, z["cycloid"] + c.cap.flange.thickness))])
        L["output_bearing"] = (make_bearing(tp.output_bearing),
                               [Location((0, 0, z["top"] + tp.cap.flange.thickness))])

        # ring rollers: midplane on the cycloid midplane, spacers down to the base and up to
        # the top, all on a bolt from under the base through the top's base plate
        rb = c.roller_bearing
        z_rb0 = z["cycloid_mid"] - rb.th / 2
        z_rb1 = z_rb0 + rb.th
        ring = [(x, y) for x, y, _ in _polar(c.roller_R, c.ratio + 1)]
        L["roller_bearing"] = (make_bearing(rb), [Location((x, y, z_rb0)) for x, y in ring])
        L["roller_spacer_bottom"] = (make_spacer(rb, z_rb0 - self.base.t, tol.delta("roller_spacer.bore")),
                                     [Location((x, y, self.base.t)) for x, y in ring])
        L["roller_spacer_top"] = (make_spacer(rb, z["top"] - z_rb1, tol.delta("roller_spacer.bore")),
                                  [Location((x, y, z_rb1)) for x, y in ring])
        # head on the top's base plate; passes through the top, spacers and bearing and
        # threads into the base
        z_ring_head = z["top"] + tp.base_thickness
        ring_bolt = socket_head_bolt(rb.ID, z_ring_head - z["base"])
        L[f"ring_bolt_{bolt_name(ring_bolt)}"] = (make_bolt(ring_bolt),
                                                  [Location((x, y, z_ring_head)) for x, y in ring])

        # output needles: roller midplane on the cycloid midplane, spacer up to the output
        # plate, pin from the roller bottom up through the plate
        nr = o.output_needle_roller
        z_nr0 = z["cycloid_mid"] - nr.th / 2
        z_nr1 = z_nr0 + nr.th
        pins = [(x, y) for x, y, _ in _polar(o.output_holes_R, o.output_holes_N)]
        L["needle_roller"] = (make_bearing(nr), [Location((x, y, z_nr0)) for x, y in pins])
        L["needle_spacer"] = (make_spacer(nr, z["output"] - z_nr1, tol.delta("needle_spacer.bore")), [Location((x, y, z_nr1)) for x, y in pins])
        L["needle_pin"] = (bored_cylinder(nr.ID / 2, 0, z["output"] + o.plate_t - z_nr0),
                           [Location((x, y, z_nr0)) for x, y in pins])

        # cap screws: head on the cap, engaging the part below by the bearing thickness
        for name, cp, b, cx in (("cycloid_cap", c.cap, c.input_bearing, e),
                                ("output_cap", o.cap, o.output_bearing, 0.0),
                                ("top_cap", tp.cap, tp.output_bearing, 0.0)):
            rh = cp.bearing_retainer_holes
            bt = socket_head_bolt(2 * rh.r, cp.flange.thickness + b.th)
            z_head = z[name] + cp.flange.thickness
            L[f"{name}_bolt_{bolt_name(bt)}"] = (make_bolt(bt),
                [Location((cx + x, y, z_head)) for x, y, _ in _polar(rh.R, rh.N)])

        for name, (part, _) in L.items():
            part.label = name
        return L

    def build(self) -> dict[str, Part]:
        """Every unique part (nominal) at its own origin; returns {name: Part}."""
        return {name: part for name, (part, _) in self._layout_cache.items()}

    def locations(self) -> dict[str, list[Location]]:
        """Assembled location of every instance of each part."""
        return {name: locs for name, (_, locs) in self._layout_cache.items()}

    # stock (bought) parts by name prefix; everything else is custom: made to this model's
    # dimensions, whether printed or machined
    _STOCK = ("input_bearing", "output_bearing", "roller_bearing", "needle_roller", "needle_pin",
              "ring_bolt", "cycloid_cap_bolt", "output_cap_bolt", "top_cap_bolt")

    def part_kinds(self) -> dict[str, str]:
        """{part name: "custom" | "stock"}."""
        return {name: "stock" if any(name == k or name.startswith(k + "_") for k in self._STOCK)
                else "custom" for name in self._layout_cache}

    # which rigid body each part moves with in operation (see web GUI)
    _MOTION_GROUPS = {
        "base": "housing", "top": "housing", "top_cap": "housing", "output_bearing": "housing",
        "roller_bearing": "housing", "roller_spacer_bottom": "housing",
        "roller_spacer_top": "housing", "ring_bolt": "housing", "top_cap_bolt": "housing",
        "eccentric_insert": "input",
        "cycloid": "cycloid", "cycloid_cap": "cycloid", "input_bearing": "cycloid",
        "cycloid_cap_bolt": "cycloid",
        "output": "output", "output_cap": "output", "needle_roller": "output",
        "needle_spacer": "output", "needle_pin": "output", "output_cap_bolt": "output",
    }

    def motion_groups(self) -> dict[str, str]:
        """{part name: "housing" | "input" | "cycloid" | "output"}. Bolt names carry a size
        suffix, so the longest matching prefix wins."""
        out = {}
        for name in self._layout_cache:
            keys = [k for k in self._MOTION_GROUPS if name == k or name.startswith(k + "_")]
            if not keys:
                raise KeyError(f"part {name!r} has no motion group")
            out[name] = self._MOTION_GROUPS[max(keys, key=len)]
        return out

    def assembly(self, layout: dict | None = None) -> Compound:
        children = []
        for name, (part, locs) in (layout or self._layout_cache).items():
            for i, loc in enumerate(locs):
                placed = part.moved(loc)
                placed.label = name if len(locs) == 1 else f"{name}_{i + 1}"
                children.append(placed)
        return Compound(children=children, label="gearbox")

    def export_assembly(self, path: str = "gearbox.step") -> Compound:
        """Write all parts (nominal), in assembled position, to one STEP file."""
        if os.path.dirname(path):
            os.makedirs(os.path.dirname(path), exist_ok=True)
        asm = self.assembly()
        export_step(asm, path)
        return asm

    def export(self, out_dir: str, tol: tolerancing = NOMINAL, fmt: str = "step",
               scope: str = "all") -> list[str]:
        """Write each unique part, at its own origin, to out_dir; returns the file paths.
        fmt: "step" (also writes the assembled gearbox.step) or "stl".
        scope: "all" or "custom" (skip stock bearings, bolts and pins)."""
        if fmt not in ("step", "stl"):
            raise ValueError(f"fmt must be 'step' or 'stl', got {fmt!r}")
        if scope not in ("all", "custom"):
            raise ValueError(f"scope must be 'all' or 'custom', got {scope!r}")
        layout = self._layout_cache if not tol.enabled else self._layout(tol)
        kinds = self.part_kinds()
        keep = {n: v for n, v in layout.items() if scope == "all" or kinds[n] == "custom"}
        os.makedirs(out_dir, exist_ok=True)
        paths = []
        for name, (part, _) in keep.items():
            path = os.path.join(out_dir, f"{name}.{fmt}")
            if fmt == "step":
                export_step(part, path)
            else:
                export_stl(part, path, tolerance=0.01, angular_tolerance=0.1)
            paths.append(path)
        if fmt == "step":
            path = os.path.join(out_dir, "gearbox.step")
            export_step(self.assembly(keep), path)
            paths.append(path)
        return paths

    def generate(self, out_dir: str = ".") -> dict[str, Part]:
        """Write every nominal part, at its own origin, to out_dir as separate STEP files."""
        self.export(out_dir)
        return self.parts


# example gearbox: NEMA17 drive, 23:1
EXAMPLE_KWARGS = dict(
    ratio=23,
    roller_bearing=bearing(ID=2, OD=5, th=2.5),                # MR52-ish
    roller_R=40.0,
    outer_r=48.0,
    plate_spacing=1.0,

    e=1.2,
    input_bearing=bearing(ID=10, OD=19, th=5),                 # 6800-ish
    output_needle_roller=bearing(ID=3, OD=6, th=2.5),          # MR63-ish
    output_holes_R=24.0,
    output_holes_N=6,
    cycloid_cap=cap(
        flange=flange(thickness=2.0, offset=1.5),
        bearing_retainer_holes=patterned_hole(r=1.0, R=12.0, N=4),   # M2 pilot
        r=15.0,
    ),

    drive_motor=motor(
        shaft_r=2.5,                                           # NEMA17
        mounting_holes=patterned_hole(r=1.6, R=21.92, N=4),    # M3 on 31 mm square
    ),
    shaft_hole_offset=9.0,
    base_t=5.0,
    mounting_holes=patterned_hole(r=2.2, R=55.0, N=2),         # M4
    mounting_hole_offset=3.0,
    mounting_hole_fillet_r=3.0,

    output_bearing=bearing(ID=25, OD=37, th=7),                # 6805-ish
    output_bearing_offset=6.0,
    top_base_thickness=4.0,
    top_cap=cap(
        flange=flange(thickness=2.0, offset=2.0),
        bearing_retainer_holes=patterned_hole(r=1.25, R=21.5, N=6),  # M2.5 pilot
        r=24.5,
    ),

    eccentric_insert_flange_offset=1.5,

    output_flange_offset=2.0,
    output_plate_r=34.0,
    output_plate_t=4.0,
    output_bore_r=6.0,
    output_cap=cap(
        flange=flange(thickness=2.0, offset=1.5),
        bearing_retainer_holes=patterned_hole(r=1.0, R=9.5, N=4),    # M2 pilot
        r=6.0,                                                # matches bore
    ),
)


if __name__ == "__main__":
    gb = gearbox(**EXAMPLE_KWARGS, assembly_path="gearbox_out/gearbox.step")
    for name, part in gb.generate("gearbox_out").items():
        print(f"{name}: volume={part.volume:.1f} valid={part.is_valid}")
