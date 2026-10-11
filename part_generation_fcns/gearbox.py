import dataclasses
import math
import os
from build123d import Compound, Location, Part, export_step, export_stl
from part_generation_fcns.classdefs import *
from part_generation_fcns.errors import collecting_issues, geometry_issue
from part_generation_fcns.base_step import make_base
from part_generation_fcns.top_step import make_top, _radii as top_radii
from part_generation_fcns.cycloid_step import cycloid_thickness, make_cycloid, make_cycloid_cap
from part_generation_fcns.eccentric_insert_step import make_eccentric_insert
from part_generation_fcns.output_step import make_output, output_height, output_plate_r
from part_generation_fcns.cap_step import cap_radii, make_cap
from part_generation_fcns.hardware import bolt_name, bored_cylinder, make_bearing, make_bolt, make_spacer, socket_head_bolt
from part_generation_fcns.tolerancing import NOMINAL, tolerancing


def _polar(R: float, N: int, start: float = 0.0):
    """(x, y, angle_deg) of N points on radius R, first at `start` degrees from +X (matches
    PolarLocations)."""
    return [(R * math.cos(math.radians(a)), R * math.sin(math.radians(a)), a)
            for a in (start + 360 * k / N for k in range(N))]


class gearbox:
    """Cycloidal gearbox. Dimensions shared between parts (roller bearing, roller ring
    radius and count, housing radius) are given once and propagated to each part.

    Axial stack: base | base_cycloid_spacing | cycloid + cycloid cap | cap_output_spacing |
    output plate | output_top_spacing | top.

    Constructing a gearbox builds every part and, unless assembly_path is None,
    writes them all, in assembled position, to one STEP file."""

    def __init__(
        self, *,
        # shared
        ratio: int,
        roller_bearing: bearing,
        roller_R: float,
        housing_edge_offset: float, # base/top edge beyond the roller holes' outer edge
        base_cycloid_spacing: float, # axial gap: top of base -> bottom of cycloid
        cap_output_spacing: float, # axial gap: top of cycloid cap -> bottom of output plate
        output_top_spacing: float, # axial gap: top of output plate -> bottom of top
        inter_disk_spacing: float = 3.0, # axial gap: top of cycloid cap -> bottom of next disc
        num_disks: int = 1, # number of stacked cycloid discs
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
        base_thickness: float = None,
        base_t: float | None = None,
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
        output_plate_edge_offset: float, # plate edge beyond the needle screw holes' outer edge
        output_plate_thickness: float = None,
        output_plate_t: float | None = None,
        output_bore_r: float,
        output_cap: cap, # inner-race cap; r is its inner radius
        # export
        assembly_path: str | None = "steps/gearbox/gearbox.step",
        strict: bool = True, # False: record geometry problems in .issues and build what it can
    ):
        if base_thickness is None:
            if base_t is not None:
                base_thickness = base_t
            else:
                raise TypeError("gearbox missing required argument: 'base_thickness'")
        if output_plate_thickness is None:
            if output_plate_t is not None:
                output_plate_thickness = output_plate_t
            else:
                raise TypeError("gearbox missing required argument: 'output_plate_thickness'")

        self.num_disks = num_disks
        self.inter_disk_spacing = inter_disk_spacing
        self.base_cycloid_spacing = base_cycloid_spacing
        self.cap_output_spacing = cap_output_spacing
        self.output_top_spacing = output_top_spacing
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
            edge_offset=housing_edge_offset,
            thickness=base_thickness,
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
            edge_offset=housing_edge_offset,
        )
        # insert flange hangs base_cycloid_spacing below the base, then spans the base, the
        # base-cycloid gap and the cycloid's support lip
        self.eccentric_insert = eccentric_insert(
            input_bearing=input_bearing,
            flange=flange(
                thickness=base_cycloid_spacing + base_thickness + base_cycloid_spacing + cycloid_cap.flange.thickness,
                offset=eccentric_insert_flange_offset,
            ),
            motor=drive_motor,
            e=e,
        )
        # inter-disk eccentric insert for subsequent discs:
        # bottom thickness = cycloid flange thickness
        # thickness above flange = cycloid thickness - cycloid flange thickness + cycloid cap thickness
        self.eccentric_insert_stacked = eccentric_insert(
            input_bearing=input_bearing,
            flange=flange(
                thickness=cycloid_cap.flange.thickness,
                offset=eccentric_insert_flange_offset,
            ),
            motor=drive_motor,
            e=e,
            journal_height=input_bearing.thickness + cycloid_cap.flange.thickness,
        )
        # output shoulder spans the output-top gap and the top's support lip,
        # so the output bearing lands on both the shoulder and the lip
        self.output = output(
            output_bearing=output_bearing,
            flange=flange(
                thickness=output_top_spacing + top_cap.flange.thickness,
                offset=output_flange_offset,
            ),
            plate_edge_offset=output_plate_edge_offset,
            plate_thickness=output_plate_thickness,
            output_needle_roller=output_needle_roller,
            output_holes_R=output_holes_R,
            output_holes_N=output_holes_N,
            bore_r=output_bore_r,
            cap=output_cap,
        )
        self.strict = strict
        self.issues: list[tuple[str, str]] = []
        if strict:
            self._check_assembly()
            self._layout_cache = self._layout()
        else:
            with collecting_issues() as found:
                self._check_assembly()
                self._layout_cache = self._layout()
            self.issues = found
        self.parts = self.build()
        if assembly_path is not None:
            self.export_assembly(assembly_path)

    def _check_assembly(self):
        insert_r = self.eccentric_insert.input_bearing.ID / 2 + self.eccentric_insert.flange.offset
        base_hole_r = self.base.drive_motor.shaft_r + self.base.shaft_hole_offset
        cycloid_lip_r = self.cycloid.input_bearing.OD / 2 - self.cycloid.cap.flange.offset
        if insert_r + self.eccentric_insert.e >= base_hole_r:
            geometry_issue("assembly.insert_hits_base_hole", "eccentric insert flange hits the base center hole as it orbits")
        if insert_r >= cycloid_lip_r:
            geometry_issue("assembly.insert_hits_cycloid_lip", "eccentric insert flange hits the cycloid's bearing support lip")

        rh = self.cycloid.cap.bearing_retainer_holes
        cap_screw = socket_head_bolt(2 * rh.r, 1.0)
        if self.num_disks > 1 and self.inter_disk_spacing < cap_screw.head_h:
            geometry_issue("assembly.inter_disk_spacing_hits_screws", "inter_disk_spacing is smaller than the cycloid cap screw head height")

        shoulder_r = self.output.output_bearing.ID / 2 + self.output.flange.offset
        top_lip_r = self.top.output_bearing.OD / 2 - self.top.cap.flange.offset
        if shoulder_r >= top_lip_r:
            geometry_issue("assembly.shoulder_hits_top_lip", "output shoulder hits the top's bearing support lip")
        if output_plate_r(self.output) >= self.cycloid.roller_R - self.cycloid.roller_bearing.OD / 2:
            geometry_issue("assembly.plate_hits_rollers", "output plate hits the ring rollers")
        # output cap and top cap sit in the same plane (journal top = tower top)
        if cap_radii(self.output.cap, self.output.output_bearing, "inner")[1] >= \
           cap_radii(self.top.cap, self.top.output_bearing, "outer")[0]:
            geometry_issue("assembly.output_cap_hits_top_cap", "output cap hits the top cap")

    def stack(self) -> dict[str, float | list[float]]:
        """Z of the underside of each plate-like part. z=0 is the underside of the base."""
        sbc = self.base_cycloid_spacing
        t_cyc = cycloid_thickness(self.cycloid)
        t_cap = self.cycloid.cap.flange.thickness
        delta_disk = t_cyc + t_cap + self.inter_disk_spacing

        z_cycloids = [self.base.thickness + sbc + k * delta_disk for k in range(self.num_disks)]
        z_caps = [zc + t_cyc for zc in z_cycloids]
        z_mids = [zc + t_cyc / 2 for zc in z_cycloids]

        z = {
            "base": 0.0,
            "eccentric_insert": -sbc,
            "cycloid": z_cycloids[0],
            "cycloids": z_cycloids,
            "cycloid_cap": z_caps[0],
            "cycloid_caps": z_caps,
            "cycloid_mid": z_mids[0],
            "cycloid_mids": z_mids,
        }
        z["output"] = z_caps[-1] + t_cap + self.cap_output_spacing
        z["top"] = z["output"] + self.output.plate_thickness + self.output_top_spacing
        z["top_cap"] = z["top"] + top_radii(self.top)[3]
        z["output_cap"] = z["output"] + output_height(self.output)
        return z

    def _layout(self, tol: tolerancing = NOMINAL) -> dict[str, tuple[Part, list[Location]]]:
        """Every unique part, built at its own origin, with the location of each instance.
        The motor axis is Z; the cycloid sits offset by +e along X, meshing with the roller at +X.
        tol applies export fits; the default builds the nominal model."""
        z, e = self.stack(), self.cycloid.e
        c, tp, o = self.cycloid, self.top, self.output
        N = self.num_disks
        L = {}

        z_cycloids = z["cycloids"]
        z_caps = z["cycloid_caps"]
        z_mids = z["cycloid_mids"]

        # custom parts
        L["base"] = (self._try(lambda: make_base(self.base, tol), "base"), [Location((0, 0, z["base"]))])
        # insert's shaft hole is at local +e; turn it 180 deg so the hole lands on the motor axis
        L["eccentric_insert"] = (self._try(lambda: make_eccentric_insert(self.eccentric_insert, tol), "eccentric_insert"),
                                 [Location((e, 0, z["eccentric_insert"]), (0, 0, 180))])
        if N > 1:
            locs_stacked = []
            for k in range(1, N):
                phi_k = k * 360.0 / N
                rad = math.radians(phi_k)
                z_k = z_cycloids[k]
                locs_stacked.append(Location((e * math.cos(rad), e * math.sin(rad), z_k), (0, 0, 180 + phi_k)))
            L["eccentric_insert_stacked"] = (
                self._try(lambda: make_eccentric_insert(self.eccentric_insert_stacked, tol), "eccentric_insert_stacked"),
                locs_stacked
            )

            r_sp_inner = self.base.drive_motor.shaft_r
            r_sp_outer = c.input_bearing.ID / 2 - e
            # spacer between insert 0 and insert 1: spans from top of insert 0 (z_caps[0])
            # through cap 0 to the bottom of insert 1 (z_cycloids[1])
            h_sp_1 = z_cycloids[1] - z_caps[0]
            L["inter_eccentric_spacer"] = (
                self._try(lambda: bored_cylinder(r_sp_outer, r_sp_inner, h_sp_1), "inter_eccentric_spacer"),
                [Location((0, 0, z_caps[0]))]
            )
            if N > 2:
                h_sp_k = self.inter_disk_spacing
                locs_sp_k = [Location((0, 0, z_caps[k - 1] + c.cap.flange.thickness)) for k in range(2, N)]
                L["inter_eccentric_spacer_upper"] = (
                    self._try(lambda: bored_cylinder(r_sp_outer, r_sp_inner, h_sp_k), "inter_eccentric_spacer_upper"),
                    locs_sp_k
                )

        locs_cycloid = []
        locs_cap = []
        locs_ib = []
        for k in range(N):
            phi_k = k * 360.0 / N
            psi_k = phi_k
            rad = math.radians(phi_k)
            cx, cy = e * math.cos(rad), e * math.sin(rad)
            locs_cycloid.append(Location((cx, cy, z_cycloids[k]), (0, 0, psi_k)))
            locs_cap.append(Location((cx, cy, z_caps[k]), (0, 0, psi_k)))
            locs_ib.append(Location((cx, cy, z_cycloids[k] + c.cap.flange.thickness)))

        L["cycloid"] = (self._try(lambda: make_cycloid(c, tol), "cycloid"), locs_cycloid)
        L["cycloid_cap"] = (self._try(lambda: make_cycloid_cap(c, tol), "cycloid_cap"), locs_cap)
        L["input_bearing"] = (self._try(lambda: make_bearing(c.input_bearing), "input_bearing"), locs_ib)

        L["output"] = (self._try(lambda: make_output(o, tol), "output"), [Location((0, 0, z["output"]))])
        L["output_cap"] = (self._try(lambda: make_cap(o.cap, o.output_bearing, race="inner", hole_delta=tol.delta("output_cap.holes")), "output_cap"), [Location((0, 0, z["output_cap"]))])
        L["top"] = (self._try(lambda: make_top(tp, tol), "top"), [Location((0, 0, z["top"]))])
        L["top_cap"] = (self._try(lambda: make_cap(tp.cap, tp.output_bearing, hole_delta=tol.delta("top_cap.holes")), "top_cap"), [Location((0, 0, z["top_cap"]))])

        L["output_bearing"] = (self._try(lambda: make_bearing(tp.output_bearing), "output_bearing"),
                               [Location((0, 0, z["top"] + tp.cap.flange.thickness))])

        # ring rollers: midplane on each cycloid midplane, spacers down to the base and up to
        # the top, with inter-roller spacers between adjacent discs
        rb = c.roller_bearing
        ring = [(x, y) for x, y, _ in _polar(c.roller_R, c.ratio + 1)]
        z_rb0 = z_mids[0] - rb.thickness / 2
        z_rb_last = z_mids[-1] + rb.thickness / 2

        locs_rb = []
        for k in range(N):
            z_rb_k = z_mids[k] - rb.thickness / 2
            locs_rb.extend([Location((x, y, z_rb_k)) for x, y in ring])
        L["roller_bearing"] = (self._try(lambda: make_bearing(rb), "roller_bearing"), locs_rb)

        L["roller_spacer_bottom"] = (
            self._try(lambda: make_spacer(rb, z_rb0 - self.base.thickness, tol.delta("roller_spacer.bore")), "roller_spacer_bottom"),
            [Location((x, y, self.base.thickness)) for x, y in ring]
        )

        if N > 1:
            h_inter = (z_mids[1] - z_mids[0]) - rb.thickness
            locs_inter_spacer = []
            for k in range(1, N):
                z_sp = z_mids[k - 1] + rb.thickness / 2
                locs_inter_spacer.extend([Location((x, y, z_sp)) for x, y in ring])
            L["inter_roller_spacer"] = (
                self._try(lambda: make_spacer(rb, h_inter, tol.delta("roller_spacer.bore")), "inter_roller_spacer"),
                locs_inter_spacer
            )

        L["roller_spacer_top"] = (
            self._try(lambda: make_spacer(rb, z["top"] - z_rb_last, tol.delta("roller_spacer.bore")), "roller_spacer_top"),
            [Location((x, y, z_rb_last)) for x, y in ring]
        )

        z_ring_head = z["top"] + tp.base_thickness
        ring_bolt = socket_head_bolt(rb.ID, z_ring_head - z["base"])
        L[f"ring_bolt_{bolt_name(ring_bolt)}"] = (
            self._try(lambda: make_bolt(ring_bolt), f"ring_bolt_{bolt_name(ring_bolt)}"),
            [Location((x, y, z_ring_head)) for x, y in ring]
        )

        # output needles: roller midplane on each cycloid midplane, spacer up to the output
        # plate; a screw goes in from below, head under the roller, and threads into the plate
        nr = o.output_needle_roller
        pins = [(x, y) for x, y, _ in _polar(o.output_holes_R, o.output_holes_N)]
        z_nr0 = z_mids[0] - nr.thickness / 2
        z_nr_last = z_mids[-1] + nr.thickness / 2

        locs_nr = []
        for k in range(N):
            z_nr_k = z_mids[k] - nr.thickness / 2
            locs_nr.extend([Location((x, y, z_nr_k)) for x, y in pins])
        L["needle_roller"] = (self._try(lambda: make_bearing(nr), "needle_roller"), locs_nr)

        if N > 1:
            h_nr_inter = (z_mids[1] - z_mids[0]) - nr.thickness
            locs_nr_inter = []
            for k in range(1, N):
                z_sp = z_mids[k - 1] + nr.thickness / 2
                locs_nr_inter.extend([Location((x, y, z_sp)) for x, y in pins])
            L["inter_needle_spacer"] = (
                self._try(lambda: make_spacer(nr, h_nr_inter, tol.delta("needle_spacer.bore")), "inter_needle_spacer"),
                locs_nr_inter
            )

        L["needle_spacer"] = (
            self._try(lambda: make_spacer(nr, z["output"] - z_nr_last, tol.delta("needle_spacer.bore")), "needle_spacer"),
            [Location((x, y, z_nr_last)) for x, y in pins]
        )
        needle_bolt = socket_head_bolt(nr.ID, z["output"] + o.plate_thickness - z_nr0)
        # flipped: head below the roller, shank pointing up
        L[f"needle_bolt_{bolt_name(needle_bolt)}"] = (
            self._try(lambda: make_bolt(needle_bolt), f"needle_bolt_{bolt_name(needle_bolt)}"),
            [Location((x, y, z_nr0), (180, 0, 0)) for x, y in pins]
        )

        # cap screws: head on the cap, engaging the part below by the bearing thickness
        rh = c.cap.bearing_retainer_holes
        bt = socket_head_bolt(2 * rh.r, c.cap.flange.thickness + c.input_bearing.thickness)
        locs_cap_bolts = []
        for k in range(N):
            phi_k = k * 360.0 / N
            psi_k = phi_k
            rad = math.radians(phi_k)
            cx, cy = e * math.cos(rad), e * math.sin(rad)
            z_head = z_caps[k] + c.cap.flange.thickness
            for _, _, a in _polar(rh.R, rh.N, rh.start_angle):
                a_rot = math.radians(a + psi_k)
                locs_cap_bolts.append(Location((cx + rh.R * math.cos(a_rot), cy + rh.R * math.sin(a_rot), z_head)))
        L[f"cycloid_cap_bolt_{bolt_name(bt)}"] = (
            self._try(lambda: make_bolt(bt), f"cycloid_cap_bolt_{bolt_name(bt)}"),
            locs_cap_bolts
        )

        for name, cp, b in (("output_cap", o.cap, o.output_bearing),
                            ("top_cap", tp.cap, tp.output_bearing)):
            rh = cp.bearing_retainer_holes
            bt = socket_head_bolt(2 * rh.r, cp.flange.thickness + b.thickness)
            z_head = z[name] + cp.flange.thickness
            L[f"{name}_bolt_{bolt_name(bt)}"] = (self._try(lambda: make_bolt(bt), f"{name}_bolt_{bolt_name(bt)}"),
                [Location((x, y, z_head)) for x, y, _ in _polar(rh.R, rh.N, rh.start_angle)])

        L = {name: v for name, v in L.items() if v[0] is not None}   # lenient: skip unbuildable parts
        for name, (part, _) in L.items():
            part.label = name
        return L

    def _try(self, build, name: str):
        """Build one part. Lenient gearboxes report a part whose CAD fails and carry on."""
        try:
            return build()
        except Exception as ex:
            if self.strict:
                raise
            geometry_issue(f"build.{name}", f"{name} could not be built: {ex}")
            return None

    def build(self) -> dict[str, Part]:
        """Every unique part (nominal) at its own origin; returns {name: Part}."""
        return {name: part for name, (part, _) in self._layout_cache.items()}

    def locations(self) -> dict[str, list[Location]]:
        """Assembled location of every instance of each part."""
        return {name: locs for name, (_, locs) in self._layout_cache.items()}

    # stock (bought) parts by name prefix, with their type; everything else is custom: made to
    # this model's dimensions, whether printed or machined
    _STOCK = {
        "input_bearing": "bearing", "output_bearing": "bearing", "roller_bearing": "bearing",
        "needle_roller": "bearing",
        "needle_bolt": "screw", "ring_bolt": "screw", "cycloid_cap_bolt": "screw",
        "output_cap_bolt": "screw", "top_cap_bolt": "screw",
    }

    def stock_types(self) -> dict[str, str | None]:
        """{part name: "bearing" | "screw" for stock parts, None for custom ones}."""
        out = {}
        for name in self._layout_cache:
            keys = [k for k in self._STOCK if name == k or name.startswith(k + "_")]
            out[name] = self._STOCK[max(keys, key=len)] if keys else None
        return out

    def part_kinds(self) -> dict[str, str]:
        """{part name: "custom" | "stock"}."""
        return {name: "custom" if t is None else "stock" for name, t in self.stock_types().items()}

    # which rigid body each part moves with in operation (see web GUI)
    _MOTION_GROUPS = {
        "base": "housing", "top": "housing", "top_cap": "housing", "output_bearing": "housing",
        "roller_bearing": "housing", "roller_spacer_bottom": "housing",
        "roller_spacer_top": "housing", "inter_roller_spacer": "housing", "ring_bolt": "housing", "top_cap_bolt": "housing",
        "eccentric_insert": "input", "eccentric_insert_stacked": "input", "inter_eccentric_spacer": "input",
        "cycloid": "cycloid", "cycloid_cap": "cycloid", "input_bearing": "cycloid",
        "cycloid_cap_bolt": "cycloid",
        "output": "output", "output_cap": "output", "needle_roller": "output",
        "needle_spacer": "output", "inter_needle_spacer": "output", "needle_bolt": "output", "output_cap_bolt": "output",
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

    def collisions(self, min_volume: float = 1e-3, parallel: bool = True) -> list[tuple[str, str, float]]:
        """Exact solid-solid overlaps in the nominal assembly: [(label_a, label_b, volume)].
        Touching faces (zero-volume contact) don't count. Pairs whose bounding boxes overlap
        are intersected exactly, spread over a pool of worker processes when parallel."""
        placed = list(self.assembly().children)
        boxes = [p.bounding_box() for p in placed]
        pairs = []
        for i in range(len(placed)):
            a = boxes[i]
            for j in range(i + 1, len(placed)):
                b = boxes[j]
                if not (a.min.X >= b.max.X or b.min.X >= a.max.X or a.min.Y >= b.max.Y or
                        b.min.Y >= a.max.Y or a.min.Z >= b.max.Z or b.min.Z >= a.max.Z):
                    pairs.append((i, j))

        results = None
        if parallel and len(pairs) > 40:
            try:
                results = _parallel_overlaps(placed, pairs)
            except Exception:           # broken pool / no worker support: fall back to serial
                global _POOL
                _POOL = None
        if results is None:
            results = [(i, j, _overlap(placed[i], placed[j])) for i, j in pairs]
        return [(placed[i].label, placed[j].label, v) for i, j, v in results if v > min_volume]

    def assembly(self, layout: dict | None = None) -> Compound:
        children = []
        for name, (part, locs) in (layout or self._layout_cache).items():
            for i, loc in enumerate(locs):
                placed = part.moved(loc)
                placed.label = name if len(locs) == 1 else f"{name}_{i + 1}"
                children.append(placed)
        return Compound(children=children, label="gearbox")

    def export_assembly(self, path: str = "steps/gearbox/gearbox.step") -> Compound:
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
        if tol.enabled:   # the motor-hole fit follows the motor's mounting style
            fit = "loose" if self.base.drive_motor.face_mount else "tight"
            tol = dataclasses.replace(tol, fits={**tol.fits, "base.motor_holes": fit})
        if not tol.enabled:
            layout = self._layout_cache
        elif self.strict:
            layout = self._layout(tol)
        else:
            with collecting_issues():
                layout = self._layout(tol)
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

    def generate(self, out_dir: str = "steps/gearbox") -> dict[str, Part]:
        """Write every nominal part, at its own origin, to out_dir as separate STEP files."""
        self.export(out_dir)
        return self.parts



# ---------- exact overlap checks, optionally across worker processes ----------
# OCCT booleans don't run concurrently under the GIL, so collisions() fans the pairs out to a
# persistent process pool. Shapes reach the workers as BREP files (they don't pickle).
_POOL = None


def _overlap(a, b) -> float:
    common = a & b
    if common is None or common.volume <= 0:
        return 0.0
    # In rare cases with periodic B-spline tangent contact, OpenCASCADE BOP produces
    # a false common volume whose center of mass is outside the intersecting solids.
    try:
        c = common.center()
        if not (a.is_inside(c) or b.is_inside(c)):
            return 0.0
    except Exception:
        pass
    return common.volume


def _overlap_chunk(args) -> list[tuple[int, int, float]]:
    from build123d import import_brep
    paths, pairs = args
    cache = {}

    def load(k):
        if k not in cache:
            cache[k] = import_brep(paths[k])
        return cache[k]

    return [(i, j, _overlap(load(i), load(j))) for i, j in pairs]


def _pool():
    global _POOL
    if _POOL is None:
        from concurrent.futures import ProcessPoolExecutor
        _POOL = ProcessPoolExecutor(max_workers=max(2, min(12, (os.cpu_count() or 4) - 2)))
    return _POOL


def _parallel_overlaps(placed, pairs) -> list[tuple[int, int, float]]:
    import tempfile
    from build123d import export_brep
    pool = _pool()
    with tempfile.TemporaryDirectory() as d:
        used = sorted({k for pair in pairs for k in pair})
        paths = {k: os.path.join(d, f"{k}.brep") for k in used}
        for k in used:
            export_brep(placed[k], paths[k])
        # expensive pairs (the cycloid's) first, then round-robin so every worker gets a mix
        n = pool._max_workers * 3
        chunks = [pairs[c::n] for c in range(n)]
        out = []
        for part in pool.map(_overlap_chunk, [({k: paths[k] for p in ch for k in p}, ch) for ch in chunks if ch]):
            out.extend(part)
    return out

# example gearbox: NEMA17 drive, 23:1
EXAMPLE_KWARGS = dict(
    ratio=23,
    roller_bearing=bearing(ID=2, OD=5, thickness=2.5),         # MR52-ish
    roller_R=40.0,
    housing_edge_offset=7.0,                                   # 48 mm radius
    base_cycloid_spacing=1.0,
    cap_output_spacing=3.0,                                    # clears the 2 mm cap screw heads
    output_top_spacing=1.0,
    inter_disk_spacing=3.0,
    num_disks=1,

    e=1.2,
    input_bearing=bearing(ID=10, OD=19, thickness=5),          # 6800-ish
    output_needle_roller=bearing(ID=3, OD=6, thickness=2.5),   # MR63-ish
    output_holes_R=24.0,
    output_holes_N=6,
    cycloid_cap=cap(
        flange=flange(thickness=2.0, offset=1.5),
        bearing_retainer_holes=patterned_hole(r=1.0, R=12.0, N=4),   # M2 pilot
        edge_offset=2.0,                                      # 15 mm radius
    ),

    drive_motor=motor(
        shaft_r=2.5,                                           # NEMA17
        mounting_holes=patterned_hole(r=1.6, R=21.92, N=4),    # M3 on 31 mm square
    ),
    shaft_hole_offset=9.0,
    base_thickness=5.0,
    base_t=5.0,
    mounting_holes=patterned_hole(r=2.2, R=7.0, N=2),          # M4, centers 7 mm beyond the edge (R 55)
    mounting_hole_offset=3.0,
    mounting_hole_fillet_r=3.0,

    output_bearing=bearing(ID=25, OD=37, thickness=7),         # 6805-ish
    output_bearing_offset=6.0,
    top_base_thickness=4.0,
    top_cap=cap(
        flange=flange(thickness=2.0, offset=2.0),
        bearing_retainer_holes=patterned_hole(r=1.25, R=21.5, N=6),  # M2.5 pilot
        edge_offset=1.75,                                     # 24.5 mm: flush with tower wall
    ),

    eccentric_insert_flange_offset=1.5,

    output_flange_offset=2.0,
    output_plate_edge_offset=8.5,                              # 34 mm radius
    output_plate_thickness=4.0,
    output_plate_t=4.0,
    output_bore_r=6.0,
    output_cap=cap(
        flange=flange(thickness=2.0, offset=1.5),
        bearing_retainer_holes=patterned_hole(r=1.0, R=9.5, N=4),    # M2 pilot
        edge_offset=2.5,                                      # 6 mm bore: matches the output bore
    ),
)


if __name__ == "__main__":
    gb = gearbox(**EXAMPLE_KWARGS)
    for name, part in gb.generate("steps/gearbox").items():
        print(f"{name}: volume={part.volume:.1f} valid={part.is_valid}")
