"""Simple mode: derive every gearbox dimension from the bearings, motor, ratio, e and roller R.

derive() turns a simple_spec into gearbox() kwargs using the design rules below. solve()
builds that, and when a geometry check or the exact collision check fails it nudges the
offending derived value (smaller cap screw, wider gap, ...) and tries again.
"""
import math
import time
from collections import defaultdict
from dataclasses import dataclass, field

from part_generation_fcns.classdefs import *
from part_generation_fcns.errors import GeometryError
from part_generation_fcns.gearbox import gearbox
from part_generation_fcns.hardware import _SHCS, socket_head_bolt

SCREW_SIZES = sorted(_SHCS)            # metric SHCS sizes we know head dimensions for


@dataclass
class simple_spec:
    ratio: int
    e: float
    roller_R: float
    roller_bearing: bearing
    input_bearing: bearing
    output_needle_roller: bearing
    output_bearing: bearing
    drive_motor: motor
    num_disks: int = 1
    inter_disk_spacing: float = 3.0


@dataclass
class design_rules:
    min_wall: float = 1.5          # thinnest wall anywhere...
    insert_wall: float = 1.0       # ...except the eccentric insert's, at one point beside the shaft
    clearance: float = 1.0         # running clearance between moving parts and plates
    lip_frac: float = 0.3          # lip / cap overlap onto a race, as a fraction of race width
    screw_frac: float = 0.10       # largest cap screw diameter as a fraction of bearing OD...
    screw_thickness_frac: float = 0.5  # ...and of bearing width (keeps screws sane on big thin bearings)
    thread_engage: float = 1.5     # threaded depth >= this * screw diameter
    min_plate: float = 3.0         # thinnest plate

    @property
    def screw_th_frac(self) -> float:
        return self.screw_thickness_frac

    @screw_th_frac.setter
    def screw_th_frac(self, val: float):
        self.screw_thickness_frac = val


RULES = design_rules()


class SolveError(ValueError):
    """Simple-mode inputs that no adjustment of the derived values can satisfy."""

    def __init__(self, msg: str, log: list[str] | None = None):
        super().__init__(msg)
        self.log = log or []


@dataclass
class SolveResult:
    kwargs: dict
    gearbox: gearbox
    log: list[str] = field(default_factory=list)
    iterations: int = 0
    collisions: list = field(default_factory=list)
    check_seconds: float = 0.0
    ok: bool = True                      # False: best-effort design with unresolved problems
    problems: list[str] = field(default_factory=list)


def _screw(b: bearing, rules: design_rules, step: int = 0) -> float:
    """Cap screw diameter for bearing b: the largest size <= screw_frac * OD and
    <= screw_thickness_frac * width (at least the smallest size), then `step` sizes smaller."""
    limit = max(SCREW_SIZES[0], min(rules.screw_frac * b.OD, rules.screw_thickness_frac * b.thickness))
    idx = max(i for i, d in enumerate(SCREW_SIZES) if d <= limit) - step
    if idx < 0:
        raise SolveError(f"no screw small enough for the {b.ID:g}x{b.OD:g} bearing's cap")
    return SCREW_SIZES[idx]


def _up(x: float, step: float = 0.1) -> str:
    """x rounded *up* to step, for "at least" suggestions that must actually work."""
    return f"{math.ceil(x / step - 1e-9) * step:.1f}"


def _head(d: float) -> tuple[float, float]:
    """(head diameter, head height) exactly as the hardware generator builds the screw."""
    b = socket_head_bolt(d, 1.0)
    return b.head_d, b.head_h


def _lip(b: bearing, rules: design_rules) -> float:
    return rules.lip_frac * (b.OD - b.ID) / 2


def _flange(b: bearing, rules: design_rules) -> flange:
    t = max(rules.min_wall, math.ceil(0.3 * b.thickness * 2) / 2)       # round up to 0.5 mm
    return flange(thickness=t, offset=round(_lip(b, rules), 3))


def _pattern_n(R: float, pitch: float, lo: int, hi: int) -> int:
    return max(lo, min(hi, int(2 * math.pi * R / pitch)))


def derive(spec: simple_spec, rules: design_rules = RULES, adj: dict | None = None,
           problems: list[str] | None = None) -> dict:
    """gearbox() kwargs from a simple spec. adj holds the solver's accumulated corrections.
    With a problems list, unsatisfiable rules are recorded there and a best-effort value is
    used instead of raising, so the design can still be built and inspected."""
    adj = adj if adj is not None else defaultdict(float)

    def problem(msg: str):
        if problems is None:
            raise SolveError(msg)
        if msg not in problems:
            problems.append(msg)

    def screw(b: bearing, key: str) -> float:
        try:
            return _screw(b, rules, int(adj[key]))
        except SolveError as ex:
            problem(str(ex))
            return SCREW_SIZES[0]

    r3 = lambda x: round(x, 3)
    mw, cl, te = rules.min_wall, rules.clearance, rules.thread_engage
    rb, ib, nr, ob, m = (spec.roller_bearing, spec.input_bearing, spec.output_needle_roller,
                         spec.output_bearing, spec.drive_motor)
    e = spec.e

    # the eccentric insert carries the motor shaft offset by e inside the input bearing's ID
    need_id = 2 * (m.shaft_r + e + rules.insert_wall)
    if ib.ID < need_id:
        problem(f"the input bearing's ID ({ib.ID:g} mm) is too small for the motor shaft "
                         f"(Ø{2 * m.shaft_r:g}) offset by e={e:g} with a {rules.insert_wall:g} mm insert wall: "
                         f"needs ID >= {_up(need_id)} mm")

    # cycloid cap screws: just outside the input bearing seat (the cap's outline and the screw
    # count follow from the output holes below)
    d_c = screw(ib, "cycloid_cap_screw")
    R_c = ib.OD / 2 + mw + d_c / 2

    # top cap: screws in the tower wall just outside the output bearing seat
    d_t = screw(ob, "top_cap_screw")
    R_t = ob.OD / 2 + mw + d_t / 2
    top_cap = cap(flange=_flange(ob, rules),
                  bearing_retainer_holes=patterned_hole(r=d_t / 2, R=r3(R_t),
                                                        N=_pattern_n(R_t, 4 * _head(d_t)[0], 3, 8)),
                  edge_offset=mw)

    # output cap: clamps the output bearing inner race, screws in the journal
    d_o = screw(ob, "output_cap_screw")
    R_o = ob.ID / 2 - mw - d_o / 2
    bore = R_o - d_o / 2 - mw
    if bore < 0:
        if problems is None:
            raise GeometryError("output_cap.no_bore", "output cap screws don't fit in the output journal")
        problem("output cap screws don't fit in the output journal")
    out_cap = cap(flange=_flange(ob, rules),
                  bearing_retainer_holes=patterned_hole(r=d_o / 2, R=r3(R_o),
                                                        N=_pattern_n(R_o, 4 * _head(d_o)[0], 3, 8)),
                  edge_offset=mw)

    # output holes, as far out as the walls allow (largest moment arm: lowest pin forces).
    # The cycloid cap is a small ring whose edge is its screw circle + 1/4 screw radius (the
    # screws notch it); the holes must clear the cap and the screw heads.
    r_oh = nr.OD / 2 + e                      # cycloid hole (needle roller orbits by e)
    pin_r = nr.ID / 2                         # needle screw / output plate hole
    root = spec.roller_R - rb.OD / 2 - e      # smallest cycloid radius
    plate_r = spec.roller_R - rb.OD / 2 - cl  # output plate edge, clear of the rollers
    shoulder = ob.ID / 2 + _lip(ob, rules)
    hi = min(root - mw - r_oh, plate_r - mw - pin_r)
    lo = max(ib.OD / 2 + mw + r_oh,           # wall between the bearing seat and the holes
             shoulder + mw + pin_r,           # wall between the output shoulder and the pins
             R_c + _head(d_c)[0] / 2 + cl / 2 + r_oh)   # cap screw heads clear of the holes
    if lo > hi + 1e-6:                        # (tolerance: float noise is not a real shortfall)
        cap_limits = lo == R_c + _head(d_c)[0] / 2 + cl / 2 + r_oh
        if cap_limits and d_c > SCREW_SIZES[0] and problems is None:
            raise GeometryError("derive.output_holes_dont_fit", "cycloid cap screw heads crowd the output holes")
        problem(f"the output holes (Ø{2 * r_oh:g} for a Ø{nr.OD:g} needle roller orbiting by e) "
                f"need {_up(lo - hi)} mm more radial room: increase roller R to at least "
                f"{_up(spec.roller_R + lo - hi)} mm, or use a smaller e, needle roller, "
                f"input or output bearing")
    R_oh = hi if lo <= hi + 1e-6 else (lo + hi) / 2
    N_oh = _pattern_n(R_oh, 4 * r_oh, 3, 8)   # web between holes >= one hole diameter
    cyc_cap = cap(flange=_flange(ib, rules),
                  bearing_retainer_holes=patterned_hole(r=d_c / 2, R=r3(R_c),
                                                        N=_pattern_n(R_c, 4 * _head(d_c)[0], 3, 8)),
                  edge_offset=r3(-0.75 * d_c / 2))   # (the generator sets this: screws notch the edge)

    # stack gaps, driven by the screw heads that sit in them
    needle_d = nr.ID
    cycloid_thickness_val = _flange(ib, rules).thickness + ib.thickness
    needle_head_drop = _head(needle_d)[1] - (cycloid_thickness_val - nr.thickness) / 2
    base_cycloid = cl + max(0.0, needle_head_drop) + adj["base_cycloid_spacing"]
    cap_output = cl + _head(d_c)[1] + adj["cap_output_spacing"]
    inter_disk = cl + _head(d_c)[1] + adj.get("inter_disk_spacing", 0.0)
    output_top = cl + adj["output_top_spacing"]

    motor_d = 2 * m.mounting_holes.r
    base_thickness = max(4.0, te * rb.ID, 0.0 if m.face_mount else te * motor_d)
    ring = motor_d + motor_d / 2               # mounting tab ring radius (hole r + wall of one d)
    tab_size = min(SCREW_SIZES, key=lambda s: abs(s - motor_d))

    output_plate_thickness = max(rules.min_plate, te * nr.ID)
    insert_flange_offset = _lip(ib, rules)

    return dict(
        ratio=spec.ratio,
        roller_bearing=rb,
        roller_R=spec.roller_R,
        housing_edge_offset=r3(max(2 * mw, 1.5 * rb.ID)),
        base_cycloid_spacing=r3(base_cycloid),
        cap_output_spacing=r3(cap_output),
        output_top_spacing=r3(output_top),
        inter_disk_spacing=r3(inter_disk),
        num_disks=getattr(spec, "num_disks", 1),
        e=e,
        input_bearing=ib,
        output_needle_roller=nr,
        output_holes_R=r3(R_oh),
        output_holes_N=N_oh,
        cycloid_cap=cyc_cap,
        drive_motor=m,
        shaft_hole_offset=r3(ib.ID / 2 + _lip(ib, rules) + e + cl - m.shaft_r + adj["shaft_hole_offset"]),
        base_thickness=r3(base_thickness),
        base_t=r3(base_thickness),
        mounting_holes=patterned_hole(r=tab_size / 2, R=r3(tab_size / 2 + tab_size), N=2),
        mounting_hole_offset=r3(tab_size),
        mounting_hole_fillet_r=r3(0.5 * (tab_size / 2 + tab_size)),
        output_bearing=ob,
        output_bearing_offset=r3(2 * mw + d_t),
        top_base_thickness=r3(max(rules.min_plate, te * rb.ID) + adj["top_base_thickness"]),
        top_cap=top_cap,
        eccentric_insert_flange_offset=r3(insert_flange_offset),
        output_flange_offset=r3(_lip(ob, rules)),
        output_plate_edge_offset=r3(plate_r - R_oh - pin_r),
        output_plate_thickness=r3(output_plate_thickness),
        output_plate_t=r3(output_plate_thickness),
        output_bore_r=r3(max(0.0, bore)),
        output_cap=out_cap,
    )


def _step_screw(key: str, what: str):
    def fix(adj):
        adj[key] += 1
        return f"stepped the {what} screw down one size"
    return fix


def _fail(msg: str):
    def fix(adj):
        raise SolveError(msg)
    return fix


# geometry-check code -> correction to the derived values (or a plain-language failure)
FIXES = {
    "derive.output_holes_dont_fit": _step_screw("cycloid_cap_screw", "cycloid cap"),
    "cycloid.cap_covers_output_holes": _step_screw("cycloid_cap_screw", "cycloid cap"),
    "cycloid_cap.holes_overlap_seat": _step_screw("cycloid_cap_screw", "cycloid cap"),
    "cycloid_cap.edge_offset": _step_screw("cycloid_cap_screw", "cycloid cap"),
    "top_cap.holes_overlap_seat": _step_screw("top_cap_screw", "top cap"),
    "top_cap.edge_offset": _step_screw("top_cap_screw", "top cap"),
    "top.retainer_holes_outside_wall": _step_screw("top_cap_screw", "top cap"),
    "top.roller_holes_hit_wall": _step_screw("top_cap_screw", "top cap"),
    "output_cap.no_bore": _step_screw("output_cap_screw", "output cap"),
    "output_cap.holes_break_journal": _step_screw("output_cap_screw", "output cap"),
    "output_cap.edge_offset": _step_screw("output_cap_screw", "output cap"),
    "top.base_too_thick": lambda adj: (adj.__setitem__("top_base_thickness", adj["top_base_thickness"] - 0.5),
                                       "thinned the top's base plate")[1],
    "assembly.insert_hits_base_hole": lambda adj: (adj.__setitem__("shaft_hole_offset", adj["shaft_hole_offset"] + 0.5),
                                                   "opened up the base's center hole")[1],
    "cycloid.cusps": _fail("e is too large for this roller R and ratio: needs roller_R > e * (ratio + 1)"),
    "cycloid.undercut": _fail("the roller bearing is too large for this e, ratio and roller R (the cycloid "
                              "profile undercuts): reduce e or the roller OD, or increase roller R"),
    "insert.shaft_hole_breaks_journal": _fail("the motor shaft plus e doesn't fit inside the input bearing's ID"),
    "base.center_hole_hits_motor_holes": _fail("the motor's screw pattern sits inside the center hole the eccentric "
                                               "insert needs; use a smaller input bearing or a larger motor"),
    "base.motor_holes_hit_roller_holes": _fail("the motor's screw pattern reaches the ring rollers; increase roller R"),
    "output.holes_overlap_shoulder": _fail("the output bearing is too large for the output holes; increase roller R "
                                           "or use a smaller output bearing"),
    "assembly.plate_hits_rollers": _fail("the output plate can't clear the ring rollers; increase roller R"),
}

# colliding part pairs (by name prefix) -> correction
COLLISION_FIXES = [
    (("cycloid_cap_bolt", "output"), "cap_output_spacing", "widened the cycloid cap -> output gap"),
    (("needle_bolt", "base"), "base_cycloid_spacing", "widened the base -> cycloid gap"),
    (("ring_bolt", "output"), "output_top_spacing", "widened the output -> top gap"),
]


def _part(label: str) -> str:
    """Instance label -> part name ('roller_bearing_12' -> 'roller_bearing')."""
    head, _, tail = label.rpartition("_")
    return head if tail.isdigit() else label


def _collision_fix(a: str, b: str):
    pa, pb = _part(a), _part(b)
    for (x, y), key, msg in COLLISION_FIXES:
        if (pa.startswith(x) and pb == y) or (pb.startswith(x) and pa == y):
            return key, msg
    return None


def solve(spec: simple_spec, rules: design_rules = RULES, max_iter: int = 12,
          check_collisions: bool = True, progress=None, best_effort: bool = False) -> SolveResult:
    """Derive, build and check until it builds with no collisions. progress(stage) is called
    with "solving" / "checking collisions" for UI feedback.

    If no adjustment works this raises SolveError, unless best_effort: then the best-effort
    design is built anyway (geometry checks recorded, not enforced) and returned with
    ok=False, its problems and its collisions, so it can be inspected."""
    adj = defaultdict(float)
    try:
        return _solve(spec, rules, max_iter, check_collisions, progress, adj)
    except SolveError as ex:
        if not best_effort:
            raise
        problems = [str(ex)]
        kwargs = derive(spec, rules, adj, problems)
        gb = gearbox(**kwargs, assembly_path=None, strict=False)
        problems += [m for _, m in gb.issues if m not in problems]
        hits, dt = [], 0.0
        if check_collisions:
            if progress:
                progress("checking collisions")
            t0 = time.time()
            hits = gb.collisions()
            dt = time.time() - t0
        return SolveResult(kwargs, gb, ex.log, 0, hits, dt, ok=False, problems=problems)


def _solve(spec, rules, max_iter, check_collisions, progress, adj) -> SolveResult:
    log: list[str] = []
    last: GeometryError | None = None     # the check we were fixing, for a useful failure message
    for it in range(1, max_iter + 1):
        if progress:
            progress("solving")
        try:
            kwargs = derive(spec, rules, adj)
            gb = gearbox(**kwargs, assembly_path=None)
        except SolveError as ex:
            if last is not None:      # ran out of room while fixing an earlier check
                raise SolveError(f"{last} (no smaller screw left to try: increase roller R or use "
                                 f"smaller bearings)", log) from None
            raise SolveError(str(ex), log) from None
        except GeometryError as g:
            last = g
            fix = FIXES.get(g.code)
            if fix is None:
                raise SolveError(f"can't satisfy: {g}", log) from None
            try:
                log.append(f"{g} -> {fix(adj)}")
            except SolveError as ex:
                raise SolveError(str(ex), log) from None
            continue

        if not check_collisions:
            return SolveResult(kwargs, gb, log, it)
        if progress:
            progress("checking collisions")
        t0 = time.time()
        hits = gb.collisions()
        dt = time.time() - t0
        if not hits:
            return SolveResult(kwargs, gb, log, it, [], dt)
        fixes = {}
        for a, b, v in hits:
            f = _collision_fix(a, b)
            if f is None:
                pairs = ", ".join(f"{a} x {b} ({v:.2f} mm^3)" for a, b, v in hits[:6])
                raise SolveError(f"parts collide and no rule fixes it: {pairs}", log)
            fixes[f[0]] = f[1]
        for key, msg in fixes.items():
            adj[key] += 0.5
            log.append(f"collision -> {msg} by 0.5 mm")
    raise SolveError(f"didn't converge in {max_iter} iterations", log)


def spec_from_kwargs(kw: dict) -> simple_spec:
    """The simple-mode inputs contained in a full set of gearbox() kwargs."""
    fields = simple_spec.__dataclass_fields__
    return simple_spec(**{f: kw[f] for f in fields if f in kw})


if __name__ == "__main__":
    from part_generation_fcns.gearbox import EXAMPLE_KWARGS
    res = solve(spec_from_kwargs(EXAMPLE_KWARGS), progress=print)
    print(f"solved in {res.iterations} iteration(s), collision check {res.check_seconds:.1f} s")
    for line in res.log:
        print("  ", line)
    for k, v in res.kwargs.items():
        old = EXAMPLE_KWARGS[k]
        print(f"{k:32s} {v!s:70.70s}" + ("" if v == old else f"   (example: {old})"))
