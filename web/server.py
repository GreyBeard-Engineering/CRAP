"""Live gearbox viewer.  Run:  python -m uvicorn web.server:app --port 8000"""
import dataclasses
import inspect
import io
import json
import tempfile
import threading
import zipfile
from pathlib import Path

from fastapi import Body, FastAPI
from fastapi.concurrency import run_in_threadpool
from fastapi.responses import FileResponse, JSONResponse, Response
from fastapi.staticfiles import StaticFiles

from part_generation_fcns.gearbox import EXAMPLE_KWARGS, gearbox
from part_generation_fcns.solver import SolveError, simple_spec, solve, spec_from_kwargs
from part_generation_fcns.tolerancing import DEFAULT_MAGNITUDES, FEATURES, tolerancing
from web.mesh import gearbox_payload

STATIC = Path(__file__).parent / "static"
app = FastAPI(title="CRAP: Cycloidal Reducer Algorithm & Parametrizer")
app.mount("/static", StaticFiles(directory=STATIC), name="static")
_build_lock = threading.Lock()
_progress = {"stage": "idle"}          # what a simple-mode build is doing, for /api/progress
_solved: dict[str, object] = {}       # simple spec (json) -> SolveResult, so export needn't re-solve

# constructor argument -> type (dataclass, int or float); assembly_path is not user-facing
_ARG_TYPES = {name: p.annotation for name, p in inspect.signature(gearbox.__init__).parameters.items()
              if name not in ("self", "assembly_path")}


# Manual Configuration layout: always-visible boxes on top, then tabs (a tab holds either a
# list of constructor arguments or further sub-tabs)
LAYOUT = {
    "pinned": {
        "Key Parameters": ["e", "roller_R", "ratio"],
        "Plate Spacing": ["base_cycloid_spacing", "cap_output_spacing", "output_top_spacing"],
    },
    "tabs": {
        "Bearings": ["roller_bearing", "input_bearing", "output_needle_roller", "output_bearing"],
        "Motor": ["drive_motor"],
        "Parts": {
            "Housing": ["housing_edge_offset"],
            "Cycloid": ["output_holes_R", "output_holes_N", "cycloid_cap"],
            "Base": ["shaft_hole_offset", "base_t", "mounting_holes", "mounting_hole_offset",
                     "mounting_hole_fillet_r"],
            "Top": ["output_bearing_offset", "top_base_thickness", "top_cap"],
            "Eccentric insert": ["eccentric_insert_flange_offset"],
            "Output": ["output_flange_offset", "output_plate_edge_offset", "output_plate_t", "output_bore_r", "output_cap"],
        },
    },
}
LABELS = {"ratio": "gearbox ratio", "e": "eccentricity e",
          "base_cycloid_spacing": "base → cycloid", "cap_output_spacing": "cycloid cap → output",
          "output_top_spacing": "output → top",
          "roller_R": "roller R (from centerline)",
          "housing_edge_offset": "edge offset (beyond roller holes)",
          "output_plate_edge_offset": "plate edge offset (beyond needle screw holes)",
          "mounting_holes.R": "R (beyond base edge)",
          "cycloid_cap.edge_offset": "edge offset (beyond screw holes)",
          "top_cap.edge_offset": "edge offset (beyond screw holes)",
          "output_cap.edge_offset": "bore offset (inside screw holes)",
          "drive_motor.face_mount": "face mount (screws into the motor face)"}
# what Simple mode asks for; everything else is derived by the solver
SIMPLE_ARGS = list(simple_spec.__dataclass_fields__)


def _layout_args(node) -> list[str]:
    if isinstance(node, list):
        return node
    return [a for child in node.values() for a in _layout_args(child)]


_laid_out = _layout_args(LAYOUT["pinned"]) + _layout_args(LAYOUT["tabs"])
assert sorted(_laid_out) == sorted(_ARG_TYPES), "LAYOUT out of sync with gearbox()"


def _types(tp):
    """Mirror of a value's structure with "int" / "float" leaves."""
    if dataclasses.is_dataclass(tp):
        return {f.name: _types(f.type) for f in dataclasses.fields(tp)}
    return tp.__name__


def _to_json(v):
    return dataclasses.asdict(v) if dataclasses.is_dataclass(v) else v


def _from_json(tp, v):
    if dataclasses.is_dataclass(tp):
        return tp(**{f.name: _from_json(f.type, v[f.name]) for f in dataclasses.fields(tp)})
    return tp(v)


def _build(body: dict, kwargs: dict) -> dict:
    with _build_lock:
        try:
            kwargs, res = _resolve(body, kwargs)
        finally:
            _progress.update(stage="idle")
        if res is None:
            return gearbox_payload(gearbox(**kwargs, assembly_path=None))
        return {**gearbox_payload(res.gearbox),
                "derived": {k: _to_json(v) for k, v in kwargs.items()},
                "solver": _solver_report(res)}


@app.get("/api/progress")
def progress():
    return _progress


@app.get("/")
def index():
    return FileResponse(STATIC / "index.html")


@app.get("/api/defaults")
def defaults():
    return {
        "values": {k: _to_json(v) for k, v in EXAMPLE_KWARGS.items()},
        "types": {k: _types(tp) for k, tp in _ARG_TYPES.items()},
        "layout": LAYOUT,
        "labels": LABELS,
        "simple_args": SIMPLE_ARGS,
        "tolerancing": {
            "magnitudes": DEFAULT_MAGNITUDES,
            "features": [{"id": k, "part": v[0], "label": v[1], "kind": v[2], "shape": v[3], "default": v[4]}
                         for k, v in FEATURES.items()],
        },
    }


def _kwargs(body: dict) -> dict:
    return {k: _from_json(_ARG_TYPES[k], body["kwargs"][k]) for k in _ARG_TYPES}


def _solve(kwargs: dict):
    """Simple mode: solve from the simple inputs in kwargs (cached by those inputs)."""
    spec = spec_from_kwargs(kwargs)
    key = json.dumps(dataclasses.asdict(spec), sort_keys=True)
    if key not in _solved:
        res = solve(spec, progress=lambda stage: _progress.update(stage=stage))
        if len(_solved) > 32:
            _solved.clear()
        _solved[key] = res
    return _solved[key]


def _resolve(body: dict, kwargs: dict):
    """(kwargs to build, SolveResult or None) for the request's mode."""
    if body.get("mode") == "simple":
        res = _solve(kwargs)
        return res.kwargs, res
    return kwargs, None


def _solver_report(res) -> dict:
    return {"iterations": res.iterations, "log": res.log, "check_seconds": round(res.check_seconds, 1),
            "collisions": [[a, b, round(v, 3)] for a, b, v in res.collisions]}


def _export_zip(body: dict, kwargs: dict, tol: tolerancing, fmt: str, scope: str) -> bytes:
    with _build_lock:
        kwargs, _ = _resolve(body, kwargs)
        gb = gearbox(**kwargs, assembly_path=None)
        with tempfile.TemporaryDirectory() as d:
            paths = gb.export(d, tol, fmt, scope)
            buf = io.BytesIO()
            with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as z:
                for p in paths:
                    z.write(p, Path(p).name)
                z.writestr("export_settings.json", json.dumps({
                    "format": fmt, "scope": scope, "mode": body.get("mode", "advanced"),
                    "tolerancing": dataclasses.asdict(tol),
                    "dimensions": {k: _to_json(v) for k, v in kwargs.items()},
                }, indent=2))
            return buf.getvalue()


@app.post("/api/export")
async def export(body: dict = Body(...)):
    try:
        kwargs = _kwargs(body)
        t = body.get("tolerancing", {})
        tol = tolerancing(enabled=bool(t.get("enabled", False)),
                          **{k: float(t.get("magnitudes", {}).get(k, v)) for k, v in DEFAULT_MAGNITUDES.items()},
                          fits={**{k: v[4] for k, v in FEATURES.items()}, **t.get("fits", {})})
        fmt, scope = body.get("format", "step"), body.get("scope", "all")
    except (KeyError, TypeError, ValueError) as ex:
        return JSONResponse({"error": f"bad input: {ex}"}, status_code=422)
    try:
        data = await run_in_threadpool(_export_zip, body, kwargs, tol, fmt, scope)
    except ValueError as ex:
        return JSONResponse({"error": str(ex)}, status_code=422)
    except Exception as ex:
        return JSONResponse({"error": f"CAD kernel failed: {ex}"}, status_code=422)
    name = f"gearbox_{scope}_{fmt}{'_toleranced' if tol.enabled else ''}.zip"
    return Response(data, media_type="application/zip",
                    headers={"Content-Disposition": f'attachment; filename="{name}"'})


@app.post("/api/build")
async def build(body: dict = Body(...)):
    seq = body.get("seq")
    try:
        kwargs = _kwargs(body)
    except (KeyError, TypeError, ValueError) as ex:
        return JSONResponse({"seq": seq, "error": f"bad input: {ex}"}, status_code=422)
    try:
        payload = await run_in_threadpool(_build, body, kwargs)
    except SolveError as ex:                        # simple mode: inputs the solver can't satisfy
        return JSONResponse({"seq": seq, "error": str(ex), "solver_log": ex.log}, status_code=422)
    except (ValueError, ZeroDivisionError) as ex:   # geometry checks in the generators
        return JSONResponse({"seq": seq, "error": str(ex)}, status_code=422)
    except Exception as ex:                         # OCC failures on odd inputs
        return JSONResponse({"seq": seq, "error": f"CAD kernel failed: {ex}"}, status_code=422)
    return {"seq": seq, **payload}
