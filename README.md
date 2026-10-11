# CRAP: Cycloidal Reducer Algorithm & Parametrizer

A parametric generator for **cycloidal reducers**. You describe the gearbox (bearings, motor, ratio, eccentricity and roller radius) and CRAP builds every part as solid CAD with [build123d](https://github.com/gumyr/build123d). The parts are made to be machined on a 2.5-axis mill or 3D printed. CRAP:

- assembles them, checks them for collisions, animates the mechanism in a browser, and
- exports toleranced STEP or STL files, ready to make.

Everything is in millimetres.

## Quick start

```bash
python -m venv venv
venv\Scripts\pip install -r requirements.txt     # build123d, numpy, fastapi, uvicorn
launch_gui.bat                                    # or: python launch_gui.py
```

The GUI opens at <http://127.0.0.1:8000>.

## The GUI

**Left panel: Manual Configuration**
- **Simple mode:** you enter the bearings, the motor (with a *face mount* option), the ratio, the eccentricity and the roller radius. A solver derives every other dimension, rebuilds until the geometry checks pass, and finishes with an exact 3D collision check (about 15 s). If the inputs can't work, it tells you why in plain language, for example "the roller bearing is too large for this e, ratio and roller R".
- **Advanced mode:** every dimension is editable. Switching from Simple to Advanced pre-fills the solver's choices.
- **Key Parameters and Plate Spacing:** always visible at the top.
- **Tabs:** Bearings, Motor, and (Advanced only) Parts, with a sub-tab per custom part.
- **Rebuild:** turns orange when the form no longer matches the model on screen. Enter also rebuilds.

**Left panel: Export**
- STEP (individual parts plus the assembled gearbox) or STL.
- Every part, or custom parts only (everything except the bought bearings and screws).
- Optional **tolerancing**, applied only at export:
  - bearing seats and journals;
  - tight (threaded) and loose (clearance) screw holes;
  - cycloid running clearance.
- Every fit can be overridden under *Advanced fits*. The motor-hole fit follows *face mount*.
- The download is a zip. It includes `export_settings.json`, which records every dimension and tolerance used.

**Right panel: View**
- X / Y / Z / home camera views.
- Input speed and pause, with input and output turn counters. The cycloid wobbles and counter-rotates, and the output turns at 1/ratio.
- Show or hide screws and bearings.
- A section view through the central axis.
- Colour-coded toggles for each custom part.

## Python API

```python
from part_generation_fcns.gearbox import gearbox, EXAMPLE_KWARGS
from part_generation_fcns.solver import solve, spec_from_kwargs
from part_generation_fcns.tolerancing import tolerancing, DEFAULT_MAGNITUDES

gb = gearbox(**EXAMPLE_KWARGS)            # build every part; writes steps/gearbox/gearbox.step
gb.generate("steps/gearbox")              # one STEP per part
print(gb.collisions())                    # exact solid-solid overlaps, [] when clean

res = solve(spec_from_kwargs(EXAMPLE_KWARGS))   # Simple mode from Python
res.gearbox.export("out", tolerancing(enabled=True, **DEFAULT_MAGNITUDES), fmt="stl", scope="custom")
```

## Repository layout

| path | what |
|---|---|
| `part_generation_fcns/classdefs.py` | dataclasses: bearing, patterned_hole, motor, flange, cap, cycloid, base, top, eccentric_insert, output, bolt |
| `part_generation_fcns/*_step.py` | one generator per part: cycloid, base, top, cap, eccentric insert, output |
| `part_generation_fcns/gearbox.py` | the assembly: shared dimensions, axial stack, placement, motion groups, collision check, export |
| `part_generation_fcns/hardware.py` | bearings, spacers and ISO 4762 socket-head screws |
| `part_generation_fcns/tolerancing.py` | export-time fits |
| `part_generation_fcns/solver.py` | Simple-mode derivation rules and the iterate-until-it-builds solver |
| `part_generation_fcns/errors.py` | coded geometry errors the solver reacts to |
| `web/` | FastAPI server (`server.py`), mesh payload (`mesh.py`) and the three.js front end (`static/`) |
| `steps/` | generated STEP files (`steps/gearbox/` holds the full assembly) |
| `test_cycloid.py`, `test_solver.py` | checks; `python test_solver.py --full` runs the exact collision check on every case in the sweep |

## Design notes

- **Axial stack, bottom to top:**
  - base
  - base → cycloid gap
  - cycloid and its cap
  - cap → output gap
  - output plate
  - output → top gap
  - top, then the caps on top

  In Simple mode, each gap is sized from the screw heads that sit in it.
- **Part radii are edge offsets** beyond each part's perimeter hole pattern. Base mounting tabs are positioned beyond the base's resulting radius.
- **Screws follow the bearings:**
  - Ring-roller bolts thread into the base, and their size comes from the roller bearing's ID.
  - Needle screws go in from below and thread into the output plate; their size comes from the needle roller's ID.
  - Cap screw sizes come from the bearing each cap retains.
- **Cycloid validity:** the cycloid is rejected if it would cusp (roller R ≤ e·(ratio+1)) or **undercut** (the roller is larger than the profile's tightest curve).

### Known limitations

- Screw lengths come straight from the stack and aren't rounded to stock lengths.
- An M3 needle-screw head is almost as wide as a 6 mm needle roller, so it rubs the roller's outer race. A washer would fix this.
- Simple-mode builds take about 15 s because of the exact collision check.

## Credits

CRAP was built in a long back-and-forth between **Dash** and **Claude** (Anthropic's Claude Opus 5.5, working in Claude Code).

**Dash: concept, design and engineering direction**
- Came up with the project and decided what it should be: a parametric, manufacturable cycloidal reducer, made for 2.5-axis machining as well as printing.
- Wrote the foundation: the original dataclasses and the cycloid profile and STEP generator, which still generates the disc today.
- Designed the parts. Hand-drawn sketches and cross-sections defined the base and its mounting tabs, the revolved "top hat" top, the bearing caps and the eccentric insert, and the full section of the output and its stack.
- Made the key engineering calls:
  - bearing support flanges and caps;
  - roller and needle bearings with spacers, midplanes on the cycloid's midplane;
  - which holes are tight or loose, and the face-mount option;
  - bolts entering from the top into the base, and needle screws from below;
  - radii as offsets from hole patterns;
  - separate plate gaps driven by screw heads;
  - bearing-driven screw sizes;
  - a Simple mode that derives everything and iterates until it builds.
- Checked the generated assemblies in SolidWorks and steered every iteration of the GUI's layout and workflow: tabs, modes, the view panel, part toggles, the Rebuild button and section views.

**Claude: implementation**
- Wrote most of the code from those designs:
  - the generators for the base, top, caps, eccentric insert and output;
  - the `gearbox` assembly class with its stack, placement and checks;
  - the hardware helpers;
  - the tolerancing system;
  - the Simple-mode solver and its derivation rules.
- Built the web app: the FastAPI server, the three.js viewer, the motion model (it checked the cycloid's counter-rotation direction against the CAD before animating it), and the export pipeline.
- Did the verification work:
  - the collision checks, which found and fixed the cap-screw/output-plate overlap;
  - the cycloid **undercut** check, after the solver sweep turned up broken profiles;
  - the solver sweep tests and browser testing.
- Wrote this README.
