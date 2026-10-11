# CRAP: Cycloidal Reducer Algorithm & Parametrizer

A parametric generator for **cycloidal reducers**. You describe the gearbox (bearings, motor, ratio, eccentricity and roller radius) and CRAP builds every part as solid CAD with [build123d](https://github.com/gumyr/build123d). The parts are made to be machined on a 2.5-axis mill or 3D printed. CRAP:

- assembles them, checks them for collisions through a full turn of the input, animates the mechanism in a browser, and
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
- **Simple mode:** you enter the bearings, the motor (with a *face mount* option), the ratio, the eccentricity and the roller radius. A solver derives every other dimension, including the plate gaps (hidden in this mode), and adjusts until the design builds. Each attempt gets the fast clearance check (below).
- **Advanced mode:** every dimension is editable. Switching from Simple to Advanced pre-fills the solver's choices.
- **Key Parameters** (e, roller R, ratio) are always visible; **Plate Spacing** only in Advanced.
- **Tabs:** Bearings, Motor, and (Advanced only) Parts, with a sub-tab per custom part.
- **Rebuild:** turns orange when the form no longer matches the model on screen. Enter also rebuilds.
- **A design is good to go when nothing collides**, checked in both modes over a full turn of the input. If the solver can't find a clean design, it still builds its best attempt. Colliding parts are tinted, and the colliding part pairs are listed with how deep they overlap and at what input angle; click one to turn the model to that angle, where the overlap is drawn in red. **Revert to last valid design** restores the last clean build. Detailed solver messages go to the server log.

**Left panel: Export**
- STEP (individual parts plus the assembled gearbox) or STL.
- Every part, or custom parts only (everything except the bought bearings and screws).
- Optional **tolerancing**, applied only at export:
  - bearing seats and journals;
  - tight (threaded) and loose (clearance) screw holes;
  - cycloid running clearance.
- Every fit can be overridden under *Advanced fits*. The motor-hole fit follows *face mount*.
- The download is a zip. It includes `export_settings.json`, which records every dimension and tolerance used.
- Designs with colliding parts can't be exported.
- **Configuration:** save every parameter, the mode, the tolerances and the export choices to a JSON file, and load one back later.

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
print(gb.collisions())                    # fast check over a full turn: [(a, b, depth_mm, angle_deg)], [] when clean
print(gb.collisions(mode="exact"))        # exact OCC booleans in the assembled pose (slow)

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
| `part_generation_fcns/envelope.py` | the fast clearance check: cylinder / outline envelopes, posed through a full input turn |
| `part_generation_fcns/errors.py` | coded geometry errors the solver reacts to (raised when strict; recorded when building a best-effort design) |
| `web/` | FastAPI server (`server.py`), mesh payload (`mesh.py`) and the three.js front end (`static/`) |
| `steps/` | generated STEP files (`steps/gearbox/` holds the full assembly) |
| `test_cycloid.py`, `test_solver.py`, `test_envelope.py` | checks; `test_envelope.py` keeps the envelopes honest against the CAD and the exact check; `python test_solver.py --full` also runs the exact check on every solved case |

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
  - Cap screw sizes come from the bearing each cap retains: the largest metric size up to 10% of its OD and half its width.
- **Cycloid cap:** a small ring around the input bearing. Its outer radius is its screw circle + ¼ of the screw-hole radius, so the screws notch its edge, with ¼-screw-radius fillets on the notch corners.
- **Output holes** go as far out as the walls allow, which gives the largest moment arm and so the lowest pin forces. The output plate is sized to clear the ring rollers.
- **Eccentric insert:** the motor shaft, offset by e, keeps at least 1 mm of insert wall inside the input bearing's bore.
- **Fast clearance check:** every part has an *envelope* written next to its generator: Z-aligned cylinders and rings, with its holes as voids. Only the cycloid isn't round, so its outline and the edges of its holes are points instead.
  - Cylinder pairs are compared exactly (centre distance against radii). The cycloid's points are tested against the other parts' cylinders, plus one interior point of each cylinder against the cycloid (for one buried inside it).
  - Every rigid body (housing, input, output, each disc) is posed like the animation, at 360 input angles. One input turn is a full period: the disc turns one lobe per turn.
  - Touching (a screw in a tight hole, a roller on the profile) is allowed; depth is the smaller of the radial and axial overlap.
  - It takes about 0.2 s for one disc, against 5–8 s for the exact check in one pose. Fillets aren't in the envelopes; `test_envelope.py` checks each envelope against its CAD (≤ 1% disagreement) and the fast check against the exact one.
- **Cycloid validity:** a cycloid that would cusp (roller R ≤ e·(ratio+1)) or **undercut** (the roller is larger than the profile's tightest curve) is flagged.

### Known limitations

- Screw lengths come straight from the stack and aren't rounded to stock lengths.
- An M3 needle-screw head is almost as wide as a 6 mm needle roller, so it rubs the roller's outer race. A washer would fix this.
- The fast check samples the motion every 1°, so a collision that comes and goes between samples could be missed. Clearances that small are well under printing tolerances.
- In Simple mode, the cycloid cap's screws sit on a plain ring, so the output holes have to clear their heads radially. That sets the floor on compactness. Placing the screws between the output holes would allow smaller designs.
- Python changes take effect only after the server restarts (close and reopen `launch_gui.bat`).

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
  - a Simple mode that derives everything and iterates until it builds;
  - building failed designs anyway so their collisions are visible, with a revert button;
  - judging designs by collisions alone;
  - the notched cycloid cap and moving the output holes outward.
- Checked the generated assemblies in SolidWorks and steered every iteration of the GUI's layout and workflow: tabs, modes, the view panel, part toggles, the Rebuild button, section views and configuration save/load.

**Claude: implementation**
- Wrote most of the code from those designs:
  - the generators for the base, top, caps, eccentric insert and output;
  - the `gearbox` assembly class with its stack, placement and checks;
  - the hardware helpers;
  - the tolerancing system;
  - the Simple-mode solver, its derivation rules and its best-effort fallback.
- Built the web app: the FastAPI server, the three.js viewer, the motion model (it checked the cycloid's counter-rotation direction against the CAD before animating it), and the export pipeline.
- Did the verification work:
  - the collision checks, which found and fixed the cap-screw/output-plate overlap;
  - the cycloid **undercut** check, after the solver sweep turned up broken profiles;
  - the parallel collision check (about 3× faster), then the envelope-based clearance check (Dash's idea: fit the parts with cylinders, test the cycloid's outline as points);
  - the edge-case and compact-design build campaigns, the solver sweep tests and browser testing.
- Wrote this README.
