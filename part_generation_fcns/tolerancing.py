"""Export-time fits. The nominal model has none; a tolerancing grows or shrinks each
fitted feature's radius according to its fit ("tight" or "loose") and its kind:

  bolt    hole  tight: shrink by `threaded`  (bolt or pin threads / presses in)
                loose: grow by `loose`       (bolt passes through)
  bearing hole  tight: shrink by `bearing`   loose: grow by `bearing`
          shaft tight: grow by `bearing`     loose: shrink by `bearing`

`cycloid_clearance` offsets the cycloid's rolling surfaces (outer profile and output holes)
away from the rollers.
"""
from dataclasses import dataclass, field

# feature id -> (part, description, kind, shape, default fit)
FEATURES = {
    "base.motor_holes":           ("base", "motor mounting holes", "bolt", "hole", "tight"),
    "base.mounting_holes":        ("base", "mounting tab holes", "bolt", "hole", "loose"),
    "base.roller_holes":          ("base", "ring roller bolt holes", "bolt", "hole", "tight"),
    "top.roller_holes":           ("top", "ring roller bolt holes", "bolt", "hole", "loose"),
    "top.retainer_holes":         ("top", "top cap screw holes", "bolt", "hole", "tight"),
    "top.bearing_seat":           ("top", "output bearing seat (OD)", "bearing", "hole", "loose"),
    "top_cap.holes":              ("top_cap", "screw holes", "bolt", "hole", "loose"),
    "cycloid.bearing_seat":       ("cycloid", "input bearing seat (OD)", "bearing", "hole", "loose"),
    "cycloid.retainer_holes":     ("cycloid", "cycloid cap screw holes", "bolt", "hole", "tight"),
    "cycloid_cap.holes":          ("cycloid_cap", "screw holes", "bolt", "hole", "loose"),
    "eccentric_insert.journal":   ("eccentric_insert", "journal (input bearing ID)", "bearing", "shaft", "loose"),
    "eccentric_insert.shaft_hole": ("eccentric_insert", "motor shaft hole", "bearing", "hole", "loose"),
    "output.journal":             ("output", "journal (output bearing ID)", "bearing", "shaft", "loose"),
    "output.needle_holes":        ("output", "needle screw holes", "bolt", "hole", "tight"),
    "output.retainer_holes":      ("output", "output cap screw holes", "bolt", "hole", "tight"),
    "output_cap.holes":           ("output_cap", "screw holes", "bolt", "hole", "loose"),
    "roller_spacer.bore":         ("roller spacers", "bore (ring roller bolt)", "bolt", "hole", "loose"),
    "needle_spacer.bore":         ("needle_spacer", "bore (needle screw)", "bolt", "hole", "loose"),
}

DEFAULT_MAGNITUDES = {"bearing": 0.05, "threaded": 0.10, "loose": 0.15, "cycloid_clearance": 0.05}


@dataclass
class tolerancing:
    enabled: bool = False
    bearing: float = 0.0            # radial, mm
    threaded: float = 0.0
    loose: float = 0.0
    cycloid_clearance: float = 0.0
    fits: dict = field(default_factory=lambda: {k: v[4] for k, v in FEATURES.items()})

    def __post_init__(self):
        for k in ("bearing", "threaded", "loose", "cycloid_clearance"):
            if getattr(self, k) < 0:
                raise ValueError(f"tolerance {k} must be >= 0")
        for k, v in self.fits.items():
            if k not in FEATURES:
                raise ValueError(f"unknown fitted feature {k!r}")
            if v not in ("tight", "loose"):
                raise ValueError(f"fit for {k} must be 'tight' or 'loose', got {v!r}")

    def delta(self, fid: str) -> float:
        """Signed change to the feature's nominal radius (positive = larger radius)."""
        if not self.enabled:
            return 0.0
        _, _, kind, shape, _ = FEATURES[fid]
        loose = self.fits[fid] == "loose"
        mag = self.bearing if kind == "bearing" else (self.loose if loose else self.threaded)
        grow = loose if shape == "hole" else not loose     # a loose shaft gets smaller
        return mag if grow else -mag

    @property
    def clearance(self) -> float:
        return self.cycloid_clearance if self.enabled else 0.0


NOMINAL = tolerancing()
