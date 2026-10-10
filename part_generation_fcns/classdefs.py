from dataclasses import dataclass

@dataclass
class bearing:
    ID: float
    OD: float
    th: float

@dataclass
class patterned_hole:
    r: float
    R: float
    N: int

@dataclass
class motor:
    shaft_r: float
    mounting_holes: patterned_hole

@dataclass
class cap:
    bearing_support_flange_offset: float # lip on the part and cap inner edge both overhang the bearing seat by this
    bearing_support_flange_thickness: float # thickness of the lip and of the cap
    bearing_retainer_holes: patterned_hole
    r: float # cap outer radius

@dataclass
class cycloid:
    roller_bearing: bearing
    roller_R: float
    input_bearing: bearing
    output_needle_roller: bearing # output pin rollers; hole r = OD/2 + e
    output_holes_R: float
    output_holes_N: int
    e: float
    ratio: int
    cap: cap # disc thickness = cap.bearing_support_flange_thickness + input_bearing.th

@dataclass
class base:
    drive_motor: motor
    shaft_hole_offset: float # r_hole - r_motor_shaft
    roller_bearing: bearing
    roller_holes_R: float # hole r = roller_bearing.ID/2
    roller_holes_N: int
    outer_r: float # radius of the part sans the mounting holes
    t: float
    mounting_holes: patterned_hole # the R of these holes must exceed outer_R
    mounting_hole_offset: float # mounting hole is concentric with a ring of material; r_ring - r_hole = mounting_hole_offset. Like a donut
    mounting_hole_fillet_r: float # fillet radius between mounting holes and base

@dataclass
class top:
    output_bearing: bearing
    output_bearing_offset: float
    base_thickness: float
    roller_bearing: bearing
    roller_holes_R: float # hole r = roller_bearing.ID/2
    roller_holes_N: int
    cap: cap
    outer_r: float # radius of the base disc