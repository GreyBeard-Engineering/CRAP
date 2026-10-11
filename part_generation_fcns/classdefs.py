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
    start_angle: float = 0.0 # degrees from +X to the first hole

@dataclass
class motor:
    shaft_r: float
    mounting_holes: patterned_hole
    face_mount: bool = False # screws pass through the base into the motor face (base holes loose)

@dataclass
class flange:
    thickness: float
    offset: float # radial overhang past the bearing race it supports

@dataclass
class cap:
    flange: flange # lip on the part and cap inner edge overhang the bearing OD by offset; both are flange.thickness thick
    bearing_retainer_holes: patterned_hole
    edge_offset: float # free edge's distance beyond the screw holes' far edge: outward for an
                       # outer-race cap (outer radius), inward for an inner-race cap (bore)

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
    cap: cap # disc thickness = cap.flange.thickness + input_bearing.th

@dataclass
class base:
    drive_motor: motor
    shaft_hole_offset: float # r_hole - r_motor_shaft
    roller_bearing: bearing
    roller_holes_R: float # hole r = roller_bearing.ID/2
    roller_holes_N: int
    edge_offset: float # outer edge beyond the roller holes' outer edge (radius sans the mounting tabs)
    t: float
    mounting_holes: patterned_hole # R: hole centers' distance beyond the base radius
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
    edge_offset: float # outer edge beyond the roller holes' outer edge

@dataclass
class eccentric_insert:
    input_bearing: bearing # journal r = ID/2, journal height = th
    flange: flange # supports the bearing inner race; r = ID/2 + offset
    motor: motor # shaft hole r = shaft_r
    e: float # shaft hole offset from the journal axis

@dataclass
class output:
    output_bearing: bearing # journal r = ID/2, journal height = th
    flange: flange # shoulder supporting the bearing inner race; r = ID/2 + offset
    plate_edge_offset: float # plate edge beyond the needle screw holes' outer edge
    plate_t: float
    output_needle_roller: bearing # pin holes r = ID/2
    output_holes_R: float
    output_holes_N: int
    bore_r: float
    cap: cap # inner-race cap bolted to the journal top

@dataclass
class bolt:
    d: float # shank diameter
    length: float # under-head length
    head_d: float
    head_h: float
    socket_af: float # hex socket across-flats
