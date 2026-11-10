"""Sud Aviation SA 316B Alouette III data for the JSBSim model, with sources.

  [TC]  EASA TCDS R.123 issue 3 (2017), section 1: SE 3160 and SA 316 B: dimensions, rotor diameters, engine and
        transmission limits, rotor speed limits (420 / 270 / 353 rpm), VNE against mass and altitude, masses, CG range
        and datum (3000 mm forward of the main rotor centre line), fuel capacity
  [TE]  EASA TCDS E.091 issue 3 (2024): Artouste III B and B1: one axial and one centrifugal compressor stage, annular
        combustor with a centrifugal fuel injection wheel, three-stage axial turbine, co-axial reduction gearbox;
        33 500 +-200 rpm (transient +-1000, overspeed 35 500 for 10 s), output shaft 5773 rpm; take-off 420 kW (B)
        and 440 kW (B1), maximum continuous 405 kW; T4 630 C starting, 550 C take-off, 500 C continuous; exhaust pipe
        outlet 348 mm; bleed air 170 g/s at most; hydromechanical control; dry weight 178 kg
  [TD]  This Day in Aviation, "Sud-Aviation SA 316B Alouette III" and "World record for altitude without payload"
        (Lama), after Aerospatiale's data: main rotor clockwise seen from above (advancing blade on the left), 353.2
        rpm, 270-420 rpm in autorotation; tail rotor three-bladed, on the right side of the boom, a pusher, clockwise
        seen from the left (advancing blade below: the top blade moves aft), 1.912 m, 2001 rpm (with a main rotor at
        about 365 rpm: see [SR]); at 2200 kg: cruise 185
        km/h, VNE 210 km/h, hover ceiling 1650 m in ground effect and 100 m out of it; at 1750 kg: 5550 m and 1650 m;
        climb 4.3 to 8.7 m/s at those weights
  [HA]  heli-archive.ch, "SE 3160 Alouette III": fully articulated rotor, three metal blades (light alloy extruded
        spar, sheet skin, stainless steel leading edge), constant chord, twisted; centrifugal clutch, three-stage main
        gearbox; Artouste IIIB single-spool, constant speed, governed at 33 500 rpm; hydraulic servo controls;
        performance at 2100 kg (Swiss Air Force)
  [E76] Aerospatiale, "Design philosophy for helicopter rotor heads", 2nd ERF 1976: the Alouette rotor head weighs 100
        kg (377 parts), with torsion tie-bars of stacked stainless steel strips
  [E77] Turbomeca, "Engine regulation", 3rd ERF 1977: the Artouste's control system holds the engine speed constant
        whatever the power, with a hydromechanical isochronous PI governor (remanent error under 0.25 %) and a gear
        fuel pump; the automatic start runs on the pump's characteristic with a relay sequence (starter, igniters);
        the pilot selects only the collective
  [E88] "Eight years of flight operations with composite rotor blades", 14th ERF 1988: the Alouette II's metal blade,
        28.9 kg, 4250 mm long, 280 mm chord, NACA 0012, twist 1 deg 36' per metre
  [E94] "Preliminary three-dimensional CFD model of a helicopter fuselage" (Aerotek, CSIR), 20th ERF 1994: the Alouette
        III fuselage's lift and drag against angle of attack, 1/5-scale wind tunnel tests (body; body and engine)
        and full scale, on the rotor disc (95 m^2) (figs. 14, 15, read in data/e94_fuselage.py)
  [MI]  Michelin, Aircraft Tire Engineering Data: SA 319 Alouette III, main and nose 355 x 150-4, 4 ply, tube type;
        the 355 x 150-4: rated load 550 kg at 2.7 bar, bottoming load 1650 kg, outside diameter 13.42-14.42 in, width
        5.75-6.50 in, static loaded radius 142 mm at the rated load, rim 4 in
  [GY]  Goodyear, Global Aviation Tires data: SA319 Alouette III, 355x150 tube type, main and nose
  [WA]  Wikipedia, Turbomeca Artouste (Artouste IIIB): overall pressure ratio 5.2, air mass flow 4.3 kg/s, specific
        fuel consumption 0.46 kg/kWh (equivalent), take-off 410 kW plus 0.39 kN of jet thrust
  [KU]  KU Leuven, Turbomeca Artouste (an Artouste II on display): reduction gearbox 5.8025:1
  [AC]  aviation-collector.be, "Alouette 3 tail rotor blade": an SA 316B tail rotor blade, 840 mm long, 170 mm wide,
        75 mm deep (its root fitting), 1.436 kg
  [VP]  Vertipedia (VFS), Sud SA.316B Alouette III: empty weight 1143 kg, climb 846 ft/min, top blade aft
  [LM]  SA 315B Lama, the user manual of a flight simulator add-on (2023), its procedures after the flight manual
        (the Lama has the Alouette III's dynamic system and an Artouste III): the fuel flow lever at 35 % for idle,
        the rotor starts turning at 19 500-21 000 rpm (the centrifugal clutch); rotor brake below 175 rpm; a 1 m
        hover for the take-off. Not the manufacturer's document: its own idle, about 19 000 rpm, is not used
  [RW]  S. Skola, "ROTORwrench: Tips, Tricks and Info", JustHelicopters, November 2017, on the SA 315B Lama's
        Artouste IIIB1 (the Alouette III's engine, at the same 33 500 rpm): the engine idles at 17 000-18 000 rpm,
        the starter comes off line about 14 000 rpm, the fuel flow lever is closed for the start
  [HL]  Hindustan Aeronautics' technical description of an SA 316B (a licence-built 1976 aircraft, quoted in its sale
        listing): main rotor chord 350 mm, NACA 63A section of 11.5 %, twist 6 deg 30', hydraulic lag dampers; tail
        rotor NACA 0015 to 0018; self-centring nose wheel steerable 45 deg; disc parking brakes; average fuel burn
        175-185 l/h
  [RS]  "Aerodynamic analysis of Alouette III rotary wings", Review of the Air Force Academy (Brasov) 1/2020, after the
        Alouette III pilot's handbook: the blade's chord, 355 mm at 0.6 m from the shaft to 340 mm at the tip
  [SR]  sound recordings of governed Alouette III and Lama rotors (seven of eight, in the cabin, the cockpit and
        outside, analysed with the PyWopWop sound model's tools): the tail rotor's blade passing at 5.487 times the
        main rotor's, so 1938 rpm at 353.2 rpm; HAL's pair, 2001 rpm with a 363 rpm main rotor [HL], gives 5.51.
        2001 rpm [TD] is thus the tail rotor's speed at a main rotor's of about 365 rpm, not at 353.2; and two recorded
        starts, the spool's speed from the engine's 11th and 16th orders and the rotor's from the main gearbox's lines:
        ZU-REC's (cockpit, the checklist start: the spool at 10 % 4 s after the starter, light-off at 16 % at about
        14.5 s, 30 % at 27 s, 45 % at 38 s, idle at 16 330 rpm, its tachometer about 16 500, by about 45 s; the spool held at 77-80 % while the clutch slips, the rotor
        from rest to 55 % in 21 s, 75 % in 39 s, locked at 80 % in 45 s) and A-301's (outside, the spool held at
        59-63 %: the rotor locked at about 63 % in 64 s)
  [AO]  R. W. Sweazey, "Pilot Flight Check: Alouette III", The AOPA Pilot, October 1972 (an SE 3160): the engine
        idles at about 15 000 rpm and the clutch engages as it passes 19 000 (the model's clutch from there);
        autorotation at about 1700 ft/min
  [3D]  the FlightGear Alouette-III 3-D model (Gerard Robin): the blades' flap hinge 0.119 m from the shaft, the
        blade's aerodynamic span 0.96-5.49 m and chord 0.348 m, the hub, the tail rotor's hub, its 5 deg toe and its
        blades' 0.16 m chord outboard of 0.20 m, the stabiliser, the end plates, the wheels

Frames: the JSBSim structural frame in inches, FS aft from 3000 mm (118.11 in) ahead of the main rotor axis, the
type certificate's datum [TC]; BL right +; WL up +, zero at the FlightGear model's origin (the JSBSim VRP). The
3-D model's frame after its offsets in Alouette-III-sc_jsb.xml (pitched 1 deg nose down, raised 0.115 m) is the
aircraft's body frame (YASim frame: x forward, y left, z up, metres: fdm/acread.py).

Mirror: the Alouette's rotor turns clockwise seen from above, and its tail rotor sits on the right pushing left: the
mirror image of the Bo 105's and the UH-60's layouts, whose equations (anticlockwise rotor, tail rotor on the left
pushing right) the generator keeps. The aerodynamics are therefore worked out in the aircraft's mirror image across
its plane of symmetry (BL negated, the lateral velocity, the roll and yaw rates and the lateral acceleration
negated), and their side force, rolling and yawing moments negated back. Geometry here is given as it is on the real
aircraft; the generator mirrors it (_M names).
"""
import math, os

# the FDM rate (FlightGear's /sim/model-hz, the harness's step), and the frames between runs of a part that needs only
# hz: every(240) is the execrate of a 240 Hz channel. AL3_HZ overrides it (for testing the model at other rates)
FDM_HZ = int(os.environ.get('AL3_HZ', 480))


def every(hz):
    return max(1, int(round(FDM_HZ / float(hz))))


DEG = math.pi / 180
G0 = 32.174
IN_PER_M = 39.3701
FT_PER_M = 3.28084
KG_LB = 2.20462
KGM2 = 0.737562            # kg m^2 -> slug ft^2

# ---------------------------------------------------------------- frames
# the 3-D model's hub (rotor model origin, Alouette-III-sc_jsb.xml offsets -0.0051, 0, 1.59275 m, pitched 1.5 deg
# forward) in the body frame (YASim): x 0.0329 m forward of the model's origin, z 1.7074 m above it
HUB_YASIM = (0.0329, 0.0, 1.7074)
FS_AXIS = 3000.0 / 25.4     # the main rotor axis [TC: the datum is 3000 mm forward of it]


def model_to_station(x, y, z):
    """body frame (YASim: x forward, y left, z up, m) -> (FS, BL, WL) in"""
    return FS_AXIS + (HUB_YASIM[0] - x) * IN_PER_M, -y * IN_PER_M, z * IN_PER_M


FS_MODEL0, WL_MODEL0 = model_to_station(0.0, 0.0, 0.0)[0], 0.0     # the VRP: the model's origin

# ---------------------------------------------------------------- main rotor
R = 5.51 * FT_PER_M         # ft, 11.02 m diameter [TC SA 316 C, SA 319 B; TD; HA] (the SE 3160 section: 11.00)
NB = 3
OMEGA = 353.2 * 2 * math.pi / 60     # rad/s at 100 % [TD; TC: 353 rpm maximum continuous]
NR_MAX_RPM, NR_MIN_RPM = 420.0, 270.0   # power-off limits [TC]
CHORD = 0.35 * FT_PER_M     # ft: 350 mm [HL] (the handbook's 355-340 mm taper [RS] averages 347; the 3-D model's 0.348 m)
R_ROOT = 0.96 * FT_PER_M    # ft: the blade's aerodynamic span starts 0.96 m from the shaft [3D] (0.174 R)
E_FLAP = 0.119 * FT_PER_M   # ft: flap hinge offset [3D: the blade's flap animation centre], 2.2 % R
# twist: 6 deg 30' [HL], taken as linear from the shaft (zero there) to the tip. (The Alouette II blade's 1 deg 36' per
# metre [E88], 8.8 deg over this span, stood in for it before.) The section: NACA 63A, 11.5 % thick [HL], taken as the
# symmetrical 63A011.5; its tables are C-81's 0012 with the 63A's measured differences (data/sections.py)
TWIST = -6.5                # deg, shaft to tip [HL]
TWIST_PER_M = TWIST / 5.51
SHAFT_TILT = 2.5 * DEG      # forward: the rotor model pitched 1.5 deg forward on the model, which is pitched 1 deg [3D]
FS_MR, BL_MR, WL_MR = model_to_station(*HUB_YASIM)
TIP_LOSS = 0.97             # B (square tip), as the UH-60 and Bo 105 models
NSEG = 8
# section drag increment over the tables' smooth section: a production metal blade's (its steel leading-edge cap and
# skin joints), the UH-60 model's for its metal-spar blades [Bousman, NASA TP-2003-212265]
DCD = 0.0012
SWASH_PHASE = 0.0           # deg: the control phasing is not published; set to cancel the hinge offset's phase below
# Blade mass: the Alouette II's metal blade (the same construction [HA, E88]) is 28.9 kg for 4.25 m of span and 0.28 m of
# chord [E88]; scaled by span and chord to the Alouette III's 4.55 m and 0.35 m: 38.7 kg, uniform over 0.96-5.51 m.
# Outboard of the flap hinge the sleeve, pitch bearing and lag hinge flap with it: 12 kg about 0.45 m from the shaft
# (estimate: part of the head's 100 kg [E76]; the rest, 64 kg, turns with the hub)
M_BLADE = 38.7              # kg
M_SLEEVE, X_SLEEVE = 12.0, 0.45
M_HUB = 100.0 - NB * M_SLEEVE


def blade_mass_properties():
    """(mass kg, first moment kg m, inertia kg m^2) of one flapping blade about its hinge"""
    e, a, b = 0.119, 0.96, 5.51
    a_, b_ = a - e, b - e
    I = M_BLADE * (b_ ** 3 - a_ ** 3) / (3 * (b_ - a_)) + M_SLEEVE * (X_SLEEVE - e) ** 2
    S = M_BLADE * (a_ + b_) / 2 + M_SLEEVE * (X_SLEEVE - e)
    return M_BLADE + M_SLEEVE, S, I


_m, _S, _I = blade_mass_properties()
W_BLADE = _m * KG_LB                    # lb
S_B = _S * KG_LB / G0 * FT_PER_M        # slug ft
I_B = _I * KGM2                         # slug ft^2 (444 kg m^2: Lock number 5.1 at sea level)
FLAP_STOP = (-3.0 * DEG, 20.0 * DEG)    # droop and flap stops (not published)
# lead-lag: a lag hinge at the flap hinge (the type's are not published), with the blade's mass properties; lag
# frequency sqrt(e S / I) 0.18 / rev. The lag damper is not published: viscous, 25 % of critical at 100 % rotor speed
LAG_STOP = (-12.0 * DEG, 8.0 * DEG)     # lag / lead stops (not published)
LAG_ZETA = 0.25
LAG_C = LAG_ZETA * 2 * I_B * math.sqrt(E_FLAP * S_B / I_B) * OMEGA   # ft lb s / rad

# ---------------------------------------------------------------- tail rotor
R_TR = 0.956 * FT_PER_M     # ft: 1.912 m [TD; TC 1.91]
NB_TR = 3
GEAR_TR = 5.487            # tail / main rotor speed [SR]: 1938 rpm at 353.2 (2001 rpm [TD] at 364.7)
CHORD_TR = 0.17 * FT_PER_M  # ft: a real SA 316B blade's 170 mm width [AC] (the 3-D model's blade 0.16 m)
R_ROOT_TR = 0.20 * FT_PER_M # ft: the blade's aerodynamic span starts 0.20 m from the hub [3D]
# a real blade: 840 mm, 1.436 kg [AC]: from 0.116 m to the tip, uniform; its flap hinge at 0.06 m (not published)
M_BLADE_TR, E_TR_M = 1.436, 0.06
_a, _b = 0.956 - 0.84 - E_TR_M, 0.956 - E_TR_M
I_BLADE_TR = M_BLADE_TR * (_b ** 3 - _a ** 3) / (3 * (_b - _a)) * KGM2     # slug ft^2 (0.42 kg m^2)
E_TR = E_TR_M * FT_PER_M
TWIST_TR = 0.0              # flat [3D]; its sections NACA 0018 (root) to 0015 (tip) [HL] (data/sections.py)
DELTA3_TR = 0.0             # pitch-flap coupling: not published
FLAP_STOP_TR = 12.0 * DEG
TOE_TR = 5.0 * DEG          # the shaft toed 5 deg, thrust forward of lateral [3D: spin axis (0.0875, 1, 0)]
# hub [3D]: body frame (-6.2496, -0.3073, 0.6621), on the right of the boom
FS_TR, BL_TR, WL_TR = model_to_station(-6.2496, -0.3073, 0.6621)
TIP_LOSS_TR = 0.94
NSEG_TR = 6
TR_MODEL = 'blades'

# ---------------------------------------------------------------- engine and drive [TE, TC, WA, KU, E77]
N_ENGINE = 33500.0          # rpm at 100 %
N_OUTPUT = 5773.0           # rpm, the engine's output shaft at 100 % [TE]
GEAR_ENGINE = N_ENGINE / (OMEGA * 60 / (2 * math.pi))   # engine / main rotor speed (94.85)
P_TO_KW, P_MCP_KW = 420.0, 405.0          # engine ratings at the reduction gear [TE, TC]
P_TRANS_TO_KW, P_TRANS_MCP_KW = 440.0, 420.0   # SA 316 B transmission limits [TC]
P_THERMAL_KW = 640.0        # maximum thermal power on the ground, sea level [TC]
T4_START, T4_TO, T4_MCP = 630.0, 550.0, 500.0   # C [TE]
PRESSURE_RATIO, AIR_FLOW = 5.2, 4.3       # at 100 %, sea level [WA]
SFC = 0.46                  # kg/kWh at the rating [WA]

# ---------------------------------------------------------------- airframe [3D unless noted]
# horizontal stabiliser: the 3-D model's (0.86 m^2 in plan with its end plates' tops; 2.38 m span), fixed
S_H = 0.80 * 10.7639        # ft^2 (the surface between the end plates)
SPAN_H = 2.38
FS_H, BL_H, WL_H = model_to_station(-4.95, 0.0, 0.40)
INC_H = 0.0                 # deg, incidence: not published (the test programme checks the trim attitude)
# end plates: the 3-D model's two plates, 0.37 m^2 together in side view, at the stabiliser's tips
S_EP = 0.18 * 10.7639
FS_EP, BL_EP, WL_EP = model_to_station(-4.95, -1.17, 0.40)   # the right one

# ---------------------------------------------------------------- mass and loading [TC, VP]
EMPTY_KG = 1143.0           # [VP] (heli-archive: about 1105 kg for the SE 3160; 1122-1230 kg elsewhere)
MTOW_KG = 2200.0            # [TC]
CG_FWD_MM, CG_AFT_MM = 2780.0, 3080.0      # from the datum [TC] (3080-3150 mm single-seat, VNE 190 km/h)
CG_LAT_MM = (-140.0, 120.0)                # left, right [TC]
FUEL_L, FUEL_USABLE_L = 565.0, 555.0       # cubic tank [TC]
FUEL_KG_L = 0.8

# ---------------------------------------------------------------- landing gear [MI, GY, 3D]
# tricycle: a castoring, self-centring nose wheel on a trailing link and two main wheels on swing arms with
# oleo-pneumatic struts; 355 x 150-4 tyres on all three [MI, GY]
TYRE_DIAMETER = 0.354       # m: the 3-D model's wheels (0.36 m), the tyre's 13.42-14.42 in [MI]
TYRE_RATED_KG, TYRE_PSI = 550.0, 2.7 * 14.5038
TYRE_STATIC_RADIUS = 0.142  # m at the rated load [MI]
TYRE_BOTTOMING_KG = 1650.0
# wheels' contact points (body frame, the tyres' lowest points unloaded) [3D]
NOSE_WHEEL = (2.02, 0.0, -1.25)
MAIN_WHEELS = ((-0.98, 1.185, -1.20), (-0.98, -1.185, -1.20))


# tail rotor blade's first moment about its hinge (uniform, 1.436 kg over 0.116-0.956 m [AC])
S_BLADE_TR = M_BLADE_TR * (_a + _b) / 2 * KG_LB / G0 * FT_PER_M      # slug ft

# ---------------------------------------------------------------- controls (not published: estimates)
# Hydraulic servos on the collective and cyclic [HA]; no stability augmentation. The rigging is not public (the type
# certificate refers to the maintenance manual for it). The collective spans what the hover, the climb at altitude and
# an autorotation at 420 rpm need: -1.2 to 14.8 deg at 0.7 R (the hover at 2200 kg wants about 8.7 there); the cyclic
# and pedal ranges are typical of the type's era. Pitch at the shaft (the twist is zero there).
COLL_ROOT = (3.38, 19.38)        # deg, lever down / up: -1.17 and 14.83 deg at 0.7 R with the twist
B1S_RANGE = (-10.0, 14.0)        # deg, full aft / full forward (+ = disc forward)
A1S_RANGE = (-7.0, 7.0)          # deg, full left / full right (+ = disc right, the real aircraft's)
TR_PITCH_RANGE = (-8.0, 22.0)    # deg, full left / full right pedal; + = thrust to the left (anti-torque)
COLL_IN, LON_IN, LAT_IN, PED_IN = 9.0, 12.0, 9.0, 4.4    # control travels, in (the cockpit's animations)

# ---------------------------------------------------------------- mass build-up (estimates against the totals)
# The empty mass and its CG are the type's [VP, TC]; how it is distributed is not published: the components below are
# estimates (engine [TE], rotor head [E76] and blades as above) from the 3-D model's layout, scaled so that they add up
# to the empty mass. Body frame (YASim: x forward, y left, z up, m); each a point mass plus its own radii of gyration
# (kx, ky, kz, m). They give the inertias; the blades' own rotation (flapping, lag, spin) is the rotor model's, so the
# blades enter as point masses at the hub.
COMPONENTS = [
    # name, kg, (x, y, z), (kx, ky, kz)
    ('engine', 178.0, (-1.55, 0.0, 0.85), (0.20, 0.55, 0.55)),
    ('main gearbox, mast', 120.0, (0.03, 0.0, 1.05), (0.25, 0.25, 0.20)),
    ('rotor hub', M_HUB, HUB_YASIM, (0.15, 0.15, 0.20)),
    ('blades (3)', NB * (M_BLADE + M_SLEEVE), HUB_YASIM, (0.0, 0.0, 0.0)),
    ('cabin structure, floor, glazing', 230.0, (1.20, 0.0, -0.10), (0.75, 1.05, 1.00)),
    ('centre frame, tank structure', 90.0, (-0.40, 0.0, 0.10), (0.60, 0.40, 0.60)),
    ('tail boom', 65.0, (-3.70, 0.0, 0.30), (0.20, 1.10, 1.10)),
    ('tail rotor, gearbox, guard', 32.0, (-6.20, 0.0, 0.55), (0.25, 0.30, 0.30)),
    ('stabiliser, end plates', 10.0, (-4.95, 0.0, 0.40), (0.70, 0.20, 0.70)),
    ('main gear', 60.0, (-0.98, 0.0, -0.85), (1.15, 0.20, 1.15)),
    ('nose gear', 18.0, (2.02, 0.0, -0.95), (0.10, 0.10, 0.10)),
    ('seats, controls, instruments, systems', 150.0, (1.30, 0.0, 0.05), (0.60, 0.60, 0.60)),
]


def segments(n=NSEG, r0=R_ROOT, r1=R):
    """equal annulus area segments: (r_mid, dr) in ft, and the edges"""
    a = [r0 * r0 + (r1 * r1 - r0 * r0) * k / n for k in range(n + 1)]
    edges = [math.sqrt(x) for x in a]
    return [(math.sqrt((edges[k] ** 2 + edges[k + 1] ** 2) / 2), edges[k + 1] - edges[k]) for k in range(n)], edges


def twist(r):
    """deg at r (ft): linear from the shaft"""
    return TWIST * r / R


# ---------------------------------------------------------------- inflow, ground effect
PH_COS = [(1, 0), (3, 0), (5, 0), (2, 1)]
PH_SIN = [(2, 1)]
GE_K, GE_EXP = 0.174, -0.5   # the Bo 105's and UH-60's (not fitted: the report sets the ground effect the autopilot flies against
                             # the published ceilings in and out of it)
GE_ZR_MIN = 0.3

if __name__ == '__main__':
    print('blade %.1f kg, S %.1f kg m, I %.0f kg m^2; Lock number %.2f' % (_m, _S, _I, 1.225 * 5.73 * 0.35 * 5.51 ** 4 / _I))
    print('lag frequency %.3f /rev, damper %.0f ft lb s/rad' % (math.sqrt(E_FLAP * S_B / I_B), LAG_C))
    print('hub FS %.2f WL %.2f; TR FS %.2f BL %.2f WL %.2f; VRP FS %.2f' % (FS_MR, WL_MR, FS_TR, BL_TR, WL_TR, FS_MODEL0))
    print('gear ratios: engine/rotor %.3f, tail %.4f; TR blade I %.4f slug ft^2' % (GEAR_ENGINE, GEAR_TR, I_BLADE_TR))
