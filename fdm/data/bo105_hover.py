"""The full-size Bo 105 rotor in hover, for the hover inflow correction (hoverinflow.py).

One test, on NASA Ames' Rotor Test Apparatus in the 40 x 80 ft wind tunnel, its access doors open, the shaft tilted
10 deg forward to reduce recirculation, 1/rev flapping nulled with cyclic:
  [TM-85990]  W. Warmbrodt and R. L. Peterson, "Hover Test of a Full-Scale Hingeless Rotor", NASA TM-85990, 1984:
              the rotor (4 blades, radius 4.91 m, chord 0.27 m, solidity 0.070, -8 deg linear twist, NACA 23012) and
              fig. 3, power (3a) and figure of merit (3b) against CT/sigma at 350, 375, 400 and 425 rpm
  [TM-85892]  R. L. Peterson and W. Warmbrodt, "Hover Test of a Full-Scale Hingeless Helicopter Rotor: Aeroelastic
              Stability, Performance, and Loads Data", NASA TM-85892, 1984: appendix A, the points as numbers
The reports give no blade root cut-out."""
import math

R_M, CHORD_M, BLADES, SIGMA, TWIST_DEG = 4.91, 0.27, 4, 0.070, -8.0
SOUND_MS = 340.0                       # for the figures' rpm; the table's own tip Mach numbers are about 0.634 at 425 rpm

# fig. 3b (rpm, CT/sigma, figure of merit), read at the symbols' centre dots on a 500 dpi rendering, with the axes'
# ticks and tilt measured from the pixels: the points over CT/sigma 0.06 that no other symbol hides
FIG3B = [(375, .0611, .660), (350, .0633, .643), (425, .0624, .616), (350, .0679, .669), (425, .0694, .635),
         (400, .0756, .667), (425, .0796, .656), (350, .0895, .684), (425, .1045, .672), (425, .1208, .643)]
FIG3A_CP0_SIGMA = 0.0010               # fig. 3a at no thrust: its symbols lie from 0.0009 to 0.0012

# TM-85892 appendix A, 425 rpm: collective at 0.7 R (deg), lift CLR/sigma (positive up), propulsive force CXR/sigma
# (positive forward), power CP/sigma. Runs 10.6 and 8.9, 10.5 (low thrust) are left out; the first row is run 10.7
TABLE = [(3.3, -0.000044, 0.000514, 0.001087), (8.9, 0.041104, 0.007211, 0.003093), (11.2, 0.061961, 0.010604, 0.004791),
         (11.8, 0.068577, 0.011804, 0.005423), (13.0, 0.078788, 0.012678, 0.006412), (14.9, 0.104688, 0.017392, 0.009573),
         (16.5, 0.119362, 0.019211, 0.012141)]
TABLE_RPM, TABLE_TIP_MACH, SHAFT_TILT_DEG = 425.0, 0.634, 10.0


def table_thrust(lift, forward):
    """CT/sigma along the shaft, tilted forward: what fig. 3's CT/sigma plots"""
    t = math.radians(SHAFT_TILT_DEG)
    return lift * math.cos(t) + forward * math.sin(t)
