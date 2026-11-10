"""Alouette III fuselage aerodynamics (the cabin pod with its engine and transmission deck; without the tail boom, the
empennage, the landing gear and the rotor), wind axes.

Measured [E94: "Preliminary three-dimensional CFD model of a helicopter fuselage", Aerotek (CSIR), 20th ERF 1994,
figs. 14 and 15]: lift and drag of the Alouette III fuselage against angle of attack (-15..6 deg) at zero yaw, on the
rotor disc's 95.0 m^2, from Aerotek's low-speed wind tunnel (1/5 scale, 115 m/s equivalent, Reynolds number 12.6
million on the fuselage length): the body alone (BO) and the body with the engine (BE, the test's body + tail +
engine less body + tail). Read off the figures here (+-0.0001); the full-scale points of the 7 m wind tunnel (ER-7mWT,
its configuration not stated) are kept for comparison: 15 % above the 1/5-scale body with engine.

Not measured (no public data): the yaw, the moments and the large angles. Beyond the tests the coefficients are faired
into a bluff-body cross-flow model whose areas and centres come from the FlightGear 3-D model (fdm/acread.py: the
cabin pod forward of the boom, x > -2.3 m): planform 6.83 m^2 with its centroid 0.68 m forward of the model's
origin, side 3.60 m^2 at x -0.08, z -0.11 m; normal-force coefficients 0.9 (planform) and 0.85 (side) [Hoerner,
Fluid-Dynamic Drag, ch. 3 and 13], as the Bo 105 model's fuselage. The pitching and yawing moments are Munk's
(slender-body potential flow) on the pod's volume, 0.55 of its 4.4 x 1.96 x 1.72 m box (8.2 m^3), with an apparent
mass factor of 0.5 for a bluff body (k2 - k1 is 0.9 for a slender one): destabilising, M/q = 0.5 Vol sin 2 alpha.

Units: forces over dynamic pressure in ft^2, moments in ft^3, about the reference point (FS_REF, BL_REF, WL_REF): the
pod's side-area centroid. Signs as the Bo 105 model's (C-81's): yaw psi + nose right of the wind; side force + right;
rolling moment + right side down; yawing moment + nose right; pitching moment + nose up; lift + up; drag + along the
wind.
"""
import math, os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import al3data as U

M2_FT2 = 10.7639
M3_FT3 = 35.3147
S_REF = 95.0            # m^2 [E94]

# ---------------------------------------------------------------- [E94] figs. 14 and 15, read here
ALPHA_E94 = [-15, -12, -9, -6, -3, 0, 3, 6]
CD_BE = [0.00752, 0.00670, 0.00576, 0.00527, 0.00490, 0.00468, 0.00456, 0.00452]   # body + engine, 1/5 scale
CD_BO = [0.00567, 0.00501, 0.00432, 0.00395, 0.00387, 0.00387, 0.00387, 0.00383]   # body alone
CD_FULL = [None, 0.00706, 0.00642, 0.00573, 0.00553, 0.00535, 0.00531, None]       # full scale, 7 m tunnel
CL_BE = [-0.00832, -0.00632, -0.00388, -0.00250, -0.00120, -0.00040, -0.00047, -0.00047]
CL_BO = [-0.00708, -0.00547, -0.00371, -0.00226, -0.00113, -0.00010, -0.00013, -0.00013]

# ---------------------------------------------------------------- the pod (3-D model) for the cross-flow model
A_TOP, A_SIDE = 6.83 * M2_FT2, 3.60 * M2_FT2      # ft^2
CN_TOP, CN_SIDE = 0.9, 0.85
VOL = 0.55 * 4.4 * 1.96 * 1.72 * M3_FT3          # ft^3
K_MUNK = 0.5
FS_REF, BL_REF, WL_REF = U.model_to_station(-0.08, 0.0, -0.11)
# the planform's centroid lies 0.76 m forward of the reference: the cross-flow normal force's arm
X_TOP = 0.76 * U.FT_PER_M

ALPHA = [-90, -75, -60, -45, -35, -25, -20, -15, -12, -9, -6, -3, 0, 3, 6, 9, 12, 15, 20, 25, 35, 45, 60, 75, 90]
PSI_ABS = [0, 5, 10, 15, 20, 30, 45, 60, 75, 90]
PSI = [-90, -75, -60, -45, -30, -20, -15, -10, -5, 0, 5, 10, 15, 20, 30, 45, 60, 75, 90]


def _interp(xs, ys, x):
    if x <= xs[0]:
        return ys[0]
    for i in range(1, len(xs)):
        if x <= xs[i]:
            t = (x - xs[i - 1]) / (xs[i] - xs[i - 1])
            return ys[i - 1] + t * (ys[i] - ys[i - 1])
    return ys[-1]


def _fair(v0, x0, v1, x1, x):
    """smooth (cosine) blend from v0 at x0 to v1 at x1"""
    t = min(1.0, max(0.0, (x - x0) / (x1 - x0)))
    w = 0.5 - 0.5 * math.cos(math.pi * t)
    return v0 + w * (v1 - v0)


def measured(a):
    """(L/q, D/q) ft^2 at zero yaw, the body with the engine: the tests' -15..6 deg, held flat to 15 deg nose up
    (above 0 deg the lift is flat and the drag nearly so in the tests)"""
    if a > 6:
        return CL_BE[-1] * S_REF * M2_FT2, _interp([6, 15], [CD_BE[-1], CD_BE[-1] + 0.0004], a) * S_REF * M2_FT2
    return _interp(ALPHA_E94, CL_BE, a) * S_REF * M2_FT2, _interp(ALPHA_E94, CD_BE, a) * S_REF * M2_FT2


def crossflow_alpha(a):
    """bluff-body lift and drag over the planform at pitch a (deg)"""
    s, c = math.sin(math.radians(a)), math.cos(math.radians(a))
    n = CN_TOP * A_TOP * s * abs(s)
    return n * c, n * s + measured(0)[1] * c * c


def munk(a):
    return K_MUNK * VOL * math.sin(math.radians(2 * a))


def tables():
    """dict of lists for genal3.fuselage(): L0(a), D0(a), dD(|psi|), dL(|psi|), M(a, |psi|), Y(psi), R(psi), N(psi)"""
    L0, D0, M0 = [], [], []
    for a in ALPHA:
        if abs(a) <= 15:
            l, d = measured(a)
        else:
            e = 15 if a > 0 else -15
            le, de = measured(e)
            cl, cd = crossflow_alpha(a)
            l, d = _fair(le, 15, cl, 45, abs(a)), _fair(de, 15, cd, 45, abs(a))
        L0.append(l); D0.append(d)
        # Munk's moment, and beyond 15 deg the planform's normal force at its centroid ahead of the reference
        mc = CN_TOP * A_TOP * math.sin(math.radians(a)) * abs(math.sin(math.radians(a))) * X_TOP
        M0.append(munk(a) if abs(a) <= 15 else _fair(munk(15 if a > 0 else -15), 15, mc, 60, abs(a)))
    dD, dL = [], []
    for p in PSI_ABS:
        s = math.sin(math.radians(p))
        dD.append(CN_SIDE * A_SIDE * s * s)         # the side's cross-flow drag
        dL.append(0.0)
    Mtab = [[m * math.cos(math.radians(p)) ** 2 for p in PSI_ABS] for m in M0]
    Y, R, N = [], [], []
    zc = 0.0                                        # the side force acts at the reference (the side area's centroid)
    for p in PSI:
        s, c = math.sin(math.radians(p)), math.cos(math.radians(p))
        # psi + nose right of the wind: the wind from the left pushes the pod right (side force +)
        Y.append(CN_SIDE * A_SIDE * s * abs(s))
        R.append(0.0)
        N.append(-K_MUNK * VOL * math.sin(math.radians(2 * p)) * (-1))   # Munk: destabilising, nose further right
    return dict(alpha=ALPHA, L0=L0, D0=D0, psi_abs=PSI_ABS, dD=dD, dL=dL, M=Mtab, psi=PSI, Y=Y, R=R, N=N)


if __name__ == '__main__':
    t = tables()
    print('D/q at 0 deg %.2f ft^2 (%.3f m^2); full scale %.3f m^2' % (t['D0'][ALPHA.index(0)], t['D0'][ALPHA.index(0)] / M2_FT2,
                                                                 CD_FULL[5] * S_REF))
    for a, l, d, m in zip(t['alpha'], t['L0'], t['D0'], [r[0] for r in t['M']]):
        print('%4d  L %7.2f  D %7.2f  M %8.1f' % (a, l, d, m))
