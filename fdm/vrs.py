#!/usr/bin/env python3
"""Vortex ring state: Johnson's model of a rotor's mean induced velocity in descent [W. Johnson, Model for Vortex Ring
State Influence on Rotorcraft Flight Dynamics, NASA/TP-2005-213477, 2005: table 3's algorithm with table 4's
parameters]. All velocities over vh, the hover's ideal induced velocity at the rotor's thrust: z = Vz/vh the rotor's
axial velocity (climb +), x = Vx/vh its edgewise velocity, nu = v/vh its mean induced velocity.

Momentum theory (Glauert's, nu^2 (x^2 + (z + nu)^2) = 1) is used for climb and for x >= XC; below it a cubic joins its
normal working branch at A to its windmill brake branch at B, removing the singularity at ideal autorotation (the
baseline model); within the vortex ring state three cubics add an increment to the total flow z + nu between D and E
with zero slope at N and X, the stability boundary, so that the total flow rises with the descent rate between them (the
VRS model). The points move with x as table 3 gives; the increment fades out by x = XM.

Two misprints of table 3 are read as its text gives them [TP p. 14]: 3.2.3's last match is at E, where the slope is not
matched and the constant term is zero (the table has D); 3.2.4's polynomial slope is b + 2c z + 3d z^2 (2c z^2). The
increments at N and X are table 4's inflow there less axial momentum theory's at Vz_N and Vz_X (0.05 and 0.75), times
the fade (1 - (x/XM)^6)^0.5: so the points of figure 39 fall (at x = 0.8 the curve's minimum and maximum, z + nu = 0.537
and 0.768, the figure's 0.53 and 0.76; with the momentum value at the moved point and x instead, 0.78 and 1.04).

  python3 vrs.py            the model's check points against figures 38 and 39, and its tables' interpolation error
"""
import math
import numpy as np

# table 4 [TP]: the baseline's A, B, C and the VRS model's D, N, X, E, M
ZA, ZB, XC = -1.5, -2.1, 0.75
ZD, ZN, VN, ZX, VX, ZE, XM = -0.2, -0.45, 0.85, -1.5, 1.25, -2.0, 0.95
# the time lag of the mean inflow's change of flow state, rotor revolutions times lambda_h [TP p. 16: 14 revs for the
# D6075 at lambda_h 0.05, from its flight test's Vz drop; t ~ R/vh, so tau_rev = 0.7 / lambda_h]
TAU_REV_LH = 0.7


def momentum(z, x):
    """the positive real roots nu of Glauert's momentum theory, nu^2 (x^2 + (z + nu)^2) = 1, ascending"""
    r = np.roots([1.0, 2 * z, z * z + x * x, 0.0, -1.0])
    r = sorted(float(q.real) for q in r if abs(q.imag) < 1e-9 and q.real > 0)
    return r


def dmomentum(z, x, nu):
    """d nu / dz along a momentum branch"""
    w = z + nu
    return -nu * w / (x * x + w * w + nu * w)


def upper(z, x):
    return momentum(z, x)[-1]


def lower(z, x):
    return momentum(z, x)[0]


def cubic(conds):
    """coefficients (a, b, c, d) of a + b z + c z^2 + d z^3 from conditions (kind, z, value): kind 0 a value, 1 a
    slope, 'a' the constant term"""
    A, y = [], []
    for kind, z, val in conds:
        if kind == 0:
            A.append([1, z, z * z, z ** 3])
        elif kind == 1:
            A.append([0, 1, 2 * z, 3 * z * z])
        else:
            A.append([1, 0, 0, 0])
        y.append(val)
    return np.linalg.solve(np.array(A, float), np.array(y, float))


def poly(c, z):
    return c[0] + c[1] * z + c[2] * z * z + c[3] * z ** 3, c[1] + 2 * c[2] * z + 3 * c[3] * z * z


def base_points(x):
    """table 3, 2.1: the baseline cubic's ends A and B at x"""
    za = ZA + 0.2 * (x / XC) ** 2
    zb = ZB + 0.2 * (x / XC) ** 2
    if x / XC > 0.5:
        zb += 0.7 * (za - zb) * (2 * x / XC - 1) ** 3
    return za, zb


def baseline(z, x):
    """table 3, 1-2: the baseline model's nu and d nu / dz"""
    if z >= 0 or x >= XC:
        nu = upper(z, x)            # one root there (for x >= XC at least down to z = -2.5: the check below)
        return nu, dmomentum(z, x, nu)
    za, zb = base_points(x)
    if zb < z < za:
        na, nb = upper(za, x), lower(zb, x)
        c = cubic([('a', 0, 0.0), (0, za, na), (1, za, dmomentum(za, x, na)), (0, zb, nb)])
        return poly(c, z)
    nu = upper(z, x) if z >= za else lower(z, x)
    return nu, dmomentum(z, x, nu)


def vrs_points(x):
    """table 3, 3.1: D, N, X, E at x (the stability boundary N, X)"""
    q = 1 - (x / XM) ** 2
    zn = 0.5 * (ZN + ZX) + 0.5 * (ZN - ZX) * q ** 0.2
    zx = 0.5 * (ZN + ZX) - 0.5 * (ZN - ZX) * q ** 1.5
    return ZD, zn, zx, ZE + (zx - ZX)


def increment(z, x):
    """table 3, 3: the VRS model's increment of z + nu (of nu: z is the same) over the baseline"""
    if z >= 0 or x >= XM:
        return 0.0
    zd, zn, zx, ze = vrs_points(x)
    if not ze < z < zd:
        return 0.0
    fade = (1 - (x / XM) ** 6) ** 0.5
    dn = (VN - (ZN + upper(ZN, 0.0))) * fade
    dx = (VX - (ZX + upper(ZX, 0.0))) * fade
    sn = -(1 + baseline(zn, x)[1])
    sx = -(1 + baseline(zx, x)[1])
    if zn <= z:
        c = cubic([(0, zd, 0.0), (1, zd, 0.0), (0, zn, dn), (1, zn, sn)])
    elif zx <= z:
        c = cubic([(0, zn, dn), (1, zn, sn), (0, zx, dx), (1, zx, sx)])
    else:
        c = cubic([(0, zx, dx), (1, zx, sx), (0, ze, 0.0), ('a', 0, 0.0)])
    return poly(c, z)[0]


def nu(z, x, f=1.0, kappa=1.0):
    """Johnson's v / vh = kappa (v_base + f dv_vrs)"""
    return kappa * (baseline(z, x)[0] + f * increment(z, x))


def lower_edge(x):
    """the lowest z where the model departs from momentum theory at x (E or B), for x < XM"""
    ze = vrs_points(x)[3]
    return min(ze, base_points(x)[1]) if x < XC else ze


# ---------------------------------------------------------------- the tables genv22.py writes
# z from below B to D; across, s = (1 - (x/XM)^2)^0.2, the coordinate table 3 moves N in, which keeps the region's
# shape smooth up to M (in x the points close up as its fifth root): at the x of a 0.025 grid to 0.9 and every 0.05 of
# s below. The region where the model departs from momentum theory lies within; outside it (genv22 tests the edges) the
# tables are not read
TAB_Z = [round(-2.3 + 0.02 * i, 2) for i in range(106)]


def s_of(x):
    return max(0.0, 1 - (x / XM) ** 2) ** 0.2


def x_of(s):
    return XM * math.sqrt(max(0.0, 1 - s ** 5))


TAB_S = sorted({round(s_of(0.025 * i), 5) for i in range(37)} | {round(0.05 * i, 5) for i in range(13)})


def tables():
    """the baseline's nu and the VRS increment on TAB_Z x TAB_S"""
    nb = [[baseline(z, x_of(s))[0] for s in TAB_S] for z in TAB_Z]
    dv = [[increment(z, x_of(s)) for s in TAB_S] for z in TAB_Z]
    return nb, dv


def edge_table(n=96):
    """lower_edge against x, on a grid fine enough for linear interpolation"""
    xs = [XM * i / (n - 1) for i in range(n)]
    return xs, [lower_edge(x) for x in xs]


def table_error(dz=0.0073, dx=0.0091):
    """the largest error of the tables' bilinear interpolation in nu over the region, and where"""
    nb, dv = tables()
    err, at = 0.0, None
    for z in np.arange(TAB_Z[0] + 0.01, ZD, dz):
        for x in list(np.arange(0.0, 0.9, dx)) + list(np.linspace(0.9, XM - 1e-5, 121)):
            if lower_edge(x) < z:
                e = abs(bilinear(nb, z, x) + bilinear(dv, z, x) - nu(z, x))
                if e > err:
                    err, at = e, (z, x)
    return err, at


def bilinear(tab, z, x):
    s = s_of(x)
    i = min(max(np.searchsorted(TAB_Z, z) - 1, 0), len(TAB_Z) - 2)
    j = min(max(np.searchsorted(TAB_S, s) - 1, 0), len(TAB_S) - 2)
    tz = (z - TAB_Z[i]) / (TAB_Z[i + 1] - TAB_Z[i]); tx = (s - TAB_S[j]) / (TAB_S[j + 1] - TAB_S[j])
    return ((1 - tz) * (1 - tx) * tab[i][j] + tz * (1 - tx) * tab[i + 1][j] + (1 - tz) * tx * tab[i][j + 1]
            + tz * tx * tab[i + 1][j + 1])


# ---------------------------------------------------------------- checks
def check_points():
    """the end points figures 38 and 39 mark (Vz/vh, Vz/vh + v/vh) at x = 0, 0.4, 0.6, 0.8: the baseline's A and B,
    the VRS model's N, X and E"""
    rows = []
    for x in (0.0, 0.4, 0.6, 0.8):
        za, zb = base_points(x)
        zd, zn, zx, ze = vrs_points(x)
        rows.append((x, dict(A=(za, za + baseline(za, x)[0]), B=(zb, zb + baseline(zb, x)[0]), N=(zn, zn + nu(zn, x)),
                             X=(zx, zx + nu(zx, x)), E=(ze, ze + nu(ze, x)))))
    return rows


def boundary(x, h=1e-4):
    """the zero-slope points of z + nu at x, numerically (the stability boundary the model is built on)"""
    zs = np.arange(-2.2, -0.05, 0.005)
    s = [1 + (nu(z + h, x) - nu(z - h, x)) / (2 * h) for z in zs]
    out = []
    for i in range(len(zs) - 1):
        if s[i] * s[i + 1] < 0:
            out.append(zs[i] - s[i] * (zs[i + 1] - zs[i]) / (s[i + 1] - s[i]))
    return out


if __name__ == '__main__':
    roots = max(len(momentum(z, x)) for x in np.arange(XC, 1.5, 0.01) for z in np.arange(-2.5, 0, 0.01))
    print('momentum theory for x >= XC, z >= -2.5: at most %d root(s)' % roots)
    for x, pts in check_points():
        print('x %.1f  ' % x + '  '.join('%s (%.3f, %.3f)' % (k, a, b) for k, (a, b) in pts.items()))
    for x in (0.0, 0.3, 0.6, 0.9):
        print('x %.1f  zero slope of z + nu at' % x, ['%.3f' % z for z in boundary(x)], ' table: N, X', ['%.3f' % z for z in vrs_points(x)[1:3]])
    err, at = table_error()
    print('tables %d x %d: largest interpolation error of nu %.4f at z %.3f, x %.4f' % (len(TAB_Z), len(TAB_S), err, *at))
