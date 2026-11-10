"""NACA 0012 section of the Bo 105's tail rotor, for the blade-element tail rotor: C-81's rotor airfoil equations as the
TR-75-50 deck sets them for the tail rotor [TR cards 32A-32E; TR section 3.2.1: "Equations for a 0012 airfoil were
generally used for the tail rotor"], tabulated over +-180 deg and Mach number.

  lift   slope a(M) = 0.11 + 0.02468 M - 0.1956 M^2 + 0.3779 M^3 per deg (card 32B) up to the maximum lift
         CLmax(M) = 1.334 + 0.8334 M - 4.924 M^2 + 3.853 M^3 (card 32A, normal flow), both held from M 0.85 on; past it
         the lift falls over STALL_SPAN deg to a flat plate's (normal force 1.98 sin alpha [Hoerner, Fluid-Dynamic
         Lift]); in reversed flow the same about 180 deg with the reversed flow's maximum, 0.78 (card 32A)
  drag   CD = 0.008 - 0.00099 |alpha| + 0.000278 alpha^2 (alpha in deg, card 32B; the card prints the last coefficient
         as 0.00278, which would make the section's drag 0.07 at 5 deg, seven times a 0012's), with Lock's rise
         20 (M - Mdd + 0.108)^4 above the drag divergence Mach number Mdd = 0.79 (card 32A) less 0.028 per deg of angle
         of attack (card 32C's drag rise coefficient, taken as that slope); past the stall a flat plate's 1.98 sin^2
         alpha; in reversed flow 1.5 times the attached drag of the angle from the trailing edge
"""
import math

MACH = [0.0, 0.3, 0.4, 0.5, 0.6, 0.7, 0.75, 0.8, 0.85, 0.9, 1.0]
STALL_SPAN = 6.0            # deg from the maximum lift to the flat plate's
CN_PLATE = 1.98


def slope(M):
    M = min(M, 0.85)
    return 0.11 + 0.02468 * M - 0.1956 * M ** 2 + 0.3779 * M ** 3


def cl_max(M, reversed_flow=False):
    if reversed_flow:
        return 0.78
    M = min(M, 0.85)
    return 1.334 + 0.8334 * M - 4.924 * M ** 2 + 3.853 * M ** 3


def _attached_cd(a, M):
    """drag of the attached section at a (deg, >= 0), with the drag rise"""
    cd = 0.008 - 0.00099 * a + 0.000278 * a * a
    mdd = max(0.5, 0.79 - 0.028 * a)
    if M > mdd - 0.108:
        cd += 20 * (M - mdd + 0.108) ** 4
    return cd


def _plate(a):
    r = math.radians(a)
    cn = CN_PLATE * math.sin(r)
    return cn * math.cos(r), cn * math.sin(r)


def coefficients(alpha, M):
    """(cl, cd) at alpha (deg, -180..180) and Mach number M"""
    sgn = -1.0 if alpha < 0 else 1.0
    a = abs(alpha)
    k = slope(M)
    lp, dp = _plate(a)
    rev = a > 90
    b = 180.0 - a if rev else a                     # the angle from the edge the flow meets
    cm_ = cl_max(M, rev)
    bs = cm_ / k                                     # stall angle
    if b <= bs:
        cl = k * b
        cd = _attached_cd(b, M) * (1.5 if rev else 1.0)
    elif b <= bs + STALL_SPAN:
        w = (b - bs) / STALL_SPAN
        lp_e, dp_e = _plate(bs + STALL_SPAN)
        cl = (1 - w) * cm_ + w * lp_e
        cd0 = _attached_cd(bs, M) * (1.5 if rev else 1.0)
        cd = (1 - w) * cd0 + w * max(dp_e, cd0)
    else:
        cl, cd = abs(lp), max(dp, _attached_cd(min(b, bs), M))
    if rev:
        cl = -cl                                     # reversed flow: the lift opposes the attached flow's
    return sgn * cl, min(2.0, cd)


def alphas():
    a = list(range(0, 26)) + [27.5, 30, 32.5, 35, 37.5, 40] + list(range(45, 165, 5)) + list(range(165, 181))
    return sorted(set([-x for x in a] + a))


def tables():
    """(alphas, Mach numbers, cl rows, cd rows): rows over alpha, columns over Mach"""
    al = alphas()
    cl = [[coefficients(x, m)[0] for m in MACH] for x in al]
    cd = [[coefficients(x, m)[1] for m in MACH] for x in al]
    return al, MACH, cl, cd
