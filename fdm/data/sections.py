"""The rotors' blade sections, as C-81 tables over +-180 deg and Mach number: the NACA 0012's C-81 equations
(data/c81_0012.py, the deck the UH-60 and Bo 105 models fly) with the differences that like-for-like wind tunnel
tests give between the 0012 and the real sections. The main blades are NACA 63A of 11.5 % thickness [HL], taken as
the symmetrical 63A011.5 (HAL's designation names no camber); the tail rotor's blades are NACA 0015 to 0018 [HL],
taken as 0018 at the root to 0015 at the tip.

Sources (the Langley two-dimensional low-turbulence pressure tunnel unless noted; R the Reynolds number):
  [N903]   L. K. Loftin, NACA Report 903 (1948), fig. 4: the NACA 63A010: maximum lift 1.19, 1.21 and 1.22 smooth at
           R 3, 6 and 9 million (hardly any scale effect), 0.82 with standard leading-edge roughness at 6 million;
           drag at 6 million 0.0046 smooth and 0.0095 rough at zero lift, 0.0078 and 0.0158 at a lift of 0.6;
           lift-curve slope 0.107 per deg; 6A and 6-series sections of one thickness have the same maximum lift and
           minimum drag (its conclusion 1)
  [N824]   I. H. Abbott, A. E. von Doenhoff, L. S. Stivers, NACA Report 824 (1945): the NACA 63(1)-012: maximum lift
           1.44, 1.47 and 1.51 smooth at 3, 6 and 9 million, 1.05 rough at 6; drag at 6 million 0.0045 smooth and
           0.0093 rough at zero lift, 0.0075 and 0.013 at 0.6; and the predicted critical Mach numbers at zero lift:
           NACA 0012 0.725, 63-009 0.79, 63(1)-012 0.745, 63(2)-015 0.705, 63(3)-018 0.67, the curves meeting near a
           lift of 0.4
  [TM4074] C. L. Ladson, NASA TM 4074 (1988), tables I and VII: the NACA 0012 at Mach 0.15: maximum lift 1.408 and
           1.484 at 2.0 and 3.94 million with free transition (1.66 at 6), 1.165 and 1.207 with wraparound No. 60
           grit (the standard roughness's grain; 1.217 at 6); drag at 6 million 0.0065 and 0.0094 at zero lift,
           0.0070 and 0.0109 at a lift of 0.6; slope 0.107 per deg
  [N460]   E. N. Jacobs, K. E. Ward, R. M. Pinkerton, NACA Report 460 (1933), tables II, IV and VII, the variable-
           density tunnel at about 3 million: the 0012, 0015 and 0018: slopes 0.101, 0.100 and 0.098 per deg, maximum
           lift 1.53, 1.53 and 1.49, minimum profile drag 0.0083, 0.0093 and 0.0108

How the differences are applied:
  - the C-81 0012 lies between the smooth and the rough 0012: its maximum lift at Mach 0.15, 1.36, is 0.67 of the
    way from the rough section's (1.19) to the smooth one's (1.45) at 3 million, the Reynolds number of the
    retreating blade's stall; its zero-lift drag with the production blade's increment, 0.0092, is 0.07 of the way
    from the rough's (0.0094) to the smooth's (0.0065) at 6 million, near the advancing and hovering blade's. The
    63A011.5 (interpolated in thickness between the 63A010 [N903] and the 63(1)-012 [N824]) is placed the same way
    between its own smooth and rough values, the rough maximum lift taken to 3 million with the 0012's scale effect;
  - maximum lift: the ratio to the 0012's so found (0.91) up to Mach 0.3, going to 1 at Mach 0.6, where both
    sections' stall is the shock's and their critical Mach numbers at high lift are about the same [N824] (an
    estimate: no high-speed maximum lift of a 63A section was found); reversed flow as the 0012;
  - drag: the 63A's increment over the 0012 at a lift of 0.6 so found (0.0026) grows with the angle of attack
    squared; the 0015 and 0018 scale the 0012's attached drag by their minimum drags' ratios (1.12 and 1.30 [N460]);
  - lift-curve slope: the 63A's is the 0012's (0.107 per deg both, smooth); the 0015 and 0018 take their ratios
    (0.99 and 0.97 [N460]);
  - drag divergence: the 0012's Mach number moved by the difference of the critical Mach numbers at zero lift
    (63A011.5 +0.028, 0015 -0.040, 0018 -0.075, the thicker 00 sections taken as the 63-series' trend [N824]),
    the difference fading to nothing at 3.7 deg (a lift of 0.4), where the curves meet.
"""
import os, sys
if __name__ == '__main__':
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from data import c81_0012 as B

MACH = B.MACH
ALPHA_MEET = 3.7                # deg: the critical Mach numbers' curves meet near a lift of 0.4 [N824]

# the 0012 [TM4074]: maximum lift smooth and rough at 3 million, interpolated between 2.0 and 3.94 million
CLMAX_0012_SMOOTH = 1.408 + (1.484 - 1.408) * (3.0 - 2.0) / (3.94 - 2.0)
CLMAX_0012_ROUGH = 1.165 + (1.207 - 1.165) * (3.0 - 2.0) / (4.0 - 2.0)
ROUGH_SCALE = CLMAX_0012_ROUGH / 1.217          # the rough 0012's maximum lift at 3 million over its 6 million's
CD0_0012_SMOOTH, CD0_0012_ROUGH, CD06_0012_SMOOTH, CD06_0012_ROUGH = 0.0065, 0.0094, 0.0070, 0.0109   # 6 million
# the 63A010 [N903] and 63(1)-012 [N824]: maximum lift smooth at 3 million and rough at 6; drag at a lift of 0.6
CLMAX_SMOOTH = {10: 1.19, 12: 1.44}
CLMAX_ROUGH = {10: 0.82, 12: 1.05}
CD06_SMOOTH = {10: 0.0078, 12: 0.0075}
CD06_ROUGH = {10: 0.0158, 12: 0.013}
# where the C-81 0012 lies between the rough (0) and smooth (1) 0012: by maximum lift at Mach 0.15, and by drag
W_LIFT = (B.cl_max(0.15) - CLMAX_0012_ROUGH) / (CLMAX_0012_SMOOTH - CLMAX_0012_ROUGH)
W_DRAG = (0.008 + 0.0012 - CD0_0012_ROUGH) / (CD0_0012_SMOOTH - CD0_0012_ROUGH)   # with the production increment
MCR_0012, MCR_63009, MCR_63012, MCR_63015, MCR_63018 = 0.725, 0.79, 0.745, 0.705, 0.67   # [N824]
# the 00 series by thickness [N460]: slope per deg, maximum lift, minimum profile drag
N460 = {12: (0.101, 1.53, 0.0083), 15: (0.100, 1.53, 0.0093), 18: (0.098, 1.49, 0.0108)}


def lerp(x, x0, x1, y0, y1):
    return y0 + (y1 - y0) * (x - x0) / (x1 - x0)


def naca_63a(t=11.5):
    """the symmetrical NACA 63A section of t % thickness, 10 to 12, placed between its smooth and rough values as the
    C-81 0012 lies between the 0012's"""
    f = lambda d: lerp(t, 10, 12, d[10], d[12])
    rough = f(CLMAX_ROUGH) * ROUGH_SCALE
    clmax = rough + W_LIFT * (f(CLMAX_SMOOTH) - rough)
    d06_rough, d06_smooth = f(CD06_ROUGH) - CD06_0012_ROUGH, f(CD06_SMOOTH) - CD06_0012_SMOOTH
    d06 = d06_rough + W_DRAG * (d06_smooth - d06_rough)
    mcr = lerp(t, 9, 12, MCR_63009, MCR_63012)
    a06 = 0.6 / 0.107                                    # deg at a lift of 0.6
    return dict(name='NACA 63A0%g' % t, clmax_ratio=clmax / B.cl_max(0.15), slope_ratio=1.0, cd_scale=1.0,
                cd_k=d06 / a06 ** 2, dmdd=mcr - MCR_0012)


def naca_00(t):
    """the NACA 00 section of t % thickness, 12 to 18"""
    if t <= 15:
        s, c, d = [lerp(t, 12, 15, N460[12][i], N460[15][i]) for i in range(3)]
        mcr = lerp(t, 12, 15, MCR_63012, MCR_63015)
    else:
        s, c, d = [lerp(t, 15, 18, N460[15][i], N460[18][i]) for i in range(3)]
        mcr = lerp(t, 15, 18, MCR_63015, MCR_63018)
    return dict(name='NACA 00%02.0f' % t, clmax_ratio=c / N460[12][1], slope_ratio=s / N460[12][0],
                cd_scale=d / N460[12][2], cd_k=0.0, dmdd=mcr - MCR_63012)


MAIN = naca_63a(11.5)
NACA0012 = dict(name='NACA 0012', clmax_ratio=1.0, slope_ratio=1.0, cd_scale=1.0, cd_k=0.0, dmdd=0.0)   # c81_0012 itself


def tail(r_frac):
    """the tail rotor's section at r_frac of its blade's span from root (0) to tip (1): 0018 to 0015"""
    return naca_00(18.0 - 3.0 * max(0.0, min(1.0, r_frac)))


def _ratio(sec, M):
    """the maximum lift's ratio to the 0012's: held to Mach 0.3, to 1 at 0.6"""
    w = max(0.0, min(1.0, (M - 0.3) / 0.3))
    return sec['clmax_ratio'] + (1 - sec['clmax_ratio']) * w


def _attached_cd(sec, a, M):
    cd = (0.008 - 0.00099 * a + 0.000278 * a * a) * sec['cd_scale'] + sec['cd_k'] * a * a
    mdd = max(0.5, 0.79 - 0.028 * a + sec['dmdd'] * max(0.0, 1 - a / ALPHA_MEET))
    if M > mdd - 0.108:
        cd += 20 * (M - mdd + 0.108) ** 4
    return cd


def coefficients(sec, alpha, M):
    """(cl, cd) at alpha (deg, -180..180) and Mach number M: c81_0012.coefficients with the section's differences"""
    sgn = -1.0 if alpha < 0 else 1.0
    a = abs(alpha)
    k = B.slope(M) * sec['slope_ratio']
    lp, dp = B._plate(a)
    rev = a > 90
    b = 180.0 - a if rev else a
    cm_ = B.cl_max(M, rev) * (1.0 if rev else _ratio(sec, M))
    bs = cm_ / k
    att = (lambda x: 1.5 * B._attached_cd(x, M)) if rev else (lambda x: _attached_cd(sec, x, M))   # reversed: the 0012's
    if b <= bs:
        cl = k * b
        cd = att(b)
    elif b <= bs + B.STALL_SPAN:
        w = (b - bs) / B.STALL_SPAN
        lp_e, dp_e = B._plate(bs + B.STALL_SPAN)
        cl = (1 - w) * cm_ + w * lp_e
        cd0 = att(bs)
        cd = (1 - w) * cd0 + w * max(dp_e, cd0)
    else:
        cl, cd = abs(lp), max(dp, att(min(b, bs)))
    if rev:
        cl = -cl
    return sgn * cl, min(2.0, cd)


_cache = {}


def tables(sec):
    """(alphas, Mach numbers, cl rows, cd rows) for the section, as c81_0012.tables()"""
    key = tuple(sorted((k, v) for k, v in sec.items() if k != 'name'))
    if key not in _cache:
        al = B.alphas()
        cl = [[coefficients(sec, x, m)[0] for m in MACH] for x in al]
        cd = [[coefficients(sec, x, m)[1] for m in MACH] for x in al]
        _cache[key] = (al, MACH, cl, cd)
    return _cache[key]


if __name__ == '__main__':
    print('W_LIFT %.3f  W_DRAG %.3f' % (W_LIFT, W_DRAG))
    for sec in (MAIN, tail(0.0), tail(1.0)):
        print(sec['name'], {k: round(v, 4) for k, v in sec.items() if k != 'name'})
        for M in (0.2, 0.4, 0.6, 0.8):
            k = B.slope(M) * sec['slope_ratio']
            cm_ = B.cl_max(M) * _ratio(sec, M)
            print('   M %.1f  clmax %.3f (0012 %.3f)  cd at 0/4/8 deg %.4f %.4f %.4f (0012 %.4f %.4f %.4f)' % (
                M, cm_, B.cl_max(M), *[coefficients(sec, x, M)[1] for x in (0, 4, 8)],
                *[B.coefficients(x, M)[1] for x in (0, 4, 8)]))
