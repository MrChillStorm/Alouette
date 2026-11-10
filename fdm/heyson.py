"""Heyson NASA TN D-234: ground-induced interference velocity at the rotor centre. Wake = skewed cylinder of
horizontal ring vortices (rotor at zero tilt, uniform loading) from the rotor down to the ground at skew chi from
the vertical, plus its mirror image below the ground (reversed circulation). Normalised by the no-ground velocity
at the centre (semi-infinite wake), so dw/w0 -> -1 on the ground and -> 0 far above it."""
import numpy as np
from scipy.special import ellipk, ellipe
from scipy.integrate import quad

def uz(r, z, R=1.0):
    """axial velocity of a unit-circulation ring (radius R) at radial distance r from its axis, height z"""
    a = (R + r) ** 2 + z * z
    m = 4 * R * r / a
    if m >= 1: m = 1 - 1e-12
    return 1 / (2 * np.pi) / np.sqrt(a) * (ellipk(m) + (R * R - r * r - z * z) / ((R - r) ** 2 + z * z) * ellipe(m))

def w_line(chi, s0, s1, zoff=0.0, sgn=1):
    c, s = np.cos(chi), np.sin(chi)
    f = lambda t: uz(t * s, zoff - sgn * t * c) if zoff else uz(t * s, t * c)
    return quad(f, s0, s1, limit=400, points=[1.0, 2.0])[0]

def dw(ZR, chi_deg):
    chi = np.radians(chi_deg)
    c, s = np.cos(chi), np.sin(chi)
    w0 = quad(lambda t: uz(t * s, t * c), 0, np.inf, limit=400)[0]
    L = ZR / c if c > 1e-9 else np.inf
    real = quad(lambda t: uz(t * s, t * c), 0, L, limit=400)[0]
    image = quad(lambda t: uz(t * s, 2 * ZR - t * c), 0, L, limit=400)[0] if np.isfinite(L) else \
            quad(lambda t: uz(t * s, 2 * ZR), 0, np.inf, limit=400)[0]
    return (real - image - w0) / w0, w0

if __name__ == '__main__':
    chis = [0, 14.04, 26.56, 45, 63.43, 75.97, 84.29, 89.9]
    zs = [0.1, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0, 4.0]
    print('Z/R  ' + ' '.join('%7.2f' % c for c in chis))
    for z in zs:
        print('%4.2f ' % z + ' '.join('%7.3f' % dw(z, c)[0] for c in chis))
    print('w0 (x gamma):', ' '.join('%.3f' % dw(1.0, c)[1] for c in chis))
