#!/usr/bin/env python3
"""The main rotor wake's induced velocity at the tail rotor, the horizontal stabiliser, its end plates and the fuselage,
from a skewed cylinder of ring vortices (uniform loading, Heyson & Katzoff NACA TR 1319), as the UH-60 model's
mrwake.py: rings parallel to the tip-path plane, leaving the hub along the skew line at chi from the plane's normal
towards where the air carries them (against the aircraft's in-plane motion). Averaged over each target and
normalised by the velocity at the rotor's centre (the model's mean inflow).

Geometry [al3data, from the 3-D model]: hub FS 118.1, WL 67.2, shaft 2.5 deg forward; tail rotor hub FS 365.5, its
mirror image's BL -12.1, WL 26.1, radius 3.14 ft, disc in the body's x-z plane; stabiliser 2.38 m x 0.34 m at FS 314;
end plates at its tips, 0.5 m high, 0.36 m chord; the cabin pod with its engine deck (body x > -2.3 m) and the tail
boom (-6.3 to -2.3 m): the 3-D model's planform at its top surface. Tip-path plane: shaft tilt less the longitudinal
flapping a1 (+ = disc aft); lateral flapping and the ground's image are left out. The tables are the aircraft's mirror
image's (genal3.py), as the flight model uses them.

  python3 mrwake.py --write [tr|stab|ep|fus|boom]   recompute data/mrwake_<target>.json
"""
import math, os, json, sys
import numpy as np
from scipy.special import ellipk, ellipe
from scipy.integrate import quad_vec
import al3data as U

CHI = [0, 10, 20, 30, 40, 50, 60, 70, 80, 85, 90]      # deg, wake skew from the plane's normal
PSI = list(range(0, 361, 30))                          # deg, direction of the aircraft's in-plane motion: 0 forward, 90 right
A1 = [-6.0, 0.0, 6.0]                                  # deg, longitudinal flapping, + = disc aft
HERE = os.path.dirname(os.path.abspath(__file__))


def ring(r, z, R=1.0):
    """axial and radial velocity of a unit-circulation ring (radius R) at radial distance r, axial distance z"""
    a = (R + r) ** 2 + z * z
    m = min(4 * R * r / a, 1 - 1e-12)
    K, E = ellipk(m), ellipe(m)
    d = (R - r) ** 2 + z * z
    uz = (K + (R * R - r * r - z * z) / d * E) / (2 * math.pi * math.sqrt(a))
    ur = z / (2 * math.pi * r * math.sqrt(a)) * (-K + (R * R + r * r + z * z) / d * E) if r > 1e-9 else 0.0
    return uz, ur


# wake contraction: in the hover the tip vortices contract to 0.78 R within a quarter radius or so below the disc
# [Landgrebe, J. AHS 17(4), 1972: 0.78 + 0.22 exp(-Lambda s)]; edgewise the wake is a flattened sheet that does not
# contract: the contraction fades with cos(chi)
CONTRACTION, LAMBDA_C = 0.22, 5.0


def wake_velocity(P, n, s_hat, chi):
    """induced air velocity at P (body axes, rotor radii from the hub) of a semi-infinite tube of unit rings parallel
    to the disc, axis n into the wake, skewed by chi towards s_hat, contracting with the distance s along it"""
    w = math.cos(chi) * n + math.sin(chi) * s_hat
    kc = CONTRACTION * math.cos(chi)

    def vel(s):
        d = P - s * w
        z = float(d @ n)
        rv = d - z * n
        r = float(np.linalg.norm(rv))
        Rs = 1.0 - kc * (1.0 - math.exp(-LAMBDA_C * s))
        # a ring of radius Rs carrying the same circulation: its velocities scale as 1/Rs at scaled distances
        uz, ur = ring(r / Rs, z / Rs)
        return (uz * n + (ur * rv / r if r > 1e-9 else 0.0)) / Rs

    pts = [x for x in (float(P @ n) / math.cos(chi) if abs(math.cos(chi)) > 1e-6 else None, float(P @ w)) if x is not None and 0 < x < 60]
    return quad_vec(vel, 0, 60, limit=500, points=pts or None)[0] + quad_vec(vel, 60, np.inf, limit=200)[0]


def mr_frame(a1_deg):
    """wake axis (into the wake, down), in-plane forward and right unit vectors of the tip-path plane (body axes)"""
    tilt = U.SHAFT_TILT - math.radians(a1_deg)
    up = np.array([math.sin(tilt), 0.0, -math.cos(tilt)])
    a = np.array([math.cos(tilt), 0.0, math.sin(tilt)])
    return -up, a, np.array([0.0, 1.0, 0.0])


def body(fs, bl, wl):
    """station (in) -> body axes from the hub, rotor radii"""
    return np.array([-(fs - U.FS_MR), bl - U.BL_MR, -(wl - U.WL_MR)]) / 12.0 / U.R


def tr_points(nr=5, na=12):
    """the tail rotor's disc (the mirror image's: on the left of the boom), in the body's x-z plane"""
    out = []
    for i in range(nr):
        r0, r1 = U.R_TR * math.sqrt(i / nr), U.R_TR * math.sqrt((i + 1) / nr)
        rm = math.sqrt((r0 * r0 + r1 * r1) / 2) * 12
        for j in range(na):
            ph = 2 * math.pi * (j + 0.5) / na
            out.append((body(U.FS_TR + rm * math.cos(ph), -U.BL_TR, U.WL_TR + rm * math.sin(ph)), 1.0 / (nr * na)))
    return out


def stab_points(ns=12, nc=3):
    span = U.SPAN_H * U.IN_PER_M
    chord = U.S_H / 10.7639 / U.SPAN_H * U.IN_PER_M
    pts = []
    for i in range(ns):
        y = -span / 2 + (i + 0.5) * span / ns
        for j in range(nc):
            fs = U.FS_H - chord / 4 + (j + 0.5) * chord / nc
            pts.append((body(fs, y, U.WL_H), 1.0 / (ns * nc)))
    return pts


def ep_points(nh=4, nc=3):
    """the right end plate (the left one mirrors it): about 0.5 m high, 0.36 m chord [3D]"""
    h, chord = 0.5 * U.IN_PER_M, 0.36 * U.IN_PER_M
    pts = []
    for i in range(nh):
        wl = U.WL_EP - h / 2 + (i + 0.5) * h / nh
        for j in range(nc):
            fs = U.FS_EP - chord / 4 + (j + 0.5) * chord / nc
            pts.append((body(fs, U.BL_EP, wl), 1.0 / (nh * nc)))
    return pts


def _planform(xmin, xmax, cell=0.25):
    """the 3-D model's planform between body x xmin and xmax on a coarse grid at its top surface, equal weights"""
    import acread
    m = acread.load()
    skip = ('Queue-Disc_M', 'object 0', 'Rotor-Protec', 'HGouv')
    tris = np.concatenate([t for n, t in m.items() if n not in skip])
    cx = tris[:, :, 0].mean(1)
    tris = tris[(cx > xmin) & (cx < xmax)]
    A, grid, (lo, c) = acread.projected_area(tris, 2, 0.02)
    V = tris.reshape(-1, 3)
    pts = []
    for x in np.arange(lo[0], lo[0] + grid.shape[0] * c, cell):
        for y in np.arange(lo[1], lo[1] + grid.shape[1] * c, cell):
            i, j = int((x - lo[0]) / c), int((y - lo[1]) / c)
            if grid[i:i + int(cell / c), j:j + int(cell / c)].mean() > 0.5:
                near = V[(np.abs(V[:, 0] - x) < cell) & (np.abs(V[:, 1] - y) < cell)]
                z = near[:, 2].max() if len(near) else 0.5
                pts.append(body(*U.model_to_station(x, y, z)))
    return [(P, 1.0 / len(pts)) for P in pts]


def fus_points():
    """the cabin pod with its engine deck (body x > -2.3 m)"""
    return _planform(-2.3, 10.0)


def boom_points():
    """the tail boom (body x -6.3 to -2.3 m)"""
    return _planform(-6.3, -2.3, cell=0.15)


TARGETS = {'tr': tr_points, 'stab': stab_points, 'ep': ep_points, 'fus': fus_points, 'boom': boom_points}
_PTS = {}


def _case(args):
    c, p, a1, target = args
    if target not in _PTS:
        _PTS[target] = TARGETS[target]()
    chi = math.radians(min(c, 89.5))
    n, a, b = mr_frame(a1)
    s_hat = -(math.cos(math.radians(p)) * a + math.sin(math.radians(p)) * b)
    w0 = float(wake_velocity(np.zeros(3), n, s_hat, chi) @ n)
    k = sum(wake_velocity(P, n, s_hat, chi) * wt for P, wt in _PTS[target]) / w0
    return k.tolist()


def tables(target):
    """k[a1][chi][psi] = (kx, ky, kz): the induced air velocity, mean over the target, over the mean inflow, body axes"""
    from multiprocessing import Pool
    cases = [(c, p, a1, target) for a1 in A1 for c in CHI for p in PSI]
    with Pool(4) as pool:
        res = dict(zip(cases, pool.map(_case, cases)))
    return {'chi': CHI, 'psi': PSI, 'a1': A1, 'k': [[[res[(c, p, a1, target)] for p in PSI] for c in CHI] for a1 in A1]}


def path(target):
    return os.path.join(HERE, 'data', 'mrwake_%s.json' % target)


def load(target):
    return json.load(open(path(target)))


if __name__ == '__main__':
    target = next((a for a in sys.argv[1:] if a in TARGETS), 'tr')
    if '--write' in sys.argv:
        json.dump(tables(target), open(path(target), 'w'), indent=0)
        print('wrote', path(target))
    d = load(target)
    comp = 1 if target in ('tr', 'ep') else 2
    for ia, a1 in enumerate(d['a1']):
        print('a1 %+.0f: k%s over the %s; rows chi, columns psi (aircraft motion: 0 fwd, 90 right)' % (a1, 'xyz'[comp], target))
        print('      ' + ' '.join('%6d' % p for p in d['psi']))
        for ic, c in enumerate(d['chi']):
            print('%5d ' % c + ' '.join('%6.2f' % d['k'][ia][ic][ip][comp] for ip in range(len(d['psi']))))
