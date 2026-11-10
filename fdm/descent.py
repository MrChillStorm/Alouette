#!/usr/bin/env python3
"""Steady descents through the vortex ring state (vrs.py, genal3.Model.vrs), offline, as the MV-22B model's descent.py.

Trims at 1000 ft out of ground effect, 2200 kg, still air, heading into the motion, at each forward speed from level
flight down in steps of 0.05 vh until the trim fails: the rotor's axial and edgewise velocity over its own vh (the
wake's) and its uniform inflow nu = v0 / vh against Johnson's model at the same point, and the collective, whose
extremes against the descent rate are the heave mode's stability boundary at a fixed collective (where
d(Vz + v)/dVz = 0).

  python3 descent.py              the descents' table (out/descent.json)
"""
import sys, json, math
import numpy as np
import harness as H
import vrs

KT = 1.6878
SPEEDS_KT = (0, 10, 20, 30, 40)
KEYS = ('aero/rotor/vrs/z', 'aero/rotor/vrs/x', 'aero/rotor/vrs/vh-fps', 'aero/rotor/vrs/in', 'aero/rotor/vrs/nu-base',
        'aero/rotor/vrs/dnu', 'rotor/v0-fps', 'aero/rotor/thrust-lbs', 'drive/load-ftlb', 'rotor/omega-rad_sec')


def point(e):
    d = dict(zip(KEYS, e))
    z, x, vh = d['aero/rotor/vrs/z'], d['aero/rotor/vrs/x'], d['aero/rotor/vrs/vh-fps']
    return dict(z=z, x=x, vh=vh, nu=d['rotor/v0-fps'] / vh, nu_j=vrs.nu(z, x), nu_mom=vrs.upper(z, x), inside=d['aero/rotor/vrs/in'],
                thrust=d['aero/rotor/thrust-lbs'], power_kw=d['drive/load-ftlb'] * d['rotor/omega-rad_sec'] / 737.562)


def boundary(rows):
    """the descent rates (over vh) where the collective against the descent rate turns: the stability boundary"""
    c = [r['coll'] for r in rows]; z = [r['z'] for r in rows]
    out = []
    for i in range(1, len(c) - 1):
        if (c[i] - c[i - 1]) * (c[i + 1] - c[i]) < 0:
            out.append(z[i])
    return out


def sweep(speeds=SPEEDS_KT, alt_ft=1000.0, kg=2200.0):
    f = H.load(model='al3-2200')
    H.set_weight(f, kg)
    x, r, e = H.trim(f, 0.0, alt_ft=alt_ft, extra=KEYS)
    vh0 = point(e)['vh']
    step = 0.05 * vh0
    out = dict(weight=f['inertia/weight-lbs'], alt=alt_ft, vh_hover=vh0, speeds=[])
    xl = x
    for V in speeds:
        rows, x0, fails = [], xl, 0
        for i in range(60):
            sink = i * step
            x, r, e = H.trim(f, 0.0, alt_ft=alt_ft, x0=x0, extra=KEYS, ned=(V * KT, sink), iters=40)
            ok = float(np.abs(r * np.array([1, 1, 1, 10, 10, 10.0])).max()) < 0.02
            if i == 0:
                xl = x
            if not ok:
                fails += 1
                if fails >= 2:
                    break
                continue
            x0 = x
            rows.append(dict(sink_fps=sink, coll=float(x[0]), coll07=H.coll_07(x), **point(e)))
            p = rows[-1]
            print('%3d kt  sink %5.1f ft/s (%5.0f ft/min)  z %6.3f  x %5.3f  nu %5.3f  Johnson %5.3f  momentum %5.3f  coll 0.7R %5.2f deg  %s'
                  % (V, sink, sink * 60, p['z'], p['x'], p['nu'], p['nu_j'], p['nu_mom'], p['coll07'], 'VRS' if p['inside'] > 0.5 else ''))
        out['speeds'].append(dict(kt=V, rows=rows, boundary=boundary(rows)))
        print('  boundary z:', ['%.3f' % b for b in out['speeds'][-1]['boundary']])
    return out


if __name__ == '__main__':
    res = sweep()
    json.dump(res, open('out/descent.json', 'w'), indent=1)
