#!/usr/bin/env python3
"""Level-flight trims against speed: controls, attitudes, main and tail rotor power and the engine's (the drive's load).

  python3 sweep.py [kg] [alt_ft]
"""
import sys, json
import numpy as np
import harness as H

SPEEDS = [0, 20, 40, 60, 80, 100, 113, 125]


def run(kg=2200.0, alt_ft=0.0, speeds=SPEEDS, verbose=True):
    f = H.load(model='al3-2200')
    H.set_weight(f, kg)
    out, x0 = [], None
    extra = ('aero/rotor/power-hp', 'aero/rotor/tr/power-hp', 'drive/load-ftlb', 'rotor/omega-rad_sec', 'aero/rotor/thrust-lbs',
             'aero/rotor/a1-deg', 'aero/rotor/b1-deg', 'aero/airframe/FX-lbs', 'aero/airframe/FZ-lbs')
    for V in speeds:
        x, r, e = H.trim(f, V, alt_ft=alt_ft, x0=x0, extra=extra)
        x0 = x
        d = H.deg(x)
        row = dict(V=V, x=x.tolist(), res=float(np.abs(r).max()), coll_deg=d[0], B1s=d[1], A1s=d[2], tr_deg=d[3], coll07=H.coll_07(x),
                   theta=float(f['attitude/theta-deg']), phi=float(f['attitude/phi-deg']), mr_hp=e[0], tr_hp=e[1],
                   engine_kw=e[2] * e[3] / 737.562, thrust=e[4], a1=e[5], b1=e[6], fx=e[7], fz=e[8])
        out.append(row)
        if verbose:
            print('%5.0f kt  coll %5.2f (0.7R %5.2f)  B1s %6.2f  A1s %6.2f  TR %6.2f  theta %6.2f  phi %6.2f  MR %5.0f hp  TR %4.0f hp  engine %5.1f kW  res %.3f'
                  % (V, d[0], row['coll07'], d[1], d[2], d[3], row['theta'], row['phi'], e[0], e[1], row['engine_kw'], row['res']))
    return out


if __name__ == '__main__':
    kg = float(sys.argv[1]) if len(sys.argv) > 1 else 2200.0
    alt = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    res = run(kg, alt)
    json.dump(res, open('out/sweep_%d_%d.json' % (kg, alt), 'w'), indent=1)
