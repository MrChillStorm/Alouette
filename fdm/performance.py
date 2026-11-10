#!/usr/bin/env python3
"""Hover performance against the type's hover ceilings [TD, HA, aerostories]: the engine power the hover needs (main
and tail rotors, the losses) against pressure altitude in the standard atmosphere, out of ground effect and in it (the
wheels HOVER_WHEELS_M above the ground), and where it meets the engine's available power (harness.p_avail_kw).

  python3 performance.py [kg ...]
"""
import sys, json, math
import numpy as np
import harness as H
import al3data as U

HOVER_WHEELS_M = 1.0        # in ground effect: the take-off's 1 m hover [LM]
ALTS_FT = [0, 4000, 8000, 12000, 16000, 20000]


def hover_power_kw(f, alt_ft, ige=False, x0=None):
    terr = None
    if ige:
        terr = alt_ft - H.wheels_above_ground_ft(f) - HOVER_WHEELS_M / 0.3048
    x, r, e = H.trim(f, 0.0, alt_ft=alt_ft, x0=x0, extra=H.ENGINE_POWER, terrain_ft=terr)
    return e[0] * e[1] / 737.562, x, r


def ceiling(f, kg, ige):
    H.set_weight(f, kg)
    pts, x0 = [], None
    for a in ALTS_FT:
        pk, x0, r = hover_power_kw(f, a, ige, x0)
        pts.append((a, pk, H.p_avail_kw(a), float(np.abs(r).max())))
    alts = np.array([p[0] for p in pts]); margin = np.array([p[2] - p[1] for p in pts])
    c = None
    for i in range(len(alts) - 1):
        if margin[i] >= 0 > margin[i + 1]:
            c = alts[i] + margin[i] / (margin[i] - margin[i + 1]) * (alts[i + 1] - alts[i])
    if c is None and margin[0] < 0:
        c = float('nan')
    return pts, c


if __name__ == '__main__':
    kgs = [float(a) for a in sys.argv[1:]] or [2200.0, 1750.0]
    f = H.load(model='al3-2200')
    out = {}
    for kg in kgs:
        for ige in (False, True):
            pts, c = ceiling(f, kg, ige)
            print('%.0f kg %s: ceiling %s m' % (kg, 'IGE' if ige else 'OGE', 'above %d' % (ALTS_FT[-1] * 0.3048) if c is None else '%.0f' % (c * 0.3048)))
            for a, pk, pa, res in pts:
                print('   %6.0f ft  required %6.1f kW  available %6.1f kW  (residual %.3f)' % (a, pk, pa, res))
            out['%d-%s' % (kg, 'ige' if ige else 'oge')] = dict(points=pts, ceiling_m=None if c is None else c * 0.3048)
    json.dump(out, open('out/performance.json', 'w'), indent=1)
