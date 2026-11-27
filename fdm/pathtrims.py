#!/usr/bin/env python3
"""Steady climbs and autorotations: trims along a sloping flight path, the rotor at 353 rpm.

  climb         at each airspeed, the flight path angle at which the engine's power (the drive's load: main and tail
                rotors, losses, accessories) is the available power (harness.p_avail_kw: the maximum continuous rating,
                405 kW [TE], or a lower reading of it), and the rate of climb along it [TD, VP: 4.3 m/s at 2200 kg]
  autorotation  at each airspeed, the flight path angle at which that power is nil: the steady autorotation's rate of
                descent with the rotor at its governed speed

  python3 pathtrims.py [climb|autorotation] [--kg=2200]     writes out/pathtrims_<stage>.json
"""
import sys, json, math
import numpy as np
import harness as H

KT_FPS = 1.68781
CLIMB_KT = (30, 40, 50, 60, 70, 80)
WALK_KT = 2.0               # a speed that doesn't trim from its neighbour's solution is walked up to in steps this fine
ALPHA_MAX = 60.0            # deg: slow and steep, the near-level airframe meets the air 25 to 50 deg off its nose
# from 40 kt: below about 33 kt the model has no steady autorotation (descent.py's descents): the path steepens into the
# vortex ring state, where the engine must still give power
AUTO_KT = (40, 50, 60, 70, 80, 90, 100)


def power_kw(f, V, gamma, alt_ft, x0):
    x, r, e = H.trim(f, V, alt_ft=alt_ft, gamma=gamma, x0=x0, extra=H.ENGINE_POWER, alpha_max=ALPHA_MAX)
    return e[0] * e[1] / 737.562, x, float(np.abs(r).max())


def solve(f, V, target_kw, alt_ft, g_lo, g_hi, x0=None, tol_kw=1.0, iters=16, res_max=0.05):
    """the flight path angle (deg) at which the engine's power is target_kw, the power rising with the angle: a bracket
    [g_lo, g_hi] widened until it holds the answer, then the Illinois method in it. A trim that does not converge
    (residual over res_max) counts as beyond the bracket's end it was tried from (the steep descents' windmill and
    vortex ring states, the steep climbs' stall), and the bracket shrinks toward the converged end."""
    def at(g, x):
        p, xn, r = power_kw(f, V, g, alt_ft, x)
        return p, xn, r
    p_lo, x_lo, r_lo = at(g_lo, x0)
    p_hi, x_hi, r_hi = at(g_hi, x_lo if r_lo < res_max else x0)
    for _ in range(6):                                   # widen until the target lies inside (both ends converged)
        if r_lo < res_max and r_hi < res_max and (p_lo - target_kw) * (p_hi - target_kw) <= 0:
            break
        w = g_hi - g_lo
        if r_lo >= res_max:
            g_lo += 0.4 * w; p_lo, x_lo, r_lo = at(g_lo, x_hi)
        elif r_hi >= res_max:
            g_hi -= 0.4 * w; p_hi, x_hi, r_hi = at(g_hi, x_lo)
        elif p_lo > target_kw:
            g_lo -= w; p_lo, x_lo, r_lo = at(g_lo, x_lo)
        else:
            g_hi += w; p_hi, x_hi, r_hi = at(g_hi, x_hi)
    if not (r_lo < res_max and r_hi < res_max and (p_lo - target_kw) * (p_hi - target_kw) <= 0):
        return float('nan'), float('nan'), x0 if x0 is not None else x_lo, 1e9
    side = 0
    g, p, x, r = g_lo, p_lo, x_lo, r_lo
    for _ in range(iters):
        g = g_hi - (p_hi - target_kw) * (g_hi - g_lo) / (p_hi - p_lo)
        p, x, r = at(g, x_lo)
        if r >= res_max:                                 # unconverged: bisect instead
            g = 0.5 * (g_lo + g_hi); p, x, r = at(g, x_lo)
            if r >= res_max:
                break
        if abs(p - target_kw) < tol_kw:
            break
        if (p - target_kw) * (p_hi - target_kw) < 0:
            g_lo, p_lo, x_lo = g_hi, p_hi, x_hi
            if side == -1: p_lo = target_kw + (p_lo - target_kw) / 2      # Illinois
            side = -1
        else:
            if side == 1: p_lo = target_kw + (p_lo - target_kw) / 2
            side = 1
        g_hi, p_hi, x_hi = g, p, x
    return g, p, x, r


def walk(f, v_from, v_to, target_kw, alt_ft, g, x):
    """the point at v_to reached from the converged one at v_from in steps of at most WALK_KT; None if a step fails"""
    n = max(1, int(math.ceil(abs(v_to - v_from) / WALK_KT)))
    for i in range(1, n + 1):
        g1, p, x1, r = solve(f, v_from + (v_to - v_from) * i / n, target_kw, alt_ft, g - 4.0, g + 4.0, x)
        if r >= 0.05:
            return None
        g, x = g1, x1
    return g, p, x, r


BY_POWER_KW = (250.0, 300.0, 330.0, 360.0, 390.0, 405.0, 420.0)     # 390: the hover charts'; 405, 420: the ratings


def by_power(kg=2200.0, V=55.0, powers=BY_POWER_KW, alt_ft=1000.0):
    """the rate of climb at one airspeed (about the least power's) against the power available"""
    out = []
    for pa in powers:
        pts = run('climb', kg, alt_ft, pa, False, (V,))['points']
        if pts:
            out.append(dict(p_avail=pa, **pts[0]))
    return out


def run(stage, kg=2200.0, alt_ft=1000.0, p_avail=None, verbose=True, speeds=None):
    def fresh():                        # a failed trim can leave the rotor's states wild: the next starts afresh
        f = H.load(model='al3-2200')
        H.set_weight(f, kg)
        return f
    f = fresh()
    pa = p_avail or H.p_avail_kw(alt_ft, rating='mcp')
    out = dict(kg=kg, alt=alt_ft, p_avail=pa if stage == 'climb' else 0.0, points=[], missing=[])
    x0, g_prev = None, None

    def point(V, g, p, x, r):
        rate = V * KT_FPS * math.sin(math.radians(g)) * 60
        out['points'].append(dict(V=V, gamma=g, kw=p, fpm=rate, x=[float(v) for v in x], res=r, coll07=H.coll_07(x)))
        if verbose:
            print('%s %3d kt: path %6.2f deg  %6.0f ft/min  power %6.1f kW  collective 0.7R %5.2f deg  residual %.3f'
                  % (stage, V, g, rate, p, H.coll_07(x), r))
    # climbs from the slowest speed up, autorotations from the fastest down, each from its neighbour's solution
    order = speeds or (CLIMB_KT if stage == 'climb' else AUTO_KT[::-1])
    for V in order:
        target = pa if stage == 'climb' else 0.0
        if g_prev is None or math.isnan(g_prev):
            g0, g1 = (0.0, 20.0) if stage == 'climb' else (-30.0, -8.0)
        else:
            g0, g1 = g_prev - 4.0, g_prev + 4.0
        g, p, x, r = solve(f, V, target, alt_ft, g0, g1, x0)
        if r >= 0.05:
            out['missing'].append(V)
            f = fresh()
            continue
        x0, g_prev = x, g
        point(V, g, p, x, r)
    # the speeds missed: each walked to from the nearest trimmed one, on either side, the nearest first
    tried = set()
    while out['points']:
        left = [v for v in out['missing'] if v not in tried]
        if not left:
            break
        gap = lambda v: min(abs(v - q['V']) for q in out['points'])
        V = min(left, key=gap)
        q = min(out['points'], key=lambda q: abs(q['V'] - V))
        tried.add(V)
        got = walk(fresh(), q['V'], V, target, alt_ft, q['gamma'], q['x'])
        if got:
            point(V, *got)
            out['missing'].remove(V)
    if verbose and out['missing']:
        print('%s: no converged trim at %s kt' % (stage, out['missing']))
    out['points'].sort(key=lambda q: q['V'])
    return out


if __name__ == '__main__':
    kg = next((float(a[5:]) for a in sys.argv[1:] if a.startswith('--kg=')), 2200.0)
    stages = [a for a in sys.argv[1:] if not a.startswith('--')] or ['climb', 'autorotation']
    for st in stages:
        json.dump(run(st, kg), open('out/pathtrims_%s.json' % st, 'w'), indent=1)
