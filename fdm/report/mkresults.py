#!/usr/bin/env python3
"""Run the test programme of the report into report/results/*.json (the figures, tables and numbers are made from
these by mktables.py and figures.py). Regenerate the harness's model first:

    cd fdm && python3 genal3.py root/aircraft/alouette3
    python3 report/mkresults.py [stage ...]      all stages in parallel (4 at a time), or the ones named

After the others, one at a time (each runs 4 workers of its own): hovercharts (the ground effect against the flight
manual's hover charts, with the fit at other wheel heights), hoverinflow (the hover correction's shape, strength and
checks), cruisecharts (the cruise fuel against its charts); each writes out/<stage>.json, copied to report/results/,
and fails if its own checks do.

Stages: trims_2200, trims_1750 (level flight, sea level: sweep.py), hover (hover power and ceilings in and out of ground
effect: performance.py), ge (the hover in ground effect flown by the test autopilot), mission (a test flight flown by
it: take-off, climb, cruise, practice autorotation, recovery, approach, landing: flights.py), climb, autorotation
(steady climbs at the maximum continuous power and autorotations: pathtrims.py), derivs (derivatives and modes:
derivatives.py), steps (hover step responses), engine (start, governor, power failure: engine_response.py), descent
(the vortex ring state: descent.py)."""
import os, sys, json, math, time, shutil, subprocess
from multiprocessing import Pool
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
FDM = os.path.dirname(HERE)
sys.path.insert(0, FDM)
os.chdir(FDM)
OUT = os.environ.get('AL3_RESULTS') or os.path.join(HERE, 'results')   # AL3_RESULTS: elsewhere (tests)
TRIM_KT = [0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 105, 110, 115, 120, 125]


def plain(x):
    if isinstance(x, dict):
        return {str(k): plain(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [plain(v) for v in x]
    if isinstance(x, np.ndarray):
        return plain(x.tolist())
    if isinstance(x, complex) or isinstance(x, np.complexfloating):
        return [float(x.real), float(x.imag)]
    if isinstance(x, (np.floating, np.integer)):
        return x.item()
    return x


def dump(name, obj):
    json.dump(plain(obj), open(os.path.join(OUT, name + '.json'), 'w'), indent=1)


def trims(kg):
    import sweep
    dump('trims_%d' % kg, sweep.run(float(kg), 0.0, TRIM_KT, verbose=False))


def hover_limit(kg=2200.0, start_ft=8000.0):
    """how high the hover out of ground effect trims, walked up from start_ft in steps halved down to 50 ft: at the last
    trimmed altitude and at sea level, the tail rotor's thrust against the pedals (the other controls held at the trim),
    its peak against what the trim needs, and each rotor's largest blade angle of attack over a revolution"""
    import re, performance as P, harness as H
    PEDALS = np.arange(0.40, 0.851, 0.05)          # of +-1, + right: the peak lies inside

    def fresh():                        # a failed trim can leave the rotor's states wild: the next starts afresh
        f = H.load(model='al3-2200')
        H.set_weight(f, kg)
        return f

    def trim(f, ft, x0):
        x, r, e = H.trim(f, 0.0, alt_ft=ft, x0=x0, extra=H.ENGINE_POWER + ('aero/rotor/tr/thrust-lbs',), tol=0.002, iters=30)
        return x, e[0] * e[1] / 737.562, float(np.abs(r).max()), float(e[2])

    def held(f, ft, x):
        """the airframe held at the controls x, the rotors settled: the tail and main rotors' mean thrusts (lb). A model's
        first hold after loading still has the controls' defaults: the second holds x"""
        f['ctl/init'] = 1
        H.freeze(f)
        for _ in range(2):
            e = H.residual(f, x, 0.0, ft, 0.0, extra=('aero/rotor/tr/thrust-lbs', 'aero/rotor/thrust-lbs'))[1]
        assert e[1] > 0.9 * f['inertia/weight-lbs'], 'the main rotor does not carry the weight: %.0f lb' % e[1]
        return float(e[0])

    def look(ft, x, trimmed_lb):
        curve = []
        for ped in PEDALS:
            xp = x.copy(); xp[3] = ped
            curve.append((float(ped), held(fresh(), ft, xp)))
        peak = max(curve, key=lambda c: c[1])
        assert curve[0][0] < peak[0] < curve[-1][0], 'the tail rotor thrust has no peak inside the pedals tried: %s' % curve
        f = fresh()
        need = held(f, ft, x)
        assert abs(need / trimmed_lb - 1) < 0.01, 'held at the trim, the tail thrust is not the trim\'s: %.1f and %.1f lb' % (need, trimmed_lb)
        cat = f.get_property_catalog()
        cat = [q.split(' ')[0] for q in (cat.split('\n') if isinstance(cat, str) else cat)]
        rotors = [[q for q in cat if re.fullmatch(pat, q)] for pat in (r'aero/rotor/tr/b\d/s\d/alpha-deg', r'aero/rotor/b\d/s\d/alpha-deg')]
        assert all(rotors), 'the blade segments\' angles of attack are not there'
        top = [0.0, 0.0]
        for _ in range(int(round(2 * math.pi / f['rotor/omega-rad_sec'] / f['simulation/dt']))):
            f.run()
            top = [max(t, max(abs(f[q]) for q in names)) for t, names in zip(top, rotors)]
        return dict(pedal=float(x[3]), need_lb=need, peak_lb=peak[1], peak_pedal=peak[0], curve=curve,
                    tr_alpha=top[0], mr_alpha=top[1])

    x = None
    for a in (0.0, start_ft / 2, start_ft):
        f = fresh()
        x, kw, res, tail = trim(f, a, x)
        if a == 0.0:
            x_sea, tail_sea = x, tail
    assert res < 0.05, 'no hover at the walk\'s start'
    ft, step = start_ft, 500.0
    while step >= 50:
        x1, kw1, res, tail1 = trim(f, ft + step, x)
        if res < 0.05:
            ft, x, kw, tail = ft + step, x1, kw1, tail1
        else:
            step /= 2
            f = fresh()
            trim(f, ft, x)
    return dict(kg=kg, alt_m=ft * 0.3048, kw=kw, avail_kw=H.p_avail_kw(ft), at_limit=look(ft, x, tail),
                sea=look(0.0, x_sea, tail_sea))


def hover():
    import performance as P, harness as H
    f = H.load(model='al3-2200')
    out = {}
    for kg in (2200.0, 2100.0, 1750.0):
        for ige in (False, True):
            pts, c = P.ceiling(f, kg, ige)
            out['%d-%s' % (kg, 'ige' if ige else 'oge')] = dict(points=pts, ceiling_m=None if c is None else c * 0.3048)
    # the power to hover at each published ceiling, trimmed there: the flight manual's (its table, at the masses the
    # engine's limit sets) and Aérostories'
    from data import al3_hover_charts as C, al3_published as PB
    pub = [('SPEC', kg, mode, C.TABLE[kg][0 if mode == 'ige' else 1]) for kg in (2200, 2100) for mode in ('oge', 'ige')]
    pub += [(src, kg, mode, m) for (src, kg, mode), m in PB.HOVER_CEILING_M.items()]
    out['implied'] = []
    for src, kg, mode, m in pub:
        f = H.load(model='al3-2200')
        H.set_weight(f, float(kg))
        kw, x, r = P.hover_power_kw(f, m / 0.3048, mode == 'ige')
        res = float(np.abs(r).max())
        if not res < 0.05:
            raise RuntimeError('no hover trim at the %s ceiling, %d kg %s, %.0f m (residual %.2f)' % (src, kg, mode, m, res))
        out['implied'].append(dict(src=src, kg=kg, mode=mode, ceiling_m=m, kw=kw))
    out['limit'] = hover_limit()
    dump('hover', out)


def ge():
    import flights
    r = flights.ge(2200.0, verbose=False)
    dump('ge', dict(points=r['points'], rec=r['rec'][::2]))


def mission():
    import flights
    dump('mission', flights.mission(2200.0, verbose=False))


def climb():
    import pathtrims
    out = dict(mcp=pathtrims.run('climb', 2200.0, verbose=False),
               mcp_1750=pathtrims.run('climb', 1750.0, verbose=False),
               by_power=pathtrims.by_power(2200.0))
    dump('climb', out)
    short = [(k, out[k]['missing']) for k in ('mcp', 'mcp_1750') if out[k]['missing']]
    lost = sorted(set(pathtrims.BY_POWER_KW) - {p['p_avail'] for p in out['by_power']})
    if short or lost:
        raise RuntimeError('climb points not trimmed: speeds %s, powers %s' % (short, lost))


def autorotation():
    import pathtrims
    out = pathtrims.run('autorotation', 2200.0, verbose=False)
    dump('autorotation', out)
    if out['missing']:
        raise RuntimeError('autorotation speeds not trimmed: %s' % out['missing'])


def derivs():
    import derivatives as DV
    dump('derivs', DV.run((0.0, 60.0, 100.0)))


def steps():
    """from the trimmed hover (2200 kg, 1000 ft, out of ground effect), 10 % of each control's travel, held: the free
    aircraft's rates, attitudes and vertical speed for 3 s (as the Bo 105 and UH-60 reports' step responses)"""
    import harness as H
    out = {}
    for axis in ('lat', 'lon', 'ped', 'coll'):
        f = H.load(model='al3-2200')
        H.set_weight(f, 2200.0)
        x, r, e = H.trim(f, 0.0, alt_ft=1000.0)
        f['drive/hold-rpm'] = 0
        H.freeze(f, False); f['ctl/init'] = 0
        c = dict(coll=x[0], lon=x[1], lat=x[2], ped=x[3])
        c[axis] += 0.1 if axis == 'coll' else (-0.2 if axis == 'lon' else 0.2)
        H.controls(f, **c)
        dt = f['simulation/dt']; rec = []
        th0, ph0 = f['attitude/theta-deg'], f['attitude/phi-deg']
        for i in range(int(3.0 / dt) + 1):
            if i % 24 == 0:
                rec.append(dict(t=i * dt, p=math.degrees(f['velocities/p-rad_sec']), q=math.degrees(f['velocities/q-rad_sec']),
                                r=math.degrees(f['velocities/r-rad_sec']), phi=f['attitude/phi-deg'] - ph0, theta=f['attitude/theta-deg'] - th0,
                                vs=f['velocities/h-dot-fps'] * 60, nr=f['rotor/omega-rad_sec'] * 30 / math.pi))
            f.run()
        out[axis] = rec
    dump('steps', out)


def engine():
    import engine_response as ER
    dump('engine', {k: ER.STAGES[k]() for k in ('start', 'governor', 'over_limit', 'failure_held')})


def descent():
    import descent as DS
    dump('descent', DS.sweep())


STAGES = dict(trims_2200=lambda: trims(2200), trims_1750=lambda: trims(1750), hover=hover, ge=ge, mission=mission, climb=climb,
              autorotation=autorotation, derivs=derivs, steps=steps, engine=engine, descent=descent)
CHARTS = dict(hovercharts=['--fit'], hoverinflow=[], cruisecharts=[])      # scripts of their own, with their arguments
SLOW_FIRST = ['descent', 'hover', 'trims_2200', 'trims_1750', 'climb', 'autorotation', 'derivs', 'mission', 'ge', 'engine', 'steps']


def run(name):
    t0 = time.time()
    try:
        STAGES[name]()
        return name, time.time() - t0, None
    except Exception:                       # one stage's failure leaves the others' results
        import traceback
        return name, time.time() - t0, traceback.format_exc()


def chart(name):
    t0 = time.time()
    p = subprocess.run([sys.executable, name + '.py'] + CHARTS[name], cwd=FDM, capture_output=True, text=True)
    err = None if p.returncode == 0 else (p.stdout + p.stderr)[-3000:]
    if not err:
        shutil.copyfile(os.path.join(FDM, 'out', name + '.json'), os.path.join(OUT, name + '.json'))
    return name, time.time() - t0, err


def done(name, dt, err):
    print('%-13s %6.0f s %s' % (name, dt, 'FAILED\n' + err if err else 'ok'), flush=True)
    return bool(err)


if __name__ == '__main__':
    os.makedirs(OUT, exist_ok=True)
    names = sys.argv[1:] or SLOW_FIRST + list(CHARTS)
    stages = [n for n in names if n not in CHARTS]
    failed = 0
    if stages:
        with Pool(min(len(stages), 4)) as pool:
            for r in pool.imap_unordered(run, stages):
                failed += done(*r)
    for name in (n for n in names if n in CHARTS):
        failed += done(*chart(name))
    sys.exit(1 if failed else 0)
