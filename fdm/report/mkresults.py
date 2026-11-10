#!/usr/bin/env python3
"""Run the test programme of the report into report/results/*.json (the figures, tables and numbers are made from
these by mktables.py and figures.py). Regenerate the harness's model first:

    cd fdm && python3 genal3.py root/aircraft/alouette3
    python3 report/mkresults.py [stage ...]      all stages in parallel (4 at a time), or the ones named

Stages: trims_2200, trims_1750 (level flight, sea level: sweep.py), hover (hover power and ceilings in and out of ground
effect: performance.py), ge (the hover in ground effect flown by the test autopilot), mission (a test flight flown by
it: take-off, climb, cruise, practice autorotation, recovery, approach, landing: flights.py), climb, autorotation
(steady climbs at the maximum continuous power and autorotations: pathtrims.py), derivs (derivatives and modes:
derivatives.py), steps (hover step responses), engine (start, governor, power failure: engine_response.py), descent
(the vortex ring state: descent.py)."""
import os, sys, json, math, time
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


def hover():
    import performance as P, harness as H
    f = H.load(model='al3-2200')
    out = {}
    for kg in (2200.0, 2100.0, 1750.0):
        for ige in (False, True):
            pts, c = P.ceiling(f, kg, ige)
            out['%d-%s' % (kg, 'ige' if ige else 'oge')] = dict(points=pts, ceiling_m=None if c is None else c * 0.3048)
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
    dump('climb', dict(mcp=pathtrims.run('climb', 2200.0, verbose=False),
                       mcp_1750=pathtrims.run('climb', 1750.0, verbose=False),
                       by_power=pathtrims.by_power(2200.0), by_power_1750=pathtrims.by_power(1750.0)))


def autorotation():
    import pathtrims
    dump('autorotation', pathtrims.run('autorotation', 2200.0, verbose=False))


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
    dump('engine', {k: ER.STAGES[k]() for k in ('start', 'governor', 'failure_held')})


def descent():
    import descent as DS
    dump('descent', DS.sweep())


STAGES = dict(trims_2200=lambda: trims(2200), trims_1750=lambda: trims(1750), hover=hover, ge=ge, mission=mission, climb=climb,
              autorotation=autorotation, derivs=derivs, steps=steps, engine=engine, descent=descent)
SLOW_FIRST = ['descent', 'hover', 'trims_2200', 'trims_1750', 'climb', 'autorotation', 'derivs', 'mission', 'ge', 'engine', 'steps']


def run(name):
    t0 = time.time()
    try:
        STAGES[name]()
        return name, time.time() - t0, None
    except Exception:                       # one stage's failure leaves the others' results
        import traceback
        return name, time.time() - t0, traceback.format_exc()


if __name__ == '__main__':
    os.makedirs(OUT, exist_ok=True)
    names = sys.argv[1:] or SLOW_FIRST
    with Pool(min(len(names), 4)) as pool:
        for name, dt, err in pool.imap_unordered(run, names):
            print('%-13s %6.0f s %s' % (name, dt, 'FAILED\n' + err if err else 'ok'), flush=True)
