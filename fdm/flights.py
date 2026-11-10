#!/usr/bin/env python3
"""Test flights flown offline by the test autopilot (genap.py), as the UH-60A model's ground-effect stage:

  ge           on the wheels, the engine running: the autopilot lifts off and hovers at each wheel height of GE_HEIGHTS,
               the engine's power averaged over its last 8 s: the ground effect against height
  (more stages below)

  python3 flights.py [stage ...] [--kg=2200]      prints, and writes out/flights_<stage>.json
"""
import sys, json, math
import numpy as np
import harness as H
import genap as AP

GE_HEIGHTS = ((3, 30), (5, 25), (10, 25), (20, 25), (40, 30), (100, 45))
KW = 1 / 737.562


def on_wheels(kg=None, model='al3-2200', heading=0.0):
    f = H.load(model=model, hold_rpm=False)
    if kg:
        H.set_weight(f, kg)
    f['eng/init-running'] = 1
    f['ic/terrain-elevation-ft'] = 0.0; f['ic/h-sl-ft'] = H.wheels_above_ground_ft(f) + 0.05
    f['ic/vt-kts'] = 0; f['ic/psi-true-deg'] = heading
    f.run_ic()
    H.controls(f, 0.0, 0, 0, 0)
    f['fcs/brake-parking-cmd-norm'] = 1.0; f['fcs/left-brake-cmd-norm'] = 1.0; f['fcs/right-brake-cmd-norm'] = 1.0
    s = AP.Sim(f, max(1, int(round(1 / (40 * f['simulation/dt'])))))      # the autopilot at 40 Hz
    return f, s


def state(f, s):
    return dict(t=f['simulation/sim-time-sec'], h=s['internal/h-wheel-ft'], u=s['internal/u-fps'], v=s['internal/v-fps'],
                theta=f['attitude/theta-deg'], phi=f['attitude/phi-deg'], psi=f['attitude/psi-deg'], vs=f['velocities/h-dot-fps'],
                kt=f['velocities/vc-kts'], nr=f['rotor/omega-rad_sec'] * AP.RPM, kw=f['drive/load-ftlb'] * f['rotor/omega-rad_sec'] * KW,
                coll=f[AP.CTL['thr']], lon=f['fcs/elevator-cmd-norm'], lat=f['fcs/aileron-cmd-norm'], ped=-f['fcs/rudder-cmd-norm'],
                wow=f['gear/unit[0]/WOW'] + f['gear/unit[1]/WOW'] + f['gear/unit[2]/WOW'], t4=f['eng/t4-degc'])


def fly(f, s, sec, rec=None, every=0.25):
    """the autopilot flies; the wheel brakes on while it is on its wheels, as a pilot holds them for the lift-off"""
    dt = f['simulation/dt']
    k = int(round(every / dt))
    for i in range(int(round(sec / dt))):
        if s['engage'] > 0.5:       # (FlightGear makes the parking brake the wheel brakes' command; offline it is set here)
            brk = 0.0 if s['internal/air'] > 0.5 else 1.0
            f['fcs/left-brake-cmd-norm'] = brk; f['fcs/right-brake-cmd-norm'] = brk
        s.step()
        if rec is not None and i % k == 0:
            rec.append(state(f, s))


def ge(kg=2200.0, verbose=True):
    f, s = on_wheels(kg)
    rec = []
    fly(f, s, 3.0, rec)
    s['engage'] = 1; s['mode'] = 0
    out = []
    for h, sec in GE_HEIGHTS:
        s['target/height-ft'] = h
        r0 = len(rec)
        fly(f, s, sec, rec)
        last = [r for r in rec[r0:] if r['t'] > rec[-1]['t'] - 8]
        a = lambda k: float(np.mean([r[k] for r in last]))
        sd = lambda k: float(np.std([r[k] for r in last]))
        out.append(dict(height=h, wheel_ft=a('h'), wheel_sd=sd('h'), kw=a('kw'), kw_sd=sd('kw'), coll=a('coll'), theta=a('theta'), phi=a('phi'),
                        drift=float(np.hypot(a('u'), a('v'))), weight=f['inertia/weight-lbs']))
        if verbose:
            p = out[-1]
            print('target %4.0f ft: wheels %6.1f (sd %.2f) ft  power %6.1f kW (sd %.1f)  coll %.3f  pitch %5.1f  roll %5.1f  drift %.2f ft/s'
                  % (h, p['wheel_ft'], p['wheel_sd'], p['kw'], p['kw_sd'], p['coll'], p['theta'], p['phi'], p['drift']))
    s['target/height-ft'] = -1.0                       # land: down at the reference's 2 ft/s, the lever lowered on the wheels
    for _ in range(120):
        fly(f, s, 1.0, rec)
        if rec[-1]['wow'] >= 3 and rec[-1]['coll'] < 0.02:
            break
    fly(f, s, 3.0, rec)
    if verbose:
        r = rec[-1]
        td = next(x for x in rec if x['t'] > rec[-1]['t'] - 200 and x['wow'] > 0 and x['h'] < 1)
        print('landed: wheels %.2f ft, on %d wheels, collective %.2f, roll %.1f, pitch %.1f; touchdown at %.1f ft/s, %.1f ft/s across'
              % (r['h'], r['wow'], r['coll'], r['phi'], r['theta'], td['vs'], float(np.hypot(td['u'], td['v']))))
    return dict(points=out, rec=rec)


def phase(name, f, s, sec, rec, until=None, every=0.25):
    """fly for sec seconds (or until until(state) holds, at most sec), marking the records with the phase's name"""
    r0 = len(rec)
    dt = 1.0
    t = 0.0
    while t < sec:
        fly(f, s, dt, rec, every)
        t += dt
        if until is not None and until(rec[-1]):
            break
    for r in rec[r0:]:
        r['phase'] = name
    return rec[r0:]


def mode(f, s, m, rec, **targets):
    """change the autopilot's mode, fly a step so that its references start from the aircraft, then set the targets"""
    s['mode'] = m
    fly(f, s, 0.25, rec)
    for k, v in targets.items():
        s['target/' + k.replace('_', '-')] = v


def mission(kg=2200.0, verbose=True):
    """a test flight: take-off to a 10 ft hover; accelerate to 40 kt climbing (hover mode: ground speed and height),
    then to 60 kt and 1000 ft (cruise mode); cruise at 100 kt; slow to 65 kt; a practice autorotation (the fuel flow
    lever to idle, the collective on the rotor speed, 65 kt) to 300 ft; power back on: the lever to flight, the rotor
    speed held until the engine has caught the rotor again, then a climb; slow to 20 kt and to a hover at 10 ft; land"""
    f, s = on_wheels(kg)
    rec = []
    fly(f, s, 3.0, rec)
    s['engage'] = 1; s['mode'] = 0; s['target/height-ft'] = 10.0
    phase('take-off', f, s, 40, rec)
    s['target/u-fps'] = 40 * 1.6878; s['target/height-ft'] = 200.0
    phase('accelerate', f, s, 60, rec, until=lambda r: r['kt'] > 35)
    mode(f, s, 1, rec, alt_ft=1000.0, speed_kt=60.0)
    phase('climb at 60 kt', f, s, 150, rec, until=lambda r: abs(r['kt'] - 60) < 2 and r['h'] > 990)
    s['target/speed-kt'] = 100.0
    phase('cruise 100 kt', f, s, 60, rec)
    s['target/speed-kt'] = 65.0
    phase('slow to 65 kt', f, s, 40, rec)
    f['eng/flow-lever'] = 0.35
    mode(f, s, 2, rec, nr_rpm=353.2)
    phase('autorotation', f, s, 60, rec, until=lambda r: r['h'] < 300)
    f['eng/flow-lever'] = 1.0
    phase('power on', f, s, 30, rec, until=lambda r: f['eng/engaged'] > 0.5)
    mode(f, s, 1, rec, alt_ft=f['position/h-sl-ft'] + 200.0)
    phase('recovery climb', f, s, 40, rec)
    s['target/speed-kt'] = 20.0; s['target/alt-ft'] = 100.0
    phase('approach', f, s, 120, rec, until=lambda r: r['kt'] < 22 and r['h'] < 110)
    mode(f, s, 0, rec, u_fps=0.0, v_fps=0.0, height_ft=10.0)
    phase('to the hover', f, s, 120, rec, until=lambda r: r['h'] < 11 and abs(r['u']) < 1)
    s['target/height-ft'] = -1.0
    phase('landing', f, s, 60, rec, until=lambda r: r['wow'] >= 3 and r['coll'] < 0.02)
    if verbose:
        for name in dict.fromkeys(r['phase'] for r in rec if 'phase' in r):
            ph = [r for r in rec if r.get('phase') == name]
            e = ph[-1]
            print('%-16s %4.0f s  end: h %5.0f ft %5.1f kt vs %5.0f fpm pitch %5.1f roll %4.1f NR %5.1f %4.0f kW coll %.2f | max|roll| %4.1f max|dpsi| %4.1f NR %5.1f-%5.1f'
                  % (name, ph[-1]['t'] - ph[0]['t'], e['h'], e['kt'], e['vs'] * 60, e['theta'], e['phi'], e['nr'], e['kw'], e['coll'],
                     max(abs(r['phi']) for r in ph), max(abs(((r['psi'] - ph[0]['psi'] + 180) % 360) - 180) for r in ph),
                     min(r['nr'] for r in ph), max(r['nr'] for r in ph)))
    return dict(rec=rec)


STAGES = dict(ge=ge, mission=mission)


if __name__ == '__main__':
    kg = next((float(a[5:]) for a in sys.argv[1:] if a.startswith('--kg=')), 2200.0)
    names = [a for a in sys.argv[1:] if not a.startswith('--')] or list(STAGES)
    for n in names:
        res = STAGES[n](kg)
        json.dump(res, open('out/flights_%s.json' % n, 'w'), indent=1)
