#!/usr/bin/env python3
"""The engine and the rotor speed, offline (artouste.py), against the start as described [RW, LM, E77]:

  start      on the ground, the rotor brake off and the fuel flow lever at idle (35 %), the starter pressed: light-off,
             the starter's cut-out (about 14 000 rpm [RW]), idle (16 330 rpm recorded [SR]; 15 000 [AO], 17 000-18 000
             [RW]); then the lever advanced to flight over 10 s, as a pilot does it: the centrifugal clutch engages
             from 19 000 rpm [AO], the rotor turning at 19 500-21 000 rpm [LM], the run-up to the governed 353 rpm
  governor   hovering at 2200 kg, the airframe held, the collective raised 10 % of its travel in 0.5 s: the rotor's droop
             and recovery (the isochronous governor [E77] has no droop of its own) and the engine's power and T4
  failure    at 60 kt level, 2200 kg, the airframe held, the fuel cut: the rotor's decay with the collective held, and
             with it lowered to the autorotation's after a 1 s delay (the TCDS's 270 rpm the lower limit [TC])

  python3 engine_response.py [stage ...]      prints, and writes out/engine.json
"""
import sys, json
import numpy as np
import harness as H
import al3data as U
import artouste as A

RPM = 30 / np.pi
KEYS = dict(t='simulation/sim-time-sec', n='eng/n-pct', rpm='eng/rpm', t4='eng/t4-degc', wf='eng/fuel-flow-kgh', lit='eng/lit',
            engaged='eng/engaged', nr='rotor/omega-rad_sec', load='drive/load-ftlb', coll='/controls/engines/engine[0]/throttle')


def record(f, out, every):
    row = {k: f[p] for k, p in KEYS.items()}
    row['nr'] *= RPM
    row['kw'] = f['drive/load-ftlb'] * f['rotor/omega-rad_sec'] / A.KW_FTLB
    out.append(row)


def run_for(f, s, out, every=0.1):
    dt = f['simulation/dt']
    k = max(1, int(round(every / dt)))
    for i in range(int(round(s / dt))):
        f.run()
        if i % k == 0:
            record(f, out, every)


def start():
    f = H.load(model='al3-2200', hold_rpm=False)
    f['eng/init-running'] = 0
    f['ic/terrain-elevation-ft'] = 300; f['ic/h-sl-ft'] = 300 + H.wheels_above_ground_ft(f) + 0.05; f['ic/vt-kts'] = 0
    f['/controls/engines/engine[0]/throttle'] = 0.0
    f.run_ic()
    out = []
    run_for(f, 3.0, [])                          # settled on its wheels
    t0 = f['simulation/sim-time-sec']
    f['/controls/rotor/brake'] = 0; f['eng/fuel-valve'] = 1; f['eng/flow-lever'] = A.LEVER_IDLE
    f['eng/starter-cmd'] = 1
    run_for(f, 1.0, out)
    f['eng/starter-cmd'] = 0
    run_for(f, 59.0, out)
    for i in range(100):                         # the lever to flight over 10 s
        f['eng/flow-lever'] = A.LEVER_IDLE + (1 - A.LEVER_IDLE) * (i + 1) / 100
        run_for(f, 0.1, out)
    run_for(f, 50.0, out)
    for r in out:
        r['t'] -= t0
    ev = {}
    for r in out:
        if 'lightoff' not in ev and r['lit'] > 0.5: ev['lightoff'] = r['t']
        if 'rotor_turns' not in ev and r['nr'] > 5: ev['rotor_turns'] = r['t']; ev['rotor_turns_rpm'] = r['rpm']
        if 'engaged' not in ev and r['engaged'] > 0.5: ev['engaged'] = r['t']
        if 'rotor_95' not in ev and r['nr'] > 0.95 * U.OMEGA * RPM: ev['rotor_95'] = r['t']
    idle = [r for r in out if 50 < r['t'] < 60]
    ev['idle_rpm'] = float(np.mean([r['rpm'] for r in idle])); ev['idle_t4'] = float(np.mean([r['t4'] for r in idle]))
    ev['idle_wf'] = float(np.mean([r['wf'] for r in idle]))
    ev['t4_peak_start'] = max(r['t4'] for r in out if r['t'] < 60)
    ev['t4_peak_runup'] = max(r['t4'] for r in out if r['t'] >= 60)
    ev['final_nr'] = out[-1]['nr']; ev['final_rpm'] = out[-1]['rpm']
    return dict(rec=out, events=ev)


def held_trim(V, kg=2200.0):
    f = H.load(model='al3-2200')
    H.set_weight(f, kg)
    x, r, e = H.trim(f, V, alt_ft=1000.0, extra=H.ENGINE_POWER)
    f['drive/hold-rpm'] = 0; f['ctl/init'] = 0     # the rotor speed free, on the governor; the airframe stays held
    for _ in range(int(2.0 / f['simulation/dt'])):
        f.run()
    return f, x


def governor(step=0.10, ramp=0.5):
    f, x = held_trim(0.0)
    out = []
    run_for(f, 1.0, out, 0.02)
    c0 = x[0]
    n = int(round(ramp / f['simulation/dt']))
    for i in range(n):
        f['/controls/engines/engine[0]/throttle'] = c0 + step * (i + 1) / n
        f.run()
    run_for(f, 12.0, out, 0.02)
    t0 = out[0]['t']
    for r in out:
        r['t'] -= t0 + 1.0
    nr0 = out[0]['nr']
    low = min(out, key=lambda r: r['nr'])
    ev = dict(nr0=nr0, droop_pct=100 * (nr0 - low['nr']) / nr0, droop_t=low['t'], kw0=out[0]['kw'], kw1=out[-1]['kw'],
              t4_0=out[0]['t4'], t4_peak=max(r['t4'] for r in out), nr_end=out[-1]['nr'],
              settle_t=next((r['t'] for r in reversed(out) if abs(r['nr'] - nr0) > 0.005 * nr0), 0.0))
    return dict(rec=out, events=ev)


def failure(lower=True, delay=1.0, V=60.0):
    f, x = held_trim(V)
    out = []
    run_for(f, 0.5, out, 0.02)
    f['eng/fuel-valve'] = 0
    run_for(f, delay, out, 0.02)
    if lower:
        # the autorotation's collective at this speed (autorotation.py's trims, 2200 kg): the lever down to 0.10
        c0 = f['/controls/engines/engine[0]/throttle']; c1 = 0.10
        for i in range(48):
            f['/controls/engines/engine[0]/throttle'] = c0 + (c1 - c0) * (i + 1) / 48
            run_for(f, 1 / 96, out, 0.02)
    run_for(f, 8.0, out, 0.02)
    t0 = out[0]['t'] + 0.5
    for r in out:
        r['t'] -= t0
    ev = dict(nr0=out[0]['nr'], nr_min=min(r['nr'] for r in out), nr_end=out[-1]['nr'])
    for lim in (310.0, 270.0):
        ev['t_%d' % lim] = next((r['t'] for r in out if r['nr'] < lim), None)
    return dict(rec=out, events=ev)


STAGES = dict(start=start, governor=governor, failure_held=lambda: failure(lower=False), failure_lowered=lambda: failure(lower=True))

if __name__ == '__main__':
    names = sys.argv[1:] or list(STAGES)
    res = {}
    for n in names:
        res[n] = STAGES[n]()
        print(n, {k: (round(v, 2) if isinstance(v, float) else v) for k, v in res[n]['events'].items()})
    json.dump(res, open('out/engine.json', 'w'), indent=1)
