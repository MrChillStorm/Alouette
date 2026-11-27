#!/usr/bin/env python3
"""Cruise fuel consumption against the SA 316B's published cruise charts (data/al3_cruise_charts.py).

At each point of the charts the model is trimmed in level flight at the chart's weight, altitude (standard atmosphere)
and true airspeed, and its fuel flow, by its own engine's fuel curve (artouste.py), is set against the chart's (litres
per km times the speed). This is a comparison, not a check the model passes: nothing is fitted to these charts.

A difference in fuel is the power the rotors and airframe take, the engine's fuel for that power, or both, and the
charts cannot tell them apart. Two figures show how much rests on the engine's curve: the power at which that curve
burns the chart's fuel, and the consumption at the rating that would give the chart's fuel at the model's power (its
no-load fuel unchanged). The curve's own consumption at the rating is U.SFC.

One comparison does not rest on the engine's curve at all: at one altitude more power burns more fuel, whatever the
curve, so the order of the weights' fuel flows is the order of their powers. The report gives the heaviest weight's
fuel flow less the lightest's, the chart's and the model's.

  python3 cruisecharts.py    writes out/cruisecharts.json and .md; exits 1 if a trim fails other than at the points
                             the model can't fly (SHORT), or the chart readings disagree with themselves (true against
                             indicated airspeed, TOL_TAS)

The test model is generated into a temporary folder; nothing in the aircraft is touched. A few minutes on 4 cores.
"""
import json, math, os, sys, tempfile
from multiprocessing import Pool
import numpy as np
import al3data as U
import artouste as E
import harness as H
import hovercharts as HC
from atmosphere import isa
from data import al3_cruise_charts as C

HERE = os.path.dirname(os.path.abspath(__file__))
WORKERS = 4
TOL_TAS = 0.015             # the charts' dashed true airspeeds against their indicated ones / sqrt(density ratio)
TRIM_MAX = 0.15             # the largest weighted acceleration left in a trim that counts (harness.trim's scale)
# points the model can't fly (kg, altitude m): level flight stops short of the chart's speed, at about 92 kt for 98 and
# 74 kt for 81, with no control near its stop; what is left over is mostly lift, and the power climbs steeply towards it
# (which looks like the blades stalling; their angles of attack were not examined)
SHORT = {(1800, 4000), (2000, 4000)}
STEPS_KT = (10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 95.0, 105.0)    # from the hover: a poor start can find
#                                              another equilibrium (at 4000 m, 2000 kg, steps of 20 kt lost it at 40 kt)
EXTRA = H.ENGINE_POWER + ('eng/wf-for-load', 'eng/delta-sqrt-theta')


def level(f, kg, alt_m, v_kt, x0):
    H.set_weight(f, kg)
    x, r, e = H.trim(f, v_kt, alt_ft=alt_m / 0.3048, x0=x0, extra=EXTRA, tol=0.008, iters=40)
    cost = float(np.abs(r * HC.SCALE).max())
    if not cost <= TRIM_MAX:
        raise RuntimeError('no level trim: %.0f kg, %.0f m, %.0f kt (residual %.2f)' % (kg, alt_m, v_kt, cost))
    return x, e, cost


def from_hover(f, kg, alt_m, v_kt):
    x = HC.X0
    for v in [0.0] + [s for s in STEPS_KT if s < v_kt - 4]:
        x = level(f, kg, alt_m, v, x)[0]
    return level(f, kg, alt_m, v_kt, x)


def climb_to(root, kg, last, alt_m, v_kt):
    """from the last trimmed point (altitude, speed, controls) in steps along the straight line to the new one, halved
    where a trim fails, down to a sixteenth"""
    a0, v0, x = last
    done, step = 0.0, 1.0
    while True:
        s = min(1.0, done + step)
        try:
            x1, e, cost = level(HC.model(root), kg, a0 + s * (alt_m - a0), v0 + s * (v_kt - v0), x)
        except RuntimeError:
            HC._models.clear()                       # a failed trim can leave the rotor wild: the next starts afresh
            step /= 2
            if step < 1 / 16.0:
                raise
            continue
        x, done = x1, s
        if done >= 1.0:
            return x, e, cost


def weight(job):
    """one weight's points, each altitude reached from the one below it"""
    root, kg = job
    out, last = [], None
    for alt in sorted(C.CRUISE[kg]):
        tas, ias, per_km = C.CRUISE[kg][alt]
        v_kt = tas / 1.852
        try:
            try:
                if last is None:
                    raise RuntimeError
                x, e, cost = climb_to(root, kg, last, alt, v_kt)
            except RuntimeError:                     # none yet, or no way up from it
                HC._models.clear()
                x, e, cost = from_hover(HC.model(root), kg, alt, v_kt)
        except RuntimeError as err:
            HC._models.clear()
            out.append(dict(kg=kg, alt_m=alt, error=str(err)))
            continue
        last = (alt, v_kt, x)
        load, omega, wf_c, dsth = (float(q) for q in e)
        power = load * omega / 737.562
        chart_lh = C.fuel_flow_lh(kg, alt)
        chart_kgh = chart_lh * U.FUEL_KG_L
        model_kgh = wf_c * dsth
        slope = (E.WF_MCP - E.WF_NL) / U.P_MCP_KW    # kg/h per kW; the no-load fuel goes as delta sqrt(theta)
        sig = isa(float(alt))[2] / isa(0.0)[2]
        out.append(dict(kg=kg, alt_m=alt, pitch=C.pitch(alt), tas_kmh=tas, chart_l_km=per_km, chart_lh=chart_lh,
                        model_lh=model_kgh / U.FUEL_KG_L, model_kw=power,
                        chart_kw_on_model_curve=(chart_kgh - E.WF_NL * dsth) / slope,
                        sfc_for_chart=(E.WF_NL + (chart_kgh - E.WF_NL * dsth) * U.P_MCP_KW / power) / U.P_MCP_KW,
                        collective_shaft_deg=H.deg(x)[0], tas_vs_ias=tas / (ias / math.sqrt(sig)) - 1,
                        curve_check_kw=(model_kgh - E.WF_NL * dsth) / slope - power, trim_residual=cost))
    return out


def table(head, rows):
    return HC.table(head, rows)


def report(res):
    ok = [p for p in res['points'] if 'error' not in p]
    s = ['# The model\'s cruise fuel consumption against the SA 316B\'s charts', '', C.SOURCE + '.',
         'Level flight at the charts\' weight, altitude (standard atmosphere) and true airspeed. Fuel at %g kg per litre.'
         % U.FUEL_KG_L, '',
         table(['kg', 'altitude m', 'true airspeed km/h', 'chart L/h', 'model L/h', 'model less chart', 'model kW',
                'kW at which the model\'s engine burns the chart\'s fuel', 'kg/kWh at the rating that would match'],
               [[p['kg'], p['alt_m'], '%.1f' % p['tas_kmh'], '%.1f' % p['chart_lh'], '%.1f' % p['model_lh'],
                 '%+.1f %%' % (100 * (p['model_lh'] / p['chart_lh'] - 1)), '%.0f' % p['model_kw'],
                 '%.0f' % p['chart_kw_on_model_curve'], '%.3f' % p['sfc_for_chart']] for p in ok]),
         '', 'The engine\'s curve takes %.2f kg/kWh at the %.0f kW rating and %.0f kg/h unloaded at sea level.'
         % (U.SFC, U.P_MCP_KW, E.WF_NL), '',
         '## All weights at one altitude (one pitch on the charts)', '']
    rows = []
    for alt in sorted({p['alt_m'] for p in ok}):
        q = [p for p in ok if p['alt_m'] == alt]
        rng = lambda key, fmt: (fmt + ' to ' + fmt) % (min(p[key] for p in q), max(p[key] for p in q))
        rows.append([alt, '%.2f' % C.pitch(alt), ', '.join(str(p['kg']) for p in q), rng('chart_lh', '%.0f'),
                     rng('model_lh', '%.0f'), rng('model_kw', '%.0f'), rng('chart_kw_on_model_curve', '%.0f'),
                     rng('sfc_for_chart', '%.3f')])
    s.append(table(['altitude m', 'chart pitch', 'weights kg', 'chart L/h', 'model L/h', 'model kW', 'chart kW (model\'s curve)',
                    'kg/kWh that would match'], rows))
    rows = []
    for alt in sorted({p['alt_m'] for p in ok}):
        q = sorted((p for p in ok if p['alt_m'] == alt), key=lambda p: p['kg'])
        if len(q) > 1:
            lo, hi = q[0], q[-1]
            change = lambda key: '%+.1f %%' % (100 * (hi[key] / lo[key] - 1))
            rows.append([alt, '%d less %d' % (hi['kg'], lo['kg']), change('chart_lh'), change('model_lh'), change('model_kw')])
    s += ['', '## The order of the weights, which no engine curve changes', '',
          table(['altitude m', 'kg', 'chart fuel flow', 'model fuel flow', 'model power'], rows)]
    s += ['', '## Checks', '',
          table(['check', 'worst', 'limit', ''],
                [[c['check'], c['worst'], c['limit'], 'pass' if c['ok'] else 'FAIL'] for c in res['checks']])]
    short = ['- %d kg at %d m%s: %s' % (p['kg'], p['alt_m'], ', a point the model can\'t fly' if (p['kg'], p['alt_m']) in SHORT
                                      else '', p['error']) for p in res['points'] if 'error' in p]
    if short:
        s += ['', '## Not flown', ''] + short
    return '\n'.join(s) + '\n'


def main():
    os.makedirs(os.path.join(HERE, 'out'), exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        root = HC.build(U.GE_K, tmp)
        with Pool(WORKERS) as pool:
            points = [p for ps in pool.map(weight, [(root, kg) for kg in sorted(C.CRUISE)], chunksize=1) for p in ps]
    ok = [p for p in points if 'error' not in p]
    failed = [(p['kg'], p['alt_m']) for p in points if 'error' in p]
    tas = max((abs(p['tas_vs_ias']) for p in ok), default=float('inf'))
    curve = max((abs(p['curve_check_kw']) for p in ok), default=float('inf'))
    res = dict(source=C.SOURCE, points=points, checks=[
        dict(check='points that trimmed, but for those the model can\'t fly', limit='none',
             worst='%d of %d not flown, %d of them unexpected' % (len(failed), len(points), len(set(failed) - SHORT)),
             ok=not set(failed) - SHORT),
        dict(check='chart readings: true against indicated airspeed', worst='%.1f %%' % (100 * tas),
             limit='%.1f %%' % (100 * TOL_TAS), ok=tas <= TOL_TAS),
        dict(check='the engine curve used here against the model\'s own fuel', worst='%.2f kW' % curve, limit='0.50 kW',
             ok=curve <= 0.5)])
    json.dump(res, open(os.path.join(HERE, 'out', 'cruisecharts.json'), 'w'), indent=1, default=float)
    text = report(res)
    open(os.path.join(HERE, 'out', 'cruisecharts.md'), 'w').write(text)
    print(text)
    sys.exit(0 if all(c['ok'] for c in res['checks']) else 1)


if __name__ == '__main__':
    main()
