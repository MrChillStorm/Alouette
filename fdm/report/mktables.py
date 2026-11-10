#!/usr/bin/env python3
"""The report's tables (tab/tab_*.tex) and the numbers quoted in its text (tab/numbers.tex, as \\N... macros), from
report/results/*.json (mkresults.py) and the published data they are compared with."""
import os, sys, json, math, re
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
FDM = os.path.dirname(HERE)
sys.path.insert(0, FDM)
import al3data as U
import artouste as A
from data import al3_published as PB
TAB = os.environ.get('AL3_TAB') or os.path.join(HERE, 'tab')   # AL3_TAB: elsewhere (tests)
NUM = {}
KT_KMH = 1.852
FT_M = 0.3048


def R(name):
    d = json.load(open(os.path.join(os.environ.get('AL3_RESULTS') or os.path.join(HERE, 'results'), name + '.json')))
    if name.startswith('trims_'):
        d = [r for r in d if r['res'] < TRIM_RES]       # trims that converged: past the blades' stall they do not
    return d


TRIM_RES = 0.05                 # ft/s^2, rad/s^2: a trim's largest residual acceleration


def num(name, value, fmt='%.1f'):
    assert re.fullmatch('[A-Za-z]+', name), name
    s = (fmt % value) if not isinstance(value, str) else value
    NUM[name] = '$-$' + s[1:] if s.startswith('-') else s


def write(name, body):
    open(os.path.join(TAB, 'tab_%s.tex' % name), 'w').write(body)


def f(x, fmt='%+.2f'):
    return '--' if x is None or (isinstance(x, float) and math.isnan(x)) else (fmt % x).replace('-', '$-$')


def tabular(cols, head, rows, foot=''):
    return ('\\begin{tabular}{@{}%s@{}}\n\\toprule\n%s \\\\\n\\midrule\n' % (cols, head) +
            ''.join(r + ' \\\\\n' for r in rows) + foot + '\\bottomrule\n\\end{tabular}\n')


def cross(xs, ys, y0):
    """the first x where y crosses y0 (linear), or nan"""
    for i in range(len(xs) - 1):
        if (ys[i] - y0) * (ys[i + 1] - y0) <= 0 and ys[i] != ys[i + 1]:
            return xs[i] + (y0 - ys[i]) * (xs[i + 1] - xs[i]) / (ys[i + 1] - ys[i])
    return float('nan')


# ---------------------------------------------------------------- the generated model
def model_table():
    root = os.path.join(FDM, 'root', 'aircraft', 'alouette3', 'Systems')
    files = [('al3-aero.xml', 'the aerodynamic functions: blade segments, wake, tail rotor, the airframe; the six force and moment axes', 'function'),
             ('al3-rotor.xml', 'blade flap and lag, rotor azimuth and speed, Peters--He wake states, the vortex ring, tail rotor flapping and inflow', 'fcs'),
             ('al3-airframe.xml', 'fuselage, hub, gear and stabiliser loads, the ring-vortex wake on them, at 240\\,Hz', 'fcs'),
             ('al3-drive.xml', 'engine, fuel control and governor, starter, clutch, freewheel, rotor brake, fuel', 'fcs'),
             ('al3-controls.xml', 'pilot controls, the mirror image, rigging, servos', 'fcs'),
             ('al3-outputs.xml', 'properties for the 3-D model and the instruments', 'fcs')]
    rows, nfun, nfcs = [], 0, 0
    for fn, what, kind in files:
        p = os.path.join(root, fn)
        if not os.path.exists(p):
            continue
        s = open(p).read()
        n_f = len(re.findall(r'<function name=', s))
        n_c = len(re.findall(r'<(?:fcs_function|pure_gain|summer|lag_filter|integrator|switch|kinematic|actuator|lead_lag_filter|'
                             r'second_order_filter|deadband|aerosurface_scale)\b', s))
        nfun += n_f; nfcs += n_c
        rows.append('{\\footnotesize\\texttt{%s}} & %s & %d' % (fn, what, n_f + n_c))
    write('model', '\\begin{tabularx}{\\columnwidth}{@{}>{\\raggedright\\arraybackslash}p{2.3cm}X r@{}}\n\\toprule\nFile & Content & Items \\\\\n\\midrule\n'
          + ''.join(r + ' \\\\\n' for r in rows) + '\\bottomrule\n\\end{tabularx}\n')
    num('nFunc', '%d' % nfun); num('nFcs', '%d' % nfcs)


# ---------------------------------------------------------------- the data
def params_table():
    m = lambda ft: ft * FT_M
    nu_b = math.sqrt(1 + U.E_FLAP * U.S_B / U.I_B)
    nu_l = math.sqrt(U.E_FLAP * U.S_B / U.I_B)
    sigma = U.NB * U.CHORD / (math.pi * U.R)
    vtip = U.OMEGA * m(U.R)
    lock = 0.002377 * 5.73 * U.CHORD * U.R ** 4 / U.I_B
    rows = [('Main rotor', 'radius, blades, chord', '%.2f\\,m, %d, %.3f\\,m' % (m(U.R), U.NB, m(U.CHORD)), 'TC, HL'),
            ('', 'speed, tip speed, solidity', '%.1f\\,rpm, %.0f\\,m/s, %.4f' % (U.OMEGA * 30 / math.pi, vtip, sigma), 'TD'),
            ('', 'twist, root cut-out, shaft tilt', '%.1f$^\\circ$, %.2f\\,m, %.1f$^\\circ$ fwd' % (U.TWIST, m(U.R_ROOT), U.SHAFT_TILT / U.DEG), 'HL, 3D'),
            ('', 'section', 'NACA 63A011.5', 'HL'),
            ('', 'flap/lag hinge offset', '%.3f\\,m (%.1f\\,\\%%\\,$R$)' % (m(U.E_FLAP), 100 * U.E_FLAP / U.R), '3D'),
            ('', 'blade mass, Lock number', '%.1f\\,kg, %.1f' % (U.M_BLADE, lock), 'E88 (est.)'),
            ('', 'flap, lag frequency', '%.3f, %.3f\\,/rev' % (nu_b, nu_l), 'derived'),
            ('', 'lag damper', '%.0f\\,\\%% of critical' % (100 * U.LAG_ZETA), 'est.'),
            ('Tail rotor', 'radius, blades, chord', '%.3f\\,m, %d, %.2f\\,m' % (m(U.R_TR), U.NB_TR, m(U.CHORD_TR)), 'TD, AC'),
            ('', 'speed, sections', '%.0f\\,rpm; NACA 0018--0015' % (U.OMEGA * U.GEAR_TR * 30 / math.pi), 'SR, HL'),
            ('Engine', 'Artouste III B: speed, ratings', '%.0f\\,rpm; %.0f, %.0f\\,kW' % (U.N_ENGINE, U.P_TO_KW, U.P_MCP_KW), 'TE'),
            ('', 'reduction to the rotor', '%.2f:1' % U.GEAR_ENGINE, 'TE, TD'),
            ('Mass', 'empty, maximum', '%.0f, %.0f\\,kg' % (U.EMPTY_KG, U.MTOW_KG), 'VP, TC')]
    body = ''.join(('\\multicolumn{3}{@{}l}{\\textit{%s}} \\\\\n' % it if it else '') + '\\quad %s & %s & %s \\\\\n' % (d, v, src)
                   for it, d, v, src in rows)
    write('params', '\\begin{tabularx}{\\columnwidth}{@{}>{\\raggedright\\arraybackslash}X>{\\raggedright\\arraybackslash}p{2.75cm}l@{}}\n'
          '\\toprule\nItem & Value & Source \\\\\n\\midrule\n' + body + '\\bottomrule\n\\end{tabularx}\n')
    num('uFlap', '%.3f' % nu_b); num('uLag', '%.3f' % nu_l); num('Lock', '%.1f' % lock); num('Sigma', '%.4f' % sigma)
    num('Vtip', '%.0f' % vtip)


# ---------------------------------------------------------------- performance against the published figures
def perf_table():
    rows = []
    t22, t17 = R('trims_2200'), R('trims_1750')
    V = [r['V'] for r in t22]; P = [r['engine_kw'] for r in t22]
    vmax_mcp = cross(V[4:], P[4:], U.P_MCP_KW)
    num('VmaxMcp', '%.0f' % vmax_mcp); num('VmaxMcpKmh', '%.0f' % (vmax_mcp * KT_KMH))
    pv = lambda kmh: float(np.interp(kmh / KT_KMH, V, P))
    num('Pcruise', '%.0f' % pv(PB.CRUISE_KMH['TD'])); num('Pvne', '%.0f' % pv(PB.VMAX_KMH['TD']))
    num('Phover', '%.0f' % P[0]); num('Pmin', '%.0f' % min(P)); num('Vmin', '%.0f' % V[int(np.argmin(P))])
    num('HovBank', '%.1f' % t22[0]['phi'])
    num('VtrimMax', '%.0f' % max(V)); num('VtrimMaxLight', '%.0f' % max(r['V'] for r in t17))
    num('PtrimMax', '%.0f' % P[int(np.argmax(V))])
    rows.append('Maximum level speed, 405\\,kW & 210 ($V_{NE}$) & %.0f & km/h' % (vmax_mcp * KT_KMH))
    rows.append('Power at 185\\,km/h & -- & %.0f & kW' % pv(185))
    rows.append('Power at $V_{NE}$, 210\\,km/h & -- & %.0f & kW' % pv(210))
    c = R('climb')
    for key, kg, pub in (('mcp', 2200, '4.3 (TD), 5.5 (AS)'), ('mcp_1750', 1750, '8.7 (TD)')):
        pts = c[key]['points']
        best = max(pts, key=lambda p: p['fpm'])
        rows.append('Rate of climb at 405\\,kW, %d\\,kg & %s & %.1f & m/s' % (kg, pub, best['fpm'] * FT_M / 60))
        num('Climb' + ('Heavy' if kg == 2200 else 'Light'), '%.1f' % (best['fpm'] * FT_M / 60))
        num('ClimbV' + ('Heavy' if kg == 2200 else 'Light'), '%.0f' % best['V'])
    h = R('hover')
    for kg, mode in ((2200, 'oge'), (2200, 'ige'), (1750, 'oge'), (1750, 'ige')):
        pts = [p for p in h['%d-%s' % (kg, mode)]['points'] if p[3] < 0.05]        # converged trims
        alt = [p[0] * FT_M for p in pts]; pk = [p[1] for p in pts]
        pubs = [(src, cm) for (src, k2, m2), cm in PB.HOVER_CEILING_M.items() if k2 == kg and m2 == mode]
        implied = ', '.join('%.0f (%s)' % (np.interp(cm, alt, pk), src) for src, cm in pubs)
        rows.append('Hover %s %d\\,kg: power at sea level; at the published ceilings & %s & %.0f; %s & kW'
                    % (mode.upper(), kg, ', '.join('%.0f\\,m (%s)' % (cm, src) for src, cm in pubs), pk[0], implied))
        num('Hov%s%s' % (mode.capitalize(), 'Heavy' if kg == 2200 else 'Light'), '%.0f' % pk[0])
    # the power the published ceilings imply (the model's power to hover at each), and the model's climb at it
    implied = {}
    for (src, kg, mode), cm in PB.HOVER_CEILING_M.items():
        pts = [p for p in h['%d-%s' % (kg, mode)]['points'] if p[3] < 0.05]
        implied[(src, kg, mode)] = float(np.interp(cm, [p[0] * FT_M for p in pts], [p[1] for p in pts]))
    for (src, kg, mode), pw in implied.items():
        num('Impl%s%s%s' % (src.capitalize(), mode.capitalize(), 'Heavy' if kg == 2200 else 'Light'), '%.0f' % pw)
    heavy = [pw for (src, kg, mode), pw in implied.items() if kg == 2200]
    num('ImplMin', '%.0f' % min(heavy)); num('ImplMax', '%.0f' % max(heavy)); num('ImplMean', '%.0f' % np.mean(heavy))
    if 'by_power' in c:
        bp = c['by_power']
        roc = lambda pw: float(np.interp(pw, [p['p_avail'] for p in bp], [p['fpm'] * FT_M / 60 for p in bp]))
        num('ClimbAtImpl', '%.1f' % roc(np.mean(heavy)))
        num('ClimbAtImplMin', '%.1f' % roc(min(heavy))); num('ClimbAtImplMax', '%.1f' % roc(max(heavy)))
        rows.append('Rate of climb, 2200\\,kg, at the power the ceilings imply (%.0f--%.0f\\,kW) & 4.3 (TD), 5.5 (AS) & %.1f--%.1f & m/s'
                    % (min(heavy), max(heavy), roc(min(heavy)), roc(max(heavy))))
    vh_impl = cross(V[4:], P[4:], np.mean(heavy))
    num('VmaxImpl', '%.0f' % (vh_impl * KT_KMH))
    rows.append('Maximum level speed at %.0f\\,kW & 180--195 (cruise) & %.0f & km/h' % (np.mean(heavy), vh_impl * KT_KMH))
    # where the hover cannot be trimmed (the rotor stalls), 2200 kg out of ground effect
    pts = h['2200-oge']['points']
    lim = next((p[0] * FT_M for p in pts if p[3] >= 0.05), float('nan'))
    last = max((p[0] * FT_M for p in pts if p[3] < 0.05 and p[0] * FT_M < lim), default=float('nan'))
    num('HovStallAlt', '%.0f' % lim); num('HovLastAlt', '%.0f' % last)
    # the blade loading there: C_T / sigma of the weight at the last trimmed and the first untrimmed altitude
    rho = lambda hm: 1.225 * ((288.15 - 0.0065 * hm) / 288.15) ** 4.2559
    Rm, cm = U.R * FT_M, U.CHORD * FT_M
    ct_s = lambda hm: 2200 * 9.80665 / (rho(hm) * math.pi * Rm ** 2 * (U.OMEGA * Rm) ** 2) / (U.NB * cm / (math.pi * Rm))
    num('HovStallCtLo', '%.2f' % ct_s(last)); num('HovStallCtHi', '%.2f' % ct_s(lim))
    a = R('autorotation')['points']
    best = min(a, key=lambda p: -p['fpm'])
    rows.append('Autorotation, least rate of descent, 2200\\,kg & about %.0f (AO) & %.0f at %d\\,kt & ft/min' % (PB.AUTOROTATION_FPM['AO'], -best['fpm'], best['V']))
    num('AutoRod', '%.0f' % -best['fpm']); num('AutoV', '%d' % best['V'])
    write('perf', '\\begin{tabularx}{\\textwidth}{@{}X l l l@{}}\n\\toprule\nQuantity (sea level, standard day) & Published & Model & Unit \\\\\n\\midrule\n'
          + ''.join(r + ' \\\\\n' for r in rows) + '\\bottomrule\n\\end{tabularx}\n')


# ---------------------------------------------------------------- derivatives and modes
DERIVS = ['Xu', 'Zw', 'Mu', 'Mw', 'Mq', 'Yv', 'Lv', 'Nv', 'Lp', 'Lq', 'Mp', 'Np', 'Nr', 'Zcol', 'Mlon', 'Llat', 'Nped', 'Ncol', 'Lped']


def derivs_table():
    d = R('derivs')
    head = 'Derivative & ' + ' & '.join('%d\\,kt' % r['V'] for r in d)
    rows = []
    for k in DERIVS:
        vals = [r['D'][k] * (100 if k[1:] in ('col', 'lon', 'lat', 'ped') else 1) for r in d]
        rows.append('$%s_{%s}$ & ' % (k[0], k[1:] if k[1:] in ('col', 'lon', 'lat', 'ped') else k[1:]) + ' & '.join(f(v, '%+.3f') for v in vals))
    write('derivs', tabular('l' + 'r' * len(d), head, rows))
    for r in d:
        tag = {0: 'Hov', 60: 'Sixty', 100: 'Hundred'}[int(r['V'])]
        for k in ('Lp', 'Mq', 'Nr', 'Zw'):
            num(k + tag, '%.2f' % r['D'][k])
    # modes
    import derivatives as DV
    rows = []
    for r in d:
        ev = [complex(*e) for e in r['eig']]
        rows.append('%d\\,kt & %s' % (r['V'], ', '.join(DV.describe(ev)).replace('-', '$-$')))
    write('modes', tabular('lp{6.2cm}', 'Speed & Modes: [damping ratio, frequency rad/s], (inverse time constant 1/s)', rows))


# ---------------------------------------------------------------- engine, ground effect, the VRS
def engine_numbers():
    e = R('engine')
    ev = e['start']['events']
    for k, n in (('lightoff', 'LightOff'), ('rotor_turns', 'RotorTurns'), ('engaged', 'Engaged'), ('rotor_95', 'RotorNinetyFive')):
        num('St' + n, '%.1f' % ev[k])
    num('StRotorRpm', '%.0f' % ev['rotor_turns_rpm']); num('StIdleRpm', '%.0f' % ev['idle_rpm']); num('StIdleTfour', '%.0f' % ev['idle_t4'])
    num('StTfourStart', '%.0f' % ev['t4_peak_start']); num('StTfourRunup', '%.0f' % ev['t4_peak_runup'])
    g = e['governor']['events']
    num('GovDroop', '%.2f' % g['droop_pct']); num('GovKwA', '%.0f' % g['kw0']); num('GovKwB', '%.0f' % g['kw1'])
    fl = e['failure_held']['events']
    num('FailTthreeten', '%.1f' % fl['t_310']); num('FailTtwoseventy', '%.1f' % fl['t_270'])


def ge_table():
    g = R('ge')['points']
    hub = (U.HUB_YASIM[2] - U.MAIN_WHEELS[0][2]) / FT_M
    rows = ['%.0f & %.1f & %.2f & %.1f & %.3f' % (p['height'], p['wheel_ft'], (p['wheel_ft'] + hub) / (2 * U.R), p['kw'], p['kw'] / g[-1]['kw']) for p in g]
    write('ge', tabular('rrrrr', 'Target, ft & Wheels, ft & $z/D$ & Power, kW & Ratio', rows))
    num('GeLowRatio', '%.2f' % (g[0]['kw'] / g[-1]['kw'])); num('GeHundredKw', '%.0f' % g[-1]['kw'])


def vrs_numbers():
    d = R('descent')
    num('Vh', '%.1f' % d['vh_hover']); num('VhFpm', '%.0f' % (d['vh_hover'] * 60))
    for sp in d['speeds']:
        tag = {0: 'Zero', 10: 'Ten', 20: 'Twenty', 30: 'Thirty', 40: 'Forty'}[int(sp['kt'])]
        b = sp['boundary']
        if len(b) >= 2:
            num('Vrs%sLo' % tag, '%.0f' % (-b[0] * d['vh_hover'] * 60)); num('Vrs%sHi' % tag, '%.0f' % (-b[1] * d['vh_hover'] * 60))
            num('Vrs%sZlo' % tag, '%.2f' % -b[0]); num('Vrs%sZhi' % tag, '%.2f' % -b[1])


def mission_numbers():
    rec = R('mission')['rec']
    ph = lambda n: [r for r in rec if r.get('phase') == n]
    au = ph('autorotation')
    steady = [r for r in au if r['t'] > au[0]['t'] + 10]
    num('MiAutoFpm', '%.0f' % -np.mean([r['vs'] * 60 for r in steady])); num('MiAutoKt', '%.0f' % np.mean([r['kt'] for r in steady]))
    num('MiAutoNrMin', '%.0f' % min(r['nr'] for r in au))
    num('MiCruiseKw', '%.0f' % np.mean([r['kw'] for r in ph('cruise 100 kt')[-40:]]))
    land = ph('landing')
    td = next((r for r in land if r['wow'] > 0), land[-1])
    num('MiTouchFpm', '%.0f' % (-td['vs'] * 60))
    num('MiDuration', '%.0f' % (rec[-1]['t'] - rec[0]['t']))


if __name__ == '__main__':
    os.makedirs(TAB, exist_ok=True)
    for fn in (model_table, params_table, perf_table, derivs_table, engine_numbers, ge_table, vrs_numbers, mission_numbers):
        try:
            fn()
        except FileNotFoundError as e:
            print(fn.__name__, 'skipped:', e)
    open(os.path.join(TAB, 'numbers.tex'), 'w').write(''.join('\\newcommand{\\N%s}{%s}\n' % (k, v) for k, v in sorted(NUM.items())))
    print(len(NUM), 'numbers')
