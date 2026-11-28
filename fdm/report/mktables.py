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
from data import al3_hover_charts as HOV
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
            ('', 'speed, sections', '%.0f\\,rpm; NACA 0018--0015' % (U.OMEGA * U.GEAR_TR * 30 / math.pi), 'SPEC, HL'),
            ('Engine', 'Artouste III B: speed, ratings', '%.0f\\,rpm; %.0f, %.0f\\,kW' % (U.N_ENGINE, U.P_TO_KW, U.P_MCP_KW), 'TE'),
            ('', 'reduction to the rotor', '%.2f:1' % U.GEAR_ENGINE, 'TE, TD'),
            ('Mass', 'empty, maximum', '%.0f, %.0f\\,kg' % (U.EMPTY_KG, U.MTOW_KG), 'SPEC, TC'),
            ('Inflow', 'hover correction $\\epsilon_h$, slope per 0.1 Mach', '%.4f, %.3f' % (U.HOVER_VI, U.HOVER_VI_MACH), 'SPEC, BO'),
            ('', 'ground effect $K_{GE}$', '%.3f' % U.GE_K, 'SPEC')]
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
    # the airframe's drag at 100 kt as an equivalent flat plate: its force along the airspeed over the dynamic pressure
    q100 = t22[V.index(100)]
    th = math.radians(q100['theta'])                       # level flight: the angle of attack is the pitch attitude
    drag_lb = -(q100['fx'] * math.cos(th) + q100['fz'] * math.sin(th))
    q_psf = 0.5 * 0.002377 * (100 * 1.68781) ** 2
    num('FlatPlate', '%.1f' % (drag_lb / q_psf * FT_M ** 2))
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
    at = {(q['src'], q['kg'], q['mode']): q['kw'] for q in h['implied']}     # trimmed at each published ceiling
    for kg, mode in ((2200, 'oge'), (2200, 'ige'), (1750, 'oge'), (1750, 'ige')):
        pts = [p for p in h['%d-%s' % (kg, mode)]['points'] if p[3] < 0.05]        # converged trims
        pubs = [(src, cm) for (src, k2, m2), cm in PB.HOVER_CEILING_M.items() if k2 == kg and m2 == mode]
        implied = ', '.join('%.0f (%s)' % (at[(src, kg, mode)], src) for src, cm in pubs)
        rows.append('Hover %s %d\\,kg: power at sea level; at the published ceilings & %s & %.0f; %s & kW'
                    % (mode.upper(), kg, ', '.join('%.0f\\,m (%s)' % (cm, src) for src, cm in pubs), pts[0][1], implied))
        num('Hov%s%s' % (mode.capitalize(), 'Heavy' if kg == 2200 else 'Light'), '%.0f' % pts[0][1])
    # the power the published ceilings at 2100 and 2200 kg imply (the model's power to hover at each), and the model's
    # climb and level speed at it
    imp = {(q['src'], q['kg'], q['mode']): q['kw'] for q in h['implied']}
    man = [kw for (src, kg, mode), kw in imp.items() if src == 'SPEC']
    aer = [kw for (src, kg, mode), kw in imp.items() if src == 'AS']
    num('ImplManLo', '%.0f' % min(man)); num('ImplManHi', '%.0f' % max(man))
    num('ImplAsLo', '%.0f' % min(aer)); num('ImplAsHi', '%.0f' % max(aer))
    rows.append('Power to hover at the manual\'s ceilings, 2100 and 2200\\,kg, in and out of ground effect & about %.0f (SPEC) '
                '& %.0f--%.0f & kW' % (HOV.POWER_USED_KW, min(man), max(man)))
    rows.append('Power to hover at A\\\'erostories\' ceilings, 2200\\,kg & -- & %.0f--%.0f & kW' % (min(aer), max(aer)))
    bp = sorted(c['by_power'], key=lambda p: p['p_avail'])
    kws = [p['p_avail'] for p in bp]; rocs = [p['fpm'] * FT_M / 60 for p in bp]
    roc = lambda pw: float(np.interp(pw, kws, rocs))
    assert kws[0] <= min(man) and max(aer) <= kws[-1], 'the climb against power does not span the implied powers'
    num('ClimbAtManLo', '%.1f' % roc(min(man))); num('ClimbAtManHi', '%.1f' % roc(max(man)))
    rows.append('Rate of climb, 2200\\,kg, at the manual\'s hover powers (%.0f--%.0f\\,kW) & 4.3 (TD), 5.5 (AS) & %.1f--%.1f & m/s'
                % (min(man), max(man), roc(min(man)), roc(max(man))))
    pub = sorted((PB.CLIMB_MS['TD-2200'], PB.CLIMB_MS['AS']))
    assert rocs[0] <= pub[0] and pub[1] <= rocs[-1], 'the published climb rates lie outside the powers tried'
    need = [float(np.interp(v, rocs, kws)) for v in pub]
    num('PwClimbLo', '%.0f' % need[0]); num('PwClimbHi', '%.0f' % need[1])
    rows.append('Power at which the model climbs at the published 4.3--5.5\\,m/s, 2200\\,kg & -- & %.0f--%.0f & kW' % tuple(need))
    lo_s, hi_s = min(PB.CRUISE_KMH.values()), max(PB.CRUISE_KMH.values())
    num('PcruiseLo', '%.0f' % pv(lo_s)); num('PcruiseHi', '%.0f' % pv(hi_s))
    rows.append('Power at the published cruise speeds, %.0f--%.0f\\,km/h & -- & %.0f--%.0f & kW' % (lo_s, hi_s, pv(lo_s), pv(hi_s)))
    v_chart = cross(V[4:], P[4:], HOV.POWER_USED_KW)
    num('VmaxAtChartKmh', '%.0f' % (v_chart * KT_KMH))
    rows.append('Level speed on %.0f\\,kW & %.0f--%.0f (cruise) & %.0f & km/h' % (HOV.POWER_USED_KW, lo_s, hi_s, v_chart * KT_KMH))
    # how high the 2200 kg hover out of ground effect trims, and what stops it there
    lim = h['limit']
    at, sea = lim['at_limit'], lim['sea']
    spare = lambda q: 100 * (q['peak_lb'] / q['need_lb'] - 1)
    assert spare(at) < 4 < 40 < spare(sea) and at['tr_alpha'] > at['mr_alpha'], \
        'the text says the tail rotor stops the high hover: %.1f %% to spare there, %.1f %% at sea level' % (spare(at), spare(sea))
    assert lim['kw'] > lim['avail_kw'], 'the text says that is past the engine\'s power'
    num('HovLimitAlt', '%.0f' % (100 * round(lim['alt_m'] / 100))); num('HovLimitKw', '%.0f' % lim['kw'])
    num('HovLimitAvail', '%.0f' % lim['avail_kw'])
    num('HovLimitMr', '%.0f' % at['mr_alpha']); num('HovLimitTr', '%.0f' % at['tr_alpha'])
    num('HovLimitSpare', '%.1f' % max(spare(at), 0.0)); num('HovSeaSpare', '%.0f' % spare(sea))
    num('HovLimitPeakN', '%.0f' % (at['peak_lb'] * 4.448222))
    a = R('autorotation')['points']
    best = min(a, key=lambda p: -p['fpm'])
    rows.append('Autorotation, least rate of descent, 2200\\,kg & about %.0f (AO) & %.0f at %d\\,kt & ft/min' % (PB.AUTOROTATION_FPM['AO'], -best['fpm'], best['V']))
    num('AutoRod', '%.0f' % -best['fpm']); num('AutoV', '%d' % best['V'])
    write('perf', '\\begin{tabularx}{\\textwidth}{@{}X l l l@{}}\n\\toprule\nQuantity (sea level, standard day) & Published & Model & Unit \\\\\n\\midrule\n'
          + ''.join(r + ' \\\\\n' for r in rows) + '\\bottomrule\n\\end{tabularx}\n')


# ---------------------------------------------------------------- the hover charts: the hover correction, ground effect
def span(name, values, fmt):
    num(name + 'Lo', min(values), fmt); num(name + 'Hi', max(values), fmt)


def inflow_table():
    d = R('hoverinflow')
    P0 = HOV.POWER_USED_KW
    num('ChartKw', '%.0f' % P0)
    pairs = list(zip(d['with'], d['without']))
    std = [(w, o) for w, o in pairs if w['temp_c'] is None]
    off = [(w, o) for w, o in pairs if w['temp_c'] is not None]
    rows = ['%d & %.0f & %.1f & %.1f' % (w['h_m'], w['kg'], o['kw'], w['kw']) for w, o in std]
    mid = '\\midrule\n\\multicolumn{4}{@{}l}{\\textit{off the standard atmosphere (pressure altitude)}} \\\\\n'
    foot = mid + ''.join('%d, %s$^\\circ$C & %.0f & %.1f & %.1f \\\\\n' % (w['h_m'], f(w['temp_c'], '%+d'), w['kg'], o['kw'], w['kw'])
                         for w, o in off)
    write('inflow', tabular('lrrr', 'Altitude, m & Chart kg & Without, kW & With, kW', rows, foot))
    first, last = std[0], std[-1]
    assert (first[0]['h_m'], first[0]['kg']) == HOV.FIG35_OGE_LOW[0], 'the first point is the one the strength is fitted to'
    num('FitAltM', '%d' % first[0]['h_m']); num('FitKg', '%.0f' % first[0]['kg'])
    num('TopAltM', '%d' % last[0]['h_m'])
    num('WoFit', '%.0f' % first[1]['kw']); num('WoTop', '%.0f' % last[1]['kw']); num('WTop', '%.0f' % last[0]['kw'])
    num('WoFitPct', '%.0f' % (100 * (1 - first[1]['kw'] / P0))); num('WoTopPct', '%.0f' % (100 * (1 - last[1]['kw'] / P0)))
    num('WTopShort', '%.0f' % (P0 - last[0]['kw'])); num('WTopPct', '%.0f' % (100 * (1 - last[0]['kw'] / P0)))
    span('OffShort', [P0 - w['kw'] for w, o in off], '%.0f')
    s, p = d['shape'], d['pitch']
    num('ViStrength', '%.4f' % U.HOVER_VI); num('ViMach', '%.3f' % U.HOVER_VI_MACH); num('ViMachZero', '%.2f' % U.HOVER_VI_M0)
    num('BoLevel', '%.3f' % s['level']); num('BoLevelErr', '%.3f' % s['err'][0])
    num('BoMachSlope', '%.3f' % s['mach']); num('BoMachErr', '%.3f' % s['err'][2])
    num('BoLoadSlope', '%+.3f' % s['loading']); num('BoLoadErr', '%.3f' % s['err'][1]); num('BoRms', '%.3f' % s['rms'])
    span('BoSens', [q['level'] for q in d['sensitivity']], '%.3f')
    span('BoSensMach', [q['mach_ratio'] for q in d['sensitivity']], '%.2f')
    num('BoPitchWith', '%.2f' % p['rms_with']); num('BoPitchWithout', '%.2f' % p['rms_without'])
    num('BoPoints', '%d' % len(s['points']))
    # TM-85892's own power at its one rotor speed, against the chart's fit there (where the thrust is as in the fit)
    fit = lambda cts: s['level'] + s['loading'] * (cts - 0.08) / 0.01 + p['eps'] - s['level']
    ratio = [q['eps_power'] / fit(q['cts']) for q in p['points'] if q['cts'] > 0.06]
    num('BoTablePct', '%.0f' % (100 * (1 - float(np.mean(ratio)))))


def ge_fit_table():
    d = R('hovercharts')
    a, b = d['after'], d['before']
    num('GeK', '%.3f' % a['ge_k']); num('GeKBefore', '%.3f' % b['ge_k']); num('GeWheels', '%g' % a['wheels_m'])
    rows = ['%d & %.3f & %.3f & %.3f' % (p['h_m'], p['ratio_chart'], q['ratio'], p['ratio']) for p, q in zip(a['fig35'], b['fig35'])]
    mid = '\\midrule\n\\multicolumn{4}{@{}l}{\\textit{off the standard atmosphere, not fitted}} \\\\\n'
    foot = mid + ''.join('%d, %s$^\\circ$C & %.3f & %.3f & %.3f \\\\\n' % (p['h_m'], f(p['temp_c'], '%+d'), p['ratio_chart'], q['ratio'], p['ratio'])
                         for p, q in zip(a['off_isa'], b['off_isa']))
    write('gefit', tabular('lrrr', 'Altitude, m & Chart & $K_{GE}$ %.3f & %.3f' % (b['ge_k'], a['ge_k']), rows, foot))
    span('GeChart', [p['ratio_chart'] for p in a['fig35']], '%.3f'); span('GeModel', [p['ratio'] for p in a['fig35']], '%.3f')
    span('GeWas', [p['ratio'] for p in b['fig35']], '%.3f')
    span('GeOffChart', [p['ratio_chart'] for p in a['off_isa']], '%.3f'); span('GeOffModel', [p['ratio'] for p in a['off_isa']], '%.3f')
    num('GeOffWorst', '%.3f' % max(abs(p['ratio'] - p['ratio_chart']) for p in a['off_isa']))
    fits = {w['wheels_m']: w['ge_k'] for w in d['fit']}
    num('GeKHalf', '%.3f' % fits[0.5]); num('GeKOneHalf', '%.3f' % fits[1.5]); num('GeKTwo', '%.3f' % fits[2.0])
    for p, q, tag in zip(a['gain'], b['gain'], ('Heavy', 'Next')):
        assert p['kg'] == {'Heavy': 2200, 'Next': 2100}[tag]
        num('GainChart' + tag, '%.0f' % (p['h_ige_chart'] - p['h_oge']))
        num('GainModel' + tag, '%.0f' % (p['h_ige'] - p['h_oge'])); num('GainWas' + tag, '%.0f' % (q['h_ige'] - q['h_oge']))
    oge = {c['kg']: c for c in d['oge']['ceiling']}
    ige = {c['kg']: c for c in a['ceiling_ige']}
    num('CeilOge', '%.0f' % oge[2200]['h']); num('CeilIge', '%.0f' % ige[2200]['h'])


def gauge_numbers():
    """the cockpit's pitch indicator in the hover at 2200 kg, sea level, against the manual's hover setting: the cruise
    charts' pitch there plus the 0.03 of figs. 3-6 and 3-7. The indicator's pitch at 0.7 R for 0.6 and 0.7 on its dial is
    read from the aircraft's own scale (Instruments/Afficheur.xml), linear from there up"""
    from data import al3_cruise_charts as CRZ
    s = open(os.path.join(FDM, '..', 'Alouette-3', 'Instruments', 'Afficheur.xml')).read()
    at = {int(pct): float(rad) for rad, pct in re.findall(r'<ind>([\d.]+)</ind><!--(\d+)%:', s)}
    x = math.radians(R('trims_2200')[0]['coll07'])
    num('GaugeModel', 0.6 + (x - at[60]) / (at[70] - at[60]) * 0.1, '%.2f')
    num('GaugeManual', CRZ.pitch(0.0) + 0.03, '%.2f')


# ---------------------------------------------------------------- cruise fuel against the charts
def cruise_table():
    d = R('cruisecharts')
    ok = [p for p in d['points'] if 'error' not in p]
    alts = sorted({p['alt_m'] for p in ok})
    both = lambda q, key: ('%.0f' % q[0][key] if len(q) == 1 else
                           '%.0f--%.0f' % (min(p[key] for p in q), max(p[key] for p in q)))
    rows, pitch = [], {}
    from data import al3_cruise_charts as CRZ
    for alt in alts:
        q = [p for p in ok if p['alt_m'] == alt]
        rows.append('%d & %s & %.2f & %s & %s & %s & %s' % (alt, ', '.join('%d' % p['kg'] for p in q) if len(q) < 4 else 'all four',
                    CRZ.pitch(alt), both(q, 'chart_lh'), both(q, 'model_lh'), both(q, 'model_kw'), both(q, 'chart_kw_on_model_curve')))
    write('cruise', '\\begin{tabular}{@{}r l c r r r r@{}}\n\\toprule\n & & & \\multicolumn{2}{c}{Fuel, L/h} & \\multicolumn{2}{c}{Power, kW} \\\\\n'
          'Altitude, m & Weights, kg & Pitch & chart & model & model\'s flight & chart\'s fuel \\\\\n\\midrule\n'
          + ''.join(r + ' \\\\\n' for r in rows) + '\\bottomrule\n\\end{tabular}\n')
    less = lambda p: 100 * (1 - p['model_lh'] / p['chart_lh'])
    num('CrN', '%d' % len(d['points'])); num('CrFlown', '%d' % len(ok))
    span('CrLess', [less(p) for p in ok], '%.1f')
    sea = [p for p in ok if p['alt_m'] == 0]
    span('CrSeaChart', [p['chart_lh'] for p in sea], '%.0f'); span('CrSeaModel', [p['model_lh'] for p in sea], '%.0f')
    span('CrSeaLess', [less(p) for p in sea], '%.0f'); span('CrSeaKw', [p['model_kw'] for p in sea], '%.0f')
    span('CrSeaChartKw', [p['chart_kw_on_model_curve'] for p in sea], '%.0f')
    span('CrSeaSfc', [p['sfc_for_chart'] for p in sea], '%.2f')
    num('Sfc', '%.2f' % U.SFC)
    # the order of the weights at 2000 m, heaviest less lightest: no engine curve changes it (higher up, the heaviest
    # can't be flown)
    chart, power = [], []
    for alt in (2000,):
        q = sorted((p for p in ok if p['alt_m'] == alt), key=lambda p: p['kg'])
        assert (q[0]['kg'], q[-1]['kg']) == (1600, 2200)
        chart.append(100 * (q[-1]['chart_lh'] / q[0]['chart_lh'] - 1))
        power.append(-100 * (q[-1]['model_kw'] / q[0]['model_kw'] - 1))
    span('CrWeightChart', chart, '%.0f'); span('CrWeightModel', power, '%.0f')
    failed = sorted((p['kg'], p['alt_m']) for p in d['points'] if 'error' in p)
    assert failed == [(1800, 4000), (2000, 3000), (2000, 4000), (2200, 3000)], 'the text names the points the model cannot fly: %s' % failed


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
    # the hover's modes the text names, by what they move: velocities in m/s, rates on a 10 m arm, angles as the speed
    # gravity gives them in a second
    names, weight = 'u v w p q r phi theta'.split(), [1, 1, 1, 10, 10, 10, 9.81, 9.81]
    def parts(vec):
        a = [abs(complex(*c)) * w for c, w in zip(vec, weight)]
        return dict(zip(names, [q / max(a) for q in a]))
    hov = [(complex(*e), parts(v)) for e, v in zip(d[0]['eig'], d[0]['vec'])]
    reals = [q for q in hov if abs(q[0].imag) < 1e-9]
    roll = min(reals, key=lambda q: q[0].real)
    div = [q for q in reals if q[0].real > 0]
    osc = [q for q in hov if q[0].imag > 1e-9 and q[0].real > 0]
    assert len(div) == 1 and len(osc) == 1, 'the text names one growing real mode and one growing oscillation'
    top = lambda q, n: sorted(q[1], key=q[1].get)[::-1][:n]
    assert top(roll, 1) == ['p'] and top(div[0], 1) == ['u'] and div[0][1]['theta'] > 0.3 and set(top(osc[0], 2)) == {'v', 'r'}, \
        'the hover modes are no longer what the text says: %s %s %s' % (top(roll, 3), top(div[0], 3), top(osc[0], 3))
    num('ModeRollTau', '%.2f' % (-1 / roll[0].real)); num('ModeDivDouble', '%.1f' % (math.log(2) / div[0][0].real))
    num('ModeOscPeriod', '%.0f' % (2 * math.pi / osc[0][0].imag)); num('ModeOscDouble', '%.0f' % (math.log(2) / osc[0][0].real))
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
    num('GovSwing', '%.2f' % g['swing_pct']); num('GovKwA', '%.0f' % g['kw0']); num('GovKwB', '%.0f' % g['kw1'])
    assert abs(g['nr_end'] - g['nr0']) < 0.1 and g['pw_peak'] < U.P_MAX_KW, 'the text says the governor holds the speed: %s' % g
    o = e['over_limit']['events']
    assert abs(o['pw_end'] - U.P_MAX_KW) < 0.5 and o['pw_peak'] < U.P_MAX_KW + 0.5 and o['nr_end_change'] < 0.2 and \
        abs(o['load_end'] - o['pw_end']) < 1.0, 'the text says the engine holds 420 kW and the rotor settles there: %s' % o
    num('CapKw', '%.0f' % o['pw_end']); num('CapNr', '%.0f' % o['nr_end']); num('CapSettle', '%.0f' % o['settle_end_t'])
    num('CapFromKw', '%.0f' % o['kw0'])
    fl = e['failure_held']['events']
    num('FailTthreeten', '%.1f' % fl['t_310']); num('FailTtwoseventy', '%.1f' % fl['t_270'])


def step_numbers():
    """the collective's step response asks for more than the engine's power: the rotor's speed 3 s into it"""
    rec = R('steps')['coll']
    assert rec[-1]['t'] >= 2.9 and rec[-1]['nr'] < 350, 'the text says the rotor slows in the collective step: %s' % rec[-1]
    num('StepCollNr', '%.0f' % rec[-1]['nr'])


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


def autorotation_floor():
    """the slowest steady autorotation on the descents (descent.py: the horizontal speed held, the sink at which the
    engine's power is nil), and the least power the engine must give at 30 kt airspeed on them"""
    d = R('descent')
    found, never, at30 = [], [], []
    for sp in d['speeds']:
        u, rows = sp['kt'] * 1.6878, sp['rows']
        for a, b in zip(rows, rows[1:]):
            if a['power_kw'] > 0 >= b['power_kw']:
                s = a['sink_fps'] + a['power_kw'] / (a['power_kw'] - b['power_kw']) * (b['sink_fps'] - a['sink_fps'])
                found.append((math.hypot(u, s) / 1.6878, math.degrees(math.atan2(s, u))))
                break
        else:
            never.append(sp['kt'])
        if u <= 30 * 1.6878:
            s = math.sqrt((30 * 1.6878) ** 2 - u ** 2)
            assert s <= rows[-1]['sink_fps'], 'the %d kt descents stop short of 30 kt airspeed' % sp['kt']
            at30.append(float(np.interp(s, [r['sink_fps'] for r in rows], [r['power_kw'] for r in rows])))
    slow = min(found)
    assert never == [0, 10] and slow[0] > 30 and min(at30) > 0, \
        'the text says there is no steady autorotation at 30 kt: %s, never %s, at 30 kt %s' % (found, never, at30)
    num('AutoSlowKt', '%.0f' % slow[0]); num('AutoSlowPath', '%.0f' % slow[1]); num('AutoThirtyKw', '%.0f' % min(at30))


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
    # the flight's heights, as flown (flights.mission's defaults), its most power, and its lowest point in the recovery
    import inspect, flights
    d = inspect.signature(flights.mission).parameters
    num('MiCruiseFt', '%d' % d['cruise_ft'].default); num('MiRecoverFt', '%d' % d['recover_ft'].default)
    back = ph('power on') + ph('recovery climb')
    low = min(r['h'] for r in back)
    out = min(r['nr'] for r in rec if r.get('phase') != 'autorotation')
    peak = max(r['kw'] for r in rec)
    assert low > 50 and out > 350 and peak < U.P_MAX_KW + 5, \
        'the text says the flight keeps within the power and clear of the ground: %.0f ft, %.1f rpm, %.0f kW' % (low, out, peak)
    num('MiRecoveryLow', '%.0f' % (10 * round(low / 10))); num('MiNrOutside', '%.0f' % out); num('MiPeakKw', '%.0f' % peak)


if __name__ == '__main__':
    os.makedirs(TAB, exist_ok=True)
    for fn in (model_table, params_table, perf_table, inflow_table, ge_fit_table, cruise_table, gauge_numbers, derivs_table,
               engine_numbers, step_numbers, ge_table, vrs_numbers, autorotation_floor, mission_numbers):
        try:
            fn()
        except FileNotFoundError as e:
            print(fn.__name__, 'skipped:', e)
    open(os.path.join(TAB, 'numbers.tex'), 'w').write(''.join('\\newcommand{\\N%s}{%s}\n' % (k, v) for k, v in sorted(NUM.items())))
    print(len(NUM), 'numbers')
