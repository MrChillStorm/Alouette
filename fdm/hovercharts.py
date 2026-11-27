#!/usr/bin/env python3
"""Hover performance against the SA 316B's published hover charts (data/al3_hover_charts.py), in and out of ground
effect, and the fit of the ground effect to them.

Below fig. 3-5's knee the charts' hover ceilings in ground effect and out of it are flown on the same power (the main
gearbox's limit, "390 kW approx."), and above it on the same collective pitch. The ground effect is therefore checked at
equal power, which takes the model's own power out of ground effect out of the comparison: at each point of the chart,
the power the model needs to hover out of ground effect at the chart's weight, and the weight it hovers in ground effect
on that same power, against the chart's weight in ground effect. Whether that power is right is a question of its own,
reported apart; the ground effect is not fitted to it.

  python3 hovercharts.py            the comparison with the ground effect before the fit (GE_K_BEFORE, the Bo 105's and
                                    UH-60's) and the model's (al3data.GE_K); writes out/hovercharts.json, .md and .png,
                                    and exits 1 if the model's ground effect misses the tolerances (TOL)
  python3 hovercharts.py --fit      also refit GE_K to fig. 3-5, at the wheel height WHEELS_M and at others

The test models are generated into a temporary folder; nothing in the aircraft is touched. A few minutes on 4 cores.
"""
import argparse, contextlib, ctypes, json, math, os, subprocess, sys, tempfile
from multiprocessing import Pool
import numpy as np
import al3data as U
import harness as H
from atmosphere import isa
from performance import HOVER_WHEELS_M as WHEELS_M     # in ground effect: the wheels this high (the charts don't say)
from data import al3_hover_charts as C

HERE = os.path.dirname(os.path.abspath(__file__))
GE_K_BEFORE = 0.174
HEIGHTS_M = [0.5, 1.0, 1.5, 2.0]
WORKERS = 4
# the model's ground effect passes within these, as weight in ground effect / out of it on the same power (a chart
# reading is good to about +-0.005): fig. 3-5's points each and their mean, and figs. 3-6 and 3-7's off the standard
# atmosphere each
TOL = {'fig35': 0.007, 'fig35_mean': 0.002, 'off_isa': 0.010}
X0 = [0.6, -0.38, -0.08, 0.2, 0.1, 4.0]                  # a hover, for the first trim
SCALE = np.array([1, 1, 1, 10, 10, 10.0])                # harness.trim's weighting of the accelerations
TRIM_TOL = 0.002                                         # tighter than the trims' usual 0.02: powers to about 0.03 kW

_models = {}


@contextlib.contextmanager
def quiet():
    """JSBSim's banner and notes go to the process's own output: send them nowhere"""
    sys.stdout.flush(); sys.stderr.flush()
    keep = os.dup(1), os.dup(2)
    null = os.open(os.devnull, os.O_WRONLY)
    os.dup2(null, 1); os.dup2(null, 2)
    try:
        yield
    finally:
        sys.stdout.flush(); sys.stderr.flush()        # Python's and C's buffers, which would else empty after this
        ctypes.CDLL(None).fflush(None)
        os.dup2(keep[0], 1); os.dup2(keep[1], 2)
        for fd in keep + (null,):
            os.close(fd)


def build(ge_k, tmp):
    """a test model with the ground effect's constant ge_k, in a process of its own (the generator keeps module state)"""
    root = os.path.join(tmp, 'ge%.5f' % ge_k)
    if not os.path.exists(root):
        code = ('import sys; sys.path.insert(0, %r); import al3data as U; U.GE_K = %r; import genal3; genal3.build(%r)'
                % (HERE, float(ge_k), os.path.join(root, 'aircraft', 'alouette3')))
        subprocess.run([sys.executable, '-c', code], check=True, cwd=HERE, stdout=subprocess.DEVNULL)
    return root


def model(root):
    if root not in _models:
        with quiet():
            f = H.load(root=root, model='al3-2200')
            f.run_ic()
        _models[root] = f
    return _models[root]


def air(temp_c, pa_m):
    """(altitude ft, atmosphere/delta-T degR) at which JSBSim's air is at temp_c and the pressure altitude pa_m (its
    temperature bias moves its pressure with it); temp_c None: the standard atmosphere, pa_m then also the density
    altitude"""
    if temp_c is None:
        return pa_m / 0.3048, 0.0
    T = temp_c + 273.15
    Ts = T / (isa(pa_m)[1] / 101325.0) ** (1 / 5.25588)      # the sea level temperature of that column
    return (Ts - T) / 0.0065 / 0.3048, (Ts - 288.15) * 1.8


def hover(root, kg, h_m, wheels_m=None, temp_c=None, x0=None):
    """trim a hover: engine shaft power (rotors, gearbox, accessories) kW, the controls, the trim"""
    f = model(root)
    alt_ft, dT = air(temp_c, h_m)
    f['atmosphere/delta-T'] = dT
    H.set_weight(f, kg)
    terr = None if wheels_m is None else alt_ft - H.wheels_above_ground_ft(f) - wheels_m / 0.3048
    x, r, e = H.trim(f, 0.0, alt_ft=alt_ft, x0=X0 if x0 is None else x0, extra=H.ENGINE_POWER, terrain_ft=terr,
                     tol=TRIM_TOL, iters=24)
    cost = float(np.abs(r * SCALE).max())
    if not cost <= 0.1:                                  # (a trim gone to NaN too)
        _models.pop(root, None)                          # it can leave the rotor's states wild: the next starts afresh
        raise RuntimeError('no hover trim: %.0f kg, %.0f m, wheels %s m (residual %.2f)' % (kg, h_m, wheels_m, cost))
    if temp_c is not None:
        assert abs(f['atmosphere/T-R'] / 1.8 - 273.15 - temp_c) < 0.1 and \
            abs(f['atmosphere/P-psf'] * 47.880259 / isa(h_m)[1] - 1) < 0.002, 'the atmosphere is not the one asked for'
    return e[0] * e[1] / 737.562, x


def secant(fun, a, b, target, tol, max_step, iters=14):
    """the argument at which fun (returning value, controls; rising or falling) meets target within tol, by the secant
    method from a and b with steps of at most max_step; returns it with fun's result"""
    fa, xa = fun(a, None)
    if abs(fa - target) < tol:
        return a, fa, xa
    for _ in range(iters):
        fb, xb = fun(b, xa)
        if abs(fb - target) < tol:
            return b, fb, xb
        if fb == fa:
            break
        step = (target - fb) * (b - a) / (fb - fa)
        a, b, fa, xa = b, b + max(-max_step, min(max_step, step)), fb, xb
    raise RuntimeError('no convergence to %g' % target)


def equal_power(job):
    """one chart point: the model's power out of ground effect at the chart's weight there; the weight it hovers in
    ground effect on that power; and its power and collective in ground effect at the chart's weight"""
    root, wheels, temp_c, h_m, w_oge, w_ige = job
    p_oge, x_oge = hover(root, w_oge, h_m, None, temp_c)
    p_pub, x_pub = hover(root, w_ige, h_m, wheels, temp_c, x_oge)
    dw = w_ige * ((p_oge / p_pub) ** 0.75 - 1)            # the hover's power grows about as weight to the 1.3
    w, _, _ = secant(lambda w, x: hover(root, w, h_m, wheels, temp_c, x_pub if x is None else x),
                     w_ige, w_ige + math.copysign(max(abs(dw), 15.0), dw), p_oge, 0.05, 120.0)
    return dict(temp_c=temp_c, h_m=h_m, w_oge=w_oge, w_ige=w_ige, p_oge=p_oge, p_ige_at_w_ige=p_pub, w_model=w,
                ratio_chart=w_ige / w_oge, ratio=w / w_oge,
                coll_oge=H.deg(x_oge)[0], coll_ige_at_w_ige=H.deg(x_pub)[0])


def gain(job):
    """the table's ceilings: the altitude gained in ground effect, on the power the model needs at the chart's ceiling
    out of it"""
    root, wheels, kg, h_oge, h_ige = job
    p, x = hover(root, kg, h_oge)
    h, _, _ = secant(lambda h, x0: hover(root, kg, h, wheels, None, x if x0 is None else x0), h_ige, h_ige - 300, p, 0.05,
                     800.0)
    return dict(kg=kg, h_oge=h_oge, h_ige_chart=h_ige, h_ige=h, p_oge=p)


def ceiling(job):
    """the model's hover ceiling on the charts' power (standard atmosphere): from h0 down in steps while the hover needs
    more there, then up until it does, then the secant method between the last two. Where the trim fails on the way up
    (the rotor stalls), h is None, with the highest altitude it holds (to 60 m) and the power there"""
    root, wheels, kg, h0 = job
    h = float(h0)
    p, x = hover(root, kg, h, wheels)
    while p >= C.POWER_USED_KW:
        if h <= -1000:
            return dict(kg=kg, wheels=wheels, h=None, h_top=None, p_top=p)
        h -= 500
        p, x = hover(root, kg, h, wheels, None, x)
    step, last = 500.0, (h, p)
    h += step
    while True:
        try:
            p, xn = hover(root, kg, h, wheels, None, x)
        except RuntimeError:
            if step < 60:
                return dict(kg=kg, wheels=wheels, h=None, h_top=last[0], p_top=last[1])
            step /= 2
            h = last[0] + step
            continue
        if p >= C.POWER_USED_KW:
            break
        last, x = (h, p), xn
        h += step
    (a, pa), b, pb = last, h, p
    for _ in range(20):                                  # between them: the secant method, kept bracketed
        c = a + (C.POWER_USED_KW - pa) * (b - a) / (pb - pa)
        pc, x = hover(root, kg, c, wheels, None, x)
        if abs(pc - C.POWER_USED_KW) < 0.05:
            break
        if pc < C.POWER_USED_KW:
            a, pa = c, pc
        else:
            b, pb = c, pc
    return dict(kg=kg, wheels=wheels, h=c)


def oge_power(job):
    root, h_m, kg = job
    return dict(h_m=h_m, kg=kg, p=hover(root, kg, h_m)[0])


def points35(root, wheels):
    return [(root, wheels, None, h, wo, wi) for h, wo, wi in C.FIG35_AREA1]


def points67(root, wheels):
    return [(root, wheels, T, pa, C.chart_weight(C.FIG36_OGE, T, pa), C.chart_weight(C.FIG37_IGE, T, pa))
            for T, pa in C.OFF_ISA_POINTS]


def mean_error(pool, ge_k, wheels, tmp):
    res = pool.map(equal_power, points35(build(ge_k, tmp), wheels))
    return float(np.mean([r['ratio'] - r['ratio_chart'] for r in res]))


def fit(pool, wheels, tmp):
    """GE_K for which the mean of fig. 3-5's ratios (area 1) is the chart's, by the secant method"""
    k0, k1 = 0.17, 0.10
    e0, e1 = mean_error(pool, k0, wheels, tmp), mean_error(pool, k1, wheels, tmp)
    for _ in range(8):
        k2 = k1 - e1 * (k1 - k0) / (e1 - e0)
        k0, e0, k1 = k1, e1, round(min(max(k2, 0.02), 0.6), 5)
        e1 = mean_error(pool, k1, wheels, tmp)
        if abs(e1) < 2e-4:
            break
    res = pool.map(equal_power, points35(build(k1, tmp), wheels))
    dev = [r['ratio'] - r['ratio_chart'] for r in res]
    return dict(wheels_m=wheels, ge_k=k1, mean=float(np.mean(dev)), worst=float(max(dev, key=abs)))


def compare(pool, ge_k, wheels, tmp):
    root = build(ge_k, tmp)
    out = dict(ge_k=ge_k, wheels_m=wheels)
    out['fig35'] = pool.map(equal_power, points35(root, wheels))
    out['off_isa'] = pool.map(equal_power, points67(root, wheels))
    out['pitch'] = pool.map(equal_power, [(root, wheels, None, h, wo, wi) for h, wo, wi in C.FIG35_AREA2])
    # the table's 1900 kg ceiling in ground effect (4500 m) is just above the knee: pitch limited
    out['gain'] = pool.map(gain, [(root, wheels, kg, C.TABLE[kg][1], C.TABLE[kg][0]) for kg in (2200, 2100)])
    out['ceiling_ige'] = pool.map(ceiling, [(root, wheels, kg, C.TABLE[kg][0]) for kg in (2200, 2100, 1900)])
    return out


def verdict(r):
    d35 = [p['ratio'] - p['ratio_chart'] for p in r['fig35']]
    d67 = [p['ratio'] - p['ratio_chart'] for p in r['off_isa']]
    checks = [('fig. 3-5, each point', max(map(abs, d35)), TOL['fig35']),
              ('fig. 3-5, mean', abs(float(np.mean(d35))), TOL['fig35_mean']),
              ('figs. 3-6 and 3-7, each point', max(map(abs, d67)), TOL['off_isa'])]
    return [dict(check=n, error=e, tolerance=t, ok=e <= t) for n, e, t in checks]


def table(head, rows):
    return '\n'.join(['| ' + ' | '.join(head) + ' |', '|' + '---|' * len(head)] +
                     ['| ' + ' | '.join(str(c) for c in row) + ' |' for row in rows])


def report(res):
    b, a, oge = res['before'], res['after'], res['oge']
    m = lambda c: ('%.0f' % c['h'] if c['h'] is not None else 'below -1000' if c['h_top'] is None
                   else 'stalls above %.0f (%.0f kW)' % (c['h_top'], c['p_top']))
    s = ['# The model against the SA 316B\'s hover charts', '', C.SOURCE + '.',
         'In ground effect: the wheels %g m above the ground. Before: GE_K %g; after: GE_K %g.'
         % (a['wheels_m'], b['ge_k'], a['ge_k']), '',
         '## Ground effect: weight in it / weight out of it, on the same power (fig. 3-5)', '',
         table(['density altitude m', 'chart out kg', 'chart in kg', 'chart ratio', 'before: in kg', 'before ratio',
                'after: in kg', 'after ratio'],
               [[p['h_m'], '%.0f' % p['w_oge'], '%.0f' % p['w_ige'], '%.3f' % p['ratio_chart'], '%.0f' % q['w_model'],
                 '%.3f' % q['ratio'], '%.0f' % p['w_model'], '%.3f' % p['ratio']] for p, q in zip(a['fig35'], b['fig35'])]),
         '', '## Off the standard atmosphere (figs. 3-6 and 3-7), not fitted', '',
         table(['outside temperature C', 'pressure altitude m', 'chart ratio', 'before', 'after'],
               [[p['temp_c'], p['h_m'], '%.3f' % p['ratio_chart'], '%.3f' % q['ratio'], '%.3f' % p['ratio']]
                for p, q in zip(a['off_isa'], b['off_isa'])]),
         '', '## The table\'s ceilings (p. 1.2): altitude gained in ground effect, on the same power', '',
         table(['kg', 'chart out m', 'chart in m', 'chart gain m', 'before gain m', 'after gain m'],
               [[p['kg'], '%.0f' % p['h_oge'], '%.0f' % p['h_ige_chart'], '%.0f' % (p['h_ige_chart'] - p['h_oge']),
                 '%.0f' % (q['h_ige'] - q['h_oge']), '%.0f' % (p['h_ige'] - p['h_oge'])]
                for p, q in zip(a['gain'], b['gain'])]),
         '', '## Above the knee, on the pitch limit: collective in ground effect less out of it, deg (chart: 0)', '',
         table(['density altitude m', 'before', 'after'],
               [[p['h_m'], '%+.2f' % (q['coll_ige_at_w_ige'] - q['coll_oge']), '%+.2f' % (p['coll_ige_at_w_ige'] - p['coll_oge'])]
                for p, q in zip(a['pitch'], b['pitch'])]),
         '', '## Not fitted: power out of ground effect at the chart\'s ceilings (chart: %.0f kW)' % C.POWER_USED_KW, '',
         table(['density altitude m', 'chart kg', 'model kW', 'model / chart power'],
               [[p['h_m'], '%.0f' % p['kg'], '%.1f' % p['p'], '%.3f' % (p['p'] / C.POWER_USED_KW)] for p in oge['power']]),
         '', '## Not fitted: hover ceilings on %.0f kW (standard atmosphere), m' % C.POWER_USED_KW, '',
         table(['kg', 'chart out', 'model out', 'chart in', 'before in', 'after in'],
               [[p['kg'], '%.0f' % C.TABLE[p['kg']][1], m(o), '%.0f' % C.TABLE[p['kg']][0], m(q), m(p)]
                for p, q, o in zip(a['ceiling_ige'], b['ceiling_ige'], oge['ceiling'])]),
         '', '## Checks of the model\'s ground effect', '',
         table(['check', 'worst error in the ratio', 'tolerance', ''],
               [[c['check'], '%.4f' % c['error'], '%.3f' % c['tolerance'], 'pass' if c['ok'] else 'FAIL'] for c in res['checks']])]
    if 'fit' in res:
        s += ['', '## GE_K fitted to fig. 3-5 at other wheel heights', '',
              table(['wheels above the ground m', 'GE_K', 'mean error', 'worst error'],
                    [[f['wheels_m'], '%.4f' % f['ge_k'], '%+.4f' % f['mean'], '%+.4f' % f['worst']] for f in res['fit']]),
              '', '## The model\'s ground effect (GE_K %g) with the wheels at other heights: mean ratio, fig. 3-5' % a['ge_k'], '',
              table(['wheels above the ground m', 'model', 'chart'],
                    [[f['wheels_m'], '%.3f' % f['ratio'], '%.3f' % f['chart']] for f in res['heights']])]
    return '\n'.join(s) + '\n'


def plot(res, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    ink, ink2, grid, old, new = '#0b0b0b', '#52514e', '#e4e3df', '#eb6834', '#2a78d6'
    plt.rcParams.update({'font.size': 9.5, 'axes.edgecolor': grid, 'axes.labelcolor': ink2, 'xtick.color': ink2,
                         'ytick.color': ink2, 'axes.titlecolor': ink, 'figure.facecolor': '#fcfcfb',
                         'axes.facecolor': '#fcfcfb', 'axes.titlesize': 10.5, 'axes.titlelocation': 'left'})
    fig, ax = plt.subplots(1, 3, figsize=(13.5, 5.2), gridspec_kw=dict(wspace=0.28))
    b, a, oge = res['before'], res['after'], res['oge']
    h = [p['h_m'] for p in a['fig35']]
    oge35 = C.FIG35_OGE_LOW + [(x, wo) for x, wo, wi in C.FIG35_AREA1]
    ax[0].plot([w for _, w in oge35], [x for x, _ in oge35], color=ink, lw=2, ls=(0, (5, 2)), label='chart, out of ground effect')
    ax[0].plot([p['w_ige'] for p in a['fig35']], h, color=ink, lw=2, label='chart, in ground effect')
    ax[0].plot([p['w_model'] for p in b['fig35']], h, 'o', ms=8, color=old, mec='#fcfcfb', mew=1.5, label='model in it, before')
    ax[0].plot([p['w_model'] for p in a['fig35']], h, 's', ms=8, color=new, mec='#fcfcfb', mew=1.5, label='model in it, after')
    ax[0].set_xlabel('weight, kg'); ax[0].set_ylabel('density altitude, m')
    ax[0].set_title('Hover on the same power (fig. 3-5)')
    ax[0].legend(frameon=False, loc='upper right', fontsize=8.5, labelcolor=ink)
    ax[0].set_ylim(0, 5600)
    ax[1].plot([p['ratio_chart'] for p in a['fig35']], h, color=ink, lw=2, label='chart, fig. 3-5')
    for r, col, mk, name in ((b, old, 'o', 'before'), (a, new, 's', 'after')):
        ax[1].plot([p['ratio'] for p in r['fig35']], h, mk + '-', ms=8, lw=2, color=col, mec='#fcfcfb', mew=1.5, label='model, ' + name)
        ax[1].plot([p['ratio'] for p in r['off_isa']], [p['h_m'] for p in r['off_isa']], mk, ms=7, mfc='none', mec=col, mew=1.5,
                   label='model, %s, off standard temperature' % name)
    ax[1].plot([p['ratio_chart'] for p in a['off_isa']], [p['h_m'] for p in a['off_isa']], 'x', ms=7, mew=1.8, color=ink,
               label='chart, figs. 3-6, 3-7')
    ax[1].set_xlabel('weight in ground effect / out of it'); ax[1].set_ylabel('density or pressure altitude, m')
    ax[1].set_title('Ground effect, wheels at %g m' % a['wheels_m'])
    lo = min(p['ratio_chart'] for p in a['fig35'] + a['off_isa'])
    hi = max(p['ratio'] for p in b['fig35'] + b['off_isa'])
    ax[1].set_xlim(lo - 0.006, hi + 0.004)
    ax[1].set_ylim(0, 5600)
    ax[1].legend(frameon=False, loc='upper center', fontsize=8, labelcolor=ink, ncol=1)
    ax[2].axvline(C.POWER_USED_KW, color=ink, lw=2)
    ax[2].plot([p['p'] for p in oge['power']], [p['h_m'] for p in oge['power']], 'D-', ms=7, lw=2, color=ink2, mec='#fcfcfb', mew=1.5)
    ax[2].annotate('charts: about %.0f kW' % C.POWER_USED_KW, (C.POWER_USED_KW, 250), xytext=(-6, 0), textcoords='offset points',
                   ha='right', color=ink)
    ax[2].annotate('model', (oge['power'][0]['p'], oge['power'][0]['h_m']), xytext=(8, -2), textcoords='offset points', color=ink)
    ax[2].set_xlim(min(p['p'] for p in oge['power']) - 6, 398)
    ax[2].set_ylim(0, 5600)
    ax[2].set_xlabel('engine shaft power, kW'); ax[2].set_ylabel('density altitude, m')
    ax[2].set_title('Not fitted: power out of ground effect')
    for x in ax:
        x.grid(True, color=grid, lw=0.8); x.set_axisbelow(True)
        for s in ('top', 'right'):
            x.spines[s].set_visible(False)
    fig.text(0.008, 0.005, 'Charts: ' + C.SOURCE + '.', color=ink2, fontsize=8)
    fig.savefig(path, dpi=160, bbox_inches='tight')


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--fit', action='store_true', help='also refit GE_K, and the ground effect at other wheel heights')
    args = ap.parse_args()
    os.makedirs(os.path.join(HERE, 'out'), exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp, Pool(WORKERS) as pool:
        res = dict(source=C.SOURCE)
        res['before'] = compare(pool, GE_K_BEFORE, WHEELS_M, tmp)
        res['after'] = compare(pool, U.GE_K, WHEELS_M, tmp)
        root = build(U.GE_K, tmp)
        pts = C.FIG35_OGE_LOW + [(h, wo) for h, wo, wi in C.FIG35_AREA1]
        res['oge'] = dict(power=pool.map(oge_power, [(root, h, kg) for h, kg in pts]),
                          ceiling=pool.map(ceiling, [(root, None, kg, C.TABLE[kg][1]) for kg in (2200, 2100, 1900)]))
        res['checks'] = verdict(res['after'])
        if args.fit:
            res['fit'] = [fit(pool, w, tmp) for w in HEIGHTS_M]
            res['heights'] = []
            for w in HEIGHTS_M:
                r = pool.map(equal_power, points35(root, w))
                res['heights'].append(dict(wheels_m=w, ratio=float(np.mean([p['ratio'] for p in r])),
                                           chart=float(np.mean([p['ratio_chart'] for p in r]))))
    json.dump(res, open(os.path.join(HERE, 'out', 'hovercharts.json'), 'w'), indent=1, default=float)
    text = report(res)
    open(os.path.join(HERE, 'out', 'hovercharts.md'), 'w').write(text)
    plot(res, os.path.join(HERE, 'out', 'hovercharts.png'))
    print(text)
    sys.exit(0 if all(c['ok'] for c in res['checks']) else 1)


if __name__ == '__main__':
    main()
