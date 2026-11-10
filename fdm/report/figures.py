"""Figures for the report (vector PDF) from report/results/*.json.  python3 report/figures.py"""
import sys, os, json, math
HERE = os.path.dirname(os.path.abspath(__file__))
FDM = os.path.dirname(HERE)
sys.path.insert(0, FDM)
import numpy as np
import matplotlib
matplotlib.use('pdf')
import matplotlib.pyplot as plt
import al3data as U
import vrs as VRS
from data import al3_published as PB

plt.rcParams.update({'font.family': 'serif', 'font.serif': ['STIX Two Text', 'STIXGeneral', 'DejaVu Serif'], 'mathtext.fontset': 'stix',
                     'font.size': 8.5, 'axes.labelsize': 8.5, 'axes.titlesize': 9, 'legend.fontsize': 7.5, 'xtick.labelsize': 7.5,
                     'ytick.labelsize': 7.5, 'axes.linewidth': 0.6, 'lines.linewidth': 1.1, 'grid.linewidth': 0.3, 'grid.alpha': 0.5,
                     'axes.grid': True, 'figure.dpi': 150, 'savefig.bbox': 'tight', 'savefig.pad_inches': 0.02,
                     'axes.spines.top': False, 'axes.spines.right': False})
C_MOD, C_MOD2, C_TEST, C_REF = '#b2182b', '#2166ac', '#222222', '#7a7a7a'
W1, W2 = 3.35, 6.9          # single / double column width (in)
OUT = os.path.join(HERE, 'fig')
RES = os.path.join(HERE, 'results')
KT_KMH = 1.852


def res(name):
    d = json.load(open(os.path.join(RES, name + '.json')))
    if name.startswith('trims_'):
        d = [r for r in d if r['res'] < 0.05]           # converged trims only: past the blades' stall they do not
    return d


def have(name):
    return os.path.exists(os.path.join(RES, name + '.json'))


def save(fig, name):
    fig.savefig(os.path.join(OUT, name + '.pdf')); plt.close(fig)


def tag(ax, s):
    ax.set_title('(%s)' % s, loc='left', fontsize=8.5, pad=3)


def trims():
    sets = [(res('trims_2200'), '2200 kg', C_MOD), (res('trims_1750'), '1750 kg', C_MOD2)]
    panels = [('coll07', r'collective at 0.7$R$, deg'), ('B1s', r'longitudinal cyclic $B_{1s}$, deg (+ fwd)'),
              ('A1s', r'lateral cyclic $A_{1s}$, deg (+ right)'), ('tr_deg', 'tail rotor pitch, deg'),
              ('att', 'attitude, deg (pitch, roll)'), ('engine_kw', 'engine power, kW')]
    fig, axs = plt.subplots(2, 3, figsize=(W2, 4.0))
    for i, (ax, (k, lab)) in enumerate(zip(axs.flat, panels)):
        for rows, name, c in sets:
            V = [r['V'] for r in rows]
            if k == 'att':
                ax.plot(V, [r['theta'] for r in rows], '-', color=c, label='pitch, ' + name)
                ax.plot(V, [r['phi'] for r in rows], '--', color=c, label='roll, ' + name)
            else:
                ax.plot(V, [r[k] for r in rows], '-', color=c, label=name)
        if k == 'B1s':
            ax.axhline(U.B1S_RANGE[1], color=C_REF, lw=0.7, ls=':'); ax.text(2, U.B1S_RANGE[1] - 1.3, 'full forward', fontsize=6.5, color=C_REF)
        if k == 'engine_kw':
            for p, lab2 in ((U.P_TO_KW, 'take-off 420'), (U.P_MCP_KW, 'max. continuous 405')):
                ax.axhline(p, color=C_REF, lw=0.7, ls=':')
            ax.text(2, U.P_TO_KW + 8, 'take-off, max. continuous', fontsize=6.5, color=C_REF)
            for src, kmh in PB.CRUISE_KMH.items():
                ax.axvline(kmh / KT_KMH, color=C_TEST, lw=0.5, ls='--')
            ax.axvline(PB.VMAX_KMH['TD'] / KT_KMH, color=C_TEST, lw=0.8)
            ax.text(PB.VMAX_KMH['TD'] / KT_KMH + 1, 150, r'$V_{NE}$', fontsize=7, rotation=90)
            ax.text(97 - 9, 150, 'cruise', fontsize=7, rotation=90)
        ax.set_xlabel('airspeed, kt'); ax.set_ylabel(lab, fontsize=7.5)
        ax.set_xlim(0, 126)
        tag(ax, 'abcdef'[i])
        if i in (0, 4):
            ax.legend(frameon=False, loc='best')
    fig.tight_layout()
    save(fig, 'trims')


def hover():
    h = res('hover')
    fig, ax = plt.subplots(figsize=(W1, 2.7))
    for kg, c in ((2200, C_MOD), (1750, C_MOD2)):
        for mode, ls in (('oge', '-'), ('ige', '--')):
            pts = [p for p in h['%d-%s' % (kg, mode)]['points'] if p[3] < 0.05]      # converged trims: above, the rotor stalls
            alt = [p[0] * 0.3048 for p in pts]; pk = [p[1] for p in pts]
            ax.plot(alt, pk, ls, color=c, label='%d kg, %s' % (kg, mode.upper()))
            for (src, k2, m2), cm in PB.HOVER_CEILING_M.items():
                if k2 == kg and m2 == mode:
                    y = float(np.interp(cm, alt, pk))
                    ax.plot(cm, y, 'o' if src == 'AS' else 's', mfc='none', mec=C_TEST, ms=4, mew=0.8)
                    ax.annotate(src, (cm, y), xytext=(3, -9), textcoords='offset points', fontsize=6.5)
    ax.axhline(U.P_TO_KW, color=C_REF, lw=0.7, ls=':'); ax.text(100, U.P_TO_KW + 4, 'take-off rating 420 kW', fontsize=6.5, color=C_REF)
    ax.set_xlabel('pressure altitude, m (standard day)'); ax.set_ylabel('engine power to hover, kW')
    ax.legend(frameon=False, fontsize=6.5, ncol=2, loc='lower right')
    ax.set_xlim(0, 6200)
    fig.tight_layout()
    save(fig, 'hover')


def ge():
    g = res('ge')['points']
    hub = U.HUB_YASIM[2] - U.MAIN_WHEELS[0][2]                  # m: the hub above the wheels' contact
    z = np.array([p['wheel_ft'] + hub * 3.28084 for p in g])          # ft, as U.R
    oge = res('hover')['2200-oge']['points'][0][1] if have('hover') else g[-1]['kw']
    ratio = np.array([p['kw'] for p in g]) / g[-1]['kw']
    fig, ax = plt.subplots(figsize=(W1, 2.4))
    ax.plot(z / (2 * U.R), ratio, 'o-', color=C_MOD, ms=3.5, label='model, hover flown by the test autopilot')
    zz = np.linspace(0.3, 2.0, 100) * 2 * U.R
    ki = 1 - (U.R / (4 * zz)) ** 2                                    # Cheeseman and Bennett, induced power at constant thrust
    # the induced share of the power out of ground effect: the weight times the hover's induced velocity (descent.py)
    vh = res('descent')['vh_hover'] * 0.3048 if have('descent') else 9.96
    fi = g[-1]['weight'] * 4.448222 * vh / 1000 / g[-1]['kw']
    ax.plot(zz / (2 * U.R), 1 - fi * (1 - ki), '--', color=C_REF, label='Cheeseman--Bennett, %.0f %% of it induced' % (100 * fi))
    ax.set_xlabel(r'rotor height over diameter, $z/D$'); ax.set_ylabel(r'power over power at 100 ft')
    ax.legend(frameon=False, fontsize=6.5, loc='lower right')
    fig.tight_layout()
    save(fig, 'ge')


def mission():
    rec = res('mission')['rec']
    t = np.array([r['t'] for r in rec]); t -= t[0]
    fig, axs = plt.subplots(3, 2, figsize=(W2, 4.6), sharex=True)
    series = [(('h',), 'height of the wheels, ft', 1.0), (('kt',), 'airspeed, kt', 1.0), (('nr',), 'rotor speed, rpm', 1.0),
              (('kw',), 'drive load, kW', 1.0), (('theta', 'phi'), 'pitch, roll, deg', 1.0), (('coll', 'lon', 'lat', 'ped'), 'controls (units)', 1.0)]
    for i, (ax, (keys, lab, sc)) in enumerate(zip(axs.flat, series)):
        for j, k in enumerate(keys):
            ax.plot(t, [r[k] * sc for r in rec], lw=0.9, label=k if len(keys) > 1 else None,
                    color=[C_MOD, C_MOD2, '#4d9221', '#8c510a'][j])
        ax.set_ylabel(lab, fontsize=7.5)
        if len(keys) > 1:
            ax.legend(frameon=False, fontsize=6.5, ncol=len(keys), loc='upper right')
        if k == 'nr':
            ax.axhline(PB.NR_AUTO_MIN, color=C_REF, lw=0.6, ls=':')
        tag(ax, 'abcdef'[i])
    names = list(dict.fromkeys(r.get('phase') for r in rec if r.get('phase')))
    for ax in axs.flat:
        for n in names:
            t0 = next(r['t'] for r in rec if r.get('phase') == n) - rec[0]['t']
            ax.axvline(t0, color=C_REF, lw=0.4)
    for n in names:
        t0 = next(r['t'] for r in rec if r.get('phase') == n) - rec[0]['t']
        axs[0, 0].text(t0 + 2, axs[0, 0].get_ylim()[1] * 0.98, n, fontsize=5.5, rotation=90, va='top')
    for ax in axs[-1]:
        ax.set_xlabel('time, s')
    fig.tight_layout()
    save(fig, 'mission')


def steps():
    s = res('steps')
    panels = [('lat', 'p', 'roll rate, deg/s', 'lateral cyclic'), ('lon', 'q', 'pitch rate, deg/s', 'longitudinal cyclic (aft)'),
              ('ped', 'r', 'yaw rate, deg/s', 'pedal (right)'), ('coll', 'vs', 'vertical speed, ft/min', 'collective')]
    fig, axs = plt.subplots(1, 4, figsize=(W2, 1.9))
    for i, (ax, (axis, k, lab, title)) in enumerate(zip(axs, panels)):
        rec = s[axis]
        ax.plot([r['t'] for r in rec], [r[k] for r in rec], color=C_MOD)
        ax.set_xlabel('time, s'); ax.set_ylabel(lab, fontsize=7.5)
        ax.set_title('(%s) %s, 10 %%' % ('abcd'[i], title), loc='left', fontsize=7.5, pad=3)
    fig.tight_layout()
    save(fig, 'steps')


def engine():
    e = res('engine')
    st = e['start']['rec']; gv = e['governor']['rec']; fl = e['failure_held']['rec']
    fig, axs = plt.subplots(1, 3, figsize=(W2, 2.2))
    ax = axs[0]
    ax.plot([r['t'] for r in st], [r['n'] for r in st], color=C_MOD, label='engine, %')
    ax.plot([r['t'] for r in st], [100 * r['nr'] / PB.NR_RPM for r in st], color=C_MOD2, label='rotor, %')
    ax2 = ax.twinx(); ax2.plot([r['t'] for r in st], [r['t4'] for r in st], color='#4d9221', lw=0.8, label='T4')
    ax2.set_ylabel(r'T4, $^\circ$C', fontsize=7.5); ax2.grid(False)
    ax.set_xlabel('time from the starter, s'); ax.set_ylabel('speed, %'); ax.legend(frameon=False, fontsize=6.5, loc='center right')
    tag(ax, 'a')
    ax = axs[1]
    ax.plot([r['t'] for r in gv], [r['nr'] for r in gv], color=C_MOD2)
    ax.set_xlabel('time, s'); ax.set_ylabel('rotor speed, rpm')
    ax3 = ax.twinx(); ax3.plot([r['t'] for r in gv], [r['kw'] for r in gv], color=C_MOD, lw=0.8); ax3.set_ylabel('engine power, kW', fontsize=7.5); ax3.grid(False)
    tag(ax, 'b')
    ax = axs[2]
    ax.plot([r['t'] for r in fl], [r['nr'] for r in fl], color=C_MOD2)
    ax.axhline(PB.NR_AUTO_MIN, color=C_REF, lw=0.7, ls=':'); ax.text(0.2, PB.NR_AUTO_MIN + 4, '270 rpm', fontsize=6.5, color=C_REF)
    ax.set_xlabel('time from the fuel cut, s'); ax.set_ylabel('rotor speed, rpm')
    tag(ax, 'c')
    fig.tight_layout()
    save(fig, 'engine')


def climb_autorotation():
    c = res('climb'); a = res('autorotation')
    fig, ax = plt.subplots(figsize=(W1, 2.4))
    for key, name, col in (('mcp', '2200 kg', C_MOD), ('mcp_1750', '1750 kg', C_MOD2)):
        pts = c[key]['points']
        ax.plot([p['V'] for p in pts], [p['fpm'] / 196.85 for p in pts], 'o-', color=col, ms=3, label='climb at 405 kW, ' + name)
    pts = a['points']
    ax.plot([p['V'] for p in pts], [p['fpm'] / 196.85 for p in pts], 's-', color=C_REF, ms=3, label='autorotation, 2200 kg')
    for k, v in PB.CLIMB_MS.items():
        ax.axhline(v, color=C_TEST, lw=0.5, ls='--')
        ax.text(101, v, k, fontsize=6, va='center')
    ax.axhline(0, color='k', lw=0.5)
    ax.set_xlabel('airspeed, kt'); ax.set_ylabel('rate of climb, m/s')
    ax.legend(frameon=False, fontsize=6.5, loc='center left')
    ax.set_xlim(25, 110)
    fig.tight_layout()
    save(fig, 'climb')


def vrs():
    d = res('descent')
    fig, axs = plt.subplots(1, 2, figsize=(W2, 2.4))
    ax = axs[0]
    for sp, col in zip(d['speeds'][:2], (C_MOD, C_MOD2)):
        rows = sp['rows']
        z = [-r['z'] for r in rows]
        ax.plot(z, [r['nu'] for r in rows], '-', color=col, label='model, %d kt' % sp['kt'])
        ax.plot(z, [r['nu_mom'] for r in rows], ':', color=col, lw=0.8)
    ax.set_xlabel(r'descent rate over hover induced velocity, $-V_z/v_h$'); ax.set_ylabel(r'induced velocity, $v/v_h$')
    ax.legend(frameon=False, fontsize=6.5); tag(ax, 'a')
    ax = axs[1]
    for sp, col in zip(d['speeds'], (C_MOD, C_MOD2, '#4d9221', '#8c510a', C_REF)):
        rows = sp['rows']
        ax.plot([r['sink_fps'] * 60 for r in rows], [r['coll07'] for r in rows], '-', color=col, label='%d kt' % sp['kt'])
        for b in sp['boundary']:
            ax.axvline(-b * d['vh_hover'] * 60, color=col, lw=0.5, ls='--')
    ax.set_xlabel('rate of descent, ft/min'); ax.set_ylabel(r'collective at 0.7$R$, deg')
    ax.legend(frameon=False, fontsize=6.5, ncol=2); tag(ax, 'b')
    fig.tight_layout()
    save(fig, 'vrs')


ALL = dict(trims=trims, hover=hover, ge=ge, mission=mission, steps=steps, engine=engine, climb=climb_autorotation, vrs=vrs)

if __name__ == '__main__':
    os.makedirs(OUT, exist_ok=True)
    for name in (sys.argv[1:] or ALL):
        try:
            ALL[name]()
            print('fig', name)
        except FileNotFoundError as e:
            print('fig', name, 'skipped:', e)
