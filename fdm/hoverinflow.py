#!/usr/bin/env python3
"""The hover's inflow correction (al3data.HOVER_VI, HOVER_VI_MACH): where its two numbers come from, and checks of them.

In the hover the blades meet the wake's induced velocity times 1 + HOVER_VI (1 + HOVER_VI_MACH (M - 0.60) / 0.1)
cos^2(wake skew), M the tip Mach number (genal3.py). Its shape comes from the full-size Bo 105 rotor, its strength from
the Alouette (data/bo105_hover.py: the Bo 105's data and their sources).

  1. shape, from the Bo 105 alone: the model's own hover method (blade segments, tip loss, section tables, steady
     Peters-He inflow), run on the Bo 105's rotor, solved at each of fig. 3's points for the factor eps (the blades
     meeting (1 + eps) times the wake's induced velocity) with which thrust adds the measured power:
     CP(CT) - CP(0) measured. Taking the power at no thrust out of both sides leaves out the two blade sections'
     difference in drag at no lift (the Bo 105's 23012 against the model's 0012). eps is then fitted against blade loading
     and tip Mach. Checks: HOVER_VI_MACH is the fit's Mach slope over its level at HOVER_VI_M0, and the loading slope is
     within two standard errors of none (the same factor at every loading).
  2. a test the shape was not fitted to: TM-85892's tabulated hover at 425 rpm has the collective too. Its rise from the
     no-thrust setting (which leaves out the cambered section's zero-lift angle) against the method's, without the
     correction and with the Bo 105's own (the fit's level). Check: with it the method is closer, within TOL_PITCH rms.
  3. strength, from the Alouette: one number, refitted here on the generated model: the hover out of ground effect at
     100 m density altitude and 2190 kg on the charts' 390 kW [SPEC fig. 3-5]. Check: HOVER_VI is the refit's.
  4. not fitted, reported: the model's power at the other points of figs. 3-5 and 3-6 less the charts' 390 kW, and the
     hover without the correction.

  python3 hoverinflow.py     writes out/hoverinflow.json and .md; exits 1 if a check fails. About a minute on 4 cores.

The test models are generated into a temporary folder; nothing in the aircraft is touched.
"""
import json, math, os, subprocess, sys, tempfile
from multiprocessing import Pool
import numpy as np
from scipy import optimize
import al3data as U
import peters_he as PH
import hovercharts as HC
from data import sections as SEC
from data import bo105_hover as BO
from data import al3_hover_charts as C

HERE = os.path.dirname(os.path.abspath(__file__))
WORKERS = 4
BO_ROOT = 0.20                          # the Bo 105's blade root cut-out, not given: 0.15 and 0.25 are also tried
RHO = 1.225                             # the results are coefficients: the density cancels
TOL_SHAPE = 0.01                        # HOVER_VI_MACH against the refit
TOL_PITCH = 0.25                        # deg rms, the collective's rise at TM-85892's points
TOL_VI = 0.002                          # HOVER_VI against the refit (0.002 is about 0.6 kW at sea level)
ST = [s for s in U.PH_COS if s[1] == 0]                       # the axisymmetric states: all a hover excites
L0 = PH.Lmat(ST, 0.0, 'c')


class Hover:
    """the model's hover aerodynamics (genal3.py's) for the Bo 105's rotor on a stand, without flapping"""

    def __init__(s, rpm, mach, root):
        s.R, s.c, s.nb = BO.R_M, BO.CHORD_M, BO.BLADES
        s.om = rpm * math.pi / 30.0
        s.vtip = s.om * s.R
        s.a_snd = s.vtip / mach
        segs, edges = U.segments(U.NSEG, root * s.R, s.R)
        s.r = np.array([x[0] for x in segs]); s.dr = np.array([x[1] for x in segs]); e = np.array(edges)
        s.fl = np.clip((U.TIP_LOSS * s.R - e[:-1]) / (e[1:] - e[:-1]), 0, 1)
        al, mm, cl, cd = SEC.tables(SEC.NACA0012)
        s.al, s.mm, s.cl, s.cd = np.array(al, float), np.array(mm, float), np.array(cl, float), np.array(cd, float)
        s.phi = np.array([[PH.phi(j, rh, r / s.R) for (j, rh) in ST] for r in s.r])

    def look(s, tab, a, M):
        j = np.clip(np.searchsorted(s.mm, M) - 1, 0, len(s.mm) - 2)
        t = np.clip((M - s.mm[j]) / (s.mm[j + 1] - s.mm[j]), 0, 1)
        c0 = np.array([np.interp(x, s.al, tab[:, k]) for x, k in zip(a, j)])
        c1 = np.array([np.interp(x, s.al, tab[:, k + 1]) for x, k in zip(a, j)])
        return (1 - t) * c0 + t * c1

    def loads(s, root_pitch, w, eps):
        UT, UP = s.om * s.r, (1 + eps) * s.om * s.R * (s.phi @ w)
        V = np.hypot(UT, UP)
        alpha = root_pitch + BO.TWIST_DEG * s.r / s.R - np.degrees(np.arctan2(UP, UT))
        M = np.minimum(V / s.a_snd, 1.0)
        Lk = 0.5 * s.c * s.dr * s.fl * RHO * V * s.look(s.cl, alpha, M)
        Dk = 0.5 * s.c * s.dr * RHO * V * (s.look(s.cd, alpha, M) + U.DCD)
        Fn = Lk * UT - Dk * UP
        return Fn, s.nb * s.om * np.sum(s.r * (Lk * UP + Dk * UT))

    def solve(s, cts, eps):
        """the collective at 0.7 R (deg) and CP for the thrust CT/sigma, collective and inflow states solved together"""
        ct = cts * BO.SIGMA
        unit = RHO * math.pi * s.R ** 2 * s.vtip ** 2

        def residual(y):
            w, th = y[:3], y[3]
            Fn = s.loads(th, w, eps)[0]
            tau = (1 / U.TIP_LOSS ** 2) / (2 * math.pi) * s.nb * (s.phi.T @ Fn) / (RHO * s.om ** 2 * s.R ** 4)
            lam = max(math.sqrt(3) * w[0], 1e-4)
            Vp = np.array([lam] + [2 * lam] * (len(ST) - 1))
            return np.r_[w - (L0 @ (0.5 * tau)) / Vp, (s.nb * np.sum(Fn)) / unit - ct]
        y0 = [math.sqrt(max(ct, 2e-4) / 2) / math.sqrt(3), 0.0, 0.0, 12.0 if ct > 1e-3 else 6.0]
        sol = optimize.root(residual, np.array(y0), method='hybr', tol=1e-12)
        if np.max(np.abs(residual(sol.x))) > 1e-7:
            raise RuntimeError('no Bo 105 hover at CT/sigma %.4f, eps %.3f' % (cts, eps))
        return sol.x[3] + 0.7 * BO.TWIST_DEG, s.loads(sol.x[3], sol.x[:3], eps)[1] / (unit * s.vtip)

    def eps_for(s, cts, cp, cp0):
        ideal = (cts * BO.SIGMA) ** 1.5 / math.sqrt(2)

        def miss(e):
            return (s.solve(cts, e)[1] - s.solve(0.0, e)[1] - (cp - cp0)) / ideal
        return optimize.brentq(miss, -0.2, 0.6, xtol=1e-5)


def shape(root, cp0_sigma):
    """eps at fig. 3's points and its fit: level at HOVER_VI_M0, slope per 0.01 CT/sigma and per 0.1 of tip Mach"""
    rows = []
    for rpm, cts, fm in BO.FIG3B:
        mach = rpm * math.pi / 30 * BO.R_M / BO.SOUND_MS
        ct = cts * BO.SIGMA
        cp = ct ** 1.5 / (math.sqrt(2) * fm)
        rows.append(dict(rpm=rpm, cts=cts, mach=mach, eps=Hover(rpm, mach, root).eps_for(cts, cp, cp0_sigma * BO.SIGMA)))
    X = np.array([[1.0, (r['cts'] - 0.08) / 0.01, (r['mach'] - U.HOVER_VI_M0) / 0.1] for r in rows])
    y = np.array([r['eps'] for r in rows])
    coef = np.linalg.lstsq(X, y, rcond=None)[0]
    res = y - X @ coef
    err = np.sqrt(np.diag(np.linalg.inv(X.T @ X)) * np.sum(res ** 2) / (len(y) - 3))
    return dict(root=root, cp0_sigma=cp0_sigma, points=rows, level=float(coef[0]), loading=float(coef[1]),
                mach=float(coef[2]), err=[float(e) for e in err], rms=float(np.sqrt(np.mean(res ** 2))),
                mach_ratio=float(coef[2] / coef[0]))


def pitch(level, mach_ratio, root):
    """TM-85892's collective, risen from no thrust, against the method's without and with the Bo 105's own correction"""
    h = Hover(BO.TABLE_RPM, BO.TABLE_TIP_MACH, root)
    eps = level * (1 + mach_ratio * (BO.TABLE_TIP_MACH - U.HOVER_VI_M0) / 0.1)
    th00, cp00 = h.solve(0.0, 0.0)
    th0w = h.solve(0.0, eps)[0]
    measured0, _, _, cp0 = BO.TABLE[0]
    rows = []
    for th, lift, forward, cp in BO.TABLE[1:]:
        cts = BO.table_thrust(lift, forward)
        rows.append(dict(cts=cts, measured=th - measured0, without=h.solve(cts, 0.0)[0] - th00,
                         with_=h.solve(cts, eps)[0] - th0w, eps_power=h.eps_for(cts, cp * BO.SIGMA, cp0 * BO.SIGMA)))
    rms = lambda key: float(np.sqrt(np.mean([(r[key] - r['measured']) ** 2 for r in rows])))
    return dict(eps=eps, points=rows, rms_without=rms('without'), rms_with=rms('with_'))


# ------------------------------------------------------------------ the Alouette
def build(vi, tmp):
    root = os.path.join(tmp, 'vi%.5f' % vi)
    if not os.path.exists(root):
        code = 'import sys; sys.path.insert(0, %r); import al3data as U; U.HOVER_VI = %r; import genal3; genal3.build(%r)' % (
            HERE, float(vi), os.path.join(root, 'aircraft', 'alouette3'))
        subprocess.run([sys.executable, '-c', code], check=True, cwd=HERE, stdout=subprocess.DEVNULL)
    return root


def power(job):
    root, temp, h_m, kg = job
    HC._models.clear()
    return dict(temp_c=temp, h_m=h_m, kg=kg, kw=HC.hover(root, kg, h_m, None, temp)[0])


def strength(pool, tmp):
    """HOVER_VI with which the first chart point takes the charts' power, by the secant method"""
    h, kg = C.FIG35_OGE_LOW[0]
    miss = lambda vi: pool.apply(power, ((build(vi, tmp), None, h, kg),))['kw'] - C.POWER_USED_KW
    a, b = 0.0, U.HOVER_VI
    fa, fb = miss(a), miss(b)
    for _ in range(8):
        if abs(fb) < 0.1:
            break
        a, b, fa = b, b - fb * (b - a) / (fb - fa), fb
        fb = miss(b)
    return b, fb


def report(res):
    s, p = res['shape'], res['pitch']
    t = HC.table
    out = ['# The hover\'s inflow correction', '', '## 1. Its shape, from the Bo 105 (NASA TM-85990, fig. 3b)', '',
           t(['rpm', 'tip Mach', 'CT/sigma', 'eps'], [[r['rpm'], '%.3f' % r['mach'], '%.4f' % r['cts'], '%+.3f' % r['eps']] for r in s['points']]),
           '', 'eps = %.3f +-%.3f at tip Mach %.2f, %+.3f +-%.3f per 0.01 of CT/sigma, %+.3f +-%.3f per 0.1 of tip Mach; rms %.3f.'
           % (s['level'], s['err'][0], U.HOVER_VI_M0, s['loading'], s['err'][1], s['mach'], s['err'][2], s['rms']),
           'Mach slope over level: %.3f (the model: %.3f).' % (s['mach_ratio'], U.HOVER_VI_MACH), '',
           t(['blade root, of the radius', 'CP/sigma at no thrust', 'level', 'per 0.01 CT/sigma', 'Mach slope over level'],
             [['%.2f' % q['root'], '%.6f' % q['cp0_sigma'], '%.3f' % q['level'], '%+.3f' % q['loading'], '%.3f' % q['mach_ratio']]
              for q in res['sensitivity']]),
           '', '## 2. Not fitted: the collective\'s rise from no thrust (TM-85892, 425 rpm), deg', '',
           t(['CT/sigma', 'measured', 'method', 'with the Bo 105\'s correction (%.3f)' % p['eps'], 'eps from the power there'],
             [['%.4f' % r['cts'], '%.2f' % r['measured'], '%.2f' % r['without'], '%.2f' % r['with_'], '%+.3f' % r['eps_power']] for r in p['points']]),
           '', 'rms off the measured: %.2f deg without, %.2f deg with.' % (p['rms_without'], p['rms_with']),
           '', '## 3. Its strength, from the Alouette', '',
           'On %.0f kW at %d m and %d kg: HOVER_VI %.4f (the model: %.4f).' % (
               C.POWER_USED_KW, C.FIG35_OGE_LOW[0][0], C.FIG35_OGE_LOW[0][1], res['strength'], U.HOVER_VI),
           '', '## 4. Not fitted: hover out of ground effect, the model\'s power less the charts\' %.0f kW' % C.POWER_USED_KW, '',
           t(['outside temperature C', 'altitude m', 'kg', 'without the correction', 'with it'],
             [['standard' if a['temp_c'] is None else a['temp_c'], a['h_m'], '%.0f' % a['kg'],
               '%+.1f' % (b['kw'] - C.POWER_USED_KW), '%+.1f' % (a['kw'] - C.POWER_USED_KW)]
              for a, b in zip(res['with'], res['without'])]),
           '', '## Checks', '',
           t(['check', 'value', 'limit', ''], [[c['check'], c['value'], c['limit'], 'pass' if c['ok'] else 'FAIL'] for c in res['checks']])]
    return '\n'.join(out) + '\n'


def main():
    os.makedirs(os.path.join(HERE, 'out'), exist_ok=True)
    s = shape(BO_ROOT, BO.FIG3A_CP0_SIGMA)
    sens = [shape(r, c) for r in (0.15, BO_ROOT, 0.25) for c in (0.0009, BO.TABLE[0][3], 0.0012)]
    p = pitch(s['level'], s['mach_ratio'], BO_ROOT)
    pts = [(None, h, kg) for h, kg in C.FIG35_OGE_LOW + [(h, wo) for h, wo, wi in C.FIG35_AREA1]]
    pts += [(T, pa, C.chart_weight(C.FIG36_OGE, T, pa)) for T, pa in C.OFF_ISA_POINTS]
    with tempfile.TemporaryDirectory() as tmp, Pool(WORKERS) as pool:
        vi, left = strength(pool, tmp)
        with_ = pool.map(power, [(build(U.HOVER_VI, tmp),) + q for q in pts])
        without = pool.map(power, [(build(0.0, tmp),) + q for q in pts])
    res = dict(shape=s, sensitivity=sens, pitch=p, strength=vi, **{'with': with_, 'without': without})
    res['checks'] = [
        dict(check='HOVER_VI_MACH against the Bo 105 fit', value='%.3f against %.3f' % (U.HOVER_VI_MACH, s['mach_ratio']),
             limit='within %.3f' % TOL_SHAPE, ok=abs(U.HOVER_VI_MACH - s['mach_ratio']) <= TOL_SHAPE),
        dict(check='the Bo 105 fit\'s loading slope', value='%+.3f +-%.3f' % (s['loading'], s['err'][1]),
             limit='within two standard errors of 0', ok=abs(s['loading']) <= 2 * s['err'][1]),
        dict(check='the collective\'s rise, not fitted', value='%.2f deg rms with, %.2f without' % (p['rms_with'], p['rms_without']),
             limit='closer with, and within %.2f deg' % TOL_PITCH, ok=p['rms_with'] <= min(TOL_PITCH, p['rms_without'])),
        dict(check='HOVER_VI against the refit on the model', value='%.4f against %.4f (%+.2f kW left)' % (U.HOVER_VI, vi, left),
             limit='within %.4f' % TOL_VI, ok=abs(U.HOVER_VI - vi) <= TOL_VI and abs(left) < 0.2)]
    json.dump(res, open(os.path.join(HERE, 'out', 'hoverinflow.json'), 'w'), indent=1, default=float)
    text = report(res)
    open(os.path.join(HERE, 'out', 'hoverinflow.md'), 'w').write(text)
    print(text)
    sys.exit(0 if all(c['ok'] for c in res['checks']) else 1)


if __name__ == '__main__':
    main()
