#!/usr/bin/env python3
"""The blade slap's level for the flight model: PyWopWop's slap map (tools/slap_map.py: the main rotor trimmed over a grid
of airspeed and vertical speed, its advance ratio mu and inflow through the tip-path plane lambda, and the slap layer's
level in the cockpit and outside) turned into tables over (mu, lambda), relative to the bank's slap loops at SLAP_REF's
state, as the MV-22B's rotorsound.py does it (slap_table). genal3.py looks the rotor's state up in them, so that a flare,
a decelerating turn or a descent at any attitude plays the slap at the map's level, not only the bank's one descent.

    python3 al3slap.py [rotor bank]          (default ~/git/PyWopWop/out/alouette3; writes data/slap_al3.json)

The table: mu and lam (the grid), ext and int (the slap loops' amplitude outside and in the cockpit, 1 at SLAP_REF's
state), mu_z and lam0 (PyWopWop's mean induced inflow over mu and the upflow through the tip-path plane, at the map's
thrust: the flight model takes its inflow from these at its own kinematics, since the map's levels come from PyWopWop's
wake at PyWopWop's inflow), thrust_ref_lbf (the thrust at SLAP_REF's state: the vortices' strength goes with it), and
each view's largest amplitude (peak).
"""
import json
import os
import sys

import numpy as np
from scipy.interpolate import griddata

HERE = os.path.dirname(os.path.abspath(__file__))
SLAP_REF = 'bvi'                 # the bank's condition whose slap loops the amplitude is relative to (the approach descent)
SLAP_VIEW = {'side': ('ext', 'side'), 'cockpit': ('int', 'cockpit')}   # a loop's view: the table's key, the map's observer
SLAP_FLOOR_DB = -60.0            # no slap: where the map has none, and outside it


def slap_table(bank):
    m = json.load(open(os.path.join(bank, 'manifest.json')))
    d = json.load(open(os.path.join(bank, 'slap_map.json')))
    ref = next(c for c in m['conditions'] if c['name'] == SLAP_REF)
    pts = [q for q in d['points'] if 'error' not in q]
    vc = np.array([(q['airspeed_kt'], q['climb_fpm'] / 100.0) for q in pts])
    at_ref = lambda vals: float(griddata(vc, vals, [(ref['airspeed_kt'], ref['climb_fpm'] / 100.0)], 'linear')[0])
    mu = np.array([q['mu'] for q in pts])
    lam = np.array([q['lam'] for q in pts])
    muz = np.array([q['mu_z'] for q in pts])
    mu_g = np.round(np.arange(0.0, mu.max() + 0.0101, 0.01), 3)
    lam_g = np.round(np.arange(np.floor(lam.min() / 0.0025) * 0.0025 - 0.01, lam.max() + 0.0126, 0.0025), 4)
    muz_g = np.round(np.arange(np.floor(muz.min() / 0.005) * 0.005, muz.max() + 0.0051, 0.005), 4)
    out = dict(source='PyWopWop slap map (%s, %s quality, %g s per state) of the bank %s, its %s loops at %g kt, '
                      '%+g ft/min' % (d['aircraft'], d['quality'], d['duration'], os.path.basename(os.path.normpath(bank)),
                                      SLAP_REF, ref['airspeed_kt'], ref['climb_fpm']),
               mu=mu_g.tolist(), lam=lam_g.tolist(), mu_z=muz_g.tolist(),
               thrust_ref_lbf=round(at_ref([q['thrust_N'] for q in pts]) / 4.448222, 1), peak={})
    MU, MUZ = np.meshgrid(mu_g, muz_g)
    l0 = np.array([q['lambda0'] for q in pts])
    g = griddata(np.c_[mu, muz], l0, (MU, MUZ), 'linear')
    out['lam0'] = np.round(np.where(np.isnan(g), griddata(np.c_[mu, muz], l0, (MU, MUZ), 'nearest'), g), 5).tolist()
    MU, LAM = np.meshgrid(mu_g, lam_g)
    for view, (key, obs) in SLAP_VIEW.items():
        L = np.array([q['levels'][obs]['slap']['dba'] for q in pts])
        L = np.where(L < -100, np.nan, L)                     # no slap at all (the hover)
        rel = L - at_ref(np.nan_to_num(L, nan=L[np.isfinite(L)].min() - 60))
        rel = np.where(np.isfinite(rel), np.maximum(rel, SLAP_FLOOR_DB), SLAP_FLOOR_DB)
        g = griddata(np.c_[mu, lam], rel, (MU, LAM), 'linear')
        g = np.where(np.isnan(g), griddata(np.c_[mu, lam], rel, (MU, LAM), 'nearest'), g)    # past its edge: the nearest
        g[0, :] = g[-1, :] = SLAP_FLOOR_DB                    # beyond the map's inflow: the wake far from the disc
        g[:, 0] = SLAP_FLOOR_DB                               # the hover
        amp = 10 ** (g / 20)
        out[key] = np.round(amp, 4).tolist()
        out['peak'][key] = float(amp.max())
    return out


def slap_levels(bank):
    """the map's slap level (dBA) at an airspeed and vertical speed relative to SLAP_REF's state, by view"""
    m = json.load(open(os.path.join(bank, 'manifest.json')))
    d = json.load(open(os.path.join(bank, 'slap_map.json')))
    ref = next(c for c in m['conditions'] if c['name'] == SLAP_REF)
    pts = [q for q in d['points'] if 'error' not in q]
    vc = np.array([(q['airspeed_kt'], q['climb_fpm'] / 100.0) for q in pts])

    def rel(view, kt, fpm):
        L = np.array([q['levels'][SLAP_VIEW[view][1]]['slap']['dba'] for q in pts])
        L = np.where(L < -100, L[L > -100].min() - 60, L)
        at = lambda x, y: float(griddata(vc, L, [(x, y / 100.0)], 'linear')[0])
        return at(kt, fpm) - at(ref['airspeed_kt'], ref['climb_fpm'])
    return rel


if __name__ == '__main__':
    bank = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/git/PyWopWop/out/alouette3')
    t = slap_table(bank)
    os.makedirs(os.path.join(HERE, 'data'), exist_ok=True)
    path = os.path.join(HERE, 'data', 'slap_al3.json')
    json.dump(t, open(path, 'w'))
    print('%s: mu 0-%g, lambda %+g..%+g, thrust %g lbf; peak amplitude outside %.2f (%+.1f dB), in the cockpit %.2f '
          '(%+.1f dB) over the bank\'s slap loops' % (path, t['mu'][-1], t['lam'][0], t['lam'][-1], t['thrust_ref_lbf'],
                                                     t['peak']['ext'], 20 * np.log10(t['peak']['ext']),
                                                     t['peak']['int'], 20 * np.log10(t['peak']['int'])))
