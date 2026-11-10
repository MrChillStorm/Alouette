#!/usr/bin/env python3
"""Do the Alouette III's sounds keep to SimGear's limit? Checks on the written file (Alouette-3/Sounds/al3-sound.xml).

    python3 al3soundcheck.py

SimGear has 128 sources for everything, and a looped sound holds one while its condition is true, however quiet. This
evaluates every loop's real <condition> and <volume> the way SimGear reads them, along flights (a start, a ground run, a
hover in and out of ground effect, a climb, the cruise, descents, an approach, a landing and a shutdown, with the doors
shut and open, from inside and outside) and over random states:

  - how many run at once, against LIMIT (kept well under 128, which other sounds share);
  - how much of the sources' time is held at under -60 dB of the loop's own full volume (a gate wider than its fade);
  - that no loop is cut off while it could be heard (over -60 dB of that, its condition false);
  - that every loop plays at its own pitch while it could be heard: SimGear plays a <pitch> block at its <offset> (1.0
    when none is given) plus its factor times its property within <min> and <max>, and caps the sum at 2.0
    (simgear/sound/xmlsound.cxx, sample.hxx);
  - that nothing runs on a parked, cold aircraft;
  - that every condition is one FlightGear loads.
"""
import os
import sys
import numpy as np
import scipy.io.wavfile as wavfile
import xml.etree.ElementTree as ET
import json
from scipy.interpolate import griddata

HERE = os.path.dirname(os.path.abspath(__file__))
XML = os.path.join(HERE, '..', 'Alouette-3', 'Sounds', 'al3-sound.xml')
LIMIT = 64
HEARD = 1e-3
DOORS = ('fdm/jsbsim/animation/left-door-pos-norm', 'fdm/jsbsim/animation/right-door-pos-norm',
         'fdm/jsbsim/animation/left-cabin_door-pos-norm', 'fdm/jsbsim/animation/right-cabin_door-pos-norm')
NOM, NOM_T, N_IDLE = 353.2, 1938.0, 48.7
fails = []
BANK = os.path.expanduser('~/git/PyWopWop/out/alouette3')


def slap_amps(kt, vs_fps):
    """the flight model's slap amplitudes (/rotors/main/slap-ext, -int) as its table gives them in a trimmed state: the
    slap map's level at the airspeed and vertical speed against the bank's approach descent's (al3slap.py's reference)"""
    if not os.path.exists(os.path.join(BANK, 'slap_map.json')):
        return {}
    d = json.load(open(os.path.join(BANK, 'slap_map.json')))
    ref = next(c for c in json.load(open(os.path.join(BANK, 'manifest.json')))['conditions'] if c['name'] == 'bvi')
    pts = [q for q in d['points'] if 'error' not in q]
    vc = np.array([(q['airspeed_kt'], q['climb_fpm'] / 100.0) for q in pts])
    X = np.c_[np.clip(kt, vc[:, 0].min(), vc[:, 0].max()), np.clip(vs_fps * 0.6, vc[:, 1].min(), vc[:, 1].max())]
    out = {}
    for key, obs in (('ext', 'side'), ('int', 'cockpit')):
        L = np.array([q['levels'][obs]['slap']['dba'] for q in pts])
        L = np.where(L < -100, L[L > -100].min() - 60, L)
        r = float(griddata(vc, L, [(ref['airspeed_kt'], ref['climb_fpm'] / 100.0)], 'linear')[0])
        g = griddata(vc, L, X, 'linear')
        g = np.where(np.isnan(g), griddata(vc, L, X, 'nearest'), g)
        out['rotors/main/slap-' + key] = 10.0 ** ((g - r) / 20.0)
    return out


def check(name, ok, detail=''):
    print('%-4s %s%s' % ('ok' if ok else 'FAIL', name, ('  ' + detail) if detail else ''))
    if not ok:
        fails.append(name)


def val(e, P):
    t = e.tag
    if t == 'value':
        return float(e.text)
    if t == 'property':
        return P[e.text.strip().lstrip('/')]
    k = [c for c in e if c.tag not in ('clipMin', 'clipMax')]
    if t == 'sum':
        return sum(val(c, P) for c in k)
    if t == 'difference':
        r = val(k[0], P)
        for c in k[1:]:
            r = r - val(c, P)
        return r
    if t == 'product':
        r = 1.0
        for c in k:
            r = r * val(c, P)
        return r
    if t == 'div':
        return val(k[0], P) / val(k[1], P)
    if t == 'sqrt':
        return np.sqrt(np.maximum(val(k[0], P), 0.0))
    if t == 'pow':
        return np.power(np.maximum(val(k[0], P), 0.0), val(k[1], P))
    if t == 'clip':
        return np.clip(val(k[0], P), float(e.find('clipMin').text), float(e.find('clipMax').text))
    raise ValueError('expression: ' + t)


def test(e, P):
    t = e.tag
    if t == 'and':
        r = True
        for c in e:
            r = r & test(c, P)
        return r
    if t == 'or':
        r = False
        for c in e:
            r = r | test(c, P)
        return r
    if t == 'not':
        return ~np.asarray(test(list(e)[0], P), bool)
    if t == 'property':
        return P[e.text.strip().lstrip('/')] > 0.5
    a, b = list(e)
    if t == 'greater-than':
        return val(a, P) > val(b, P)
    if t == 'less-than':
        return val(a, P) < val(b, P)
    if t == 'equals':
        return val(a, P) == val(b, P)
    raise ValueError('condition: ' + t)


def entries():
    root = ET.parse(XML).getroot()
    return list(root.find('fx'))


def loads(E):
    """what SimGear loads (props/condition.cxx): and, or, not, a property and comparisons of a property or a value"""
    CMP = ('greater-than', 'less-than', 'equals', 'greater-than-equals', 'less-than-equals', 'not-equals')

    def ok(e):
        if e.tag in ('and', 'or', 'not'):
            return all(ok(c) for c in e)
        if e.tag == 'property':
            return True
        return e.tag in CMP and len(e) == 2 and all(c.tag in ('property', 'value') for c in e)
    bad = [x.findtext('name') for x in E if x.find('condition') is not None and not all(ok(c) for c in x.find('condition'))]
    internal = [x.findtext('name') for x in E if x.find('.//internal') is not None]
    nooff = [x.findtext('name') for x in E if (x.findtext('name') or '').startswith('pww')
             and any(pt.find('expression') is None and pt.find('offset') is None for pt in x.findall('pitch'))]
    check('every bank loop states its pitch offset (SimGear adds 1.0 otherwise)', not nooff,
          f'{len(nooff)}, e.g. {nooff[:2]}')
    check("every sound's condition is one FlightGear loads", not bad, f'{len(bad)} refused, e.g. {bad[:2]}' if bad else f'{len(E)} sounds')
    check('no <internal> (it reads 0 for ever)', not internal, str(internal[:2]))


def props(E):
    names = set()
    for x in E:
        for el in x.iter('property'):
            names.add(el.text.strip().lstrip('/'))
    return names


_RMS = {}


def rms(path):
    """a loop's rms, full scale 1"""
    if path not in _RMS:
        fs, x = wavfile.read(os.path.join(HERE, '..', 'Alouette-3', path))
        x = x.astype(float) / (32768.0 if x.dtype == np.int16 else 1.0)
        _RMS[path] = float(np.sqrt(np.mean(x ** 2)))
    return _RMS[path]


def evaluate(E, P):
    n = len(next(iter(P.values())))
    full = {k: np.broadcast_to(np.asarray(v, float), (n,)).copy() for k, v in P.items()}
    for k in props(E):
        full.setdefault(k, np.zeros(n))
    loops = [x for x in E if x.findtext('mode') == 'looped']
    C = np.zeros((len(loops), n), bool)
    V = np.zeros((len(loops), n))
    for i, x in enumerate(loops):
        c = x.find('condition')
        C[i] = np.broadcast_to(test(list(c)[0], full), (n,)) if c is not None else True
        vol = x.find('volume')
        ex = vol.find('expression') if vol is not None else None
        V[i] = np.broadcast_to(val(list(ex)[0], full), (n,)) if ex is not None else 1.0
    return loops, C, V


def sg_pitch(x, P):
    """the pitch SimGear plays a loop at (xmlsound.cxx's pitch(), sample.hxx's cap), and the one its bank meant (the
    factor times the property within <min> and <max>, nothing added)"""
    n = len(next(iter(P.values())))
    pitch, offset, meant = np.ones(n), np.zeros(n), np.ones(n)
    for pt in x.findall('pitch'):
        ex = pt.find('expression')
        if ex is not None:
            v = np.broadcast_to(val(list(ex)[0], P), (n,))
            pitch, meant = pitch * v, meant * v
            continue
        prop = pt.findtext('property')
        q = P.get(prop.strip().lstrip('/'), np.zeros(n)) if prop else np.ones(n)
        q = q * float(pt.findtext('factor') or 1.0)
        lo, hi = float(pt.findtext('min') or 0.0), float(pt.findtext('max') or 0.0)
        q = np.where(hi and q > hi, hi, np.where(q < lo, lo, q)) if hi else np.maximum(q, lo)
        offset = offset + float(pt.findtext('offset') or 1.0)
        pitch, meant = pitch * q, meant * q
    return np.clip(offset + pitch, 0.01, 2.0), meant


def flight(dt=0.5):
    """a flight, state by state: (seconds, {property: value})"""
    rng = np.random.default_rng(5)
    segs = []

    def seg(t, **kw):
        segs.append((t, kw))
    # parked, cold; the start: cranking to light-off, to ground idle (held), the clutch from 58 % taking the rotor up
    seg(20, n=(0, 0), rpm=(0, 0), lit=0, wow=1, agl=4, spd=0, vs=0)
    seg(15, n=(0, 16), rpm=(0, 0), lit=0, wow=1, agl=4, spd=0, vs=0)
    seg(30, n=(16, N_IDLE), rpm=(0, 0), lit=1, wow=1, agl=4, spd=0, vs=0)
    seg(60, n=(N_IDLE, N_IDLE), rpm=(0, 0), lit=1, wow=1, agl=4, spd=0, vs=0)
    seg(20, n=(N_IDLE, 58), rpm=(0, 0), lit=1, wow=1, agl=4, spd=0, vs=0)
    seg(40, n=(58, 100), rpm=(0, NOM), lit=1, wow=1, agl=4, spd=0, vs=0)
    seg(60, n=(100, 100), rpm=(NOM, NOM), lit=1, wow=1, agl=4, spd=0, vs=0)          # the ground run, governed
    # lift off, hover in and out of ground effect, transition, climb, cruise
    seg(10, n=(100, 100), rpm=(NOM, NOM), lit=1, wow=0, agl=(4, 8), spd=0, vs=0.5)
    seg(30, n=(100, 100), rpm=(NOM, NOM), lit=1, wow=0, agl=(8, 8), spd=(0, 3), vs=0)
    seg(20, n=(100, 100), rpm=(NOM, NOM), lit=1, wow=0, agl=(8, 60), spd=(3, 5), vs=3)
    seg(30, n=(100, 100), rpm=(NOM, NOM), lit=1, wow=0, agl=(60, 100), spd=(5, 60), vs=2)
    seg(60, n=(100, 100), rpm=(NOM, NOM), lit=1, wow=0, agl=(100, 1000), spd=(60, 70), vs=(10, 16))
    seg(30, n=(100, 100), rpm=(NOM, NOM), lit=1, wow=0, agl=1000, spd=(70, 100), vs=(16, 0))
    seg(120, n=(100, 100), rpm=(NOM, NOM), lit=1, wow=0, agl=1000, spd=(100, 100), vs=0)
    # descents: the approach's slap, a steep one, slowing to a hover and landing
    seg(30, n=(100, 100), rpm=(NOM, NOM), lit=1, wow=0, agl=(1000, 800), spd=(100, 65), vs=(0, -13))
    seg(60, n=(100, 100), rpm=(NOM, NOM), lit=1, wow=0, agl=(800, 300), spd=(65, 65), vs=-13)
    seg(30, n=(100, 100), rpm=(NOM, NOM), lit=1, wow=0, agl=(300, 100), spd=(65, 50), vs=(-13, -20))
    seg(30, n=(100, 100), rpm=(NOM, NOM), lit=1, wow=0, agl=(100, 10), spd=(50, 20), vs=(-20, -2))
    seg(10, n=(100, 100), rpm=(NOM, NOM), lit=1, wow=0, agl=(10, 5), spd=(20, 15), vs=-1.5)
    # a running landing, slowing on the wheels, a taxi and a stop
    seg(15, n=(100, 100), rpm=(NOM, NOM), lit=1, wow=1, agl=4, spd=(15, 0), vs=0, gs=(15, 0))
    seg(20, n=(100, 100), rpm=(NOM, NOM), lit=1, wow=1, agl=4, spd=(0, 8), vs=0, gs=(0, 8))
    seg(20, n=(100, 100), rpm=(NOM, NOM), lit=1, wow=1, agl=4, spd=(8, 0), vs=0, gs=(8, 0))
    # shut down: back to idle, the rotor running down, the engine cut and spooling down
    seg(30, n=(100, N_IDLE), rpm=(NOM, 200), lit=1, wow=1, agl=4, spd=0, vs=0)
    seg(60, n=(N_IDLE, N_IDLE), rpm=(200, 20), lit=1, wow=1, agl=4, spd=0, vs=0)
    seg(40, n=(N_IDLE, 0.3), rpm=(20, 0.5), lit=0, wow=1, agl=4, spd=0, vs=0)
    seg(30, n=(0.3, 0), rpm=(0.5, 0), lit=0, wow=1, agl=4, spd=0, vs=0)
    T, rows = 0.0, []
    for dur, kw in segs:
        k = int(dur / dt)
        for j in range(k):
            u = j / max(k - 1, 1)
            row = {'gs': 0.0}
            for key, x in kw.items():
                row[key] = x[0] + (x[1] - x[0]) * u if isinstance(x, tuple) else x
            rows.append(row)
    P = {}
    n = len(rows)
    col = lambda key: np.array([r[key] for r in rows], float)
    gov = col('rpm') >= NOM - 1e-6
    # the governor's wander and the gauges' noise
    P['rotors/main/rpm'] = np.where(gov, NOM + 0.6 * rng.standard_normal(n), col('rpm'))
    P['rotors/tail/rpm'] = P['rotors/main/rpm'] * NOM_T / NOM
    nn = col('n')
    P['fdm/jsbsim/eng/n-pct'] = np.where((nn == 100) | (nn == N_IDLE), nn + 0.15 * rng.standard_normal(n), nn)
    P['fdm/jsbsim/eng/lit'] = col('lit')
    P['position/altitude-agl-ft'] = col('agl')
    P['velocities/airspeed-kt'] = np.maximum(col('spd') + 0.5 * rng.standard_normal(n) * (col('spd') > 0), 0.0)
    P['velocities/vertical-speed-fps'] = col('vs') + 0.3 * rng.standard_normal(n) * (col('wow') < 0.5)
    tq = np.where(col('wow') > 0.5, 17.8, 60.0) * (P['rotors/main/rpm'] > 300)
    P['fdm/jsbsim/drive/torque-pct'] = tq
    # the starter while it cranks, to its cut at 41.8 %, the start only (the spool rising)
    rising = np.concatenate([[False], np.diff(nn) > 0])
    P['fdm/jsbsim/eng/starter-on'] = ((nn < 41.8) & (rising | ((nn > 0) & (nn < 16) & (col('lit') < 0.5)))).astype(float)
    P['velocities/equivalent-kt'] = P['velocities/airspeed-kt']
    P['fdm/jsbsim/velocities/vtrue-kts'] = P['velocities/airspeed-kt']
    P['environment/density-slugft3'] = np.full(n, 0.00237689)
    P['environment/temperature-degc'] = np.full(n, 15.0)
    # the turbine's outlet as the flight model's: a 1.5 s lag on its steady value (about 300 deg C at idle, 480 at
    # 100 %), the air's temperature unlit
    ss = 15.0 + col('lit') * (285.0 + 3.5 * (nn - N_IDLE))
    t4 = np.empty(n); t4[0] = ss[0]
    for i in range(1, n):
        t4[i] = t4[i - 1] + (ss[i] - t4[i - 1]) * (1.0 - np.exp(-dt / 1.5))
    P['fdm/jsbsim/eng/t4-degc'] = t4
    P.update(slap_amps(P['velocities/airspeed-kt'], P['velocities/vertical-speed-fps']))
    gs = col('gs') * 0.514444
    for k in range(3):
        P[f'gear/gear[{k}]/wow'] = col('wow')
        P[f'gear/gear[{k}]/rollspeed-ms'] = np.where(col('wow') > 0.5, gs, 0.0) + 1e-11 * rng.random(n)
        P[f'gear/gear[{k}]/compression-norm'] = 0.18 * col('wow')
    return n, P


def with_view(P, n, internal, doors):
    Q = dict(P)
    Q['sim/current-view/internal'] = np.full(n, float(internal))
    for d in DOORS:
        Q[d] = np.full(n, float(doors))
    return Q


def above_floor(loops, P):
    """per loop and state: its pitch's property times its factor at or above its <min>"""
    out = []
    n = len(next(iter(P.values())))
    for x in loops:
        pt = x.find('pitch')
        if pt is None or pt.find('property') is None:
            out.append(np.ones(n, bool))
            continue
        raw = P[pt.findtext('property').strip().lstrip('/')] * float(pt.findtext('factor') or 1.0)
        out.append(raw >= float(pt.findtext('min') or 0.0))
    return np.array(out)


def flights(E):
    n, P = flight()
    worst = {}
    held = nil = held_s = nil_s = 0.0
    cut, offpitch = [], []
    for internal in (0, 1):
        for d in ((0.0, 1.0) if internal else (0.0,)):
            loops, C, V = evaluate(E, with_view(P, n, internal, d))
            on = C.sum(0)
            k = int(np.argmax(on))
            worst[(internal, d)] = (int(on.max()), k)
            # each bank loop's level in its own view (its volume times its file's rms), against the loudest of that view
            # at that moment, but never less than -60 dB of the loudest it reaches in the flight: a whisper among
            # nothing isn't heard
            bank = np.array([[x.findtext('name').startswith('pww')] for x in loops])
            mine = bank & np.array([[('-cockpit' in x.findtext('name') or x.findtext('name').startswith('pww-wind'))
                                     == bool(internal)] for x in loops])
            lev = np.where(mine, V * np.array([[rms(x.findtext('path')) if b else 0.0] for x, b in zip(loops, bank[:, 0])]), 0.0)
            now = lev.max(0, keepdims=True)
            rel = lev / np.maximum(now, 1e-30)
            # heard: within 60 dB of the loudest playing, above 80 dB under the loudest of the flight, and inside its
            # pitch range (below its floor it can't follow its speed)
            audible = (rel >= HEARD) & (lev >= 1e-4 * now.max()) & above_floor(loops, with_view(P, n, internal, d))
            # held at nil: under -60 dB of the loop's own full volume (its gate wider than its fade); whether something
            # louder masks it is a gate's business only where it is cut off (below)
            own = V / np.maximum(V.max(1, keepdims=True), 1e-30)
            # (the slap loops follow the flight model's slap over 40 dB and more: under -60 dB of their loudest they
            # may still be heard, which the cut-off check below sees; their time apart)
            slapl = np.array([[x.findtext('name').endswith('-slap')] for x in loops])
            held += (C & mine & ~slapl).sum()
            nil += (C & mine & ~slapl & (own < HEARD)).sum()
            held_s += (C & mine & slapl).sum()
            nil_s += (C & mine & slapl & (own < HEARD)).sum()
            for i in np.where(((~C) & mine & audible).any(1))[0]:
                cut.append(loops[i].findtext('name'))
            Q = with_view(P, n, internal, d)
            for i, x in enumerate(loops):
                if not (C[i] & mine[i] & audible[i]).any():
                    continue
                sg, meant = sg_pitch(x, Q)
                if (np.abs(sg / meant - 1.0)[C[i] & audible[i]] > 1e-3).any():
                    offpitch.append(x.findtext('name'))
    view = {(0, 0.0): 'outside', (1, 0.0): 'cockpit, doors shut', (1, 1.0): 'cockpit, doors open'}
    for key, (m, k) in worst.items():
        check(f'along the flight, {view[key]}: at most {m} sources at once', m <= LIMIT, f'(state {k})')
    check('source-time held under -60 dB of the loop\'s own full volume', nil / max(held, 1) < 0.05, f'{100 * nil / max(held, 1):.1f} % of the source-time')
    print(f'     (the slap loops apart: {100 * nil_s / max(held_s, 1):.1f} % of their source-time under -60 dB of their loudest)')
    check('no loop cut off while it could be heard', not cut, f'{len(set(cut))}: {sorted(set(cut))[:4]}')
    check('every loop at its own pitch while it could be heard', not offpitch,
          f'{len(set(offpitch))}: {sorted(set(offpitch))[:4]}')


def parked(E):
    n = 4
    P = {'rotors/main/rpm': np.zeros(n), 'rotors/tail/rpm': np.zeros(n), 'fdm/jsbsim/eng/n-pct': np.zeros(n),
         'fdm/jsbsim/eng/lit': np.zeros(n), 'gear/gear[0]/wow': np.ones(n), 'gear/gear[1]/wow': np.ones(n),
         'sim/current-view/internal': np.array([0, 1, 0, 1.0])}
    for d in DOORS:
        P[d] = np.array([0, 0, 1, 1.0])
    loops, C, V = evaluate(E, P)
    on = [loops[i].findtext('name') for i in np.where(C.any(1))[0] if loops[i].findtext('name').startswith('pww')]
    check('nothing of the banks runs on a parked, cold aircraft', not on, str(on[:4]))


def sweep(E, n=20000, seed=3):
    r = np.random.default_rng(seed)
    pick = lambda grid, lo, hi: np.where(r.random(n) < 0.5, r.choice(grid, n), r.uniform(lo, hi, n))
    P = {'sim/current-view/internal': r.integers(0, 2, n).astype(float)}
    rpm = pick([0, 0.2 * NOM, 0.35 * NOM, 0.5 * NOM, 0.65 * NOM, 0.8 * NOM, 0.9 * NOM, NOM], 0, 1.05 * NOM)
    P['rotors/main/rpm'] = rpm
    P['rotors/tail/rpm'] = rpm * NOM_T / NOM
    P['fdm/jsbsim/eng/n-pct'] = pick([0, 3, 12, 20, 30, 40, N_IDLE, 56, 63, 72, 81, 90, 100, 104], 0, 106)
    P['fdm/jsbsim/eng/lit'] = r.integers(0, 2, n).astype(float)
    P['fdm/jsbsim/eng/t4-degc'] = r.uniform(0, 500, n)
    P['rotors/main/slap-ext'] = r.uniform(0, 4, n)
    P['rotors/main/slap-int'] = r.uniform(0, 4, n)
    P['environment/temperature-degc'] = r.uniform(-30, 45, n)
    w = r.integers(0, 2, n).astype(float)
    P['gear/gear[0]/wow'] = w
    P['gear/gear[1]/wow'] = np.where(r.random(n) < 0.1, 1 - w, w)
    P['velocities/airspeed-kt'] = pick([0, 40, 50, 65, 70, 80, 100], 0, 130)
    P['velocities/vertical-speed-fps'] = pick([0, 5, 10, 16.7, -5, -13.3, -20], -30, 30)
    P['position/altitude-agl-ft'] = pick([4, 9, 18, 27], 0, 2000)
    P['fdm/jsbsim/drive/torque-pct'] = r.uniform(-10, 105, n)
    for d in DOORS:
        P[d] = np.where(r.random(n) < 0.7, 0.0, r.uniform(0, 1, n))
    loops, C, V = evaluate(E, P)
    on = C.sum(0)
    check(f'over {n} random states: at most {on.max()} sources at once', on.max() <= LIMIT,
          f'(median {np.median(on):.0f})')


def main():
    E = entries()
    loads(E)
    parked(E)
    flights(E)
    sweep(E)
    print('all ok' if not fails else f'{len(fails)} failed')
    sys.exit(1 if fails else 0)


if __name__ == '__main__':
    main()
