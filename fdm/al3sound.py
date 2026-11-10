#!/usr/bin/env python3
"""The Alouette III's sound: PyWopWop's rotor and engine banks wired into the aircraft.

    python3 al3sound.py [rotor bank] [engine bank]

The banks default to ~/git/PyWopWop/out/alouette3 and alouette3_engines. This writes ../Alouette-3/Sounds/al3-sound.xml,
copies the loops it uses into ../Alouette-3/Sounds/pywopwop, and leaves Sounds/sound.xml (the old generic set) as it is.

Sources. SimGear has 128 sources for everything, and a looped sound holds one while its condition is true, however
quiet. So every loop's condition here is where its fade isn't nil: its view (the cockpit's loops only inside, the
outside's only outside), on the ground or off it, the airspeed, vertical speed and height its fade spans, its rotor's or
the spool's speed range, lit or not, a door open (the open cockpit's loops) or not all of them fully open (the shut
cockpit's), and not crashed. The bank's own sections gated only on the view and a turning rotor, so all of a view's
loops held sources at once. The governor holds the rotor and the spool on grid points (100 %, and the spool's ground
idle), which a speed only approaches, so there each fade has a flat top: within a small band of the grid point only that
loop plays, its neighbours are off, and the ramps on either side move with it so the fades still add up. A layer more
than MASKED_DB under the loudest in its own loop isn't heard and isn't wired. al3soundcheck.py tests all of this on the
written XML, over random states and along flights.

Fades. As PyWopWop's sections make them: the rotors' loops by airspeed (hover, translational lift, 80 and 100 kt), the
climb and the descents by vertical speed inside an airspeed window, the level loops taking what those leave, the hover in
ground effect by height, the ground run by rotor speed on the ground; linear in amplitude. The engine's by the spool's
speed, equal power, lit or cold. In the cockpit the shut and the all-open cockpit's loops by how much of the doors is open,
by equal power over their areas.

Levels. The banks' groups (rotors and engine, cockpit and outside) have their own gains (manifest gain_db); each loop's
volume carries its group's back to one scale, the loudest group's at 1, so nothing needs a volume over one (FlightGear
clips it). The outside's loops are PyWopWop's levels at their recording distance (100 m in flight, 50 m on the ground and
in ground effect), which is their reference distance here.

The wind on the canopy (in the cockpit), the gear in the airflow and the tyres rolling are PyWopWop's too (mkbanks.py):
the wind by the equivalent sea level airspeed PyWopWop's own section works out (its gate on the equivalent airspeed,
widened to hold that speed at any height), the gear by true airspeed with the air's density, each tyre by its rolling
speed with weight on its wheel, at its contact. A parked wheel's speed jitters about nil: its lowest loop's gate is
where it falls to -60 dB.

Kept from sound.xml: the rain (in the cockpit only) and the crash; its generic wind, now PyWopWop's, is not. Not kept: the generic rotor, turbine, start and shutdown (the banks
replace them), a skid's scrape on touchdown (the Alouette has wheels) and an airspeed overspeed horn above 146 kt, out of
the aircraft's reach.
"""
import json
import math
import os
import shutil
import sys
import xml.etree.ElementTree as ET

import numpy as np
import scipy.io.wavfile as wavfile

import al3slap

HERE = os.path.dirname(os.path.abspath(__file__))
AC = os.path.join(HERE, '..', 'Alouette-3')
SND = os.path.join(AC, 'Sounds')
PWW = os.path.expanduser('~/git/PyWopWop/out')
SOUND_DIR = 'Sounds/pywopwop'                # as the sound file's paths have it (from the aircraft's folder)
OUT_XML = os.path.join(SND, 'al3-sound.xml')

INTERNAL = 'sim/current-view/internal'
CRASHED = 'fdm/jsbsim/systems/crash-detect/crashed'
WOW = ('gear/gear[0]/wow', 'gear/gear[1]/wow')     # the two main wheels
SPD = 'velocities/airspeed-kt'
VS = 'velocities/vertical-speed-fps'
AGL = 'position/altitude-agl-ft'
MASKED_DB = 40.0        # a layer this far (dB(A)) under its loop's loudest isn't wired
HEARD = 1e-3            # -60 dB: where a fade's last loop stops
FLAT_RPM = 0.01         # half the width of the rotor's flat tops at its run-up speeds, of its nominal speed
FLAT_PCT = 0.6          # the spool's, percent, at its ground idle and governed speed
KEEP = ('rain', 'intense ground contact')
# the blade slap's level from the flight model (data/slap_al3.json from PyWopWop's slap map by al3slap.py; genal3.py
# writes /rotors/main/slap-ext and -int, 1 at the bank's approach descent, al3slap.SLAP_REF): that state's slap loop alone,
# in flight with the rotor turning, at the flight model's level, so that a flare, a turn or a descent at any attitude
# plays the map's level; the other states' slap layers (rendered 20 dB and more weaker, their files' headroom too small to
# carry the map's level through the airspeed and descent fades) not wired. The file raised where the map's peak is
# louder than the reference, up to SLAP_PEAK_DBFS
SLAP_TABLE = os.path.join(HERE, 'data', 'slap_al3.json')
SLAP_PEAK_DBFS = -0.5
GEARBOX_STOP = 0.05       # of the rotor's speed: the gearbox's pitch floor, its lowest mesh then under 40 Hz
# the light-off: the lit loops come in, the unlit ones go, by equal power as the turbine's outlet warms, over its first
# LIGHTOFF_K over the air's temperature: the flight model's T4 (a 1.5 s lag) passes that 0.4 s after it lights, as a
# start recorded from outside with the camera's gain held (A-301) completes its step at light-off (every band 3-4 dB up)
# in about 0.5 s. The fuel's cut at a shutdown is a step, as the lit flag drops
T4 = 'fdm/jsbsim/eng/t4-degc'
T_AIR = 'environment/temperature-degc'
LIGHTOFF_K = 100.0
T4_COLD_GATE = 150.0      # deg C: the unlit loops held while the outlet is under it (the air at up to 50 deg C)


# ---------------------------------------------------------------- SimGear XML

def e(tag, *kids):
    return f'<{tag}>' + ''.join(kids) + f'</{tag}>'


def v(x):
    return f'<value>{x:.6g}</value>'


def p(name):
    return f'<property>{name}</property>'


def ramp(prop, a, b):
    """0 at a, 1 at b of a property (either order), clipped"""
    k = 1.0 / (b - a)
    return e('clip', e('sum', e('product', p(prop), v(k)), v(-a * k)), '<clipMin>0</clipMin><clipMax>1</clipMax>')


def gt(prop, x):
    return e('greater-than', p(prop), v(x))


def lt(prop, x):
    return e('less-than', p(prop), v(x))


def tents(nodes, flat, lo0=None):
    """fades over sorted grid points: for each, the (from, to) of its rising and falling ramps (None at an end) and its
    open interval. With flat tops of half width flat[i]: within it only point i plays, and both ramps either side move
    with it so neighbours still add to one. lo0: where the first point's fade starts (None: it holds to the bottom)"""
    out = []
    n = len(nodes)
    for i, x in enumerate(nodes):
        up = dn = None
        a, b = -1e30, 1e30
        if i > 0:
            up = (nodes[i - 1] + flat[i - 1], x - flat[i])
            a = up[0]
        elif lo0 is not None:
            up = (lo0, x - flat[i])
            a = lo0
        if i < n - 1:
            dn = (nodes[i + 1] - flat[i + 1], x + flat[i])
            b = dn[0]
        out.append((up, dn, a, b))
    return out


def gates(prop, a, b):
    g = []
    if a > -1e29:
        g.append(gt(prop, a))
    if b < 1e29:
        g.append(lt(prop, b))
    return g


def entry(tag, name, path, cond, vol, pitch=None, ref_dist=None):
    s = (f'\t\t<{tag}>\n\t\t\t<name>{name}</name>\n\t\t\t<mode>looped</mode>\n\t\t\t<path>{path}</path>\n'
         f'\t\t\t<condition>{e("and", *cond)}</condition>\n'
         f'\t\t\t<volume><expression>{e("product", *vol) if len(vol) > 1 else vol[0]}</expression></volume>\n')
    if pitch:
        # SimGear plays a pitch block at its <offset> plus its factor times its property, the offset 1.0 when none is
        # given (simgear/sound/xmlsound.cxx), and caps the sum at 2.0 (sample.hxx): every loop states its offset, 0
        if '<offset>' not in pitch:
            pitch = pitch.replace('</pitch>', '<offset>0</offset></pitch>')
        s += f'\t\t\t{pitch}\n'
    if ref_dist:
        s += f'\t\t\t<reference-dist>{ref_dist:g}</reference-dist>\n\t\t\t<max-dist>4000</max-dist>\n'
    return s + f'\t\t</{tag}>\n'


# ---------------------------------------------------------------- the views and the doors

def doors(openings):
    total = sum(o['share'] for o in openings)
    u = e('clip', e('sum', *[e('product', p(o['property'].lstrip('/')), v(o['share'] / total)) for o in openings]),
          '<clipMin>0</clipMin><clipMax>1</clipMax>')
    shut_vol = e('sqrt', e('difference', v(1.0), u))
    open_vol = e('sqrt', u)
    any_open = e('or', *[gt(o['property'].lstrip('/'), 0.001) for o in openings])
    not_all_open = e('not', e('and', *[gt(o['property'].lstrip('/'), 0.999) for o in openings]))
    return dict(cockpit=(shut_vol, not_all_open), cockpit_open=(open_vol, any_open))


def view_gate(view, door):
    g = [p(INTERNAL) if view.startswith('cockpit') else e('not', p(INTERNAL)), e('not', p(CRASHED))]
    v_ = []
    if view in door:
        v_.append(door[view][0])
        g.append(door[view][1])
    return g, v_


ON_GROUND = e('or', *[p(w) for w in WOW])
AIRBORNE_VOL = e('product', *[e('difference', v(1.0), p(w)) for w in WOW])
ON_GROUND_VOL = e('difference', v(1.0), AIRBORNE_VOL)


# ---------------------------------------------------------------- the rotors

def rotor_entries(bank, door, g_min):
    m = json.load(open(os.path.join(bank, 'manifest.json')))
    gains = {g: 10.0 ** ((g_min - x) / 20.0) for g, x in m['gain_db'].items()}
    conds = {c['name']: c for c in m['conditions']}
    files = {(f['condition'], f['observer'], f['source']): f for f in m['files']}
    main, tail = m['rotors']['main'], m['rotors']['tail']
    nom = main['nominal_rpm']
    R_ft = 5.51 / 0.3048
    z_lo, z_hi = 0.5 * R_ft, 1.5 * R_ft
    level = sorted([c for c in conds.values() if abs(c['climb_fpm']) < 1 and c['ground_clearance_m'] is None],
                   key=lambda c: c['airspeed_kt'])
    ige = [c for c in conds.values() if c['ground_clearance_m'] is not None and not c['ground_run']]
    runup = sorted([c for c in conds.values() if c['ground_run']], key=lambda c: c['rpm_percent'])
    climbs = sorted([c for c in conds.values() if c['climb_fpm'] > 1], key=lambda c: c['climb_fpm'])
    descents = sorted([c for c in conds.values() if c['climb_fpm'] < -1], key=lambda c: -c['climb_fpm'])
    plan = {}                       # condition -> (volume terms, gates)
    slap = None
    if os.path.exists(SLAP_TABLE) and os.path.exists(os.path.join(bank, 'slap_map.json')):
        slap = (json.load(open(SLAP_TABLE)), al3slap.slap_levels(bank))
    raised = {}                     # a raised slap file's name -> (the bank's file, its factor)

    def vertical(group, sign):
        rate = [sign * max(sign * c['climb_fpm'] / 60.0, 10.0) for c in group]
        for i, c in enumerate(group):
            lo = sign * 5.0 if i == 0 else rate[i - 1]
            vol = [ramp(VS, lo, rate[i])]
            g = [gt(VS, lo) if sign > 0 else lt(VS, lo)]
            if i < len(group) - 1:
                vol.append(ramp(VS, rate[i + 1], rate[i]))
                g.append(lt(VS, rate[i + 1]) if sign > 0 else gt(VS, rate[i + 1]))
            s0, s1 = max(c['airspeed_kt'] - 40.0, 0.0), c['airspeed_kt'] + 50.0
            vol += [ramp(SPD, s0, c['airspeed_kt']), ramp(SPD, s1, c['airspeed_kt'])]
            g += [gt(SPD, s0), lt(SPD, s1)]
            plan[c['name']] = (vol, g)
    vertical(climbs, 1.0)
    vertical(descents, -1.0)
    moving = [plan[c['name']][0] for c in climbs + descents]
    rest = e('clip', e('difference', v(1.0), *[e('product', *w) for w in moving]),
             '<clipMin>0</clipMin><clipMax>1</clipMax>') if moving else None
    for (up, dn, a, b), c in zip(tents([c['airspeed_kt'] for c in level], [0.0] * len(level)), level):
        vol = [ramp(SPD, *up)] if up else []
        vol += [ramp(SPD, *dn)] if dn else []
        g = gates(SPD, a, b)
        if ige and c['airspeed_kt'] == 0.0:
            vol.append(ramp(AGL, z_lo, z_hi))
            g.append(gt(AGL, z_lo))
        if rest:
            vol.append(rest)
        plan[c['name']] = (vol, g)
    nxt = next((c['airspeed_kt'] for c in level if c['airspeed_kt'] > 0), 40.0)
    for c in ige:
        plan[c['name']] = ([ramp(AGL, z_hi, z_lo), ramp(SPD, nxt, 0.0)] + ([rest] if rest else []),
                           [lt(AGL, z_hi), lt(SPD, nxt)])
    out, used, dropped = [], [], []
    for view in ('side', 'cockpit', 'cockpit_open'):
        group = 'cockpit' if view.startswith('cockpit') else 'exterior'
        vg, vv = view_gate(view, door)
        # the run-up's loudest loop, against which its slowest fades out to HEARD
        top = max(files[(c['name'], view, 'mix')]['dba'] for c in runup)
        low = files[(runup[0]['name'], view, 'mix')]['dba']
        rpm_nodes = [c['rpm_percent'] / 100.0 * nom for c in runup]
        flat = [FLAT_RPM * nom] * len(runup)
        rp = 'rotors/main/rpm'
        for k, ((up, dn, a, b), c) in enumerate(zip(tents(rpm_nodes, flat, lo0=0.0), runup)):
            vol = [ON_GROUND_VOL] + ([ramp(rp, *up)] if up else []) + ([ramp(rp, *dn)] if dn else [])
            if k == 0:       # the slowest stops where it falls to -60 dB of the loudest, or at its pitch floor
                a = max(rpm_nodes[0] * HEARD * 10.0 ** ((top - low) / 20.0), 0.2 * rpm_nodes[0])
            plan[c['name'] + '@' + view] = (vol, [ON_GROUND] + gates(rp, a, b))
        layers = {}
        for name, c in conds.items():
            here = {s: files[(name, view, s)] for s in ('rotor', 'slap', 'swish', 'tail') if (name, view, s) in files}
            loud = max(f['dba'] for f in here.values())
            for s, f in here.items():
                if f['dba'] < loud - MASKED_DB and not (slap and s == 'slap'):     # (the flight model may raise a slap)
                    dropped.append(f['file'])
                    continue
                layers[(name, s)] = f
        for (name, s), f in sorted(layers.items(), key=lambda kv: (conds[kv[0][0]]['ground_run'], kv[0])):
            c = conds[name]
            rot = tail if s == 'tail' else main
            prop = 'rotors/tail/rpm' if s == 'tail' else 'rotors/main/rpm'
            ref = rot['nominal_rpm'] * c['rpm_percent'] / 100.0
            if c['ground_run']:
                vol, g = plan[name + '@' + view]
                k = runup.index(c)
                lo_rpm = rot['nominal_rpm'] * (runup[k - 1]['rpm_percent'] / 100.0 if k else 0.05)
            else:
                vol, g = plan[name]
                vol = vol + [AIRBORNE_VOL, e('clip', e('product', p(prop), v(1.0 / rot['nominal_rpm'])),
                                             '<clipMin>0</clipMin><clipMax>1</clipMax>')]
                g = g + [e('not', ON_GROUND), gt(prop, 0.2 * rot['nominal_rpm'])]
                lo_rpm = 0.2 * rot['nominal_rpm']
            vol = vol + vv + ([v(gains[group])] if gains[group] < 0.9999 else [])
            fname = f['file']
            if slap and s == 'slap' and not c['ground_run']:
                if name != al3slap.SLAP_REF:
                    continue        # the slap is the reference state's loop alone, at the flight model's level
                tab, _ = slap
                key = 'ext' if view == 'side' else 'int'
                _, y = wavfile.read(os.path.join(bank, fname))
                room = 10.0 ** (SLAP_PEAK_DBFS / 20.0) / (np.abs(y.astype(np.float64)).max() / 32768.0)
                b = max(1.0, min(tab['peak'][key], room))
                # in flight with the rotor turning, by the flight model's slap alone (its map over the advance ratio
                # and the inflow has the airspeed and the descent in it), not the reference state's fades
                vol = [AIRBORNE_VOL, e('clip', e('product', p(prop), v(1.0 / rot['nominal_rpm'])),
                                       '<clipMin>0</clipMin><clipMax>1</clipMax>')] + vv \
                    + ([v(gains[group])] if gains[group] < 0.9999 else []) \
                    + [e('clip', e('product', p('rotors/main/slap-' + key), v(1.0 / b)), '<clipMin>0</clipMin><clipMax>1</clipMax>')]
                # off where it falls 66 dB under the reference state's whole sound (as rendered it stands below that
                # by its slap less its mix; 6 dB more, as a single layer of another loop may then be the loudest)
                under = files[(name, view, 'slap')]['dba'] - files[(name, view, 'mix')]['dba']
                g = [e('not', ON_GROUND), gt(prop, 0.2 * rot['nominal_rpm']),
                     gt('rotors/main/slap-' + key, 0.5 * HEARD * 10.0 ** (-under / 20.0))]
                if b > 1.0001:
                    fname = fname[:-4] + '_raised.wav'
                    raised[fname] = (os.path.join(bank, f['file']), b)
            pitch = (f'<pitch>{p(prop)}<factor>{1.0 / ref:.6g}</factor><min>{min(0.2, lo_rpm / ref):.3g}</min>'
                     f'<max>1.25</max></pitch>')
            ref_d = None if view.startswith('cockpit') else (
                (c['listener_distance_m'] or 50.0) if c['ground_clearance_m'] is not None else 100.0)
            out.append(entry('rotor', f'pww-{name}-{view}-{s}', f'{SOUND_DIR}/{fname}', vg + g, vol, pitch, ref_d))
            used.append(fname)
    return out, used, dropped, raised


# ---------------------------------------------------------------- the engine and the gearbox

def lit_share(run):
    """the lit loops' share of the power: 0 unlit, rising to 1 as the outlet warms LIGHTOFF_K over the air"""
    warm = e('clip', e('div', e('difference', p(T4), p(T_AIR)), v(LIGHTOFF_K)), '<clipMin>0</clipMin><clipMax>1</clipMax>')
    return e('product', p(run), warm)


def engine_entries(bank, door, g_min):
    m = json.load(open(os.path.join(bank, 'manifest.json')))
    gains = {g: 10.0 ** ((g_min - x) / 20.0) for g, x in m['gain_db'].items()}
    have = {(f['engine'], f['layer'], f['state'], f['ng'], f['observer']): f for f in m['files']}
    out, used = [], []
    for eng in m['engines']:
        who = eng['name']
        n1 = eng['n1_property'].lstrip('/')
        run = eng['running_property'].lstrip('/')
        crank = eng['starter_property'].lstrip('/')
        idle = eng['idle_ng']
        for view in ('side', 'cockpit', 'cockpit_open'):
            group = 'cockpit' if view.startswith('cockpit') else 'exterior'
            vg, vv = view_gate(view, door)
            for layer, state, grid, lo0 in (('compressor', 'lit', m['lit_ng'], eng['lightoff_ng'] - 5.0),
                                            ('compressor', 'cold', m['cold_ng'], 0.5),
                                            ('core', 'lit', m['lit_ng'], eng['lightoff_ng'] - 5.0),
                                            ('turbine', 'lit', m['lit_ng'], eng['lightoff_ng'] - 5.0),
                                            # the cranked engine's broadband and other tones, as far as its
                                            # recorded cranking goes
                                            ('core', 'cold', m['cold_ng'], 0.5),
                                            ('turbine', 'cold', m['cold_ng'], 0.5),
                                            ('starter', 'starter', m.get('starter_ng') or [], 0.5)):
                pts = [n for n in grid if (who, layer, state, n, view) in have]
                if not pts:
                    continue
                flat = [FLAT_PCT if state == 'lit' and (abs(n - idle) < 0.01 or abs(n - 100.0) < 0.01) else 0.0
                        for n in pts]
                fades = tents(pts, flat, lo0=lo0)
                above = [x for x in grid if x > pts[-1]]
                if above:               # a layer that stops below the grid's top fades out by its next point
                    up, _, a, _ = fades[-1]
                    fades[-1] = (up, (above[0], pts[-1]), a, above[0])
                for (up, dn, a, b), n in zip(fades, pts):
                    f = have[(who, layer, state, n, view)]
                    w = ([ramp(n1, *up)] if up else []) + ([ramp(n1, *dn)] if dn else [])
                    vol = [e('sqrt', e('product', *w) if len(w) > 1 else w[0])] if w else []
                    if state == 'lit':
                        vol.append(e('sqrt', lit_share(run)))
                        g = [gt(run, 0.5)]
                    elif state == 'starter':        # while it cranks, lit or not
                        vol.append(p(crank))
                        g = [gt(crank, 0.5)]
                    else:
                        vol.append(e('sqrt', e('difference', v(1.0), lit_share(run))))
                        g = [e('or', lt(run, 0.5), lt(T4, T4_COLD_GATE))]
                    vol = vol + vv + ([v(gains[group])] if gains[group] < 0.9999 else [])
                    lo, hi = (up[0] if up else n), (dn[0] if dn else 1.3 * n)
                    pitch = (f'<pitch>{p(n1)}<factor>{1.0 / n:.6g}</factor><min>{max(lo, 1.0) / n:.4g}</min>'
                             f'<max>{hi / n:.4g}</max></pitch>')
                    out.append(entry('engine', f'pww-{who}-{layer}-{state}-{n:g}-{view}', f'{SOUND_DIR}/{f["file"]}',
                                     vg + g + gates(n1, a, b), vol, pitch,
                                     None if view.startswith('cockpit') else m['exterior_distance_m']))
                    used.append(f['file'])
    gb = m.get('gearbox')
    f = next((f for f in m['files'] if f['engine'] == 'gearbox'), None)
    if gb and f:
        rpm = gb['rpm_property'].lstrip('/')
        tq = e('clip', e('div', e('sum', *[p(x.lstrip('/')) for x in gb['torque_properties']]),
                         v(100.0 * len(gb['torque_properties']))), '<clipMin>0.03</clipMin><clipMax>1</clipMax>')
        if gb.get('torque_exponent', 1.0) != 1.0:
            tq = e('pow', tq, v(gb['torque_exponent']))
        vol = [tq, e('clip', e('div', p(rpm), v(gb['full_rpm'])), '<clipMin>0</clipMin><clipMax>1</clipMax>'),
               v(gains['cockpit'])]
        # heard through the roof, the doors open or shut alike; it fades with the rotor, stopping at its pitch floor
        out.append(entry('engine', 'pww-gearbox-cockpit', f'{SOUND_DIR}/{f["file"]}',
                         [p(INTERNAL), e('not', p(CRASHED)), gt(rpm, GEARBOX_STOP * gb['nominal_rpm'])], vol,
                         f'<pitch>{p(rpm)}<factor>{1.0 / gb["nominal_rpm"]:.6g}</factor><min>0.05</min><max>1.3</max></pitch>'))
        used.append(f['file'])
    return out, used


# ---------------------------------------------------------------- the wind, the gear in the airflow, the tyres

SIGMA = e('div', p('environment/density-slugft3'), v(0.00237689))
HUB_FG = (-0.0329, 0.0, 1.7074)     # the hub in SimGear's frame (x aft, y left, z up, from the model's origin)


def position(pww):
    """PyWopWop's frame (x forward, y left, z up, from the hub) to SimGear's"""
    x, y, z = pww
    return (f'<position><x>{-x + HUB_FG[0]:.3f}</x><y>{y + HUB_FG[1]:.3f}</y><z>{z + HUB_FG[2]:.3f}</z></position>')


def wind_entries(bank, g_min):
    m = json.load(open(os.path.join(bank, 'manifest.json')))
    gain = 10.0 ** ((g_min - m['gain_db']['cockpit']) / 20.0)
    T = e('sum', p('environment/temperature-degc'), v(273.15))
    mu_r = e('product', e('pow', e('div', T, v(288.15)), v(1.5)), e('div', v(398.55), e('sum', T, v(110.4))))
    tas = e('div', p('velocities/equivalent-kt'), e('sqrt', SIGMA))
    veq = e('product', tas, e('pow', e('div', mu_r, SIGMA), v(-1.0 / 6.0)))
    amp = e('sqrt', e('product', SIGMA, mu_r))
    speeds = [f['airspeed_kt'] for f in m['files']]
    # below the slowest loop the wind's power goes as the speed to the power its two slowest loops give (A-weighted)
    k_low = (m['files'][1]['dba'] - m['files'][0]['dba']) / (10.0 * math.log10(speeds[1] / speeds[0]))
    out, used = [], []
    for i, ((up, dn, a, b), f) in enumerate(zip(tents(speeds, [0.0] * len(speeds), lo0=0.0), m['files'])):
        w = [e('clip', e('sum', e('product', veq, v(1.0 / (up[1] - up[0]))), v(-up[0] / (up[1] - up[0]))),
               '<clipMin>0</clipMin><clipMax>1</clipMax>')]
        if i == 0:
            w = [e('pow', w[0], v(k_low))]          # sqrt below: its amplitude as the speed to k_low / 2
            a = speeds[0] * 10.0 ** (-6.0 / k_low)  # where it is at -60 dB
        if dn:
            w.append(e('clip', e('sum', e('product', veq, v(1.0 / (dn[1] - dn[0]))), v(-dn[0] / (dn[1] - dn[0]))),
                       '<clipMin>0</clipMin><clipMax>1</clipMax>'))
        vol = [e('sqrt', e('product', *w) if len(w) > 1 else w[0]), amp] + ([v(gain)] if gain < 0.9999 else [])
        # the equivalent airspeed against PyWopWop's sea level one: within 0.8-1.25 to 20,000 ft
        g = [p(INTERNAL), e('not', p(CRASHED)), gt('velocities/equivalent-kt', 0.8 * a)]
        if dn:
            g.append(lt('velocities/equivalent-kt', 1.25 * b))
        out.append(entry('wind', f'pww-wind-{f["airspeed_kt"]:g}', f'{SOUND_DIR}/{f["file"]}', g, vol))
        used.append(f['file'])
    return out, used


def gear_entries(bank, g_min):
    m = json.load(open(os.path.join(bank, 'manifest.json')))
    tas_p = 'fdm/jsbsim/velocities/vtrue-kts'
    out, used = [], []
    for view in ('side', 'cockpit'):
        group = 'cockpit' if view == 'cockpit' else 'exterior'
        gain = 10.0 ** ((g_min - m['gain_db'][group]) / 20.0)
        fs = sorted([f for f in m['files'] if f['view'] == view], key=lambda f: f['airspeed_kt'])
        speeds = [f['airspeed_kt'] for f in fs]
        # the slowest from half its speed: under it the gear is 36 dB down, as M^6
        for (up, dn, a, b), f in zip(tents(speeds, [0.0] * len(speeds), lo0=0.5 * speeds[0]), fs):
            w = [ramp(tas_p, *up)] + ([ramp(tas_p, *dn)] if dn else [])
            vol = [e('sqrt', e('product', *w) if len(w) > 1 else w[0]), SIGMA] + ([v(gain)] if gain < 0.9999 else [])
            g = ([p(INTERNAL)] if view == 'cockpit' else [e('not', p(INTERNAL))]) + [e('not', p(CRASHED))] + gates(tas_p, a, b)
            out.append(entry('gearair', f'pww-gear-{f["airspeed_kt"]:g}kt-{view}', f'{SOUND_DIR}/{f["file"]}', g, vol,
                             None, None if view == 'cockpit' else m['exterior_distance_m']))
            used.append(f['file'])
    return out, used


def tyre_entries(bank, g_min):
    m = json.load(open(os.path.join(bank, 'manifest.json')))
    P = m['properties']
    speeds = m['speeds_kt']
    have = {(f['tyre'], f['groundspeed_kt'], f['view']): f for f in m['files']}
    out, used = [], []
    for t in m['tyres']:
        n = t['gear']
        raw = P['speed'].format(n=n).lstrip('/')
        sc = P['speed_scale']
        k = m['low_speed_power'][t['name']]
        load = e('sqrt', e('clip', e('product', p(P['load'].format(n=n).lstrip('/')),
                                     v(1.0 / (0.25 * t['static_compression_ft']))), '<clipMin>0</clipMin><clipMax>1</clipMax>'))
        for view in ('cockpit', 'side'):
            group = 'cockpit' if view == 'cockpit' else 'exterior'
            gain = 10.0 ** ((g_min - m['gain_db'][group]) / 20.0)
            for i, sp in enumerate(speeds):
                f = have.get((t['name'], sp, view))
                if f is None:
                    continue
                lo = speeds[i - 1] if i else 0.0
                rise = e('clip', e('sum', e('product', p(raw), v(sc / (sp - lo))), v(-lo / (sp - lo))),
                         '<clipMin>0</clipMin><clipMax>1</clipMax>')
                w = [e('pow', rise, v(0.5 * k)) if i == 0 else e('sqrt', rise)]
                g = [p(INTERNAL) if view == 'cockpit' else e('not', p(INTERNAL)), e('not', p(CRASHED)),
                     p(P['wow'].format(n=n).lstrip('/'))]
                if i == 0:      # where the slowest falls to -60 dB, as the speed to its power
                    g.append(gt(raw, sp * 10.0 ** (-6.0 / k) / sc))
                else:
                    g.append(gt(raw, lo / sc))
                if i < len(speeds) - 1:
                    hi = speeds[i + 1]
                    w.append(e('sqrt', e('clip', e('sum', e('product', p(raw), v(sc / (sp - hi))), v(-hi / (sp - hi))),
                                         '<clipMin>0</clipMin><clipMax>1</clipMax>')))
                    g.append(lt(raw, hi / sc))
                vol = w + [load] + ([v(gain)] if gain < 0.9999 else [])
                s = entry('tyre', f'pww-tyre-{t["name"]}-{sp:g}kt-{view}', f'{SOUND_DIR}/{f["file"]}', g, vol, None,
                          None if view == 'cockpit' else m['exterior_distance_m'])
                out.append(s.replace('\t\t</tyre>\n', f'\t\t\t{position(t["position"])}\n\t\t</tyre>\n'))
                used.append(f['file'])
    return out, used


# ---------------------------------------------------------------- the file

def kept():
    """the generic set's entries worth keeping: the crash as it is (both views), the rain on the canopy in the cockpit
    only (its file is the windscreen's, as heard inside; the set has none for outside)"""
    root = ET.parse(os.path.join(SND, 'sound.xml')).getroot()
    out = []
    for el in root.find('fx'):
        if el.findtext('name') in KEEP:
            if el.findtext('name') == 'rain':
                both = el.find('condition').find('and')
                view = ET.Element('property')
                view.text = INTERNAL
                both.insert(0, view)
            s = ET.tostring(el, encoding='unicode').strip()
            out.append('\t\t' + s + '\n')
    return out


def main():
    rb = sys.argv[1] if len(sys.argv) > 1 else os.path.join(PWW, 'alouette3')
    eb = sys.argv[2] if len(sys.argv) > 2 else os.path.join(PWW, 'alouette3_engines')
    wb, gb, tb = (os.path.join(PWW, 'alouette3_' + k) for k in ('wind', 'gear', 'tyres'))
    banks = [rb, eb, wb, gb, tb]
    g_min = min(x for b in banks for x in json.load(open(os.path.join(b, 'manifest.json')))['gain_db'].values())
    door = doors(json.load(open(os.path.join(eb, 'manifest.json')))['openings'])
    r, used_r, dropped, raised = rotor_entries(rb, door, g_min)
    en, used_e = engine_entries(eb, door, g_min)
    wi, used_w = wind_entries(wb, g_min)
    ga, used_g = gear_entries(gb, g_min)
    ty, used_t = tyre_entries(tb, g_min)
    keep = kept()
    body = ('<?xml version="1.0"?>\n<!-- generated by fdm/al3sound.py from PyWopWop\'s banks (rotors, engine, gearbox, wind, '
            'gear, tyres) and the generic set\'s rain and crash: do not edit, run the generator -->\n<PropertyList>\n\t<fx>\n'
            + ''.join(r) + ''.join(en) + ''.join(wi) + ''.join(ga) + ''.join(ty) + ''.join(keep) + '\t</fx>\n</PropertyList>\n')
    open(OUT_XML, 'w').write(body)
    dst = os.path.join(SND, 'pywopwop')
    os.makedirs(dst, exist_ok=True)
    want = {}
    for b, names in ((rb, used_r), (eb, used_e), (wb, used_w), (gb, used_g), (tb, used_t)):
        for n in names:
            if n not in raised:
                want[n] = os.path.join(b, n)
    for n in os.listdir(dst):
        if n not in want and n not in raised:
            os.remove(os.path.join(dst, n))
    for n, (src, k) in raised.items():          # the slap loops the flight model may raise: written raised
        fs, y = wavfile.read(src)
        wavfile.write(os.path.join(dst, n), fs, np.round(y.astype(np.float64) * k).clip(-32768, 32767).astype(np.int16))
    copied = len(raised)
    for n, src in want.items():
        t = os.path.join(dst, n)
        if not os.path.exists(t) or os.path.getsize(t) != os.path.getsize(src) or os.path.getmtime(t) < os.path.getmtime(src):
            shutil.copy2(src, t)
            copied += 1
    size = sum(os.path.getsize(os.path.join(dst, n)) for n in list(want) + list(raised)) / 1e6
    print(f'{OUT_XML}: {len(r)} rotor loops, {len(en)} engine and gearbox loops, {len(wi)} wind, {len(ga)} gear, '
          f'{len(ty)} tyre loops, {len(keep)} kept sounds; '
          f'{len(want)} files ({size:.0f} MB, {copied} copied); {len(dropped)} masked layers left out')


if __name__ == '__main__':
    main()
