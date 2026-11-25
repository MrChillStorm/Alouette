#!/usr/bin/env python3
"""The Alouette III's PyWopWop banks: the rotors, the engine and gearbox, the canopy's wind, the gear in the airflow and the
tyres rolling, written to ~/git/PyWopWop/out/alouette3*. al3sound.py wires them into the aircraft.

    python3 mkbanks.py [rotors|near|engines|enginecones|wind|gear|tyres ...]   (default: all of them, in this order; the
    rotors and the near bank take some 12 minutes each; the gear and tyres follow the near bank's gain)

The rotors: PyWopWop's helicopter conditions with the cruise at 100 kt, under the Alouette's 113 kt VNE (PyWopWop's own
is 120), and the ground run-up. Every bank's loops carry the rotor bank's gains (cockpit and outside), so that their
loudness against its loops is the physics'.

Wheels (al3data.py: [MI, GY, 3D]): 355 x 150-4 tyres on all three, 0.354 m across and 0.15 wide; the contacts and the
wheels' centres from the 3-D model, in PyWopWop's frame (x forward, y left, z up, from the main rotor hub). Each strut sits
0.09 m (0.295 ft) down at rest at 2200 kg (genal3.py's gear). The cockpit's transfer from a wheel through the structure
is the P2006T's recorded one (sprung gear, as the Alouette's), scaled to the Alouette's 2200 kg: an estimate, no
Alouette landing having been recorded.

The gear in the airflow (Fink's method): each main leg's oleo strut (about 1.1 m long, 0.06 m thick) and swing arm (1.1 m,
0.09 m deep) as one strut of their frontal area, 0.077 m by 2.1 m; the nose leg's fork and piston, 0.05 m by 0.6 m: read
off the 3-D model's parts, as no drawing gives them.
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PWW = os.path.expanduser('~/git/PyWopWop')
sys.path.insert(0, PWW)
sys.path.insert(0, HERE)
import al3data as U                         # noqa: E402

OUT = os.path.join(PWW, 'out')
HUB = U.HUB_YASIM
D_TYRE, W_TYRE = 0.354, 0.150
STATIC_FT = 0.09 * 3.28084


def hubwise(p):
    """the body frame (x forward, y left, z up, from the model's origin) less the hub: PyWopWop's frame"""
    return tuple(round(a - b, 4) for a, b in zip(p, HUB))


# FlightGear's gear order (genal3.py): 0 the left main, 1 the right main, 2 the nose
CONTACTS = [('left_main', 0, U.MAIN_WHEELS[0]), ('right_main', 1, U.MAIN_WHEELS[1]), ('nose', 2, U.NOSE_WHEEL)]


def view_distance():
    """the helicopter view's distance, m (/sim/chase-distance-m of the aircraft's set file): the outside loops' distance"""
    import re
    t = open(os.path.join(HERE, '..', 'Alouette-3', 'Alouette-3-set.xml')).read()
    return abs(float(re.search(r'<chase-distance-m[^>]*>\s*(-?[\d.]+)', t).group(1)))


VIEW = view_distance()


def rotor_gains():
    return json.load(open(os.path.join(OUT, 'alouette3', 'manifest.json')))['gain_db']


def outside_gains():
    """the cockpit's gain of the rotor bank and the outside's of the near bank: the gear's and tyres' loops are rendered at
    the view's distance, against the near bank's loops"""
    return dict(rotor_gains(), exterior=json.load(open(os.path.join(OUT, 'alouette3_near', 'manifest.json')))['gain_db']['exterior'])


def runups(distance=None):
    """PyWopWop's run-up conditions (on the ground at flat pitch, by rotor speed) with a node wherever neighbours stand more
    than MAX_PITCH apart: a loop follows its rotor's speed up to its pitch headroom, and the next takes over there"""
    from pywopwop import bank as B
    from pywopwop.bank import BankCondition
    return [BankCondition(f"runup_{p:g}", 0.0, 0.0, f"on the ground at flat pitch, rotor at {p:g} %", ground_clearance_m=0.0,
                          listener_distance_m=50.0, rpm_percent=p, ground_run=True)
            for p in finer(tuple(float(c.rpm_percent) for c in B.RUNUP_CONDITIONS))]


def rotors():
    from dataclasses import replace
    from pywopwop.bank import HELICOPTER_CONDITIONS, BankOptions, make_bank
    from pywopwop.model import load_aircraft
    conds = [replace(c, airspeed_kt=100.0, description='level cruise (VNE 113 kt)') if c.name == 'cruise' else c
             for c in HELICOPTER_CONDITIONS]
    make_bank(load_aircraft('alouette3'), os.path.join(OUT, 'alouette3'),
              BankOptions(conditions=conds + runups(), runup=False, pitch_headroom=MAX_PITCH))


def near():
    """the rotors' outside loops by sound cone, at the view's distance (PyWopWop's --cones): one loop per layer, made where
    it is loudest, and its cone fitted to the levels round the aircraft; the same conditions as the rotors"""
    from dataclasses import replace
    from pywopwop.bank import HELICOPTER_CONDITIONS, BankOptions, make_bank
    from pywopwop.model import load_aircraft
    conds = [replace(c, airspeed_kt=100.0, description='level cruise (VNE 113 kt)') if c.name == 'cruise' else c
             for c in HELICOPTER_CONDITIONS]
    make_bank(load_aircraft('alouette3'), os.path.join(OUT, 'alouette3_near'),
              BankOptions(conditions=conds + runups(), runup=False, cones=True, distance_m=VIEW, observers=[], cockpit=False,
                          pitch_headroom=MAX_PITCH))


def enginecones():
    """the engine's outside loops by sound cone, at the view's distance: compressor, core and turbine each one loop with its
    own cone (the intake's forward, the exhaust's aft)"""
    from pywopwop.enginecones import make_engine_cone_bank
    engine_grids()
    make_engine_cone_bank('alouette3', os.path.join(OUT, 'alouette3_enginecones'), distance=VIEW)


MAX_PITCH = 1.5         # FlightGear's practical pitch ceiling (SimGear's hard one is 2.0): neighbouring loops stand within it


def finer(grid, max_ratio=MAX_PITCH):
    """the grid with geometric nodes inserted wherever neighbours are further apart than max_ratio, so that a loop never
    needs more than max_ratio of its own speed to meet the next one"""
    import math
    out = [grid[0]]
    for a, b in zip(grid, grid[1:]):
        k = max(1, math.ceil(round(math.log(b / a) / math.log(max_ratio), 6)))
        out += [round(a * (b / a) ** (i / k), 1) for i in range(1, k)] + [b]
    return tuple(out)


def engine_grids():
    """PyWopWop's engine states with the nodes the pitch ceiling needs (its constants, as the engine bank reads them)"""
    from pywopwop import engine as E
    for name in ('SIGNATURE_LIT', 'SIGNATURE_COLD', 'SIGNATURE_STARTER', 'LIT_NG', 'COLD_NG', 'STARTER_NG'):
        setattr(E, name, finer(getattr(E, name)))


def engines():
    from pywopwop.engine import make_engine_bank
    engine_grids()
    make_engine_bank('alouette3', os.path.join(OUT, 'alouette3_engines'))


def wind():
    from pywopwop.model import load_aircraft
    from pywopwop.wind import make_wind_bank
    make_wind_bank(load_aircraft('alouette3'), os.path.join(OUT, 'alouette3_wind'),
                   speeds=(20.0, 40.0, 60.0, 80.0, 100.0, 120.0, 140.0), gain_db=rotor_gains()['cockpit'])


def gear():
    from pywopwop.gear import make_gear_bank
    from pywopwop.model import load_aircraft
    legs = []
    for name, n, c in CONTACTS:
        centre = (c[0], c[1], c[2] + D_TYRE / 2)
        strut = (0.05, 0.6) if name == 'nose' else (0.077, 2.1)
        legs.append(dict(position=hubwise(centre), wheel_m=D_TYRE, strut_m=strut[0], strut_length_m=strut[1]))
    make_gear_bank(load_aircraft('alouette3'), os.path.join(OUT, 'alouette3_gear'), legs, outside_gains(),
                   speeds=(20.0, 40.0, 60.0, 80.0, 100.0, 120.0, 140.0), exterior_m=VIEW)


def tyres():
    from pywopwop import tyre as T
    from pywopwop.model import load_aircraft
    ty = [dict(name=name, gear=n, position=hubwise(c), width_m=W_TYRE, diameter_m=D_TYRE,
               static_compression_ft=round(STATIC_FT, 4)) for name, n, c in CONTACTS]
    m = T.make_tyre_bank(load_aircraft('alouette3'), os.path.join(OUT, 'alouette3_tyres'), ty, outside_gains(), 'p2006t',
                         airframe_kg=U.MTOW_KG, exterior_m=VIEW)
    print('%d files; below the lowest speed each falls as the speed to %s' % (len(m['files']), m['low_speed_power']))


STEPS = dict(rotors=rotors, near=near, engines=engines, enginecones=enginecones, wind=wind, gear=gear, tyres=tyres)

if __name__ == '__main__':
    for s in sys.argv[1:] or STEPS:
        print('---', s, flush=True)
        STEPS[s]()
