#!/usr/bin/env python3
"""The Alouette III's PyWopWop banks: the rotors, the engine and gearbox, the canopy's wind, the gear in the airflow and the
tyres rolling, written to ~/git/PyWopWop/out/alouette3*. al3sound.py wires them into the aircraft.

    python3 mkbanks.py [rotors|engines|wind|gear|tyres ...]       (default: all of them; the rotors take some 12 minutes)

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


def rotor_gains():
    return json.load(open(os.path.join(OUT, 'alouette3', 'manifest.json')))['gain_db']


def rotors():
    from dataclasses import replace
    from pywopwop.bank import HELICOPTER_CONDITIONS, BankOptions, make_bank
    from pywopwop.model import load_aircraft
    conds = [replace(c, airspeed_kt=100.0, description='level cruise (VNE 113 kt)') if c.name == 'cruise' else c
             for c in HELICOPTER_CONDITIONS]
    make_bank(load_aircraft('alouette3'), os.path.join(OUT, 'alouette3'), BankOptions(conditions=conds, runup=True))


def engines():
    from pywopwop.engine import make_engine_bank
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
    make_gear_bank(load_aircraft('alouette3'), os.path.join(OUT, 'alouette3_gear'), legs, rotor_gains(),
                   speeds=(20.0, 40.0, 60.0, 80.0, 100.0, 120.0, 140.0))


def tyres():
    from pywopwop import tyre as T
    from pywopwop.model import load_aircraft
    ty = [dict(name=name, gear=n, position=hubwise(c), width_m=W_TYRE, diameter_m=D_TYRE,
               static_compression_ft=round(STATIC_FT, 4)) for name, n, c in CONTACTS]
    m = T.make_tyre_bank(load_aircraft('alouette3'), os.path.join(OUT, 'alouette3_tyres'), ty, rotor_gains(), 'p2006t',
                         airframe_kg=U.MTOW_KG)
    print('%d files; below the lowest speed each falls as the speed to %s' % (len(m['files']), m['low_speed_power']))


STEPS = dict(rotors=rotors, engines=engines, wind=wind, gear=gear, tyres=tyres)

if __name__ == '__main__':
    for s in sys.argv[1:] or STEPS:
        print('---', s, flush=True)
        STEPS[s]()
