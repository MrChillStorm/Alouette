#!/usr/bin/env python3
"""Quasi-steady 6-DOF derivatives and modes, as the Bo 105 model's derivatives.py (there are no identified Alouette III
derivatives to compare them with: the report sets them beside the textbook articulated rotor's and the Bo 105's).

The model (al3-2200: 2200 kg, CG 2930 mm) is trimmed in level flight at each speed, 1000 ft; then, the airframe frozen
and the rotor running, each body velocity, rate and control is moved either way of trim and the aerodynamic forces
and moments (rotor included) are averaged over two revolutions once the rotor and its wake have settled: central
differences, in body axes at the CG, the moments divided through the inertia tensor (Ixz included). Controls in % of
travel: longitudinal + aft, lateral + right, pedal + right, collective + up, each 100 % the full travel of the
model's control. The modes follow from the 8-state linear model (u v w p q r phi theta) with the trim's kinematic and
gravity terms.

  python3 derivatives.py [kt ...]       (default 0 60 100)"""
import math, sys, json
import numpy as np
import harness as H

G = 9.80665
FT = 0.3048
LBF = 4.448222
SLUGFT2 = 1.355818          # slug ft^2 -> kg m^2
ALT_FT = 1000.0
SPEEDS_KT = (0, 60, 100)
STEP = {'u': 1.0, 'v': 1.0, 'w': 0.5, 'p': 0.05, 'q': 0.05, 'r': 0.05, 'col': 2.0, 'lon': 2.0, 'lat': 2.0, 'ped': 2.0}
KEYS = ['forces/fbx-aero-lbs', 'forces/fby-aero-lbs', 'forces/fbz-aero-lbs',
        'moments/l-aero-lbsft', 'moments/m-aero-lbsft', 'moments/n-aero-lbsft']


def loads(f, trim, d, settle=1.0, revs=2):
    """forces (N) and moments (N m) at the CG with the state and control offsets d (SI, % for controls)"""
    coll, lon, lat, ped = trim['x'][:4]
    H.controls(f, coll + 0.01 * d.get('col', 0), lon - 0.02 * d.get('lon', 0), lat + 0.02 * d.get('lat', 0), ped + 0.02 * d.get('ped', 0))
    f['ic/theta-deg'] = trim['theta']; f['ic/phi-deg'] = trim['phi']; f['ic/psi-true-deg'] = 0.0
    f['ic/h-sl-ft'] = ALT_FT; f['ic/terrain-elevation-ft'] = H.TERRAIN_FT
    f['ic/u-fps'] = trim['u'] + d.get('u', 0) / FT
    f['ic/v-fps'] = trim['v'] + d.get('v', 0) / FT
    f['ic/w-fps'] = trim['w'] + d.get('w', 0) / FT
    f['ic/p-rad_sec'] = d.get('p', 0); f['ic/q-rad_sec'] = d.get('q', 0); f['ic/r-rad_sec'] = d.get('r', 0)
    f.run_ic()
    dt = f['simulation/dt']
    for _ in range(int(settle / dt)):
        f.run()
    n = int(round(revs * 2 * math.pi / f['rotor/omega-rad_sec'] / dt))
    w = np.sin(np.pi * (np.arange(n) + 0.5) / n) ** 2
    w /= w.sum()
    acc = np.zeros(6)
    for i in range(n):
        f.run()
        acc += w[i] * np.array([f[k] for k in KEYS])
    acc[:3] *= LBF
    acc[3:] *= LBF * FT
    return acc


def inertia(f):
    """mass (kg) and the inertia tensor (kg m^2) at the CG, body axes"""
    m = f['inertia/mass-slugs'] * 14.5939
    ixx, iyy, izz = (f['inertia/i%s-slugs_ft2' % a] * SLUGFT2 for a in ('xx', 'yy', 'zz'))
    ixz = f['inertia/ixz-slugs_ft2'] * SLUGFT2
    return m, np.array([[ixx, 0, -ixz], [0, iyy, 0], [-ixz, 0, izz]])


def model_derivatives(V_kt, f=None):
    f = f or H.load(model='al3-2200')
    x, res, ex = H.trim(f, V_kt, alt_ft=ALT_FT)
    f['rotor/accel-hold'] = 1           # the rotor keeps the trim's accelerations: quasi-steady, no body acceleration
    trim = dict(V=V_kt, x=[float(v) for v in x], theta=f['attitude/theta-deg'], phi=f['attitude/phi-deg'],
                u=f['velocities/u-fps'], v=f['velocities/v-fps'], w=f['velocities/w-fps'], res=float(np.abs(res).max()))
    m, I = inertia(f)
    Iinv = np.linalg.inv(I)
    D = {}
    for var, h in STEP.items():
        dF = (loads(f, trim, {var: h}) - loads(f, trim, {var: -h})) / (2 * h)
        for i, ax in enumerate('XYZ'):
            D[ax + var] = dF[i] / m
        for i, ax in enumerate('LMN'):
            D[ax + var] = float(Iinv[i] @ dF[3:])
    f['rotor/accel-hold'] = 0
    trim.update(mass=m, I=I.tolist())
    return D, trim


def modes(D, trim):
    """eigenvalues of the 8-state model (u v w p q r phi theta), SI"""
    U0, V0, W0 = trim['u'] * FT, trim['v'] * FT, trim['w'] * FT
    th, ph = math.radians(trim['theta']), math.radians(trim['phi'])
    g = lambda k: D.get(k) or 0.0
    A = np.zeros((8, 8))
    for i, ax in enumerate('XYZLMN'):
        for j, var in enumerate('uvwpqr'):
            A[i, j] = g(ax + var)
    A[0, 4] -= W0; A[0, 5] += V0; A[0, 7] = -G * math.cos(th)
    A[1, 3] += W0; A[1, 5] -= U0; A[1, 6] = G * math.cos(th) * math.cos(ph); A[1, 7] = -G * math.sin(th) * math.sin(ph)
    A[2, 3] -= V0; A[2, 4] += U0; A[2, 6] = -G * math.cos(th) * math.sin(ph); A[2, 7] = -G * math.sin(th) * math.cos(ph)
    A[6, 3] = 1.0; A[6, 4] = math.sin(ph) * math.tan(th); A[6, 5] = math.cos(ph) * math.tan(th)
    A[7, 4] = math.cos(ph); A[7, 5] = -math.sin(ph)
    return np.linalg.eig(A)


def describe(ev):
    """[damping ratio, natural frequency rad/s] for the oscillatory modes, (inverse time constant 1/s) for the real ones"""
    out = []
    for e in sorted(ev, key=lambda z: (abs(z.imag) < 1e-9, -abs(z))):
        if e.imag > 1e-9:
            wn = abs(e); out.append('[%.2f, %.2f]' % (-e.real / wn, wn))
        elif abs(e.imag) < 1e-9:
            out.append('(%.2f)' % -e.real)
    return out


def run(speeds=SPEEDS_KT):
    f = H.load(model='al3-2200')
    out = []
    for V in speeds:
        D, trim = model_derivatives(V, f)
        ev, vec = modes(D, trim)
        out.append(dict(V=V, D=D, trim=trim, eig=[[float(e.real), float(e.imag)] for e in ev],
                        vec=[[[float(c.real), float(c.imag)] for c in vec[:, k]] for k in range(8)]))
        print('%3.0f kt: pitch %.2f roll %.2f deg, residual %.3f' % (V, trim['theta'], trim['phi'], trim['res']))
        print('   ' + '  '.join('%s %.4g' % (k, v) for k, v in D.items()))
        print('   modes', ' '.join(describe(ev)))
    return out


if __name__ == '__main__':
    speeds = [float(a) for a in sys.argv[1:]] or SPEEDS_KT
    res = run(speeds)
    json.dump(res, open('out/derivatives.json', 'w'), indent=1)
