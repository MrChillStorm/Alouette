#!/usr/bin/env python3
"""The control Jacobian for the in-air start's trim loops (genal3.s_start_trim): at each airspeed, the model trimmed in
level flight at 1000 ft, 2200 kg, and the mean body accelerations (wdot, pdot, qdot, rdot) differentiated against the
collective, longitudinal, lateral and pedal inputs (harness units: 0..1 up, -1..1 forward, right, right). The loops
use its inverse, so that each control nulls its own acceleration without stirring the others (the collective's torque
yaws and rolls the aircraft, the pedals roll it).

The Jacobian is taken where the hold puts the aircraft, not at the trim: FlightGear starts it nose level, banked as in
the trim (Nasal/airstart.nas sets that bank; the script prints its table), so the four controls are first solved
there for zero wdot, pdot, qdot and rdot (the trim is the first guess). At high speed the two differ most: there the trim,
nose down near the rotor's limit, has a nearly singular Jacobian, whose inverse once gave the start loops five times
their gains.

  python3 startgains.py           prints START_JINV for genal3.py and BANK_DEG for airstart.nas
"""
import numpy as np
import harness as H

SPEEDS_KT = (0, 30, 60, 90, 105)      # to 105 kt: at 1000 ft, 2200 kg the retreating blade stalls by 115 (VNE 113)
H_STEP = 0.02
ACC = [2, 3, 4, 5]                        # wdot, pdot, qdot, rdot
SCALE = np.array([1, 10, 10, 10.0])


def jac(f, x, V):
    r0, _ = H.residual(f, x, V, 1000.0)
    J = np.zeros((4, 4))
    for j in range(4):
        xp = x.copy(); xp[j] += H_STEP
        rp, _ = H.residual(f, xp, V, 1000.0)
        J[:, j] = (rp[ACC] - r0[ACC]) / H_STEP
    return r0[ACC], J


def jacobian(f, V, x0=None, s0=None):
    """the trim, then the controls at the start's attitude (level nose, the trim's bank), and the Jacobian there;
    x0 and s0 are the previous speed's trim and start controls, the first guesses"""
    x, r, e = H.trim(f, V, alt_ft=1000.0, x0=x0, iters=30)
    assert np.abs(r * [1, 1, 1, 10, 10, 10]).max() < 0.02, 'no trim at %d kt: %s' % (V, np.round(r, 4))
    xt = x.copy()
    if s0 is not None:
        x[:4] = s0[:4]
    x[4] = 0.0                            # x[5], the roll, stays the trim's
    for it in range(30):
        r0, J = jac(f, x, V)
        if np.abs(r0 * SCALE).max() < 0.02:
            break
        dx = np.linalg.solve(J, -r0)
        dx *= min(1.0, 0.1 / np.abs(dx).max())
        x[:4] = np.clip(x[:4] + dx, [0, -1, -1, -1], [1, 1, 1, 1])
    else:
        raise RuntimeError('no start point at %d kt: %s' % (V, np.round(r0, 4)))
    return xt, x, J


if __name__ == '__main__':
    f = H.load(model='al3-2200')
    rows = []
    x = xs = None
    banks = []
    Vp = 0
    for V in SPEEDS_KT:
        for Vi in range(Vp + 10, V, 10):          # the trim walked up in 10 kt steps
            x = H.trim(f, Vi, alt_ft=1000.0, x0=x, iters=30)[0]
        x, xs, J = jacobian(f, V, x, xs)
        Vp = V
        banks.append(x[5])
        rows.append(np.linalg.inv(J))
        print('# %3d kt  trim %s  start %s' % (V, np.round(x, 3), np.round(xs[:4], 3)))
    print('START_JINV_KT = %r' % (SPEEDS_KT,))
    print('START_JINV = (   # [control][acceleration]: d(coll, lon, lat, ped) / d(wdot ft/s2, pdot, qdot, rdot rad/s2), per speed')
    for V, Ji in zip(SPEEDS_KT, rows):
        print('    (' + ', '.join('(' + ', '.join('%.5f' % v for v in row) + ')' for row in Ji) + '),   # %d kt' % V)
    print(')')
    print('var BANK_KT  = [%s];' % ', '.join('%d' % v for v in SPEEDS_KT))
    print('var BANK_DEG = [%s];   # Nasal/airstart.nas' % ', '.join('%.1f' % b for b in banks))
