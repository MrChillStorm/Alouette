"""JSBSim harness for the Alouette III model: load, freeze the airframe, run the rotor, average over revolutions, trim.
(After the Bo 105 and UH-60 models' harnesses.)"""
import os, math
import jsbsim
import numpy as np
import al3data as U
HERE = os.path.dirname(os.path.abspath(__file__))
TERRAIN_FT = -10000.0      # trims: terrain far below (out of ground effect)


def load(root=None, dt=None, model='alouette3', hold_rpm=True):     # dt: the rate FlightGear runs it (al3data.FDM_HZ)
    root = root or os.environ.get('AL3_ROOT') or os.path.join(HERE, 'root')
    f = jsbsim.FGFDMExec(root)
    f.set_debug_level(0)
    f['/controls/engines/engine[0]/throttle'] = 0.5
    f['/sim/freeze/fuel'] = 1.0
    # the step before the model, as FlightGear does: the filters (the boost servos, the cockpit vibration's) take
    # their channel's step when they are built
    f.set_dt(dt if dt is not None else 1.0 / U.FDM_HZ)
    if model == 'alouette3':
        f.load_model(model)
    else:                                  # a loading variant in the same aircraft directory
        f.set_aircraft_path('aircraft/alouette3')
        f.load_model(model, False)
    f['drive/hold-rpm'] = 1.0 if hold_rpm else 0.0
    return f


def freeze(f, on=True):
    for k in ('simulation/integrator/rate/rotational', 'simulation/integrator/rate/translational',
              'simulation/integrator/position/rotational', 'simulation/integrator/position/translational'):
        f[k] = 0 if on else 1


def controls(f, coll=0.5, lon=0.0, lat=0.0, ped=0.0):
    """coll 0..1 (up), lon -1..1 (+ forward), lat -1..1 (+ right), ped -1..1 (+ right)"""
    f['/controls/engines/engine[0]/throttle'] = coll
    f['fcs/elevator-cmd-norm'] = lon; f['fcs/aileron-cmd-norm'] = lat; f['fcs/rudder-cmd-norm'] = -ped


ACC = ['accelerations/udot-ft_sec2', 'accelerations/vdot-ft_sec2', 'accelerations/wdot-ft_sec2',
       'accelerations/pdot-rad_sec2', 'accelerations/qdot-rad_sec2', 'accelerations/rdot-rad_sec2']


def residual(f, x, V_kt, alt_ft=1000.0, gamma=0.0, settle_s=1.5, revs=2, wings_level=False, beta=0.0, extra=(), terrain_ft=None, ned=None):
    """x = coll, lon, lat, ped, alpha (pitch attitude in the hover), and roll (or sideslip when wings_level);
    Hann-weighted mean of the body accelerations over whole rotor revolutions, and of the extra properties"""
    coll, lon, lat, ped, th, ph = x
    controls(f, coll, lon, lat, ped)
    f['ic/psi-true-deg'] = 0.0
    f['ic/phi-deg'] = 0.0 if wings_level else ph
    if ned is not None:
        # a prescribed velocity, north (forward) and down, ft/s: the pitch attitude is the trim's variable
        f['ic/vt-kts'] = 0.0; f['ic/beta-deg'] = 0.0
        f['ic/theta-deg'] = th
        f['ic/vn-fps'] = ned[0]; f['ic/ve-fps'] = 0.0; f['ic/vd-fps'] = ned[1]
    else:
        f['ic/vt-kts'] = V_kt
        f['ic/beta-deg'] = ph if wings_level else beta
        if V_kt < 1.0:
            f['ic/theta-deg'] = th                 # hovering: the pitch attitude (no angle of attack)
        else:
            f['ic/gamma-deg'] = gamma; f['ic/alpha-deg'] = th
    f['ic/h-sl-ft'] = alt_ft; f['ic/terrain-elevation-ft'] = TERRAIN_FT if terrain_ft is None else terrain_ft
    f['ic/p-rad_sec'] = 0; f['ic/q-rad_sec'] = 0; f['ic/r-rad_sec'] = 0
    f.run_ic()
    for _ in range(int(settle_s / f['simulation/dt'])):
        f.run()
    n = int(round(revs * 2 * math.pi / f['rotor/omega-rad_sec'] / f['simulation/dt']))
    w = np.sin(np.pi * (np.arange(n) + 0.5) / n) ** 2
    w /= w.sum()
    acc = np.zeros(6); ex = np.zeros(len(extra))
    for i in range(n):
        f.run()
        acc += w[i] * np.array([f[k] for k in ACC])
        if extra:
            ex += w[i] * np.array([f[k] for k in extra])
    return acc, ex


def trim(f, V_kt, alt_ft=1000.0, gamma=0.0, x0=None, tol=None, iters=14, verbose=False, wings_level=False, beta=0.0,
         extra=('aero/rotor/power-hp',), terrain_ft=None, ned=None):
    """Newton trim of collective, cyclic, pedals, angle of attack and roll for steady straight flight (airframe frozen,
    rotor running), bounded, with a backtracking line search. Returns x, residuals, extra means."""
    f['ctl/init'] = 1
    freeze(f)
    lo = np.array([0.0, -1, -1, -1, -25, -25]); hi = np.array([1.0, 1, 1, 1, 25, 25])
    x = np.array(x0 if x0 is not None else [0.5, 0.0, 0.0, 0.0, 0.0, 0.0], float)
    h = np.array([0.01, 0.01, 0.01, 0.01, 0.3, 0.3])
    scale = np.array([1, 1, 1, 10, 10, 10.0])
    cost = lambda r: float(np.abs(r * scale).max())
    r0, e0 = residual(f, x, V_kt, alt_ft, gamma, wings_level=wings_level, beta=beta, extra=extra, terrain_ft=terrain_ft, ned=ned)
    for it in range(iters):
        if verbose:
            print(it, np.round(x, 4), np.round(r0, 4), np.round(e0, 1))
        if cost(r0) < (tol or 0.02):
            break
        J = np.zeros((6, 6))
        for j in range(6):
            xp = x.copy(); xp[j] += h[j]
            J[:, j] = (residual(f, xp, V_kt, alt_ft, gamma, wings_level=wings_level, beta=beta, terrain_ft=terrain_ft, ned=ned)[0] - r0) / h[j]
        if not (np.all(np.isfinite(J)) and np.all(np.isfinite(r0))):
            break
        dx = np.linalg.lstsq(J * scale[:, None], -r0 * scale, rcond=None)[0]
        lim = np.array([0.05, 0.1, 0.1, 0.1, 3, 3])
        k = np.max(np.abs(dx) / lim)
        if k > 1:
            dx /= k
        step = 1.0
        for _ in range(6):
            xn = np.clip(x + step * dx, lo, hi)
            rn, en = residual(f, xn, V_kt, alt_ft, gamma, wings_level=wings_level, beta=beta, extra=extra, terrain_ft=terrain_ft, ned=ned)
            if cost(rn) < cost(r0):
                break
            step *= 0.5
        x, r0, e0 = xn, rn, en
    return x, r0, e0


def deg(x):
    """trim controls in blade angles: collective at the shaft, cyclic, tail rotor pitch (deg)"""
    coll, lon, lat, ped = x[:4]
    return (U.COLL_ROOT[0] + (U.COLL_ROOT[1] - U.COLL_ROOT[0]) * coll,
            (U.B1S_RANGE[0] + U.B1S_RANGE[1]) / 2 + (U.B1S_RANGE[1] - U.B1S_RANGE[0]) / 2 * lon,
            (U.A1S_RANGE[0] + U.A1S_RANGE[1]) / 2 + (U.A1S_RANGE[1] - U.A1S_RANGE[0]) / 2 * lat,
            (U.TR_PITCH_RANGE[0] + U.TR_PITCH_RANGE[1]) / 2 + (U.TR_PITCH_RANGE[1] - U.TR_PITCH_RANGE[0]) / 2 * ped)


def pct(x):
    """controls in % of travel: collective from down, longitudinal from full forward (TR-75-50: F/A stick %), lateral
    and pedal from full left"""
    coll, lon, lat, ped = x[:4]
    return 100 * coll, 50 * (1 - lon), 50 * (1 + lat), 50 * (1 + ped)


def coll_07(x):
    """collective pitch at 0.7 R (deg), as the cockpit's pitch indicator"""
    return deg(x)[0] + U.twist(0.7 * U.R)


def set_weight(f, kg):
    """the test configuration's gross weight, through its fuel (every load sits at its CG): freezes the fuel"""
    import genal3
    f['/sim/freeze/fuel'] = 1
    empty = f['inertia/weight-lbs'] - f['inertia/pointmass-weight-lbs[%d]' % genal3.PM_TANK]
    lvl = kg * U.KG_LB - empty
    f['/consumables/fuel/tank[0]/level-lbs'] = lvl
    f['inertia/pointmass-weight-lbs[%d]' % genal3.PM_TANK] = lvl
    return lvl


def wheels_above_ground_ft(f):
    """the CG's height above the main wheels' contact points, the gear unloaded (ft); the mass properties need a first
    run_ic (before it inertia/cg-z-in reads 0)"""
    if f['inertia/cg-z-in'] == 0.0 and f['simulation/sim-time-sec'] == 0.0:
        f.run_ic()
    wl = U.model_to_station(*U.MAIN_WHEELS[0])[2]
    return (f['inertia/cg-z-in'] - wl) / 12.0


ENGINE_POWER = ('drive/load-ftlb', 'rotor/omega-rad_sec')


def p_avail_kw(alt_ft, dT=0.0, rating='to'):
    """the engine's available power at the reduction gear (kW), ISA + dT: the rating [TE] (take-off 420, maximum
    continuous 405) up to the thermal maximum's lapse, 640 kW delta theta^LAPSE [artouste.py]"""
    import artouste
    from atmosphere import isa
    T, P, rho = isa(alt_ft * 0.3048, dT)
    th, dl = T / 288.15, P / 101325.0
    return min(U.P_TO_KW if rating == 'to' else U.P_MCP_KW, U.P_THERMAL_KW * dl * th ** artouste.LAPSE_THETA)
