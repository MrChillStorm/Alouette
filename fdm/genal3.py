#!/usr/bin/env python3
"""Generate the Alouette III JSBSim model: articulated main rotor blade by blade, articulated tail rotor, airframe,
single-shaft engine and drive.

  python3 genal3.py OUTDIR [--fg]   writes OUTDIR/alouette3-jsb.xml (or alouette3.xml) and OUTDIR/Systems/al3-*.xml

Main rotor: the 3 blades of the fully articulated rotor are simulated one by one, each a rigid blade on coincident
flap and lag hinges 0.119 m from the shaft with a lag damper (as the UH-60 model's blades), its flapping and lead-lag
integrated in the FCS at the FDM rate. Each blade is cut into 8 segments of equal annulus area with the tables of its NACA
63A011.5 section (data/sections.py: C-81's 0012 equations with the 63A's differences from like-for-like wind tunnel
tests), at their own angle of attack and Mach number, in the air the full
kinematics give them: the aircraft's motion, the rotation, the blade's flapping and lag and the induced inflow
(Peters-He finite-state wake, with Johnson's vortex ring state model on its uniform state, as the MV-22B model's).
The blades' loads act on the hub at their hinges; their torque about the shaft goes to the rotor speed, solved with
the blades' lag accelerations (as the Bo 105 model's), and the airframe gets the drive torque's reaction.

The Alouette's rotor turns clockwise seen from above and its tail rotor pushes from the right of the boom: the
mirror image of the UH-60's and Bo 105's layouts. The aerodynamics are worked out in the aircraft's mirror image
(al3data's note): the mirrored inputs are the 'mir/' properties, and the side force, rolling and yawing moments are
mirrored back on the axes. In the mirror the rotor turns anticlockwise, azimuth from the tail; a blade at psi lies
along e_r = (-cos psi, sin psi, 0) (shaft axes x forward, y right, z down along the shaft, 2.5 deg forward), and on
the real aircraft at psi clockwise from the tail.
"""
import json, math, os, re, sys
import numpy as np
import al3data as U
import artouste as engines
from xmlfun import *
from data import sections as SEC
from data import al3_fuselage as FUS
from data import heyson_ground as HG
import peters_he as PH
import mrwake
import vrs as VRS

G0 = 32.174
R2D = 180 / math.pi
INLINE = True
KEEP = {'drive/torque-pct', 'aero/rotor/a1-deg', 'aero/rotor/b1-deg', 'aero/rotor/power-hp', 'aero/rotor/thrust-lbs', 'aero/rotor/torque-ftlb',
        'aero/rotor/tr/power-hp', 'aero/rotor/tr/thrust-lbs', 'aero/rotor/tr/torque-ftlb', 'drive/loss-ftlb',
        'drive/mr-torque-ftlb', 'aero/rotor/vrs/nu-base', 'aero/rotor/vrs/dnu', 'aero/rotor/vrs/in', 'eng/fuel-flow-kgh',
        'eng/fuel-flow-pph', 'eng/n-pct', 'eng/rpm', 'eng/power-kw', 'drive/load-ftlb', 'eng/n', 'aero/rotor/vrs/z', 'aero/rotor/vrs/x',
        'aero/rotor/vrs/vh-fps', 'aero/rotor/vrs/nu', 'aero/rotor/ph/VTu', 'aero/rotor/ph/VT', 'aero/rotor/chi-deg', 'aero/rotor/a0-deg'}

# the inputs the aerodynamics see in the mirror image (their 'mir/' copies are worked out first in each frame)
MIRROR = ['velocities/v-aero-fps', 'velocities/p-aero-rad_sec', 'velocities/r-aero-rad_sec', 'velocities/p-rad_sec',
          'velocities/r-rad_sec', 'accelerations/pdot-rad_sec2', 'accelerations/rdot-rad_sec2', 'accelerations/Ny',
          'attitude/phi-rad', 'inertia/cg-y-in']


def mirror_name(n):
    return 'mir/' + n.replace('/', '-')


def mirror_text(t):
    for n in MIRROR:
        t = t.replace('<property>%s</property>' % n, '<property>%s</property>' % mirror_name(n))
    return t


def slap_table():
    """the blade slap's loudness over the rotor's advance ratio and inflow through the tip-path plane, outside and in
    the cockpit, relative to the sound bank's slap loops (PyWopWop's map, from the sound session); None without it"""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data', 'slap_al3.json')
    return json.load(open(path)) if os.path.exists(path) else None


def _odd(xs, ys):
    return [-x for x in reversed(xs[1:])] + list(xs), [-y for y in reversed(ys[1:])] + list(ys)


def _even(xs, ys):
    return [-x for x in reversed(xs[1:])] + list(xs), list(reversed(ys[1:])) + list(ys)


def _mirror180(xs, ys, negate):
    k = -1 if negate else 1
    pts = dict(zip(xs, ys))
    for x, y in zip(xs, ys):
        if 0 <= x < 90:
            pts[180 - x] = k * y
        if -90 < x <= 0:
            pts[-180 - x] = k * y
    out = sorted(pts)
    return out, [pts[x] for x in out]


# geometry in the mirror image (BL negated)
BL_TR_M = -U.BL_TR
I_BLADE_SHAFT = U.I_B + 2 * U.E_FLAP * U.S_B + U.W_BLADE / G0 * U.E_FLAP ** 2   # one rigid blade about the shaft
SRL = U.I_B + U.E_FLAP * U.S_B                       # lag: the blade's inertia coupling with the rotor speed


class Model:
    def __init__(s):
        s.g = Graph()
        s.ctl = FCS('Alouette III controls')
        s.rot = FCS('Alouette III rotor')
        s.air = FCS('Alouette III airframe')
        s.aero = s.g
        s.X, s.Y, s.Z, s.L, s.Mm, s.N = [], [], [], [], [], []
        s.segs, s.edges = U.segments()

    # ------------------------------------------------------------ controls
    def controls(s):
        """The pilot's controls to the swashplate and the tail rotor, through the hydraulic servos [HA]; no
        augmentation. FlightGear's trims shift the controls' centre. And the mirror image's inputs."""
        c = s.ctl
        c.declare('/controls/engines/engine[0]/throttle', 0.0)
        c.declare('/controls/al3/invert-collective', 0)
        c.declare('/controls/al3/collective', 0.0)
        c.declare('ctl/throttle-prev', -1.0)
        c.declare('ctl/init', 0)
        c.channel('mirror')
        for n in MIRROR:
            c.fn(mirror_name(n), NEG(p(n)), desc='mirror image')
        c.channel('pilot')
        c.fn('ctl/init-fast', IFTHEN(LT(p('simulation/sim-time-sec'), v(1.0)), v(1), MIN(v(1), MAX(v(0), p('ctl/init')))))
        # the collective lever, 0 down .. 1 up: /controls/al3/collective, which the in-air start, the test autopilot and
        # Amelia set too. The pilot's throttle axis sets it when the axis moves, inverted (1 - throttle) when
        # /controls/al3/invert-collective is set; otherwise the throttle is kept in step with the collective, so that
        # an input that steps the throttle from where it is (the keyboard, the mouse) starts from the lever's place
        thr, inv = p('/controls/engines/engine[0]/throttle'), p('/controls/al3/invert-collective')
        flip = lambda x: S(x, M(inv, D(v(1), M(v(2), x))))          # x, or 1 - x when inverted
        prev = p('ctl/throttle-prev')
        moved = c.fn('ctl/throttle-moved', MIN(GT(ABS(D(thr, prev)), v(1e-6)), GT(prev, v(-0.5))),
                     desc='the pilot moved the throttle axis since the last frame')
        c.fn('ctl/collective-next', MAX(v(0), MIN(v(1), IFTHEN(moved, flip(thr), p('/controls/al3/collective')))),
             out='/controls/al3/collective')
        c.fn('ctl/throttle-sync', flip(p('/controls/al3/collective')), out=['/controls/engines/engine[0]/throttle', 'ctl/throttle-prev'],
             desc='the throttle in step with the collective')
        coll = c.fn('ctl/coll-norm', p('/controls/al3/collective'), clip=('0', '1'),
                    desc='collective lever, 0 down .. 1 up (FlightGear: the throttle axis, inverted by option)')
        # the original aircraft's autopilot (Systems/autopilot.xml: heading and altitude holds, its commands in the old
        # model's control radians) and Amelia, the copilot who hovers for the hoist (Systems/amelia.xml, in stick units),
        # add to the pilot's controls as they did there
        for n in ('systems/amelia/pitch-cmd', 'systems/amelia/roll-cmd', 'systems/amelia/yaw-cmd',
                  'autopilot/ap-pitch-cmd', 'autopilot/ap-roll-cmd', 'autopilot/ap-yaw-cmd'):
            c.declare(n, 0)
        lon = c.fn('ctl/lon-norm', S(p('fcs/elevator-cmd-norm'), p('fcs/pitch-trim-cmd-norm'), p('systems/amelia/pitch-cmd'),
                                     M(v(1 / 0.170), p('autopilot/ap-pitch-cmd'))), clip=('-1', '1'), desc='+ forward')
        lat = c.fn('ctl/lat-norm', S(p('fcs/aileron-cmd-norm'), p('fcs/roll-trim-cmd-norm'), p('systems/amelia/roll-cmd'),
                                     M(v(1 / 0.068), p('autopilot/ap-roll-cmd'))), clip=('-1', '1'), desc='+ right')
        # JSBSim's rudder is positive left (FlightGear hands it the negated pedal): negated back
        ped = c.fn('ctl/ped-norm', NEG(S(p('fcs/rudder-cmd-norm'), p('fcs/yaw-trim-cmd-norm'), p('systems/amelia/yaw-cmd'),
                                         M(v(1 / 0.18), p('autopilot/ap-yaw-cmd')))), clip=('-1', '1'), desc='+ right')
        # fixed gear: retractable for JSBSim (so that it has gear/unit[n]/pos-norm, which the original aircraft's systems read),
        # held down
        c.fn('ctl/gear-down', v(1), out='gear/gear-pos-norm')
        c.declare('gear/NWS/engaged', 0)
        c.fn('ctl/nose-castered', LT(p('gear/NWS/engaged'), v(0.5)), out='gear/unit[2]/castered')
        c.channel('rigging')
        mid = lambda r: (r[0] + r[1]) / 2
        half = lambda r: (r[1] - r[0]) / 2
        c.fn('ctl/theta0-cmd-deg', S(v(U.COLL_ROOT[0]), M(v(U.COLL_ROOT[1] - U.COLL_ROOT[0]), coll)), desc='blade pitch at the shaft, deg')
        c.fn('ctl/B1s-cmd-deg', S(v(mid(U.B1S_RANGE)), M(v(half(U.B1S_RANGE)), lon)), desc='longitudinal cyclic, + forward')
        c.fn('ctl/A1s-cmd-deg', S(v(mid(U.A1S_RANGE)), M(v(half(U.A1S_RANGE)), lat)), desc='lateral cyclic, + right (the real aircraft)')
        c.fn('ctl/thetaTR-cmd-deg', S(v(mid(U.TR_PITCH_RANGE)), M(v(half(U.TR_PITCH_RANGE)), ped)),
             desc='tail rotor blade pitch, + thrust to the left (anti-torque); right pedal +')
        c.channel('servos')
        # hydraulic servos: no Alouette data; the UH-60's primary servo response (Ames' no-load test, TM-88360 app. A)
        for n in ('B1s', 'A1s', 'theta0', 'thetaTR'):
            c.second_order('ctl/%s-deg' % n, 'ctl/%s-cmd-deg' % n, (0, 0, 1), (0.0002081, 0.020, 1))
        c.channel('sensors')
        c.fn('sens/coll-pct', M(v(100), coll))

    # ------------------------------------------------------------ blade equations (shared by the rotor system and the hub loads)
    def flap_accel(s, k, Mflap, c, sn, body=True):
        """flap acceleration of blade k (UH-60 model [GH 5.1-28], with lag): aerodynamic moment, the blade's weight and
        the hub's specific force, gyroscopic terms, centrifugal stiffness, the lag damper's moment (none: it acts in the
        plane); body: with the airframe's angular acceleration"""
        b = 'rotor/b%d' % k
        Ie = U.I_B + U.E_FLAP * U.S_B
        W = p('rotor/omega-rad_sec')
        be = p(b + '/beta-rad')
        cf = NEG(M(S(M(v(U.I_B), COS(be), S(SQR(W), M(v(2), W, p(b + '/deltadot-rad_sec')))),
                     M(v(U.E_FLAP * U.S_B), COS(p(b + '/delta-rad')), SQR(W))), SIN(be)))
        terms = [Mflap, M(v(U.S_B), p('rotor/Az')), M(v(2 * Ie), W, D(M(p('rotor/ps'), c), M(p('rotor/qs'), sn))), cf]
        if body:
            terms.append(M(v(Ie), S(M(p('rotor/pds'), sn), M(p('rotor/qds'), c))))
        return Q(S(*terms), v(U.I_B))

    def lag_free(s, k, Mlag, c, sn):
        """the lag equation's right-hand side but for the hub's angular acceleration, times I_b [UH-60 model, GH
        5.1-31]: aerodynamic and damper moments, centrifugal stiffness through the hinge offset, Coriolis from flapping
        (a blade flapping up speeds up); in the plane the hub's specific force (its lateral and fore-aft components)"""
        b = 'rotor/b%d' % k
        W = p('rotor/omega-rad_sec')
        be, dl, dld = p(b + '/beta-rad'), p(b + '/delta-rad'), p(b + '/deltadot-rad_sec')
        cb, sb = COS(be), SIN(be)
        damper = NEG(M(v(U.LAG_C), dld))
        stop = p(b + '/lag-stop-ftlb')
        # the hub's in-plane specific force along the blade's motion (e_t = (s, c, 0)) loads its centre of mass
        at = S(M(sn, p('rotor/Ax')), M(c, p('rotor/Ay')))
        return S(Q(S(Mlag, damper, stop), SQR(cb)), NEG(M(Q(v(U.E_FLAP * U.S_B), cb), SQR(W), SIN(dl))),
                 M(v(2 * U.I_B), Q(sb, cb), p(b + '/betadot-rad_sec'), S(W, dld)), NEG(M(v(U.S_B), at)))

    # ------------------------------------------------------------ main rotor states (FCS)
    def rotor_states(s):
        r = s.rot
        ci, si = math.cos(U.SHAFT_TILT), math.sin(U.SHAFT_TILT)
        r.declare('rotor/omega-rad_sec', U.OMEGA)
        r.declare('rotor/psi-rad', 0.0)
        r.declare('rotor/tr/psi-rad', 0.0)
        for k in range(U.NB):
            for st in ('beta-rad', 'betadot-rad_sec', 'delta-rad', 'deltadot-rad_sec'):
                r.declare('rotor/b%d/%s' % (k, st), 0.0)
        r.declare('rotor/v0-fps', 0.0)
        r.declare('rotor/vrs-dv', 0.0)
        for st, tg in ((U.PH_COS, 'c'), (U.PH_SIN, 's')):
            for i in range(len(st)):
                r.declare('rotor/ph/%s%d' % (tg, i), 0.0)
        for k in range(U.NB_TR):
            r.declare('rotor/tr/b%d/beta-rad' % k, 0.0)
            r.declare('rotor/tr/b%d/betadot-rad_sec' % k, 0.0)
        r.declare('rotor/tr/v0-fps', 0.0)
        r.channel('shaft kinematics')
        r.fn('rotor/ps', S(M(v(ci), p('velocities/p-rad_sec')), M(v(si), p('velocities/r-rad_sec'))))
        r.fn('rotor/qs', p('velocities/q-rad_sec'))
        r.declare('rotor/accel-hold', 0)
        held = GT(p('rotor/accel-hold'), v(0.5))
        hold = lambda name, e: IFTHEN(held, p(name), e)
        r.fn('rotor/pds', hold('rotor/pds', S(M(v(ci), p('accelerations/pdot-rad_sec2')), M(v(si), p('accelerations/rdot-rad_sec2')))))
        r.fn('rotor/qds', hold('rotor/qds', p('accelerations/qdot-rad_sec2')))
        # the hub's specific force (body axes), the CG's plus the airframe's angular acceleration and centripetal terms
        # at the hub's arm, to shaft axes (as the Bo 105 model)
        hx, hy, hz = s.arm_cg(U.FS_MR, U.BL_MR, U.WL_MR)
        P_, Q_, R_ = p('velocities/p-rad_sec'), p('velocities/q-rad_sec'), p('velocities/r-rad_sec')
        Pd, Qd, Rd = p('accelerations/pdot-rad_sec2'), p('accelerations/qdot-rad_sec2'), p('accelerations/rdot-rad_sec2')
        wxr = (D(M(Q_, hz), M(R_, hy)), D(M(R_, hx), M(P_, hz)), D(M(P_, hy), M(Q_, hx)))
        cent = (D(M(Q_, wxr[2]), M(R_, wxr[1])), D(M(R_, wxr[0]), M(P_, wxr[2])), D(M(P_, wxr[1]), M(Q_, wxr[0])))
        ang = (D(M(Qd, hz), M(Rd, hy)), D(M(Rd, hx), M(Pd, hz)), D(M(Pd, hy), M(Qd, hx)))
        cg = (M(v(G0), p('accelerations/Nx')), M(v(G0), p('accelerations/Ny')), M(v(-G0), p('accelerations/Nz')))
        hub = [S(cg[i], ang[i], cent[i]) for i in range(3)]
        r.fn('rotor/Az', hold('rotor/Az', D(M(v(ci), hub[2]), M(v(si), hub[0]))), desc='hub specific force along the shaft (down +)')
        r.fn('rotor/Ax', hold('rotor/Ax', S(M(v(ci), hub[0]), M(v(si), hub[2]))), desc='hub specific force along the shaft x (forward +)')
        r.fn('rotor/Ay', hold('rotor/Ay', hub[1]))
        r.fn('rotor/rds-in', hold('rotor/rds-in', p('accelerations/rdot-rad_sec2')))
        r.fn('rotor/pds-in', hold('rotor/pds-in', p('accelerations/pdot-rad_sec2')))
        for k in range(U.NB):
            psi = S(p('rotor/psi-rad'), v(k * 2 * math.pi / U.NB))
            r.fn('rotor/b%d/ch' % k, COS(psi)); r.fn('rotor/b%d/sh' % k, SIN(psi))
        dt = p('simulation/dt')
        W = p('rotor/omega-rad_sec')
        init = GT(p('ctl/init-fast'), v(0.5))
        r.channel('flapping')
        for k in range(U.NB):
            b = 'rotor/b%d' % k
            bdd = r.fn(b + '/betaddot', s.flap_accel(k, p('aero/rotor/b%d/Mflap' % k), p(b + '/ch'), p(b + '/sh')), desc='flap acceleration')
            bd_new = S(p(b + '/betadot-rad_sec'), M(bdd, dt))
            b_new = S(p(b + '/beta-rad'), M(bd_new, dt))
            lo, hi = U.FLAP_STOP
            # flap stops: the blade stays on the stop and loses the velocity into it
            r.fn(b + '/betadot-next', IFTHEN(LT(b_new, v(lo)), MAX(bd_new, v(0)), IFTHEN(GT(b_new, v(hi)), MIN(bd_new, v(0)), bd_new)),
                 out=b + '/betadot-rad_sec')
            # a stopped rotor starts with its blades resting on the droop stops
            r.fn(b + '/beta-next', IFTHEN(M(init, LT(W, v(5))), v(lo), MAX(v(lo), MIN(v(hi), S(p(b + '/beta-rad'), M(p(b + '/betadot-rad_sec'), dt))))),
                 out=b + '/beta-rad')
        r.channel('lead-lag')
        for k in range(U.NB):
            b = 'rotor/b%d' % k
            lo, hi = U.LAG_STOP
            dl = p(b + '/delta-rad')
            # the lag stops: a stiff spring past them
            r.fn(b + '/lag-stop-ftlb', S(M(v(-20 * U.E_FLAP * U.S_B * U.OMEGA ** 2), MIN(v(0), D(dl, v(lo)))),
                                         M(v(-20 * U.E_FLAP * U.S_B * U.OMEGA ** 2), MAX(v(0), D(dl, v(hi))))))
            r.fn(b + '/Fl-free', s.lag_free(k, p('aero/rotor/b%d/Mlag' % k), p(b + '/ch'), p(b + '/sh')), desc='lag equation but for the hub\'s angular acceleration')
        # the rotor speed and the blades' lag accelerations solved together (engines.rotor_speed)
        engines.rotor_speed(r, S(*[p('rotor/b%d/Fl-free' % k) for k in range(U.NB)]), SRL, U.I_B, init)
        Wd = r.fn('rotor/omega-dot-abs', IFTHEN(init, v(0), D(p('drive/omega-dot'), p('drive/r-dot-shaft'))))
        # the blades start from their static lag in the first frames of a run (not after a reset of the initial conditions:
        # the trims keep the rotor's state from one evaluation to the next): the aerodynamic moment against the
        # centrifugal stiffness through the hinge offset
        r.declare('rotor/initialized', 0)
        first = GT(r.fn('rotor/blade-init', LT(p('rotor/initialized'), v(0.5))), v(0.5))
        r.fn('rotor/initialized-next', GT(p('simulation/sim-time-sec'), v(0.02)), out='rotor/initialized')
        for k in range(U.NB):
            b = 'rotor/b%d' % k
            ddd = r.fn(b + '/deltaddot', Q(D(p(b + '/Fl-free'), M(v(SRL), Wd)), v(U.I_B)), desc='lag acceleration (lead +)')
            dld, dl = p(b + '/deltadot-rad_sec'), p(b + '/delta-rad')
            lo, hi = U.LAG_STOP
            static = MAX(v(lo), MIN(v(hi), Q(p('aero/rotor/b%d/Mlag' % k), M(v(U.E_FLAP * U.S_B), MAX(SQR(W), v(25)), COS(p(b + '/beta-rad'))))))
            r.fn(b + '/deltadot-next', IFTHEN(first, v(0), S(dld, M(ddd, dt))), out=b + '/deltadot-rad_sec')
            r.fn(b + '/delta-next', IFTHEN(first, static, S(dl, M(p(b + '/deltadot-rad_sec'), dt))), out=b + '/delta-rad')
        s.inflow_states()
        s.tail_rotor_states()
        # the azimuths advance last, so that every rotor equation of a step sees the blades where their states are
        r.channel('rotor azimuth')
        psn = S(p('rotor/psi-rad'), M(p('rotor/omega-rad_sec'), p('simulation/dt')))
        r.fn('rotor/psi-next', IFTHEN(GT(psn, v(2 * math.pi)), D(psn, v(2 * math.pi)), psn), out='rotor/psi-rad')
        psn = S(p('rotor/tr/psi-rad'), M(v(U.GEAR_TR), p('rotor/omega-rad_sec'), p('simulation/dt')))
        r.fn('rotor/tr/psi-next', IFTHEN(GT(psn, v(2 * math.pi)), D(psn, v(2 * math.pi)), psn), out='rotor/tr/psi-rad')

    def inflow_states(s):
        """Peters-He finite-state wake (as the UH-60 and Bo 105 models): M a' + L^-1 V a = tau / 2 per state set, in the
        wake frame, its uniform state's mass flow parameter Johnson's in the vortex ring state (vrs()); the hub-frame
        harmonic pairs are turned into the wake frame, stepped semi-implicitly, and turned back."""
        r = s.rot
        r.channel('inflow')
        Xs = [0.05 * i for i in range(21)]
        VTu = s.vrs(Xs)
        for st, tg, kind in ((U.PH_COS, 'c', 'c'), (U.PH_SIN, 's', 's')):
            Li = [np.linalg.inv(PH.Lmat(st, x, kind)) for x in Xs]
            for i in range(len(st)):
                for kk in range(len(st)):
                    Vk = VTu if st[kk] == (1, 0) else p('aero/rotor/ph/V')
                    r.fn('rotor/ph/A%s%d-%d' % (tg, i, kk), M(T1('aero/rotor/ph/X', Xs, [float(L_[i, kk]) for L_ in Li]), Vk))
        h = M(MAX(p('rotor/omega-rad_sec'), v(1.0)), p('simulation/dt'))
        cd, sd = p('aero/rotor/ph/cd'), p('aero/rotor/ph/sd')
        rot = {1: (cd, sd), 2: (r.fn('rotor/ph/cd2', D(SQR(cd), SQR(sd))), r.fn('rotor/ph/sd2', M(v(2), cd, sd)))}
        sets = {'c': U.PH_COS, 's': U.PH_SIN}
        pair = {}
        for i, st in enumerate(U.PH_COS):
            if st[1] > 0 and st in U.PH_SIN:
                pair[('c', i)] = U.PH_SIN.index(st); pair[('s', U.PH_SIN.index(st))] = i

        def wake(tg, i, prop):
            rh = sets[tg][i][1]
            if rh == 0 or (tg, i) not in pair:
                return p(prop % (tg, i))
            cr, sr = rot[rh]; o = 's' if tg == 'c' else 'c'; po = p(prop % (o, pair[(tg, i)]))
            me = p(prop % (tg, i))
            return S(M(me, cr), M(po, sr)) if tg == 'c' else D(M(me, cr), M(po, sr))
        for tg, st in sets.items():
            for i in range(len(st)):
                r.fn('rotor/ph/w%s%d' % (tg, i), wake(tg, i, 'rotor/ph/%s%d'))
                r.fn('rotor/ph/wtau-%s%d' % (tg, i), wake(tg, i, 'aero/rotor/ph/tau-%s%d'))
        for tg, st in sets.items():
            for i, (j, rh) in enumerate(st):
                k_ = Q(h, v(2 / math.pi * PH.H(j, rh)))
                a = lambda i_, kk: p('rotor/ph/A%s%d-%d' % (tg, i_, kk))
                off = [M(a(i, kk), p('rotor/ph/w%s%d' % (tg, kk))) for kk in range(len(st)) if kk != i]
                num = S(p('rotor/ph/w%s%d' % (tg, i)), M(k_, D(M(v(0.5), p('rotor/ph/wtau-%s%d' % (tg, i))), S(*off) if off else v(0))))
                r.fn('rotor/ph/n%s%d' % (tg, i), Q(num, S(v(1), M(k_, a(i, i)))))
        for tg, st in sets.items():
            for i, (j, rh) in enumerate(st):
                if rh == 0 or (tg, i) not in pair:
                    r.fn('rotor/ph/%s%d-next' % (tg, i), p('rotor/ph/n%s%d' % (tg, i)), out='rotor/ph/%s%d' % (tg, i))
                else:
                    cr, sr = rot[rh]; o = 's' if tg == 'c' else 'c'; po = p('rotor/ph/n%s%d' % (o, pair[(tg, i)])); me = p('rotor/ph/n%s%d' % (tg, i))
                    r.fn('rotor/ph/%s%d-next' % (tg, i), D(M(me, cr), M(po, sr)) if tg == 'c' else S(M(po, sr), M(me, cr)), out='rotor/ph/%s%d' % (tg, i))
        r.fn('rotor/v0-out', MAX(v(0), M(v(math.sqrt(3) * U.R), p('rotor/ph/c0'), p('rotor/omega-rad_sec'))), out='rotor/v0-fps',
             desc='mean inflow, sqrt(3) x the first state, ft/s')

    def vrs(s, Xs):
        """Vortex ring state: Johnson's model of the mean inflow [W. Johnson, NASA/TP-2005-213477] (vrs.py), as the
        MV-22B model's. Where it departs from momentum theory, in descent below 0.2 vh and slower than 0.95 vh edgewise,
        the uniform wake state's mass flow parameter is set for his steady inflow, vh / nu (Vz / vh, Vx / vh), in place
        of momentum theory's sqrt(Vx^2 + (Vz + v)^2) (Vprime). vh is the wake's own: its steady uniform inflow times its
        mass flow, the uniform state's row of L times the forcing, so that its momentum theory is Glauert's in Vz / vh
        and Vx / vh exactly and Johnson's joins it at the region's edges. The wake states keep their own dynamics: while
        his total flow is down through the disc the mass flow is momentum theory's on the present inflow, offset to his
        steady state; his increment for the vortex ring follows with his time lag, tau = 0.7 / lambda_h revolutions,
        lambda_h the thrust's ideal. Returns the uniform state's mass flow parameter."""
        r = s.rot
        b_, w = 'aero/rotor', 'aero/rotor/vrs/'
        cd, sd = p(b_ + '/ph/cd'), p(b_ + '/ph/sd')
        Lc = [PH.Lmat(U.PH_COS, x, 'c') for x in Xs]
        terms = []
        for kk, (j, rh) in enumerate(U.PH_COS):
            tau = p(b_ + '/ph/tau-c%d' % kk)
            if rh:
                tau = S(M(tau, cd), M(p(b_ + '/ph/tau-s%d' % U.PH_SIN.index((j, rh))), sd))
            terms.append(M(T1(b_ + '/ph/X', Xs, [float(L_[0, kk]) for L_ in Lc]), tau))
        WR = p(b_ + '/ph/omega-r-fps')
        vh = r.fn(w + 'vh-fps', M(WR, SQ(MAX(v(1e-8), M(v(0.5 * math.sqrt(3)), S(*terms))))),
                  desc='hover induced velocity of the forcing: the steady uniform inflow times its mass flow, ft/s')
        r.fn(w + 'z', Q(NEG(p(b_ + '/Vhz-fps')), vh), desc='axial velocity (climb +) over vh')
        r.fn(w + 'x', Q(SQ(S(SQR(p(b_ + '/Vinplane-fps')), v(25.0))), vh), desc='edgewise velocity over vh, with the mass flow\'s floor')
        r.fn(w + 's', tag('pow', MAX(v(0), D(v(1), SQR(Q(p(w + 'x'), v(VRS.XM))))), v(0.2)), desc='the tables\' coordinate across (vrs.py)')
        xs, zs = VRS.edge_table()
        inside = r.fn(w + 'in', M(LT(p(w + 'z'), v(VRS.ZD)), LT(p(w + 'x'), v(VRS.XM)), GT(p(w + 'z'), T1(w + 'x', xs, zs))),
                      desc='in the region where Johnson\'s model departs from momentum theory')
        nb, dn = VRS.tables()
        nub = r.fn(w + 'nu-base', T2(w + 'z', w + 's', VRS.TAB_Z, VRS.TAB_S, nb), desc='the baseline\'s v / vh')
        dnu = r.fn(w + 'dnu', T2(w + 'z', w + 's', VRS.TAB_Z, VRS.TAB_S, dn), desc='the vortex ring\'s increment of v / vh')
        dv = p('rotor/vrs-dv')
        nu = r.fn(w + 'nu', S(nub, Q(dv, vh)), desc='Johnson\'s v / vh with the lagged vortex ring')
        Sj = SQ(S(SQR(p(w + 'x')), SQR(S(p(w + 'z'), nu))))
        VTj = IFTHEN(GT(S(p(w + 'z'), nu), v(0)), Q(S(p(b_ + '/Vprime-fps'), M(vh, D(Q(v(1), nu), Sj))), WR), Q(vh, M(WR, nu)))
        VTu = r.fn(b_ + '/ph/VTu', IFTHEN(inside, VTj, p(b_ + '/ph/VT')), desc='the uniform state\'s mass flow parameter')
        vhi = SQ(Q(MAX(p(b_ + '/thrust-lbs'), v(1.0)), M(v(2 * math.pi * U.R ** 2), p('atmosphere/rho-slugs_ft3'))))
        k = M(p('simulation/dt'), vhi, v(1 / (VRS.TAU_REV_LH * 2 * math.pi * U.R)))
        tgt = M(inside, vh, dnu)
        r.fn('rotor/vrs-dv-next', IFTHEN(GT(p('ctl/init-fast'), v(0.5)), tgt, Q(S(dv, M(k, tgt)), S(v(1), k))), out='rotor/vrs-dv')
        return VTu

    def tail_rotor_states(s):
        """The blade-element tail rotor's states (aerodynamics in tail_rotor()): the three blades' flapping, each on its
        own hinge [HA: individually articulated in flap], as the UH-60 model's tail rotor's blades (the hinge offset's
        centrifugal stiffness added), semi-implicit at the FDM rate (24.2 deg of azimuth a step at 1938 rpm and 480 Hz); and the
        uniform inflow (Pitt-Peters' momentum over the effective disc B^2 A, as the Bo 105 and UH-60 models'):
        (8 / 3 pi) R dv/dt + 2 V' v = T / (rho A). Its axial part goes to nearly nothing when the rotor moves into its
        own wake: then the induced velocity grows and the thrust falls away, its vortex ring state as momentum theory
        can't have it."""
        r = s.rot
        dt = p('simulation/dt')
        W = M(v(U.GEAR_TR), p('rotor/omega-rad_sec'))
        init = GT(p('ctl/init-fast'), v(0.5))
        r.channel('tail rotor flapping')
        cT, sT = math.cos(U.TOE_TR), math.sin(U.TOE_TR)
        # hub frame (mirror image: thrust t to the right, toed forward): a0 aft in the plane, b0 = t x a0 down
        t, a0, b0 = (sT, cT, 0.0), (-cT, sT, 0.0), (0.0, 0.0, 1.0)
        dot = lambda u, x: S(*[M(v(cu), xe) for cu, xe in zip(u, x) if cu != 0])
        rates = (p('velocities/p-rad_sec'), p('velocities/q-rad_sec'), p('velocities/r-rad_sec'))
        accs = (p('accelerations/pdot-rad_sec2'), p('accelerations/qdot-rad_sec2'), p('accelerations/rdot-rad_sec2'))
        wa, wb = r.fn('rotor/tr/wa', dot(a0, rates)), r.fn('rotor/tr/wb', dot(b0, rates))
        wad, wbd = r.fn('rotor/tr/wad', dot(a0, accs)), r.fn('rotor/tr/wbd', dot(b0, accs))
        I = U.I_BLADE_TR
        kcf = 1 + U.E_TR * U.S_BLADE_TR / I
        for k in range(U.NB_TR):
            b = 'rotor/tr/b%d' % k
            psi = S(p('rotor/tr/psi-rad'), v(k * 2 * math.pi / U.NB_TR))
            c, sn = COS(psi), SIN(psi)
            be = p(b + '/beta-rad')
            wr = S(M(c, wa), M(sn, wb)); wed = S(NEG(M(sn, wad)), M(c, wbd))
            bdd = r.fn(b + '/betaddot', S(Q(S(p('aero/rotor/tr/b%d/Mflap' % k), NEG(M(v(2 * I), W, wr))), v(I)),
                                         NEG(M(v(kcf), SQR(W), COS(be), SIN(be))), wed),
                       desc='flap acceleration: aerodynamic moment, gyroscopic, centrifugal stiffness, hub angular acceleration')
            bd_new = S(p(b + '/betadot-rad_sec'), M(bdd, dt))
            b_new = S(be, M(bd_new, dt))
            lim = U.FLAP_STOP_TR
            r.fn(b + '/betadot-next', IFTHEN(LT(b_new, v(-lim)), MAX(bd_new, v(0)), IFTHEN(GT(b_new, v(lim)), MIN(bd_new, v(0)), bd_new)),
                 out=b + '/betadot-rad_sec')
            r.fn(b + '/beta-next', IFTHEN(M(init, LT(W, v(20))), v(0), MAX(v(-lim), MIN(v(lim), S(be, M(p(b + '/betadot-rad_sec'), dt))))),
                 out=b + '/beta-rad')
        r.channel('tail rotor inflow')
        A = math.pi * U.R_TR ** 2 * U.TIP_LOSS_TR ** 2
        v0 = p('rotor/tr/v0-fps')
        Vm = r.fn('rotor/tr/Vprime-fps', SQ(S(SQR(p('aero/rotor/tr/Vinplane-fps')), SQR(S(p('aero/rotor/tr/Vaxial-fps'), v0)), v(25.0))),
                  desc='mass flow velocity through the disc (with a 5 ft/s floor, as the main rotor)')
        r.fn('rotor/tr/v0-next', S(v0, M(dt, Q(D(Q(p('aero/rotor/tr/thrust-lbs'), M(p('atmosphere/rho-slugs_ft3'), v(A))),
                                                M(v(2), Vm, v0)), v(8 / (3 * math.pi) * U.R_TR)))), out='rotor/tr/v0-fps')

    # ------------------------------------------------------------ geometry helpers (the mirror image's)
    def arm_rp(s, fs, bl, wl):
        return (Q(D(p('metrics/aero-rp-x-in'), v(fs)), v(12)), Q(D(v(bl), p('metrics/aero-rp-y-in')), v(12)), Q(D(p('metrics/aero-rp-z-in'), v(wl)), v(12)))

    def arm_cg(s, fs, bl, wl):
        return (Q(D(p('inertia/cg-x-in'), v(fs)), v(12)), Q(D(v(bl), p('inertia/cg-y-in')), v(12)), Q(D(p('inertia/cg-z-in'), v(wl)), v(12)))

    def wake(s, target, mirror=False):
        """the main rotor wake's induced air velocity over a target (ring-vortex tables, mrwake.py), body axes, to add
        to the target's velocity relative to the air; mirror: the target's mirror image across the plane of symmetry"""
        g, a = s.g, s.aero
        if not a.has('aero/rotor/psi-flow-deg'):
            ps = ATAN2(p('aero/rotor/uhy'), p('aero/rotor/uhx'))
            a.fn('aero/rotor/psi-flow-deg', M(v(R2D), IFTHEN(LT(ps, v(0)), S(ps, v(2 * math.pi)), ps)),
                 'direction of the in-plane flight path at the hub, from the nose to the right')
            a.fn('aero/rotor/psi-flow-mirror-deg', D(v(360), p('aero/rotor/psi-flow-deg')))
        mw = mrwake.load(target)
        psi = 'aero/rotor/psi-flow-mirror-deg' if mirror else 'aero/rotor/psi-flow-deg'
        tag_ = target + ('-m' if mirror else '')
        out = []
        for i, ax in enumerate('xyz'):
            tab = [[[c[i] for c in row] for row in t] for t in mw['k']]
            k = g.fn('aero/wake/%s/k%s' % (tag_, ax), T3('aero/rotor/chi-deg', psi, 'aero/rotor/a1-deg', mw['chi'], mw['psi'], mw['a1'], tab),
                     'main rotor induced air velocity over the %s, over its mean inflow' % target)
            sign = -1 if (mirror and ax == 'y') else 1
            out.append(M(v(-sign), k, p('aero/rotor/vi-fps')))
        return out

    def add(s, F, Mo):
        for lst, x in zip((s.X, s.Y, s.Z, s.L, s.Mm, s.N), tuple(F) + tuple(Mo)):
            if x is not None:
                lst.append(x)

    # ------------------------------------------------------------ main rotor aerodynamics and hub loads
    def main_rotor(s):
        g = s.g
        ci, si = math.cos(U.SHAFT_TILT), math.sin(U.SHAFT_TILT)
        to_s = lambda bx, by, bz: (S(M(v(ci), bx), M(v(si), bz)), by, S(M(v(-si), bx), M(v(ci), bz)))
        rx, ry, rz = s.arm_cg(U.FS_MR, U.BL_MR, U.WL_MR)
        rx, ry, rz = [g.fn('aero/rotor/hub-r%s-ft' % a, e) for a, e in zip('xyz', (rx, ry, rz))]
        Pa, Qa, Ra = p('velocities/p-aero-rad_sec'), p('velocities/q-aero-rad_sec'), p('velocities/r-aero-rad_sec')
        vb = (S(p('velocities/u-aero-fps'), D(M(Qa, rz), M(Ra, ry))),
              S(p('velocities/v-aero-fps'), D(M(Ra, rx), M(Pa, rz))),
              S(p('velocities/w-aero-fps'), D(M(Pa, ry), M(Qa, rx))))
        Vhx, Vhy, Vhz = [g.fn('aero/rotor/Vh%s-fps' % a, e) for a, e in zip('xyz', to_s(*vb))]
        ps_a = g.fn('aero/rotor/ps-aero', S(M(v(ci), Pa), M(v(si), Ra)))
        qs_a = Qa
        rs_a = g.fn('aero/rotor/rs-aero', S(M(v(-si), Pa), M(v(ci), Ra)))
        Vip = g.fn('aero/rotor/Vinplane-fps', SQ(S(SQR(Vhx), SQR(Vhy), v(1e-6))))
        wdir = MIN(v(1), Q(Vip, v(5.0)))
        g.fn('aero/rotor/uhx', S(M(wdir, Q(Vhx, Vip)), D(v(1), wdir)))
        g.fn('aero/rotor/uhy', M(wdir, Q(Vhy, Vip)))
        v0 = p('rotor/v0-fps')
        Vn = g.fn('aero/rotor/Vnormal-fps', D(v0, Vhz), 'flow through the disc (down +)')
        g.fn('aero/rotor/Vprime-fps', SQ(S(SQR(Vip), SQR(Vn), v(25.0))), 'mass flow velocity (5 ft/s floor)')
        chi = g.fn('aero/rotor/chi-rad', ATAN2(Vip, MAX(Vn, v(0.5))), 'wake skew angle')
        g.fn('aero/rotor/chi-deg', M(v(R2D), chi))
        # ground effect on the induced velocity [as the UH-60 and Bo 105 models]: the hover's against the hub height
        # (U.GE_K: the Bo 105's and UH-60's, not fitted), carried into forward flight
        # with Heyson's ratio [NASA TN D-234]
        hh = g.fn('aero/rotor/hub-agl-ft', S(p('position/h-agl-ft'), Q(D(v(U.WL_MR), p('inertia/cg-z-in')), v(12))))
        zr = g.fn('aero/rotor/ge-z-r', MAX(v(U.GE_ZR_MIN), Q(hh, v(U.R))))
        th_, ph_ = p('attitude/theta-rad'), p('attitude/phi-rad')
        dn = to_s(NEG(SIN(th_)), M(SIN(ph_), COS(th_)), M(COS(ph_), COS(th_)))
        wd = g.fn('aero/rotor/ge-wake-down', S(M(NEG(Vhx), dn[0]), M(NEG(Vhy), dn[1]), M(Vn, dn[2])))
        w2 = S(SQR(Vip), SQR(Vn))
        g.fn('aero/rotor/ge-chi-deg', M(v(R2D), ATAN2(SQ(MAX(v(0), D(w2, SQR(wd)))), wd)), 'wake angle from the ground normal')
        kh = g.fn('aero/rotor/ge-k-hover', tag('pow', S(v(1), Q(v(U.GE_K), SQR(zr))), v(U.GE_EXP)))
        g.fn('aero/rotor/kge', S(v(1), M(D(kh, v(1)), T2('aero/rotor/ge-z-r', 'aero/rotor/ge-chi-deg', HG.ZR, HG.CHI_DEG, HG.RATIO))),
             'induced velocity in / out of ground effect')
        g.fn('aero/rotor/vi-fps', M(p('aero/rotor/kge'), v0), 'uniform induced velocity with ground effect')
        # Peters-He flow parameters against the wake skew X = tan(chi / 2); the gains are the rotor system's (inflow_states)
        WR = g.fn('aero/rotor/ph/omega-r-fps', MAX(M(p('rotor/omega-rad_sec'), v(U.R)), v(30)))
        g.fn('aero/rotor/ph/X', Q(SIN(chi), S(v(1), COS(chi))))
        VT = g.fn('aero/rotor/ph/VT', Q(p('aero/rotor/Vprime-fps'), WR))
        mu, lm, la = Q(Vip, WR), Q(v0, WR), Q(Vn, WR)
        g.fn('aero/rotor/ph/V', MAX(Q(v(5), WR), Q(S(SQR(mu), M(S(la, lm), la)), VT)))
        nw = g.fn('aero/rotor/ph/vip-fps', SQ(S(SQR(Vhx), SQR(Vhy))))
        slow = LT(nw, v(0.5))
        g.fn('aero/rotor/ph/cd', IFTHEN(slow, v(1), Q(Vhx, MAX(nw, v(0.5)))))
        g.fn('aero/rotor/ph/sd', IFTHEN(slow, v(0), NEG(Q(Vhy, MAX(nw, v(0.5))))))
        ph_tau = {('c', i): [] for i in range(len(U.PH_COS))}; ph_tau.update({('s', i): [] for i in range(len(U.PH_SIN))})
        shapes = sorted({st_ for sts in (U.PH_COS, U.PH_SIN) for st_ in sts})
        rho, a_snd = p('atmosphere/rho-slugs_ft3'), p('atmosphere/a-fps')
        W = p('rotor/omega-rad_sec')
        a_rows, mach, cl_tab, cd_tab = SEC.tables(SEC.MAIN)
        hrp = s.arm_rp(U.FS_MR, U.BL_MR, U.WL_MR)
        hrp_s = [g.fn('aero/rotor/hubrp-%s-ft' % a, e) for a, e in zip('xyz', to_s(*hrp))]
        ph = math.radians(U.SWASH_PHASE)
        e = U.E_FLAP
        mb = U.W_BLADE / G0
        wrel = g.fn('aero/rotor/w-rel', D(W, rs_a), 'rotor speed less the body\'s yaw rate about the shaft, rad/s')
        # the lateral cyclic in the mirror image: the real aircraft's, negated
        A1s = NEG(p('ctl/A1s-deg'))
        Fsum = {'x': [], 'y': [], 'z': []}; Msum = {'x': [], 'y': [], 'z': []}
        T_terms, Qa_terms, torque_hinge, Qin_terms = [], [], [], []
        for k in range(U.NB):
            b = 'aero/rotor/b%d' % k
            st = 'rotor/b%d' % k
            psi = g.fn(b + '/psi', S(p('rotor/psi-rad'), v(k * 2 * math.pi / U.NB)), 'azimuth of the hinge (mirror image: anticlockwise from the tail)')
            ch, sh = g.fn(b + '/cpsi', COS(psi)), g.fn(b + '/spsi', SIN(psi))
            dl, dld = p(st + '/delta-rad'), p(st + '/deltadot-rad_sec')
            psib = g.fn(b + '/psib', S(psi, dl), 'azimuth of the lagged blade')
            c, sn = g.fn(b + '/cpsib', COS(psib)), g.fn(b + '/spsib', SIN(psib))
            beta, bd = p(st + '/beta-rad'), p(st + '/betadot-rad_sec')
            cb, sb = g.fn(b + '/cbeta', COS(beta)), g.fn(b + '/sbeta', SIN(beta))
            th0 = g.fn(b + '/theta-deg', D(D(p('ctl/theta0-deg'), M(A1s, COS(S(psib, v(ph))))), M(p('ctl/B1s-deg'), SIN(S(psib, v(ph))))),
                       'blade pitch at the shaft, deg')
            # the Peters-He states' induced velocity at the blade, by radial shape, projected on the lagged blade's azimuth
            trig = {1: (c, sn)}
            if any(rh == 2 for jj, rh in shapes):
                trig[2] = (g.fn(b + '/ph-c2', D(SQR(c), SQR(sn))), g.fn(b + '/ph-s2', M(v(2), c, sn)))
            tf = lambda tg, rh: v(1) if rh == 0 else trig[rh][0 if tg == 'c' else 1]
            vi_shape = {}
            for (jj, rh) in shapes:
                lam = S(*[M(p('rotor/ph/%s%d' % (tg, i)), tf(tg, rh)) if rh else p('rotor/ph/%s%d' % (tg, i))
                          for sts, tg in ((U.PH_COS, 'c'), (U.PH_SIN, 's')) for i, st_ in enumerate(sts) if st_ == (jj, rh)])
                vi_shape[(jj, rh)] = g.fn(b + '/vi-%d-%d' % (jj, rh), M(p('aero/rotor/kge'), WR, lam),
                                          'induced velocity of the inflow shape (%d, %d), ft/s' % (jj, rh))
            # the segment's velocity relative to the air is linear in its distance rho from the hinge: in shaft axes
            # V = V0 + rho V1 (the hub's motion, the rotation and the body's rates at the hinge; the blade's own flap and
            # lag motion along it), turned onto the lagged blade's directions (UT along its motion, UR outward)
            X0 = S(Vhx, M(wrel, v(e), sn))
            X1 = S(NEG(M(qs_a, sb)), M(wrel, cb, sn), M(bd, sb, c), M(dld, cb, sn))
            Y0 = S(Vhy, M(wrel, v(e), c))
            Y1 = S(M(wrel, cb, c), M(ps_a, sb), NEG(M(bd, sb, sn)), M(dld, cb, c))
            pq = g.fn(b + '/pq', S(M(ps_a, sn), M(qs_a, c)))
            UT0 = g.fn(b + '/UT0', S(M(X0, sn), M(Y0, c))); UT1 = g.fn(b + '/UT1', S(M(X1, sn), M(Y1, c)))
            UR0 = g.fn(b + '/UR0', D(M(Y0, sn), M(X0, c))); UR1 = g.fn(b + '/UR1', D(M(Y1, sn), M(X1, c)))
            Z0 = g.fn(b + '/Z0', S(Vhz, M(v(e), pq))); Z1 = g.fn(b + '/Z1', M(cb, D(pq, bd)))
            ft_j, fn_j, fr_j, mf_j, ml_j, q_j = [], [], [], [], [], []
            for j, (rr, dr) in enumerate(s.segs):
                es = b + '/s%d' % j
                rho_h = rr - e
                UT = g.fn(es + '/UT', S(UT0, M(v(rho_h), UT1)))
                UR = g.fn(es + '/UR', S(UR0, M(v(rho_h), UR1)), 'spanwise flow (outward), ft/s')
                vi = S(*[M(v(PH.phi(jj, rh, rr / U.R)), vi_shape[(jj, rh)]) for (jj, rh) in shapes])
                Vz = S(Z0, M(v(rho_h), Z1), NEG(vi))
                UP = g.fn(es + '/UP', NEG(S(M(UR, sb), M(Vz, cb))), 'flow up through the blade, ft/s')
                # the section's speed squared and its speed, written into their readers (the Mach number, lift and
                # drag: two reads of UT and UP each instead of their own two functions and four reads)
                V2 = S(tag('pow', UT, v(2)), tag('pow', UP, v(2)), v(1e-4))
                Vv = SQ(V2)
                th = S(th0, v(U.twist(rr)))
                # the angle of attack wrapped into -180..180 by fmod (one read of the raw angle, which is within
                # -210..230 deg, so fmod's argument stays positive)
                g.fn(es + '/alpha-deg', D(FMOD(S(D(th, M(v(R2D), ATAN2(UP, UT))), v(540)), v(360)), v(180)))
                g.fn(es + '/mach', MIN(Q(Vv, a_snd), v(1.0)))
                cl = g.fn(es + '/cl', T2(es + '/alpha-deg', es + '/mach', a_rows, mach, cl_tab))
                cd = g.fn(es + '/cd', S(T2(es + '/alpha-deg', es + '/mach', a_rows, mach, cd_tab), v(U.DCD)))
                r_in, r_out = s.edges[j], s.edges[j + 1]
                fl = max(0.0, min(1.0, (U.TIP_LOSS * U.R - r_in) / (r_out - r_in)))
                Lk = g.fn(es + '/Lk', M(v(0.5 * U.CHORD * dr * fl), rho, Vv, cl), 'lift / V')
                Dk = g.fn(es + '/Dk', M(v(0.5 * U.CHORD * dr), rho, cd, SQ(S(V2, tag('pow', UR, v(2))))), 'drag / V')
                Ft = g.fn(es + '/Ft', NEG(S(M(Lk, UP), M(Dk, UT))), 'force along the blade motion, lbf')
                Fn = g.fn(es + '/Fn', D(M(Lk, UT), M(Dk, UP)), 'force normal to the blade (up), lbf')
                ft_j.append(Ft); fn_j.append(Fn); fr_j.append(NEG(M(Dk, UR)))
                mf_j.append(M(v(rho_h), Fn)); ml_j.append(M(v(rho_h), Ft))
                q_j.append(M(S(v(e), M(v(rho_h), cb)), Ft))
            Ftb, Fnb, Fra = g.fn(b + '/Ft', S(*ft_j)), g.fn(b + '/Fn', S(*fn_j)), g.fn(b + '/Fr-aero', S(*fr_j), 'spanwise drag, outward, lbf')
            Mfl = g.fn(b + '/Mflap', S(*mf_j), 'aerodynamic flap moment about the hinge, ft lbf')
            Mlg = g.fn(b + '/Mlag', M(cb, S(*ml_j)), 'aerodynamic moment about the lag hinge, ft lbf (lead +)')
            Qa_terms.append(g.fn(b + '/Q', S(*q_j), 'aerodynamic moment about the shaft, in the direction of rotation, ft lbf'))
            T_terms.append(M(Fnb, cb))
            for (jj, rh) in shapes:
                sh_ = g.fn(b + '/Fn-%d-%d' % (jj, rh), M(cb, S(*[M(v(PH.phi(jj, rh, rr / U.R)), p(b + '/s%d/Fn' % j)) for j, (rr, dr) in enumerate(s.segs)])))
                for sts, tg in ((U.PH_COS, 'c'), (U.PH_SIN, 's')):
                    for i, st_ in enumerate(sts):
                        if st_ == (jj, rh):
                            ph_tau[(tg, i)].append(M(tf(tg, rh), sh_))
            # the flap and lag accelerations now, from this instant's loads (the rotor system's are a step older)
            bdd = g.fn(b + '/betaddot', s.flap_accel(k, Mfl, ch, sh))
            ddd = g.fn(b + '/deltaddot', Q(D(s.lag_free(k, Mlg, ch, sh), M(v(SRL), p('rotor/omega-dot-abs'))), v(U.I_B)))
            # the blade's inertial reaction on the hinge, in the hinge's radial and tangential directions (UH-60 model):
            # its centre of mass is at e + x cos(beta) cos(delta) outward, x cos(beta) sin(delta) ahead
            cdl, sdl = COS(dl), SIN(dl)
            Fr = g.fn(b + '/Finertial-r', S(M(v(mb * e), W, W), M(v(U.S_B), S(
                M(SQR(W), cb, cdl), M(v(2), W, D(M(cb, cdl, dld), M(sb, sdl, bd))), M(cb, cdl, S(M(bd, bd), M(dld, dld))),
                M(sb, cdl, bdd), NEG(M(v(2), sb, sdl, bd, dld)), M(cb, sdl, ddd)))))
            Fc = g.fn(b + '/Finertial-t', M(v(U.S_B), S(
                M(SQR(W), cb, sdl), M(v(2), W, S(M(sb, cdl, bd), M(cb, sdl, dld))), M(cb, sdl, S(M(bd, bd), M(dld, dld))),
                M(sb, sdl, bdd), M(v(2), sb, cdl, bd, dld), NEG(M(cb, cdl, ddd)))))
            Fu = g.fn(b + '/Finertial-up', NEG(M(v(U.S_B), D(M(bdd, cb), M(bd, bd, sb)))))
            # aerodynamic loads along the lagged blade: e_t = (s, c, 0), n = (sb c, -sb s, -cb), e_r = (-c, s, 0);
            # inertial loads along the hinge's radius (-ch, sh) and tangent (sh, ch); up (0, 0, -1)
            Fx = g.fn(b + '/Fx', S(M(Ftb, sn), M(Fnb, sb, c), NEG(M(Fra, c)), NEG(M(Fr, ch)), M(Fc, sh)))
            Fy = g.fn(b + '/Fy', S(M(Ftb, c), NEG(M(Fnb, sb, sn)), M(Fra, sn), M(Fr, sh), M(Fc, ch)))
            Fz = g.fn(b + '/Fz', S(NEG(M(Fnb, cb)), NEG(Fu)))
            hx = S(hrp_s[0], NEG(M(v(e), ch))); hy = S(hrp_s[1], M(v(e), sh)); hz = hrp_s[2]
            Fsum['x'].append(Fx); Fsum['y'].append(Fy); Fsum['z'].append(Fz)
            Msum['x'].append(D(M(hy, Fz), M(hz, Fy))); Msum['y'].append(D(M(hz, Fx), M(hx, Fz))); Msum['z'].append(D(M(hx, Fy), M(hy, Fx)))
            torque_hinge.append(M(v(e), NEG(S(M(ch, Fy), M(sh, Fx)))))
            Qin_terms.append(NEG(M(v(SRL), ddd)))
        g.fn('aero/rotor/torque-inertial-ftlb', S(*Qin_terms), 'the blades\' lag accelerations\' moment on the hub, rotation +')
        g.fn('aero/rotor/thrust-lbs', S(*T_terms), 'aerodynamic thrust along the shaft (up)')
        # Peters-He forcing over the effective disc B^2 A
        qn = MAX(M(rho, SQR(MAX(W, v(5))), v(U.R ** 4)), v(1e-6))
        for (tg, i), terms in ph_tau.items():
            rh = (U.PH_COS if tg == 'c' else U.PH_SIN)[i][1]
            kb = 1 / U.TIP_LOSS ** 2
            g.fn('aero/rotor/ph/tau-%s%d' % (tg, i), Q(M(v(kb / (2 * math.pi) if rh == 0 else kb / math.pi), S(*terms)), qn))
        Qa = g.fn('aero/rotor/torque-ftlb', NEG(S(*Qa_terms)), 'aerodynamic torque against the rotation, ft lbf')
        g.fn('aero/rotor/power-hp', Q(M(Qa, W), v(550)))
        Fs = [g.fn('aero/rotor/F%s-s' % a, S(*Fsum[a])) for a in 'xyz']
        Ms = [g.fn('aero/rotor/M%s-s' % a, S(*Msum[a])) for a in 'xyz']
        # about the shaft the airframe gets the drive torque's reaction (z down: + yaws the mirror image's nose right),
        # not the hinge forces' torque, which goes into the rotor speed
        Mzs = g.fn('aero/rotor/Mz-airframe-s', S(D(Ms[2], S(*torque_hinge)), p('drive/mr-torque-ftlb')))
        to_b = lambda x, y, z: (D(M(v(ci), x), M(v(si), z)), y, S(M(v(si), x), M(v(ci), z)))
        Fb = [g.fn('aero/rotor/F%s-lbs' % a, e_) for a, e_ in zip('XYZ', to_b(*Fs))]
        Mb = [g.fn('aero/rotor/M%s-ftlb' % a, e_) for a, e_ in zip('LMN', to_b(Ms[0], Ms[1], Mzs))]
        s.add(Fb, Mb)
        # the tip path's flapping harmonics (deg): coning, longitudinal (+ disc aft) and lateral (+ disc right in the
        # mirror image) for the wash maps and the 3-D model
        g.fn('aero/rotor/a0-deg', M(v(R2D / U.NB), S(*[p('rotor/b%d/beta-rad' % k) for k in range(U.NB)])))
        g.fn('aero/rotor/a1-deg', M(v(-2.0 / U.NB * R2D), S(*[M(p('rotor/b%d/beta-rad' % k), p('aero/rotor/b%d/cpsi' % k)) for k in range(U.NB)])))
        g.fn('aero/rotor/b1-deg', M(v(-2.0 / U.NB * R2D), S(*[M(p('rotor/b%d/beta-rad' % k), p('aero/rotor/b%d/spsi' % k)) for k in range(U.NB)])))

    # ------------------------------------------------------------ tail rotor
    def tr_hub_velocity(s):
        g = s.g
        rx, ry, rz = s.arm_cg(U.FS_TR, BL_TR_M, U.WL_TR)
        Pa, Qa, Ra = p('velocities/p-aero-rad_sec'), p('velocities/q-aero-rad_sec'), p('velocities/r-aero-rad_sec')
        w = s.wake('tr')
        return [g.fn('aero/rotor/tr/V%s-fps' % a, e) for a, e in zip('xyz', (
            S(p('velocities/u-aero-fps'), D(M(Qa, rz), M(Ra, ry)), w[0]),
            S(p('velocities/v-aero-fps'), D(M(Ra, rx), M(Pa, rz)), w[1]),
            S(p('velocities/w-aero-fps'), D(M(Pa, ry), M(Qa, rx)), w[2])))]

    def tail_rotor(s):
        """Blade-element tail rotor (as the Bo 105 and UH-60 models'): three blades flapping on their own hinges 0.06 m
        from the hub (states in tail_rotor_states), each of NSEG_TR segments at its own angle of attack and Mach number
        in the tables of its NACA 0018 (root) to 0015 (tip) sections (data/sections.py), in the air at the hub (with
        the main rotor's wake, mrwake.py) plus its own motion and the induced inflow. Its shaft is toed 5 deg [3D]. On the right of the
        boom and pushing (in the mirror image on the left, thrust to the right): the Alouette has no fin in its flow, only
        the boom's end and the gearbox.

        Hub frame (mirror image): t the thrust axis (right, toed forward), a0 aft in the disc, b0 = t x a0 (down); a
        blade at azimuth psi lies along r = cos psi a0 + sin psi b0 and moves along e = -sin psi a0 + cos psi b0 (the
        top blade moves aft [TD]); flapped by beta (towards t) it lies along d = cos b r + sin b t, normal
        n = -sin b r + cos b t."""
        g = s.g
        cT, sT = math.cos(U.TOE_TR), math.sin(U.TOE_TR)
        t, a0, b0 = (sT, cT, 0.0), (-cT, sT, 0.0), (0.0, 0.0, 1.0)
        dot = lambda u, x: S(*[M(v(cu), xe) for cu, xe in zip(u, x) if cu != 0])
        vb = s.tr_hub_velocity()
        Va = g.fn('aero/rotor/tr/Va-fps', dot(a0, vb)); Vb = g.fn('aero/rotor/tr/Vb-fps', dot(b0, vb))
        Vt = g.fn('aero/rotor/tr/Vaxial-fps', dot(t, vb), 'hub velocity along the thrust axis, relative to the air')
        g.fn('aero/rotor/tr/Vinplane-fps', SQ(S(SQR(Va), SQR(Vb), v(1e-6))))
        rates = (p('velocities/p-aero-rad_sec'), p('velocities/q-aero-rad_sec'), p('velocities/r-aero-rad_sec'))
        wa, wb, wt = [g.fn('aero/rotor/tr/w%s' % n, dot(u, rates)) for n, u in (('a', a0), ('b', b0), ('t', t))]
        W = g.fn('aero/rotor/tr/omega-rad_sec', M(v(U.GEAR_TR), p('rotor/omega-rad_sec')))
        v0 = p('rotor/tr/v0-fps')
        g.fn('aero/rotor/tr/vi-fps', v0, 'induced velocity at the disc')
        rho, a_snd = p('atmosphere/rho-slugs_ft3'), p('atmosphere/a-fps')
        segs, edges = U.segments(U.NSEG_TR, U.R_ROOT_TR, U.R_TR)
        sec_tabs = [SEC.tables(SEC.tail((rr - U.R_ROOT_TR) / (U.R_TR - U.R_ROOT_TR))) for rr, dr in segs]
        B = U.TIP_LOSS_TR
        e = U.E_TR
        Fa, Fb, Ft_, Ma, Mb, Mt = [], [], [], [], [], []
        k3 = math.tan(U.DELTA3_TR) * R2D
        for k in range(U.NB_TR):
            b = 'aero/rotor/tr/b%d' % k
            psi = g.fn(b + '/psi', S(p('rotor/tr/psi-rad'), v(k * 2 * math.pi / U.NB_TR)))
            c, sn = g.fn(b + '/cpsi', COS(psi)), g.fn(b + '/spsi', SIN(psi))
            beta, bd = p('rotor/tr/b%d/beta-rad' % k), p('rotor/tr/b%d/betadot-rad_sec' % k)
            cb, sb = g.fn(b + '/cbeta', COS(beta)), g.fn(b + '/sbeta', SIN(beta))
            vr = g.fn(b + '/Vr', S(M(c, Va), M(sn, Vb)))
            ve = g.fn(b + '/Ve', S(NEG(M(sn, Va)), M(c, Vb)))
            wr, we = S(M(c, wa), M(sn, wb)), g.fn(b + '/we', S(NEG(M(sn, wa)), M(c, wb)))
            vn = g.fn(b + '/Vn', S(NEG(M(sb, vr)), M(cb, S(Vt, v0))), 'hub velocity and inflow along the blade normal')
            vd = g.fn(b + '/Vd', S(M(cb, vr), M(sb, S(Vt, v0))), 'spanwise')
            rote = g.fn(b + '/rot-e', D(M(wt, cb), M(wr, sb)), 'body rotation: velocity along e per ft of radius')
            th0 = g.fn(b + '/theta-deg', D(p('ctl/thetaTR-deg'), M(v(k3), beta)) if k3 else p('ctl/thetaTR-deg'), 'pitch, deg')
            fn_j, ft_j, fr_j, mf_j, mt_j = [], [], [], [], []
            for j, (rr, dr) in enumerate(segs):
                es = b + '/s%d' % j
                rh = rr - e
                UT = g.fn(es + '/UT', S(ve, M(v(rr), rote), M(W, S(v(e), M(v(rh), cb)))))
                UP = g.fn(es + '/UP', S(vn, NEG(M(v(rr), we)), M(v(rh), bd)), 'flow through the blade, from the thrust side +')
                V2 = S(SQR(UT), SQR(UP), v(1e-4))                      # written into their readers, as the main rotor's
                Vv = SQ(V2)
                th = S(th0, v(U.TWIST_TR * rr / U.R_TR)) if U.TWIST_TR else th0
                # the angle of attack wrapped into -180..180 by fmod (one read of the raw angle, which is within
                # -210..230 deg, so fmod's argument stays positive)
                g.fn(es + '/alpha-deg', D(FMOD(S(D(th, M(v(R2D), ATAN2(UP, UT))), v(540)), v(360)), v(180)))
                g.fn(es + '/mach', MIN(Q(Vv, a_snd), v(1.0)))
                a_rows, mach, cl_tab, cd_tab = sec_tabs[j]
                cl = g.fn(es + '/cl', T2(es + '/alpha-deg', es + '/mach', a_rows, mach, cl_tab))
                cd = g.fn(es + '/cd', T2(es + '/alpha-deg', es + '/mach', a_rows, mach, cd_tab))
                r_in, r_out = edges[j], edges[j + 1]
                fl = max(0.0, min(1.0, (B * U.R_TR - r_in) / (r_out - r_in)))
                Lk = g.fn(es + '/Lk', M(v(0.5 * U.CHORD_TR * dr * fl), rho, Vv, cl), 'lift / V')
                Dk = g.fn(es + '/Dk', M(v(0.5 * U.CHORD_TR * dr), rho, cd, SQ(S(V2, SQR(vd)))), 'drag / V')
                Fn = g.fn(es + '/Fn', D(M(Lk, UT), M(Dk, UP)), 'force along the blade normal (thrust side), lbf')
                Ft = g.fn(es + '/Ft', NEG(S(M(Lk, UP), M(Dk, UT))), 'force along the blade motion, lbf')
                fn_j.append(Fn); ft_j.append(Ft); fr_j.append(NEG(M(Dk, vd))); mf_j.append(M(v(rh), Fn)); mt_j.append(M(v(rr), Ft))
            Fn_k, Ft_k, Fr_k = g.fn(b + '/Fn', S(*fn_j)), g.fn(b + '/Ft', S(*ft_j)), g.fn(b + '/Fr', S(*fr_j))
            g.fn(b + '/Mflap', S(*mf_j), 'aerodynamic flap moment about the hinge, ft lbf (towards the thrust side +)')
            Mte = g.fn(b + '/Me', S(*mt_j), 'moment of the forces along the blade motion, ft lbf')
            rad = g.fn(b + '/F-radial', D(M(Fr_k, cb), M(Fn_k, sb)))
            Fa.append(D(M(rad, c), M(Ft_k, sn))); Fb.append(S(M(rad, sn), M(Ft_k, c))); Ft_.append(S(M(Fn_k, cb), M(Fr_k, sb)))
            Ma.append(NEG(M(Mte, sb, c))); Mb.append(NEG(M(Mte, sb, sn))); Mt.append(M(Mte, cb))
        Fha, Fhb = g.fn('aero/rotor/tr/Fa-lbs', S(*Fa), 'in-plane force, aft'), g.fn('aero/rotor/tr/Fb-lbs', S(*Fb), 'in-plane force, down')
        T = g.fn('aero/rotor/tr/thrust-lbs', S(*Ft_), 'the blades\' thrust (mirror image: right; the real aircraft: left)')
        Mha, Mhb, Mht = [g.fn('aero/rotor/tr/M%s-ftlb' % n, S(*x)) for n, x in (('a', Ma), ('b', Mb), ('t', Mt))]
        Qtr = g.fn('aero/rotor/tr/torque-ftlb', NEG(Mht), 'aerodynamic torque (against the rotation)')
        g.fn('aero/rotor/tr/power-hp', Q(M(Qtr, W), v(550)))
        F = [g.fn('aero/rotor/tr/F%s-lbs' % n, S(*[M(v(u[i]), x) for u, x in ((a0, Fha), (b0, Fhb), (t, T)) if u[i] != 0]))
             for i, n in enumerate('XYZ')]
        Mh = [S(*[M(v(u[i]), x) for u, x in ((a0, Mha), (b0, Mhb), (t, Mht)) if u[i] != 0]) for i in range(3)]
        ax, ay, az = s.arm_rp(U.FS_TR, BL_TR_M, U.WL_TR)
        Mo = (S(D(M(ay, F[2]), M(az, F[1])), Mh[0]), S(D(M(az, F[0]), M(ax, F[2])), Mh[1]), S(D(M(ax, F[1]), M(ay, F[0])), Mh[2]))
        s.add(F, Mo)

    # ------------------------------------------------------------ airframe
    def airframe(s):
        """The fuselage, the hub's, the landing gear's and the tail boom's drag, and the empennage, in a system channel
        run every AIRFRAME_EXECRATE frames (as the Bo 105 model: their loads change slowly against the rotors'); summed
        into six loads for the aerodynamic axes."""
        s.air.channel('airframe', execrate=AIRFRAME_EXECRATE)
        s.g = ChannelGraph(s.air)
        axes = (s.X, s.Y, s.Z, s.L, s.Mm, s.N)
        s.X, s.Y, s.Z, s.L, s.Mm, s.N = [], [], [], [], [], []
        s.fuselage()
        s.boom_gear_hub()
        s.empennage()
        loads = [s.g.fn('aero/airframe/%s' % n, S(*terms) if terms else v(0))
                 for n, terms in zip(('FX-lbs', 'FY-lbs', 'FZ-lbs', 'ML-ftlb', 'MM-ftlb', 'MN-ftlb'), (s.X, s.Y, s.Z, s.L, s.Mm, s.N))]
        s.X, s.Y, s.Z, s.L, s.Mm, s.N = axes
        s.g = s.aero
        s.add(loads[:3], loads[3:])

    def fuselage(s):
        """The pod's coefficients (data/al3_fuselage.py: the 1/5-scale tests of the body with its engine [E94], faired
        into a cross-flow model) at their reference point, wind axes, in the local flow with the main rotor's wake;
        flying backwards they are mirrored front to back."""
        g = s.g
        t = FUS.tables()
        kw = s.wake('fus')
        Vx = g.fn('aero/fus/Vx-fps', S(p('velocities/u-aero-fps'), kw[0]))
        Vy = g.fn('aero/fus/Vy-fps', S(p('velocities/v-aero-fps'), kw[1]))
        Vz = g.fn('aero/fus/Vz-fps', S(p('velocities/w-aero-fps'), kw[2]))
        Vxz = g.fn('aero/fus/Vxz-fps', MAX(SQ(S(SQR(Vx), SQR(Vz))), v(0.01)))
        V = g.fn('aero/fus/V-fps', MAX(SQ(S(SQR(Vx), SQR(Vy), SQR(Vz))), v(0.01)))
        g.fn('aero/fus/alpha-deg', M(v(R2D), ATAN2(Vz, ABS(Vx))), 'tunnel pitch angle (front-back mirrored)')
        g.fn('aero/fus/psi-deg', M(v(-R2D), ATAN2(Vy, Vxz)), 'tunnel yaw angle = -sideslip (nose right of the wind +)')
        g.fn('aero/fus/psi-abs-deg', ABS(p('aero/fus/psi-deg')))
        kf = g.fn('aero/fus/kfwd', MAX(v(-1), MIN(v(1), Q(Vx, v(3.0)))))
        q = g.fn('aero/fus/qbar-psf', M(v(0.5), p('atmosphere/rho-slugs_ft3'), SQR(V)))
        A_, P_, PA = 'aero/fus/alpha-deg', 'aero/fus/psi-deg', 'aero/fus/psi-abs-deg'
        Dq = g.fn('aero/fus/D-ft2', S(T1(A_, t['alpha'], t['D0']), T1(PA, t['psi_abs'], t['dD'])))
        Lq = g.fn('aero/fus/L-ft2', M(kf, S(T1(A_, t['alpha'], t['L0']), T1(PA, t['psi_abs'], t['dL']))))
        Mq = g.fn('aero/fus/M-ft3', M(kf, T2(A_, PA, t['alpha'], t['psi_abs'], t['M'])))
        Yq = g.fn('aero/fus/Y-ft2', T1(P_, t['psi'], t['Y']))
        Rq = g.fn('aero/fus/R-ft3', M(kf, T1(P_, t['psi'], t['R'])))
        Nq = g.fn('aero/fus/N-ft3', T1(P_, t['psi'], t['N']))
        ca, sa = Q(Vx, Vxz), Q(Vz, Vxz)
        cb, sb = Q(Vxz, V), Q(Vy, V)
        c1 = (Q(Vx, V), Q(Vy, V), Q(Vz, V))
        c2 = (M(ca, sb), NEG(cb), M(sa, sb))
        c3 = (NEG(sa), v(0), ca)
        F = [g.fn('aero/fus/F%s-lbs' % a, M(q, NEG(S(M(Dq, c1[i]), M(Yq, c2[i]), M(Lq, c3[i]))))) for i, a in enumerate('XYZ')]
        Mw = [g.fn('aero/fus/M%s-wind' % a, M(q, S(M(Rq, c1[i]), NEG(M(Mq, c2[i])), M(Nq, c3[i])))) for i, a in enumerate('LMN')]
        rx, ry, rz = s.arm_rp(FUS.FS_REF, FUS.BL_REF, FUS.WL_REF)
        Mo = (S(Mw[0], D(M(ry, F[2]), M(rz, F[1]))), S(Mw[1], D(M(rz, F[0]), M(rx, F[2]))), S(Mw[2], D(M(rx, F[1]), M(ry, F[0]))))
        Mo = [g.fn('aero/fus/%s-ftlb' % a, e) for a, e in zip('LMN', Mo)]
        s.add(F, Mo)

    def crossflow(s, name, fs, bl, wl, f, wash=None):
        """drag areas per body axis (ft^2) at a point, in its local flow: F = -q_axis f_axis (cross-flow principle)"""
        g = s.g
        rho = p('atmosphere/rho-slugs_ft3')
        rx, ry, rz = s.arm_cg(fs, bl, wl)
        Pa, Qa, Ra = p('velocities/p-aero-rad_sec'), p('velocities/q-aero-rad_sec'), p('velocities/r-aero-rad_sec')
        w = wash or (v(0), v(0), v(0))
        Vc = [g.fn('aero/%s/V%s-fps' % (name, a), e) for a, e in zip('xyz', (
            S(p('velocities/u-aero-fps'), D(M(Qa, rz), M(Ra, ry)), w[0]),
            S(p('velocities/v-aero-fps'), D(M(Ra, rx), M(Pa, rz)), w[1]),
            S(p('velocities/w-aero-fps'), D(M(Pa, ry), M(Qa, rx)), w[2])))]
        F = [g.fn('aero/%s/F%s-lbs' % (name, a), M(v(-0.5 * fa), rho, Vi, ABS(Vi))) if fa else None for a, fa, Vi in zip('XYZ', f, Vc)]
        ax, ay, az = s.arm_rp(fs, bl, wl)
        Fz = [x or v(0) for x in F]
        s.add(F, (D(M(ay, Fz[2]), M(az, Fz[1])), D(M(az, Fz[0]), M(ax, Fz[2])), D(M(ax, Fz[1]), M(ay, Fz[0]))))

    def boom_gear_hub(s):
        """The parts the pod's tests leave out, each by the cross-flow principle in its local flow [Hoerner]: the tail
        boom (the 3-D model's closed boom, 2.71 m^2 side and 1.98 m^2 plan, normal force 0.7, skin friction 0.04 m^2,
        in the main rotor's wake: its download in the hover); the landing gear (three 355 x 150 wheels [MI], the struts
        and swing arms of the 3-D model); the rotor head, mast and controls; and the rest (the tail rotor's gearbox and
        hub, the exhaust, steps, antennas, gaps): MISC_DRAG_FT2, an estimate."""
        s.crossflow('boom', *U.model_to_station(-3.56, 0.0, 0.28)[:1], 0.0, U.model_to_station(-3.56, 0.0, 0.28)[2],
                    (0.04 * 10.7639, 0.7 * 2.71 * 10.7639, 0.7 * 1.98 * 10.7639), s.wake('boom'))
        fs, _, wl = U.model_to_station(-0.6, 0.0, -0.95)
        s.crossflow('gear', fs, 0.0, wl, GEAR_F)
        s.crossflow('hub', U.FS_MR, 0.0, U.WL_MR - 6.0, (HUB_DRAG_FT2, HUB_DRAG_FT2, 0.0))
        fs, _, wl = U.model_to_station(-1.0, 0.0, 0.5)
        s.crossflow('misc', fs, 0.0, wl, (MISC_DRAG_FT2, 0.0, 0.0))

    def empennage(s):
        """The horizontal stabiliser and its two end plates [3D], each a 2-D strip in its own plane with lift and drag
        tables over +-180 deg, in the local flow with the main rotor's wake (mrwake.py)."""
        g = s.g
        Pa, Qa, Ra = p('velocities/p-aero-rad_sec'), p('velocities/q-aero-rad_sec'), p('velocities/r-aero-rad_sec')

        def local(tag_, fs, bl, wl, wash):
            rx, ry, rz = s.arm_cg(fs, bl, wl)
            vb = (S(p('velocities/u-aero-fps'), D(M(Qa, rz), M(Ra, ry)), wash[0]),
                  S(p('velocities/v-aero-fps'), D(M(Ra, rx), M(Pa, rz)), wash[1]),
                  S(p('velocities/w-aero-fps'), D(M(Pa, ry), M(Qa, rx)), wash[2]))
            return [g.fn('aero/emp/%s/V%s-fps' % (tag_, a), e) for a, e in zip('xyz', vb)]
        xs, cl, cd = stab_tables()
        Vx, Vy, Vz = local('h', U.FS_H, U.BL_H, U.WL_H, s.wake('stab'))
        Vn = g.fn('aero/emp/h/V-fps', MAX(SQ(S(SQR(Vx), SQR(Vz))), v(0.01)))
        ah = g.fn('aero/emp/h/alpha-raw', S(M(v(R2D), ATAN2(Vz, Vx)), v(U.INC_H)))
        g.fn('aero/emp/h/alpha-deg', IFTHEN(GT(ah, v(180)), D(ah, v(360)), IFTHEN(LT(ah, v(-180)), S(ah, v(360)), ah)))
        c_l = g.fn('aero/emp/h/cl', T1('aero/emp/h/alpha-deg', *_mirror180(*_odd(xs, cl), negate=True)))
        c_d = g.fn('aero/emp/h/cd', T1('aero/emp/h/alpha-deg', *_mirror180(*_even(xs, cd), negate=False)))
        qs = g.fn('aero/emp/h/qS', M(v(0.5 * U.S_H), p('atmosphere/rho-slugs_ft3'), SQR(Vn)))
        Fx = g.fn('aero/emp/h/Fx-lbs', M(qs, Q(D(M(c_l, Vz), M(c_d, Vx)), Vn)))
        Fz = g.fn('aero/emp/h/Fz-lbs', M(qs, Q(NEG(S(M(c_l, Vx), M(c_d, Vz))), Vn)))
        rx, ry, rz = s.arm_rp(U.FS_H, U.BL_H, U.WL_H)
        s.add((Fx, None, Fz), (M(ry, Fz), D(M(rz, Fx), M(rx, Fz)), NEG(M(ry, Fx))))
        xs, cl, cd = endplate_tables()
        for side, bl in (('l', -U.BL_EP), ('r', U.BL_EP)):
            tg = 'ep' + side
            Vx, Vy, Vz = local(tg, U.FS_EP, bl, U.WL_EP, s.wake('ep', mirror=(side == 'l')))
            Vn = g.fn('aero/emp/%s/V-fps' % tg, MAX(SQ(S(SQR(Vx), SQR(Vy))), v(0.01)))
            g.fn('aero/emp/%s/beta-deg' % tg, M(v(R2D), ATAN2(Vy, Vx)))
            c_l = g.fn('aero/emp/%s/cl' % tg, T1('aero/emp/%s/beta-deg' % tg, *_mirror180(*_odd(xs, cl), negate=True)))
            c_d = g.fn('aero/emp/%s/cd' % tg, T1('aero/emp/%s/beta-deg' % tg, *_mirror180(*_even(xs, cd), negate=False)))
            qs = g.fn('aero/emp/%s/qS' % tg, M(v(0.5 * U.S_EP), p('atmosphere/rho-slugs_ft3'), SQR(Vn)))
            Fx = g.fn('aero/emp/%s/Fx-lbs' % tg, M(qs, Q(D(M(c_l, Vy), M(c_d, Vx)), Vn)))
            Fy = g.fn('aero/emp/%s/Fy-lbs' % tg, M(qs, Q(NEG(S(M(c_l, Vx), M(c_d, Vy))), Vn)))
            rx, ry, rz = s.arm_rp(U.FS_EP, bl, U.WL_EP)
            s.add((Fx, Fy, None), (NEG(M(rz, Fy)), M(rz, Fx), D(M(rx, Fy), M(ry, Fx))))

    def axes(s):
        """the six loads; the side force, rolling and yawing moments mirrored back to the real aircraft"""
        def ax(name, terms, sign):
            tot = S(*terms) if len(terms) > 1 else terms[0]
            return '<axis name="%s"><function>%s</function></axis>' % (name, tot if sign > 0 else NEG(tot))
        return '\n'.join((ax('X', s.X, 1), ax('Y', s.Y, -1), ax('Z', s.Z, 1), ax('ROLL', s.L, -1), ax('PITCH', s.Mm, 1), ax('YAW', s.N, -1)))

    # ------------------------------------------------------------ outputs for FlightGear
    def outputs(s):
        """the rotor's and the engine's state for the 3-D model, the instruments and the sound, worked out every
        SLOW_EXECRATE frames. Blade azimuths: the mirror image's anticlockwise azimuth from the tail is the real
        aircraft's clockwise one."""
        o = FCS('Alouette III outputs')
        o.channel('rotor', execrate=SLOW_EXECRATE)
        RPM = 60 / (2 * math.pi)
        W = p('rotor/omega-rad_sec')
        o.fn('out/rotor-rpm', M(v(RPM), W), out='/rotors/main/rpm')
        o.fn('out/rotor-rpm-pct', M(v(100 / U.OMEGA), W), out='/rotors/main/rpm-pct')
        o.fn('out/tail-rpm', M(v(RPM * U.GEAR_TR), W), out='/rotors/tail/rpm')
        wrap = lambda e: IFTHEN(GT(e, v(360)), D(e, v(360)), e)
        for k in range(U.NB):
            o.fn('out/b%d-position' % k, wrap(S(M(v(R2D), p('rotor/psi-rad')), v(360.0 / U.NB * k))), out='/rotors/main/blade[%d]/position-deg' % k)
            o.fn('out/b%d-flap' % k, M(v(R2D), p('rotor/b%d/beta-rad' % k)), out='/rotors/main/blade[%d]/flap-deg' % k)
            o.fn('out/b%d-lag' % k, M(v(R2D), p('rotor/b%d/delta-rad' % k)), out='/rotors/main/blade[%d]/lag-deg' % k)
            o.fn('out/b%d-pitch' % k, p('aero/rotor/b%d/theta-deg' % k), out='/rotors/main/blade[%d]/incidence-deg' % k)
        for k in range(U.NB_TR):
            o.fn('out/tr-b%d-position' % k, wrap(S(M(v(R2D), p('rotor/tr/psi-rad')), v(360.0 / U.NB_TR * k))), out='/rotors/tail/blade[%d]/position-deg' % k)
            o.fn('out/tr-b%d-flap' % k, M(v(R2D), p('rotor/tr/b%d/beta-rad' % k)), out='/rotors/tail/blade[%d]/flap-deg' % k)
        o.fn('out/tr-incidence', p('ctl/thetaTR-deg'), out='/rotors/tail/blade/incidence-deg')
        o.fn('out/total-torque', p('aero/rotor/torque-ftlb'), out='/rotors/gear/total-torque')
        o.fn('out/coll-pitch-deg', S(p('ctl/theta0-deg'), v(U.twist(0.7 * U.R))), desc='collective pitch at 0.7 R, deg (the pitch indicator)')
        sl = slap_table()
        if sl:
            # the blade slap's loudness for the sound (Sounds/al3-sound.xml's slap loops; data/slap_al3.json): PyWopWop's
            # map of it over the advance ratio and the inflow through the tip-path plane, which set where the tip
            # vortices pass the blades. The rotor's kinematics in the plane normal to its force (PyWopWop's trim puts
            # the tip-path plane there): its advance ratio and upflow; the inflow PyWopWop's at them, scaled with the
            # thrust; the vortices' strength with the thrust. Lagged so that the blade passages' (3/rev) loads stay
            # out of it. Shaft axes (the mirror image's: the magnitudes and the dot product are the real aircraft's)
            o.channel('rotor sound', execrate=SLAP_EXECRATE)
            WR = MAX(M(W, v(U.R)), v(30))
            o_ = 'out/slap'
            F = [p('aero/rotor/F%s-s' % a) for a in 'xyz']
            Vh = [p('aero/rotor/Vh%s-fps' % a) for a in 'xyz']
            Fm = o.fn(o_ + '-F-lbs', MAX(SQ(S(*[SQR(f) for f in F])), v(1.0)), desc='the rotor\'s force')
            vn = o.fn(o_ + '-Vn-fps', Q(S(*[M(a, f) for a, f in zip(Vh, F)]), Fm),
                      desc='the hub\'s velocity along the force: the air\'s through the tip-path plane, into the wake +')
            o.fn(o_ + '-muz-now', NEG(Q(vn, WR)), desc='upflow through the tip-path plane over the tip speed')
            o.fn(o_ + '-mu-now', Q(SQ(MAX(v(0), D(S(*[SQR(a) for a in Vh]), SQR(vn)))), WR))
            muz = o.lag(o_ + '-muz', o_ + '-muz-now', SLAP_TAU_S)
            o.lag(o_ + '-mu', o_ + '-mu-now', SLAP_TAU_S)
            T = o.lag(o_ + '-T-lbs', o_ + '-F-lbs', SLAP_TAU_S)
            Tr = Q(T, v(sl['thrust_ref_lbf']))
            l0 = T2(o_ + '-muz', o_ + '-mu', sl['mu_z'], sl['mu'], sl['lam0'])
            o.fn(o_ + '-lam', D(M(l0, Tr), muz), desc='inflow through the tip-path plane')
            for k in ('ext', 'int'):
                o.fn('%s-%s' % (o_, k), M(T2(o_ + '-lam', o_ + '-mu', sl['lam'], sl['mu'], sl[k]), Tr),
                     out='/rotors/main/slap-%s' % k,
                     desc='the slap loops\' amplitude %s, 1 at the bank\'s' % ('outside' if k == 'ext' else 'in the cockpit'))
        o.channel('vibration', execrate=SLOW_EXECRATE)
        hp = []
        for axn in 'xyz':
            o.washout('vib/%s-hp1' % axn, 'accelerations/a-pilot-%s-ft_sec2' % axn, 1 / (2 * math.pi * 6))
            hp.append(o.washout('vib/%s-hp' % axn, 'vib/%s-hp1' % axn, 1 / (2 * math.pi * 6)))
        o.fn('vib/ms', S(*[SQR(h) for h in hp]))
        o.lag('vib/ms-avg', 'vib/ms', 0.25)
        o.fn('vib/rotor-g', Q(SQ(MAX(v(0), p('vib/ms-avg'))), v(G0)))
        A = math.pi * U.R ** 2
        vh = SQ(MAX(v(1), Q(p('aero/rotor/thrust-lbs'), M(v(2 * A), p('atmosphere/rho-slugs_ft3')))))
        mu = Q(p('aero/rotor/Vinplane-fps'), MAX(vh, v(5)))
        o.fn('vib/etl', tag('exp', NEG(SQR(Q(D(mu, v(0.9)), v(0.5))))), desc='translational lift band')
        thrust = MAX(v(0), MIN(v(1.5), Q(p('aero/rotor/thrust-lbs'), v(U.MTOW_KG * U.KG_LB))))
        o.fn('vib/bvi-g', M(v(0.05), p('vib/etl'), thrust, GT(W, v(0.5 * U.OMEGA))))
        fade = MAX(v(0), MIN(v(1), Q(D(p('simulation/sim-time-sec'), v(1.0)), v(3.0))))
        o.fn('vib/level-g', M(fade, D(v(1), p('ctl/init')), SQ(S(SQR(p('vib/rotor-g')), SQR(p('vib/bvi-g'))))),
             out='/sim/model/alouette3/vibration-g')
        return o


def cockpit():
    """The original FlightGear aircraft's cockpit and systems (Gerard Robin's, Thorsten Renk's: its switches, levers, gauges,
    electrical system, autopilot, Amelia, crash detection and animations) on this flight model: its controls to the
    engine and drive, and this model's state to the properties its gauges, animations and systems read (the old JSBSim
    rotor and turboprop's names). FlightGear build only."""
    o = FCS('Alouette III cockpit')
    for n, val in (('animation/master-switch-cmd', 0), ('fcs/nominal-flow-cmd', 0), ('animation/rotor_brake-switch-cmd', 1),
                   ('propulsion/engine[0]/cutoff', 1), ('animation/rotor_brake-switch-lock-cmd', 1)):
        o.declare(n, val)
    # the start's machinery (the cockpit's state, the in-air start's hold and trims) runs only in the start's first
    # START_HOLD_S[1] + 1 s: then its channel stops, its outputs (the integrators restored, the trims) held
    o.declare('cockpit/starting', 1)
    o.channel('start gate')
    o.fn('cockpit/starting-next', LT(p('simulation/sim-time-sec'), v(START_HOLD_S[1] + 1.0)), out='cockpit/starting')
    o.channel('start state', execute='cockpit/starting')
    # in the first second (ctl/init-fast) the cockpit is set as the engine starts: in the air running (the fuel valve open,
    # the fuel flow lever forward, the rotor brake off, battery, alternator and the electrical master on: everything
    # electrical, the lights and the instruments, hangs off the master), on the ground cold and dark
    init = GT(p('ctl/init-fast'), v(0.5))
    run0 = p('eng/running-at-start')
    # the parking brake (fcs/brake-parking-cmd-norm, aliased to /controls/gear/brake-parking: FlightGear brakes each
    # wheel by the larger of its brake axis and the parking brake) on for a ground start, off for one in the air, so that
    # the toe brakes work after the landing
    for prop, running, cold in (('propulsion/engine[0]/cutoff', 0, 1), ('fcs/nominal-flow-cmd', 1, 0),
                                ('animation/rotor_brake-switch-cmd', 0, 1), ('electrical/switch/battery', 1, None),
                                ('electrical/switch/alternator', 1, None), ('animation/electrical-master-switch', 1, None),
                                ('fcs/brake-parking-cmd-norm', 0, 1)):
        o.declare(prop, cold if cold is not None else 0)
        o.fn('cockpit/init-%s' % prop.replace('/', '-').replace('[', '').replace(']', ''),
             IFTHEN(init, IFTHEN(run0, v(running), v(cold) if cold is not None else p(prop)), p(prop)), out=prop)
    # the cockpit doors (Systems/animation-dev.xml: open, 1, as the aircraft starts) shut for a start in the air; on the
    # ground they stay as the original has them
    for side in ('left', 'right'):
        prop = 'animation/cockpit-%s-door-pos-cmd' % side
        o.declare(prop, 1)
        o.fn('cockpit/init-%s-door' % side, IFTHEN(M(init, run0), v(0), p(prop)), out=prop)
    # in the air also the collective, for the start's airspeed from the level-flight trims at 2200 kg (sweep.py: the lever
    # 0.62 in the hover, 0.49 at 60 kt, 0.62 at 100 kt), and FlightGear's trims zeroed (its JSBSim trim at an in-air start
    # cannot trim this rotor and leaves them where it gave up); a throttle axis takes the collective over once it moves
    coll0 = T1('velocities/vc-kts', [0, 20, 40, 60, 80, 100, 120], [0.62, 0.57, 0.50, 0.49, 0.53, 0.62, 0.77])
    o.fn('cockpit/init-collective', IFTHEN(M(init, run0), coll0, p('/controls/al3/collective')),
         out='/controls/al3/collective')
    for t in ('elevator-trim', 'aileron-trim', 'rudder-trim'):
        o.fn('cockpit/init-%s' % t, IFTHEN(M(init, run0), v(0), p('/controls/flight/' + t)), out='/controls/flight/' + t)
    s_start_trim(o, init, run0)
    o.channel('controls')
    # the fuel valve and booster pump (CUTOFF), the starter switch (an automatic start on its switching on [E77, LM]), the
    # fuel flow lever, the rotor brake
    o.fn('cockpit/fuel-valve', LT(p('propulsion/engine[0]/cutoff'), v(0.5)),
         out=['eng/fuel-valve', 'propulsion/engine[0]/fuel-pump', 'systems/engine/fuel-pump', 'systems/engine/start-state'])
    o.fn('cockpit/parking-brake', p('fcs/brake-parking-cmd-norm'), out='fcs/brake-parking-pos-norm')
    o.fn('cockpit/starter', GT(p('animation/master-switch-cmd'), v(0.5)), out='eng/starter-cmd')
    o.fn('cockpit/flow-lever', MAX(v(0), MIN(v(1), p('fcs/nominal-flow-cmd'))), out=['eng/flow-lever', 'fcs/fuel-flow-control'])
    o.fn('cockpit/rotor-brake', GT(p('animation/rotor_brake-switch-cmd'), v(0.5)), out='/controls/rotor/brake')
    # the original gear system's outputs its animations read: the gear in contact (crashed: not), the nose wheel's angle
    # (steered; castoring it is JSBSim's own and not exposed: shown straight)
    o.fn('cockpit/gear-contact', LT(p('systems/crash-detect/crashed-sw'), v(0.5)), out='gear/gear-cp-cmd-norm')
    o.fn('cockpit/nose-steer', M(D(v(1), p('gear/unit[2]/castered')), p('fcs/steer-pos-deg[2]')), out='fcs/steer-pos-deg')
    o.channel('gauges', execrate=SLOW_EXECRATE)
    RPM = 60 / (2 * math.pi)
    W = p('rotor/omega-rad_sec')
    o.fn('cockpit/engine-rpm', p('eng/rpm'), out=['propulsion/engine/engine-rpm', 'propulsion/engine[0]/engine-rpm'])
    o.fn('cockpit/rotor-rpm', M(v(RPM), W), out=['propulsion/engine/rotor-rpm', 'propulsion/engine[0]/rotor-rpm', 'animation/rotor0-rotation'])
    o.fn('cockpit/tail-rpm', M(v(RPM * U.GEAR_TR), W), out=['propulsion/engine[1]/rotor-rpm', 'animation/rotor/tail/rpm'])
    o.fn('cockpit/n1', p('eng/n-pct'), out=['propulsion/engine/n1', 'propulsion/engine[0]/n1'])
    o.fn('cockpit/t4', p('eng/t4-degc'), out='propulsion/engine[0]/itt-c')
    o.fn('cockpit/torque', p('drive/mr-torque-ftlb'), out=['propulsion/engine/torque-lbsft', 'propulsion/engine[0]/torque-lbsft', 'systems/rotor/main/torque'])
    o.fn('cockpit/thrust', p('aero/rotor/thrust-lbs'), out='propulsion/engine[0]/thrust-lbs')
    o.fn('cockpit/brake', p('/controls/rotor/brake'), out='propulsion/engine[0]/brake-ctrl-norm')
    o.fn('cockpit/freewheel', p('eng/engaged'), out='propulsion/engine[0]/free-wheel-transmission')
    o.fn('cockpit/clutch', MIN(v(1), Q(p('drive/clutch-cap-ftlb'), v(engines.Q_TH))), out=['propulsion/engine/clutch-ctrl-norm', 'propulsion/engine[0]/clutch-ctrl-norm'])
    o.fn('cockpit/fuel', p('/consumables/fuel/tank[0]/level-lbs'), out='propulsion/tank/contents-lbs')
    # the old model's control readouts: cyclic and tail rotor pitch in radians
    o.fn('cockpit/lon-rad', M(v(1 / R2D), p('ctl/B1s-deg')), out=['propulsion/engine/longitudinal-ctrl-rad', 'fcs/x-lon'])
    o.fn('cockpit/lat-rad', M(v(1 / R2D), p('ctl/A1s-deg')), out=['propulsion/engine/lateral-ctrl-rad', 'fcs/x-lat'])
    o.fn('cockpit/ped-rad', M(v(1 / R2D), p('ctl/thetaTR-deg')), out=['propulsion/engine[1]/antitorque-ctrl-rad', 'fcs/x-ped'])
    # the collective pitch indicator (Instruments/Afficheur_Fake.xml): the blades' pitch at 0.7 R in radians
    o.fn('cockpit/x-col', M(v(1 / R2D), S(p('ctl/theta0-deg'), v(U.twist(0.7 * U.R)))), out='fcs/x-col')
    # the original instrumentation's radar altimeter (Systems/instrumentation.xml) subtracts the CG's height above the wheels
    o.fn('cockpit/agl-offset', Q(D(p('inertia/cg-z-in'), v(U.model_to_station(*U.MAIN_WHEELS[0])[2])), v(12)), out='instruments/agl-offset-ref')
    o.fn('cockpit/terrain', v(1), out='simulation/terrain_sw')
    o.channel('rotor model', execrate=SLOW_EXECRATE)
    # the 3-D rotor (Models/Rotors): each blade's azimuth, the model's blade 0 forward at 0 and turning clockwise seen from
    # above with it (the flight model's azimuth from the tail, clockwise on the real aircraft, plus 180 deg); its flap
    wrap = lambda e: IFTHEN(GT(e, v(360)), D(e, v(360)), e)
    for k in range(U.NB):
        o.fn('cockpit/pale%dpos' % k, wrap(S(M(v(R2D), p('rotor/psi-rad')), v(180.0 + 360.0 / U.NB * k))), out='systems/rotor/pale%dpos-deg' % k)
        o.fn('cockpit/pale%dflap' % k, M(v(R2D), p('rotor/b%d/beta-rad' % k)), out='systems/rotor/pale%dflap-deg' % k)
    s_touchdown(o)
    return o


# each wheel's rim, for the touchdowns' sounds: the 355 x 150-4 tyre and its wheel and brake disc (masses not published:
# about 4 kg of tyre at 0.15 m and 4 kg of wheel and disc near the hub), the bearings' drag, a brake that stops it
WHEEL_INERTIA_KGM2 = 0.10
WHEEL_DRAG_NM = 0.3
WHEEL_BRAKE_NM = 200.0
WHEEL_GRIP = 0.5            # the tyre's sliding friction as it spins up


def s_touchdown(o):
    """Each wheel's spin, as the speed its rim turns at (the X-57 model's): on the ground friction spins it up, its sliding
    grip times the strut's force (spring and damping, as the gear's own), until it rolls with the ground; in the air it
    spins down, its bearings dragging, and a main wheel's brake (its pedal or the parking brake) stops it at once. Then
    each wheel's sink rate (the aircraft's, with its roll and pitch about the CG) and slip (the ground speed less the
    rim's) as its weight comes on, held till it lifts: gear/unit[n]/touchdown-sink-fps and touchdown-slip-kt, which
    the touchdown sounds read."""
    o.channel('touchdown', execrate=U.every(240))
    for b_ in ('brake-left', 'brake-right', 'brake-parking'):      # FlightGear's (kept there), for offline runs
        o.declare('/controls/gear/' + b_, 0.0)
    r = U.TYRE_DIAMETER / 2
    up = WHEEL_GRIP * 4.4482216 * r ** 2 / WHEEL_INERTIA_KGM2 / 0.514444      # kt/s of rim speed a lbf of load gives
    kd = r / WHEEL_INERTIA_KGM2 / 0.514444                                    # kt/s of rim speed a N m takes away
    cdt = p('simulation/channel-dt')
    for n, xyz, share, side in ((0, U.MAIN_WHEELS[0], 0.34, 'left'), (1, U.MAIN_WHEELS[1], 0.34, 'right'), (2, U.NOSE_WHEEL, 0.32, None)):
        g = 'gear/unit[%d]/' % n
        fs, bl, wl = U.model_to_station(*xyz)
        mw = U.MTOW_KG * share
        k = mw * 9.80665 / 0.09 * 0.0685218                      # lbf/ft, as the gear's (fdm_file)
        c = 0.4 * 2 * math.sqrt(mw * 9.80665 / 0.09 * mw) * 0.0685218
        for x in ('spin-kt', 'touchdown-sink-fps', 'touchdown-slip-kt'):
            o.declare(g + x, 0.0)
        dx = Q(D(p('inertia/cg-x-in'), v(fs)), v(12.0))          # ahead of the CG, ft
        dy = Q(D(v(bl), p('inertia/cg-y-in')), v(12.0))          # right of it
        sink = o.fn(g + 'sink-now-fps', S(p('velocities/v-down-fps'), M(p('velocities/p-rad_sec'), dy), NEG(M(p('velocities/q-rad_sec'), dx))),
                    desc='the wheel\'s own speed downwards')
        down = GT(p(g + 'WOW'), v(0.5))
        kt = M(p('velocities/vg-fps'), v(0.592484))              # the ground speed (JSBSim's wheel speed is 0 in the air)
        load = MAX(v(0), S(M(v(k), p(g + 'compression-ft')), M(v(c), p(g + 'compression-velocity-fps'))))
        torque = S(v(WHEEL_DRAG_NM), M(MAX(p('/controls/gear/brake-' + side), p('/controls/gear/brake-parking')), v(WHEEL_BRAKE_NM))) if side \
            else v(WHEEL_DRAG_NM)
        o.fn(g + 'slip-now-kt', MAX(v(0), D(kt, p(g + 'spin-kt'))), desc='the ground speed less the rim\'s')
        o.fn(g + 'spin-next', IFTHEN(down, MIN(kt, S(p(g + 'spin-kt'), M(v(up), load, cdt))),
                                     MAX(v(0), D(p(g + 'spin-kt'), M(v(kd), torque, cdt)))), out=g + 'spin-kt',
             desc='the rim\'s speed: spun up by the ground, rolling, spinning down in the air')
        o.fn(g + 'touchdown-sink-next', IFTHEN(down, p(g + 'touchdown-sink-fps'), sink), out=g + 'touchdown-sink-fps',
             desc='as the weight came on, held')
        o.fn(g + 'touchdown-slip-next', IFTHEN(down, p(g + 'touchdown-slip-kt'), p(g + 'slip-now-kt')), out=g + 'touchdown-slip-kt',
             desc='likewise')


START_HOLD_S = (0.5, 4.0)  # an in-air start: the rotor settles, then the controls are trimmed on the held airframe, until
START_TAU_S = 0.4          # the trim loops' time constant
# the model's control Jacobian inverted (startgains.py, 2200 kg, 1000 ft): [control][acceleration], d(coll, lon, lat, ped)
# over d(wdot ft/s2, pdot, qdot, rdot rad/s2), at each true airspeed
START_JINV_KT = (0, 30, 60, 90, 105)
START_JINV = (
    ((-0.01221, -0.00004, -0.00242, -0.00009), (0.00445, -0.00746, -0.36469, -0.00772), (0.00104, 0.14280, -0.01031, 0.02638), (-0.01987, 0.01528, 0.00221, 0.21928)),   # 0 kt
    ((-0.01253, -0.00106, -0.03130, 0.00024), (-0.00187, -0.01210, -0.39022, 0.00070), (0.00099, 0.14339, -0.01577, 0.02672), (-0.01525, 0.01417, -0.03045, 0.22235)),   # 30 kt
    ((-0.01216, -0.00186, -0.05877, -0.00074), (-0.00877, -0.01211, -0.36974, -0.00457), (-0.00101, 0.14286, 0.00188, 0.02754), (-0.01081, 0.01065, -0.07286, 0.19006)),   # 60 kt
    ((-0.01135, -0.00272, -0.09513, -0.00192), (-0.00853, -0.01203, -0.40309, -0.00772), (-0.00203, 0.14299, 0.00568, 0.02813), (-0.00763, 0.00905, -0.09511, 0.16253)),   # 90 kt
    ((-0.01162, -0.00325, -0.11448, -0.00271), (-0.00992, -0.01228, -0.41977, -0.00946), (-0.00251, 0.14278, 0.00626, 0.02818), (-0.00698, 0.00822, -0.10384, 0.15001)),   # 105 kt
)


def s_start_trim(o, init, run0):
    """FlightGear starting in the air (the engine running from the start): the airframe held where FlightGear put it, its
    integration off, for START_HOLD_S[1] s, while from START_HOLD_S[0] s the collective, FlightGear's pitch, roll and
    rudder trims are trimmed on the vertical and angular accelerations (as the MV-22B model's in-air start, without its
    pivoting), each through a 0.05 s lag against the rotor's 3-per-rev vibration, together: the accelerations are
    multiplied by the inverse of the model's control Jacobian at the airspeed (startgains.py), so that each control nulls
    its own acceleration with a time constant START_TAU_S without stirring the others (the collective's torque yaws and
    rolls the aircraft, the pedals roll it). Then let go, trimmed in pitch, roll, yaw and height at the start's attitude
    and speed: the speed and the attitude then drift as the aircraft's own modes take them, slowly.
    (FlightGear hands the trims to JSBSim as fcs/pitch-trim-cmd-norm = elevator trim, fcs/roll-trim-cmd-norm = aileron
    trim, fcs/yaw-trim-cmd-norm = minus the rudder trim: so the stick is the elevator and aileron trims and the pedal,
    right positive, the rudder trim.)"""
    t_ = p('simulation/sim-time-sec')
    a_, b_ = START_HOLD_S
    o.declare('/sim/presets/onground', 1)
    hold = o.fn('cockpit/start-hold', M(run0, LT(p('/sim/presets/onground'), v(0.5)), LT(t_, v(b_))), desc='1: starting in the air, held')
    trimming = GT(M(hold, GT(t_, v(a_))), v(0.5))
    # the airframe's integration off while held, each integrator as it was once let go
    for k in ('rate/rotational', 'rate/translational', 'position/rotational', 'position/translational'):
        prop, kept = 'simulation/integrator/' + k, 'cockpit/integrator-%s' % k.replace('/', '-')
        o.declare(kept, -1)
        o.fn(kept + '-next', IFTHEN(LT(p(kept), v(-0.5)), p(prop), p(kept)), out=kept)
        o.fn(kept + '-set', IFTHEN(GT(hold, v(0.5)), v(0), IFTHEN(LT(p(kept), v(-0.5)), p(prop), p(kept))), out=prop)
    cdt = p('simulation/channel-dt')
    accs = ('wdot', 'pdot', 'qdot', 'rdot')
    for a in accs:
        n = 'cockpit/start-%s' % a
        o.declare(n, 0.0)
        src = 'accelerations/%s-ft_sec2' % a if a == 'wdot' else 'accelerations/%s-rad_sec2' % a
        o.fn(n + '-next', S(p(n), M(D(p(src), p(n)), MIN(v(1), Q(cdt, v(0.05))))), out=n)
    o.fn('cockpit/start-kt', M(v(1 / 1.68781), p('velocities/vt-fps')))
    kt = 'cockpit/start-kt'
    ctls = (('/controls/al3/collective', (0.0, 1.0)), ('/controls/flight/elevator-trim', (-0.6, 0.6)),
            ('/controls/flight/aileron-trim', (-0.6, 0.6)), ('/controls/flight/rudder-trim', (-0.6, 0.6)))
    for i, (prop, lim) in enumerate(ctls):
        rate = S(*[M(T1(kt, START_JINV_KT, [Ji[i][j] for Ji in START_JINV]), p('cockpit/start-%s' % a)) for j, a in enumerate(accs)])
        o.fn('cockpit/start-trim-%d' % i, IFTHEN(trimming, MAX(v(lim[0]), MIN(v(lim[1]), D(p(prop), M(v(1 / START_TAU_S), cdt, rate)))), p(prop)),
             out=prop)


# ---------------------------------------------------------------- airframe constants and tables
AIRFRAME_EXECRATE = U.every(240)   # the airframe's loads at 240 Hz, whatever the FDM rate
SLOW_EXECRATE = U.every(240)
SLAP_EXECRATE = U.every(120)   # the slap's state for the sound: 120 Hz keeps the 3/rev and 6/rev (18, 35 Hz) unaliased
SLAP_TAU_S = 0.25              # ... through this lag, which takes those loads out
HUB_DRAG_FT2 = 5.4         # the rotor head, mast and swashplate: 0.5 m^2, the Bo 105's measured 4.2 ft^2 [TM-110356] scaled to
                           # the Alouette's larger articulated head (100 kg against 60 [E76]), an estimate
MISC_DRAG_FT2 = 4.0        # an estimate, not fitted (the report reads the published speeds against the power)


def _gear():
    """three 355 x 150 wheels (0.354 m, 0.165 m wide [MI]) and their struts, swing arms and trailing link (3-D model):
    drag areas along x, y, z (ft^2), circular cylinders and discs [Hoerner]"""
    wheel_front, wheel_side = 0.354 * 0.165, 0.354 ** 2 * math.pi / 4
    struts_front, struts_side, struts_top = 0.06, 0.08, 0.10
    fx = 3 * wheel_front * 0.25 + struts_front * 1.0
    fy = 3 * wheel_side * 1.0 + struts_side * 1.0
    fz = 3 * wheel_front * 1.0 + struts_top * 1.0
    return tuple(f * 10.7639 for f in (fx, fy, fz))


GEAR_F = _gear()


def stab_tables():
    """horizontal stabiliser (aspect ratio 7.1 between its end plates, symmetric section): 3-D lift slope from the
    section's 0.105/deg (Helmbold with the end plates' 1.3 effective aspect ratio gain), stall at 13 deg, flat plate
    beyond (normal force 1.2 sin a), as the Bo 105 model's"""
    ar = U.SPAN_H ** 2 / (U.S_H / 10.7639) * 1.3
    a3 = 2 * math.pi * ar / (2 + math.sqrt(4 + ar * ar)) * math.pi / 180
    xs = [0, 4, 8, 12, 13, 15, 18, 22, 30, 45, 60, 75, 90]
    cl, cd = [], []
    for a in xs:
        r = math.radians(a)
        if a <= 12:
            c = a3 * a; d = 0.009 + c * c / (math.pi * 0.9 * ar)
        elif a <= 13:
            c = a3 * 12.5; d = 0.009 + c * c / (math.pi * 0.9 * ar)
        else:
            cn = 1.2 * math.sin(r)
            w = min(1.0, (a - 13) / 9.0)
            c = (1 - w) * 0.85 + w * cn * math.cos(r); d = (1 - w) * 0.12 + w * cn * math.sin(r)
        cl.append(c); cd.append(d)
    return xs, cl, cd


def endplate_tables():
    """end plate (aspect ratio about 1.4): lift slope 2.4/rad (Helmbold), stall near 25 deg, flat plate beyond"""
    xs = [0, 5, 10, 15, 20, 25, 30, 45, 60, 75, 90]
    cl, cd = [], []
    for a in xs:
        r = math.radians(a)
        if a <= 20:
            c = 2.4 * r; d = 0.012 + c * c / (math.pi * 0.85 * 1.4)
        else:
            cn = 1.15 * math.sin(r)
            w = min(1.0, (a - 20) / 15.0)
            c = (1 - w) * 2.4 * math.radians(20) + w * cn * math.cos(r)
            d = (1 - w) * (0.012 + (2.4 * math.radians(20)) ** 2 / (math.pi * 0.85 * 1.4)) + w * cn * math.sin(r)
        cl.append(c); cd.append(d)
    return xs, cl, cd


# ---------------------------------------------------------------- mass, fuel and gear
EMPTY_CG_MM = 3085.0       # estimate: a pilot (85 kg) and full fuel sit near the middle of the range [TC]
TANK = (-0.25, 0.0, -0.05)                      # body frame, m: the cubic tank in the centre frame (3-D model's filler)
FUEL_MAX_LB = U.FUEL_USABLE_L * U.FUEL_KG_L * U.KG_LB
# loads: (name, default lb, max lb, body frame x, y, z m): seats from the 3-D model (the pilot's eye 1.45 m forward,
# 0.56 m right, the set file's view), the barycentric hook below the rotor axis [HA: 750 kg], the hoist at its door
POINT_MASSES = [('Pilot (right seat)', 85 * U.KG_LB, 300.0, (1.55, -0.56, -0.05)),
                ('Front passenger (left seat)', 0.0, 300.0, (1.55, 0.56, -0.05)),
                ('Rear bench, left', 0.0, 480.0, (0.65, 0.5, -0.05)),
                ('Rear bench, right', 0.0, 480.0, (0.65, -0.5, -0.05)),
                ('Cargo', 0.0, 900.0, (0.9, 0.0, -0.30)),
                ('Hook load', 0.0, 750 * U.KG_LB, (0.03, 0.0, -1.30)),
                ('Hoist load', 0.0, 175 * U.KG_LB, (-0.13, 1.33, -0.60))]
PM_TANK = len(POINT_MASSES)
FUEL_DEFAULT_LB = 0.6 * FUEL_MAX_LB


def mass_config(cg=None, weight=None, inertia=None):
    """Empty mass, CG and inertias from the component build-up (al3data.COMPONENTS), the empty mass the type's [VP]
    and its CG EMPTY_CG_MM (the build-up's moved to it); loads as point masses. For a test configuration (cg, weight,
    inertia): everything at that CG."""
    comps = U.COMPONENTS
    m0 = sum(c[1] for c in comps)
    k = U.EMPTY_KG / m0
    xs = sum(c[1] * c[2][0] for c in comps) / m0
    zs = sum(c[1] * c[2][2] for c in comps) / m0
    fs_target = EMPTY_CG_MM / 25.4
    dx = (U.model_to_station(xs, 0, zs)[0] - fs_target) / U.IN_PER_M      # body x shift (forward +)
    Ixx = Iyy = Izz = Ixz = 0.0
    for name, kg, (x, y, z), (kx, ky, kz) in comps:
        m = kg * k
        x = x + dx - (xs + dx); z = z - zs; y = y
        Ixx += m * (y * y + z * z + kx * kx); Iyy += m * (x * x + z * z + ky * ky); Izz += m * (x * x + y * y + kz * kz)
        Ixz += m * x * z
    I = (Ixx * U.KGM2, Iyy * U.KGM2, Izz * U.KGM2, -Ixz * U.KGM2)
    ecg = (fs_target, 0.0, U.model_to_station(0, 0, zs)[2])
    st = lambda xyz: U.model_to_station(*xyz)
    pms = [(n, w, mx, *st(xyz)) for n, w, mx, xyz in POINT_MASSES]
    tank = (FUEL_DEFAULT_LB, *st(TANK))
    if cg:
        pms = [(n, 0.0, mx, *cg) for n, w, mx, *_ in pms]
        return dict(empty=weight - tank[0], cg=cg, I=inertia or I, pms=pms, tank=(tank[0], *cg))
    return dict(empty=U.EMPTY_KG * U.KG_LB, cg=ecg, I=I, pms=pms, tank=tank)


STRUCTURE = [('NOSE', (2.95, 0.0, -0.45)), ('CHIN', (2.6, 0.0, -0.70)),
             ('BELLY_LEFT', (0.6, 0.85, -0.75)), ('BELLY_RIGHT', (0.6, -0.85, -0.75)),
             ('ROOF', (1.3, 0.0, 0.95)), ('ROTOR_HEAD', (0.03, 0.0, 1.90)), ('ENGINE', (-2.3, 0.0, 1.05)),
             ('BOOM_FRONT', (-2.4, 0.0, -0.25)), ('BOOM_AFT', (-5.3, 0.0, 0.15)),
             ('STAB_LEFT', (-4.95, 1.19, 0.4)), ('STAB_RIGHT', (-4.95, -1.19, 0.4)),
             ('TAIL_GUARD', (-6.25, 0.0, -0.49)), ('TAIL_ROTOR', (-6.25, -0.31, 1.62))]


def fdm_file(cg=None, weight=None, inertia=None, systems=()):
    mc = mass_config(cg, weight, inertia)
    # wheels: the tyres' load-deflection [MI: 550 kg rated with 142 mm static loaded radius from about 177 mm free,
    # 35 mm] in series with the oleo struts (not published: together about 9 cm static at the maximum mass); damping
    # 40 % of critical on a wheel's share
    k_tyre = 550 * 9.80665 / 0.035
    gear = []
    # the original aircraft's order (its systems read them by number): left, right, nose. The nose wheel castors, or
    # steers 45 deg with the pedals once its NWS switch is on (ctl/nose-castered)
    for name, xyz, share, steer, grp in (('LEFT_MAIN', U.MAIN_WHEELS[0], 0.34, 0, 'LEFT'),
                                         ('RIGHT_MAIN', U.MAIN_WHEELS[1], 0.34, 0, 'RIGHT'),
                                         ('NOSE', U.NOSE_WHEEL, 0.32, 45, 'NONE')):
        fs, bl, wl = U.model_to_station(*xyz)
        mw = U.MTOW_KG * share
        k = mw * 9.80665 / 0.09                     # N/m, tyre and oleo in series
        c = 0.4 * 2 * math.sqrt(k * mw)
        gear.append('''    <contact type="BOGEY" name="%s">
      <location unit="IN"><x>%.2f</x><y>%.2f</y><z>%.2f</z></location>
      <static_friction>0.8</static_friction><dynamic_friction>0.5</dynamic_friction><rolling_friction>0.02</rolling_friction>
      <spring_coeff unit="LBS/FT">%.0f</spring_coeff><damping_coeff unit="LBS/FT/SEC">%.0f</damping_coeff>
      <max_steer unit="DEG">%d</max_steer><brake_group>%s</brake_group><retractable>1</retractable>
    </contact>''' % (name, fs, bl, wl, k * 0.0685218, c * 0.0685218, steer, grp))
    for name, xyz in STRUCTURE:
        fs, bl, wl = U.model_to_station(*xyz)
        gear.append('''    <contact type="STRUCTURE" name="%s">
      <location unit="IN"><x>%.1f</x><y>%.1f</y><z>%.1f</z></location>
      <static_friction>0.8</static_friction><dynamic_friction>0.6</dynamic_friction>
      <spring_coeff unit="LBS/FT">20000</spring_coeff><damping_coeff unit="LBS/FT/SEC">1000</damping_coeff>
    </contact>''' % (name, fs, bl, wl))
    pml = [(n, w, x, y, z) for n, w, mx, x, y, z in mc['pms']] + [('Fuel tank', *mc['tank'])]
    pms = '\n'.join('''    <pointmass name="%s">
      <weight unit="LBS">%.1f</weight>
      <location unit="IN"><x>%.1f</x><y>%.1f</y><z>%.1f</z></location>
    </pointmass>''' % pm for pm in pml)
    I = mc['I']
    eye = U.model_to_station(1.45, -0.557, 0.59)
    sysx = '\n'.join('  <system file="%s"/>' % f for f in ('al3-controls', 'al3-rotor', 'al3-airframe', 'al3-drive', 'al3-outputs') + tuple(systems))
    return '''<?xml version="1.0"?>
<!-- Sud Aviation SA 316B Alouette III: generated by fdm/genal3.py -->
<fdm_config name="Alouette III" version="2.0" release="BETA">
  <fileheader>
    <author>generated</author>
    <description>Sud Aviation SA 316B Alouette III: articulated main rotor blade by blade, single-shaft Artouste IIIB</description>
  </fileheader>
  <metrics>
    <wingarea unit="FT2">%.1f</wingarea>
    <wingspan unit="FT">%.2f</wingspan>
    <chord unit="FT">%.3f</chord>
    <location name="AERORP" unit="IN"><x>%.2f</x><y>0.0</y><z>%.2f</z></location>
    <location name="EYEPOINT" unit="IN"><x>%.1f</x><y>%.1f</y><z>%.1f</z></location>
    <location name="VRP" unit="IN"><x>%.2f</x><y>0.0</y><z>%.2f</z></location>
  </metrics>
  <mass_balance>
    <ixx unit="SLUG*FT2">%.0f</ixx><iyy unit="SLUG*FT2">%.0f</iyy><izz unit="SLUG*FT2">%.0f</izz><ixz unit="SLUG*FT2">%.0f</ixz>
    <emptywt unit="LBS">%.1f</emptywt>
    <location name="CG" unit="IN"><x>%.2f</x><y>%.2f</y><z>%.2f</z></location>
%s
  </mass_balance>
  <ground_reactions>
%s
  </ground_reactions>
%s
  <aerodynamics file="Systems/al3-aero.xml"/>
</fdm_config>
''' % ((math.pi * U.R ** 2, 2 * U.R, U.CHORD, U.FS_MR, U.WL_MR - 40.0) + tuple(eye) + (U.FS_MODEL0, U.WL_MODEL0) + I + (mc['empty'],) +
       tuple(mc['cg']) + (pms, '\n'.join(gear), sysx))


# the original FlightGear aircraft's systems kept in its build (its JSBSim rotor, engine and control systems, gear and
# steering are replaced): terrain and crash detection, the electrical system and lights, instruments, the autopilot,
# Amelia, the cockpit's and the gear's animations, the annunciator
ORIGINAL_SYSTEMS = ('steady-behaviour', 'crash-detect', 'electrical-fdm-custom', 'electrical-fdm', 'electrical-lights', 'instrumentation',
                    'autopilot', 'amelia', 'animation-dev', 'annunciator')
# and the rates they run at (Hz): crash detection's over-g test at 240, the rest (electrics, lights, gauges, the
# original autopilot and Amelia, the gear and door animations) at 120; the build writes them into their channels
ORIGINAL_HZ = {'crash-detect': 240}
ORIGINAL_HZ_DEFAULT = 120


def rate_original_systems(sysdir):
    """execrate on every channel of the original systems' files, for their rates at the FDM rate"""
    for name in ORIGINAL_SYSTEMS:
        path = os.path.join(sysdir, name + '.xml')
        t = open(path).read()
        k = U.every(ORIGINAL_HZ.get(name, ORIGINAL_HZ_DEFAULT))
        t2 = re.sub(r'<channel\b([^>]*)>', lambda m: '<channel%s%s>' % (re.sub(r'\s+execrate="\d+"', '', m.group(1)),
                                                                       '' if k == 1 else ' execrate="%d"' % k), t)
        if t2 != t:
            open(path, 'w').write(t2)


# offline test configurations: (file name, CG stations, gross weight lb, inertias): the type's maximum mass at the
# middle of the CG range, and the light weight of the brochure's second column [TD]
TEST_CASES = [('alouette3', None, None, None),
              ('al3-2200', (2930 / 25.4, 0.0, 30.0), 2200 * U.KG_LB, None),
              ('al3-1750', (2930 / 25.4, 0.0, 30.0), 1750 * U.KG_LB, None)]


def build(out, fg=False, systems=()):
    mdl = Model()
    mdl.controls()
    mdl.rotor_states()
    mdl.main_rotor()
    mdl.tail_rotor()
    mdl.airframe()
    os.makedirs(os.path.join(out, 'Systems'), exist_ok=True)
    texts = {'al3-controls.xml': mdl.ctl.xml(), 'al3-rotor.xml': mdl.rot.xml(), 'al3-airframe.xml': mdl.air.xml(),
             'al3-drive.xml': engines.build(PM_TANK).xml(), 'al3-outputs.xml': mdl.outputs().xml(),
             'al3-aero.xml': '<!-- generated by fdm/genal3.py -->\n<aerodynamics>\n%s\n%s\n</aerodynamics>\n' % (mdl.g.xml(), mdl.axes())}
    # the mirror image: every system but the controls' mirror channel reads the 'mir/' copies
    for n in texts:
        if n == 'al3-controls.xml':
            head, sep, rest = texts[n].partition('<channel name="pilot">')
            texts[n] = head + sep + mirror_text(rest)
        elif n != 'al3-outputs.xml':
            texts[n] = mirror_text(texts[n])
    if INLINE:
        inline_single_use(texts, extra=[fdm_file(), cockpit().xml()], keep=KEEP)
    for name, t in texts.items():
        open(os.path.join(out, 'Systems', name), 'w').write('<?xml version="1.0"?>\n' + t)
    if fg:
        # FlightGear's FDM rate: the set file's /sim/model-hz
        sp = os.path.join(out, 'Alouette-3-set.xml')
        if os.path.exists(sp):
            st = open(sp).read()
            st2 = re.sub(r'<model-hz type="int">\d+</model-hz>', '<model-hz type="int">%d</model-hz>' % U.FDM_HZ, st)
            if st2 != st:
                open(sp, 'w').write(st2)
        open(os.path.join(out, 'Systems', 'al3-cockpit.xml'), 'w').write('<?xml version="1.0"?>\n' + cockpit().xml())
        rate_original_systems(os.path.join(out, 'Systems'))
        open(os.path.join(out, 'alouette3.xml'), 'w').write(fdm_file(systems=('al3-cockpit',) + tuple(systems)))
        return len(mdl.g.items)
    for name, cg, w, I in TEST_CASES:
        open(os.path.join(out, name + '.xml'), 'w').write(fdm_file(cg, w, I))
    return len(mdl.g.items)


if __name__ == '__main__':
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    out = args[0] if args else 'out'
    if '--no-inline' in sys.argv:
        INLINE = False
    fg = '--fg' in sys.argv
    n = build(out, fg=fg, systems=ORIGINAL_SYSTEMS if fg else ())
    print('wrote', out, '-', n, 'aerodynamic functions')
