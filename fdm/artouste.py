"""Alouette III drive system: the Turbomeca Artouste IIIB single-shaft turboshaft, its centrifugal clutch and
freewheel, the rotor brake and the rotor speed degree of freedom, as a JSBSim system (FCS components, no Nasal).

Sources (al3data): [TE] the Artouste III type certificate: 33 500 +-200 rpm in steady operation, output shaft 5773 rpm,
take-off 420 kW (IIIB) and maximum continuous 405 kW at the reduction gear, T4 limits 630 C starting, 550 C take-off,
500 C continuous; [TC] 640 kW of thermal power on the ground at sea level, the transmission's 440 kW take-off and 420
kW continuous; [HA] the gearbox-limited 420 kW held up to 4000-4500 m in the standard atmosphere, a centrifugal clutch;
[E77] Turbomeca: the Artouste's hydromechanical governor holds the engine speed constant whatever the power, an
isochronous PI (remanent error under 0.25 %), and the pilot only selects the collective; [WA] 0.46 kg/kWh, pressure
ratio 5.2, 4.3 kg/s; [SR] a cockpit recording of an SA 316B's start: idle 16 330 rpm; [AO] the AOPA Pilot's flight
test of an SE 3160: idle about 15 000 rpm; [RW] a maintenance column on the Lama's Artouste IIIB1: idle 17 000-18 000
rpm, the starter off line about 14 000 rpm, the fuel flow lever closed for the start; [LM] a flight simulator add-on's Lama manual,
its procedures after the flight manual: the fuel flow lever at 35 % for idle, the rotor starts turning between
19 500 and 21 000 rpm as the lever is advanced (the clutch), the lever full forward governs 33 500 rpm; the rotor
brake below 175 rpm.

A single-shaft engine: compressor and turbine on one spool, which the clutch couples to the rotor once it is engaged,
so the rotor speed is the engine's (94.85 to 1). The engine model (estimates where noted):
  - the spool's net torque (at the rotor, ft lb) from the fuel beyond what it needs to turn unloaded at its speed:
        Q = Q_th (Wf_c - Wf_nl(n)) / (Wf_th - Wf_nl(1)) n^0.5,  Wf_nl(n) = WF_NL n^2.5
    Wf_c the fuel flow corrected to sea level standard (Wf / (delta sqrt theta)), n the spool speed over 33 500 rpm;
    Q_th the thermal maximum's torque, 640 kW at 100 % [TC];
  - fuel at sea level: no load 70 kg/h at 100 % (estimate: a single-shaft engine turns its compressor at full speed),
    186 kg/h at the 405 kW rating (0.46 kg/kWh [WA]), so 253 kg/h at the 640 kW thermal maximum; which gives 205 l/h at
    a 300 kW cruise, the type's quoted 200 l/h [HA];
  - the thermal maximum lapses with altitude as 640 kW delta theta^-2.2: the gearbox's 420 kW up to 4250 m [HA]
    (set so, from [HA]'s statement; not fitted to the hover ceilings, which the report reads against the power);
  - T4 from the fuel's share of the thermal maximum's beyond no load, 1.99 T_amb (300 C at sea level) unloaded to
    550 C at the thermal maximum [TE], with a 1.5 s lag;
  - the governor: an isochronous PI on the engine speed [E77], its fuel limited to the thermal maximum (550 C) and,
    decelerating, to 40 % of the no-load fuel; gains estimates (1 % of speed error: half the thermal range of fuel;
    reset 1 s), its fuel reaching the turbine with a 0.1 s lag;
  - the fuel flow lever: idle, 16 330 rpm (the recording's [SR], between the Alouette's 15 000 [AO] and the Lama's
    17 000-18 000 [RW]), up to 35 % of its travel [LM], then the governor's reference rises to 33 500 rpm at the stop;
  - the start [E77, RW, SR]: the starter cranks the spool (its torque falling to nothing at 17 %), the start system's
    fixed fuel flow held until idle, light-off at 16 % about 14.5 s after the starter, the starter off line at 14 000
    rpm (42 %), idle about 30 s after light-off, as in a recorded start [SR];
  - the centrifugal clutch: slipping, its shoes' sliding friction, from 19 000 rpm [AO], fitted to two recorded starts'
    take-ups [SR] (CLUTCH_SLIP_*: about 1150 ft lb at the rotor at 62 % of the spool's speed, 1490 at 80 %); locked,
    a capacity rising from 19 000 to full (twice the thermal maximum's torque) at 24 000 rpm, so that it carries any
    flight torque; the freewheel passes torque only to the rotor.
"""
import math
from xmlfun import *
import al3data as U

G0 = 32.174
OMEGA0 = U.OMEGA
GEAR = U.GEAR_ENGINE                       # engine / rotor speed
KW_FTLB = 737.562                          # (ft lb/s) per kW
Q_TH = U.P_THERMAL_KW * KW_FTLB / OMEGA0   # ft lb at the rotor: the thermal maximum at 100 % (12 760)
WF_NL, WF_MCP = 70.0, U.SFC * U.P_MCP_KW   # kg/h at sea level, 100 %
WF_TH = WF_NL + (WF_MCP - WF_NL) * U.P_THERMAL_KW / U.P_MCP_KW
LAPSE_THETA = -2.2                         # P_th = 640 kW delta theta^LAPSE_THETA: 420 kW to 4250 m [HA]
# rotating inertias about the shaft, slug ft^2: the blades (rigid), the hub (its 64 kg about 0.2 m [E76]) and the
# mast and main gearbox's output (estimate), and the Artouste's spool referred to the rotor: about 0.15 kg m^2
# (compressor impeller, axial stage and three turbine discs; estimate) times 94.85^2
J_BLADES = U.NB * (U.I_B + 2 * U.E_FLAP * U.S_B + U.W_BLADE / G0 * U.E_FLAP ** 2)
J_HUB = (U.M_HUB * 0.2 ** 2 + 8.0) * U.KGM2
J_ENG = 0.15 * GEAR ** 2 * U.KGM2
J_ROTOR = J_BLADES + J_HUB
ACCESSORY_FTLB = 6.0 * KW_FTLB / OMEGA0    # accessories (hydraulics, generator), 6 kW (estimate)
FRICTION_FTLB = 150.0                      # the transmission's breakaway friction at the rotor (estimate)
BRAKE_FTLB = 4000.0                        # the rotor brake's torque at the rotor (estimate)
OMEGA_STUCK = 0.05                         # rad/s: below it the rotor can stick
SLIP_W = 0.1                               # rad/s (at the rotor): the slipping clutch's torque rises to its full value over
                                           # this slip (its sliding friction holds until the speeds meet and it locks: the lock
                                           # needs the rotor within LOCK_W of the spool, or the speeds crossing in a step)
LOCK_W = 0.2                               # rad/s: the clutch locks once the speeds are this close (or cross)
GEARBOX_LOSS = 0.025                       # of the main rotor torque (estimate)
N_IDLE, N_ENGAGE0, N_ENGAGE1 = 16330.0, 19000.0, 24000.0
N_STARTER_OFF = 14000.0 / U.N_ENGINE       # the starter off line [RW]
N_STARTER_FREE = 0.17                      # the starter-generator's torque falls to nothing at 17 % (an electric starter's
                                           # characteristic): the crank fast at first, creeping to light-off (fitted [SR])
N_LIGHT = 0.16                             # light-off at 16 % [SR]
LEVER_IDLE = 0.35
Q_CLUTCH = 2.0 * Q_TH                       # the locked clutch's capacity at full engagement (it carries any flight torque)
# the slipping clutch's torque at the rotor against the spool's speed (rpm): its shoes engage from 19 000 [AO]; above,
# fitted to two recorded starts' take-ups [SR], the rotor from rest at flat pitch, its brake let go, the spool held:
# ZU-REC's at 80 %, the rotor to 55 % in 21 s, 75 % in 39 s, locked at 80 % in 45 s (the model: 19, 36, 44 s); A-301's
# at 61-63 %, locked at 63 % in 64 s (the model: at 61 % in 62 s). The locked clutch carries Q_CLUTCH
CLUTCH_SLIP_RPM = (19000.0, 20800.0, 26800.0, 33500.0)
CLUTCH_SLIP_FTLB = (0.0, 1154.0, 1489.0, 3175.0)    # above 80 % as a centrifugal clutch's, its shoes' force with the spool
                                                    # speed squared past their engagement: 2.13 times at 100 %
Q_TRANS_MCP = U.P_TRANS_MCP_KW * KW_FTLB / OMEGA0   # ft lb: the transmission's 420 kW continuous limit [TC] at 353.2 rpm
GOV_KP = 0.5 * (WF_TH - WF_NL) / 0.01      # kg/h per unit speed error
GOV_TI = 1.0                               # s
TAU_FUEL = 0.1                             # s
WF_START = 16.0                            # kg/h, the start system's fuel at sea level until idle: fitted to ZU-REC's start
                                           # [SR], 30 % 12.5 s after light-off, 45 % at 23.5 s, idle by 30.5 s (the model: 11.2,
                                           # 25.0, 29.8 s)
Q_STARTER = 1350.0                         # ft lb at the rotor at rest, falling to nothing at N_STARTER_FREE: fitted to the
                                           # spool's 10 % 4 s after the starter and light-off at 14.5 s [SR] (the model: 4.1,
                                           # 14.1 s)


def build(pm_tank, fcs_name='Alouette III drive'):
    f = FCS(fcs_name)
    for name, val in (('eng/fuel-valve', 1), ('eng/starter-cmd', 0), ('eng/flow-lever', 1.0), ('eng/ignition', 1),
                      ('eng/n-rad_sec', 0.0), ('eng/lit', 0), ('eng/engaged', 0), ('eng/starter-on', 0),
                      ('eng/wf-c', 0.0), ('eng/gov-int', 0.0), ('eng/t4-degc', 15.0), ('eng/init-running', 1),
                      ('eng/q-ftlb', 0.0), ('eng/starter-prev', 0)):
        f.declare(name, val)
    f.declare('/consumables/fuel/tank[0]/level-lbs', 500.0)
    f.declare('/controls/rotor/brake', 0.0)
    f.declare('/sim/freeze/fuel', 0)
    f.declare('eng/gov-kp', GOV_KP)
    f.declare('eng/gov-ti', GOV_TI)
    f.declare('eng/lapse-theta', LAPSE_THETA)
    f.declare('drive/hold-rpm', 0)
    dt = p('simulation/dt')
    cdt = p('simulation/channel-dt')
    init = GT(p('ctl/init-fast'), v(0.5))
    clamp = lambda x, lo, hi: MAX(v(lo), MIN(v(hi), x))
    f.channel('start state')
    run0 = f.fn('eng/running-at-start', MAX(p('eng/init-running'), GT(p('position/h-agl-ft'), v(50))))
    f.fn('eng/lever-init', IFTHEN(init, IFTHEN(run0, v(1), v(0)), p('eng/flow-lever')), out='eng/flow-lever')
    f.fn('drive/brake-init', IFTHEN(init, IFTHEN(run0, v(0), v(1)), p('/controls/rotor/brake')), out='/controls/rotor/brake')
    W = p('rotor/omega-rad_sec')
    f.channel('ambient', execrate=U.every(240))
    th = f.fn('eng/theta', Q(p('atmosphere/T-R'), v(518.67)))
    dl = f.fn('eng/delta', Q(p('atmosphere/P-psf'), v(2116.22)))
    dsth = f.fn('eng/delta-sqrt-theta', M(dl, SQ(th)))
    f.fn('eng/wf-max-c', S(v(WF_NL), M(v(WF_TH - WF_NL), tag('pow', th, S(p('eng/lapse-theta'), v(-0.5))))),
         desc='the thermal maximum\'s corrected fuel (T4 550 C), kg/h')
    f.fn('eng/tamb-k', Q(p('atmosphere/T-R'), v(1.8)))
    f.channel('rotor load')
    Qmr = p('aero/rotor/torque-ftlb')
    Qtr = M(v(U.GEAR_TR), p('aero/rotor/tr/torque-ftlb'))
    Wn = Q(W, v(OMEGA0))
    f.fn('drive/loss-ftlb', S(M(v(ACCESSORY_FTLB), SQR(Wn)), M(v(GEARBOX_LOSS), ABS(Qmr))), desc='accessories and gearbox losses')
    # the brake, and the transmission's breakaway friction (estimate), both smooth at standstill: without it the blades'
    # swing on the hub at rest rocks the rotor, and its speed's floor at zero would ratchet that into turning
    f.fn('drive/brake-ftlb', M(S(M(p('/controls/rotor/brake'), v(BRAKE_FTLB)), v(FRICTION_FTLB)), Q(W, S(ABS(W), v(0.3)))),
         desc='rotor brake and friction, smooth at standstill')
    Qload = f.fn('drive/load-ftlb', S(Qmr, Qtr, p('drive/loss-ftlb'), p('drive/brake-ftlb')))
    f.channel('engine', execrate=U.every(240))
    N = p('eng/n-rad_sec')
    n = f.fn('eng/n', Q(N, v(OMEGA0)), desc='spool speed over 33 500 rpm')
    f.fn('eng/rpm', M(v(U.N_ENGINE), n))
    f.fn('eng/n-pct', M(v(100), n))
    lit = p('eng/lit')
    fuel_ok = GT(p('/consumables/fuel/tank[0]/level-lbs'), v(0.5))
    valve = M(GT(p('eng/fuel-valve'), v(0.5)), fuel_ok)
    # the starter: pressed, it runs until the spool reaches N_STARTER_OFF (or the button is released before light-off)
    st = p('eng/starter-on')
    # (the starter switch's rising edge starts it: left on, it does not crank again once the engine has run down)
    sw = GT(p('eng/starter-cmd'), v(0.5))
    press = M(sw, LT(p('eng/starter-prev'), v(0.5)))
    f.fn('eng/starter-next', IFTHEN(GT(S(press, st), v(0.5)), LT(n, v(N_STARTER_OFF)), v(0)), out='eng/starter-on')
    f.fn('eng/starter-prev-next', sw, out='eng/starter-prev')
    lightable = M(GT(p('eng/ignition'), v(0.5)), valve, GT(n, v(N_LIGHT)), st)
    keep = M(valve, GT(n, v(0.10)))
    f.fn('eng/lit-next', IFTHEN(init, run0, M(keep, MAX(lit, lightable))), out='eng/lit')
    # the governor's reference from the fuel flow lever
    lev = p('eng/flow-lever')
    nref = f.fn('eng/n-ref', Q(S(v(N_IDLE), M(v(U.N_ENGINE - N_IDLE), clamp(Q(D(lev, v(LEVER_IDLE)), v(1 - LEVER_IDLE)), 0, 1))), v(U.N_ENGINE)),
                desc='governed speed over 33 500 rpm')
    err = f.fn('eng/n-err', D(nref, n))
    wf_nl = f.fn('eng/wf-nl-c', M(v(WF_NL), tag('pow', MAX(n, v(0)), v(2.5))))
    # the fuel's ceiling: the thermal maximum's T4 (550 C) at this airflow (the spool's speed), as the T4 below has it
    wmax = f.fn('eng/wf-limit-c', S(wf_nl, M(D(p('eng/wf-max-c'), v(WF_NL)), MAX(n, v(0.1)))))
    wmin = M(v(0.4), wf_nl)
    pi_out = S(p('eng/gov-int'), M(p('eng/gov-kp'), err), wf_nl)
    # the start: from the starter's press until the spool reaches idle the fuel is the start system's flow (the governor
    # takes over at idle), which sets the pace from light-off to idle
    f.declare('eng/start-phase', 0)
    phase = p('eng/start-phase')
    f.fn('eng/start-phase-next', IFTHEN(init, v(0), IFTHEN(GT(st, v(0.5)), v(1),
                                                           IFTHEN(GT(n, v(N_IDLE / U.N_ENGINE - 0.003)), v(0), phase))),
         out='eng/start-phase')
    starting = M(GT(phase, v(0.5)), LT(n, v(N_IDLE / U.N_ENGINE)))
    # until idle the fuel is the start flow; above, the governor's
    cmd = f.fn('eng/wf-cmd-c', IFTHEN(starting, v(WF_START), MAX(wmin, MIN(wmax, pi_out))))
    # the integrator: held while the fuel is limited (no wind-up), reset in the start
    sat = MAX(GT(pi_out, wmax), LT(pi_out, wmin))
    gi = p('eng/gov-int')
    # (in the start it is set so that the governor takes over at idle from the start flow, without a step)
    f.fn('eng/gov-int-next', IFTHEN(init, IFTHEN(run0, D(p('eng/wf-for-load'), wf_nl), v(0)),
                                     IFTHEN(starting, D(v(WF_START), S(wf_nl, M(p('eng/gov-kp'), err))),
                                            IFTHEN(sat, gi, S(gi, M(cdt, Q(p('eng/gov-kp'), p('eng/gov-ti')), err))))),
         out='eng/gov-int')
    # the fuel reaching the turbine, after its lag; none with the valve shut or before light-off
    wf = p('eng/wf-c')
    f.fn('eng/wf-c-next', IFTHEN(init, M(run0, p('eng/wf-for-load')), M(lit, S(wf, M(Q(cdt, v(TAU_FUEL)), D(cmd, wf))))), out='eng/wf-c')
    f.fn('eng/fuel-flow-kgh', M(p('eng/wf-c'), dsth))
    f.fn('eng/fuel-flow-pph', M(v(U.KG_LB), p('eng/fuel-flow-kgh')))
    # the spool's net torque at the rotor, and the fuel the load needs (for a start in flight)
    # (unlit, the compressor's work is a drag: the no-load fuel's worth of torque, negative)
    f.fn('eng/q-next', M(v(Q_TH / (WF_TH - WF_NL)), dsth, D(p('eng/wf-c'), wf_nl), SQ(MAX(n, v(0)))), out='eng/q-ftlb')
    f.fn('eng/wf-for-load', S(v(WF_NL), M(v((WF_TH - WF_NL) / Q_TH), Q(p('drive/load-ftlb'), MAX(dsth, v(0.05))))))
    f.fn('eng/power-kw', Q(M(p('eng/q-ftlb'), N), v(KW_FTLB)))
    # T4: unloaded 1.99 T_amb, 550 C at the thermal maximum, with the fuel's share of the airflow's (n) range
    xs = Q(D(p('eng/wf-c'), wf_nl), M(D(p('eng/wf-max-c'), v(WF_NL)), MAX(n, v(0.1))))
    t4nl = M(v(1.99), p('eng/tamb-k'))
    t4ss = f.fn('eng/t4-ss-k', IFTHEN(lit, S(t4nl, M(D(v(823.15), t4nl), MAX(v(-0.3), xs))), p('eng/tamb-k')))
    t4 = p('eng/t4-degc')
    f.fn('eng/t4-next', IFTHEN(init, D(t4ss, v(273.15)), S(t4, M(Q(cdt, IFTHEN(lit, v(1.5), v(20))), D(D(t4ss, v(273.15)), t4)))), out='eng/t4-degc')
    f.channel('fuel', execrate=U.every(240))
    frozen = GT(p('/sim/freeze/fuel'), v(0.5))
    t0 = '/consumables/fuel/tank[0]/level-lbs'
    f.fn('fuel/burn', IFTHEN(frozen, p(t0), MAX(v(0), D(p(t0), M(p('eng/fuel-flow-pph'), cdt, v(1 / 3600.0))))), out=t0)
    f.fn('fuel/mass', p(t0), out='inertia/pointmass-weight-lbs[%d]' % pm_tank)
    f.channel('drive train')
    # the clutch's capacity with the spool's speed, and the freewheel
    rpm = M(v(U.N_ENGINE / OMEGA0), N)
    cap = f.fn('drive/clutch-cap-ftlb', M(v(Q_CLUTCH), SQR(clamp(Q(D(rpm, v(N_ENGAGE0)), v(N_ENGAGE1 - N_ENGAGE0)), 0, 1))))
    eng = p('eng/engaged')
    qe = p('eng/q-ftlb')
    qst = M(st, v(Q_STARTER), MAX(v(0), D(v(1), Q(n, v(N_STARTER_FREE)))))
    # slipping (not engaged): the clutch drags the rotor with its capacity while the spool turns faster than the rotor,
    # rising to it over SLIP_W of slip (the freewheel: nothing the other way)
    qsl = f.fn('drive/clutch-slip-cap-ftlb', T1('eng/rpm', CLUTCH_SLIP_RPM, CLUTCH_SLIP_FTLB),
                desc='the slipping clutch\'s torque at this spool speed')
    qcl = f.fn('drive/clutch-ftlb', M(qsl, MIN(v(1), MAX(v(0), Q(D(N, W), v(SLIP_W))))))
    f.fn('drive/net-torque-ftlb', D(IFTHEN(eng, qe, qcl), Qload), desc='into the rotor (with the spool when engaged), less its load')
    f.fn('drive/j-eng', M(eng, v(J_ENG)))
    wdot = p('drive/omega-dot')
    w_new = f.fn('drive/omega-new', IFTHEN(GT(p('drive/hold-rpm'), v(0.5)), W,
                                          IFTHEN(GT(p('drive/stuck'), v(0.5)), v(0), MAX(v(0), S(W, M(dt, wdot))))))
    n_free = f.fn('eng/n-free', MAX(v(0), S(N, M(dt, Q(S(qe, qst, NEG(qcl)), v(J_ENG))))))
    # engaged, the clutch carries the spool's torque less what accelerates it: it lets go when that exceeds its capacity or
    # falls below zero (the freewheel: the rotor overruns the engine in autorotation); slipping, it locks once the spool
    # has caught up with the rotor and its capacity holds the load
    q_in = S(qe, NEG(M(v(J_ENG), wdot)))
    stay = M(GT(q_in, v(-50)), LT(q_in, cap))
    # (the speeds meet when their difference changes sign in this step: the rotor catching up with the spool as the clutch
    # slips, or the spool catching up with the rotor from below after an autorotation; the engine driving)
    meet = MAX(LT(M(D(N, W), D(n_free, w_new)), v(1e-9)), LT(ABS(D(n_free, w_new)), v(LOCK_W)))
    lock = M(meet, GT(qe, v(0)), GT(cap, M(v(0.9), qe)))
    # the torque into the main rotor shaft, whose reaction acts on the airframe: the clutch's less the tail rotor and the
    # losses; with the rotor speed held (trims), what the rotor takes. (Before the clutch's state updates: the rotor's
    # acceleration in q_in is this step's, with the state it had; with the new one the step the clutch locked put the
    # spool's inertia times the slipping hub's acceleration, -400 000 ft lb, on the airframe)
    held = GT(S(init, p('drive/hold-rpm')), v(0.5))
    f.fn('drive/mr-torque-ftlb', IFTHEN(held, D(Qmr, p('aero/rotor/torque-inertial-ftlb')),
                                        D(IFTHEN(eng, q_in, qcl), S(Qtr, p('drive/loss-ftlb'), p('drive/brake-ftlb')))),
         desc='torque into the main rotor shaft; its reaction acts on the airframe')
    f.fn('drive/torque-pct', M(v(100.0 / Q_TRANS_MCP), p('drive/mr-torque-ftlb')),
         desc='the main rotor shaft torque in % of the transmission\'s continuous rating (420 kW at 353.2 rpm [TC]); negative '
              'when the rotor drives the tail rotor and gearbox in autorotation')
    f.fn('eng/engaged-next', IFTHEN(init, run0, IFTHEN(eng, stay, lock)), out='eng/engaged')
    w_init = M(run0, v(OMEGA0))
    f.fn('eng/n-next', IFTHEN(init, w_init, IFTHEN(p('eng/engaged'), w_new, n_free)), out='eng/n-rad_sec')
    f.fn('drive/omega-next', IFTHEN(init, w_init, w_new), out='rotor/omega-rad_sec')
    return f


def rotor_speed(r, lag_forcing, Srl, Ml, init):
    """The rotor speed's acceleration relative to the airframe (drive/omega-dot), in the rotor system r, solved with
    the blades' lag accelerations (as the Bo 105 model): per blade Ml dl'' = F_k - Srl W'_abs (F_k the rest of its lag
    equation), and J W' = Q_net - sum Srl dl_k'' + J_blades r'_s (the rotor keeps its own angular momentum as the
    airframe yaws about the shaft), with W'_abs = W' - r'_s, give
        (J - NB Srl^2 / Ml) W' = Q_net - Srl / Ml sum F_k + (J_blades - NB Srl^2 / Ml) r'_s.
    J the hub's and the rigid blades' inertia about the shaft, with the spool's when the clutch is engaged."""
    Jl = U.NB * Srl ** 2 / Ml
    rds = r.fn('drive/r-dot-shaft', S(M(v(math.cos(U.SHAFT_TILT)), p('rotor/rds-in')), M(v(-math.sin(U.SHAFT_TILT)), p('rotor/pds-in'))),
               desc='airframe angular acceleration about the main rotor shaft (down +)')
    # (the spool's inertia by the clutch's state itself: drive/j-eng is the drive channel's, a step behind it when the
    # clutch locks, and the rotor's acceleration then had the hub's inertia alone, a hundredth of it)
    J = S(v(J_ROTOR - Jl), M(v(J_ENG), p('eng/engaged')))
    still = GT(S(p('drive/hold-rpm'), init), v(0.5))
    q_turn = r.fn('drive/q-turn-ftlb', S(p('drive/net-torque-ftlb'), M(v(-Srl / Ml), lag_forcing), M(v(J_BLADES - Jl), rds)),
                  desc='the torque that turns the rotor, less what its blades take')
    # standing still, the rotor brake and the transmission's breakaway friction hold it while the torque from outside
    # the rotor (the clutch or the engine, the air) is less than theirs (static friction: the blades' swing on their
    # hinges at rest reacts on the hub, and the smooth friction in drive/brake-ftlb holds nothing at zero speed, where
    # the speed's floor ratchets the swing into turning); then the hub is fixed for the blades' lag equations too. (Not
    # q_turn: with the blades resting on their lag stops their reaction takes up the clutch's torque in it.)
    hold = S(M(p('/controls/rotor/brake'), v(BRAKE_FTLB)), v(FRICTION_FTLB))
    stuck = r.fn('drive/stuck', M(LT(p('rotor/omega-rad_sec'), v(OMEGA_STUCK)), LT(ABS(p('drive/net-torque-ftlb')), hold)),
                 desc='1: the rotor held by friction')
    return r.fn('drive/omega-dot', IFTHEN(GT(S(still, stuck), v(0.5)), v(0), Q(q_turn, J)),
                desc='rotor speed acceleration relative to the airframe, rad/s2')
