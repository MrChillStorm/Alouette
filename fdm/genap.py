"""Test autopilot for the Alouette III, as a FlightGear XML autopilot (Systems/al3-autopilot.xml), after the UH-60A
model's fdm/genap.py. It holds wheel height, ground velocity and heading in the hover (mode 0); altitude, airspeed,
heading (by bank) and sideslip in forward flight (mode 1); and in autorotation (mode 2) the rotor speed with the
collective, as a pilot does, the airspeed, heading and sideslip as in mode 1. It flies like a pilot: its outputs are
the cyclic, pedal and collective controls (/controls/...). The Alouette has no stability augmentation, so the inner
loops damp the aircraft themselves, on its body rates. It flies the report's test flights offline (take-off, the
hover in and out of ground effect, the autorotation, the landing); in FlightGear it is not wired to the cockpit (the
original aircraft's own autopilot and Amelia are), but can be set through its properties.

Properties, /autopilot/al3/:
  engage            0/1, master
  mode              0 hover, 1 cruise, 2 autorotation
  axis-coll, axis-lon, axis-lat, axis-yaw   0/1: an axis that is off leaves its control where it is
                    (on the wheels the cyclic holds the attitude; ground velocity is held from lift-off)
  target/...        height-ft (wheels above ground), heading-deg, u-fps, v-fps (ground velocity, heading axes),
                    alt-ft, speed-kt (calibrated), beta-deg, nr-rpm (autorotation). While disengaged, or in another
                    mode, a target follows the aircraft, so engaging or changing mode holds what it is doing.
  gain/...          the loop gains, tunable in flight

The same component list runs offline against JSBSim (Sim), with the FlightGear controller semantics, to tune it.
python3 genap.py [out.xml]"""
import math, os, sys

AP = '/autopilot/al3/'
JSB = '/fdm/jsbsim/'
import al3data as U
WL_WHEELS_IN = U.model_to_station(*U.MAIN_WHEELS[0])[2]   # the main wheels' contact points, unloaded (WL, in): the wheel height
                                                          # is the CG's height less the CG's above them (its loading's)
FGROOT = AP


# ------------------------------------------------------------------ expressions (SGExpression subset)
class E:
    def __init__(s, op, *a):
        s.op, s.a = op, a

    def xml(s, ind):
        t = '  ' * ind
        if s.op == 'p':
            return '%s<property>%s</property>\n' % (t, s.a[0])
        if s.op == 'v':
            return '%s<value>%r</value>\n' % (t, s.a[0])
        return '%s<%s>\n%s%s</%s>\n' % (t, s.op, ''.join(x.xml(ind + 1) for x in s.a), t, s.op)

    def ev(s, get):
        o, a = s.op, s.a
        if o == 'p': return get(a[0])
        if o == 'v': return a[0]
        x = [y.ev(get) for y in a]
        if o == 'sum': return sum(x)
        if o == 'difference': return x[0] - sum(x[1:])
        if o == 'product':
            r = 1.0
            for y in x: r *= y
            return r
        if o == 'div': return x[0] / x[1]
        if o == 'min': return min(x)
        if o == 'max': return max(x)
        if o == 'sin': return math.sin(x[0])
        if o == 'cos': return math.cos(x[0])
        if o == 'mod': return math.fmod(x[0], x[1])
        raise ValueError(o)


def ex(x):
    if isinstance(x, E): return x
    if isinstance(x, (int, float)): return E('v', float(x))
    return E('p', x)


P = lambda name: E('p', name)
V = lambda c: E('v', float(c))
SUM = lambda *a: E('sum', *map(ex, a))
DIF = lambda *a: E('difference', *map(ex, a))
MUL = lambda *a: E('product', *map(ex, a))
MIN = lambda *a: E('min', *map(ex, a))
MAX = lambda *a: E('max', *map(ex, a))
SIN = lambda a: E('sin', ex(a))
COS = lambda a: E('cos', ex(a))
CLIP = lambda x, lo, hi: MAX(lo, MIN(hi, x))
WRAP = lambda x: DIF(E('mod', SUM(x, 540), V(360)), 180)     # -180..180 for x in -540..180+360
J = lambda name: P(JSB + name)
A = lambda name: P(AP + name)
G = lambda name: P(AP + 'gain/' + name)
D2R = math.pi / 180


def cond_on(*flags):
    """enable when every flag property (doubles 0/1 written by this autopilot) is set"""
    return [(AP + 'internal/' + f) for f in flags]


# ------------------------------------------------------------------ components (FlightGear semantics)
class Gain:
    """<filter><type>gain</type>: out = clip(input)"""
    def __init__(s, name, x, out, enable=(), lo=None, hi=None):
        s.name, s.x, s.out, s.en, s.lo, s.hi = name, ex(x), out, list(enable), lo, hi

    def step(s, get, dt, first):
        y = s.x.ev(get)
        if s.lo is not None: y = max(s.lo, y)
        if s.hi is not None: y = min(s.hi, y)
        return y

    def body(s):
        b = '    <type>gain</type>\n    <gain>1.0</gain>\n    <input>\n      <expression>\n%s      </expression>\n    </input>\n' % s.x.xml(4)
        if s.lo is not None: b += '    <u_min>%r</u_min>\n' % s.lo
        if s.hi is not None: b += '    <u_max>%r</u_max>\n' % s.hi
        return 'filter', b


class RateLimit:
    """<filter><type>noise-spike</type>, starting from its input (or its output's value) when enabled"""
    def __init__(s, name, x, out, rate, enable=(), init='input'):
        s.name, s.x, s.out, s.rate, s.en, s.init = name, ex(x), out, rate, list(enable), init

    def step(s, get, dt, first):
        x = s.x.ev(get)
        if first: s.y = x if s.init == 'input' else get(s.out)
        d = x - s.y; m = s.rate * dt
        s.y = x if abs(d) <= m else s.y + math.copysign(m, d)
        return s.y

    def body(s):
        return 'filter', ('    <type>noise-spike</type>\n    <initialize-to>%s</initialize-to>\n    <max-rate-of-change>%r</max-rate-of-change>\n'
                          '    <input>\n      <expression>\n%s      </expression>\n    </input>\n' % (s.init, s.rate, s.x.xml(4)))


class Lag:
    """<filter><type>exponential</type>: y += (x - y) / (tau/dt + 1)"""
    def __init__(s, name, x, out, tau, enable=()):
        s.name, s.x, s.out, s.tau, s.en = name, ex(x), out, tau, list(enable)

    def step(s, get, dt, first):
        x = s.x.ev(get)
        if first: s.y = x
        tau = get(AP + 'gain/' + s.tau) if isinstance(s.tau, str) else s.tau
        s.y += (x - s.y) / (tau / dt + 1)
        return s.y

    def body(s):
        tau = AP + 'gain/' + s.tau if isinstance(s.tau, str) else repr(s.tau)
        return 'filter', ('    <type>exponential</type>\n    <initialize-to>input</initialize-to>\n    <filter-time>%s</filter-time>\n'
                          '    <input>\n      <expression>\n%s      </expression>\n    </input>\n' % (tau, s.x.xml(4)))


class Delta:
    """update interval: the input minus its 2-sample moving average, doubled (diagnostic)"""
    def __init__(s, name, out):
        s.name, s.out, s.en = name, out, []

    def step(s, get, dt, first):
        return dt

    def components(s):
        avg = AP + 'internal/time-avg'
        return [('filter', 'update interval: average', '    <type>moving-average</type>\n    <samples>2</samples>\n'
                 '    <input>%ssimulation/sim-time-sec</input>\n' % JSB, avg, []),
                ('filter', s.name, '    <type>gain</type>\n    <gain>2.0</gain>\n    <input>\n      <expression>\n%s      </expression>\n    </input>\n'
                 % DIF(J('simulation/sim-time-sec'), P(avg)).xml(4), s.out, [])]


class PID:
    """<pid-controller>, FlightGear's velocity form (P and I only): from its output's value when enabled,
    u += Kp ((e - e_prev) + dt/Ti e), e = reference - input, and an increment past u_min/u_max is cut short.
    The first step after enabling has e_prev = 0."""
    def __init__(s, name, ref, inp, out, kp, ti, lo, hi, enable=()):
        s.name, s.ref, s.inp, s.out, s.kp, s.ti, s.lo, s.hi, s.en = name, ex(ref), ex(inp), out, kp, ti, lo, hi, list(enable)

    def step(s, get, dt, first):
        if first:
            s.e1 = 0.0; s.u = get(s.out)
        e = s.ref.ev(get) - s.inp.ev(get)
        kp, ti = get(AP + 'gain/' + s.kp), get(AP + 'gain/' + s.ti)
        du = kp * ((e - s.e1) + dt / ti * e)
        du = min(max(du, s.lo - s.u), s.hi - s.u)
        s.u += du; s.e1 = e
        return s.u

    def body(s):
        return 'pid-controller', ('    <input>\n      <expression>\n%s      </expression>\n    </input>\n'
                                  '    <reference>\n      <expression>\n%s      </expression>\n    </reference>\n'
                                  '    <config>\n      <Kp>%sgain/%s</Kp>\n      <beta>1.0</beta>\n      <alpha>0.1</alpha>\n      <gamma>0.0</gamma>\n'
                                  '      <Ti>%sgain/%s</Ti>\n      <Td>0.0</Td>\n      <u_min>%r</u_min>\n      <u_max>%r</u_max>\n    </config>\n'
                                  % (s.inp.xml(4), s.ref.xml(4), AP, s.kp, AP, s.ti, s.lo, s.hi))


# ------------------------------------------------------------------ the autopilot
GAINS = dict(
    h_k=0.5,            # 1/s: vertical speed per ft of height error (hover)
    alt_k=0.2,          # 1/s: (cruise)
    coll_kp=0.02,       # throttle (1 = collective up) per ft/s of vertical speed error
    coll_ti=2.0,
    nr_kp=-0.004, nr_ti=3.0,   # collective per rpm of rotor speed error (autorotation): up slows the rotor
    u_kp=-0.3, u_ti=14.0,      # pitch deg per ft/s of forward ground velocity error
    v_kp=0.6, v_ti=25.0,       # roll deg per ft/s of lateral ground velocity error
    spd_kp=-0.8, spd_ti=15.0,  # pitch deg per kt of airspeed error
    hdg_kp=1.5, hdg_ti=10.0,   # roll deg per deg of heading error at 100 kt (cruise)
    # The inner loops: the Alouette has no augmentation, and its pitch damping is low (Mq -1.6/s in the hover against
    # Lp -6.3/s: derivatives.py), so the pitch loop leads with the pitch rate (th_tq) more than the roll loop
    # (crossovers about 2 rad/s in pitch, 3 in roll: Mlon 2.7 and Llat 7 rad/s2 per unit stick in the hover; the UH-60's
    # gains, 0.012 and 0.008, set a 6 s roll oscillation of 7 deg in the hover going on top of the Alouette's own)
    th_kp=-0.02, th_ti=1.0, th_tq=0.6,    # elevator (+ forward) per deg of pitch error; error = cmd - (theta + tq q)
    ph_kp=0.04, ph_ti=1.0, ph_tp=0.15,    # aileron (+ right) per deg of roll error
    psi_k=0.5,                            # yaw rate deg/s per deg of heading error (hover)
    r_kp=0.015, r_ti=3.0, r_tau=0.3,      # rudder (+ right) per deg/s of yaw rate error (hover); yaw rate lag, s
    # on the wheels, per unit collective: the hover trims' pedal and lateral cyclic over their collective (sweep.py:
    # 0.49-0.62 collective, pedal 0.00-0.17 right, lateral cyclic 0.08 left), as a pilot's feet and hand follow the lever
    # and the longitudinal: the Alouette hovers with the stick aft (its CG forward of the mast: -0.38 at 2200 kg, -0.22 at
    # 1500 kg), so that it lifts off level and does not roll forward on its wheels as the thrust comes up
    ped_coll=0.15, lat_coll=-0.15, lon_coll=-0.5,
    th_lift=1.0, ph_lift=3.8,             # deg, the hover attitude (right side down: the tail rotor's push), at lift-off
    light_frac=0.2,                       # light on the wheels below this fraction of the weight on them (0.35, 0.5 overshot
                                          # the hover bank by 7 and 12 deg at lift-off)
    thc_kp=-0.03, thc_ti=2.0, thc_tq=0.3, # cruise
    phc_kp=0.015, phc_ti=2.0, phc_tp=0.3,
    b_kp=-0.02, b_ti=3.0,                 # rudder per deg of sideslip error (cruise)
)
LIMITS = dict(coll_up=0.08, coll_down=0.08, vs_accel=3.0, ground_rate=0.15, climb=5.0, descent=4.0, cruise_vs=15.0, accel=3.0, height_rate=2.0, speed_rate=2.0, turn=10.0,
              theta_hover=(-12.0, 12.0), phi_hover=(-15.0, 15.0), theta_cruise=(-15.0, 10.0), phi_cruise=(-25.0, 25.0))
CTL = dict(elev='/controls/flight/elevator', ail='/controls/flight/aileron', rud='/controls/flight/rudder',
           thr='/controls/al3/collective')
RPM = 30 / math.pi
DEFAULTS = {AP + 'engage': 0.0, AP + 'mode': 0.0, AP + 'axis-coll': 1.0, AP + 'axis-lon': 1.0, AP + 'axis-lat': 1.0,
            AP + 'axis-yaw': 1.0, AP + 'target/beta-deg': 0.0, AP + 'target/nr-rpm': 353.2}


def components():
    L = LIMITS
    I = lambda n: AP + 'internal/' + n
    T = lambda n: AP + 'target/' + n
    psi = J('attitude/psi-deg')
    psir = MUL(psi, D2R)
    vn, ve = J('velocities/v-north-fps'), J('velocities/v-east-fps')
    c = []
    # flags (doubles, for the enable conditions)
    eng = A('engage')
    hov, cru = DIF(1, MIN(1, MAX(0, A('mode')))), MIN(1, MAX(0, A('mode')))
    aut = MIN(1, MAX(0, DIF(A('mode'), 1)))          # autorotation: the cyclic and pedals as cruise, the collective on rotor speed
    # airborne: the outer loops (ground velocity, airspeed, heading by bank) start at lift-off; on the wheels the
    # aircraft rolls as the collective comes up, and those loops would wind up the attitude. Airborne from 1 ft wheel
    # height until both main wheels are down again (the nose wheel may touch first or last).
    c.append(Gain('wheel height', DIF(J('position/h-agl-ft'), MUL(DIF(J('inertia/cg-z-in'), WL_WHEELS_IN), 1 / 12.0)), I('h-wheel-ft')))
    # touchdown: landing (hover mode, height target below the ground) and any wheel on the ground. From then on (until a new height target) the flying loops are off, the collective goes down at a
    # steady rate and cyclic and pedals go to their on-the-wheels positions, as a pilot lands.
    contact = MAX(*[J('gear/unit[%d]/WOW' % i) for i in range(3)])
    landing = MUL(MIN(1, MAX(0, MUL(DIF(0.5, P(AP + 'target/height-ft')), 1000))), hov)
    c.append(Gain('flag: touchdown', MAX(MUL(eng, landing, contact), MUL(P(I('touchdown')), landing, eng)), I('touchdown')))
    td = P(I('touchdown'))
    up = MUL(MIN(1, MAX(0, MUL(DIF(P(I('h-wheel-ft')), 1.0), 1000))), DIF(1, contact))
    c.append(Gain('flag: airborne', MUL(MAX(up, MUL(P(I('air')), DIF(1, MUL(J('gear/unit[0]/WOW'), J('gear/unit[1]/WOW'))))), DIF(1, td)), I('air')))
    air = P(I('air'))
    # light on the wheels (they carry less than light_frac of the weight, or none): the attitude loops fly from here, to the
    # hover attitude, as a pilot brings in the cyclic the hover needs before the wheels leave (the Alouette's tail rotor
    # pushes left, its thrust leans right against it in the hover: lifted level with the hover's cyclic it rolls over its
    # left wheel within a foot); the outer loops wait for lift-off
    c.append(Gain('flag: light', MIN(1, MAX(0, MUL(SUM(MUL(G('light_frac'), J('inertia/weight-lbs')), J('forces/fbz-gear-lbs')), 1000))), I('light')))
    lite = MAX(air, P(I('light')))
    c.append(Gain('flag: off', DIF(1, eng), I('off')))
    for ax in ('coll', 'lon', 'lat', 'yaw'):
        on = MUL(eng, A('axis-' + ax))
        out = MUL(on, air) if ax in ('lon', 'lat') else (MUL(on, lite) if ax == 'yaw' else on)
        c.append(Gain('flag: %s' % ax, on, I(ax)))
        if ax in ('lon', 'lat'):     # on the wheels the stick is held (lateral: a schedule, below)
            c.append(Gain('flag: %s inner loop, hover' % ax, MUL(on, hov, lite), I(ax + '-in-hover')))
        if ax in ('lon', 'lat'):
            c.append(Gain('flag: %s inner loop, cruise' % ax, MUL(on, cru), I(ax + '-in-cruise')))
        c.append(Gain('flag: %s outer loop off' % ax, DIF(1, out), I(ax + '-off')))
        c.append(Gain('flag: %s hover' % ax, MUL(out, hov), I(ax + '-hover')))
        c.append(Gain('flag: %s cruise' % ax, MUL(out, cru, DIF(1, aut)) if ax == 'coll' else MUL(out, cru), I(ax + '-cruise')))
    c.append(Gain('flag: coll autorotation', MUL(eng, A('axis-coll'), aut), I('coll-autorotation')))
    c.append(Gain('flag: hover targets follow', MAX(DIF(1, eng), cru), I('track-hover')))
    c.append(Gain('flag: cruise targets follow', MAX(DIF(1, eng), hov), I('track-cruise')))
    # measurements
    c.append(Gain('ground velocity, forward', SUM(MUL(vn, COS(psir)), MUL(ve, SIN(psir))), I('u-fps')))
    c.append(Gain('ground velocity, right', DIF(MUL(ve, COS(psir)), MUL(vn, SIN(psir))), I('v-fps')))
    for k, n in (('p', 'p'), ('q', 'q'), ('r', 'r')):
        c.append(Lag('%s filter' % k, MUL(J('velocities/%s-rad_sec' % n), 180 / math.pi), I('%s-dps' % k), 0.03))
    # targets follow the aircraft while not in use
    c.append(Gain('target: heading', psi, T('heading-deg'), cond_on('off')))
    c.append(Gain('target: height', P(I('h-wheel-ft')), T('height-ft'), cond_on('track-hover')))
    c.append(Gain('target: u', P(I('u-fps')), T('u-fps'), cond_on('track-hover')))
    c.append(Gain('target: v', P(I('v-fps')), T('v-fps'), cond_on('track-hover')))
    c.append(Gain('target: altitude', J('position/h-sl-ft'), T('alt-ft'), cond_on('track-cruise')))
    c.append(Gain('target: airspeed', J('velocities/vc-kts'), T('speed-kt'), cond_on('track-cruise')))
    c.append(Gain('flag: rotor speed target follows', DIF(1, MUL(eng, aut)), I('track-nr')))
    c.append(Gain('target: rotor speed', MUL(J('rotor/omega-rad_sec'), RPM), T('nr-rpm'), cond_on('track-nr')))
    # attitude commands follow the aircraft while their outer loop is off (axis off); on the wheels they are the
    # hover attitude, for the lift-off
    c.append(Gain('pitch command follows', SUM(MUL(J('attitude/theta-deg'), air), MUL(G('th_lift'), DIF(1, air))), I('theta-cmd'), cond_on('lon-off')))
    c.append(Gain('roll command follows', SUM(MUL(J('attitude/phi-deg'), air), MUL(G('ph_lift'), DIF(1, air))), I('phi-cmd'), cond_on('lat-off')))
    # references: the targets, rate limited
    c.append(RateLimit('ref: height', P(T('height-ft')), I('ref-height'), L['height_rate'], cond_on('coll-hover')))
    c.append(RateLimit('ref: altitude', P(T('alt-ft')), I('ref-alt'), L['cruise_vs'], cond_on('coll-cruise')))
    c.append(RateLimit('ref: u', P(T('u-fps')), I('ref-u'), L['accel'], cond_on('lon-hover')))
    c.append(RateLimit('ref: v', P(T('v-fps')), I('ref-v'), L['accel'], cond_on('lat-hover')))
    c.append(RateLimit('ref: airspeed', P(T('speed-kt')), I('ref-speed'), L['speed_rate'], cond_on('lon-cruise')))
    # collective: height (hover) or altitude (cruise) -> vertical speed -> collective
    c.append(Gain('vertical speed command, hover', MUL(G('h_k'), DIF(P(I('ref-height')), P(I('h-wheel-ft')))), I('vs-cmd'),
                  cond_on('coll-hover'), -L['descent'], L['climb']))
    c.append(Gain('vertical speed command, cruise', MUL(G('alt_k'), DIF(P(I('ref-alt')), J('position/h-sl-ft'))), I('vs-cmd'),
                  cond_on('coll-cruise'), -L['cruise_vs'], L['cruise_vs']))
    lifting = MUL(eng, A('axis-coll'), hov, DIF(1, lite), DIF(1, td), MIN(1, MAX(0, MUL(DIF(P(T('height-ft')), 0.5), 1000))))
    c.append(Gain('flag: collective lifting', lifting, I('coll-lift')))
    c.append(Gain('flag: collective flying', MUL(eng, A('axis-coll'), DIF(1, td), DIF(1, aut), DIF(1, lifting)), I('coll-fly')))
    # on the wheels with a height to go to the lever comes up steadily, as a pilot raises it, until the aircraft is light
    c.append(RateLimit('collective up for the lift-off', V(0.9), CTL['thr'], L['coll_up'], cond_on('coll-lift'), init='output'))
    c.append(Gain('flag: collective lowering', MUL(eng, A('axis-coll'), td), I('coll-down')))
    # the vertical speed reference: the command, rate limited, from the aircraft's own vertical speed when the loop takes
    # over (out of an autorotation the command is a climb, 40 ft/s from the descent: the lever would jump, sag the rotor
    # and yaw and roll the aircraft with its torque; a pilot raises it over a few seconds)
    c.append(Gain('flag: collective not flying', DIF(1, P(I('coll-fly'))), I('coll-idle')))
    c.append(Gain('vertical speed reference follows', J('velocities/h-dot-fps'), I('vs-ref'), cond_on('coll-idle')))
    c.append(RateLimit('vertical speed reference', P(I('vs-cmd')), I('vs-ref'), L['vs_accel'], cond_on('coll-fly'), init='output'))
    c.append(PID('collective', P(I('vs-ref')), J('velocities/h-dot-fps'), CTL['thr'], 'coll_kp', 'coll_ti', 0.0, 0.9, cond_on('coll-fly')))
    c.append(RateLimit('collective down after touchdown', V(0.0), CTL['thr'], L['coll_down'], cond_on('coll-down'), init='output'))
    # autorotation: the collective keeps the rotor speed (lowered as the power goes, raised to hold it in the flare)
    c.append(PID('collective, autorotation', P(T('nr-rpm')), MUL(J('rotor/omega-rad_sec'), RPM), CTL['thr'], 'nr_kp', 'nr_ti', 0.0, 1.0,
                 cond_on('coll-autorotation')))
    # pitch: ground velocity (hover) or airspeed (cruise) -> pitch attitude -> longitudinal cyclic
    c.append(PID('pitch command, hover', P(I('ref-u')), P(I('u-fps')), I('theta-cmd'), 'u_kp', 'u_ti', *L['theta_hover'], cond_on('lon-hover')))
    c.append(PID('pitch command, cruise', P(I('ref-speed')), J('velocities/vc-kts'), I('theta-cmd'), 'spd_kp', 'spd_ti', *L['theta_cruise'],
                 cond_on('lon-cruise')))
    for m, g in (('hover', 'th'), ('cruise', 'thc')):
        c.append(PID('longitudinal cyclic, %s' % m, P(I('theta-cmd')), SUM(J('attitude/theta-deg'), MUL(G(g + '_tq'), P(I('q-dps')))), CTL['elev'],
                     g + '_kp', g + '_ti', -1.0, 1.0, cond_on('lon-in-' + m)))
    # roll: lateral ground velocity (hover) or heading (cruise) -> bank -> lateral cyclic
    c.append(PID('roll command, hover', P(I('ref-v')), P(I('v-fps')), I('phi-cmd'), 'v_kp', 'v_ti', *L['phi_hover'], cond_on('lat-hover')))
    # bank for a heading error: the turn rate a bank gives falls with speed, so the gain rises with it (hdg_kp at 100 kt)
    c.append(PID('roll command, cruise', MUL(WRAP(DIF(P(T('heading-deg')), psi)), MAX(30, J('velocities/vc-kts')), 0.01), V(0), I('phi-cmd'),
                 'hdg_kp', 'hdg_ti', *L['phi_cruise'], cond_on('lat-cruise')))
    for m, g in (('hover', 'ph'), ('cruise', 'phc')):
        c.append(PID('lateral cyclic, %s' % m, P(I('phi-cmd')), SUM(J('attitude/phi-deg'), MUL(G(g + '_tp'), P(I('p-dps')))), CTL['ail'],
                     g + '_kp', g + '_ti', -1.0, 1.0, cond_on('lat-in-' + m)))
    # yaw: heading -> yaw rate -> pedals (hover); sideslip -> pedals (cruise)
    c.append(Gain('yaw rate command', MUL(G('psi_k'), WRAP(DIF(P(T('heading-deg')), psi))), I('r-cmd'), cond_on('yaw-hover'),
                  -L['turn'], L['turn']))
    c.append(Lag('yaw rate for the heading hold', P(I('r-dps')), I('r-slow-dps'), 'r_tau'))
    c.append(PID('pedals, hover', P(I('r-cmd')), P(I('r-slow-dps')), CTL['rud'], 'r_kp', 'r_ti', -1.0, 1.0, cond_on('yaw-hover')))
    # on the wheels the brakes and the tyres hold the heading and the heading hold has nothing to work on: the pedals
    # follow the collective as a pilot's would, to about the hover pedal at lift-off, where the heading hold takes over
    c.append(Gain('flag: pedals on the wheels', MUL(eng, A('axis-yaw'), DIF(1, lite)), I('yaw-ground')))
    c.append(RateLimit('pedals on the wheels', MUL(G('ped_coll'), P(CTL['thr'])), CTL['rud'], L['ground_rate'], cond_on('yaw-ground'), init='output'))
    # likewise the lateral cyclic, to the hover's
    c.append(Gain('flag: lateral cyclic on the wheels', MUL(eng, A('axis-lat'), DIF(1, lite)), I('lat-ground')))
    c.append(RateLimit('lateral cyclic on the wheels', MUL(G('lat_coll'), P(CTL['thr'])), CTL['ail'], L['ground_rate'], cond_on('lat-ground'), init='output'))
    c.append(Gain('flag: longitudinal cyclic on the wheels', MUL(eng, A('axis-lon'), DIF(1, lite)), I('lon-ground')))
    c.append(RateLimit('longitudinal cyclic on the wheels', MUL(G('lon_coll'), P(CTL['thr'])), CTL['elev'], L['ground_rate'], cond_on('lon-ground'),
                       init='output'))
    c.append(PID('pedals, cruise', P(T('beta-deg')), J('aero/beta-deg'), CTL['rud'], 'b_kp', 'b_ti', -1.0, 1.0, cond_on('yaw-cruise')))
    c.append(Delta('update interval', I('update-s')))
    return c


def xml():
    out = ['<?xml version="1.0"?>\n<!-- Generated by fdm/genap.py: edit that, not this. -->\n<PropertyList>\n']
    for comp in components():
        if hasattr(comp, 'components'):
            parts = comp.components()
        else:
            tag, body = comp.body()
            parts = [(tag, comp.name, body, comp.out, comp.en)]
        for tag, name, body, out_prop, en in parts:
            s = '  <%s>\n    <name>%s</name>\n' % (tag, name)
            if en:
                conds = ''.join('          <greater-than><property>%s</property><value>0.5</value></greater-than>\n' % f for f in en)
                s += '    <enable>\n      <condition>\n        <and>\n%s        </and>\n      </condition>\n    </enable>\n' % conds
            s += body + '    <output>%s</output>\n  </%s>\n' % (out_prop, tag)
            out.append(s)
    out.append('</PropertyList>\n')
    return ''.join(out)


def defaults_xml():
    """the properties' starting values, for the -set file (type double, as the enable conditions compare numbers)"""
    lines = ['<?xml version="1.0"?>\n<!-- Generated by fdm/genap.py. Starting values of /autopilot/al3. -->\n<PropertyList>\n']
    tree = {}
    for k, val in list(DEFAULTS.items()) + [(AP + 'gain/' + g, x) for g, x in GAINS.items()]:
        node = tree
        parts = k[len(AP):].split('/')
        for part in parts[:-1]:
            node = node.setdefault(part, {})
        node[parts[-1]] = val
    def emit(node, ind):
        s = ''
        for k, val in node.items():
            if isinstance(val, dict):
                s += '%s<%s>\n%s%s</%s>\n' % ('  ' * ind, k, emit(val, ind + 1), '  ' * ind, k)
            else:
                s += '%s<%s type="double">%r</%s>\n' % ('  ' * ind, k, val, k)
        return s
    lines.append(emit(tree, 1))
    lines.append('</PropertyList>\n')
    return ''.join(lines)


# ------------------------------------------------------------------ offline: the components against JSBSim
class Sim:
    """FlightGear's controls and /fdm/jsbsim on a JSBSim FGFDMExec; the autopilot runs every n_ap FDM steps"""
    def __init__(s, f, n_ap=1):
        s.f, s.n_ap, s.k = f, n_ap, 0
        s.props = dict(DEFAULTS)
        s.props.update({AP + 'gain/' + g: x for g, x in GAINS.items()})
        s.comps = components()
        s.was = [False] * len(s.comps)

    def get(s, name):
        f = s.f
        if name.startswith(JSB): return f[name[len(JSB):]]
        if name == CTL['elev']: return f['fcs/elevator-cmd-norm']
        if name == CTL['ail']: return f['fcs/aileron-cmd-norm']
        if name == CTL['rud']: return -f['fcs/rudder-cmd-norm']
        if name == CTL['thr']: return f[CTL['thr']]
        return s.props.get(name, 0.0)

    def set(s, name, x):
        f = s.f
        if name.startswith(JSB): f[name[len(JSB):]] = x
        elif name == CTL['elev']: f['fcs/elevator-cmd-norm'] = x
        elif name == CTL['ail']: f['fcs/aileron-cmd-norm'] = x
        elif name == CTL['rud']: f['fcs/rudder-cmd-norm'] = -x
        elif name == CTL['thr']: f[CTL['thr']] = x
        else: s.props[name] = x

    def __getitem__(s, k): return s.get(k if k.startswith('/') else AP + k)
    def __setitem__(s, k, x): s.set(k if k.startswith('/') else AP + k, x)

    def step(s):
        if s.k % s.n_ap == 0:
            dt = s.n_ap * s.f['simulation/dt']
            for i, comp in enumerate(s.comps):
                on = all(s.get(e) > 0.5 for e in comp.en)
                if on:
                    s.set(comp.out, comp.step(s.get, dt, not s.was[i]))
                s.was[i] = on
        s.k += 1
        s.f.run()


if __name__ == '__main__':
    here = os.path.dirname(os.path.abspath(__file__))
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, '..', 'Alouette-3', 'Systems', 'al3-autopilot.xml')
    with open(out, 'w') as fh:
        fh.write(xml())
    with open(os.path.join(os.path.dirname(out), 'al3-autopilot-props.xml'), 'w') as fh:
        fh.write(defaults_xml())
    print('wrote', out, len(components()), 'components')
