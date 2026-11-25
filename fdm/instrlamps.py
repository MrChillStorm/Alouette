#!/usr/bin/env python3
"""The instruments' lighting for FlightGear's HDR pipeline, which leaves out the material animation's glow the
instruments had (Instruments/Instrument-Light*.xml): a ring of small lamps round each dial lighting its face, as the
UH-60 model's lighted bezels (its fdm/cockpit/instruments.py, bezel_lamps and lamp_xml, used here as they are).

Each instrument gets two rings of spot lights, aimed inward across its face from just in front of it: the NVIS ones
(night vision goggle compatible, NVIS Green A as the UH-60's) and the standard red ones. The intensity of each is a
property of the lighting system (Systems/electrical-lights.xml: the dimmer knob's level, the sun's height, the
Configuration dialog's light level, 0 for the ring not in use), and a select animation takes the ring out of the
scene when its level is 0: lights off draw no lamps.

The light level is the mean illuminance of the face at full brightness, LIGHT_LUX; the pipeline has no photometric
units, its unit of a light's intensity being 635 cd (see the UH-60 model's HDR_CD).

    python3 instrlamps.py ../Alouette-3

writes the lamps between the markers in the instruments' XML (Alouette-3/Instruments/*.xml).
"""
import math, os, re, sys
import numpy as np
import acread

NVIS_GREEN = (0.10, 1.00, 0.45)        # NVIS Green A (MIL-L-85762A: u' 0.088, v' 0.543), sRGB about (0, 255, 110)
STD_RED = (1.00, 0.10, 0.03)           # the standard instrument lights
RED_SCALE = 1.7                        # their intensity relative to the NVIS ones: red on the screen is brighter than the same candela of green
LIGHT_LUX = 20.0                       # the mean illuminance of an instrument's face at full brightness, lux: the UH-60 model's 6 lux, raised for the HDR renderer's exposure in a dark cockpit
HDR_CD = 635.0                         # the HDR pipeline's unit of a light's intensity, cd
LEVEL = '/fdm/jsbsim/electrical/lighting/instruments-'    # + std or nvis: Systems/electrical-lights.xml
BEGIN = '<!-- lamps: begin (written by fdm/instrlamps.py) -->'
END = '<!-- lamps: end -->'

# the instruments that have lamps: the mesh's object that is the dial's face (a ring of lamps round it)
FACE = {'Fuel': 'Cadran', 'ALT-AL_III': 'Cadran', 'ASI-S51': 'Cadran', 'IVSI-Nord': 'Cadran', 'DME': 'Cadran',
        'HI-al3': 'Cadran', 'N1-al3': 'Cadran', 'Rotor-rpm-Al_III': 'Cadran', 'Rotor-Torque-Al_III': 'Cadran',
        'Turbine_Rotor-rpm-Al_III': 'Cadran', 'Tuyere-temp': 'Cadran', 'Voltage': 'Cadran',
        'Altitude-radio-Al3': 'Cadran', 'Altitude-radio-Puma': 'Cadran', 'GSDI-Al_III': 'Cadran',
        'Attitude': 'Dial', 'Afficheur': 'Cadran'}


def bezel_lamps(face, half, round=False, glass=None, out=None):
    """The UH-60 model's: the lamps of a lighted bezel: spot lights round the face, a quarter of its half size (the
    smaller one, m) above it but behind its glass (at depth `glass`) and 0.15 of it out from its edge, 0.9 apart
    (eight round a dial), each aimed inward across the face at the nearest point of its centre line (a round face's
    centre) 0.1 above it. Their exponent of 3 holds back the lamp's light on the face near it, so the face is lit about
    evenly, a little brighter toward the edge as a lighted bezel is; the cone takes in the face, and nothing outside
    the bezel: lights cast no shadows in FlightGear. face: (d, y, z) of the face's centre; half: its half width and
    height, or with `round` its radius twice. Returns [(position, aim point)], the exponent, cutoff (deg), range (m),
    the intensity (cd) that gives the face a mean illuminance of 1 lux, and the face's evenness (its illuminance's
    98th percentile over its 2nd)."""
    d, y, z = face; hw, hh = half
    m = min(hw, hh)
    h, off, up, gap, e = 0.25 * m, 0.15 * m, 0.10 * m, 0.9 * m, 3.0
    if glass is not None:
        h = min(h, glass - 0.0015 - d)
    if out is not None:
        off = out
    if round:
        R = max(hw, hh)
        path = [((R + off) * math.cos(t), (R + off) * math.sin(t)) for t in np.linspace(0, 2 * math.pi, 721)[:-1]]
    else:
        path = []
        for cy, cz, a0 in ((hw, hh, 0), (-hw, hh, 90), (-hw, -hh, 180), (hw, -hh, 270)):
            for a in np.radians(np.arange(a0, a0 + 90, 1.0)):
                path.append((cy + off * math.cos(a), cz + off * math.sin(a)))
        dense = []
        for (y0, z0), (y1, z1) in zip(path, path[1:] + path[:1]):
            k = max(1, int(math.hypot(y1 - y0, z1 - z0) / 0.0002))
            dense += [(y0 + (y1 - y0) * t / k, z0 + (z1 - z0) * t / k) for t in range(k)]
        path = dense
    P = np.array(path); seg = np.hypot(*(np.roll(P, -1, 0) - P).T); s = np.concatenate([[0], np.cumsum(seg)[:-1]])
    n = max(4, int(math.floor(s[-1] / gap + 0.5)))
    if round:
        n = 8
    lamps = []
    for k in range(n):
        q = P[np.searchsorted(s, (k + 0.5) * seg.sum() / n) - 1]
        ay = float(np.clip(q[0], -(hw - m), hw - m)); az = float(np.clip(q[1], -(hh - m), hh - m))
        lamps.append(((d + h, y + q[0], z + q[1]), (d + up, y + ay, z + az)))
    g = np.linspace(-0.97, 0.97, 41); GY, GZ = np.meshgrid(g * hw, g * hh)
    inside = (GY / hw) ** 2 + (GZ / hh) ** 2 <= 0.97 ** 2 if round else np.ones_like(GY, bool)
    F = np.stack([np.full(inside.sum(), d), y + GY[inside], z + GZ[inside]], 1)
    far = max(np.linalg.norm(F - np.array(p), axis=1).max() for p, a in lamps); rng = 1.16 * far
    cut, E = 0.0, np.zeros(len(F))
    for p, a in lamps:
        v = F - np.array(p); dist = np.linalg.norm(v, axis=1); u = v / dist[:, None]
        axis = np.array(a) - np.array(p); axis /= np.linalg.norm(axis)
        cd = np.clip(u @ axis, 1e-6, 1)
        cut = max(cut, math.degrees(math.acos(cd.min())))
        win = np.clip(1 - (dist / rng) ** 4, 0, None) ** 2
        E += cd ** e * (-u[:, 0]) * win / dist ** 2
    q2, q98 = np.percentile(E, [2, 98])
    return lamps, e, min(89.0, cut + 2), rng, 1 / E.mean(), q98 / q2


def lamp_xml(name, p, a, color, level, cd, e, cut, rng):
    """A spot light at p aimed at a, of the sRGB `color`, its intensity cd (at full brightness) times the property `level`."""
    return ('\n  <light>\n    <name>%s</name>\n    <type>spot</type>\n    <position>\n      <x-m> %.4f </x-m>\n'
            '      <y-m> %.4f </y-m>\n      <z-m> %.4f </z-m>\n    </position>\n    <direction>\n'
            '      <lookat-x-m> %.4f </lookat-x-m>\n      <lookat-y-m> %.4f </lookat-y-m>\n      <lookat-z-m> %.4f </lookat-z-m>\n'
            '    </direction>\n    <color>\n      <r> %.2f </r>\n      <g> %.2f </g>\n      <b> %.2f </b>\n    </color>\n'
            '    <intensity>\n      <property>%s</property>\n      <scale> %.6g </scale>\n    </intensity>\n'
            '    <spot-exponent> %g </spot-exponent>\n    <spot-cutoff> %.1f </spot-cutoff>\n    <range-m> %.3f </range-m>\n'
            '  </light>\n' % ((name,) + tuple(p) + tuple(a) + tuple(color) + (level, cd / HDR_CD, e, cut, rng)))


def select_xml(names, level):
    """The lamps `names` in the scene only while the property `level` is above 0."""
    return ('\n  <animation>\n    <type>select</type>\n%s    <condition>\n      <greater-than>\n        <property>%s</property>\n'
            '        <value>0</value>\n      </greater-than>\n    </condition>\n  </animation>\n'
            % (''.join('    <object-name>%s</object-name>\n' % n for n in names), level))


def face_of(ac, obj):
    """The face's centre (d, y, z) and radius in the instrument's model frame (x out of the face, y right, z up)."""
    root = acread._objects(open(ac, encoding='latin-1').read().splitlines())
    pts = []
    def walk(o, base):
        loc = base + o['loc']
        if o['name'] == obj and len(o['verts']):
            pts.append(o['verts'] + loc)
        for k in o['kids']:
            walk(k, loc)
    walk(root, np.zeros(3))
    v = np.concatenate(pts)
    m = np.stack([v[:, 0], -v[:, 2], v[:, 1]], 1)            # AC3D (x, y, z) is the model's (x, z, -y)
    lo, hi = m.min(0), m.max(0)
    return (float(np.median(m[:, 0])), float((lo[1] + hi[1]) / 2), float((lo[2] + hi[2]) / 2)), float(max(hi[1] - lo[1], hi[2] - lo[2]) / 2)


def lamps_xml(face, radius):
    lamps, e, cut, rng, cd1, even = bezel_lamps(face, (radius, radius), round=True)
    out, names = '', {}
    for tag, color, scale in (('nvis', NVIS_GREEN, 1.0), ('std', STD_RED, RED_SCALE)):
        names[tag] = ['lamp-%s-%d' % (tag, k) for k in range(len(lamps))]
        for nm, (p, a) in zip(names[tag], lamps):
            out += lamp_xml(nm, p, a, color, LEVEL + tag, LIGHT_LUX * cd1 * scale, e, cut, rng)
        out += select_xml(names[tag], LEVEL + tag)
    return out, len(lamps), even


def write(xml, text):
    block = '  %s\n%s\n  %s\n' % (BEGIN, text, END)
    if BEGIN in open(xml, encoding='latin-1').read():
        t = open(xml, encoding='latin-1').read()
        t = re.sub(r'[ \t]*' + re.escape(BEGIN) + r'.*?' + re.escape(END) + r'\n', lambda m: block, t, flags=re.S)
    else:
        t = open(xml, encoding='latin-1').read()
        i = t.rindex('</PropertyList>')
        t = t[:i] + '\n' + block + '\n' + t[i:]
    open(xml, 'w', encoding='latin-1').write(t)


def main(root):
    instr = os.path.join(root, 'Instruments')
    for name, obj in sorted(FACE.items()):
        xml = os.path.join(instr, name + '.xml')
        mesh = re.search(r'<path>(Models/[^<]+)</path>', open(xml, encoding='latin-1').read()).group(1)
        face, radius = face_of(os.path.join(instr, mesh), obj)
        text, n, even = lamps_xml(face, radius)
        write(xml, text)
        print('%-26s face x %.4f y %+.4f z %+.4f  R %.4f  %d lamps x 2, evenness %.1f' % ((name,) + face + (radius, n, even)))


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else '../Alouette-3')
