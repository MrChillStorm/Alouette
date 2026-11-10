#!/usr/bin/env python3
"""The Alouette III's rotors for FlightGear's HDR pipeline (from 2024.2 its only one), driven blade by blade by the
flight model, with a motion blur of the blades' own geometry: the Bo 105 model's fdm/rotorvis.py, the UH-60's method.

The 3-D model's blades faded out, and their blur discs in, by blend animations (the material's transparency), which
the HDR pipeline leaves out: there the blades never faded and the discs never showed. And the blades bent upward by
five times the flap angle, the old flight model's stand-in for coning. Here:

- the main rotor's three blade instances (Models/Rotors/AL3-Rotor-Pale<k>_rb.xml, on the 3-D model's own blade,
  Models/Rotors/Models/al3-rotor-pale-base.ac) turn to the flight model's azimuths (systems/rotor/pale<k>pos-deg,
  through the multiplayer properties as before), lag and flap about the coincident hinges 0.119 m from the shaft
  (/rotors/main/blade[k]/lag-deg, lead +; pale<k>flap-deg, up +) as the rigid blades they are, and pitch about the
  feathering axis (/rotors/main/blade[k]/incidence-deg);
- as the rotor speeds up each blade trails a fan of STRIPS faint wedges over SWEEP deg per rpm, which reaches the next
  blade at flight rpm: a translucent disc as dense on average as the blades' time-averaged coverage x GAIN, a smear at
  the blade's own position; opaque below SOLID_RPM, the blade fades out to nothing at FADE_RPM as the fan fades in
  (Models/Effects/RotorBlur, with its shadow; the select animations switch them at the same rpm). The fan lags and
  cones with the blade but does not pitch. Its colour is the blade's paint along the span;
- the old blurred blade and disc sector (Rotor-PaleD, Rotor-Disc_M-Pale) are hidden;
- the tail rotor, whose three blades are one object of the 3-D model, keeps its spin and disc: the blades fade out and
  the disc in through the same effect (/rotors/tail/rpm), in place of the blend animations.

Frames: the blade instances' animations act in the blade model's own frame (FlightGear's model frame, x aft, y right,
z up, the .ac's offsets inside them), where the blade lies along -x from the hub, its leading edge toward +y, turning
clockwise seen from above (about -z); the instance's heading offset of 180 deg turns the whole outside them.

    python3 rotorvis.py      writes Models/Rotors/Models/al3-blade-rotor.ac and its blur.png, Models/Rotors/AL3-Rotor-Base_rb.xml
                             and AL3-Rotor-Pale<k>_rb.xml, and the tail rotor's animations in Models/Alouette-III-sc_jsb.xml
"""
import math, os, re
import numpy as np
from PIL import Image

import al3data as U

HERE = os.path.dirname(os.path.abspath(__file__))
MODELS = os.path.join(HERE, '..', 'Alouette-3', 'Models')
ROT = os.path.join(MODELS, 'Rotors')
SRC_AC = os.path.join(ROT, 'Models', 'al3-rotor-pale-base.ac')
OUT_AC = os.path.join(ROT, 'Models', 'al3-blade-rotor.ac')
BLUR = os.path.join(ROT, 'Models', 'Textures', 'al3-blade-blur.png')
EFFECT = 'Aircraft/Alouette-3/Models/Effects/RotorBlur/rotor-blur'
MAIN_XML = os.path.join(MODELS, 'Alouette-III-sc_jsb.xml')

RPM = U.OMEGA * 30 / math.pi            # 100 %: 353.2 rpm
STRIPS = 24                             # wedges per blade
SWEEP = 360.0 / U.NB / RPM              # deg per rpm: the fan reaches the next blade at 100 % rotor speed
GAIN = 2.5                              # x the blades' time-averaged coverage (as the Bo 105's)
IMAGE, SMEAR = 0.5, 2.0                 # the share of it in the blades' smears; their decay, in wedges
SOLID_RPM, FADE_RPM = 0.08 * RPM, 0.62 * RPM
E = 0.119                               # m: the flap and lag hinges [3D: the blade's flap animation centre]
ROOT, TIP = 0.96, 5.49                  # m: the blade's span in the 3-D model
PIECES = 16
SPAN = [(ROOT * (TIP / ROOT) ** (i / PIECES), ROOT * (TIP / ROOT) ** ((i + 1) / PIECES)) for i in range(PIECES)]
CHORD = 0.35

SEGMENTS = ['Rotor-Pale-S%d' % k for k in range(8)]
GROUP = 'Rotor-Pale'                    # the segments' group
TBLADE = ['t' + s for s in SEGMENTS]    # the blade's fading copy
FITTINGS = ['Rotor-Pale-Base', 'Rotor-Pale-Fix', 'Rotor-Pale-Rot', 'Rotor-Pale-Cable']
OLD_BLUR = ['Rotor-PaleD', 'Rotor-Disc_M-Pale']
CABLE_PIVOT = (-0.767709, 0.0, 0.06164)          # the damper cable's far end: it counter-turns with the flap (as the original)
CRASH_TIPS = SEGMENTS[4:]

# the tail rotor (Models/Alouette-III-sc_jsb.xml): its blades one object, spun with its hub; its blur disc
TR_RPM = RPM * U.GEAR_TR
TR_SOLID, TR_FADE = SOLID_RPM * U.GEAR_TR, FADE_RPM * U.GEAR_TR
TR_BEGIN, TR_END = '\t<!--== tail rotor\'s blur: written by fdm/rotorvis.py ==-->', '\t<!--== end of fdm/rotorvis.py\'s tail rotor ==-->'


def wedge(k):
    return 'mr_blur%d' % k


WEDGES = [wedge(k) for k in range(STRIPS)]


# ---------------------------------------------------------------- the .ac
def ac_objects(text):
    """[(name, start line, end line (its 'kids 0' or kids line), loc, absolute loc, mats, parent)] of an AC3D file's
    objects, depth first, with their positions in the file's lines"""
    L = text.split('\n')
    out = []
    i = next(k for k, l in enumerate(L) if l.startswith('OBJECT world'))

    def walk(i, off, parent):
        start = i
        name, loc, mats, kids = None, [0.0, 0.0, 0.0], [], 0
        i += 1
        while True:
            l = L[i]
            if l.startswith('name '):
                name = l[5:].strip('"')
            elif l.startswith('loc '):
                loc = [float(c) for c in l.split()[1:4]]
            elif l.startswith('numvert'):
                i += int(l.split()[1])
            elif l.startswith('mat '):
                mats.append(int(l.split()[1]))
            elif l.startswith('kids'):
                kids = int(l.split()[1])
                end = i
                absloc = [o + c for o, c in zip(off, loc)]
                out.append(dict(name=name, start=start, end=end, loc=loc, abs=absloc, mats=mats, parent=parent, kids=kids))
                i += 1
                for _ in range(kids):
                    i = walk(i, absloc, name)
                return i
            i += 1
    walk(i, [0.0, 0.0, 0.0], None)
    return L, out


def materials(L):
    return [l for l in L if l.startswith('MATERIAL')]


def mat_rgb(m):
    return tuple(float(c) for c in re.search(r'rgb ([\d.]+) ([\d.]+) ([\d.]+)', m).groups())


def extent_x(L, ob):
    """the object's vertices' x range in its frame plus its absolute loc (the model frame's x is the .ac's)"""
    i = ob['start']
    while not L[i].startswith('numvert'):
        i += 1
    n = int(L[i].split()[1])
    xs = [float(L[i + 1 + k].split()[0]) + ob['abs'][0] for k in range(n)]
    return min(xs), max(xs)


def span_colours(L, objs):
    """the blade's paint over SPAN's pieces: each segment's commonest material, by where along the blade it lies"""
    mats = materials(L)
    segs = []
    for ob in objs:
        if ob['name'] in SEGMENTS:
            x0, x1 = extent_x(L, ob)
            m = max(set(ob['mats']), key=ob['mats'].count)
            segs.append((-x1, -x0, mat_rgb(mats[m])))
    out = []
    for r0, r1 in SPAN:
        rm = 0.5 * (r0 + r1)
        out.append(next((c for a, b, c in segs if a - 0.02 <= rm <= b + 0.02), segs[-1][2]))
    return out


def point(r, a, zp=0.0):
    """a point of the blade's plane at radius r, a rad along the rotation (toward +y), in the .ac's frame (x, z up, -y)"""
    X, Y, Z = -r * math.cos(a), r * math.sin(a), zp
    return (X, Z, -Y)


def fan_objects(colours, mat_index):
    """the fan's wedges (all at the blade: the XML turns them back), their texture: a swatch of colour and opacity each
    (the HDR transparent shader takes its alpha from the texture)"""
    d = math.radians(360.0 / U.NB / (STRIPS - 1))       # their spacing at flight rpm; each 2 d wide
    tex = Image.new('RGBA', (256, 256), (0, 0, 0, 0)); cells = []; seen = {}
    g = [math.exp(-k / SMEAR) for k in range(STRIPS)]

    def swatch(rgb, a):
        key = (rgb, int(round(255 * a)))
        if key in seen:
            return seen[key]
        n = len(cells); x, y = 8 * (n % 32), 8 * (n // 32)
        assert n < 1024
        tex.paste(tuple(int(round(255 * c)) for c in rgb) + (int(round(255 * a)),), (x, y, x + 8, y + 8))
        cells.append(((x + 4) / 256.0, 1 - (y + 4) / 256.0)); seen[key] = cells[-1]
        return cells[-1]
    objs = []
    for k in range(STRIPS):
        w = (1 - IMAGE) * (0.5 if k in (0, STRIPS - 1) else 1.0) + IMAGE * STRIPS * g[k] / sum(g)
        faces = []
        for i, (r0, r1) in enumerate(SPAN):
            sigma = U.NB * CHORD / (2 * math.pi * math.sqrt(r0 * r1))
            a = min(0.9, GAIN * sigma / 2 * w)
            faces.append((swatch(colours[i], a), [point(r0, 0.0), point(r1, 0.0), point(r1, -2 * d), point(r0, -2 * d)]))
        V = [pt for _, pts in faces for pt in pts]
        lines = ['OBJECT poly', 'name "%s"' % wedge(k), 'loc 0 0 0', 'texture "%s"' % os.path.basename(BLUR), 'texrep 1 1',
                 'numvert %d' % len(V)] + ['%.5f %.5f %.5f' % p for p in V] + ['numsurf %d' % len(faces)]
        for fi, (uv, _) in enumerate(faces):
            lines += ['SURF 0x30', 'mat %d' % mat_index, 'refs 4'] + ['%d %.5f %.5f' % ((4 * fi + q,) + uv) for q in range(4)]
        objs.append('\n'.join(lines + ['kids 0']))
    return objs, tex


def build_ac():
    text = open(SRC_AC, encoding='latin-1').read()
    L, objs = ac_objects(text)
    world_kids = next(k for k, l in enumerate(L) if l.startswith('kids'))
    nw = int(L[world_kids].split()[1])
    new = []
    # the blade's fading copy: the segments out of their group, at their absolute place
    for ob in objs:
        if ob['name'] in SEGMENTS:
            blk = L[ob['start']:ob['end'] + 1]
            blk = [('name "t%s"' % ob['name']) if l.startswith('name ') else
                   ('loc %.6f %.6f %.6f' % tuple(ob['abs'])) if l.startswith('loc ') else l for l in blk]
            if not any(l.startswith('loc ') for l in blk):
                blk.insert(2, 'loc %.6f %.6f %.6f' % tuple(ob['abs']))
            new.append('\n'.join(blk))
    mats = materials(L)
    mat_index = len(mats)
    last_mat = max(k for k, l in enumerate(L) if l.startswith('MATERIAL'))
    L.insert(last_mat + 1, 'MATERIAL "blur" rgb 1 1 1  amb 1 1 1  emis 0 0 0  spec 0 0 0  shi 0  trans 0')
    world_kids += 1
    colours = span_colours(L, ac_objects('\n'.join(L))[1])
    fan, tex = fan_objects(colours, mat_index)
    new += fan
    L[world_kids] = 'kids %d' % (nw + len(new))
    while L and L[-1] == '':
        L.pop()
    open(OUT_AC, 'w', encoding='latin-1').write('\n'.join(L + new) + '\n')
    tex.save(BLUR)
    return colours


# ---------------------------------------------------------------- the XML
def objects(names, indent='    '):
    return ''.join('%s<object-name>%s</object-name>\n' % (indent, n) for n in names)


def rotate(objs, prop, axis, centre=(0, 0, 0), factor=None, offset=None, comment=None):
    out = '\n  <!-- %s -->\n' % comment if comment else '\n'
    out += '  <animation>\n    <type>rotate</type>\n' + objects(objs)
    if prop:
        out += '    <property>%s</property>\n' % prop
    if factor is not None:
        out += '    <factor> %.6g </factor>\n' % factor
    if offset is not None:
        out += '    <offset-deg> %.6g </offset-deg>\n' % offset
    out += ('    <center>\n      <x-m> %.5f </x-m>\n      <y-m> %.5f </y-m>\n      <z-m> %.5f </z-m>\n    </center>\n'
            '    <axis>\n      <x> %g </x>\n      <y> %g </y>\n      <z> %g </z>\n    </axis>\n  </animation>\n'
            % (tuple(centre) + tuple(axis)))
    return out


def select(objs, cond, comment=None):
    out = '\n  <!-- %s -->\n' % comment if comment else '\n'
    return out + '  <animation>\n    <type>select</type>\n' + objects(objs) + '    <condition>\n%s    </condition>\n  </animation>\n' % cond


def fade(objs, f, t, rough, shadow, comment, rpm=None, indent='  '):
    """the rotor-blur effect on objs, faded from f to t rpm (of /rotors/main/rpm, or rpm)"""
    use = '      <rotor-rpm type="float"><use>%s</use></rotor-rpm>\n' % rpm if rpm else ''
    x = ('\n  <!-- %s -->\n  <effect>\n    <inherits-from>%s</inherits-from>\n'
         '    <parameters>\n%s      <fade-from type="float"> %g </fade-from>\n      <fade-to type="float"> %g </fade-to>\n'
         '      <pbr-roughness type="float"> %g </pbr-roughness>\n      <shadow-gain type="float"> %g </shadow-gain>\n'
         '    </parameters>\n' % (comment, EFFECT, use, f, t, rough, shadow) + objects(objs) + '  </effect>\n')
    return x if indent == '  ' else x.replace('\n  ', '\n' + indent)


def base_xml():
    rpm = '/rotors/main/rpm'
    lt = lambda r: '<less-than><property>%s</property><value> %g </value></less-than>' % (rpm, r)
    x = ('<?xml version="1.0" encoding="ISO-8859-1"?>\n\n<!-- One main rotor blade, written by fdm/rotorvis.py (see there): the 3-D model\'s blade\n'
         '     (Gerard Robin\'s) with its fading copy and the fan of its blur; AL3-Rotor-Pale<k>_rb.xml turn it -->\n\n<PropertyList>\n\n'
         '  <path>Models/al3-blade-rotor.ac</path>\n  <texture-path>Models/Textures</texture-path>\n'
         '  <offsets>\n    <heading-deg>180</heading-deg>\n  </offsets>\n')
    x += fade(WEDGES, SOLID_RPM, FADE_RPM, 1.0, 2.0, 'the fan fades in with the rpm; its shadow x 2: two wedges over each point')
    x += fade(TBLADE, FADE_RPM, SOLID_RPM, 0.5, 1.0, 'the blade fades out')
    x += select(SEGMENTS, '      %s\n' % lt(SOLID_RPM), 'the opaque blade at low rpm')
    x += select(TBLADE, '      <and>\n        <not>%s</not>\n        %s\n      </and>\n' % (lt(SOLID_RPM), lt(FADE_RPM)), 'then its fading copy')
    x += select(WEDGES, '      <not>%s</not>\n' % lt(SOLID_RPM), 'and the fan')
    x += select(OLD_BLUR, '      <property>sim/model/al3/old-rotor-blur</property>\n', 'the old blurred blade and disc sector: not shown (the fan replaces them)')
    for k in range(1, STRIPS):
        x += rotate([wedge(k)], rpm, (0, 0, -1), factor=-SWEEP * k / (STRIPS - 1),
                    comment='wedge k trails the blade by k / %d of the fan (the rotation is about -z)' % (STRIPS - 1) if k == 1 else None)
    f = {a: ('    <%s-factor> %.7f </%s-factor>\n    <%s-offset> 0 </%s-offset>\n    <%s-min> 0.02 </%s-min>\n    <%s-max> 1 </%s-max>\n'
             % (a, 1.0 / RPM, a, a, a, a, a, a, a)) if a == 'y' else
         ('    <%s-factor> 0 </%s-factor>\n    <%s-offset> 1 </%s-offset>\n' % (a, a, a, a)) for a in 'xyz'}
    x += ('\n  <!-- the wedges as wide as twice their spacing at any rpm (al3-blade-rotor.ac: at 100 %) -->\n  <animation>\n    <type>scale</type>\n'
          + objects(WEDGES) + '    <property>%s</property>\n' % rpm + f['x'] + f['y'] + f['z'] + '  </animation>\n')
    return x + '\n</PropertyList>\n'


def blade_xml(k):
    blade = [GROUP] + TBLADE
    moving = blade + WEDGES
    az = 'sim/multiplay/generic/float[%d]' % (5 + k)
    x = ('<?xml version="1.0" encoding="ISO-8859-1"?>\n\n<!-- Main rotor blade %d, written by fdm/rotorvis.py (see there): the blade on the flight model\'s\n'
         '     azimuth, lag, flap and pitch of its blade %d -->\n\n<PropertyList>\n\n'
         '  <model>\n    <name>Rotor</name>\n    <path>Aircraft/Alouette-3/Models/Rotors/AL3-Rotor-Base_rb.xml</path>\n  </model>\n' % (k, k))
    x += rotate(moving + FITTINGS, az, (0, 0, -1),
                comment='the azimuth (systems/rotor/pale%dpos-deg, through the multiplayer property), clockwise from above' % k)
    x += rotate(moving + ['Rotor-Pale-Base', 'Rotor-Pale-Fix', 'Rotor-Pale-Cable'], '/rotors/main/blade[%d]/lag-deg' % k, (0, 0, -1), (-E, 0, 0),
                comment='lag about the hinge, lead +')
    x += rotate(moving + FITTINGS, 'fdm/jsbsim/systems/rotor/pale%dflap-deg' % k, (0, 1, 0), (-E, 0, 0),
                comment='flap about the hinge, up +: a rigid blade')
    x += rotate(['Rotor-Pale-Cable'], 'fdm/jsbsim/systems/rotor/pale%dflap-deg' % k, (0, -1, 0), CABLE_PIVOT,
                comment='the damper cable stays on its far end')
    x += rotate(blade + ['Rotor-Pale-Base'], '/rotors/main/blade[%d]/incidence-deg' % k, (1, 0, 0),
                comment='pitch about the feathering axis: collective and cyclic, leading edge up +')
    x += select(CRASH_TIPS + ['t' + s for s in CRASH_TIPS], '      <property>fdm/jsbsim/systems/crash-detect/aero-on</property>\n',
                'the tips break off in a crash (as the original)')
    return x + '\n</PropertyList>\n'


# ---------------------------------------------------------------- the tail rotor
def tail():
    s = open(MAIN_XML).read()
    if TR_BEGIN in s:
        s = s[:s.index(TR_BEGIN)] + s[s.index(TR_END) + len(TR_END) + 1:]
    # the blend animations: out (the HDR pipeline leaves them out)
    for objs in (('Rotor-Pales-Queue', 'Rotor-Queue-Tete'), ('Queue-Disc_M',)):
        m = re.search(r'(?s)\n\t<animation>\n\t\t<type>blend</type>\n%s.*?</animation>' %
                      ''.join(r'\s*<object-name>%s</object-name>' % o for o in objs), s)
        if m:
            s = s[:m.start()] + s[m.end():]
    # the disc out of the model-transparent effect (the blur effect is model-transparent's, with the fade)
    s = re.sub(r'(?s)(<inherits-from>Effects/model-transparent</inherits-from>\s*)<object-name>Queue-Disc_M</object-name>\s*', r'\1', s)
    # the switches at the fade's ends
    s = re.sub(r'(<property>sim/multiplay/generic/float\[8\]</property>\s*<value>)1000(</value>)', r'\g<1>%g\2' % TR_FADE, s)
    s = re.sub(r'(<property>sim/multiplay/generic/float\[8\]</property>\s*<value>)950(</value>)', r'\g<1>%g\2' % TR_SOLID, s)
    block = (TR_BEGIN + '\n\t<!-- the blades fade out and the disc in with the tail rotor\'s rpm (the Bo 105\'s blur effect) -->'
             + fade(['Rotor-Pales-Queue'], TR_FADE, TR_SOLID, 0.5, 1.0, 'the blades fade out', rpm='/rotors/tail/rpm', indent='\t')
             + fade(['Queue-Disc_M'], TR_SOLID, TR_FADE, 1.0, 1.0, 'the disc fades in', rpm='/rotors/tail/rpm', indent='\t')
             + TR_END + '\n')
    i = s.index('<!--==============END   ROTOR   TAIL=============-->')
    s = s[:i] + block + s[i:]
    open(MAIN_XML, 'w').write(s)


if __name__ == '__main__':
    colours = build_ac()
    open(os.path.join(ROT, 'AL3-Rotor-Base_rb.xml'), 'w', encoding='latin-1').write(base_xml())
    for k in range(U.NB):
        open(os.path.join(ROT, 'AL3-Rotor-Pale%d_rb.xml' % k), 'w', encoding='latin-1').write(blade_xml(k))
    tail()
    print('main rotor: %d wedges per blade over %.0f deg at %.0f rpm, fan from %.0f rpm, blade gone at %.0f; colours %s'
          % (STRIPS, 360.0 / U.NB, RPM, SOLID_RPM, FADE_RPM, sorted(set(colours))))
    print('tail rotor: blades fade out by %.0f rpm, the disc in from %.0f' % (TR_FADE, TR_SOLID))
