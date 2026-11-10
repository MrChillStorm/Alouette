#!/usr/bin/env python3
"""The 3-D model's exterior lights and glass for FlightGear's HDR pipeline (from 2024.2 its only one; Nasal/hdr.nas
sets /sim/rendering/hdr/hdr-enabled), as the Bo 105 and UH-60 models'.

- Exterior lights: the old ones are ALS procedural lights (Models/Lights/procedural_*.xml, shown with the ALS sky
  dome) and, without it, the classic billboards; the HDR pipeline renders neither as a light. Under HDR they are
  hidden, and each light gets the X-57 model's halo and an HDR spot light aimed out through its lens (that model's
  fdm/x57lights.py, after the Bo 105's and the UH-60's; its halo assets in fdm/lights): the halo glows through
  Effects/model-pbr-transparent, whose emissive factor is the lamp's intensity spread over the halo at its largest
  (the HDR pipeline leaves out Effects/model-transparent's emission, so a halo there shone only by the light falling
  on it, at night not at all); it sits just outside its lens along its beam, or the lens hides its bright middle;
  the narrow searchlight's is a disc facing along the beam. Intensities in candela / 635 (the pipeline's unit), at
  the Bo 105's and X-57's levels. The lamps are the 3-D model's lenses (Models/alouetteIII_SC-ss_cockpit.ac: the side
  lights' spheres, the beacons' domes, the tail light's sphere, the searchlight's lamp SAR-light). They keep the
  original aircraft's states (Systems/electrical-lights.xml): the red left and green right position lights flash in
  turn on the STROBE switch; the red anticollision beacons on the cabin roof and the belly flash twice a second on
  BEACON, and the white tail light between them; the searchlight under the nose (LANDING LIGHT) turns with its lamp,
  which swings out 30 deg as it lights (animation/landing-light-pos-norm), to 12 deg below the fuselage's axis.
- Glass: the canopy, the door windows and the mirror (Models/alouetteIII_SC-ss_cockpit.ac) had no texture, a legacy
  chrome shader and the material's transparency, all of which the HDR pipeline leaves out: opaque grey there. They
  get a plain white texture (Models/Textures/glass.png) and their own effect, Models/Effects/Glass/al3-glass: HDR's
  physically based transparent shader (Effects/model-pbr-transparent) as the X-57 pilot's sunglasses,
  with the tint and the opacity in its base colour (light, 28 % opaque: clear) and a smooth surface (roughness 0.04,
  metallic 0.1) that reflects the sky and the sun, which is what makes it read as glass. Its surfaces are one-sided, facing out: seen from
  the cabin they are culled, and the inside of the glass is a second layer facing in (alouetteIII-windows.ac: the
  doors' windows; alouetteIII-glass-inner.ac: the canopy), which had the ALS glass shader, drawn opaque and dark by
  the HDR pipeline. It gets the same texture and effect: from outside the outer layer shows, from inside the inner
  (the PBR shader culls back faces).
- The engine's heat blur (an ALS shader) is hidden under HDR likewise.

    python3 hdrmodel.py      writes Models/Lights/HDR, edits Models/Alouette-III-sc_jsb.xml (idempotent)
"""
import os, re, shutil

import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
MODELS = os.path.join(HERE, '..', 'Alouette-3', 'Models')
MAIN = os.path.join(MODELS, 'Alouette-III-sc_jsb.xml')
ASSETS = os.path.join(HERE, 'lights')     # the X-57 model's halos (its fdm/lights; the UH-60 model's lights.png)
HDR_CD = 635.0
HDR = '/sim/rendering/hdr/hdr-enabled'
J = 'fdm/jsbsim/'

RED, GREEN, WHITE, WARM = (1.0, 0.06, 0.03), (0.15, 1.0, 0.45), (1.0, 1.0, 1.0), (1.0, 0.92, 0.80)
# the original states (Models/Lights/procedural_*.xml), as condition bodies
NAV_GREEN = ('<and>\n<property>%ssystems/effets/lights/blinker-kinemat-in</property>\n<property>%selectrical/strobe-lights</property>\n</and>' % (J, J))
NAV_RED = ('<and>\n<not-equals>\n<property>%ssystems/effets/lights/blinker-kinemat-in</property>\n<value>1</value>\n</not-equals>\n'
           '<property>%selectrical/strobe-lights</property>\n</and>' % (J, J))
BEACON = '<and>\n<not>\n<property>%ssystems/effets/lights/beacon2-on</property>\n</not>\n<property>%selectrical/beacon</property>\n</and>' % (J, J)
TAIL = '<and>\n<property>%ssystems/effets/lights/beacon1-on</property>\n<property>%selectrical/beacon</property>\n</and>' % (J, J)
LANDING = '<property>%selectrical/landing-light</property>' % J
# name, halo ('disc': a flat one facing along the beam), lamp (model frame: x aft, y right, z up, m: its lens's centre in
# the 3-D model), direction, colour, emission (other pipelines), cd on axis, spot exponent, cutoff (deg), range (m), state
LIGHTS = [('nav-green', 'GreenLight.ac', (-2.199, 0.932, -0.486), (0, 1, 0), GREEN, (0, 2, 0), 40, 3, 70, 20, NAV_GREEN),
          ('nav-red', 'RedLight.ac', (-2.199, -0.932, -0.486), (0, -1, 0), RED, (2, 0, 0), 40, 3, 70, 20, NAV_RED),
          ('beacon-top', 'RedLight.ac', (-1.240, 0.0, 0.930), (0, 0, 1), RED, (2, 0, 0), 50, -1, 80, 30, BEACON),
          ('beacon-bottom', 'RedLight.ac', (1.034, 0.0, -0.900), (0, 0, -1), RED, (2, 0, 0), 50, -1, 80, 30, BEACON),
          ('tail', 'WhiteLight.ac', (6.365, 0.0, 0.312), (1, 0, 0), WHITE, (2, 2, 2), 400, 1, 80, 40, TAIL),
          # the searchlight: its reflector (Phare_Refl) faces 42 deg below the fuselage's axis stowed, and the lamp turns 30 deg
          # up about its pivot as it lights (animation/landing-light-pos-norm, the light with it): 12 deg below the axis, a
          # landing light's. A warm white beam of 200 000 cd at its centre, a halogen landing lamp's, as the Bo 105's and X-57's
          ('landing', 'disc', (-2.583, 0.0, -0.567), (-0.743, 0, -0.669), WARM, (3, 3, 3), 200000, 20, 12, 400, LANDING)]
# how far out along the beam each halo sits from its lamp: the lens's reach that way (the 3-D model) and 5 mm
HALO_OUT = {'nav-green': 0.011, 'nav-red': 0.011, 'beacon-top': 0.066, 'beacon-bottom': 0.068, 'tail': 0.024, 'landing': 0.033}
HALO_SCALE = [(0, 0.15), (20, 0.35), (1500, 4.0)]      # m -> the halo's size (m)
GLASS = ['Vitrage', 'AV-DPorte-Vitrage', 'AR-DPorte-Vitrage', 'AV-GPorte-Vitrage', 'AR-GPorte-Vitrage', 'rear-view mirror-glass']
GLASS_RGBA = (255, 255, 255, 255)       # white: the effect's base colour gives the tint and the opacity
# the glass's effect (HDR's transparent PBR shader blends the lit surface, the sun's and the sky's reflections included,
# by its alpha). Glass shows itself by what it reflects, not by light it scatters: metallic 1 leaves the surface no
# diffuse part, so no milky film over the view and no shadows cast on it (the shadow maps darken only the sun's
# light, which a smooth reflector shows only in its glint). 12 % opaque (88 % of the view through it); the base colour
# is the reflectance, a light grey-green, so that it reflects about 8 % face on (a pane's two faces) with a faint tint
# and more towards grazing; smooth (roughness 0.05: the sun's glint sharp). (Before, a 28 % diffuse grey-green showed
# cast shadows; the Bo 105's ALS glass under HDR: a dark grey film 9 % opaque. The X-57's sunglasses, dark: 0.045
# 0.085 0.045 at 0.8, roughness 0.04.) From inside a pane reflects the cabin's side, which HDR's light probe fills
# with the ground, so clean glass shows little there in daylight: 18 % on the inner layers changed the view through
# them by 2-3 % (FlightGear, EFHK at noon in November).
GLASS_EFFECT = 'Aircraft/Alouette-3/Models/Effects/Glass/al3-glass'
GLASS_BASE_COLOR = (0.60, 0.67, 0.64, 0.12)
GLASS_METALLIC = 1.0
GLASS_ROUGHNESS = 0.05
MAIN_AC = os.path.join(MODELS, 'alouetteIII_SC-ss_cockpit.ac')
GLOW_REF_M = HALO_SCALE[-1][0]
PROCEDURAL = ['upper_beacon_red', 'lower_beacon_red', 'rear_beacon', 'flashlight_green', 'flashlight_red', 'landing_light_white']
GLASS_LAYERS = ['alouetteIII-glass-inner.xml', 'alouetteIII-windows.xml']
# the inner layers (facing the cabin): their .ac files' glass objects
INNER_GLASS = {'alouetteIII-windows': ['AR-DPorte-Vitrage', 'AR-GPorte-Vitrage', 'AV-DPorte-Vitrage', 'AV-GPorte-Vitrage'],
               'alouetteIII-glass-inner': ['Windshield']}
BEGIN = '\t<!--== HDR lights: written by fdm/hdrmodel.py ==-->'
END = '\t<!--== end of fdm/hdrmodel.py\'s lights ==-->'
SKYDOME = '<property>/sim/rendering/shaders/skydome</property>'
NOT_HDR = '<not>\n\t\t\t<property>%s</property>\n\t\t</not>' % HDR


# The halos' glow (the X-57 model's fdm/x57lights.py, which explains it): the lamp's intensity spread over the halo at
# its largest, sum(alpha x luminance) over the quad x its size squared = cd on axis, so that no halo carries more light
# than its lamp from any distance; HDR's bloom spreads its brightest part.

def disc_glow(n=256):
    """the disc's texture: white, alpha x linear value = 1 / (1 + (r / 0.05)^2) x (1 - (r / 0.5)^2)^2 across the quad (r
    from its centre, the quad 1 wide), split evenly between alpha and colour"""
    c = (np.arange(n) + 0.5) / n - 0.5
    r = np.hypot(*np.meshgrid(c, c))
    g = np.where(r < 0.5, 1.0 / (1.0 + (r / 0.05) ** 2) * np.clip(1.0 - (r / 0.5) ** 2, 0.0, 1.0) ** 2, 0.0)
    a = np.sqrt(g)
    srgb = np.where(a <= 0.0031308, 12.92 * a, 1.055 * a ** (1 / 2.4) - 0.055)
    return np.dstack([srgb, srgb, srgb, a])


def unit(d):
    n = sum(c * c for c in d) ** 0.5
    return tuple(c / n for c in d)


def halo_pos(l):
    """where a light's model sits: its halo, just outside the lens"""
    return tuple(p + HALO_OUT[l[0]] * c for p, c in zip(l[2], unit(l[3])))


def halo_quad(ac):
    """the texture coordinates of a halo's first quad"""
    t = open(os.path.join(ASSETS, ac)).read()
    return [tuple(map(float, m.groups())) for m in re.finditer(r'^\d+ (\S+) (\S+)$', t, re.M)][:4]


def glow_k(ac, rgb=(1.0, 1.0, 1.0)):
    """the mean over a halo's quad of alpha x the luminance of the texture's linear colour times rgb"""
    if ac == 'disc':
        r = np.round(disc_glow() * 255.0) / 255.0
    else:
        im = np.asarray(Image.open(os.path.join(ASSETS, 'lights.png')).convert('RGBA'), float) / 255.0
        uv = halo_quad(ac)
        H, W = im.shape[:2]
        u, v = [q[0] for q in uv], [q[1] for q in uv]
        r = im[int((1 - max(v)) * H):int((1 - min(v)) * H), int(min(u) * W):int(max(u) * W)]
    lin = np.where(r[..., :3] <= 0.04045, r[..., :3] / 12.92, ((r[..., :3] + 0.055) / 1.055) ** 2.4)
    return float((((lin * rgb) @ (0.2126, 0.7152, 0.0722)) * r[..., 3]).mean())


def glow(halo, rgb, cd):
    """the emissive factor, in the pipeline's unit"""
    s = float(np.interp(GLOW_REF_M, [e[0] for e in HALO_SCALE], [e[1] for e in HALO_SCALE]))
    lum = cd / (glow_k(halo, rgb) * s * s)
    return tuple(lum * q / HDR_CD for q in rgb)


def disc_ac(d):
    """a halo as one quad facing along the beam d (model frame: x aft, y right, z up), in AC3D's (x aft, y up, z left)"""
    n = np.array((d[0], d[2], -d[1]), float)
    n /= np.linalg.norm(n)
    a = np.cross(n, (0.0, 1.0, 0.0))
    a = a / np.linalg.norm(a) if np.linalg.norm(a) > 1e-6 else np.array((1.0, 0.0, 0.0))
    b = np.cross(n, a)
    corners = [(-1, -1, 0, 0), (1, -1, 1, 0), (1, 1, 1, 1), (-1, 1, 0, 1)]
    vs = [0.5 * (i * a + j * b) for i, j, _, _ in corners]
    out = ['AC3Db', 'MATERIAL "DefaultWhite" rgb 1 1 1  amb 1 1 1  emis 0 0 0  spec 0.5 0.5 0.5  shi 64  trans 0', 'OBJECT world', 'kids 1',
           'OBJECT poly', 'name "Light"', 'texture "disc.png"', 'crease 45.000001', 'numvert 4']
    out += ['%.6f %.6f %.6f' % tuple(q) for q in vs]
    out += ['numsurf 1', 'SURF 0x10', 'mat 0', 'refs 4'] + ['%d %.6f %.6f' % (k, c[2], c[3]) for k, c in enumerate(corners)] + ['kids 0']
    return '\n'.join(out) + '\n'


def light_xml(name, halo, d, rgb, emis, cd, exp, cut, rng, state):
    sel = lambda obj, extra='': ('  <animation>\n    <type>select</type>\n    <object-name>%s</object-name>\n    <condition>\n'
                                 '      <and>\n%s%s\n      </and>\n    </condition>\n  </animation>\n' % (obj, extra, state))
    g = glow(halo, rgb, cd)
    d = unit(d)
    s = 0.04 - HALO_OUT[name]           # the spot light 4 cm out from the lamp
    path = name + '-disc.ac' if halo == 'disc' else halo
    tex = 'disc.png' if halo == 'disc' else 'lights.png'
    x = ('<?xml version="1.0" encoding="ISO-8859-1"?>\n\n<!-- %s, written by fdm/hdrmodel.py (see there): the X-57 model\'s halo and, in\n'
         '     the HDR pipeline, a spot light out through the lens -->\n\n<PropertyList>\n\n  <path>%s</path>\n  <nopreview/>\n\n' % (name, path))
    x += '  <animation>\n    <type>noshadow</type>\n    <object-name>Light</object-name>\n  </animation>\n\n'
    x += ('  <effect>      <!-- its glow: %g cd over the halo at %g m -->\n    <inherits-from>Effects/model-pbr-transparent</inherits-from>\n    <parameters>\n'
          '      <metallic-factor type="float">0</metallic-factor>\n      <roughness-factor type="float">1</roughness-factor>\n'
          '      <texture n="3">\n        <image>%s</image>\n        <type>2d</type>\n'
          '        <filter>linear-mipmap-linear</filter>\n        <wrap-s>clamp</wrap-s>\n        <wrap-t>clamp</wrap-t>\n      </texture>\n'
          '      <emissive-factor type="vec3d">%.4g %.4g %.4g</emissive-factor>\n    </parameters>\n    <object-name>Light</object-name>\n  </effect>\n\n'
          % ((cd, GLOW_REF_M, tex) + g))
    x += '  <animation>\n    <type>alpha-test</type>\n    <alpha-factor>0.01</alpha-factor>\n  </animation>\n\n'
    if halo != 'disc':
        x += '  <animation>\n    <type>billboard</type>\n    <object-name>Light</object-name>\n    <spherical type="bool">true</spherical>\n  </animation>\n\n'
    x += ('  <animation>\n    <type>material</type>\n    <object-name>Light</object-name>\n    <emission>\n      <red> %g </red>\n'
          '      <green> %g </green>\n      <blue> %g </blue>\n    </emission>\n  </animation>\n\n' % emis)
    x += ('  <animation>\n    <type>dist-scale</type>\n    <object-name>Light</object-name>\n    <interpolation>\n' +
          ''.join('      <entry><ind> %g </ind><dep> %g </dep></entry>\n' % e for e in HALO_SCALE) + '    </interpolation>\n  </animation>\n\n')
    x += sel('Light') + '\n'
    x += ('  <light>\n    <name>%s-light</name>\n    <type>spot</type>\n    <position>\n      <x-m> %g </x-m>\n      <y-m> %g </y-m>\n'
          '      <z-m> %g </z-m>\n    </position>\n    <direction>\n      <lookat-x-m> %g </lookat-x-m>\n      <lookat-y-m> %g </lookat-y-m>\n'
          '      <lookat-z-m> %g </lookat-z-m>\n    </direction>\n    <color>\n      <r> %g </r>\n      <g> %g </g>\n      <b> %g </b>\n'
          '    </color>\n    <intensity> %.5f </intensity>      <!-- %g cd -->\n    <spot-exponent> %g </spot-exponent>\n'
          '    <spot-cutoff> %g </spot-cutoff>\n    <range-m> %g </range-m>\n  </light>\n\n'
          % ((name,) + tuple(round(s * c, 4) for c in d) + tuple(round(s * c + c, 4) for c in d) + tuple(rgb) + (cd / HDR_CD, cd, exp, cut, rng)))
    x += '  <animation>\n    <type>noshadow</type>\n    <object-name>%s-light</object-name>\n  </animation>\n\n' % name
    x += sel('%s-light' % name, '        <property>%s</property>\n' % HDR)
    return x + '\n</PropertyList>\n'


def lights():
    folder = os.path.join(MODELS, 'Lights', 'HDR'); os.makedirs(folder, exist_ok=True)
    for f in ('RedLight.ac', 'GreenLight.ac', 'WhiteLight.ac', 'lights.png'):
        shutil.copy(os.path.join(ASSETS, f), folder)
    Image.fromarray(np.round(disc_glow() * 255.0).astype(np.uint8), 'RGBA').save(os.path.join(folder, 'disc.png'))
    for name, halo, pos, d, rgb, emis, cd, exp, cut, rng, state in LIGHTS:
        if halo == 'disc':
            open(os.path.join(folder, name + '-disc.ac'), 'w').write(disc_ac(d))
        open(os.path.join(folder, name + '.xml'), 'w').write(light_xml(name, halo, d, rgb, emis, cd, exp, cut, rng, state))


def model_block():
    x = [BEGIN, '\t<!-- shown under HDR only (the halos in every pipeline would double the old lights) -->']
    for l in LIGHTS:
        name, pos = l[0], halo_pos(l)
        x.append('\t<model>\n\t\t<name>light-%s</name>\n\t\t<path>Aircraft/Alouette-3/Models/Lights/HDR/%s.xml</path>\n\t\t<offsets>\n'
                 '\t\t\t<x-m>%g</x-m>\n\t\t\t<y-m>%g</y-m>\n\t\t\t<z-m>%g</z-m>\n\t\t</offsets>\n\t\t<condition>\n\t\t\t<property>%s</property>\n'
                 '\t\t</condition>\n\t</model>' % ((name, name) + tuple(round(c, 4) for c in pos) + (HDR,)))
    x.append(END)
    return '\n'.join(x) + '\n\n'


def model_condition(s, name, cond):
    """the <model> named name (by <name> or <path>) gets the condition cond (replacing its own)"""
    m = re.search(r'(?s)<model>(?:(?!</model>).)*?(?:<name>%s</name>|<path>[^<]*/%s</path>)(?:(?!</model>).)*?</model>' % (re.escape(name), re.escape(name)), s)
    assert m, name
    blk = m.group(0)
    blk2 = re.sub(r'(?s)\s*<condition>.*?</condition>', '', blk)
    blk2 = blk2.replace('</model>', '\t<condition>\n\t\t%s\n\t    </condition>\n        </model>' % cond)
    return s[:m.start()] + blk2 + s[m.end():]


def tint(path, names):
    """the glass objects named of an .ac get the tint texture (their faces' texture coordinates are all one point of it)"""
    s = open(path, encoding='latin-1').read()
    for n in names:
        i = s.index('name "%s"\n' % n)
        j = s.index('numvert', i)
        if 'texture' not in s[i:j]:
            s = s[:j] + 'texture "Textures/glass.png"\ntexrep 1 1\n' + s[j:]
    open(path, 'w', encoding='latin-1').write(s)


def glass_ac():
    tint(MAIN_AC, GLASS)
    for f, names in INNER_GLASS.items():
        tint(os.path.join(MODELS, f + '.ac'), names)
        # their model files: the glass effect in place of the ALS one
        p = os.path.join(MODELS, f + '.xml'); x = open(p).read()
        for old in ('Models/Effects/glass_inner', 'Aircraft/Alouette-3/Models/Effects/Glass/glass', GLASS_EFFECT + '-inner'):
            x = x.replace('<inherits-from>%s</inherits-from>' % old, '<inherits-from>%s</inherits-from>' % GLASS_EFFECT)
        open(p, 'w').write(x)
    Image.new('RGBA', (8, 8), GLASS_RGBA).save(os.path.join(MODELS, 'Textures', 'glass.png'))
    open(os.path.join(MODELS, 'Effects', 'Glass', 'al3-glass.eff'), 'w').write(
        '<?xml version="1.0"?>\n<!-- the Alouette\'s glass: written by fdm/hdrmodel.py -->\n<PropertyList>\n'
        '  <name>al3-glass</name>\n  <inherits-from>Effects/model-pbr-transparent</inherits-from>\n  <parameters>\n'
        '    <base-color-factor type="vec4d">%.3f %.3f %.3f %.3f</base-color-factor>\n'
        '    <metallic-factor type="float">%.3f</metallic-factor>\n'
        '    <roughness-factor type="float">%.3f</roughness-factor>\n  </parameters>\n</PropertyList>\n'
        % (GLASS_BASE_COLOR + (GLASS_METALLIC, GLASS_ROUGHNESS)))


def glass_xml(s):
    """the legacy chrome shader on the canopy and mirror out, the glass effect on all the glass"""
    m = re.search(r'(?s)\n\t<animation>\s*<type>shader</type>\s*<shader>chrome</shader>.*?</animation>\n', s)
    begin, end = '\t<!--== glass: written by fdm/hdrmodel.py ==-->', '\t<!--== end of fdm/hdrmodel.py\'s glass ==-->'
    block = (begin + '\n\t<!-- the physically based glass: tint, opacity and a smooth reflecting surface -->\n\t<effect>\n'
             '\t\t<inherits-from>%s</inherits-from>\n' % GLASS_EFFECT +
             ''.join('\t\t<object-name>%s</object-name>\n' % o for o in GLASS) + '\t</effect>\n' + end + '\n')
    if begin in s:
        s = s[:s.index(begin)] + block + s[s.index(end) + len(end) + 1:]
    else:
        assert m, 'the chrome shader animation'
        s = s[:m.start()] + '\n' + block + s[m.end():]
    # the doors' windows had a chrome shader animation each: out too (it would override the effect)
    for o in GLASS:
        s = re.sub(r'(?s)\n[ \t]*<animation>\s*<type>shader</type>\s*<shader>chrome</shader>\s*<texture>[^<]*</texture>\s*'
                   r'<object-name>%s</object-name>\s*</animation>' % re.escape(o), '', s)
    return s


def install():
    s = open(MAIN).read()
    s = glass_xml(s)
    if BEGIN in s:
        s = s[:s.index(BEGIN)] + s[s.index(END) + len(END) + 2:]
    i = s.index('\t<!--============END OUTSIDE LIGHTS')
    s = s[:i] + model_block() + s[i:]
    als_only = '<and>\n\t\t\t%s\n\t\t\t%s\n\t\t</and>' % (SKYDOME, NOT_HDR)
    for n in PROCEDURAL + ['heat-blur.xml']:
        s = model_condition(s, n, als_only)
    for n in GLASS_LAYERS:                       # shown in every pipeline again (they were hidden under HDR)
        m = re.search(r'(?s)<model>(?:(?!</model>).)*?<path>[^<]*/%s</path>(?:(?!</model>).)*?</model>' % re.escape(n), s)
        blk = re.sub(r'(?s)\s*<condition>.*?</condition>', '', m.group(0))
        s = s[:m.start()] + blk + s[m.end():]
    # the searchlight's HDR light turns with its lamp
    rot = re.search(r'(?s)<type>rotate</type>\s*<object-name>SAR-light</object-name>.*?<property>fdm/jsbsim/animation/landing-light-pos-norm', s)
    if 'light-landing' not in rot.group(0):
        j = rot.start() + rot.group(0).index('<object-name>SAR-light</object-name>') + len('<object-name>SAR-light</object-name>')
        s = s[:j] + '\n\t<object-name>light-landing</object-name>' + s[j:]
    open(MAIN, 'w').write(s)


if __name__ == '__main__':
    glass_ac(); lights(); install()
    print("%d lights; procedural lights and heat blur ALS-only; glass tinted, inside and out" % len(LIGHTS))
