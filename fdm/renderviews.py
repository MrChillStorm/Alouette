#!/usr/bin/env python3
"""The cockpit screen with the rear-view mirror's picture and two cameras, for a FlightGear that renders extra views into
textures.

Stock FlightGear 2024.2 cannot draw a second view of the scene into a texture. A FlightGear built with the render-view
patch can: it reads /sim/rendering/render-views. This writes what the aircraft needs for that. Any other FlightGear is
left as before: it does not read those properties, and the screen stays hidden.

- A screen right of the right-hand pilot's panel shows what the rear-view mirror sees, larger than the mirror itself and
  flipped as a mirror reads: one camera at the mirror, aimed along what its domed glass reflects to the pilot's eye. A
  click goes on to a tail camera, to a camera beside the rescue hoist looking down, and back (Nasal/render_screen.nas).
  The mirror's own glass is left as it is: seen from the seat it is a few dozen pixels, and the screen does its job.
- An extra view draws the scene again, which costs frame rate. So that it costs less: only the view the screen shows is
  drawn, only while the screen is in sight (<shown-on>), and every second frame (frame-interval; 1 draws it every
  frame); the pictures are small; and the views' pipeline leaves out ambient occlusion, the 3-D clouds, the two far
  shadow cascades (shadows reach 40 m: the aircraft's own on the ground stays) and the finer levels of distant models,
  and has its own small sun shadow map.
- The extra views' pipeline is FlightGear's own HDR pipeline cut down (FGDATA/Compositor/HDR, GPL-2.0-or-later, by the
  FlightGear authors): run this again when FlightGear's changes.

    python3 renderviews.py FGDATA

writes Compositor/RenderView, Models/Effects/RenderView and Models/RenderView, and edits Alouette-3-set.xml and
Models/Alouette-III-sc_jsb.xml (idempotent). Written by hand: Shaders/HDR/render-view.frag, Nasal/render_screen.nas.
"""
import math, os, shutil, sys
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image

import acread

HERE = os.path.dirname(os.path.abspath(__file__))
AIRCRAFT = os.path.join(HERE, '..', 'Alouette-3')
MODELS = os.path.join(AIRCRAFT, 'Models')
MAIN = os.path.join(MODELS, 'Alouette-III-sc_jsb.xml')
SET = os.path.join(AIRCRAFT, 'Alouette-3-set.xml')
PIPELINE = os.path.join(AIRCRAFT, 'Compositor', 'RenderView')
EFFECTS = os.path.join(MODELS, 'Effects', 'RenderView')
SCREEN = os.path.join(MODELS, 'RenderView')
PREFIX = 'Aircraft/Alouette-3/'

EYE = np.array((-1.45, 0.556831, 0.59))     # view 0, model frame (x aft, y right, z up), m
HEAD_ROOM = 0.15                            # how far the head may move and still be covered by the mirror's camera, m
SCREEN_CENTRE = np.array((-2.05, 0.80, 0.22))
SCREEN_SIZE = (0.20, 0.15)
CAMERA_SIZE = (384, 288)                    # each view's picture, pixels: the screen's shape
FRAME_INTERVAL = 2                          # the extra views are drawn every second frame; two on show take turns
# views by index, as in the set file and the effects (FlightGear's patch offers four)
MIRROR_VIEW, TAIL_VIEW, HOIST_VIEW = 0, 1, 2
# the tail camera: above the tail rotor, looking forward over the aircraft
TAIL_CAMERA = dict(x=6.6, y=0.0, z=1.9, heading=0.0, pitch=-14.0, roll=0.0, fov=75.0, near=0.1)
# the hoist camera: straight down, the nose at the top of the picture. The arm swung out holds the cable at about
# x -1.22, y -1.33, z 0.99 (the cable's pivot in Alouette-III-sc_jsb.xml). The camera is a little aft of that and outboard,
# below the winch block, so the cable comes in from the upper right.
HOIST_CAMERA = dict(x=-1.10, y=-1.58, z=0.98, heading=0.0, pitch=-90.0, roll=0.0, fov=75.0, near=0.05)

# ------------------------------------------------------------------------------------------------ the extra views' pipeline

DROP_PASSES = ('histogram-', 'copy-prev-luminance', 'bloom-', 'highlights', 'post', 'no-aa-display', 'fxaa', 'smaa-', 'debug-')
DROP_BUFFERS = ('histogram', 'bloom', 'highlight', 'smaa-', 'final')
NO_SHADOWS = ('csm2', 'csm3')               # the cascades from 40 m out
LOD_SCALE = 2.0                             # models change to coarser levels of detail at half the distance
SHADOW_MAP = 2048                           # the views' own sun shadow map (each cascade a quarter of it): FlightGear's own is
                                            # the user's setting (8192 by default), far more than a picture this small shows


def child(node, tag, text, **attributes):
    e = node.find(tag)
    if e is None:
        e = ET.SubElement(node, tag)
    e.text = text
    e.attrib.update(attributes)


def clear_only(p, mask, color):
    """the pass goes on clearing its buffers, which later passes read, and draws nothing into them"""
    for e in p.findall('effect') + p.findall('binding') + p.findall('memory-barrier'):
        p.remove(e)
    child(p, 'cull-mask', '0x0')
    child(p, 'clear-mask', mask)
    child(p, 'clear-color', color, type='vec4d')
    child(p, 'clear-depth', '1.0')


def economise(root, source):
    passes = {p.findtext('name', '').strip(): p for p in root.findall('pass')}
    # ambient occlusion: its result cleared to 'nothing occluded'
    root.remove(passes['gtao'])
    for b in root.findall('buffer'):
        if b.findtext('name', '').strip() == 'ao-noisy':
            root.remove(b)
    clear_only(passes['gtao-denoiser'], 'color', '1.0 1.0 1.0 1.0')
    # the 3-D clouds: cleared to 'none' (the cloud layers drawn with the scene stay)
    clear_only(passes['clouds'], 'color depth', '0.0 0.0 0.0 0.0')
    # the far shadow cascades: nothing drawn into their parts of the map, which then reads as lit
    for name in NO_SHADOWS:
        p = passes[name]
        own = {e.tag for e in p}
        for e in ET.parse(os.path.join(source, p.attrib.pop('include'))).getroot():
            if e.tag not in own:
                p.append(e)
        child(p, 'cull-mask', '0x0')
    child(passes['geometry'], 'lod-scale', '%g' % LOD_SCALE)
    # their own, small sun shadow map, whatever the size of the main view's
    atlas = next(b for b in root.findall('buffer') if b.findtext('name', '').strip() == 'sun-shadowmap-atlas')
    for side in ('width', 'height'):
        e = atlas.find(side)
        for c in list(e):
            e.remove(c)
        e.text = str(SHADOW_MAP)


def pipeline(fgdata):
    """FlightGear's HDR pipeline without what a picture for a mirror or a screen does not need: automatic exposure (it stays at
    the 0 EV it is reset to; the effect showing the texture applies the main view's), bloom, pick highlights, tone mapping,
    antialiasing and the debug passes. It ends at the demosaiced HDR picture, in the buffer render-view-output."""
    source = os.path.join(fgdata, 'Compositor', 'HDR')
    root = ET.parse(os.path.join(source, 'hdr.xml')).getroot()
    root.find('name').text = 'RenderView'
    for node in list(root):
        if node.tag in ('pass', 'buffer'):
            drop = DROP_PASSES if node.tag == 'pass' else DROP_BUFFERS
            if node.findtext('name', '').strip().startswith(drop):
                root.remove(node)
    renamed = 0
    for node in root.iter():
        if node.tag in ('buffer', 'name') and (node.text or '').strip() == 'hdr-final':
            node.text = 'render-view-output'
            renamed += 1
    assert renamed == 2, 'hdr.xml is not the file this was written for'
    economise(root, source)
    buffers = {b.findtext('name', '').strip() for b in root.findall('buffer')}
    for p in root.findall('pass'):
        for ref in p.findall('binding') + p.findall('attachment') + p.findall('image-binding'):
            assert ref.findtext('buffer', '').strip() in buffers, (p.findtext('name'), 'uses a dropped buffer')
        assert p.findtext('type') in ('compute', 'csm') or p.findall('attachment') or p.get('include') == 'csm-pass.xml', (
            p.findtext('name'), 'would draw over the window')
    shutil.rmtree(PIPELINE, ignore_errors=True)
    os.makedirs(PIPELINE)
    for node in root.iter():
        if node.get('include'):
            shutil.copy(os.path.join(source, node.get('include')), PIPELINE)
    text = ET.tostring(root, encoding='unicode')
    with open(os.path.join(PIPELINE, 'render-view.xml'), 'w') as f:
        f.write('<?xml version="1.0" encoding="utf-8"?>\n<!-- FlightGear\'s HDR pipeline (Compositor/HDR/hdr.xml, GPL-2.0-or-later, the FlightGear\n'
                '     authors) cut down for a view into a texture. Written by fdm/renderviews.py (see there). -->\n' + text.rstrip() + '\n')
    return len(root.findall('pass'))


# ------------------------------------------------------------------------------------------------ the two effects

SHADER = PREFIX + 'Shaders/HDR/render-view.frag'


def technique(n, view, texture, mode, gain):
    """used only while FlightGear reports the view built and enabled; else the next technique down, the parent effect's"""
    uniforms = [('render_view_tex', 'sampler-2d', '<value type="int">2</value>'),
                ('render_view_mode', 'int', '<value type="int">%d</value>' % mode),
                ('render_view_gain', 'float', '<value type="float">%g</value>' % gain),
                ('screen_aspect', 'float', '<value type="float">%g</value>' % (SCREEN_SIZE[1] / SCREEN_SIZE[0])),
                ('aerial_perspective_tex', 'sampler-3d', '<value type="int">11</value>'),
                ('lum_tex', 'sampler-2d', '<value type="int">14</value>'),
                ('exposure_compensation', 'float', '<value><use>exposure-compensation</use></value>'),
                ('clouds_depth_tex', 'sampler-2d', '<value type="int">15</value>')]
    body = ''.join('      <uniform>\n        <name>%s</name>\n        <type>%s</type>\n        %s\n      </uniform>\n' % u for u in uniforms)
    vertex = ('model_transparent.vert', 'logarithmic_depth.glsl', 'aerial_perspective.glsl', 'mvr.vert')
    fragment = ('math.glsl', 'logarithmic_depth.glsl', 'exposure.glsl', 'forward_pack.glsl', 'mvr.frag')
    shaders = (''.join('        <vertex-shader>Shaders/HDR/%s</vertex-shader>\n' % s for s in vertex) +
               '        <fragment-shader>%s</fragment-shader>\n' % SHADER +
               ''.join('        <fragment-shader>Shaders/HDR/%s</fragment-shader>\n' % s for s in fragment))
    return '''  <technique n="%d">
    <predicate>
      <and>
        <property>/sim/rendering/render-views/view[%d]/ready</property>
        <property>/sim/rendering/render-views/view[%d]/enabled</property>
      </and>
    </predicate>
    <scheme>hdr-forward</scheme>
    <pass>
      <depth>
        <enabled>true</enabled>
        <function>less</function>
        <write-mask>false</write-mask>
      </depth>
      <texture-unit>
        <unit>2</unit>
        <type>buffer</type>
        <buffer>%s</buffer>
      </texture-unit>
      <blend>
        <active>true</active>
        <source-rgb>src-alpha</source-rgb>
        <destination-rgb>one-minus-src-alpha</destination-rgb>
        <source-alpha>one-minus-dst-alpha</source-alpha>
        <destination-alpha>one</destination-alpha>
      </blend>
      <rendering-hint>transparent</rendering-hint>
      <cull-face><use>cull-face</use></cull-face>
      <program>
%s      </program>
%s    </pass>
  </technique>
''' % (n, view, view, texture, shaders, body)


def effect(name, parent, note, techniques):
    text = ('<?xml version="1.0" encoding="utf-8"?>\n<!-- %s Written by fdm/renderviews.py. -->\n<PropertyList>\n  <name>%s</name>\n'
            '  <inherits-from>%s</inherits-from>\n%s</PropertyList>\n') % (note, name, parent, ''.join(techniques))
    with open(os.path.join(EFFECTS, name + '.eff'), 'w') as f:
        f.write(text)


def effects():
    shutil.rmtree(EFFECTS, ignore_errors=True)
    os.makedirs(EFFECTS)
    # techniques 125 to 127 come before the parent's 128; mode: see Shaders/HDR/render-view.frag
    effect('al3-screen', 'Effects/model-transparent',
           'The cockpit screen: the hoist or tail camera while that view is on, else the rear-view mirror.',
           [technique(125, HOIST_VIEW, 'al3-hoistcam', 2, 1.0), technique(126, TAIL_VIEW, 'al3-tailcam', 2, 1.0),
            technique(127, MIRROR_VIEW, 'al3-mirror', 3, 1.0)])


# ------------------------------------------------------------------------------------------------ the mirror's camera

def unit(v):
    return v / np.linalg.norm(v)


def mirror_pane():
    """the mirror's pane: its vertices (model frame) and its faces, as the loader sees them"""
    def find(o, origin):
        here = origin + o['loc']
        if o['name'] == 'rear-view mirror-glass':
            return o['verts'] + here, o['surfs']
        for kid in o['kids']:
            found = find(kid, here)
            if found:
                return found
    v, faces = find(acread._objects(open(acread.MODEL, encoding='latin-1').read().splitlines()), np.zeros(3))
    m = np.c_[v[:, 0], -v[:, 2], v[:, 1]]               # AC3D (x, y, z) -> model (x, -z, y)
    p = math.radians(acread.PITCH)                      # and Alouette-III-sc_jsb.xml's offsets
    rotation = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    return (rotation @ m.T).T + np.array(acread.OFFSET), faces


def mirror_camera():
    """a camera at the glass, aimed along the mean of what the pane reflects from the pilot's eye, as wide as all it reflects,
    the head moved HEAD_ROOM any way (the screen, 4:3, shows that width)"""
    verts, faces = mirror_pane()
    centre = verts.mean(axis=0)
    n = np.linalg.svd(verts - centre)[2][2]             # the fitted plane's normal, towards the pilot
    if np.dot(EYE - centre, n) < 0:
        n = -n
    normals = np.zeros_like(verts)                      # the smooth normals: each face's, weighted by its area, at its corners
    for face in faces:
        corners = verts[face] - verts[face[0]]
        m = np.cross(corners, np.roll(corners, -1, axis=0)).sum(axis=0)
        m = m if np.dot(m, n) > 0 else -m
        normals[face] += m
    normals = np.array([unit(m) for m in normals])

    def reflected(eye):
        d = np.array([unit(p - eye) for p in verts])
        return d - 2 * (d * normals).sum(axis=1)[:, None] * normals

    seated = reflected(EYE)
    forward = unit(seated.mean(axis=0))
    up = unit(np.array((0.0, 0.0, 1.0)) - forward * forward[2])
    right = np.cross(forward, up)
    tan_h = 0.0
    for dy in (-HEAD_ROOM, 0.0, HEAD_ROOM):
        for dz in (-HEAD_ROOM, 0.0, HEAD_ROOM):
            r = reflected(EYE + np.array((0.0, dy, dz)))
            depth = r @ forward
            tan_h = max(tan_h, np.abs((r @ right) / depth).max())
    tan_h *= 1.04
    position = centre + n * (((verts - centre) @ n).max() + 0.01)   # just clear of the dome
    return dict(x=position[0], y=position[1], z=position[2], heading=math.degrees(math.atan2(-forward[1], -forward[0])),
                pitch=math.degrees(math.asin(forward[2])), roll=0.0, fov=2 * math.degrees(math.atan(tan_h)), near=0.03)


# ------------------------------------------------------------------------------------------------ the screen

def screen_corners(w, h, back=0.0):
    """the corners of a rectangle on the screen's plane, facing the right-hand seat: counter-clockwise from the lower left"""
    n = unit(EYE - SCREEN_CENTRE)
    up = unit(np.array((0.0, 0.0, 1.0)) - n * n[2])
    right = np.cross(-n, up)
    c = SCREEN_CENTRE - n * back
    return [c - right * w / 2 - up * h / 2, c + right * w / 2 - up * h / 2, c + right * w / 2 + up * h / 2, c - right * w / 2 + up * h / 2]


def screen():
    """a flat screen facing the right-hand seat, a dark bezel behind it; hidden unless FlightGear renders its views"""
    os.makedirs(SCREEN, exist_ok=True)

    def quad(name, w, h, back, material, texture=None):
        corners = screen_corners(w, h, back)
        verts = ''.join('%.5f %.5f %.5f\n' % (p[0], p[2], -p[1]) for p in corners)     # AC3D: (x, z, -y) of the model frame
        line = 'texture "%s"\n' % texture if texture else ''
        return ('OBJECT poly\nname "%s"\n%snumvert 4\n%snumsurf 1\nSURF 0x20\nmat %d\nrefs 4\n0 0 0\n1 1 0\n2 1 1\n3 0 1\nkids 0\n'
                % (name, line, verts, material))

    # the loader keeps texture coordinates only for an object that names a texture: a white one, which the effect never reads
    Image.new('RGBA', (8, 8), (255, 255, 255, 255)).save(os.path.join(SCREEN, 'screen.png'))
    with open(os.path.join(SCREEN, 'screen.ac'), 'w') as f:
        f.write('AC3Db\n'
                'MATERIAL "screen" rgb 1 1 1  amb 1 1 1  emis 0 0 0  spec 0 0 0  shi 10  trans 0\n'
                'MATERIAL "bezel" rgb 0.02 0.02 0.02  amb 0.02 0.02 0.02  emis 0 0 0  spec 0.1 0.1 0.1  shi 20  trans 0\n'
                'OBJECT world\nkids 2\n' + quad('al3-camera-screen', *SCREEN_SIZE, 0.0, 0, 'screen.png') +
                quad('al3-camera-screen-bezel', SCREEN_SIZE[0] + 0.016, SCREEN_SIZE[1] + 0.016, 0.002, 1))
    enabled = ''.join('          <property>/sim/rendering/render-views/view[%d]/enabled</property>\n' % v
                      for v in (MIRROR_VIEW, TAIL_VIEW, HOIST_VIEW))
    with open(os.path.join(SCREEN, 'screen.xml'), 'w') as f:
        f.write('''<?xml version="1.0" encoding="utf-8"?>
<!-- The cockpit screen: there only while FlightGear renders its views. Written by fdm/renderviews.py. -->
<PropertyList>
  <path>screen.ac</path>
  <effect>
    <inherits-from>%sModels/Effects/RenderView/al3-screen</inherits-from>
    <object-name>al3-camera-screen</object-name>
  </effect>
  <animation>
    <type>select</type>
    <object-name>al3-camera-screen</object-name>
    <object-name>al3-camera-screen-bezel</object-name>
    <condition>
      <and>
        <property>/sim/rendering/render-views/view[%d]/ready</property>
        <or>
%s        </or>
      </and>
    </condition>
  </animation>
  <animation>
    <type>pick</type>
    <object-name>al3-camera-screen</object-name>
    <action>
      <button>0</button>
      <binding>
        <command>nasal</command>
        <script>al3screen.cycle();</script>
      </binding>
    </action>
    <hovered>
      <binding>
        <command>set-tooltip</command>
        <tooltip-id>al3-camera-screen</tooltip-id>
        <label>Screen: mirror / tail camera / hoist camera</label>
      </binding>
    </hovered>
  </animation>
</PropertyList>
''' % (PREFIX, MIRROR_VIEW, enabled))


# ------------------------------------------------------------------------------------------------ the set and model files

def marked(text, name, body, anchor):
    """body between this script's markers: replaced where it already is, else put right after anchor"""
    begin = '<!--== %s: written by fdm/renderviews.py ==-->' % name
    end = '<!--== end of fdm/renderviews.py\'s %s ==-->' % name
    block = begin + '\n' + body + '    ' + end
    if begin in text:
        return text[:text.index(begin)] + block + text[text.index(end) + len(end):]
    assert text.count(anchor) == 1, ('no single place for', name, anchor)
    return text.replace(anchor, anchor + '    ' + block + '\n')


def shown_on(surfaces):
    """where a view's picture is seen: [(corners, a condition's body or '')]; FlightGear draws the view only while one is in sight"""
    xml = ''
    for corners, condition in surfaces:
        xml += '          <surface>\n' + ''.join(
            '            <point><x-m type="double">%.4f</x-m><y-m type="double">%.4f</y-m><z-m type="double">%.4f</z-m></point>\n' % tuple(c)
            for c in corners)
        if condition:
            xml += '            <condition>%s</condition>\n' % condition
        xml += '          </surface>\n'
    return '        <shown-on>\n' + xml + '        </shown-on>\n'


def view(index, name, texture, camera, width, height, enabled, surfaces):
    keys = ('x-m', 'y-m', 'z-m', 'heading-deg', 'pitch-deg', 'roll-deg', 'fov-deg', 'near-m')
    values = (camera[k] for k in ('x', 'y', 'z', 'heading', 'pitch', 'roll', 'fov', 'near'))
    body = ''.join('          <%s type="double">%.4f</%s>\n' % (k, v, k) for k, v in zip(keys, values))
    return ('      <view n="%d">\n        <name>%s</name>\n        <texture>%s</texture>\n'
            '        <compositor>%sCompositor/RenderView/render-view</compositor>\n        <width type="int">%d</width>\n'
            '        <height type="int">%d</height>\n        <enabled type="bool">%s</enabled>\n        <frame-interval type="int">%d</frame-interval>\n'
            '%s        <camera>\n%s        </camera>\n      </view>\n') % (index, name, texture, PREFIX, width, height,
                                                                         'true' if enabled else 'false', FRAME_INTERVAL, shown_on(surfaces), body)


def configure(camera):
    with open(SET, encoding='utf-8', errors='surrogateescape', newline='') as f:
        s = f.read()
    screen_surface = screen_corners(*SCREEN_SIZE)
    # the mirror's picture is seen on the screen unless a camera has taken the screen
    the_mirror = [(screen_surface, '<not><or>' + ''.join(
        '<property>/sim/rendering/render-views/view[%d]/enabled</property>' % v for v in (TAIL_VIEW, HOIST_VIEW)) + '</or></not>')]
    views = ('    <!-- views rendered into textures: only a FlightGear with the render-view patch reads these -->\n'
             '    <rendering>\n      <render-views>\n' +
             view(MIRROR_VIEW, 'rear-view mirror', 'al3-mirror', camera, *CAMERA_SIZE, True, the_mirror) +
             view(TAIL_VIEW, 'tail camera', 'al3-tailcam', TAIL_CAMERA, *CAMERA_SIZE, False, [(screen_surface, '')]) +
             view(HOIST_VIEW, 'hoist camera', 'al3-hoistcam', HOIST_CAMERA, *CAMERA_SIZE, False, [(screen_surface, '')]) +
             '      </render-views>\n    </rendering>\n')
    s = marked(s, 'render views', views, '\n <sim>\n')
    s = marked(s, 'screen script', '    <al3screen>\n        <file>%sNasal/render_screen.nas</file>\n    </al3screen>\n' % PREFIX,
               '\n<nasal>\n')
    with open(SET, 'w', encoding='utf-8', errors='surrogateescape', newline='') as f:
        f.write(s)
    with open(MAIN, encoding='latin-1', newline='') as f:
        s = f.read()
    s = marked(s, 'screen model', '    <model>\n        <name>camera-screen</name>\n        <path>%sModels/RenderView/screen.xml</path>\n'
               '    </model>\n' % PREFIX, '<!--== end of fdm/hdrmodel.py\'s glass ==-->\n')
    with open(MAIN, 'w', encoding='latin-1', newline='') as f:
        f.write(s)


def main():
    if len(sys.argv) != 2 or not os.path.isfile(os.path.join(sys.argv[1], 'Compositor', 'HDR', 'hdr.xml')):
        sys.exit('usage: python3 renderviews.py FGDATA   (FlightGear\'s data directory)')
    passes = pipeline(sys.argv[1])
    effects()
    screen()
    camera = mirror_camera()
    configure(camera)
    print('render views: pipeline of %d passes; mirror camera at (%.3f, %.3f, %.3f), heading %.1f, pitch %.1f, %.1f deg wide, %dx%d' % (
        passes, camera['x'], camera['y'], camera['z'], camera['heading'], camera['pitch'], camera['fov'], *CAMERA_SIZE))


if __name__ == '__main__':
    main()
