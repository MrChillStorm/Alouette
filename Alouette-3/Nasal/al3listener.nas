# The listener's direction from the aircraft, for the sound's outside loops (Sounds/al3-sound.xml, written by
# fdm/al3sound.py): PyWopWop rendered the rotors from ahead and to the right (45 deg), abeam (90) and behind (180), and
# a rotor's sound differs with the direction (the tail rotor's tones nearly vanish on its axis, abeam), so the sound
# cross-fades them by where the viewer is. Written ten times a second outside the cockpit, and only when it has moved:
#   /sim/sound/al3/listener-az-deg   the viewer's bearing from the aircraft less its heading, 0 ahead, 90 abeam,
#                                    180 behind, the two sides alike
#   /sim/sound/al3/listener-el-deg   the viewer's elevation seen from the aircraft, + above
var _internal = props.globals.getNode("/sim/current-view/internal", 1);
var _heading = props.globals.getNode("/orientation/heading-deg", 1);
var _az = props.globals.getNode("/sim/sound/al3/listener-az-deg", 1);
var _el = props.globals.getNode("/sim/sound/al3/listener-el-deg", 1);

var _put = func (node, value) {
    if (math.abs(node.getValue() - value) > 0.01) node.setValue(value);
};

var _listener = func {
    if (_internal.getValue()) return;
    var ac = geo.aircraft_position();
    var vw = geo.viewer_position();
    var d = ac.distance_to(vw);
    if (d < 0.5) return;
    var az = geo.normdeg180(ac.course_to(vw) - (_heading.getValue() or 0));
    _put(_az, math.abs(az));
    _put(_el, math.atan2(vw.alt() - ac.alt(), d) * R2D);
};

_az.setValue(90.0);
_el.setValue(0.0);
var _listener_timer = maketimer(0.1, _listener);
_listener_timer.start();
