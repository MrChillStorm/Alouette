# The listener's direction from the aircraft, for the sound's outside loops (Sounds/al3-sound.xml, written by
# fdm/al3sound.py): PyWopWop rendered the rotors from ahead and to the right (45 deg), abeam (90) and behind (180), and
# a rotor's sound differs with the direction (the tail rotor's tones nearly vanish on its axis, abeam), so the sound
# cross-fades them by where the viewer is. Written ten times a second outside the cockpit:
#   /sim/sound/al3/listener-az-deg   the viewer's bearing from the aircraft less its heading, 0 ahead, 90 abeam,
#                                    180 behind, the two sides alike
#   /sim/sound/al3/listener-el-deg   the viewer's elevation seen from the aircraft, + above
var _listener = func {
    if (getprop("/sim/current-view/internal")) return;
    var ac = geo.aircraft_position();
    var vw = geo.viewer_position();
    var d = ac.distance_to(vw);
    if (d < 0.5) return;
    var az = geo.normdeg180(ac.course_to(vw) - (getprop("/orientation/heading-deg") or 0));
    setprop("/sim/sound/al3/listener-az-deg", math.abs(az));
    setprop("/sim/sound/al3/listener-el-deg", math.atan2(vw.alt() - ac.alt(), d) * R2D);
};

setprop("/sim/sound/al3/listener-az-deg", 90.0);
setprop("/sim/sound/al3/listener-el-deg", 0.0);
var _listener_timer = maketimer(0.1, _listener);
_listener_timer.start();
