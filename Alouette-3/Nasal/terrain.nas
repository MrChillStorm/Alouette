# The terrain under the aircraft, for the flight model (Systems/landing-gear.xml reads the friction and the rolling friction,
# Models/Alouette-III-sc_jsb.xml the solid flag) and for the rotor's dust (effect_manager.nas reads the material from
# /sim/al3/terrain-material, so the ground is asked about once, here). Probed ten times a second; a value is written only when it
# has changed, through node references.

var min_carrier_alt = 2;

# Do terrain modelling ourselves.
setprop("sim/fdm/surface/override-level", 1);

var _lat = props.globals.getNode("position/latitude-deg", 1);
var _lon = props.globals.getNode("position/longitude-deg", 1);
var _material = props.globals.getNode("sim/al3/terrain-material", 1);
var _env = props.globals.getNode("fdm/jsbsim/environment", 1);
var _out = {};                                      # the flight model's nodes by name
foreach (var name; ["terrain-hight", "terrain-undefined", "terrain-solid", "terrain-oncarrier", "terrain-light-coverage",
                    "terrain-load-resistance", "terrain-friction-factor", "terrain-bumpiness", "terrain-rolling-friction",
                    "terrain-names"])
    _out[name] = _env.getNode(name, 1);

# compared with the property itself, not with the last value written: the flight model's reset puts its own values back
var _put = func (name, value) {
    var node = _out[name];
    if (node.getValue() != value) node.setValue(value);
};

var terrain_survol = func {
    var info = geodinfo(_lat.getValue(), _lon.getValue());
    var material = "";

    if (info != nil) {
        if (info[0] != nil)
            _put("terrain-hight", info[0]);
        var m = info[1];
        if (m != nil) {
            if (m.solid != nil) {
                _put("terrain-undefined", 0);
                _put("terrain-solid", m.solid);
            }
            if (m.light_coverage != nil) _put("terrain-light-coverage", m.light_coverage);
            if (m.load_resistance != nil) _put("terrain-load-resistance", m.load_resistance);
            if (m.friction_factor != nil) _put("terrain-friction-factor", m.friction_factor);
            if (m.bumpiness != nil) _put("terrain-bumpiness", m.bumpiness);
            if (m.rolling_friction != nil) _put("terrain-rolling-friction", m.rolling_friction);
            if (m.names != nil and size(m.names) > 0) {
                material = m.names[0];
                _put("terrain-names", material);
            }
            # (on a carrier info[1] is nil and only info[0] is valid)
        } else {
            _put("terrain-undefined", 1);
        }
    } else {
        _put("terrain-hight", 0);
        _put("terrain-solid", 1);
        _put("terrain-oncarrier", 0);
        _put("terrain-light-coverage", 1);
        _put("terrain-load-resistance", 1e+30);
        _put("terrain-friction-factor", 1);
        _put("terrain-bumpiness", 0);
        _put("terrain-rolling-friction", 0.02);
        _put("terrain-names", "unknown");
    }

    if (_material.getValue() != material) _material.setValue(material);
};

var _timer = maketimer(0.1, terrain_survol);
_timer.simulatedTime = 1;               # as settimer: it waits while the simulation does
terrain_survol();
_timer.start();
