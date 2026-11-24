var helper  = screen.display.new(10, 20);

var display_helper = func {
helper.setcolor(0.4,1.0,0.0);

helper.add("fdm/jsbsim/fcs/x-col");

helper.add("fdm/jsbsim/fcs/x-lon");
helper.add("fdm/jsbsim/propulsion/engine/longitudinal-ctrl-rad");

helper.add("fdm/jsbsim/fcs/x-lat");
helper.add("fdm/jsbsim/propulsion/engine/lateral-ctrl-rad");

helper.add("fdm/jsbsim/fcs/x-ped");
helper.add("fdm/jsbsim/propulsion/engine[1]/antitorque-ctrl-rad");

helper.add("fdm/jsbsim/propulsion/engine[0]/a1-rad");
helper.add("fdm/jsbsim/propulsion/engine[0]/b1-rad");
}

# The autostart, paced as a recorded start in an SA 316B's cockpit (ZU-REC): the starter with the fuel flow lever closed;
# light-off about 10 s on and the lever to its idle position; idle (16 330 rpm) held 54 s; the lever forward to where the
# spool holds at 77-80 % while the clutch slips and the rotor runs up (about 45 s); the rotor caught up, about 10 s more,
# then the lever to its stop over some 18 s, spool and rotor rising together to governed. Each step waits for the flight
# model's own state (lit, at idle, the clutch engaged), so the start goes at the model's pace; the waits between them are
# the recording's. Running it again starts over.
# The flight controls are held neutral for the whole start, as in the cockpit: the collective at the bottom, the cyclic
# and the pedals centred. The clutch has little to spare while it slips (the spool holds at 80 % and its torque is
# about 1 480 ft lb), and every load on the rotor takes from it: the main rotor at its collective's lowest, the tail
# rotor through its gearbox (5.5 times the rotor's speed) and the cyclic's drag. Controllers with no centring rest where
# they were left (a pedal axis at half travel put the tail rotor at 14.7 degrees, the tail rotor then taking a third of
# the clutch's torque, and the rotor stopped at 68 % while the spool ran at 80 %); a throttle axis sets the collective
# whenever it sends a value (with Invert collective axis on, a lever resting at its back end is collective nearly up).
# So the properties are set to neutral every 0.25 s until the start is done; an axis takes its own again at its next
# movement.
var START_LEVER_IDLE = 0.35;        # the lever's idle position (fdm/artouste.py LEVER_IDLE)
var START_IDLE_HOLD = 50.0;         # s at idle before the lever goes forward (the recording's 54)
var START_LEVER_SLIP = 0.74;        # the governor's reference about 26 600 rpm: the recording's 77-80 % while the clutch slips
var START_LOCKED_HOLD = 10.0;       # s with the rotor caught up before the last of the lever
var START_FINAL_S = 18.0;           # s for the lever to its stop
var _start = { stage: 0, t: 0.0, timer: nil, hold: nil, msg: nil };    # hold: the timer that keeps the controls neutral

var _start_say = func(text) {
    if (_start.msg == nil) _start.msg = screen.window.new(nil, -210, 1, 15);
    _start.msg.write(text, 0.5, 1, 0.9);
};

var _start_hold_controls = func {
    if ((getprop("/controls/al3/collective") or 0) != 0) setprop("/controls/al3/collective", 0);
    foreach (var axis; ["rudder", "aileron", "elevator"]) {
        var path = "/controls/flight/" ~ axis;
        if (math.abs(getprop(path) or 0) > 0.0001) setprop(path, 0);
    }
};

var _start_step = func {
    _start_hold_controls();
    var now = getprop("/sim/time/elapsed-sec");
    var n = getprop("fdm/jsbsim/eng/n-pct") or 0;
    var lit = getprop("fdm/jsbsim/eng/lit") or 0;
    var engaged = getprop("fdm/jsbsim/eng/engaged") or 0;
    if (_start.stage == 1 and now - _start.t > 3.0) {
        setprop("fdm/jsbsim/animation/master-switch-cmd", 1);       # the starter
        _start.stage = 2;
    } elsif (_start.stage == 2 and lit > 0.5 and n > 14.0) {
        interpolate("fdm/jsbsim/fcs/nominal-flow-cmd", START_LEVER_IDLE, 2.0);
        _start.stage = 3;
    } elsif (_start.stage == 3 and n >= 47.5) {
        _start.t = now;
        _start.stage = 4;
        _start_say("Engine at idle: warming up before the rotor is engaged.");
    } elsif (_start.stage == 4 and now - _start.t >= START_IDLE_HOLD) {
        interpolate("fdm/jsbsim/fcs/nominal-flow-cmd", START_LEVER_SLIP, 6.0);
        _start.stage = 5;
        _start_say("Fuel flow lever forward: the clutch takes up the rotor.");
    } elsif (_start.stage == 5 and engaged > 0.5) {
        _start.t = now;
        _start.stage = 6;
    } elsif (_start.stage == 6 and now - _start.t >= START_LOCKED_HOLD) {
        interpolate("fdm/jsbsim/fcs/nominal-flow-cmd", 1.0, START_FINAL_S);
        _start.t = now;
        _start.stage = 7;
    } elsif (_start.stage == 7 and now - _start.t >= START_FINAL_S + 1.0) {
        _start.timer.stop();
        _start.hold.stop();
        _start.stage = 0;
        _start_say("Rotor at governed speed.");
    }
};

var init_Run = func {
    if (_start.timer != nil) _start.timer.stop();
    if (_start.hold != nil) _start.hold.stop();
    setprop("/fdm/jsbsim/animation/cockpit-left-door-pos-cmd", 0);
    setprop("/fdm/jsbsim/animation/cockpit-right-door-pos-cmd", 0);
    setprop("fdm/jsbsim/electrical/switch/battery", 1);
    setprop("fdm/jsbsim/electrical/switch/alternator", 1);
    setprop("fdm/jsbsim/animation/rotor_brake-switch-lock-cmd", 0);
    setprop("fdm/jsbsim/animation/rotor_brake-switch-cmd", 0);
    setprop("fdm/jsbsim/animation/rotor_clutch-switch-cmd", 0);
    setprop("fdm/jsbsim/propulsion/engine[0]/cutoff", 0);
    setprop("fdm/jsbsim/fcs/nominal-flow-cmd", 0);                  # closed for the start
    setprop("fdm/jsbsim/animation/master-switch-cmd", 0);           # the starter takes a fresh press
    _start_hold_controls();
    _start.stage = 1;
    _start.t = getprop("/sim/time/elapsed-sec");
    _start.timer = maketimer(0.25, _start_step);
    _start.timer.start();
    _start.hold = maketimer(0.05, _start_hold_controls);            # 20 times a second: an axis that jitters gets only a moment
    _start.hold.start();
    _start_say("Autostart: battery, alternator, rotor brake off, controls neutral; starter.");
};

#display_helper();



close_helper = func { helper.close(); }


# flex rope animation


#setlistener("/sim/signals/fdm-initialized",     func {rescue.rope_animation(0); }    );



# the rotor's animation is the flight model's own, blade by blade (Systems/al3-cockpit.xml): the old time_rotate loop,
# which advanced an animation azimuth here, is taken out
