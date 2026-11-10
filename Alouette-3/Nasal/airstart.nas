# Starting in the air: FlightGear puts the aircraft wings level, but the Alouette flies with its right side down (the
# tail rotor pushes the tail to the left, and the main rotor's thrust leans right to balance it). Held level, it would
# slide sideways the moment the flight model lets it go (Systems/al3-cockpit.xml holds it for the first seconds while
# it trims the controls), so the start's bank is the flight model's trimmed bank at the start's airspeed.

var BANK_KT  = [0, 30, 60, 90, 105];
var BANK_DEG = [4.1, 2.6, 2.2, 3.0, 3.8];

var trimmed_bank = func(kt) {
    if (kt <= BANK_KT[0]) return BANK_DEG[0];
    for (var i = 1; i < size(BANK_KT); i += 1)
        if (kt <= BANK_KT[i])
            return BANK_DEG[i - 1] + (BANK_DEG[i] - BANK_DEG[i - 1]) * (kt - BANK_KT[i - 1]) / (BANK_KT[i] - BANK_KT[i - 1]);
    return BANK_DEG[size(BANK_DEG) - 1];
}

var ours = 0;     # the preset's bank is this file's: cleared again for a start on the ground

var set_bank = func {
    if (getprop("/sim/presets/onground")) {
        if (ours) setprop("/sim/presets/roll-deg", 0);
        ours = 0;
        return;
    }
    ours = 1;
    var phi = trimmed_bank(getprop("/sim/presets/airspeed-kt") or 0);
    setprop("/sim/presets/roll-deg", phi);
    setprop("/orientation/roll-deg", phi);
}

set_bank();
setlistener("/sim/presets/airspeed-kt", set_bank, 0, 0);
setlistener("/sim/presets/onground", set_bank, 0, 0);
