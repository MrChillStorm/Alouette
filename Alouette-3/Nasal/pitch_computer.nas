# The collective pitch indicator's computer (Instruments/Afficheur.xml), as the real unit works it: its mobile ring is turned
# until the outside temperature on its scale meets the pressure altitude on the fixed ring's scale, and the outer pitch scale
# on the mobile ring then shows, against the fixed weight scale, the pitch each weight needs (and the weight each pitch
# carries) in the hover; the reminder triangle is set to the density altitude on the "Z km" arc, which puts it at the
# highest pitch the aircraft may be pulled to. The pilot sets it by hand; here it is set from the environment.
#
# Everything is taken from the real unit and the flight manual, none of it from this aircraft's flight model:
#  - The scales are logarithmic, 300 degrees a decade (the log of the pressure for the altitude scale, of the absolute
#    temperature for the temperature scale, of the weight for the weight scale). Their angles, measured on the dial's
#    artwork (ALIII_AfficheurN0/N1/N2/3.png), follow those formulas to 0.8 degree.
#  - The mobile ring turns by (the angle of the pressure altitude on the altitude scale) less (the angle of the temperature
#    on its scale). In the standard atmosphere this puts the ring where the density altitude equals the pressure
#    altitude; away from it the turn is K ln(sigma) from there: the ring measures the density, as the Z scale's graduation
#    (a density altitude of the standard atmosphere) shows. The density altitude is that of the International Standard
#    Atmosphere, the one the manual's charts are drawn in.
#  - The highest pitch is a function of the density altitude, read off the dial's "Z km" arc: .80 at 0, .85 at 1 km, .90 at 2,
#    .95 at 3, 1 at 4.7 km (the manual's collective pitch scale, beside its figures 3-2 and 3-3, agrees to 0.05 km).
#  - The weight the dial shows at a pitch is read on the weight scale where the pitch scale (moved by the ring) lands.
#    In the standard atmosphere and at that highest pitch it agrees with the manual's hover ceiling in ground effect (figure
#    3-5): 2189 against 2186 kg at 1730 m, 1500 against 1506 kg at 6470 m, 1859 against 1920 kg at 4430 m, where the
#    manual's curve has its knee.
# If the aircraft's rotor then needs more or less pitch than the real one for a weight, it is the aircraft that is wrong.

var PC = "/instrumentation/pitch-computer/";
var K = 300 / math.ln(10);                      # degrees for each e-fold of the scales

# the measured scales (angles from straight down, clockwise on the dial)
var PITCH = [0.40, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 1.00];
var PITCH_PSI = [108.57, 124.20, 138.50, 151.15, 163.42, 173.62, 183.60, 192.95, 201.72, 209.90, 217.60, 224.92, 231.65];
var NEEDLE_S = [0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.00];        # the needle's angle at each printed value, as in the dial's table
var NEEDLE_DEG = [70, 93, 116, 143, 172, 201, 234];
var ZKM = [0, 1, 2, 3, 4.7];                                      # the "Z km" arc
var ZPITCH = [0.80, 0.85, 0.90, 0.95, 1.00];

var interp = func(x, xs, ys) {
    var n = size(xs);
    if (x <= xs[0]) return ys[0];
    if (x >= xs[n - 1]) return ys[n - 1];
    for (var i = 1; i < n; i += 1) {
        if (x <= xs[i]) return ys[i - 1] + (ys[i] - ys[i - 1]) * (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
    }
    return ys[n - 1];
};

# the International Standard Atmosphere, troposphere
var p_ratio = func(h_m) { return math.pow(1 - 0.0065 * h_m / 288.15, 5.25588); };
var pressure_altitude_m = func(p_hpa) { return 44330.77 * (1 - math.pow(p_hpa / 1013.25, 0.190263)); };
var density_altitude_m = func(sigma) { return 288.15 / 0.0065 * (1 - math.pow(sigma, 1 / 4.25588)); };

var psi_altitude = func(h_m) { return 88.40 + K * math.ln(p_ratio(h_m)); };               # 0 km at 88.4 degrees, 6 km at -11
var psi_temperature = func(t_c) { return 53.20 + K * math.ln((t_c + 273.15) / 273.15); };   # 0 deg C at 53.2 degrees
var WEIGHT_PSI_1000 = 119.62;                                                              # the weight scale: 1 000 kg at 119.62

var update = func {
    var t_c = getprop("/environment/temperature-degc");
    var p_inhg = getprop("/environment/pressure-inhg");
    if (t_c == nil or p_inhg == nil or p_inhg < 5) return;
    var h = pressure_altitude_m(p_inhg * 33.8639);
    var sigma = p_ratio(h) * 288.15 / (t_c + 273.15);
    var z = density_altitude_m(sigma);
    var ring = psi_altitude(h) - psi_temperature(t_c);                # how far the mobile ring is turned, clockwise
    var s_max = interp(z / 1000, ZKM, ZPITCH);
    var w_max = 1000 * math.exp((interp(s_max, PITCH, PITCH_PSI) + ring - WEIGHT_PSI_1000) / K);
    setprop(PC ~ "pressure-altitude-m", h);
    setprop(PC ~ "density-altitude-m", z);
    setprop(PC ~ "ring-deg", ring);
    setprop(PC ~ "max-pitch", s_max);
    setprop(PC ~ "max-pitch-deg", interp(s_max, NEEDLE_S, NEEDLE_DEG));   # where the reminder triangle stands
    setprop(PC ~ "max-hover-weight-kg", w_max);
};

var _timer = maketimer(1.0, update);
setlistener("/sim/signals/fdm-initialized", func {
    update();
    _timer.start();
}, 0, 0);
