"""The SA 316B Alouette III's published cruise speed and fuel consumption, digitized for cruisecharts.py.

  [SPEC]  Aerospatiale, "SA 316 B Alouette III, multipurpose helicopter: Type Specifications", issue of January 1,
          1973, appendix I, the performance curves "drawn from Appendix I of the Flight Manual": the cruising speed
          against density altitude (fig. 3-1, p. 1.4: indicated, solid, and true, dashed) and the fuel consumption per
          kilometre in cruising flight, standard atmosphere, against pressure altitude (fig. 3-3, p. 1.8). A public
          scan: https://www.alouettelama.com/pdf/316B-SPEC-19730101.pdf (not in this repository)

Both figures print one collective pitch scale beside the altitude, the same for every weight: 0.80 up to 1000 m, then
0.05 more for each 1000 m. In the standard atmosphere pressure and density altitude are the same, so a fuel consumption
times the true airspeed read at the same weight and altitude is the cruise's fuel flow. The figures give no engine
power, and nothing about the engine's fuel consumption against power.

Reading: the pages rendered at 300 dpi; the grid lines fitted as straight lines where the scan draws them (it is turned
slightly and its lines are not evenly spaced); each curve's centre traced row by row, the thin lines taken out by a
morphological opening, and fitted with a quadratic within 300 m of the altitude read. Two checks of the readings: the
dashed true airspeeds agree within 1 % with the solid indicated ones divided by the square root of the density ratio
(cruisecharts.py checks this), and at sea level the four weights' fuel flows agree within 1 %. Take a fuel flow as
about +-1 %.
"""

SOURCE = ('Aerospatiale, SA 316 B Alouette III Type Specifications, 1 January 1973, appendix I (figs. 3-1 and 3-3 of '
          'the SA 316 B flight manual)')

# kg: {altitude m: (true airspeed km/h (fig. 3-1, dashed), indicated airspeed km/h (fig. 3-1, solid),
#                   fuel consumption litre/km (fig. 3-3))}; 2200 kg's curves end just above 3000 m
CRUISE = {
    1600: {0: (202.4, 202.4, 1.111), 1000: (208.0, 197.9, 0.968), 2000: (208.6, 190.6, 0.873),
           3000: (207.0, 179.3, 0.842), 4000: (200.4, 163.2, 0.858)},
    1800: {0: (197.7, 197.6, 1.139), 1000: (199.8, 190.4, 1.006), 2000: (200.8, 180.5, 0.924),
           3000: (193.4, 166.6, 0.907), 4000: (181.5, 147.5, 0.961)},
    2000: {0: (192.0, 192.3, 1.173), 1000: (191.0, 181.3, 1.060), 2000: (184.7, 167.9, 1.010),
           3000: (172.0, 149.1, 1.049), 4000: (150.2, 121.9, 1.196)},
    2200: {0: (185.9, 185.9, 1.217), 1000: (177.6, 169.8, 1.143), 2000: (164.4, 149.6, 1.155),
           3000: (141.2, 121.0, 1.288)},
}


def pitch(alt_m):
    """the collective pitch indicator's reading the figures print beside the altitude"""
    return 0.80 + 0.05 * max(0.0, alt_m - 1000.0) / 1000.0


def fuel_flow_lh(kg, alt_m):
    tas, _, per_km = CRUISE[kg][alt_m]
    return tas * per_km
