"""The SA 316B Alouette III's published hover performance, in and out of ground effect, digitized for hovercharts.py.

  [SPEC]  Aerospatiale, "SA 316 B Alouette III, multipurpose helicopter: Type Specifications", issue of January 1,
          1973, appendix I: the main performance data (p. 1.2) and the performance curves "drawn from Appendix I of the
          Flight Manual": the hover ceilings in and out of ground effect against weight and density altitude (fig. 3-5,
          p. 1.10) and against pressure altitude and outside temperature (fig. 3-6 out of ground effect, p. 1.12; fig.
          3-7 in it, p. 1.14). A public scan: https://www.alouettelama.com/pdf/316B-SPEC-19730101.pdf (not in this
          repository)

Figs. 3-6 and 3-7 are headed "Pitch law increased by 0.03 (power used: 390 kW approx.)": in ground effect "with pitch
reserve relative to engine limitations", out of it "without pitch reserve relative to engine limitations". Their areas:
1 the main gearbox's power limit, 2 the pitch limit (1.0 on the collective pitch indicator), 3 the maximum thermal
power. So in area 1, below fig. 3-5's knee, a hover ceiling in ground effect and one out of it are flown on the same
power, and the weight between them is the ground effect. The document gives no wheel or skid height for "in ground
effect", and does not say what the pitch reserve is (if it keeps some power back in ground effect, the ground effect
is stronger than read here).

Reading: the pages rendered at 300 dpi; the curves separated from the grid by a morphological opening; pixels turned into
chart values on the grid lines as drawn (the scan is turned by 0.3 deg and its lines are not evenly spaced). Smooth fits
leave about 1 kg rms on fig. 3-5 and 5-10 m rms on figs. 3-6 and 3-7. The three figures and the table agree to about
60 m and 10 kg, but for fig. 3-5's 1700 kg in ground effect, about 80 m below the table: take a single reading as
+-10 kg (+-0.5 % on a weight).
"""

SOURCE = ('Aerospatiale, SA 316 B Alouette III Type Specifications, 1 January 1973, appendix I (figs. 3-5, 3-6, 3-7 of '
          'the SA 316 B flight manual; table p. 1.2)')

POWER_USED_KW = 390.0          # "power used: 390 kW approx." (figs. 3-6, 3-7)

# the main performance data (p. 1.2): hover ceilings, m (standard atmosphere): kg: (in ground effect, out of it)
TABLE = {1700: (5550.0, 4900.0), 1900: (4500.0, 3400.0), 2100: (2750.0, 1300.0), 2200: (1650.0, 100.0)}

# fig. 3-5: density altitude m, weight out of ground effect kg, weight in it kg. Area 1 (the gearbox's power limit):
FIG35_AREA1 = [(1650, 2074, 2200), (2000, 2044, 2167), (2500, 1998, 2120), (3000, 1950, 2071), (3500, 1899, 2020),
               (4000, 1847, 1966)]
# below 1650 m the weight in ground effect is above the chart's 2200 kg: out of ground effect only
FIG35_OGE_LOW = [(100, 2190), (500, 2163), (1000, 2127)]
# above the knee, about 4300 m: area 2, both on the pitch limit
FIG35_AREA2 = [(4900, 1706, 1810), (5500, 1602, 1693), (6000, 1518, 1601)]

# figs. 3-6 and 3-7: each weight's ceiling in area 1 as pressure altitude (m) = a T^2 + b T + c over the outside
# temperature T (deg C) from Tmin to Tmax: kg: (a, b, c, Tmin, Tmax)
FIG36_OGE = {2200: (0.0160, -28.10, 457.6, -50, 15), 2100: (0.0006, -28.30, 1469.6, -50, 50),
             2000: (0.0045, -28.39, 2417.8, -50, 45), 1900: (0.0001, -28.04, 3198.7, -50, 35),
             1800: (0.0002, -27.40, 3984.6, -50, 20)}
FIG37_IGE = {2200: (0.0007, -29.56, 1766.2, -50, 50), 2100: (0.0067, -28.90, 2662.7, -50, 30),
             2000: (0.0081, -28.16, 3477.1, -50, 15), 1900: (-0.0031, -28.67, 4076.5, -50, -2)}

# check points off the standard atmosphere, within area 1 on both charts: (outside temperature C, pressure altitude m)
OFF_ISA_POINTS = [(-20, 2500), (-20, 3500), (0, 2000), (0, 3000), (20, 1500), (20, 2000)]


def chart_weight(lines, temp_c, pa_m):
    """the weight (kg) whose ceiling is at pa_m at temp_c: linear in weight between the two lines either side"""
    at = sorted((a * temp_c ** 2 + b * temp_c + c, kg) for kg, (a, b, c, t0, t1) in lines.items() if t0 <= temp_c <= t1)
    for (h0, w0), (h1, w1) in zip(at, at[1:]):
        if h0 <= pa_m <= h1:
            return w0 + (w1 - w0) * (pa_m - h0) / (h1 - h0)
    raise ValueError('no lines either side of %g m at %g C' % (pa_m, temp_c))
