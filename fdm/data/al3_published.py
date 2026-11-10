"""The SA 316B Alouette III's published performance, for the report's comparisons (sea level, standard atmosphere,
the masses given). Sources as in al3data.py:

  [TD]  This Day in Aviation, "Sud-Aviation SA 316B Alouette III", after Aerospatiale's data
  [AS]  Aérostories, "SE 3160 Alouette III" (refs/aerostories), its specification table: the Artouste III B version
        (SA 316B), against the Astazou XIV (SA 319B)
  [AL]  ALAT, Alouette III fact sheet (refs/ALAT)
  [VP]  Vertipedia (VFS), Sud SA.316B Alouette III
  [TC]  EASA TCDS R.123: VNE, rotor speed limits
  [LM]  SA 315B Lama, a flight simulator add-on's manual after the flight manual: the clutch's engagement
  [RW]  ROTORwrench, JustHelicopters, November 2017: the Lama's Artouste IIIB1 idle
  [AO]  R. W. Sweazey, "Pilot Flight Check: Alouette III", The AOPA Pilot, October 1972 (an SE 3160): autorotation
        at about 1700 ft/min; idle about 15 000 rpm, the clutch engaging as the engine passes 19 000 rpm; cruise 125
        mph indicated with four aboard and full fuel, at 80 % on the collective pitch indicator
"""
KMH_KT = 1 / 1.852

# speeds, km/h at 2200 kg, sea level
CRUISE_KMH = {'TD': 185.0, 'AS': 180.0, 'AL': 195.0}
VMAX_KMH = {'TD': 210.0, 'AS': 210.0, 'AL': 210.0}         # VNE at sea level [TC]
# rate of climb, m/s (AS: "en vol oblique", in forward flight)
CLIMB_MS = {'TD-2200': 4.3, 'TD-1750': 8.7, 'VP': 846 * 0.3048 / 60, 'AS': 5.5}
# hover ceilings, m (standard day), in ground effect (IGE) and out of it (OGE)
HOVER_CEILING_M = {('TD', 2200, 'ige'): 1650.0, ('TD', 2200, 'oge'): 100.0, ('TD', 1750, 'ige'): 5550.0, ('TD', 1750, 'oge'): 1650.0,
                   ('AS', 2200, 'ige'): 2850.0, ('AS', 2200, 'oge'): 1100.0}
SERVICE_CEILING_M = {'TD': 6000.0, 'AS': 3300.0, 'AL': 6500.0}
RANGE_KM = {'TD': 470.0, 'AS': 540.0}
ENDURANCE_H = {'AS': 3 + 16 / 60}
FUEL_L = {'AS': 560.0}
# rotor speed [TD, TC]: 353.2 rpm at 100 %; in autorotation 270-420 rpm
NR_RPM, NR_AUTO_MIN, NR_AUTO_MAX = 353.2, 270.0, 420.0
# the start: idle about 15 000 rpm [AO] (an SE 3160), 17 000-18 000 rpm [RW] (the Lama's engine); the rotor starts
# turning at 19 500-21 000 rpm [LM], the clutch engaging from 19 000 [AO]
START_IDLE_RPM = (15000.0, 18000.0)
START_CLUTCH_RPM = (19500.0, 21000.0)
AUTOROTATION_FPM = {'AO': 1700.0}
