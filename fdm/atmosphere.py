"""The International Standard Atmosphere below 11 km."""
def isa(h_m, dT=0.0):
    """(T K, p Pa, rho kg/m^3) at geopotential height h_m, temperature ISA + dT"""
    T0 = 288.15 - 0.0065 * h_m
    P = 101325.0 * (T0 / 288.15) ** 5.25588
    T = T0 + dT
    return T, P, P / (287.053 * T)
