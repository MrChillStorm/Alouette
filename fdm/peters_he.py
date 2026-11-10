"""Peters-He generalized dynamic wake: shape functions, Gamma and L matrices (closed form as summarised in the
METU thesis eqs. 2.17-2.28, after Peters & He 1991/1995)."""
import numpy as np
from math import sqrt, pi

def df(n):
    """double factorial with n!! = 1 for n = 0, -1 and -1 for n = -3"""
    if n in (0, -1): return 1.0
    if n == -3: return -1.0
    r = 1.0
    while n > 1: r *= n; n -= 2
    return r

def H(j, r):
    return df(j + r - 1) * df(j - r - 1) / (df(j + r) * df(j - r))

def phi(j, r, rb):
    s = 0.0
    for q in range(r, j, 2):
        s = s + rb ** q * (-1) ** ((q - r) // 2) * df(j + q) / (df(q - r) * df(q + r) * df(j - q - 1))
    return sqrt((2 * j + 1) * H(j, r)) * s

def Gamma(j, r, n, m):
    """row: inflow state (j, r); column: pressure (n, m)"""
    if (r + m) % 2 == 0:
        return (-1) ** ((n + j - 2 * r) // 2) * 2 * sqrt((2 * n + 1) * (2 * j + 1)) / (
            sqrt(H(n, m) * H(j, r)) * (j + n) * (j + n + 2) * ((j - n) ** 2 - 1))
    if abs(j - n) == 1:
        return pi * np.sign(r - m) / (2 * sqrt(H(n, m) * H(j, r)) * sqrt((2 * n + 1) * (2 * j + 1)))
    return 0.0

def states(mmax, jmax_by_m):
    """list of (j, r) for the cosine set (r = 0..mmax) and the sine set (r >= 1)"""
    cs = [(j, r) for r in range(mmax + 1) for j in range(r + 1, jmax_by_m[r] + 1, 2)]
    ss = [(j, r) for (j, r) in cs if r > 0]
    return cs, ss

def Lmat(st, X, kind):
    N = len(st); L = np.zeros((N, N))
    for a, (j, r) in enumerate(st):
        for b, (n, m) in enumerate(st):
            G = Gamma(j, r, n, m)
            if r == 0:
                f = X ** m
            else:
                l = min(r, m)
                f = X ** abs(m - r) + (1 if kind == 'c' else -1) * (-1) ** l * X ** abs(m + r)
            L[a, b] = f * G
    return L
