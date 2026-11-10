"""A small AC3D reader for measuring the FlightGear model: objects with their world-space triangles in the
aircraft's YASim frame (x forward, y left, z up, metres), the FDM's origin.

AC3D stores the model frame (x aft, y right, z up) as (x, z, -y); Alouette-III-sc_jsb.xml places the model at offsets z 0.115 m,
pitched 1 deg nose down.
"""
import math, os
import numpy as np

MODEL = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'Alouette-3', 'Models', 'alouetteIII_SC-ss_cockpit.ac')
OFFSET = (0.0, 0.0, 0.115)         # model frame, m (x aft): Alouette-III-sc_jsb.xml
PITCH = -1.0                       # deg, nose up (the model is pitched 1 deg nose down)


def _objects(lines):
    """parse OBJECT blocks recursively: dict(name, loc, rot, verts, surfs, kids)"""
    i = 0
    def obj():
        nonlocal i
        o = dict(name='', loc=np.zeros(3), rot=np.eye(3), verts=np.zeros((0, 3)), surfs=[], kids=[])
        i += 1                                  # OBJECT line
        while i < len(lines):
            t = lines[i].split()
            if not t:
                i += 1; continue
            k = t[0]
            if k == 'name':
                o['name'] = lines[i].split('"')[1] if '"' in lines[i] else t[1]; i += 1
            elif k == 'loc':
                o['loc'] = np.array([float(x) for x in t[1:4]]); i += 1
            elif k == 'rot':
                o['rot'] = np.array([float(x) for x in t[1:10]]).reshape(3, 3); i += 1
            elif k == 'numvert':
                n = int(t[1]); o['verts'] = np.array([[float(x) for x in lines[i + 1 + j].split()[:3]] for j in range(n)]); i += 1 + n
            elif k == 'numsurf':
                n = int(t[1]); i += 1
                for _ in range(n):
                    refs = []
                    while not lines[i].startswith('refs'):
                        i += 1
                    m = int(lines[i].split()[1]); i += 1
                    refs = [int(lines[i + j].split()[0]) for j in range(m)]; i += m
                    o['surfs'].append(refs)
            elif k == 'kids':
                n = int(t[1]); i += 1
                for _ in range(n):
                    o['kids'].append(obj())
                return o
            else:
                i += 1
        return o
    while i < len(lines) and not lines[i].startswith('OBJECT'):
        i += 1
    return obj()


def load(path=MODEL):
    """{name: [triangles (n, 3, 3) in the YASim frame]} (names can repeat: lists are merged)"""
    root = _objects(open(path, encoding='latin-1').read().splitlines())
    p = math.radians(PITCH)
    # model frame (x aft, y right, z up) pitched nose up by p about y: the nose (-x) rises
    Rp = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    out = {}

    def walk(o, M, t):
        # AC3D: a kid's vertices are in its parent's frame translated by loc (rot rarely used)
        M2 = M @ o['rot'].T if o['rot'] is not None else M
        t2 = t + M @ o['loc']
        if len(o['verts']) and o['surfs']:
            V = (M2 @ o['verts'].T).T + t2
            # AC (x, y, z) -> model (x aft, y right, z up) = (x, -z, y)
            Vm = np.c_[V[:, 0], -V[:, 2], V[:, 1]]
            Vm = (Rp @ Vm.T).T + np.array(OFFSET)
            Vy = np.c_[-Vm[:, 0], -Vm[:, 1], Vm[:, 2]]          # YASim: x fwd, y left, z up
            tris = []
            for s in o['surfs']:
                for j in range(1, len(s) - 1):
                    tris.append([Vy[s[0]], Vy[s[j]], Vy[s[j + 1]]])
            if tris:
                out.setdefault(o['name'], []).extend(tris)
        for k in o['kids']:
            walk(k, M2, t2)
    walk(root, np.eye(3), np.zeros(3))
    return {k: np.array(v) for k, v in out.items()}


def projected_area(tris, axis, cell=0.01):
    """area (m^2) of the union of the triangles' projection along axis (0 x, 1 y, 2 z), rasterised"""
    keep = [i for i in range(3) if i != axis]
    P = tris[:, :, keep]
    lo = P.reshape(-1, 2).min(0); hi = P.reshape(-1, 2).max(0)
    nx, ny = int((hi[0] - lo[0]) / cell) + 2, int((hi[1] - lo[1]) / cell) + 2
    grid = np.zeros((nx, ny), bool)
    gx = lo[0] + (np.arange(nx) + 0.5) * cell; gy = lo[1] + (np.arange(ny) + 0.5) * cell
    for a, b, c in P:
        x0, x1 = min(a[0], b[0], c[0]), max(a[0], b[0], c[0]); y0, y1 = min(a[1], b[1], c[1]), max(a[1], b[1], c[1])
        i0, i1 = int((x0 - lo[0]) / cell), int((x1 - lo[0]) / cell) + 1
        j0, j1 = int((y0 - lo[1]) / cell), int((y1 - lo[1]) / cell) + 1
        X, Y = np.meshgrid(gx[i0:i1], gy[j0:j1], indexing='ij')
        d = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])
        if abs(d) < 1e-14:
            continue
        l1 = ((b[1] - c[1]) * (X - c[0]) + (c[0] - b[0]) * (Y - c[1])) / d
        l2 = ((c[1] - a[1]) * (X - c[0]) + (a[0] - c[0]) * (Y - c[1])) / d
        inside = (l1 >= 0) & (l2 >= 0) & (l1 + l2 <= 1)
        grid[i0:i1, j0:j1] |= inside
    return grid.sum() * cell * cell, grid, (lo, cell)


if __name__ == '__main__':
    m = load()
    for name in ('rotoraxis', 'star_hub', 'tailrotorshaft', 'tailsleeverot', 'tailrotor_blade1', 'tailstab_L', 'tailstab_R',
                 'tailplate', 'tail', 'fuselage', 'skid_L', 'skid_R', 'tail_skid', 'eye', 'blade1'):
        if name in m:
            T = m[name].reshape(-1, 3)
            print('%-16s min %s max %s mean %s' % (name, np.round(T.min(0), 3), np.round(T.max(0), 3), np.round(T.mean(0), 3)))
