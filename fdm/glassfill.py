"""Closes the holes in the cockpit's glass.

The glass is two layers sharing one vertex list: the outer layer (in alouetteIII_SC-ss_cockpit.ac) faces out and shows from
outside, the inner layer (alouetteIII-glass-inner.ac: the canopy, alouetteIII-windows.ac: the doors' windows) faces in and
shows from the cabin. With the glass tinted every hole in it shows as a strip or a patch where the view is not. There were
three kinds:

1. The inner layers lacked some of the outer layer's faces: 8 pieces of the canopy (the roof strips above the pilot, a patch
   above each corner of the dash, a patch in each lower front corner) and 5 faces of each front door window. Each face of an
   outer glass object that no inner face covers is added to the inner object, its vertices in the reverse order (facing in).
   A face with repeated vertices (the outer layer has some) loses the repeats.
2. Both layers are made of panes with gaps of 2 to 5 cm between them where the frame sits (the arches, the belt line, the
   door windows' middle bar). Where two panes' edges face each other across such a gap a strip of triangles, wound like the
   panes, joins the two edges.
3. What is left where several gaps meet is a small hole, a fan of triangles fills it (and cracks of a millimetre or two between
   pane corners that are nearly one vertex are welded first). Narrow notches left in a rim, their mouth under 6 cm, are filled
   too.

The notch in the canopy's front, right of the centre, is not a hole: a piece of the nose's structure (Int-FuselageAVStruct3)
fills it, and its sides are 24 cm apart, far more than a gap.

Every outer and inner glass object ends as one surface with one boundary, its rim. Idempotent: run again, it finds nothing.

    python3 glassfill.py          closes the holes in the .ac files
    python3 glassfill.py --check  only reports them (exit status 1 when there are some)
"""
import os
import sys

MODELS = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'Alouette-3', 'Models')
MAIN = 'alouetteIII_SC-ss_cockpit.ac'
INNER_FILES = ('alouetteIII-glass-inner.ac', 'alouetteIII-windows.ac')
# (the inner layer's file, the object in it, the outer layer's object in MAIN)
PAIRS = [('alouetteIII-glass-inner.ac', 'Windshield', 'Vitrage')] + [
    ('alouetteIII-windows.ac', n, n) for n in ('AV-DPorte-Vitrage', 'AR-DPorte-Vitrage', 'AV-GPorte-Vitrage', 'AR-GPorte-Vitrage')]
GAP = 0.08          # m: panes whose edges are nearer than this across a gap are joined (the widest gap is 5.2 cm)
SMALL_HOLE = 0.4    # m: a boundary shorter than this is a hole to fill, not a rim
POCKET = 0.06       # m: a notch in a rim whose mouth is narrower than this (and that is at least 3 times as deep) is a hole to fill
WELD = 2.5e-3       # m: boundary vertices nearer than this are one


# ---- reading and writing AC3D ---------------------------------------------------------------------------------------------
class Obj:
    """One OBJECT of an AC3D file: its vertices and faces, and the lines its surfaces take in the file."""

    def __init__(self, lines, start):
        i = start + 1
        self.name = None
        while not lines[i].startswith('numvert'):
            if lines[i].startswith('name'):
                self.name = lines[i].split('"')[1]
            i += 1
        nv = int(lines[i].split()[1])
        self.verts = [tuple(float(x) for x in lines[i + 1 + k].split()[:3]) for k in range(nv)]
        i += 1 + nv
        self.numsurf_line = i
        ns = int(lines[i].split()[1])
        i += 1
        self.recs = []                    # per face: [flags line, mat line, [(index, "u v"), ...]]
        for _ in range(ns):
            r = int(lines[i + 2].split()[1])
            refs = []
            for q in range(r):
                p = lines[i + 3 + q].split(None, 1)
                refs.append((int(p[0]), p[1] if len(p) > 1 else '0 0'))
            self.recs.append([lines[i], lines[i + 1], refs])
            i += 3 + r
        self.end = i                      # the line after the last surface: the object's "kids"
        self.faces = [[v for v, _ in rec[2]] for rec in self.recs]


def objects(lines, wanted):
    found = {}
    for i, l in enumerate(lines):
        if l.startswith('OBJECT'):
            j = i + 1
            while not lines[j].startswith('numvert') and not lines[j].startswith('OBJECT'):
                if lines[j].startswith('name') and lines[j].split('"')[1] in wanted:
                    found.setdefault(lines[j].split('"')[1], Obj(lines, i))
                j += 1
    return found


def surfaces_text(obj, faces, n_old):
    """The object's surface section: the first n_old faces as they were (rewritten only when their vertices changed), then the
    new ones. Returns the lines from numsurf on."""
    flags, mat = obj.recs[0][0], obj.recs[0][1]
    out = ['numsurf %d' % len(faces)]
    for k, f in enumerate(faces):
        if k < n_old:
            fl, mt, refs = obj.recs[k]
            if f == obj.faces[k]:
                out += [fl, mt, 'refs %d' % len(refs)] + ['%d %s' % r for r in refs]
                continue
            uv = {}
            for v_old, u in refs:
                uv.setdefault(v_old, u)
            keep = [(v, uv.get(v, refs[q][1] if len(refs) == len(f) else '0 0')) for q, v in enumerate(f)]
            out += [fl, mt, 'refs %d' % len(keep)] + ['%d %s' % r for r in keep]
        else:
            out += [flags, mat, 'refs %d' % len(f)] + ['%d 0 0' % v for v in f]
    return out


# ---- geometry -------------------------------------------------------------------------------------------------------------
def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def norm(a):
    return dot(a, a) ** 0.5


def area(verts, f):
    n = (0.0, 0.0, 0.0)
    for q in range(len(f)):
        c = cross(verts[f[q]], verts[f[(q + 1) % len(f)]])
        n = (n[0] + c[0], n[1] + c[1], n[2] + c[2])
    return norm(n) / 2


def seg_dist(p, a, b):
    ab = sub(b, a)
    t = max(0.0, min(1.0, dot(sub(p, a), ab) / (dot(ab, ab) + 1e-18)))
    return norm(sub(p, (a[0] + t * ab[0], a[1] + t * ab[1], a[2] + t * ab[2])))


def pos_ids(verts):
    ids, out = {}, []
    for v in verts:
        out.append(ids.setdefault(tuple(round(c, 4) for c in v), len(ids)))
    return out


def clean(verts, f, min_area=1e-7):
    """The face without consecutive vertices at one place; None when no face is left (fewer than 3 vertices, or less than
    min_area m2)."""
    g = []
    for v in f:
        if not g or max(abs(a - b) for a, b in zip(verts[g[-1]], verts[v])) > 1e-6:
            g.append(v)
    while len(g) > 1 and max(abs(a - b) for a, b in zip(verts[g[0]], verts[g[-1]])) <= 1e-6:
        g.pop()
    return g if len(g) >= 3 and area(verts, g) > min_area else None


# ---- 1. faces the inner layer lacks ---------------------------------------------------------------------------------------
def missing(verts, inner, outer):
    """The pieces (vertex lists, the outer face's winding) of the outer faces that no inner face covers."""
    sets = [(set(f), f, area(verts, f)) for f in inner]
    out = []
    for fo in outer:
        so = set(fo)
        if any(so <= si for si, _, _ in sets):
            continue                                              # inside an inner face
        part = [(si, f, a) for si, f, a in sets if si <= so]      # inner faces inside this outer face
        if sum(a for _, _, a in part) >= 0.98 * area(verts, fo):
            continue
        if len(fo) == 4 and len(part) == 1 and len(part[0][1]) == 3:    # one half of a quad is there: the other half
            w = [v for v in fo if v not in part[0][0]]
            if len(w) == 1:
                k = fo.index(w[0])
                out.append([fo[k - 1], fo[k], fo[(k + 1) % 4]])
                continue
        out.append(fo)
    return out


def covers(verts, inner, piece):
    """True when the inner faces cover the piece, whatever their triangulation: the piece's centroid and the points half way from
    it to each corner lie on an inner face. (The two layers are one surface, so 'on' is a distance of a tenth of a millimetre.)"""
    def on_triangle(p, a, b, c):
        n = cross(sub(b, a), sub(c, a))
        nn = norm(n)
        if nn < 1e-12 or abs(dot(sub(p, a), n)) / nn > 1e-4:
            return False
        d00, d01, d11 = dot(sub(b, a), sub(b, a)), dot(sub(b, a), sub(c, a)), dot(sub(c, a), sub(c, a))
        d20, d21 = dot(sub(p, a), sub(b, a)), dot(sub(p, a), sub(c, a))
        den = d00 * d11 - d01 * d01
        v, w = (d11 * d20 - d01 * d21) / den, (d00 * d21 - d01 * d20) / den
        return v >= -1e-3 and w >= -1e-3 and v + w <= 1 + 1e-3
    cen = tuple(sum(verts[v][c] for v in piece) / len(piece) for c in range(3))
    pts = [cen] + [tuple((cen[c] + verts[v][c]) / 2 for c in range(3)) for v in piece]
    tris = [(f[0], f[k], f[k + 1]) for f in inner for k in range(1, len(f) - 1)]
    return all(any(on_triangle(p, verts[a], verts[b], verts[c]) for a, b, c in tris) for p in pts)


# ---- 2. and 3. gaps and small holes ---------------------------------------------------------------------------------------
def boundary_loops(verts, faces):
    """The boundary of the surface as chains of vertex indices, each running with the surface on its left."""
    pid = pos_ids(verts)
    rep, edges = {}, set()
    for f in faces:
        for q in range(len(f)):
            a, b = f[q], f[(q + 1) % len(f)]
            if pid[a] != pid[b]:
                rep.setdefault(pid[a], a)
                rep.setdefault(pid[b], b)
                edges.add((pid[a], pid[b]))
    bound = [(a, b) for (a, b) in edges if (b, a) not in edges]
    nxt = {}
    for a, b in bound:
        nxt.setdefault(a, []).append(b)
    loops, used = [], set()
    for a, b in bound:
        if (a, b) in used:
            continue
        loop, cur = [a], (a, b)
        while cur not in used:
            used.add(cur)
            loop.append(cur[1])
            c = [x for x in nxt.get(cur[1], []) if (cur[1], x) not in used]
            if not c:
                break
            cur = (cur[1], c[0])
        if loop[0] == loop[-1]:
            loop.pop()
        loops.append([rep[k] for k in loop])
    return loops


def perimeter(verts, loop):
    return sum(norm(sub(verts[loop[k]], verts[loop[(k + 1) % len(loop)]])) for k in range(len(loop)))


def runs_near(verts, li, lj):
    """The maximal runs (cyclic, lists of positions in li) of loop li's vertices that are within GAP of loop lj's edges."""
    n = len(li)
    mark = [min(seg_dist(verts[k], verts[lj[q]], verts[lj[(q + 1) % len(lj)]]) for q in range(len(lj))) < GAP for k in li]
    if all(mark):
        return [list(range(n))]
    if not any(mark):
        return []
    s = next(k for k in range(n) if not mark[k])
    out, cur = [], []
    for step in range(1, n + 1):
        k = (s + step) % n
        if mark[k]:
            cur.append(k)
        elif cur:
            out.append(cur)
            cur = []
    if cur:
        out.append(cur)
    return out


def merge_strip(verts, A, B):
    """Triangles joining chain A with chain B, both running the same way (A on the left, the gap between): the pane on A's
    side is on the left of A, so each triangle takes the reverse of the edge it shares with a chain."""
    d = lambda x, y: norm(sub(verts[x], verts[y]))
    q0 = min(range(len(B)), key=lambda q: d(A[0], B[q]))
    p0 = min(range(len(A)), key=lambda p: d(A[p], B[0]))
    p, q = (0, q0) if d(A[0], B[q0]) <= d(A[p0], B[0]) else (p0, 0)
    tris = []
    while p < len(A) - 1 and q < len(B) - 1:
        if d(A[p + 1], B[q]) <= d(A[p], B[q + 1]):
            tris.append([A[p + 1], A[p], B[q]])
            p += 1
        else:
            tris.append([B[q], B[q + 1], A[p]])
            q += 1
    return tris


def bridge(verts, faces):
    loops = boundary_loops(verts, faces)
    centre = lambda L, r: tuple(sum(verts[L[k]][c] for k in r) / len(r) for c in range(3))
    new = []
    for i in range(len(loops)):
        for j in range(i + 1, len(loops)):
            ri, rj = runs_near(verts, loops[i], loops[j]), runs_near(verts, loops[j], loops[i])
            used = set()
            for r in ri:
                ci = centre(loops[i], r)
                cand = [(norm(sub(ci, centre(loops[j], s))), m) for m, s in enumerate(rj) if m not in used]
                if not cand:
                    continue
                m = min(cand)[1]
                used.add(m)
                A = [loops[i][k] for k in r]
                B = [loops[j][k] for k in reversed(rj[m])]
                new += merge_strip(verts, A, B)
    return new


def fill_small(verts, faces):
    """A fan over each boundary shorter than SMALL_HOLE, wound like the faces round it."""
    new = []
    for l in boundary_loops(verts, faces):
        if len(l) >= 3 and perimeter(verts, l) <= SMALL_HOLE:
            r = l[::-1]
            new += [[r[0], r[k], r[k + 1]] for k in range(1, len(r) - 1)]
    return new


def triangulate(verts, poly, normal):
    """Ear clipping of a polygon (vertex indices, counter-clockwise seen from the side normal points to); a vertex on a straight
    line is dropped without a triangle."""
    ax = (abs(normal[0]), abs(normal[1]), abs(normal[2]))
    drop = ax.index(max(ax))
    keep = [c for c in range(3) if c != drop]
    sgn = 1.0 if normal[drop] >= 0 else -1.0
    P = {v: (verts[v][keep[0]], sgn * verts[v][keep[1]]) for v in poly}
    def cr(a, b, c):
        return (P[b][0] - P[a][0]) * (P[c][1] - P[a][1]) - (P[b][1] - P[a][1]) * (P[c][0] - P[a][0])
    def inside(p, a, b, c):
        return cr(a, b, p) >= -1e-12 and cr(b, c, p) >= -1e-12 and cr(c, a, p) >= -1e-12
    poly, tris = list(poly), []
    guard = 0
    while len(poly) > 3 and guard < 1000:
        guard += 1
        for k in range(len(poly)):
            a, b, c = poly[k - 1], poly[k], poly[(k + 1) % len(poly)]
            w = cr(a, b, c)
            if abs(w) < 1e-12:
                poly.pop(k)
                break
            if w > 0 and not any(inside(p, a, b, c) for p in poly if p not in (a, b, c)):
                tris.append([a, b, c])
                poly.pop(k)
                break
        else:
            break
    if len(poly) == 3 and abs(cr(*poly)) > 1e-12:
        tris.append(list(poly))
    return tris


def fill_pockets(verts, faces):
    """The faces filling the narrowest notch of a rim that is a hole (its mouth under POCKET and at least 3 times as deep) or [];
    called again until it finds none."""
    best = None
    owner = {}
    for f in faces:
        for q in range(len(f)):
            owner.setdefault((f[q], f[(q + 1) % len(f)]), f)
    for loop in boundary_loops(verts, faces):
        n = len(loop)
        cum = [0.0]
        for k in range(2 * n):
            cum.append(cum[-1] + norm(sub(verts[loop[k % n]], verts[loop[(k + 1) % n]])))
        for i in range(n):
            for step in range(2, n - 1):
                chain = cum[i + step] - cum[i]
                if chain > 0.5:
                    break
                chord = norm(sub(verts[loop[i]], verts[loop[(i + step) % n]]))
                if chord <= POCKET and chain >= 3 * max(chord, 1e-3) and (best is None or chain < best[0]):
                    best = (chain, [loop[(i + k) % n] for k in range(step + 1)])
    if best is None:
        return []
    chain = best[1]
    # the polygon runs with the hole on its right; reversed it runs counter-clockwise seen from the panes' front
    nrm = (0.0, 0.0, 0.0)
    for a, b in zip(chain, chain[1:]):
        f = owner.get((a, b))
        if f:
            for q in range(len(f)):
                c = cross(sub(verts[f[(q + 1) % len(f)]], verts[f[q]]), sub(verts[f[(q + 2) % len(f)]], verts[f[q]]))
                nrm = (nrm[0] + c[0], nrm[1] + c[1], nrm[2] + c[2])
    return triangulate(verts, chain[::-1], nrm)


def weld(verts, faces):
    """Boundary vertices at different places that are within WELD of each other become one (the lower index) in every face;
    returns the faces and the mapping (position number -> vertex index) that was applied."""
    pid = pos_ids(verts)
    edges = set()
    for f in faces:
        for q in range(len(f)):
            a, b = pid[f[q]], pid[f[(q + 1) % len(f)]]
            if a != b:
                edges.add((a, b))
    bv = {x for a, b in edges if (b, a) not in edges for x in (a, b)}
    rep = {}
    for i, p in enumerate(pid):
        rep.setdefault(p, i)
    cand = sorted(rep[p] for p in bv)
    moved = {}
    for x, i in enumerate(cand):
        if pid[i] in moved:
            continue
        for j in cand[x + 1:]:
            if pid[j] not in moved and pid[j] != pid[i] and max(abs(a - b) for a, b in zip(verts[i], verts[j])) <= WELD:
                moved[pid[j]] = i
    return [[moved.get(pid[v], v) for v in f] for f in faces], moved


def apply_welds(verts, faces, welds):
    pid = pos_ids(verts)
    for moved in welds:
        faces = [[moved.get(pid[v], v) for v in f] for f in faces]
    return faces


def directed_edges(verts, faces):
    pid = pos_ids(verts)
    return {(pid[f[q]], pid[f[(q + 1) % len(f)]]) for f in faces for q in range(len(f)) if pid[f[q]] != pid[f[(q + 1) % len(f)]]}


def add_new(verts, faces, new):
    """new appended to faces, but not a face that would run an edge the same way as one already there (the two would overlap)."""
    pid = pos_ids(verts)
    have = directed_edges(verts, faces)
    for f in new:
        e = [(pid[f[q]], pid[f[(q + 1) % len(f)]]) for q in range(len(f))]
        e = [x for x in e if x[0] != x[1]]
        if e and not any(x in have for x in e):
            faces.append(f)
            have.update(e)
    return faces


def complete(verts, faces):
    """The faces with the gaps between panes bridged, the junctions filled and the cracks welded; also the welds that were made
    (for the other layer). The faces that were there keep their places in the list (a weld may change their vertices); what is
    added comes after them."""
    faces = [list(f) for f in faces]
    n_old = len(faces)
    welds = []
    for _ in range(4):
        n_before = len(faces)
        faces = add_new(verts, faces, bridge(verts, faces))
        faces = add_new(verts, faces, fill_small(verts, faces))
        faces, moved = weld(verts, faces)
        if moved:
            welds.append(moved)
        faces = add_new(verts, faces, fill_small(verts, faces))
        for _ in range(30):
            pf = fill_pockets(verts, faces)
            before = len(faces)
            faces = add_new(verts, faces, pf)
            if len(faces) == before:
                break
        faces = [f for k, f in enumerate(faces) if k < n_old or clean(verts, f, 1e-10) is not None]     # no new face without area
        if len(faces) == n_before and not moved:
            break
    return faces, welds


# ---- the files ------------------------------------------------------------------------------------------------------------
def main(check=False):
    main_path = os.path.join(MODELS, MAIN)
    texts = {main_path: open(main_path, encoding='latin-1').read().split('\n')}
    outers = objects(texts[main_path], {p[2] for p in PAIRS})
    inners = {}
    for fname in INNER_FILES:
        path = os.path.join(MODELS, fname)
        texts[path] = open(path, encoding='latin-1').read().split('\n')
        found = objects(texts[path], {p[1] for p in PAIRS if p[0] == fname})
        inners.update({(fname, n): (path, o) for n, o in found.items()})
    results, total = [], 0
    for fname, iname, oname in PAIRS:
        ipath, inner = inners[(fname, iname)]
        outer = outers[oname]
        verts = outer.verts
        if len(verts) != len(inner.verts) or any(abs(a - b) > 1e-4 for p, q in zip(verts, inner.verts) for a, b in zip(p, q)):
            sys.exit('%s: the inner and the outer layer do not share a vertex list' % iname)
        # 1. what the inner layer lacks of the outer layer as it is
        pieces = [g for g in (clean(verts, f) for f in missing(verts, inner.faces, outer.faces)) if g]
        pieces = [g for g in pieces if not covers(verts, inner.faces, g)]               # (the faces may be cut differently)
        # 2. the outer layer's gaps and holes closed, and the same faces, reversed, for the inner layer
        outer_done, welds = complete(verts, outer.faces)
        added = outer_done[len(outer.faces):]
        inner_done = apply_welds(verts, [list(f) for f in inner.faces] + [g[::-1] for g in pieces], welds)
        for k in range(len(inner.faces)):             # a weld that doubled a vertex in an old face: the repeat goes
            if inner_done[k] != inner.faces[k]:
                inner_done[k] = clean(verts, inner_done[k]) or inner_done[k]
        inner_done += [f[::-1] for f in added]
        for k in range(len(outer.faces)):
            if outer_done[k] != outer.faces[k]:
                outer_done[k] = clean(verts, outer_done[k]) or outer_done[k]
        for label, obj, done, path in (('inner', inner, inner_done, ipath), ('outer', outer, outer_done, main_path)):
            n_old = len(obj.faces)
            loops = boundary_loops(verts, done)
            small = sum(1 for l in loops if perimeter(verts, l) <= SMALL_HOLE)
            changed = sum(1 for k in range(n_old) if done[k] != obj.faces[k])
            print('%-18s %-6s %3d faces: %2d lacking, %3d to bridge and fill, %d old ones welded; boundaries %d (%d small)' % (
                iname, label, n_old, len(pieces) if label == 'inner' else 0, len(added), changed, len(loops), small))
            if len(done) > n_old or changed:
                results.append((path, obj, done, n_old))
                total += len(done) - n_old
    if not check:
        for path, obj, done, n_old in sorted(results, key=lambda r: -r[1].numsurf_line):   # last object first: no line moves under the next
            texts[path][obj.numsurf_line:obj.end] = surfaces_text(obj, done, n_old)
        for path in {r[0] for r in results}:
            open(path, 'w', encoding='latin-1').write('\n'.join(texts[path]))
    return total


if __name__ == '__main__':
    n = main(check='--check' in sys.argv)
    print('%d faces %s' % (n, 'to add' if '--check' in sys.argv else 'added'))
    sys.exit(1 if n and '--check' in sys.argv else 0)
