"""Helpers for writing JSBSim function XML."""
import collections, re


def v(x): return '<value>%.8g</value>' % x
def p(n): return '<property>%s</property>' % n
def tag(t, *a): return '<%s>%s</%s>' % (t, ''.join(a), t)
def S(*a): return tag('sum', *a) if len(a) > 1 else a[0]
def D(a, b): return tag('difference', a, b)
def M(*a): return tag('product', *a) if len(a) > 1 else a[0]
def Q(a, b): return tag('quotient', a, b)
def SQ(a): return tag('sqrt', a)
def SIN(a): return tag('sin', a)
def COS(a): return tag('cos', a)
def ATAN2(y, x): return tag('atan2', y, x)
def ABS(a): return tag('abs', a)
def FMOD(a, b): return tag('fmod', a, b)
def MAX(*a): return tag('max', *a)
def MIN(*a): return tag('min', *a)
def MOD(a, b): return tag('mod', a, b)
def NEG(a): return M(v(-1), a)
def SQR(a): return tag('pow', a, v(2))       # evaluates a once (M(a, a) reads its properties twice)
def LT(a, b): return tag('lt', a, b)
def GT(a, b): return tag('gt', a, b)
def IFTHEN(c, a, b): return tag('ifthen', c, a, b)


def T1(var, xs, ys):
    rows = '\n'.join('%.6g %.6g' % (x, y) for x, y in zip(xs, ys))
    return '<table><independentVar lookup="row">%s</independentVar><tableData>\n%s\n</tableData></table>' % (var, rows)


def T2(rowvar, colvar, rows, cols, data):
    """data[i][j] at rows[i], cols[j]"""
    head = '        ' + ' '.join('%.6g' % c for c in cols)
    body = '\n'.join('%.6g ' % r + ' '.join('%.6g' % d for d in data[i]) for i, r in enumerate(rows))
    return ('<table><independentVar lookup="row">%s</independentVar><independentVar lookup="column">%s</independentVar>'
            '<tableData>\n%s\n%s\n</tableData></table>') % (rowvar, colvar, head, body)


def T3(rowvar, colvar, tabvar, rows, cols, tabs, data):
    """data[k][i][j] at tabs[k], rows[i], cols[j]"""
    head = '        ' + ' '.join('%.6g' % c for c in cols)
    blocks = ''.join('<tableData breakPoint="%.6g">\n%s\n%s\n</tableData>' % (
        t, head, '\n'.join('%.6g ' % r + ' '.join('%.6g' % d for d in data[k][i]) for i, r in enumerate(rows))) for k, t in enumerate(tabs))
    return ('<table><independentVar lookup="row">%s</independentVar><independentVar lookup="column">%s</independentVar>'
            '<independentVar lookup="table">%s</independentVar>%s</table>') % (rowvar, colvar, tabvar, blocks)


class Graph:
    """a list of named functions; fn() returns the property reference"""

    def __init__(s):
        s.items = []

    def fn(s, name, body, desc=None):
        d = '<description>%s</description>' % desc if desc else ''
        s.items.append('<function name="%s">%s%s</function>' % (name, d, body))
        return p(name)

    def has(s, name):
        return any(i.startswith('<function name="%s">' % name) for i in s.items)

    def xml(s):
        return '\n'.join(s.items)


class ChannelGraph:
    """Graph's interface onto an FCS (its fcs_function components): for parts of the aerodynamics run at a channel's rate"""

    def __init__(s, fcs):
        s.fcs = fcs

    def fn(s, name, body, desc=None):
        return s.fcs.fn(name, body, desc=desc)

    def has(s, name):
        return any(i.startswith('<fcs_function name="%s">' % name) for n, c in s.fcs.channels for i in c)


class FCS:
    """a JSBSim system made of fcs_function components in one or more channels"""

    def __init__(s, name):
        s.name = name
        s.decl = []
        s.channels = []
        s.rates = {}
        s.execute = {}

    def declare(s, name, value=0.0):
        s.decl.append('<property value="%.8g">%s</property>' % (value, name))
        return p(name)

    def channel(s, name, execrate=None, execute=None):
        """a new channel; execrate N runs it every Nth frame (holding its outputs in between); execute: a property,
        the channel running only while it is non-zero (its outputs held otherwise)"""
        s.channels.append((name, []))
        if execrate:
            s.rates[name] = execrate
        if execute:
            s.execute[name] = execute

    def fn(s, name, body, out=None, clip=None, desc=None):
        o = ''.join('<output>%s</output>' % x for x in (out if isinstance(out, (list, tuple)) else [out] if out else []))
        c = '<clipto><min>%s</min><max>%s</max></clipto>' % clip if clip else ''
        d = '<description>%s</description>' % desc if desc else ''
        s.channels[-1][1].append('<fcs_function name="%s">%s<function>%s</function>%s%s</fcs_function>' % (name, d, body, c, o))
        return p(name)

    def raw(s, text):
        s.channels[-1][1].append(text)

    def _comp(s, kind, name, inp, inner, clip=None, out=None):
        c = '<clipto><min>%s</min><max>%s</max></clipto>' % clip if clip else ''
        o = ''.join('<output>%s</output>' % x for x in (out if isinstance(out, (list, tuple)) else [out] if out else []))
        s.channels[-1][1].append('<%s name="%s"><input>%s</input>%s%s%s</%s>' % (kind, name, inp, inner, c, o, kind))
        return p(name)

    def lag(s, name, inp, tau, clip=None, out=None):
        """1/(tau s + 1) of the property inp"""
        return s._comp('lag_filter', name, inp, '<c1>%.8g</c1>' % (1.0 / tau), clip, out)

    def washout(s, name, inp, tau, clip=None, out=None):
        """tau s/(tau s + 1)"""
        return s._comp('washout_filter', name, inp, '<c1>%.8g</c1>' % (1.0 / tau), clip, out)

    def second_order(s, name, inp, num, den, clip=None, out=None):
        """(n0 s^2 + n1 s + n2)/(d0 s^2 + d1 s + d2)"""
        cs = ''.join('<c%d>%.8g</c%d>' % (i + 1, x, i + 1) for i, x in enumerate(tuple(num) + tuple(den)))
        return s._comp('second_order_filter', name, inp, cs, clip, out)

    def actuator(s, name, inp, rate=None, lag=None, clip=None, out=None):
        r = ('<rate_limit>%s</rate_limit>' % (rate if isinstance(rate, str) else '%.8g' % rate)) if rate else ''
        inner = ('<lag>%.8g</lag>' % lag if lag else '') + r
        return s._comp('actuator', name, inp, inner, clip, out)

    def xml(s):
        ch = '\n'.join('<channel name="%s"%s%s>\n%s\n</channel>' % (n, ' execrate="%d"' % s.rates[n] if n in s.rates else '',
                                                                 ' execute="%s"' % s.execute[n] if n in s.execute else '', '\n'.join(c))
                       for n, c in s.channels)
        return '<system name="%s">\n%s\n%s\n</system>\n' % (s.name, '\n'.join(s.decl), ch)


_REF = re.compile(r'<property>([^<]+)</property>|<independentVar[^>]*>([^<]+)</independentVar>|<input>([^<]+)</input>'
                  r'|<output>([^<]+)</output>')
_DEFS = (re.compile(r'<function name="([^"]+)">(?:<description>[^<]*</description>)?(.*?)</function>\n?', re.S),
         re.compile(r'<fcs_function name="([^"]+)">(?:<description>[^<]*</description>)?<function>((?:(?!</fcs_function>).)*?)'
                    r'</function></fcs_function>\n?', re.S))


def inline_single_use(texts, extra=(), keep=()):
    """Writes each function that exactly one other function reads into that reader, in place of its property, and
    drops its definition. In FlightGear every property read takes a lock and every function its own evaluation; the
    arithmetic and its order stay the same, so the values do too. Only where the one read is a <property> in the same
    file after the definition (and for systems in the same channel: the same rate and order), and the function has no
    clip or output. texts: {file: XML}, rewritten; extra: other XML that reads properties (the FDM file); keep: names
    other programs read. Returns the number of functions written in."""
    count = collections.Counter()
    for t in list(texts.values()) + list(extra):
        for m in _REF.finditer(t):
            count[next(g for g in m.groups() if g).strip()] += 1
    n = 0
    for f in texts:
        t = texts[f]
        while True:
            chans = [(m.start(), m.end()) for m in re.finditer(r'<channel\b.*?</channel>', t, re.S)]
            for rx in _DEFS:
                hit = None
                for m in rx.finditer(t):
                    name = m.group(1)
                    if count[name] != 1 or name in keep:
                        continue
                    use = '<property>%s</property>' % name
                    i = t.find(use, m.end())
                    if i < 0 or (chans and not any(a <= m.start() and i < b for a, b in chans)):
                        continue
                    hit = (m, i, use)
                    break
                if hit:
                    break
            if not hit:
                break
            m, i, use = hit
            t = t[:m.start()] + t[m.end():i] + m.group(2) + t[i + len(use):]
            n += 1
        texts[f] = t
    return n
