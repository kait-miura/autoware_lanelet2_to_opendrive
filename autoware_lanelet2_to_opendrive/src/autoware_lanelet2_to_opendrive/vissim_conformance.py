"""Check an emitted ``.xodr`` against the PTV Vissim import specification.

One check per documented requirement, each reporting PASS/FAIL with the
measurement behind it, so a file can be judged against the whole
specification at once instead of one symptom at a time. Every threshold comes
from the manual's openDRIVE import section (see
``docs/vissim-compatibility.md``): the 0.5 m minimum spline spacing, the
1.1 m links inserted at a width change, the 0.25 m width-variation threshold,
the 1 m lane-width clamp, the pre-1.5 schema version and the supported
geometry primitives.

Usage::

    uv run vissim-check <file.xodr>

Exits non-zero when a check fails, so it can gate a conversion.
"""

import collections
import math
import sys
import xml.etree.ElementTree as ET

VISSIM_LANE_TYPES = {
    "driving",
    "entry",
    "exit",
    "offRamp",
    "onRamp",
    "roadWorks",
    "tram",
    "rail",
    "biking",
}
MIN_SPLINE = 0.5
INSERTED_LINK = 1.1
WIDTH_STEP = 0.25
MIN_WIDTH = 1.0


def _run(path: str) -> int:
    root = ET.parse(path).getroot()
    roads = {r.get("id"): r for r in root.iter("road")}
    juncs = {j.get("id"): j for j in root.iter("junction")}
    results: list = []

    def check(name, ok, detail):
        results.append((ok, name, detail))

    def ev(g, p):
        x, y, hdg, L = (float(g.get(k)) for k in ("x", "y", "hdg", "length"))
        c, s = math.cos(hdg), math.sin(hdg)
        pp = g.find("paramPoly3")
        if pp is None:
            arc = g.find("arc")
            if arc is not None:
                k = float(arc.get("curvature"))
                if abs(k) > 1e-12:
                    th = hdg + k * p
                    return (
                        x + (math.sin(th) - math.sin(hdg)) / k,
                        y + (math.cos(hdg) - math.cos(th)) / k,
                        th,
                    )
            return (x + c * p, y + s * p, hdg)
        a = [float(pp.get(k, "0")) for k in ("aU", "bU", "cU", "dU")]
        b = [float(pp.get(k, "0")) for k in ("aV", "bV", "cV", "dV")]
        t = p / L if pp.get("pRange", "normalized") == "normalized" else p
        u = a[0] + a[1] * t + a[2] * t * t + a[3] * t**3
        v = b[0] + b[1] * t + b[2] * t * t + b[3] * t**3
        du = a[1] + 2 * a[2] * t + 3 * a[3] * t * t
        dv = b[1] + 2 * b[2] * t + 3 * b[3] * t * t
        return (x + c * u - s * v, y + s * u + c * v, hdg + math.atan2(dv, du))

    def arc_len(g, n=400):
        L = float(g.get("length"))
        pts = [ev(g, L * i / n)[:2] for i in range(n + 1)]
        return sum(math.dist(pts[i], pts[i + 1]) for i in range(n))

    def lanes_of(r, driving_only=True):
        out = []
        for side in ("left", "right"):
            el = r.find(f"lanes/laneSection/{side}")
            if el is None:
                continue
            sign = 1 if side == "left" else -1
            for ln in sorted(el.findall("lane"), key=lambda e: sign * int(e.get("id"))):
                if driving_only and ln.get("type") not in VISSIM_LANE_TYPES:
                    continue
                out.append(ln)
        return out

    def width_of(ln):
        w = ln.find("width")
        return float(w.get("a")) if w is not None else 0.0

    def lane_centre_t(r, lane_id):
        sign = 1 if lane_id > 0 else -1
        el = r.find(f"lanes/laneSection/{'left' if lane_id > 0 else 'right'}")
        if el is None:
            return None
        edge = 0.0
        for ln in sorted(el.findall("lane"), key=lambda e: sign * int(e.get("id"))):
            w = width_of(ln)
            if int(ln.get("id")) == lane_id:
                return sign * (edge + w / 2)
            edge += w
        return None

    def lane_pt(r, lane_id, at_end):
        gs = r.findall("planView/geometry")
        if not gs:
            return None
        g = gs[-1] if at_end else gs[0]
        x, y, h = ev(g, float(g.get("length")) if at_end else 0.0)
        t = lane_centre_t(r, lane_id)
        if t is None:
            return None
        return (x - math.sin(h) * t, y + math.cos(h) * t)

    print("=" * 78)
    print(f"Vissim conformance: {path}")
    print("=" * 78)

    # 1. header version < 1.5
    h = root.find("header")
    v = (h.get("revMajor"), h.get("revMinor"))
    check(
        "header revMajor/revMinor below 1.5", v == ("1", "4"), f"declared {v[0]}.{v[1]}"
    )

    # 2. no attributes outside the declared 1.4 schema
    off = collections.Counter()
    for tag, attr in (("road", "rule"), ("lane", "rule"), ("access", "rule")):
        for e in root.iter(tag):
            if attr in e.attrib:
                off[f"{tag}@{attr}"] += 1
    for pp in root.iter("paramPoly3"):
        if "pRange" in pp.attrib:
            off["paramPoly3@pRange"] += 1
    check("no non-1.4 attributes", not off, dict(off) or "none found")

    # 3. geometry primitives limited to line / spiral / arc / cubic
    kinds = collections.Counter(
        ch.tag
        for g in root.iter("geometry")
        for ch in g
        if ch.tag in ("line", "spiral", "arc", "poly3", "paramPoly3")
    )
    check(
        "geometry primitives supported",
        set(kinds) <= {"line", "spiral", "arc", "paramPoly3"},
        dict(kinds),
    )

    # 4. declared geometry length matches the integrated arc length
    bad = []
    for rid, r in roads.items():
        for g in r.findall("planView/geometry"):
            declared = float(g.get("length"))
            actual = arc_len(g)
            if abs(actual - declared) > 1e-3:
                bad.append(
                    (abs(actual - declared), rid, float(g.get("s")), declared, actual)
                )
    bad.sort(reverse=True)
    check(
        "geometry length attribute matches its curve",
        not bad,
        f"{len(bad)} mismatches"
        + (
            f", worst road {bad[0][1]} at s={bad[0][2]:.2f}: "
            f"declared {bad[0][3]:.6f} vs {bad[0][4]:.6f}"
            if bad
            else ""
        ),
    )

    # 5. planView sums to road@length
    bad2 = [
        rid
        for rid, r in roads.items()
        if abs(
            sum(float(g.get("length")) for g in r.findall("planView/geometry"))
            - float(r.get("length"))
        )
        > 1e-3
    ]
    check(
        "planView sums to road@length",
        not bad2,
        f"{len(bad2)} roads differ: {bad2[:6]}",
    )

    # 6. geometry continuity between consecutive segments
    c0 = []
    for rid, r in roads.items():
        gs = r.findall("planView/geometry")
        for a, b in zip(gs, gs[1:]):
            xe, ye, _ = ev(a, float(a.get("length")))
            c0.append(
                (math.dist((xe, ye), (float(b.get("x")), float(b.get("y")))), rid)
            )
    c0.sort(reverse=True)
    check(
        "consecutive geometries meet (C0)",
        not c0 or c0[0][0] < 1e-3,
        f"worst {c0[0][0]:.2e} m on road {c0[0][1]}" if c0 else "no joints",
    )

    # 7. lane widths constant (Vissim treats them so)
    nonconst = sum(
        1
        for w in root.iter("width")
        if any(abs(float(w.get(k, "0"))) > 1e-12 for k in "bcd")
    )
    multi = sum(1 for ln in root.iter("lane") if len(ln.findall("width")) > 1)
    check(
        "lane widths constant",
        nonconst == 0 and multi == 0,
        f"{nonconst} non-constant records, {multi} lanes with several records",
    )

    # 8. width does not vary within a road (what the 0.25 m rule is about)
    swings = []
    for rid, r in roads.items():
        for ln in lanes_of(r):
            ws = ln.findall("width")
            vals = [float(w.get("a")) for w in ws]
            if len(vals) > 1:
                swings.append((max(vals) - min(vals), rid))
            for w in ws:
                if any(abs(float(w.get(k, "0"))) > 1e-12 for k in "bcd"):
                    swings.append((WIDTH_STEP, rid))
    swings.sort(reverse=True)
    check(
        f"width does not vary within a road by {WIDTH_STEP} m or more",
        not swings,
        f"{len(swings)} lanes vary"
        if swings
        else "every lane has one constant width record, so Vissim inserts nothing",
    )

    # 9. lane centres meet at joints
    gaps = []
    for jid, j in juncs.items():
        for c in j.findall("connection"):
            cr = roads.get(c.get("connectingRoad"))
            if cr is None:
                continue
            lk = cr.find("link")
            for side, at_end in (("predecessor", False), ("successor", True)):
                e = lk.find(side) if lk is not None else None
                if e is None or e.get("elementType") != "road":
                    continue
                nb = roads.get(e.get("elementId"))
                if nb is None:
                    continue
                nb_end = e.get("contactPoint") != "start"
                pairs = (
                    [
                        (int(x.get("to")), int(x.get("from")))
                        for x in c.findall("laneLink")
                    ]
                    if side == "predecessor"
                    else []
                )
                if side == "successor":
                    for ln in cr.iter("lane"):
                        if ln.get("type") not in VISSIM_LANE_TYPES:
                            continue
                        l2 = ln.find("link")
                        ss = l2.find("successor") if l2 is not None else None
                        if ss is not None:
                            pairs.append((int(ln.get("id")), int(ss.get("id"))))
                for own, other in pairs:
                    p = lane_pt(cr, own, at_end)
                    q = lane_pt(nb, other, nb_end)
                    if p and q:
                        gaps.append(
                            (
                                math.dist(p, q),
                                c.get("connectingRoad"),
                                e.get("elementId"),
                            )
                        )
    gaps.sort(reverse=True)
    check(
        "lane centres meet at joints (<0.1 m)",
        not gaps or gaps[0][0] < 0.1,
        f"worst {gaps[0][0]:.3f} m connector {gaps[0][1]} / road {gaps[0][2]}; "
        f">0.1 m: {sum(1 for g in gaps if g[0] > 0.1)}/{len(gaps)}"
        if gaps
        else "no joints",
    )

    # 10. no road below Vissim's spline spacing / inserted-link length
    drv = sorted(
        (float(r.get("length")), rid) for rid, r in roads.items() if lanes_of(r)
    )
    check(
        f"no importable road below {MIN_SPLINE} m",
        not drv or drv[0][0] >= MIN_SPLINE,
        f"shortest {drv[0][0]:.3f} m (road {drv[0][1]}); below {INSERTED_LINK} m: "
        f"{sum(1 for L, _ in drv if L < INSERTED_LINK)}",
    )

    # 11. no lane below the 1 m clamp
    thin = [
        (width_of(ln), rid)
        for rid, r in roads.items()
        for ln in lanes_of(r)
        if 0 < width_of(ln) < MIN_WIDTH
    ]
    check(
        f"no lane below the {MIN_WIDTH} m clamp",
        not thin,
        f"{len(thin)} lanes, thinnest {min(thin)[0]:.2f} m" if thin else "none",
    )

    def _expressed_elsewhere(road_id, lane_id):
        """Does any other road state the movement into this lane from its side?

        OpenDRIVE lets a road name one predecessor and one successor, so where
        several roads meet one endpoint only one of them can be named back. The
        movement is still there as long as somebody states it.
        """
        for other, r in roads.items():
            if other == road_id:
                continue
            for tag in ("predecessor", "successor"):
                e = r.find(f"link/{tag}")
                if e is None or e.get("elementType") != "road":
                    continue
                if e.get("elementId") != road_id:
                    continue
                for ln in lanes_of(r):
                    l2 = ln.find("link")
                    t = l2.find(tag) if l2 is not None else None
                    if t is not None and int(t.get("id")) == lane_id:
                        return True
        return False

    # 12. link/lane-link integrity
    dang = 0
    lane_bad = collections.Counter()
    for rid, r in roads.items():
        lk = r.find("link")
        for e in lk if lk is not None else []:
            if e.get("elementType") == "road" and e.get("elementId") not in roads:
                dang += 1
            if e.get("elementType") == "junction" and e.get("elementId") not in juncs:
                dang += 1
    for jid, j in juncs.items():
        for c in j.findall("connection"):
            for k in ("incomingRoad", "connectingRoad"):
                if c.get(k) not in roads:
                    dang += 1
    for rid, r in roads.items():
        for tag, idx in (("predecessor", 0), ("successor", 1)):
            e = r.find(f"link/{tag}")
            if e is None or e.get("elementType") != "road":
                continue
            nb = roads.get(e.get("elementId"))
            if nb is None:
                continue
            ids = {x.get("id") for x in nb.iter("lane")}
            claimed = collections.Counter()
            for ln in lanes_of(r):
                l2 = ln.find("link")
                t = l2.find(tag) if l2 is not None else None
                if t is None:
                    if not _expressed_elsewhere(rid, int(ln.get("id"))):
                        lane_bad["movement expressed nowhere"] += 1
                    else:
                        lane_bad["stated by the other road only"] += 1
                    continue
                claimed[t.get("id")] += 1
                if t.get("id") not in ids:
                    lane_bad["points at a missing lane"] += 1
            for k, n in claimed.items():
                if n > 1:
                    lane_bad["several lanes claim one"] += 1
    check("no dangling road or junction reference", dang == 0, f"{dang} found")
    fatal = {
        k: v
        for k, v in lane_bad.items()
        if k
        in (
            "movement expressed nowhere",
            "points at a missing lane",
            "several lanes claim one",
        )
    }
    check(
        "every lane movement is expressed somewhere",
        not fatal,
        (
            f"{fatal}"
            if fatal
            else f"clean; {lane_bad.get('stated by the other road only', 0)} lane(s) are "
            "named only by the road on the far side, which is all OpenDRIVE allows "
            "where several roads meet one endpoint"
        ),
    )

    # 13. every importable road is reachable
    succ = collections.defaultdict(set)
    for rid, r in roads.items():
        s = r.find("link/successor")
        if s is not None and s.get("elementType") == "road":
            succ[rid].add(s.get("elementId"))
        p = r.find("link/predecessor")
        if p is not None and p.get("elementType") == "road":
            succ[p.get("elementId")].add(rid)
    for jid, j in juncs.items():
        for c in j.findall("connection"):
            succ[c.get("incomingRoad")].add(c.get("connectingRoad"))
            cr = roads.get(c.get("connectingRoad"))
            if cr is None:
                continue
            for t in ("predecessor", "successor"):
                e = cr.find(f"link/{t}")
                if (
                    e is not None
                    and e.get("elementType") == "road"
                    and e.get("elementId") != c.get("incomingRoad")
                ):
                    succ[c.get("connectingRoad")].add(e.get("elementId"))
    pred = collections.defaultdict(set)
    for a, bs in succ.items():
        for b in bs:
            pred[b].add(a)
    importable = {rid for rid, r in roads.items() if lanes_of(r)}
    seen, stack = set(), [x for x in importable if not pred[x]]
    while stack:
        x = stack.pop()
        if x in seen:
            continue
        seen.add(x)
        stack.extend(succ[x])
    unreach = sorted(importable - seen, key=int)
    check(
        "every importable road reachable",
        not unreach,
        f"{len(unreach)} unreachable: {unreach[:8]}",
    )

    # 14. isolated importable roads
    iso = [
        rid
        for rid in importable
        if roads[rid].find("link") is None
        or (
            roads[rid].find("link/predecessor") is None
            and roads[rid].find("link/successor") is None
        )
    ]
    check("no isolated importable road", not iso, f"{len(iso)}: {iso[:8]}")

    # 15. elevation sane
    zs = []
    for rid, r in roads.items():
        prof = r.find("elevationProfile")
        if prof is None:
            continue
        for e in prof.findall("elevation"):
            zs.append(float(e.get("a")))
    grad = [abs(float(e.get("b", "0"))) for e in root.iter("elevation")]
    check(
        "elevation near the ground plane",
        not zs or (min(zs) > -1.0 and max(zs) < 20.0),
        f"z {min(zs):.2f}..{max(zs):.2f} m" if zs else "no profile",
    )
    check(
        "no absurd gradient (<15%)",
        not grad or max(grad) < 0.15,
        f"steepest {max(grad)*100:.1f}%" if grad else "n/a",
    )

    # 16. junctions are real intersections
    notx = []
    for jid, j in juncs.items():
        arms = {c.get("incomingRoad") for c in j.findall("connection")}
        if len(arms) < 3:
            notx.append((jid, len(arms)))
    check("junctions have 3+ approaches", not notx, f"{notx}" if notx else "all do")

    print()
    for ok, name, detail in results:
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}")
        print(f"         {detail}")
    passed = sum(1 for ok, *_ in results if ok)
    print(f"\n  {passed}/{len(results)} checks pass")

    return 0 if passed == len(results) else 1


def main() -> int:
    """Entry point: check the file named on the command line."""
    if len(sys.argv) != 2:
        print(__doc__)
        return 2
    return _run(sys.argv[1])


if __name__ == "__main__":
    raise SystemExit(main())
