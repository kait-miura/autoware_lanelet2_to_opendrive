"""Vissim-targeted topology pass, run inside the conversion pipeline.

Vissim turns every ``<road>`` into a link, every connecting road into a
connector, and every ``<junction>`` into a **node** — and a node brings the
whole intersection machinery with it: auto-generated conflict areas,
priority rules, reduced-speed areas. Since it imports no right-of-way, those
conflict areas come up with an undetermined priority and have to be reviewed
by hand.

:func:`dissolve_non_intersection_junctions` therefore removes the junctions
that are not intersections. The divergence synthesis wraps *every*
lane-level merge and diverge in a junction, so a plain widening, an off-ramp
or a lane drop otherwise arrives in Vissim as an intersection: on the Odaiba
clip only one of seven junctions has crossing movements. Their connecting
roads become ordinary roads and each neighbour is repointed at the branch
carrying the most lane links; secondary branches keep their own
predecessor/successor, and because Vissim builds connectors from
``link::predecessor``/``link::successor`` the movement survives even though
OpenDRIVE lets the neighbour name only one of them. Road and lane ids are
never touched, so the ``*.mapping.json`` sidecar stays valid.

:func:`analyze_topology` then reports two constructs that are left for
review because no safe automatic repair exists at this level:

1. A connecting road that runs *along* a through road in the same direction.
   On the Odaiba clip, connecting road 53 (road 31 → road 9) runs along
   48 % of through road 12 — the middle of the southbound-to-northbound
   three-lane corridor. It is only reported: connecting roads that look like
   duplicates at road level (Odaiba 43, 47 and 50) serve *different
   destination lanes* of the same outgoing road, so dropping one silently
   deletes a movement.
2. A connecting road shorter than Vissim's 0.5 m minimum spline spacing.
   The Odaiba clip has six 0.01 m stubs (roads 34–39), kept because CARLA's
   loader needs them.

Coverage is measured between **lane centres**, not reference lines, and only
counts stations whose tangents agree. Both gates matter: comparing reference
lines alone reports the *oncoming* carriageway of a two-way road as a 100 %
overlay, because its reference line can run within a metre of ours. That
artifact is why a naive metric flags connectors which merely pass the other
side of the road.
"""

import itertools
import logging
import math
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Set, Tuple

from .opendrive.enums import ContactPoint, ElementType
from .opendrive.junction import Connection, Junction
from .opendrive.road import Road
from .opendrive.lane_elements import LaneLink
from .opendrive.road_links import Predecessor, Successor

logger = logging.getLogger(__name__)

#: Fraction of a through road that must be covered by a connecting road
#: before the connector counts as running along that road.
DEFAULT_MIN_LINK_COVERAGE = 0.4

#: A junction with at least this many distinct incoming roads is treated as a
#: real intersection even when no two of its movements cross (an approach
#: whose turns all leave on different arms still needs the node).
DEFAULT_MIN_INTERSECTION_ARMS = 3

#: A left/right lanelet whose heading changes by less than this is an
#: approach lane (a turn pocket), not the interior of an intersection.
DEFAULT_STRAIGHT_TURN_TOLERANCE_DEG = 25.0

#: Vissim sets spline points at a minimum spacing of 0.5 m, so a shorter
#: connecting road cannot be represented as a proper connector.
VISSIM_MIN_CONNECTOR_LENGTH = 0.5

#: Two carriageways only belong to the same traffic stream when their
#: tangents agree within this angle. Without the gate, the opposing
#: carriageway of a two-way road — whose reference line can run within a
#: metre of ours in the other direction — would look like an overlay.
DEFAULT_MAX_HEADING_DIFF_DEG = 45.0

#: Stations sampled along each road when measuring coverage.
_STATIONS_PER_ROAD = 60

#: Lateral match tolerance (m). Roughly half a lane: two carriageways count
#: as overlapping at a station only if some lane centre of one lands this
#: close to a lane centre of the other.
_LATERAL_TOLERANCE = 1.5


@dataclass
class SameStreamOverlap:
    """A through road that a foreign connector runs along, same direction."""

    road_id: int
    connector_id: int
    junction_id: int
    coverage: float


@dataclass
class DegenerateConnector:
    """A connecting road shorter than Vissim's minimum spline spacing."""

    road_id: int
    junction_id: int
    length: float
    from_road: int
    to_road: int
    junction_required: bool


@dataclass
class DissolvedJunction:
    """A junction that carried no crossing movement and was dissolved."""

    junction_id: int
    name: Optional[str]
    incoming_roads: List[int]
    connecting_roads: List[int]


@dataclass
class VissimTopologyReport:
    """Constructs in the emitted network that Vissim degrades on."""

    overlaps: List[SameStreamOverlap] = field(default_factory=list)
    degenerate_connectors: List[DegenerateConnector] = field(default_factory=list)
    dissolved_junctions: List[DissolvedJunction] = field(default_factory=list)
    kept_junctions: List[Tuple[int, int, int]] = field(default_factory=list)
    """``(junction_id, arm_count, crossing_pairs)`` for real intersections."""

    def log(self, log: logging.Logger = logger) -> None:
        """Report what was dissolved and the constructs left for review."""
        for junction in self.dissolved_junctions:
            log.info(
                "Vissim topology: dissolved junction %d (%s) — %d incoming "
                "road(s), no crossing movement, so it is a merge/diverge and "
                "not an intersection; its connecting roads %s became ordinary "
                "roads so Vissim does not place a node here",
                junction.junction_id,
                junction.name or "unnamed",
                len(junction.incoming_roads),
                junction.connecting_roads,
            )
        if self.kept_junctions:
            log.info(
                "Vissim topology: kept %d real intersection(s): %s",
                len(self.kept_junctions),
                ", ".join(
                    f"junction {jid} ({arms} arms, {crossings} crossing pairs)"
                    for jid, arms, crossings in self.kept_junctions
                ),
            )
        if self.overlaps:
            log.warning(
                "Vissim topology: %d through road(s) have a connector running "
                "along them in the same direction — Vissim will cover that "
                "stretch with an undetermined conflict area:",
                len({overlap.road_id for overlap in self.overlaps}),
            )
            for overlap in self.overlaps:
                log.warning(
                    "  road %d: connector %d (junction %d) runs along %.0f%% of it",
                    overlap.road_id,
                    overlap.connector_id,
                    overlap.junction_id,
                    overlap.coverage * 100,
                )
        else:
            log.info(
                "Vissim topology: no connector runs along a through road in "
                "the same direction"
            )

        if self.degenerate_connectors:
            log.warning(
                "Vissim topology: %d connecting road(s) are shorter than "
                "Vissim's %.1f m minimum spline spacing and become degenerate "
                "connectors:",
                len(self.degenerate_connectors),
                VISSIM_MIN_CONNECTOR_LENGTH,
            )
            for stub in self.degenerate_connectors:
                note = (
                    "junction needed (merge/diverge)"
                    if stub.junction_required
                    else "could become a direct road-to-road link"
                )
                log.warning(
                    "  road %d (%.3f m, junction %d): road %d -> road %d — %s",
                    stub.road_id,
                    stub.length,
                    stub.junction_id,
                    stub.from_road,
                    stub.to_road,
                    note,
                )


def _lane_centre_offsets(road: Road) -> List[float]:
    """Signed ``t`` offsets of each driving lane centre in the first section.

    Positive ids sit left of the reference line, negative ids right of it, so
    the sign follows the lane id. Sampling lane centres — rather than the
    reference line — is what distinguishes a connector laid along our
    carriageway from the opposing carriageway of a two-way road, whose
    reference line may run within a metre of ours.
    """
    if road.lanes is None or not road.lanes.lane_sections:
        return []
    section = road.lanes.lane_sections[0]

    def width_of(lane) -> float:
        widths = getattr(lane, "widths", None) or []
        return float(getattr(widths[0], "a", 0.0) or 0.0) if widths else 0.0

    offsets: List[float] = []
    edge = 0.0
    for lane_id in sorted(getattr(section, "left_lanes", {})):
        width = width_of(section.left_lanes[lane_id])
        offsets.append(edge + width / 2.0)
        edge += width
    edge = 0.0
    for lane_id in sorted(getattr(section, "right_lanes", {}), reverse=True):
        width = width_of(section.right_lanes[lane_id])
        offsets.append(-(edge + width / 2.0))
        edge += width
    return offsets


def _band_stations(
    road: Road, count: int = _STATIONS_PER_ROAD
) -> List[Tuple[List[Tuple[float, float]], float]]:
    """Sample the carriageway: per station, its lane-centre points and heading."""
    # Imported here to avoid a module-level cycle through the mapping module.
    from .road_lanelet_geo_mapping import _evaluate_geometry_world

    if road.plan_view is None or not road.plan_view.geometries:
        return []
    geometries = list(road.plan_view.geometries)
    total = sum(geometry.length for geometry in geometries)
    if total <= 0.0:
        return []
    offsets = _lane_centre_offsets(road)
    if not offsets:
        return []

    stations: List[Tuple[List[Tuple[float, float]], float]] = []
    for index in range(count):
        target = total * index / (count - 1)
        walked = 0.0
        for geometry in geometries:
            if walked + geometry.length >= target - 1e-9:
                local = min(max(target - walked, 0.0), geometry.length)
                x, y = _evaluate_geometry_world(geometry, local)
                # Tangent from a short finite difference so every geometry
                # primitive is handled by the same evaluator.
                step = min(0.05, max(geometry.length, 1e-6))
                ahead = min(local + step, geometry.length)
                behind = max(ahead - step, 0.0)
                x1, y1 = _evaluate_geometry_world(geometry, behind)
                x2, y2 = _evaluate_geometry_world(geometry, ahead)
                heading = math.atan2(y2 - y1, x2 - x1)
                normal = (-math.sin(heading), math.cos(heading))
                stations.append(
                    (
                        [(x + normal[0] * t, y + normal[1] * t) for t in offsets],
                        heading,
                    )
                )
                break
            walked += geometry.length
    return stations


def _same_stream_coverage(
    covered: Road,
    overlay: Road,
    max_heading_diff_deg: float = DEFAULT_MAX_HEADING_DIFF_DEG,
) -> float:
    """Fraction of ``covered``'s carriageway that ``overlay`` runs along.

    A station counts only when one of its lane centres lands within
    :data:`_LATERAL_TOLERANCE` of a lane centre of ``overlay`` *and* the two
    tangents agree — an opposing carriageway is not an overlay.
    """
    ours = _band_stations(covered)
    theirs = _band_stations(overlay)
    if not ours or not theirs:
        return 0.0
    limit = math.radians(max_heading_diff_deg)
    matched = 0
    for points, heading in ours:
        hit = False
        for point in points:
            for other_points, other_heading in theirs:
                delta = abs(
                    (other_heading - heading + math.pi) % (2 * math.pi) - math.pi
                )
                if delta > limit:
                    continue
                if any(
                    math.hypot(point[0] - q[0], point[1] - q[1]) < _LATERAL_TOLERANCE
                    for q in other_points
                ):
                    hit = True
                    break
            if hit:
                break
        if hit:
            matched += 1
    return matched / len(ours)


def _centre_polyline(road: Road) -> List[Tuple[float, float]]:
    """Reference-line polyline of a road, for crossing tests."""
    return [points[0] for points, _ in _band_stations(road)] if road else []


def _segments_cross(
    a1: Tuple[float, float],
    a2: Tuple[float, float],
    b1: Tuple[float, float],
    b2: Tuple[float, float],
) -> bool:
    """Do the two open segments properly intersect?"""
    d1 = (a2[0] - a1[0], a2[1] - a1[1])
    d2 = (b2[0] - b1[0], b2[1] - b1[1])
    denominator = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(denominator) < 1e-12:
        return False
    t = ((b1[0] - a1[0]) * d2[1] - (b1[1] - a1[1]) * d2[0]) / denominator
    u = ((b1[0] - a1[0]) * d1[1] - (b1[1] - a1[1]) * d1[0]) / denominator
    return 0.05 < t < 0.95 and 0.05 < u < 0.95


def _paths_cross(first: Road, second: Road) -> bool:
    """Do two connecting roads cross each other away from their endpoints?

    Endpoint neighbourhoods are excluded by the segment parameter window, so
    two branches leaving the same approach side by side do not count.
    """
    left = _centre_polyline(first)
    right = _centre_polyline(second)
    if len(left) < 2 or len(right) < 2:
        return False
    for index in range(len(left) - 1):
        for other in range(len(right) - 1):
            if _segments_cross(
                left[index], left[index + 1], right[other], right[other + 1]
            ):
                return True
    return False


def _linked_road_ids(road: Road) -> Set[int]:
    """Road ids this road links to directly (ignoring junction links)."""
    if road.link is None:
        return set()
    out: Set[int] = set()
    for end in (road.link.predecessor, road.link.successor):
        if end is not None and end.element_type == ElementType.ROAD:
            out.add(int(end.element_id))
    return out


def _endpoints(road: Road) -> Tuple[Optional[int], Optional[int]]:
    """``(predecessor_road_id, successor_road_id)`` for a connecting road."""
    if road.link is None:
        return (None, None)

    def road_id(end) -> Optional[int]:
        if end is None or end.element_type != ElementType.ROAD:
            return None
        return int(end.element_id)

    return (road_id(road.link.predecessor), road_id(road.link.successor))


def untag_straight_turn_lanelets(
    lanelet_map,
    *,
    tolerance_deg: float = DEFAULT_STRAIGHT_TURN_TOLERANCE_DEG,
) -> List[Tuple[int, str, float]]:
    """Drop ``turn_direction`` from lanelets that do not actually turn.

    Autoware tags a turn lane from where its **pocket opens**, not from where
    the turn begins, which is right for a planner ("this lane leads to a
    turn") but not the same statement as "this lanelet lies inside an
    intersection". The converter uses the attribute as its junction-lanelet
    criterion, so a pocket becomes a connecting road inside a synthesized
    junction; consecutive junction lanelets are then chain-merged, and the
    result is a connecting road that starts far upstream of the intersection
    and runs alongside the through carriageway.

    On the Odaiba clip 12 of the 21 tagged lanelets have a heading change
    below 25°. Six of those are ``straight`` inside the real intersection and
    are correct; the other six carry ``left``/``right`` while running dead
    straight, parallel to a sibling lane. Untagging exactly those shortens
    the longest connecting road from 119.3 m to 61.6 m and takes junction
    1000 from 13 connections to 12.

    ``straight`` is never removed — inside an intersection it is the correct
    description of a through movement.

    Args:
        lanelet_map: The loaded Lanelet2 map (mutated in place).
        tolerance_deg: Heading change below which a left/right lanelet is
            treated as an approach lane rather than an intersection interior.

    Returns:
        ``(lanelet_id, turn_direction, heading_change_deg)`` per untagged
        lanelet.
    """
    untagged: List[Tuple[int, str, float]] = []
    for lanelet in lanelet_map.laneletLayer:
        # lanelet2's AttributeMap has no ``get``; membership then lookup, and
        # the value is an Attribute wrapper rather than a plain string.
        if "turn_direction" not in lanelet.attributes:
            continue
        direction = str(lanelet.attributes["turn_direction"])
        if direction not in ("left", "right"):
            continue
        points = [(point.x, point.y) for point in lanelet.centerline]
        if len(points) < 3:
            continue
        entry = math.atan2(points[1][1] - points[0][1], points[1][0] - points[0][0])
        exit_ = math.atan2(points[-1][1] - points[-2][1], points[-1][0] - points[-2][0])
        change = math.degrees((exit_ - entry + math.pi) % (2 * math.pi) - math.pi)
        if abs(change) >= tolerance_deg:
            continue
        del lanelet.attributes["turn_direction"]
        untagged.append((lanelet.id, direction, change))
    return untagged


def _elevation_at(road: Road, s: float) -> Optional[float]:
    """Reference-line elevation of ``road`` at station ``s``."""
    profile = road.elevation_profile
    records = getattr(profile, "elevations", None) if profile is not None else None
    if not records:
        return None
    z = None
    for record in records:
        if record.s > s + 1e-9:
            break
        ds = s - record.s
        z = record.a + record.b * ds + record.c * ds * ds + record.d * ds**3
    return z


def align_connector_elevations(
    roads: Sequence[Road],
) -> List[Tuple[int, float, float]]:
    """Lift each connecting road onto the neighbouring road's surface level.

    A road's ``<elevationProfile>`` describes its **reference line**, which
    for this converter sits at the inner edge of the carriageway. A
    single-lane connecting road's reference line instead follows its own lane
    centre, so on a cambered road the two carry different heights for what is
    physically the same joint: on the Odaiba clip road 32 has a 1.7 % camber,
    and its four connectors start 0.000 / 0.049 / 0.125 / 0.189 m below its
    reference line — exactly proportional to how far their lane sits from it
    (1.5 / 4.9 / 8.4 / 11.4 m).

    Vissim renders a link flat across at the reference-line height (it
    imports no superelevation), so those differences show up as steps at the
    junction. Correcting each connecting road by a linear ramp — matching the
    neighbour's reference-line height at both contact points — removes them.
    Curvature is untouched; only ``a`` and ``b`` change, and the added
    gradient is under 1 % for the observed offsets.

    Returns ``(road_id, start_correction, end_correction)`` per adjusted road.
    """
    by_id = {road.id: road for road in roads}
    adjusted: List[Tuple[int, float, float]] = []

    for road in roads:
        if road.junction == -1 or road.link is None:
            continue
        profile = road.elevation_profile
        records = getattr(profile, "elevations", None) if profile is not None else None
        if not records or road.length <= 0.0:
            continue

        def target(end) -> Optional[float]:
            if end is None or end.element_type != ElementType.ROAD:
                return None
            neighbour = by_id.get(int(end.element_id))
            if neighbour is None:
                return None
            at = 0.0 if end.contact_point == ContactPoint.START else neighbour.length
            return _elevation_at(neighbour, at)

        own_start = _elevation_at(road, 0.0)
        own_end = _elevation_at(road, road.length)
        want_start = target(road.link.predecessor)
        want_end = target(road.link.successor)
        if own_start is None or own_end is None:
            continue
        # With only one neighbour known, shift by that offset alone.
        delta_start = want_start - own_start if want_start is not None else None
        delta_end = want_end - own_end if want_end is not None else None
        if delta_start is None and delta_end is None:
            continue
        if delta_start is None:
            delta_start = delta_end
        if delta_end is None:
            delta_end = delta_start
        if abs(delta_start) < 1e-9 and abs(delta_end) < 1e-9:
            continue

        slope = (delta_end - delta_start) / road.length
        for record in records:
            record.a += delta_start + slope * record.s
            record.b += slope
        adjusted.append((road.id, delta_start, delta_end))
    return adjusted


def _driving_lanes(road: Road) -> List:
    """Lanes of the road's first section, center lane excluded."""
    if road.lanes is None or not road.lanes.lane_sections:
        return []
    section = road.lanes.lane_sections[0]
    lanes = list(getattr(section, "left_lanes", {}).values())
    lanes += list(getattr(section, "right_lanes", {}).values())
    return lanes


def _rewire_lane_links(
    neighbour: Road,
    side: str,
    connector: Road,
    connection: Optional[Connection],
) -> None:
    """Re-express a neighbour's lane links against a now-direct connector.

    While the junction existed, lane correspondence was carried by
    ``<connection><laneLink from to>``; the bare ids in the neighbour's
    ``<lane><link>`` were only meaningful through that indirection. Once the
    road-level link points straight at one connecting road, those ids resolve
    against the wrong road — which is what makes a consumer fall back to
    connecting every lane to every lane (the tell-tale lattice of connectors
    over a plain road stretch).

    The junction's lane links are the authority for the incoming side, and
    the connector's own lane links for the outgoing side. A lane with no
    correspondence has its link cleared rather than left dangling.
    """
    if side == "successor":
        # neighbour lane -> connector lane, straight from the junction record.
        mapping = {
            int(link.from_lane): int(link.to_lane)
            for link in (connection.lane_links if connection is not None else [])
        }
    else:
        # connector lane -> neighbour lane, inverted from the connector's own
        # lane links.
        mapping = {}
        for lane in _driving_lanes(connector):
            end = getattr(lane, "successor", None)
            if end is not None:
                mapping[int(end.id)] = int(lane.lane_id)

    for lane in _driving_lanes(neighbour):
        target = mapping.get(int(lane.lane_id))
        setattr(lane, side, LaneLink(id=target) if target is not None else None)


@dataclass
class MergedRoadGroup:
    """Per-lane roads of one carriageway that were merged into one road."""

    base_road_id: int
    absorbed_road_ids: List[int]
    lane_count: int


def _lateral_offset(base: Road, other: Road, fractions=(0.0, 0.25, 0.5, 0.75, 1.0)):
    """Mean signed lateral offset of ``other``'s reference line from ``base``.

    Returns ``None`` when the two are not parallel over their length.
    """
    base_stations = _band_stations(base, len(fractions))
    other_stations = _band_stations(other, len(fractions))
    if len(base_stations) != len(other_stations) or not base_stations:
        return None
    offsets = []
    for (base_points, base_heading), (other_points, other_heading) in zip(
        base_stations, other_stations
    ):
        delta = abs((other_heading - base_heading + math.pi) % (2 * math.pi) - math.pi)
        if delta > math.radians(10.0):
            return None
        # Reference-line points are the first entry only when a lane sits at
        # t = 0, so recover them from the lane centre and its own offset.
        normal = (-math.sin(base_heading), math.cos(base_heading))
        offsets.append(
            (other_points[0][0] - base_points[0][0]) * normal[0]
            + (other_points[0][1] - base_points[0][1]) * normal[1]
        )
    return sum(offsets) / len(offsets)


def _reference_offset(base: Road, other: Road):
    """Lateral offset between the two roads' *reference lines*."""
    base_offsets = _lane_centre_offsets(base)
    other_offsets = _lane_centre_offsets(other)
    if not base_offsets or not other_offsets:
        return None
    lane_offset = _lateral_offset(base, other)
    if lane_offset is None:
        return None
    # _lateral_offset compared the first lane centre of each road; convert to
    # reference lines by removing each road's own first-lane offset.
    return lane_offset - other_offsets[0] + base_offsets[0]


def _carriageway_width(road: Road) -> float:
    """Total width of the road's driving lanes in the first section."""
    total = 0.0
    for lane in _driving_lanes(road):
        widths = getattr(lane, "widths", None) or []
        if widths:
            total += float(getattr(widths[0], "a", 0.0) or 0.0)
    return total


def _link_target(road: Road, side: str):
    end = getattr(road.link, side, None) if road.link is not None else None
    if end is None:
        return None
    return (end.element_type, int(end.element_id))


def merge_parallel_lane_roads(
    roads: List[Road],
    junctions: Sequence[Junction],
    *,
    lanelet_to_road_and_lane: Optional[Dict[int, Tuple[int, int]]] = None,
    lanelet_to_emitted_segments: Optional[Dict[int, List[dict]]] = None,
    length_tolerance: float = 0.2,
    adjacency_tolerance: float = 1.0,
) -> List[MergedRoadGroup]:
    """Merge per-lane roads of one carriageway back into a single road.

    The divergence synthesis can emit each lane of a carriageway as its own
    road: on the Odaiba clip roads 23 and 24 run 3.27 m apart with a 0.03°
    heading difference, share the same predecessor *and* successor, and are
    really the two- and one-lane halves of one three-lane carriageway. In
    Vissim they arrive as separate links, so the road visibly splits.

    Merging is restricted to roads that agree on **both** link ends, which is
    what makes it safe: the merged road inherits those links unchanged, so no
    movement has to be re-expressed. Candidates must also be parallel, of
    similar length, and laterally adjacent (the gap between the carriageways
    is within ``adjacency_tolerance``).

    The innermost member keeps its id, geometry and elevation — its reference
    line already lies at the inner edge of the merged carriageway — and the
    others contribute their lanes, renumbered outward. References from
    junction connections and neighbouring roads are retargeted with the
    matching lane shift, and the lanelet mapping is rewritten so the sidecar
    keeps pointing at the right lane.

    Returns one :class:`MergedRoadGroup` per merge.
    """
    by_id = {road.id: road for road in roads}

    # ---- candidate pairs -------------------------------------------------
    parent = {road.id: road.id for road in roads}

    def find(x: int) -> int:
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    for first, second in itertools.combinations(sorted(by_id), 2):
        left, right = by_id[first], by_id[second]
        if left.junction != right.junction:
            continue
        if _link_target(left, "predecessor") != _link_target(right, "predecessor"):
            continue
        if _link_target(left, "successor") != _link_target(right, "successor"):
            continue
        if _link_target(left, "predecessor") is None and (
            _link_target(left, "successor") is None
        ):
            continue
        longest = max(left.length, right.length)
        if longest <= 0.0 or abs(left.length - right.length) / longest > (
            length_tolerance
        ):
            continue
        offset = _reference_offset(left, right)
        if offset is None:
            continue
        # The road on the negative side spans [offset, offset + its width]; the
        # carriageways touch when that meets the other's reference line.
        expected = _carriageway_width(right) if offset < 0 else _carriageway_width(left)
        if abs(abs(offset) - expected) > adjacency_tolerance:
            continue
        a, b = find(first), find(second)
        if a != b:
            parent[b] = a

    groups: Dict[int, List[int]] = {}
    for road_id in sorted(by_id):
        groups.setdefault(find(road_id), []).append(road_id)

    merged: List[MergedRoadGroup] = []
    removed: Set[int] = set()
    # (old_road_id, old_lane_id) -> (new_road_id, new_lane_id)
    remap: Dict[Tuple[int, int], Tuple[int, int]] = {}

    for members in groups.values():
        if len(members) < 2:
            continue
        anchor = by_id[members[0]]
        ordered = []
        for road_id in members:
            offset = (
                0.0
                if road_id == anchor.id
                else _reference_offset(anchor, by_id[road_id])
            )
            if offset is None:
                ordered = []
                break
            ordered.append((offset, road_id))
        if len(ordered) != len(members):
            continue
        # Lanes run outward from the reference line, so the base is the member
        # the others sit *outside* of: the innermost one on the lane side.
        lanes_on_left = any(lane.lane_id > 0 for lane in _driving_lanes(anchor))
        ordered.sort(key=lambda item: item[0], reverse=not lanes_on_left)
        base = by_id[ordered[0][1]]
        base_section = base.lanes.lane_sections[0]
        sign = 1 if lanes_on_left else -1
        next_index = len(_driving_lanes(base))

        for _, road_id in ordered[1:]:
            member = by_id[road_id]
            member_lanes = sorted(
                _driving_lanes(member), key=lambda lane: abs(lane.lane_id)
            )
            for lane in member_lanes:
                next_index += 1
                old = (road_id, lane.lane_id)
                lane.lane_id = sign * next_index
                if lanes_on_left:
                    base_section.left_lanes[lane.lane_id] = lane
                else:
                    base_section.right_lanes[lane.lane_id] = lane
                remap[old] = (base.id, lane.lane_id)
            removed.add(road_id)

        merged.append(
            MergedRoadGroup(
                base_road_id=base.id,
                absorbed_road_ids=sorted(r for _, r in ordered[1:]),
                lane_count=next_index,
            )
        )

    if not merged:
        return merged

    # ---- retarget every reference to an absorbed road ---------------------
    def shifted_lane(old_road: int, old_lane: int) -> Optional[int]:
        target = remap.get((old_road, old_lane))
        return target[1] if target else None

    for road in roads:
        if road.link is None:
            continue
        for side, lane_side in (
            ("predecessor", "predecessor"),
            ("successor", "successor"),
        ):
            end = getattr(road.link, side)
            if (
                end is None
                or end.element_type != ElementType.ROAD
                or int(end.element_id) not in removed
            ):
                continue
            old_road = int(end.element_id)
            new_road = remap_road_of(old_road, remap)
            if new_road is None:
                continue
            end.element_id = new_road
            for lane in _driving_lanes(road):
                link = getattr(lane, lane_side, None)
                if link is None:
                    continue
                new_lane = shifted_lane(old_road, int(link.id))
                if new_lane is not None:
                    link.id = new_lane

    for junction in junctions:
        for connection in junction.connections:
            if int(connection.incoming_road) in removed:
                new_road = remap_road_of(int(connection.incoming_road), remap)
                if new_road is not None:
                    for lane_link in connection.lane_links:
                        new_lane = shifted_lane(
                            int(connection.incoming_road), int(lane_link.from_lane)
                        )
                        if new_lane is not None:
                            lane_link.from_lane = new_lane
                    connection.incoming_road = new_road

    roads[:] = [road for road in roads if road.id not in removed]

    # ---- keep the mapping sidecar pointing at the right lane -------------
    if lanelet_to_road_and_lane is not None:
        for lanelet_id, (road_id, lane_id) in list(lanelet_to_road_and_lane.items()):
            target = remap.get((road_id, lane_id))
            if target is not None:
                lanelet_to_road_and_lane[lanelet_id] = target
    if lanelet_to_emitted_segments is not None:
        for segments in lanelet_to_emitted_segments.values():
            for segment in segments:
                target = remap.get((segment.get("road_id"), segment.get("lane_id")))
                if target is not None:
                    segment["road_id"], segment["lane_id"] = target

    return merged


def reciprocate_lane_links(roads: Sequence[Road]) -> int:
    """Fill in the lane link the other side of a reciprocal road link asserts.

    Where two roads name each other at road level there is no ambiguity left,
    so a lane correspondence stated by one of them can be stated by both. The
    dissolve and merge passes leave such half-stated links behind — OpenDRIVE
    forced the one-sidedness while several branches still competed for the
    same endpoint, and after merging only one remains.

    Returns the number of lane links added.
    """
    by_id = {road.id: road for road in roads}
    added = 0
    for road in roads:
        if road.link is None or road.link.successor is None:
            continue
        end = road.link.successor
        if end.element_type != ElementType.ROAD:
            continue
        other = by_id.get(int(end.element_id))
        if other is None or other.link is None:
            continue
        back = (
            other.link.predecessor
            if end.contact_point == ContactPoint.START
            else other.link.successor
        )
        if (
            back is None
            or back.element_type != ElementType.ROAD
            or int(back.element_id) != road.id
        ):
            continue
        far_side = (
            "predecessor" if end.contact_point == ContactPoint.START else "successor"
        )
        for lane in _driving_lanes(road):
            link = getattr(lane, "successor", None)
            if link is None:
                continue
            target = _lane_by_id(other, int(link.id))
            if target is None or getattr(target, far_side, None) is not None:
                continue
            setattr(target, far_side, LaneLink(id=lane.lane_id))
            added += 1
    return added


def remap_road_of(
    old_road: int, remap: Dict[Tuple[int, int], Tuple[int, int]]
) -> Optional[int]:
    """The road an absorbed road's lanes moved into."""
    for (road_id, _), (new_road, _) in remap.items():
        if road_id == old_road:
            return new_road
    return None


def _lane_by_id(road: Road, lane_id: int):
    """Return the lane with ``lane_id`` in the road's first section."""
    for lane in _driving_lanes(road):
        if lane.lane_id == lane_id:
            return lane
    return None


def absorb_degenerate_stubs(
    roads: List[Road],
    junctions: Sequence[Junction],
    *,
    min_length: float = VISSIM_MIN_CONNECTOR_LENGTH,
) -> List[Tuple[int, int, int, bool]]:
    """Replace 1 cm stub roads with a direct link between their neighbours.

    The divergence synthesis emits a 0.01 m connecting road per lane-level
    movement so CARLA's loader has something to follow. In Vissim such a road
    becomes a 1 cm link — below its 0.5 m minimum spline spacing — and the
    movement through it is unreliable. Because the stub is 1 cm long,
    deleting it and joining its neighbours directly changes the geometry by
    at most that centimetre.

    Only stubs outside a junction are touched (the junction dissolve runs
    first), and only when a road-level link is still free to carry the
    movement: OpenDRIVE allows one predecessor and one successor per road, so
    at a merge the second branch is expressed from whichever side is
    available. A stub whose movement cannot be expressed at all is kept.

    Returns ``(stub_id, from_road, to_road, reciprocal)`` per absorbed stub.
    """
    in_junction = {
        int(connection.connecting_road)
        for junction in junctions
        for connection in junction.connections
    }
    by_id = {road.id: road for road in roads}
    absorbed: List[Tuple[int, int, int, bool]] = []
    removed: Set[int] = set()

    for stub in sorted(roads, key=lambda r: r.id):
        if (
            stub.id in in_junction
            or stub.junction != -1
            or stub.length >= min_length
            or stub.link is None
        ):
            continue
        upstream_id, downstream_id = _endpoints(stub)
        if upstream_id is None or downstream_id is None:
            continue
        upstream = by_id.get(upstream_id)
        downstream = by_id.get(downstream_id)
        if upstream is None or downstream is None:
            continue

        # (upstream lane -> downstream lane) pairs the stub carried.
        movements: List[Tuple[int, int]] = []
        for lane in _driving_lanes(stub):
            entry = getattr(lane, "predecessor", None)
            exit_ = getattr(lane, "successor", None)
            if entry is not None and exit_ is not None:
                movements.append((int(entry.id), int(exit_.id)))
        if not movements:
            continue

        def names(road: Road, side: str, target: int) -> bool:
            end = getattr(road.link, side, None) if road.link is not None else None
            return (
                end is not None
                and end.element_type == ElementType.ROAD
                and int(end.element_id) == target
            )

        forward_free = upstream.link is not None and (
            upstream.link.successor is None
            or names(upstream, "successor", stub.id)
            or names(upstream, "successor", downstream_id)
        )
        backward_free = downstream.link is not None and (
            downstream.link.predecessor is None
            or names(downstream, "predecessor", stub.id)
            or names(downstream, "predecessor", upstream_id)
        )
        if not forward_free and not backward_free:
            continue

        if forward_free:
            upstream.link.successor = Successor(
                ElementType.ROAD, downstream_id, ContactPoint.START
            )
            for from_lane, to_lane in movements:
                lane = _lane_by_id(upstream, from_lane)
                if lane is not None:
                    lane.successor = LaneLink(id=to_lane)
        if backward_free:
            downstream.link.predecessor = Predecessor(
                ElementType.ROAD, upstream_id, ContactPoint.END
            )
            for from_lane, to_lane in movements:
                lane = _lane_by_id(downstream, to_lane)
                if lane is not None:
                    lane.predecessor = LaneLink(id=from_lane)

        removed.add(stub.id)
        absorbed.append(
            (stub.id, upstream_id, downstream_id, forward_free and backward_free)
        )

    if removed:
        roads[:] = [road for road in roads if road.id not in removed]
    return absorbed


def _is_intersection(
    junction: Junction,
    by_id: Dict[int, Road],
    min_arms: int,
) -> Tuple[bool, int, int]:
    """Classify a junction; returns ``(is_intersection, arms, crossing_pairs)``.

    A junction is an intersection when traffic streams actually meet: either
    it has ``min_arms`` or more distinct approaches, or two of its movements
    cross. A pure merge (several approaches, one exit) or diverge (one
    approach, several exits) is neither — it is a continuation of a road, and
    wrapping it in a junction makes Vissim place a node where there is no
    intersection.
    """
    arms = {int(connection.incoming_road) for connection in junction.connections}
    crossings = 0
    connections = junction.connections
    for index, first in enumerate(connections):
        for second in connections[index + 1 :]:
            if int(first.incoming_road) == int(second.incoming_road):
                continue
            left = by_id.get(int(first.connecting_road))
            right = by_id.get(int(second.connecting_road))
            if left is None or right is None or left is right:
                continue
            if _paths_cross(left, right):
                crossings += 1
    return (len(arms) >= min_arms or crossings > 0, len(arms), crossings)


def dissolve_non_intersection_junctions(
    roads: Sequence[Road],
    junctions: List[Junction],
    *,
    min_arms: int = DEFAULT_MIN_INTERSECTION_ARMS,
) -> VissimTopologyReport:
    """Turn merge/diverge junctions into ordinary road links.

    Vissim creates one node per ``<junction>``, and a node brings the whole
    intersection machinery with it — conflict areas, priority rules, reduced
    speed areas. The divergence synthesis wraps every lane-level merge and
    diverge in a junction, so a plain widening or an off-ramp arrives in
    Vissim as an intersection. On the Odaiba clip only one of six junctions
    is a real intersection.

    For each junction with no crossing movement and fewer than ``min_arms``
    approaches, its connecting roads become ordinary roads (``junction=-1``)
    and every road that pointed at the junction is repointed at the
    connecting road that continues it — the branch carrying the most lane
    links. Secondary branches keep their own predecessor/successor links, so
    Vissim still builds a connector for them: it generates connectors from
    ``link::predecessor``/``link::successor``, which is why the movement
    survives even though OpenDRIVE lets the neighbouring road name only one
    of them.

    Road and lane ids are untouched, so the ``*.mapping.json`` sidecar stays
    valid.

    Args:
        roads: All roads about to be emitted.
        junctions: All junctions about to be emitted (mutated in place).
        min_arms: Distinct approaches at which a junction counts as an
            intersection regardless of crossings.

    Returns:
        A :class:`VissimTopologyReport` recording dissolved and kept
        junctions.
    """
    report = VissimTopologyReport()
    by_id = {road.id: road for road in roads}
    survivors: List[Junction] = []

    for junction in junctions:
        intersection, arms, crossings = _is_intersection(junction, by_id, min_arms)
        if intersection:
            survivors.append(junction)
            report.kept_junctions.append((junction.id, arms, crossings))
            continue

        connecting_ids = [
            int(connection.connecting_road) for connection in junction.connections
        ]
        # Lane-link count per connecting road decides which branch a
        # neighbouring road should name as its predecessor/successor.
        weight = {
            int(connection.connecting_road): len(connection.lane_links)
            for connection in junction.connections
        }
        connection_of_connector = {
            int(connection.connecting_road): connection
            for connection in junction.connections
        }

        for connecting_id in connecting_ids:
            connector = by_id.get(connecting_id)
            if connector is not None:
                connector.junction = -1

        for road in roads:
            if road.link is None:
                continue
            for side, opposite in (
                ("predecessor", "successor"),
                ("successor", "predecessor"),
            ):
                end = getattr(road.link, side)
                if (
                    end is None
                    or end.element_type != ElementType.JUNCTION
                    or int(end.element_id) != junction.id
                ):
                    continue
                # Candidates: connecting roads of this junction that name this
                # road on the matching side.
                candidates = []
                for connecting_id in connecting_ids:
                    connector = by_id.get(connecting_id)
                    if connector is None or connector.link is None:
                        continue
                    far = getattr(connector.link, opposite)
                    if (
                        far is not None
                        and far.element_type == ElementType.ROAD
                        and int(far.element_id) == road.id
                    ):
                        candidates.append(connecting_id)
                if not candidates:
                    setattr(road.link, side, None)
                    for lane in _driving_lanes(road):
                        setattr(lane, side, None)
                    continue
                primary = max(candidates, key=lambda rid: (weight.get(rid, 0), -rid))
                contact = (
                    ContactPoint.END if side == "predecessor" else ContactPoint.START
                )
                if side == "predecessor":
                    road.link.predecessor = Predecessor(
                        ElementType.ROAD, primary, contact
                    )
                else:
                    road.link.successor = Successor(ElementType.ROAD, primary, contact)
                # The bare lane ids were relative to the junction; re-express
                # them against the connecting road the link now names.
                _rewire_lane_links(
                    road,
                    side,
                    by_id[primary],
                    connection_of_connector.get(primary),
                )

        report.dissolved_junctions.append(
            DissolvedJunction(
                junction_id=junction.id,
                name=junction.name,
                incoming_roads=sorted(
                    {int(c.incoming_road) for c in junction.connections}
                ),
                connecting_roads=sorted(connecting_ids),
            )
        )

    junctions[:] = survivors
    return report


def analyze_topology(
    roads: Sequence[Road],
    junctions: Sequence[Junction],
    *,
    min_link_coverage: float = DEFAULT_MIN_LINK_COVERAGE,
    min_connector_length: float = VISSIM_MIN_CONNECTOR_LENGTH,
) -> VissimTopologyReport:
    """Report Vissim-relevant topology problems without modifying anything.

    Two categories are collected:

    * connecting roads that run along a through road in the same direction
      over at least ``min_link_coverage`` of that road — Vissim covers the
      stretch with a conflict area whose priority it cannot determine,
      because both sides are really the same traffic stream;
    * connecting roads shorter than ``min_connector_length``, which fall
      below Vissim's minimum spline-point spacing. For each, whether the
      junction is structurally required is reported: it is needed when the
      roads it joins form a merge or a diverge (OpenDRIVE allows a road only
      one predecessor and one successor), and otherwise a direct
      road-to-road link would express the same movement without a degenerate
      connector.

    Args:
        roads: All roads about to be emitted.
        junctions: All junctions about to be emitted.
        min_link_coverage: Coverage above which an overlay is reported.
        min_connector_length: Length below which a connector is degenerate.

    Returns:
        A :class:`VissimTopologyReport`.
    """
    report = VissimTopologyReport()
    by_id = {road.id: road for road in roads}
    through = [road for road in roads if road.junction == -1]
    junction_of_connector = {
        int(connection.connecting_road): junction
        for junction in junctions
        for connection in junction.connections
    }

    # How many connecting roads terminate at each road endpoint: more than
    # one means the junction expresses a merge or a diverge there, which a
    # single predecessor/successor pair cannot represent.
    attachments: Dict[Tuple[int, str], int] = defaultdict(int)
    for connector_id in junction_of_connector:
        connector = by_id.get(connector_id)
        if connector is None:
            continue
        predecessor, successor = _endpoints(connector)
        if predecessor is not None:
            attachments[(predecessor, "out")] += 1
        if successor is not None:
            attachments[(successor, "in")] += 1

    for connector_id, junction in sorted(junction_of_connector.items()):
        connector = by_id.get(connector_id)
        if connector is None or connector.junction == -1:
            continue

        if connector.length < min_connector_length:
            predecessor, successor = _endpoints(connector)
            report.degenerate_connectors.append(
                DegenerateConnector(
                    road_id=connector_id,
                    junction_id=junction.id,
                    length=connector.length,
                    from_road=predecessor if predecessor is not None else -1,
                    to_road=successor if successor is not None else -1,
                    junction_required=(
                        attachments.get((predecessor, "out"), 0) > 1
                        or attachments.get((successor, "in"), 0) > 1
                    ),
                )
            )

        skip = _linked_road_ids(connector) | {connector_id}
        for candidate in through:
            if candidate.id in skip:
                continue
            coverage = _same_stream_coverage(candidate, connector)
            if coverage >= min_link_coverage:
                report.overlaps.append(
                    SameStreamOverlap(
                        road_id=candidate.id,
                        connector_id=connector_id,
                        junction_id=junction.id,
                        coverage=coverage,
                    )
                )

    return report
