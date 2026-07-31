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

import logging
import math
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Set, Tuple

from .opendrive.enums import ContactPoint, ElementType
from .opendrive.junction import Junction
from .opendrive.road import Road
from .opendrive.road_links import Predecessor, Successor

logger = logging.getLogger(__name__)

#: Fraction of a through road that must be covered by a connecting road
#: before the connector counts as running along that road.
DEFAULT_MIN_LINK_COVERAGE = 0.4

#: A junction with at least this many distinct incoming roads is treated as a
#: real intersection even when no two of its movements cross (an approach
#: whose turns all leave on different arms still needs the node).
DEFAULT_MIN_INTERSECTION_ARMS = 3

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
