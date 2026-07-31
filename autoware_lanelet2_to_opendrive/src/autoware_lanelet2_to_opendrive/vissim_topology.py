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

from .config import DEFAULT_CONFIG
from .opendrive.enums import ContactPoint, ElementType
from .opendrive.junction import Connection, Junction
from .opendrive.road import Road
from .opendrive.lane_elements import LaneLink
from .opendrive.road_links import Predecessor, RoadLink, Successor

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

#: A road this short is a fragment rather than a link. Dissolving a junction
#: turns its connecting roads into ordinary roads, and a connector that was
#: naturally a metre or two long then arrives as a link that short — which is
#: what a fragmented network looks like. Absorbing it hands the stretch back
#: to a connector, which is where a couple of metres belongs.
DEFAULT_ABSORB_MAX_LENGTH = 3.0

#: Two carriageways only belong to the same traffic stream when their
#: tangents agree within this angle. Without the gate, the opposing
#: carriageway of a two-way road — whose reference line can run within a
#: metre of ours in the other direction — would look like an overlay.
DEFAULT_MAX_HEADING_DIFF_DEG = 45.0

#: Stations sampled along each road when measuring coverage.
_STATIONS_PER_ROAD = 60

#: Lateral match tolerance (m) now lives in
#: :class:`~.config.VissimTopologyConstants`; read through
#: ``DEFAULT_CONFIG.vissim_topology.lateral_tolerance``. Kept as an alias so
#: the docstrings that name it stay resolvable.
_LATERAL_TOLERANCE = DEFAULT_CONFIG.vissim_topology.lateral_tolerance

# Sentinel upstream lane for a fan recognised from its endpoint pair alone.
_ANY_LANE = 0

# Any overlapping station at all, for the endpoint-pair reading of a fan.
_ANY_OVERLAP = 1e-9


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
class OverlappingRoads:
    """Two through roads whose lanes occupy the same space, same direction."""

    first_road_id: int
    second_road_id: int
    overlap_length: float
    first_lanes: List[int]
    second_lanes: List[int]
    same_origin_destination: bool


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
    overlapping_roads: List[OverlappingRoads] = field(default_factory=list)
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

        if self.overlapping_roads:
            log.warning(
                "Vissim topology: %d through-road pair(s) have lanes occupying "
                "the same space in the same direction — Vissim renders both "
                "links there, so that stretch has one more lane than the "
                "ground truth and vehicles pass through each other (an "
                "auto-generated conflict area defaults to passive). This comes "
                "from overlapping lanelets in the source map and has to be "
                "fixed there: a turn pocket must start where it separates from "
                "the through lane, or share a boundary with it.",
                len(self.overlapping_roads),
            )
            for pair in self.overlapping_roads:
                log.warning(
                    "  road %d lane(s) %s overlaps road %d lane(s) %s for " "%.1f m%s",
                    pair.first_road_id,
                    pair.first_lanes,
                    pair.second_road_id,
                    pair.second_lanes,
                    pair.overlap_length,
                    (
                        " (same origin and destination — a duplicated lane)"
                        if pair.same_origin_destination
                        else ""
                    ),
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


def _lane_centre_widths(road: Road) -> List[float]:
    """Widths of the driving lanes, in the order :func:`_lane_centre_offsets`
    returns their centres — so index ``i`` of one indexes the other."""
    if road.lanes is None or not road.lanes.lane_sections:
        return []
    section = road.lanes.lane_sections[0]

    def width_of(lane) -> float:
        widths = getattr(lane, "widths", None) or []
        return float(getattr(widths[0], "a", 0.0) or 0.0) if widths else 0.0

    widths = [
        width_of(section.left_lanes[lane_id])
        for lane_id in sorted(getattr(section, "left_lanes", {}))
    ]
    widths += [
        width_of(section.right_lanes[lane_id])
        for lane_id in sorted(getattr(section, "right_lanes", {}), reverse=True)
    ]
    return widths


def _road_length(road: Road) -> float:
    """Length along the plan view, which is what the stations are spread over."""
    if road.plan_view is None or not road.plan_view.geometries:
        return 0.0
    return sum(geometry.length for geometry in road.plan_view.geometries)


@dataclass(frozen=True)
class OverlapMeasurement:
    """Every reading of "these two roads overlap", measured in one pass.

    Only :attr:`coverage` drives a decision. The other fields exist because the
    criterion the passes act on — lane centres closer than
    ``lateral_tolerance`` — is not the quantity PTV's rule tests, and which
    reading of that rule applies has not been settled by measurement yet:

    * :attr:`max_overlap_width` is the lateral width of the overlapping
      surface, ``(w1 + w2) / 2 - d``. Reading X compares it to
      ``conflict_overlap_width``.
    * :attr:`overlap_length` is how far along ``covered`` the two surfaces
      overlap at all. Reading Y compares it to ``conflict_overlap_length``.

    The two are independent: surfaces can graze over a long stretch (large
    length, small width) or cross deeply over a short one.
    """

    coverage: float
    min_centre_distance: Optional[float]
    max_overlap_width: float
    overlap_length: float
    matched_stations: int
    overlap_stations: int
    station_spacing: float

    @property
    def interpretation_x(self) -> bool:
        """Whether reading X says Vissim raises a conflict area."""
        return (
            self.max_overlap_width
            > DEFAULT_CONFIG.vissim_topology.conflict_overlap_width
        )

    @property
    def interpretation_y(self) -> bool:
        """Whether reading Y says Vissim raises a conflict area."""
        return (
            self.overlap_length > DEFAULT_CONFIG.vissim_topology.conflict_overlap_length
        )

    @property
    def current_criterion(self) -> bool:
        """Whether the passes treat the pair as overlapping today."""
        return self.coverage > 0.0


def measure_stream_overlap(
    covered: Road,
    overlay: Road,
    max_heading_diff_deg: float = DEFAULT_MAX_HEADING_DIFF_DEG,
) -> OverlapMeasurement:
    """Measure how ``overlay`` runs along ``covered`` under every reading.

    ``coverage`` reproduces :func:`_same_stream_coverage` exactly — a station
    counts when one of its lane centres lands within ``lateral_tolerance`` of a
    lane centre of ``overlay`` and the two tangents agree, so an opposing
    carriageway is not an overlay. The remaining fields are recorded from the
    same station pairs and change nothing.
    """
    ours = _band_stations(covered)
    theirs = _band_stations(overlay)
    our_widths = _lane_centre_widths(covered)
    their_widths = _lane_centre_widths(overlay)
    spacing = (_road_length(covered) / (len(ours) - 1)) if len(ours) > 1 else 0.0
    if not ours or not theirs:
        return OverlapMeasurement(0.0, None, 0.0, 0.0, 0, 0, spacing)

    tolerance = DEFAULT_CONFIG.vissim_topology.lateral_tolerance
    limit = math.radians(max_heading_diff_deg)
    matched = 0
    overlapping = 0
    closest: Optional[float] = None
    widest = 0.0
    for points, heading in ours:
        hit = False
        surfaces_meet = False
        for index, point in enumerate(points):
            our_width = our_widths[index] if index < len(our_widths) else 0.0
            for other_points, other_heading in theirs:
                delta = abs(
                    (other_heading - heading + math.pi) % (2 * math.pi) - math.pi
                )
                if delta > limit:
                    continue
                for other_index, q in enumerate(other_points):
                    distance = math.hypot(point[0] - q[0], point[1] - q[1])
                    if distance < tolerance:
                        hit = True
                        if closest is None or distance < closest:
                            closest = distance
                    their_width = (
                        their_widths[other_index]
                        if other_index < len(their_widths)
                        else 0.0
                    )
                    overlap = (our_width + their_width) / 2.0 - distance
                    if overlap > 0.0:
                        surfaces_meet = True
                        widest = max(widest, overlap)
        if hit:
            matched += 1
        if surfaces_meet:
            overlapping += 1
    return OverlapMeasurement(
        coverage=matched / len(ours),
        min_centre_distance=closest,
        max_overlap_width=widest,
        overlap_length=overlapping * spacing,
        matched_stations=matched,
        overlap_stations=overlapping,
        station_spacing=spacing,
    )


def _same_stream_coverage(
    covered: Road,
    overlay: Road,
    max_heading_diff_deg: float = DEFAULT_MAX_HEADING_DIFF_DEG,
) -> float:
    """Fraction of ``covered``'s carriageway that ``overlay`` runs along.

    The decision criterion, unchanged: see :func:`measure_stream_overlap`,
    which computes it along with the readings that are only recorded.
    """
    return measure_stream_overlap(covered, overlay, max_heading_diff_deg).coverage


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


def _find_overlapping_roads(
    roads: Sequence[Road],
    *,
    min_overlap_length: float = 2.0,
) -> List[OverlappingRoads]:
    """Find through roads whose lanes occupy the same space, same direction.

    Two lanes of *different* roads sharing ground is not something OpenDRIVE
    can express away: within one road, lanes are stacked side by side by
    width, so a lane cannot lie on top of another. It only arises when the
    source map already has overlapping lanelets — on the Odaiba clip the turn
    pockets do, e.g. lanelets 1494 and 1514 start at the same coordinate and
    50 % of each centreline runs inside the other polygon, with no shared
    boundary.

    Vissim renders both links, so the stretch carries one lane more than the
    ground truth and its auto-generated conflict area defaults to passive,
    letting vehicles pass through each other. The repair belongs in the map,
    which is why this is reported rather than patched.
    """
    stations = {road.id: _band_stations(road) for road in roads}
    found: List[OverlappingRoads] = []

    for first, second in itertools.combinations(
        [road for road in roads if road.junction == -1], 2
    ):
        if second.id in _linked_road_ids(first) or first.id in (
            _linked_road_ids(second)
        ):
            continue
        ours, theirs = stations[first.id], stations[second.id]
        if not ours or not theirs or first.length <= 0.0:
            continue
        widths = {
            lane.lane_id: float((getattr(lane, "widths", None) or [None])[0].a)
            for road in (first, second)
            for lane in _driving_lanes(road)
            if getattr(lane, "widths", None)
        }
        first_ids = [lane.lane_id for lane in _driving_lanes(first)]
        second_ids = [lane.lane_id for lane in _driving_lanes(second)]
        if not first_ids or not second_ids:
            continue

        step = first.length / max(len(ours) - 1, 1)
        length = 0.0
        hit_first: Set[int] = set()
        hit_second: Set[int] = set()
        for points, heading in ours:
            touched = False
            for index, point in enumerate(points):
                lane_id = first_ids[index] if index < len(first_ids) else None
                for other_points, other_heading in theirs:
                    delta = abs(
                        (other_heading - heading + math.pi) % (2 * math.pi) - math.pi
                    )
                    if delta > math.radians(30.0):
                        continue
                    for other_index, other in enumerate(other_points):
                        other_id = (
                            second_ids[other_index]
                            if other_index < len(second_ids)
                            else None
                        )
                        tolerance = (
                            min(
                                widths.get(lane_id, 3.5),
                                widths.get(other_id, 3.5),
                            )
                            * 0.5
                        )
                        if (
                            math.hypot(point[0] - other[0], point[1] - other[1])
                            < tolerance
                        ):
                            touched = True
                            if lane_id is not None:
                                hit_first.add(lane_id)
                            if other_id is not None:
                                hit_second.add(other_id)
            if touched:
                length += step
        if length < min_overlap_length:
            continue
        found.append(
            OverlappingRoads(
                first_road_id=first.id,
                second_road_id=second.id,
                overlap_length=length,
                first_lanes=sorted(hit_first),
                second_lanes=sorted(hit_second),
                same_origin_destination=(
                    _link_target(first, "predecessor")
                    == _link_target(second, "predecessor")
                    and _link_target(first, "successor")
                    == _link_target(second, "successor")
                ),
            )
        )
    return found


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


def link_isolated_roads(
    roads: Sequence[Road],
    *,
    tolerance: float = 0.5,
    max_heading_diff_deg: float = 30.0,
) -> List[Tuple[int, int, float]]:
    """Give back the links of roads the vehicle routing graph never saw.

    Road-level links are derived from the routing graph, which is built for a
    vehicle participant, so a shoulder or bicycle carriageway — which no vehicle
    traffic rule admits — arrives with no ``<link>`` at all even where the source
    lanelets share a boundary and plainly continue one another. On the Odaiba
    clip that left four such successions unstated (roads 3->4 bicycle, 25->14,
    11->12 and 12->21 shoulder), and each road rendered as a patch floating on
    its own with a gap to the next.

    Only roads with no link on either end are considered, so nothing that the
    conversion did state can be disturbed. A succession is asserted when one
    road's far endpoint lands within ``tolerance`` of another's near endpoint
    and the two tangents agree — on the Odaiba clip those endpoints coincide
    exactly, the gap being purely the missing statement.

    Returns ``(from_road, to_road, endpoint_gap)`` per link added.
    """
    isolated = [
        road
        for road in roads
        if road.link is None
        or (road.link.predecessor is None and road.link.successor is None)
    ]
    poses = {}
    for road in isolated:
        stations = _band_stations(road)
        if len(stations) < 2:
            continue
        poses[road.id] = (
            (stations[0][0][0], stations[0][1]),
            (stations[-1][0][0], stations[-1][1]),
        )
    limit = math.radians(max_heading_diff_deg)
    by_id = {road.id: road for road in isolated if road.id in poses}
    added: List[Tuple[int, int, float]] = []
    for first in sorted(by_id):
        upstream = by_id[first]
        if upstream.link is not None and upstream.link.successor is not None:
            continue
        (_, _), (end_point, end_heading) = poses[first]
        best = None
        for second in sorted(by_id):
            if second == first:
                continue
            downstream = by_id[second]
            if downstream.link is not None and downstream.link.predecessor is not None:
                continue
            (start_point, start_heading), (_, _) = poses[second]
            delta = abs(
                (start_heading - end_heading + math.pi) % (2 * math.pi) - math.pi
            )
            if delta > limit:
                continue
            gap = math.hypot(
                end_point[0] - start_point[0], end_point[1] - start_point[1]
            )
            if gap <= tolerance and (best is None or gap < best[1]):
                best = (second, gap)
        if best is None:
            continue
        second, gap = best
        downstream = by_id[second]
        if upstream.link is None:
            upstream.link = RoadLink()
        if downstream.link is None:
            downstream.link = RoadLink()
        upstream.link.successor = Successor(
            ElementType.ROAD, downstream.id, ContactPoint.START
        )
        downstream.link.predecessor = Predecessor(
            ElementType.ROAD, upstream.id, ContactPoint.END
        )
        # Lane correspondence by centre offset: the carriageways continue one
        # another, so the lane at the same distance from the reference line is
        # the same lane.
        upstream_offsets = dict(
            zip(
                [lane.lane_id for lane in _driving_lanes(upstream)],
                _lane_centre_offsets(upstream),
            )
        )
        downstream_offsets = dict(
            zip(
                [lane.lane_id for lane in _driving_lanes(downstream)],
                _lane_centre_offsets(downstream),
            )
        )
        for lane in _driving_lanes(upstream):
            ours = upstream_offsets.get(lane.lane_id)
            if ours is None or not downstream_offsets:
                continue
            match = min(
                downstream_offsets,
                key=lambda other: abs(downstream_offsets[other] - ours),
            )
            lane.successor = LaneLink(id=match)
            target = _lane_by_id(downstream, match)
            if target is not None:
                target.predecessor = LaneLink(id=lane.lane_id)
        added.append((upstream.id, downstream.id, gap))
    return added


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


@dataclass
class CollapsedFan:
    """A lane-choice fan reduced to the one path Vissim can represent.

    ``lane_change_targets`` are the ``(road_id, lane_id)`` pairs that lost
    their feed and are now reached by changing lanes on the downstream road.
    """

    predecessor_road_id: int
    predecessor_lane_id: int
    successor_road_id: int
    kept_road_id: int
    dropped_road_ids: List[int]
    lane_change_targets: List[Tuple[int, int]] = field(default_factory=list)


def _is_road_element(end) -> bool:
    return end is not None and end.element_type == ElementType.ROAD


def _fan_key(road: Road) -> Optional[Tuple[int, int, int]]:
    """``(upstream road, upstream lane, downstream road)`` this road bridges.

    ``None`` unless both ends name a plain road and every driving lane draws
    from the *same* upstream lane — a road that carries several lanes of a
    carriageway forward is not one branch of a lane choice.
    """
    if road.link is None:
        return None
    start = road.link.predecessor
    end = road.link.successor
    if not _is_road_element(start) or not _is_road_element(end):
        return None
    upstream = {
        int(link.id)
        for lane in _driving_lanes(road)
        if (link := getattr(lane, "predecessor", None)) is not None
    }
    if len(upstream) != 1:
        return None
    return int(start.element_id), upstream.pop(), int(end.element_id)


def _fan_survivor(group: Sequence[Road], upstream_lane: int) -> Road:
    """The branch that carries the through movement of a lane-choice fan.

    Preference order: the branch carrying the most lanes forward, then the one
    whose downstream lane keeps the upstream lane's index (the straight
    continuation rather than a sideways move into a pocket), then the longest,
    then the lowest id — so the choice does not depend on road ordering.
    """

    def rank(road: Road) -> Tuple[int, int, float, int]:
        continues = any(
            (link := getattr(lane, "successor", None)) is not None
            and int(link.id) == upstream_lane
            for lane in _driving_lanes(road)
        )
        return (
            len(_driving_lanes(road)),
            1 if continues else 0,
            float(road.length or 0.0),
            -road.id,
        )

    return max(group, key=rank)


def _detach_dropped_roads(
    roads: List[Road],
    junctions: Sequence[Junction],
    removed: Set[int],
    lanelet_to_road_and_lane: Optional[Dict[int, Tuple[int, int]]],
    lanelet_to_emitted_segments: Optional[Dict[int, List[dict]]],
    replacement: Optional[Dict[int, int]] = None,
) -> None:
    """Remove roads and repoint every reference that would be left dangling.

    A neighbour naming a dropped branch is retargeted at the branch that
    survived it — the two bridge the same pair of endpoints, so the neighbour
    stays attached. Only its lane correspondence has to be restated, since the
    surviving branch lands in different lanes. Clearing the reference instead
    would strand the neighbour: on the Odaiba clip road 27 named road 28 as its
    predecessor and would be left with a link on neither end.
    """
    replacement = replacement or {}
    roads[:] = [road for road in roads if road.id not in removed]
    by_id = {road.id: road for road in roads}
    for road in roads:
        if road.link is None:
            continue
        for side in ("predecessor", "successor"):
            end = getattr(road.link, side, None)
            if not _is_road_element(end) or int(end.element_id) not in removed:
                continue
            survivor = by_id.get(replacement.get(int(end.element_id), -1))
            if survivor is None:
                setattr(road.link, side, None)
                for lane in _driving_lanes(road):
                    setattr(lane, side, None)
                continue
            end.element_id = survivor.id
            # The survivor lands in other lanes, so restate the correspondence
            # from its own lane links rather than keeping the dropped one's.
            far = "successor" if side == "predecessor" else "predecessor"
            mapping = {
                int(link.id): int(lane.lane_id)
                for lane in _driving_lanes(survivor)
                if (link := getattr(lane, far, None)) is not None
            }
            for lane in _driving_lanes(road):
                target = mapping.get(int(lane.lane_id))
                setattr(lane, side, LaneLink(id=target) if target is not None else None)
    for junction in junctions:
        junction.connections = [
            connection
            for connection in junction.connections
            if int(connection.incoming_road) not in removed
            and int(connection.connecting_road) not in removed
        ]
        # <priority> names connecting roads, so it dangles the moment one goes.
        # A reference to a road that is not in the file is worse than no
        # right-of-way statement at all: a consumer that resolves it finds
        # nothing, and a schema check flags the junction.
        junction.priorities = [
            priority
            for priority in junction.priorities
            if int(priority.high) not in removed and int(priority.low) not in removed
        ]
    if lanelet_to_road_and_lane is not None:
        for lanelet_id in [
            lanelet_id
            for lanelet_id, (road_id, _) in lanelet_to_road_and_lane.items()
            if road_id in removed
        ]:
            del lanelet_to_road_and_lane[lanelet_id]
    if lanelet_to_emitted_segments is not None:
        for lanelet_id, segments in list(lanelet_to_emitted_segments.items()):
            kept = [s for s in segments if s.get("road_id") not in removed]
            if kept:
                lanelet_to_emitted_segments[lanelet_id] = kept
            else:
                del lanelet_to_emitted_segments[lanelet_id]


def collapse_lane_choice_fans(
    roads: List[Road],
    junctions: Sequence[Junction],
    *,
    lanelet_to_road_and_lane: Optional[Dict[int, Tuple[int, int]]] = None,
    lanelet_to_emitted_segments: Optional[Dict[int, List[dict]]] = None,
    min_overlap_fraction: float = 0.25,
) -> List[CollapsedFan]:
    """Collapse overlapping branches that only differ in the lane they land in.

    Lanelet2 routinely draws "this lane may end up in any of those lanes" as
    one lanelet per destination, all starting from the same cross-section. On
    the Odaiba clip lanelets 1363/1372/1373 leave the same 3.63 m lane and stay
    within 0.10 m of each other for half their length before splitting to three
    adjacent lanes; transcribed one road per lanelet that becomes three roads
    laid on top of one another.

    OpenDRIVE has a proper representation for the downstream lanes those
    branches feed: a lane simply *starts*, with no predecessor, and is entered
    by changing lanes. Outside a junction the transcription is not even legal —
    two roads named the same successor while that road can only name one
    predecessor, so the reverse link was missing on one of them. Inside a
    junction it is legal but unusable: Vissim raises a conflict area for every
    overlapping pair of connectors, which is what fills the network with
    priorities in places where no two vehicles can meet.

    A branch is only collapsed when it shares its upstream *lane* and its
    downstream *road* with a sibling and the two physically overlap. A genuine
    diverge fails all three tests at once: its branches lead to different
    roads and separate immediately.

    Returns one :class:`CollapsedFan` per collapsed group.
    """
    by_key: Dict[Tuple[int, int, int], List[Road]] = defaultdict(list)
    for road in roads:
        key = _fan_key(road)
        if key is not None:
            by_key[key].append(road)
    # A second reading of the same shape, for the branches whose lane links are
    # too sparsely stated for _fan_key to see one upstream lane. Two plain
    # roads bridging the same pair of endpoints cannot both be represented:
    # outside a junction the downstream road names a single predecessor, so one
    # claim is silently lost (Odaiba road 32 named road 37 while road 36 also
    # claimed it). Any overlap at all is then enough — side-by-side lanes of a
    # carriageway are a lane width apart and score zero.
    for road in roads:
        if road.junction not in (-1, None):
            continue
        upstream = _link_target(road, "predecessor")
        downstream = _link_target(road, "successor")
        if upstream is None or downstream is None:
            continue
        if upstream[0] != ElementType.ROAD or downstream[0] != ElementType.ROAD:
            continue
        key = (upstream[1], _ANY_LANE, downstream[1])
        if road not in by_key[key]:
            by_key[key].append(road)

    collapsed: List[CollapsedFan] = []
    removed: Set[int] = set()
    replacement: Dict[int, int] = {}
    # Lane-identified groups first: they know which branch is the straight
    # continuation, and the endpoint-pair reading does not.
    for (upstream_road, upstream_lane, downstream_road), siblings in sorted(
        by_key.items(), key=lambda item: (item[0][1] == _ANY_LANE, item[0])
    ):
        siblings = [road for road in siblings if road.id not in removed]
        if len(siblings) < 2:
            continue
        threshold = _ANY_OVERLAP if upstream_lane == _ANY_LANE else min_overlap_fraction
        overlapping: Set[int] = set()
        for first, second in itertools.combinations(siblings, 2):
            coverage = max(
                _same_stream_coverage(first, second),
                _same_stream_coverage(second, first),
            )
            if coverage >= threshold:
                overlapping.update((first.id, second.id))
        if not overlapping:
            continue
        group = [road for road in siblings if road.id in overlapping]
        keep = _fan_survivor(group, upstream_lane)
        drop = [road for road in group if road.id != keep.id]
        if not drop:
            continue
        targets: List[Tuple[int, int]] = []
        for road in drop:
            for lane in _driving_lanes(road):
                link = getattr(lane, "successor", None)
                if link is not None:
                    targets.append((downstream_road, int(link.id)))
        removed.update(road.id for road in drop)
        replacement.update({road.id: keep.id for road in drop})
        collapsed.append(
            CollapsedFan(
                predecessor_road_id=upstream_road,
                predecessor_lane_id=(
                    0 if upstream_lane == _ANY_LANE else upstream_lane
                ),
                successor_road_id=downstream_road,
                kept_road_id=keep.id,
                dropped_road_ids=sorted(road.id for road in drop),
                lane_change_targets=sorted(set(targets)),
            )
        )

    if removed:
        _detach_dropped_roads(
            roads,
            junctions,
            removed,
            lanelet_to_road_and_lane,
            lanelet_to_emitted_segments,
            replacement,
        )
    return collapsed


def remap_road_of(
    old_road: int, remap: Dict[Tuple[int, int], Tuple[int, int]]
) -> Optional[int]:
    """The road an absorbed road's lanes moved into."""
    for (road_id, _), (new_road, _) in remap.items():
        if road_id == old_road:
            return new_road
    return None


@dataclass
class MergedChain:
    """Consecutive roads joined end to end into a single road."""

    base_road_id: int
    absorbed_road_ids: List[int]
    total_length: float


def _lane_successor_is_identity(road: Road) -> bool:
    """Does every lane continue as the lane with the same id?"""
    lanes = _driving_lanes(road)
    if not lanes:
        return False
    for lane in lanes:
        end = getattr(lane, "successor", None)
        if end is None or int(end.id) != lane.lane_id:
            return False
    return True


def _references_to_endpoint(
    road_id: int,
    contact: ContactPoint,
    roads: Sequence[Road],
    junctions,
) -> int:
    """How many roads attach to one specific end of ``road_id``.

    A joint may only be merged when exactly one road sits on the far side of
    it. Counting every mention of the road would always find at least two —
    the neighbour on each side — so the contact point decides which end a
    mention is about.
    """
    count = 0
    for road in roads:
        if road.id == road_id or road.link is None:
            continue
        for side in ("predecessor", "successor"):
            end = getattr(road.link, side)
            if (
                end is not None
                and end.element_type == ElementType.ROAD
                and int(end.element_id) == road_id
                and end.contact_point == contact
            ):
                count += 1
    # A junction reaches an incoming road through whichever end faces it, so
    # any mention counts against both.
    for junction in junctions:
        for connection in junction.connections:
            if int(connection.incoming_road) == road_id:
                count += 1
    return count


def merge_consecutive_roads(
    roads: List[Road],
    junctions: Sequence[Junction],
    *,
    lanelet_to_road_and_lane: Optional[Dict[int, Tuple[int, int]]] = None,
) -> List[MergedChain]:
    """Join consecutive roads end to end so links stop arriving in pieces.

    A carriageway is emitted as one road per source lanelet group, so a
    straight run of unchanging cross-section can still be several roads. Each
    becomes its own Vissim link joined by a connector, which is what a
    fragmented network looks like on import: on the Odaiba clip roads 8, 9,
    10 and 0 are one continuous single-lane road cut into four (27.7 + 30.0 +
    52.7 + 16.0 m), and roads 42 and 75 are 0.91 m and 1.81 m offcuts.

    Only an unambiguous joint is merged: the two roads must name each other
    at road level, both be outside a junction, carry the same number of
    lanes with a complete lane correspondence, and neither end may be
    referenced by a third road or junction — otherwise the joint is a merge
    or diverge point and the roads have to stay apart.

    The absorbed road must also carry **no source lanelet**. The mapping
    sidecar keys a lanelet by ``(road, lane)`` and its reverse index is 1:1,
    so folding two lanelet-backed roads together would put two lanelets on
    one road lane and the mapping cross-validation rightly rejects that. The
    fragments that matter are synthetic anyway: on the Odaiba clip roads 42
    and 75 are 0.91 m and 1.81 m offcuts with no lanelet behind them, while
    the lanelet-backed roads are 16–52 m, which is an ordinary Vissim link
    length.

    Geometry, elevation and every ``sOffset``-bearing lane record of the
    absorbed road are appended with its station shifted by the base road's
    length, as are its objects and signals, so the merged road is the exact
    concatenation.
    """
    by_id = {road.id: road for road in roads}
    lanelet_backed = {
        road_id for road_id, _ in (lanelet_to_road_and_lane or {}).values()
    }

    def lane_correspondence(first: Road) -> Optional[Dict[int, int]]:
        """``{lane id on first: lane id on its successor}``, or None if partial."""
        mapping: Dict[int, int] = {}
        for lane in _driving_lanes(first):
            end = getattr(lane, "successor", None)
            if end is None:
                return None
            mapping[lane.lane_id] = int(end.id)
        return mapping or None

    def joins(first: Road, second: Road) -> bool:
        if first.junction != -1 or second.junction != -1:
            return False
        forward = _link_target(first, "successor")
        back = _link_target(second, "predecessor")
        if forward != (ElementType.ROAD, second.id):
            return False
        if back != (ElementType.ROAD, first.id):
            return False
        if first.link.successor.contact_point != ContactPoint.START:
            return False
        if second.link.predecessor.contact_point != ContactPoint.END:
            return False
        if second.id in lanelet_backed:
            return False
        mapping = lane_correspondence(first)
        if mapping is None:
            return False
        second_ids = {lane.lane_id for lane in _driving_lanes(second)}
        if set(mapping.values()) != second_ids:
            return False
        # A third party at the joint means it is a merge or a diverge.
        if _references_to_endpoint(second.id, ContactPoint.START, roads, junctions) > 1:
            return False
        if _references_to_endpoint(first.id, ContactPoint.END, roads, junctions) > 1:
            return False
        return True

    successor_of: Dict[int, int] = {}
    for first in roads:
        target = _link_target(first, "successor")
        if target is None or target[0] != ElementType.ROAD:
            continue
        second = by_id.get(target[1])
        if second is not None and joins(first, second):
            successor_of[first.id] = second.id

    heads = [rid for rid in successor_of if rid not in set(successor_of.values())]
    merged: List[MergedChain] = []
    removed: Set[int] = set()

    for head in sorted(heads):
        chain = [head]
        while chain[-1] in successor_of:
            chain.append(successor_of[chain[-1]])
        if len(chain) < 2:
            continue
        base = by_id[chain[0]]
        for road_id in chain[1:]:
            member = by_id[road_id]
            shift = base.length

            for geometry in member.plan_view.geometries:
                geometry.s += shift
            base.plan_view.geometries.extend(member.plan_view.geometries)

            if (
                base.elevation_profile is not None
                and member.elevation_profile is not None
            ):
                for record in member.elevation_profile.elevations:
                    record.s += shift
                base.elevation_profile.elevations.extend(
                    member.elevation_profile.elevations
                )

            mapping = lane_correspondence(base) or {}
            member_lanes = {lane.lane_id: lane for lane in _driving_lanes(member)}
            for target_lane in _driving_lanes(base):
                lane = member_lanes.get(mapping.get(target_lane.lane_id))
                if lane is None:
                    continue
                for attribute in (
                    "widths",
                    "road_marks",
                    "borders",
                    "heights",
                    "speeds",
                    "accesses",
                ):
                    records = getattr(lane, attribute, None) or []
                    for record in records:
                        record.s_offset += shift
                    existing = getattr(target_lane, attribute, None)
                    if existing is not None:
                        existing.extend(records)
                target_lane.successor = getattr(lane, "successor", None)

            for collection in ("objects", "signals"):
                items = getattr(member, collection, None) or []
                for item in items:
                    if hasattr(item, "s"):
                        item.s += shift
                if items:
                    if getattr(base, collection, None) is None:
                        setattr(base, collection, [])
                    getattr(base, collection).extend(items)

            base.length += member.length
            base.link.successor = member.link.successor
            base.reference_end_xyz = member.reference_end_xyz
            removed.add(road_id)

        # Whatever the chain's last road pointed at must now name the base.
        tail = chain[-1]
        for road in roads:
            if road.id in removed or road.link is None:
                continue
            for side in ("predecessor", "successor"):
                end = getattr(road.link, side)
                if (
                    end is not None
                    and end.element_type == ElementType.ROAD
                    and int(end.element_id) == tail
                ):
                    end.element_id = base.id
        for junction in junctions:
            for connection in junction.connections:
                if int(connection.incoming_road) == tail:
                    connection.incoming_road = base.id

        merged.append(
            MergedChain(
                base_road_id=base.id,
                absorbed_road_ids=chain[1:],
                total_length=base.length,
            )
        )

    if removed:
        roads[:] = [road for road in roads if road.id not in removed]

    return merged


#: Lane types Vissim turns into links. A road carrying nothing else is
#: invisible to it, so keeping it only clutters the file and other viewers.
VISSIM_IMPORTED_LANE_TYPES = frozenset(
    {
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
)


def omit_unimported_roads(
    roads: List[Road],
    *,
    lanelet_to_road_and_lane: Optional[Dict[int, Tuple[int, int]]] = None,
    lanelet_to_emitted_segments: Optional[Dict[int, List[dict]]] = None,
) -> List[Tuple[int, str, bool]]:
    """Drop roads Vissim cannot use, and isolated ones it would import empty.

    Vissim builds a link only from the lane types it imports; ``shoulder`` and
    ``sidewalk`` are not among them, so such a road is invisible there while
    still cluttering the file and any other viewer. On the Odaiba clip eight
    roads carry nothing but a shoulder or sidewalk lane and none of them has a
    ``<link>`` at all, which is why they render as disconnected patches with
    gaps between them.

    Two more roads carry only a ``biking`` lane. Vissim *does* import that
    type, but with no link on either end they would arrive as links floating
    unreachable in the network, so they go too — an importable road is only
    kept when something connects to it.

    Returns ``(road_id, lane_types, was_isolated)`` per dropped road.
    """
    dropped: List[Tuple[int, str, bool]] = []
    removed: Set[int] = set()

    for road in roads:
        types = {
            lane.lane_type.value
            if hasattr(lane.lane_type, "value")
            else str(lane.lane_type)
            for lane in _driving_lanes(road)
        }
        if not types:
            continue
        importable = types & VISSIM_IMPORTED_LANE_TYPES
        isolated = not _linked_road_ids(road) and (
            road.link is None
            or (road.link.predecessor is None and road.link.successor is None)
        )
        if importable and not isolated:
            continue
        if not importable or isolated:
            removed.add(road.id)
            dropped.append((road.id, ",".join(sorted(types)), isolated))

    if not removed:
        return dropped

    roads[:] = [road for road in roads if road.id not in removed]
    if lanelet_to_road_and_lane is not None:
        for lanelet_id in [
            lanelet_id
            for lanelet_id, (road_id, _) in lanelet_to_road_and_lane.items()
            if road_id in removed
        ]:
            del lanelet_to_road_and_lane[lanelet_id]
    if lanelet_to_emitted_segments is not None:
        for lanelet_id, segments in list(lanelet_to_emitted_segments.items()):
            kept = [s for s in segments if s.get("road_id") not in removed]
            if kept:
                lanelet_to_emitted_segments[lanelet_id] = kept
            else:
                del lanelet_to_emitted_segments[lanelet_id]
    return dropped


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
    min_length: float = DEFAULT_ABSORB_MAX_LENGTH,
) -> List[Tuple[int, int, int, bool]]:
    """Replace stub roads with a direct link between their neighbours.

    Two things leave stubs behind. The divergence synthesis emits a 0.01 m
    connecting road per lane-level movement so CARLA's loader has something to
    follow, and dissolving a non-intersection junction turns its connecting
    roads into ordinary roads — so a connector that was naturally a metre or
    two long arrives as a link that short. Either way Vissim gets a fragment
    where it should have a connector: 1 cm is below its 0.5 m minimum spline
    spacing, and a 2 m link between two connectors is what a fragmented
    network looks like.

    Joining the neighbours directly leaves them ``min_length`` apart at most,
    and Vissim spans that with a connector — which is the right object for a
    couple of metres.

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

    report.overlapping_roads = _find_overlapping_roads(roads)
    return report


def conflict_area_exempted_by_road_end(
    covered: Road,
    overlay: Road,
    roads: Sequence[Road],
    junctions: Sequence[Junction],
    *,
    within: Optional[float] = None,
    max_heading_diff_deg: float = DEFAULT_MAX_HEADING_DIFF_DEG,
) -> bool:
    """Whether PTV's third exemption removes this pair's conflict area.

    No conflict area arises when, within ``within`` metres of where the overlap
    begins, one of the two links ends and no connector starts there. Recorded
    only: nothing in the merge or collapse decision consults it.

    Both roads are tested, because the exemption asks about *one* of them. For
    each, the station where its surface first meets the other's gives the start
    of the overlap; the link ends within reach when the remaining length from
    there is no more than ``within``.
    """
    if within is None:
        within = DEFAULT_CONFIG.vissim_topology.conflict_exemption_distance
    connector_ids = {road.id for road in roads if road.junction not in (-1, None)}
    incoming_to_junction = {
        int(connection.incoming_road)
        for junction in junctions
        for connection in junction.connections
    }

    def connector_starts_at_end(road: Road) -> bool:
        """Whether a connecting road picks the stretch up where ``road`` ends."""
        if road.id in incoming_to_junction:
            return True
        end = road.link.successor if road.link is not None else None
        if end is None:
            return False
        if end.element_type == ElementType.JUNCTION:
            return True
        return (
            end.element_type == ElementType.ROAD
            and int(end.element_id) in connector_ids
        )

    limit = math.radians(max_heading_diff_deg)
    for road, other in ((covered, overlay), (overlay, covered)):
        ours = _band_stations(road)
        theirs = _band_stations(other)
        if not ours or not theirs:
            continue
        our_widths = _lane_centre_widths(road)
        their_widths = _lane_centre_widths(other)
        length = _road_length(road)
        spacing = (length / (len(ours) - 1)) if len(ours) > 1 else 0.0
        start_station: Optional[int] = None
        for station, (points, heading) in enumerate(ours):
            for index, point in enumerate(points):
                our_width = our_widths[index] if index < len(our_widths) else 0.0
                for other_points, other_heading in theirs:
                    delta = abs(
                        (other_heading - heading + math.pi) % (2 * math.pi) - math.pi
                    )
                    if delta > limit:
                        continue
                    for other_index, q in enumerate(other_points):
                        their_width = (
                            their_widths[other_index]
                            if other_index < len(their_widths)
                            else 0.0
                        )
                        distance = math.hypot(point[0] - q[0], point[1] - q[1])
                        if (our_width + their_width) / 2.0 - distance > 0.0:
                            start_station = station
                            break
                    if start_station is not None:
                        break
                if start_station is not None:
                    break
            if start_station is not None:
                break
        if start_station is None:
            continue
        remaining = length - start_station * spacing
        if remaining <= within and not connector_starts_at_end(road):
            return True
    return False


#: Columns of the overlap-measurement CSV, in order.
OVERLAP_CSV_COLUMNS = (
    "road_a",
    "road_b",
    "min_centre_distance_m",
    "max_overlap_width_m",
    "overlap_length_m",
    "linked",
    "exempt_by_road_end",
    "interpretation_x",
    "interpretation_y",
    "current_criterion",
    "coverage",
    "kind",
)


def _pair_is_linked(first: Road, second: Road, junctions: Sequence[Junction]) -> bool:
    """Whether the two roads name each other, directly or through a junction."""
    for road, other in ((first, second), (second, first)):
        if road.link is None:
            continue
        for side in ("predecessor", "successor"):
            end = getattr(road.link, side, None)
            if (
                end is not None
                and end.element_type == ElementType.ROAD
                and int(end.element_id) == other.id
            ):
                return True
    for junction in junctions:
        for connection in junction.connections:
            pair = {int(connection.incoming_road), int(connection.connecting_road)}
            if pair == {first.id, second.id}:
                return True
    return False


def measure_overlap_table(
    roads: Sequence[Road],
    junctions: Sequence[Junction],
) -> List[Dict[str, object]]:
    """One row per road pair that overlaps under *any* reading.

    A pair is listed when the current criterion matches it or when the surfaces
    meet at all, so a row exists wherever the readings could disagree — which
    is the point of the table.
    """
    rows: List[Dict[str, object]] = []
    connector_ids = {road.id for road in roads if road.junction not in (-1, None)}
    for first, second in itertools.combinations(
        sorted(roads, key=lambda road: road.id), 2
    ):
        forward = measure_stream_overlap(first, second)
        backward = measure_stream_overlap(second, first)
        distances = [
            value
            for value in (forward.min_centre_distance, backward.min_centre_distance)
            if value is not None
        ]
        coverage = max(forward.coverage, backward.coverage)
        width = max(forward.max_overlap_width, backward.max_overlap_width)
        # Reading Y asks how far the overlap runs; take the longer of the two
        # views, since each is measured along its own road.
        length = max(forward.overlap_length, backward.overlap_length)
        if coverage <= 0.0 and width <= 0.0:
            continue
        kinds = tuple(
            "connector" if r.id in connector_ids else "link" for r in (first, second)
        )
        rows.append(
            {
                "road_a": first.id,
                "road_b": second.id,
                "min_centre_distance_m": (
                    round(min(distances), 3) if distances else ""
                ),
                "max_overlap_width_m": round(width, 3),
                "overlap_length_m": round(length, 2),
                "linked": _pair_is_linked(first, second, junctions),
                "exempt_by_road_end": conflict_area_exempted_by_road_end(
                    first, second, roads, junctions
                ),
                "interpretation_x": (
                    width > DEFAULT_CONFIG.vissim_topology.conflict_overlap_width
                ),
                "interpretation_y": (
                    length > DEFAULT_CONFIG.vissim_topology.conflict_overlap_length
                ),
                "current_criterion": coverage > 0.0,
                "coverage": round(coverage, 3),
                "kind": "x".join(kinds),
            }
        )
    rows.sort(key=lambda row: (-float(row["overlap_length_m"]), row["road_a"]))
    return rows


def write_overlap_measurements(
    path,
    roads: Sequence[Road],
    junctions: Sequence[Junction],
) -> int:
    """Write :func:`measure_overlap_table` to ``path`` as CSV. Returns the rows."""
    import csv
    from pathlib import Path

    rows = measure_overlap_table(roads, junctions)
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(OVERLAP_CSV_COLUMNS))
        writer.writeheader()
        writer.writerows(rows)
    return len(rows)


#: Columns of the lanelet ↔ OpenDRIVE ↔ Vissim correspondence CSV, in order.
VISSIM_MAPPING_CSV_COLUMNS = (
    "lanelet_id",
    "road_id",
    "lane_id",
    "lane_type",
    "is_connecting_road",
    "junction_id",
    "vissim_link_name",
    "vissim_lane_index",
    "imported_by_vissim",
)


def vissim_link_name(road: Road) -> str:
    """The name Vissim gives an imported road.

    Taken from what Vissim reports back, not from its documentation: an import
    error names a link ``"5: 9: Road_9-0-Left"``, which is ``<Vissim's own
    number>: <our road id>: <name>``, and a connecting road appears as
    ``"10015: 44: Road_44-0-Left Connector - 1"``. So the name is
    ``Road_<road id>-<lane section index>-<side>`` and the *second* field is our
    road id, which is what makes a correspondence table possible at all —
    Vissim's own numbering is assigned at import and is not ours to predict.

    Side follows where the driving lanes sit: positive lane ids are left of the
    reference line, which is the LHT convention this profile emits.
    """
    lanes = _driving_lanes(road)
    side = "Left" if any(lane.lane_id > 0 for lane in lanes) else "Right"
    suffix = " Connector" if road.junction not in (-1, None) else ""
    return f"Road_{road.id}-0-{side}{suffix}"


def vissim_mapping_rows(
    roads: Sequence[Road],
    lanelet_to_road_and_lane: Dict[int, Tuple[int, int]],
) -> List[Dict[str, object]]:
    """One row per lanelet: where it went, and what Vissim will call it.

    Rows are emitted for every mapped lanelet, including those on carriageways
    Vissim does not import — ``imported_by_vissim`` says which is which, so a
    shoulder lanelet is traceable even though no Vissim link corresponds to it.
    """
    by_id = {road.id: road for road in roads}
    rows: List[Dict[str, object]] = []
    for lanelet_id, target in sorted(lanelet_to_road_and_lane.items()):
        road_id, lane_id = int(target[0]), int(target[1])
        road = by_id.get(road_id)
        if road is None:
            continue
        lane = _lane_by_id(road, lane_id)
        lane_type = (
            lane.lane_type.value
            if lane is not None and hasattr(lane.lane_type, "value")
            else str(getattr(lane, "lane_type", ""))
        )
        # Vissim numbers the lanes of a link from the right; our ids count
        # outwards from the reference line, so on the left side the order is
        # the same and the index is the id.
        ordered = sorted(
            (abs(other.lane_id) for other in _driving_lanes(road)),
        )
        try:
            lane_index = ordered.index(abs(lane_id)) + 1
        except ValueError:
            lane_index = 0
        rows.append(
            {
                "lanelet_id": lanelet_id,
                "road_id": road_id,
                "lane_id": lane_id,
                "lane_type": lane_type,
                "is_connecting_road": road.junction not in (-1, None),
                "junction_id": (
                    road.junction if road.junction not in (-1, None) else ""
                ),
                "vissim_link_name": vissim_link_name(road),
                "vissim_lane_index": lane_index,
                "imported_by_vissim": lane_type in VISSIM_IMPORTED_LANE_TYPES,
            }
        )
    return rows


def write_vissim_mapping(
    path,
    roads: Sequence[Road],
    lanelet_to_road_and_lane: Dict[int, Tuple[int, int]],
    projection_metadata: Optional[dict] = None,
) -> int:
    """Write the correspondence table to ``path`` as CSV.

    The projection offsets go in a comment header rather than a column: they are
    one fact about the whole file, and without them the coordinates cannot be
    put back on the source map's frame. ``x_absolute = x + offset_x``.
    """
    import csv
    from pathlib import Path

    rows = vissim_mapping_rows(roads, lanelet_to_road_and_lane)
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open("w", newline="", encoding="utf-8") as handle:
        if projection_metadata:
            handle.write(
                "# projection: "
                + ", ".join(
                    f"{key}={projection_metadata[key]}"
                    for key in sorted(projection_metadata)
                )
                + "\n"
            )
            handle.write("# absolute = emitted + offset (x_absolute = x + offset_x)\n")
        writer = csv.DictWriter(handle, fieldnames=list(VISSIM_MAPPING_CSV_COLUMNS))
        writer.writeheader()
        writer.writerows(rows)
    return len(rows)
