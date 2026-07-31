"""PTV Vissim export profile: write-time post-processing of the OpenDRIVE tree.

Vissim's openDRIVE importer validates the file against the declared schema
version (OpenDRIVE 1.4 here) and only interprets a subset of the format
(see ``docs/vissim-compatibility.md``). This module adapts the final XML
tree — without touching the conversion pipeline — so that other targets
(CARLA, Foretify preflight) keep their existing output:

* strips attributes that are not part of OpenDRIVE 1.4 and that Vissim's
  schema validation would flag (``road@rule``, ``lane@rule``,
  ``access@rule``),
* optionally re-parameterizes ``paramPoly3`` coefficients from the
  arc-length convention (``pRange="arcLength"``, a 1.5+ attribute) to the
  normalized ``p ∈ [0, 1]`` convention, removing the non-1.4 attribute,
* replaces the header ``geoReference`` with a PROJ string that describes
  the *exported local frame* (issue #550: the default string describes
  absolute UTM while the coordinates are local offsets, which misplaces
  the network on Vissim's background map),
* collapses each lane's polynomial ``<width>`` chain to a single constant
  record (arc-length-weighted mean). Vissim only supports constant lane
  widths: every width variation ≥ 0.25 m makes its importer insert a
  connector plus two 1.1 m links, so dense polynomial width records
  shatter each road into dozens of fragments — observed as tangled
  connector webs and "node overlap on link …-Left Start" errors on
  import,
* reports constructs Vissim is known to degrade on (roads shorter than
  its 0.5 m spline spacing / 1.1 m inserted-link length, lane widths
  below the 1 m clamp, width swings beyond the 0.25 m connector
  threshold).
"""

import itertools
import logging
import math
import re
from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Union

import lxml.etree as ET

logger = logging.getLogger(__name__)

# Vissim importer behavioral constants (PTV Vissim manual, openDRIVE import).
VISSIM_MIN_LANE_WIDTH = 1.0  # widths below this are clamped to 1 m
VISSIM_WIDTH_SWING_THRESHOLD = 0.25  # larger swings insert connector + links
VISSIM_INSERTED_LINK_LENGTH = 1.1  # length of links inserted at width changes
VISSIM_MIN_SPLINE_SPACING = 0.5  # minimum spline point spacing

_P_RANGE_MODES = ("arcLength", "normalized")

# Named references for the elevation baseline shift.
_ELEVATION_BASELINES = ("none", "min", "mean")

# Lane types the Vissim importer actually turns into links; only these are
# checked in the width report (sidewalk/shoulder etc. are ignored by Vissim).
_VISSIM_IMPORTED_LANE_TYPES = frozenset(
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


@dataclass
class VissimConfig:
    """Configuration for the Vissim export profile.

    Attributes:
        enabled: Master switch. Disabled by default so that existing targets
            (default, carla) are byte-identical.
        strip_nonstandard_attributes: Remove ``road@rule`` / ``lane@rule``
            (CARLA LHT extensions) and ``access@rule`` (OpenDRIVE 1.6
            syntax) so the file passes strict 1.4 schema validation.
        param_poly3_p_range: ``"normalized"`` re-parameterizes every
            arc-length ``paramPoly3`` to ``p ∈ [0, 1]`` (exact coefficient
            scaling) and drops the non-1.4 ``pRange`` attribute.
            ``"arcLength"`` keeps coefficients and attribute unchanged.
            Which convention Vissim assumes must be verified with a one-off
            import test; see docs/vissim-compatibility.md.
        local_geo_reference: Replace the header geoReference with the
            local-frame PROJ string (``local_geo_reference_proj``) so Vissim
            places the network correctly on its background map.
        local_geo_reference_proj: The precomputed local-frame PROJ string.
            Computed by the caller (it needs the resolved projection origin
            and offsets) via :func:`local_frame_proj_string`.
        constant_lane_widths: Collapse each lane's ``<width>`` records to a
            single constant (arc-length-weighted mean over the lane
            section). Vissim treats widths as constants anyway; leaving the
            polynomial records makes its importer insert a connector and
            two 1.1 m links at every ≥ 0.25 m variation, fragmenting the
            network and causing node-overlap errors.
        dissolve_non_intersection_junctions: Turn junctions that carry no
            crossing movement (pure merges and diverges) into ordinary road
            links. Vissim places a node — with the full intersection
            machinery — at every junction, so a plain widening or off-ramp
            otherwise arrives as an intersection. Applied in the conversion
            pipeline by ``vissim_topology``, before the mapping is written.
        elevation_baseline: Shift every elevation so the network sits near
            ``z = 0``. Lanelet2 maps store **absolute** elevation (the Odaiba
            clip runs 4.4–12.7 m above sea level) and the converter subtracts
            nothing unless ``map.offset.z`` is set, so the whole network
            floats above Vissim's background plane by that amount — and by a
            varying amount where the terrain rises. ``"min"`` puts the lowest
            road surface at zero, ``"mean"`` centres the network, ``"none"``
            keeps absolute elevation, and a float shifts by that many metres.
            Only a constant offset is applied, so every gradient is preserved.
        merge_consecutive_roads: Join roads that follow one another end to end
            into a single road. A carriageway is emitted per source lanelet
            group, so a straight run can be several roads, and each becomes
            its own Vissim link joined by a connector — a fragmented network.
        merge_parallel_lane_roads: Merge roads that are per-lane halves of one
            carriageway back into a single road. Only roads agreeing on both
            link ends are merged, so the merged road inherits them unchanged.
        untag_straight_turn_lanelets: Drop ``turn_direction`` from lanelets
            that carry ``left``/``right`` while running straight. Autoware
            tags a turn lane from where its pocket opens, but the converter
            reads the attribute as "inside an intersection", so the pocket
            becomes a connecting road that starts far upstream and runs
            alongside the through carriageway. ``straight`` is never removed.
            Applied to the loaded map before conversion.
        absorb_degenerate_stubs: Replace stub roads shorter than
            ``absorb_stub_max_length`` with a direct link between their
            neighbours. Both the 0.01 m divergence stubs and the one- to
            two-metre connecting roads left behind by dissolving a junction
            arrive in Vissim as fragments where a connector belongs; joining
            the neighbours hands the stretch back to one. Applied in the
            pipeline by ``vissim_topology``.
        overlap_measurement_csv: Write an ``<output>.overlap.csv`` sidecar
            recording every reading of "these two roads overlap" — the
            lane-centre criterion the passes act on, the lateral overlap width,
            the longitudinal overlap length, and PTV's ending-road exemption.
            Purely diagnostic: no pass consults it, and which reading of the
            Vissim rule applies is still open.
        collapse_lane_choice_fans: Disabled by default — see the note below.
            Reduce branches that share an upstream lane
            and a downstream road, differing only in the lane they land in, to
            the single path Vissim can represent. Lanelet2 draws "this lane may
            end up in any of those lanes" as one lanelet per destination, all
            leaving the same cross-section, so the transcription lays several
            roads on top of one another and Vissim raises a conflict area for
            every overlapping pair. The abandoned downstream lanes keep their
            traffic: an OpenDRIVE lane may start with no predecessor and be
            entered by changing lanes, which is what a turn pocket is. That is
            also why it is off: the movement into an abandoned lane becomes a
            lane change, and in Vissim a right turn then crosses three lanes at
            once. Overlapping geometry a modeller can resolve; a turn that has
            to cross three lanes cannot.
        signal_table_csv: Write an ``<output>.vissim_signals.csv`` sidecar
            listing every signal with the Vissim link and lane a Signal Head
            belongs on, the distance from the link start, and the controllers
            that group it. Vissim imports no signalization, so the heads are
            placed by hand; this replaces the coordinate lookup per head.
        vissim_mapping_csv: Write an ``<output>.vissim_mapping.csv`` sidecar
            giving, per lanelet, the road and lane it became and the name Vissim
            will show for that road, with the projection offsets in the header.
            Vissim's own link numbering is assigned at import, but it reports our
            road id as the middle field of every link label, which is what makes
            the correspondence resolvable.
        link_isolated_roads: Assert the succession of roads the vehicle routing
            graph never saw. Links come from a routing graph built for a vehicle
            participant, so a shoulder or bicycle carriageway arrives with no
            ``<link>`` at all even where the source lanelets share a boundary
            and continue one another — four such successions on the Odaiba clip,
            each rendering as a floating patch. Only roads with no link on either
            end are touched.
        clip_false_lane_overlaps: Narrow constant lane widths that make the
            lanes of two different roads overlap on paper. The constant is the
            lane's arc-length mean width, so wherever the lane is genuinely
            narrower the emitted surface is too wide and neighbouring lanes
            cross for no physical reason — Vissim then raises a conflict area
            between lanes no vehicle can occupy at once. Pairs whose centres
            are closer than 1.5 m are left alone: those are roads genuinely
            laid on top of one another, a topology defect that must stay
            visible.
        align_connector_lanes: Slide each connecting road sideways so its lane
            meets the lane it links to. Constant-izing widths moves a lane
            centre — the further from the reference line the more — so the
            connectors, which follow the true lane centre, no longer meet the
            links; Vissim snaps them and the connector visibly kinks.
        omit_unimported_roads: Drop roads Vissim cannot use — those carrying
            only lane types it does not import (``shoulder``, ``sidewalk``) and
            importable ones with no link on either end, which would arrive as
            links floating unreachable in the network.
        absorb_stub_max_length: Length below which a road counts as a stub
            (metres). Default 3.0 — long enough to catch a dissolved
            junction's connectors, short enough that the gap the neighbours
            are left with is a plausible connector.
        merge_overlapping_junctions: Merge junctions whose connecting roads
            attach to the same road endpoint. The divergence synthesis can
            emit chained junctions that meet at one physical point (two
            connecting roads from different junctions ending at the same
            road start); Vissim builds one node per junction and reports
            "Nodes … overlap on link …/link segments … are invalid" for
            such pairs, leaving conflict areas undetermined.
    """

    enabled: bool = False
    strip_nonstandard_attributes: bool = True
    param_poly3_p_range: str = "normalized"
    local_geo_reference: bool = True
    local_geo_reference_proj: Optional[str] = None
    constant_lane_widths: bool = True
    merge_overlapping_junctions: bool = True
    dissolve_non_intersection_junctions: bool = True
    elevation_baseline: Union[str, float] = "min"
    absorb_degenerate_stubs: bool = True
    absorb_stub_max_length: float = 3.0
    omit_unimported_roads: bool = False
    align_connector_lanes: bool = True
    link_isolated_roads: bool = True
    vissim_mapping_csv: bool = True
    signal_table_csv: bool = True
    clip_false_lane_overlaps: bool = False
    untag_straight_turn_lanelets: bool = True
    merge_parallel_lane_roads: bool = True
    merge_consecutive_roads: bool = True
    collapse_lane_choice_fans: bool = False
    overlap_measurement_csv: bool = True

    def __post_init__(self) -> None:
        if self.param_poly3_p_range not in _P_RANGE_MODES:
            raise ValueError(
                f"param_poly3_p_range must be one of {_P_RANGE_MODES}, "
                f"got '{self.param_poly3_p_range}'"
            )
        if isinstance(self.elevation_baseline, str):
            if self.elevation_baseline not in _ELEVATION_BASELINES:
                raise ValueError(
                    "elevation_baseline must be a number or one of "
                    f"{_ELEVATION_BASELINES}, got '{self.elevation_baseline}'"
                )


@dataclass
class VissimProfileReport:
    """Summary of profile edits and Vissim import risk indicators."""

    rule_attributes_stripped: int = 0
    param_poly3_reparameterized: int = 0
    geo_reference_replaced: bool = False
    lanes_width_constantized: int = 0
    junctions_merged: int = 0
    elevation_shift: Optional[float] = None
    connectors_realigned: List[Tuple[str, float, float]] = field(default_factory=list)
    false_overlaps_clipped: List[Tuple[str, str, float, float]] = field(
        default_factory=list
    )
    roads_below_spline_spacing: List[str] = field(default_factory=list)
    roads_below_inserted_link_length: List[str] = field(default_factory=list)
    lanes_below_min_width: int = 0
    lanes_above_width_swing_threshold: int = 0
    checked_lanes: int = 0

    def log(self, log: logging.Logger = logger) -> None:
        """Emit the report at INFO level (WARNING for degenerate roads)."""
        log.info(
            "Vissim profile: stripped %d rule attributes, "
            "re-parameterized %d paramPoly3 segments, geoReference %s, "
            "constantized widths on %d lanes, merged %d co-located junctions, "
            "elevation shifted by %s",
            self.rule_attributes_stripped,
            self.param_poly3_reparameterized,
            "replaced" if self.geo_reference_replaced else "kept",
            self.lanes_width_constantized,
            self.junctions_merged,
            (
                f"{-self.elevation_shift:+.2f} m"
                if self.elevation_shift is not None
                else "nothing"
            ),
        )
        if self.false_overlaps_clipped:
            logger.info(
                "Vissim profile: narrowed constant widths on %d road pair(s) "
                "whose lanes only overlapped because the constant is the lane's "
                "mean width — worst %.3f m of surface overlap removed; Vissim "
                "raises a conflict area wherever two surfaces meet",
                len(self.false_overlaps_clipped),
                max(row[2] for row in self.false_overlaps_clipped),
            )
        if self.connectors_realigned:
            worst = max(
                self.connectors_realigned,
                key=lambda item: max(abs(item[1]), abs(item[2])),
            )
            log.info(
                "Vissim profile: slid %d connector(s) sideways onto the lane "
                "they link (constant widths move a lane centre); largest "
                "correction %+.3f m on connector %s",
                len(self.connectors_realigned),
                max(worst[1], worst[2], key=abs),
                worst[0],
            )
        if self.roads_below_spline_spacing:
            log.warning(
                "Vissim profile: %d roads shorter than the %.1f m Vissim "
                "spline spacing (may degenerate on import): %s",
                len(self.roads_below_spline_spacing),
                VISSIM_MIN_SPLINE_SPACING,
                ", ".join(self.roads_below_spline_spacing[:20]),
            )
        log.info(
            "Vissim import indicators: %d/%d lanes dip below the %.1f m "
            "width clamp; %d/%d lanes swing beyond the %.2f m threshold "
            "(Vissim inserts a connector and two %.1f m links there); "
            "%d roads shorter than %.1f m",
            self.lanes_below_min_width,
            self.checked_lanes,
            VISSIM_MIN_LANE_WIDTH,
            self.lanes_above_width_swing_threshold,
            self.checked_lanes,
            VISSIM_WIDTH_SWING_THRESHOLD,
            VISSIM_INSERTED_LINK_LENGTH,
            len(self.roads_below_inserted_link_length),
            VISSIM_INSERTED_LINK_LENGTH,
        )


def local_frame_proj_string(
    mgrs_code: str, offset_x: float = 0.0, offset_y: float = 0.0
) -> str:
    """Build the PROJ string that describes the exported local frame.

    The converter's local frame is the MGRS in-square coordinate system
    (identical to absolute UTM shifted by the 100 km square's south-west
    corner), further shifted by the configured map offset. A consumer that
    georeferences the network (Vissim background maps) therefore needs a
    transverse-Mercator definition whose false origin absorbs both shifts:

    ``x_0 = 500000 − E₀`` and ``y_0 = f_n − N₀`` where ``(E₀, N₀)`` is the
    absolute UTM coordinate of the local origin (square corner + offset)
    and ``f_n`` is the UTM false northing of the hemisphere.

    Args:
        mgrs_code: MGRS grid reference (e.g. ``"54SUE"``); digits beyond the
            grid square are ignored.
        offset_x: Easting offset of the local origin within the square (m).
        offset_y: Northing offset of the local origin within the square (m).

    Returns:
        PROJ string mapping WGS84 lat/lon to the exported local frame.

    Raises:
        ValueError: If the MGRS code is invalid.
    """
    # Imported lazily: lanelet2 / mgrs carry native extensions and this
    # keeps the pure-XML part of the module importable in isolation.
    import lanelet2
    import mgrs as mgrs_lib

    from .projection import _normalize_mgrs_grid

    match = re.match(r"^(\d+)([A-Z])", mgrs_code.strip())
    if not match:
        raise ValueError(f"Invalid MGRS grid reference: '{mgrs_code}'")
    zone = int(match.group(1))
    band = match.group(2)
    is_south = band < "N"

    # South-west corner of the 100 km grid square in absolute UTM. The
    # corner lies on an exact 100 km multiple; round() removes the
    # femto-scale round-trip noise of toLatLon + Forward.
    lat_sw, lon_sw = mgrs_lib.MGRS().toLatLon(_normalize_mgrs_grid(mgrs_code))
    utm = lanelet2.projection.UtmProjector(
        lanelet2.io.Origin(lat_sw, lon_sw), False, False
    )
    sw = utm.forward(lanelet2.core.GPSPoint(lat_sw, lon_sw, 0.0))
    easting_sw = round(sw.x)
    northing_sw = round(sw.y)

    e0 = easting_sw + offset_x
    n0 = northing_sw + offset_y
    false_northing = 10000000.0 if is_south else 0.0
    central_meridian = zone * 6 - 183

    x_0 = 500000.0 - e0
    y_0 = false_northing - n0
    return (
        f"+proj=tmerc +lat_0=0 +lon_0={central_meridian} +k=0.9996 "
        f"+x_0={x_0} +y_0={y_0} +datum=WGS84 +units=m +no_defs"
    )


def _strip_rule_attributes(root: ET._Element) -> int:
    """Remove non-1.4 ``rule`` attributes; returns the number removed."""
    removed = 0
    for tag in ("road", "lane", "access"):
        for elem in root.iter(tag):
            if "rule" in elem.attrib:
                del elem.attrib["rule"]
                removed += 1
    return removed


def _reparameterize_param_poly3(root: ET._Element) -> int:
    """Convert arc-length paramPoly3 records to the normalized convention.

    ``u(p) = a + b·p + c·p² + d·p³`` with ``p ∈ [0, L]`` is identical to
    the same polynomial in ``p̂ = p / L ∈ [0, 1]`` with coefficients
    ``(a, b·L, c·L², d·L³)`` — an exact transformation. The non-1.4
    ``pRange`` attribute is removed afterwards.

    Returns the number of converted segments.
    """
    # Imported here: conversion_config imports this module, and going
    # through the opendrive subpackage at module level would close an
    # import cycle (opendrive/__init__ pulls in conversion_config users).
    from .opendrive.xml_utils import replace_subnormal

    converted = 0
    for geometry in root.iter("geometry"):
        poly = geometry.find("paramPoly3")
        if poly is None or poly.get("pRange") != "arcLength":
            continue
        length = float(geometry.get("length", "0"))
        if length <= 0.0:
            # Degenerate segment: dropping the attribute alone would change
            # nothing (all powers of 0 agree); still remove it for schema
            # cleanliness.
            del poly.attrib["pRange"]
            continue
        for prefix in ("U", "V"):
            for coeff, power in (("b", 1), ("c", 2), ("d", 3)):
                key = f"{coeff}{prefix}"
                value = float(poly.get(key, "0")) * length**power
                poly.set(key, str(replace_subnormal(value)))
        del poly.attrib["pRange"]
        converted += 1
    return converted


def _shift_elevation(root: ET._Element, baseline: Union[str, float]) -> Optional[float]:
    """Translate every elevation so the network sits near ``z = 0``.

    Each ``<elevation>`` is ``z(ds) = a + b·ds + c·ds² + d·ds³``, so
    subtracting a constant from ``a`` shifts the surface without touching a
    single gradient. ``<positionInertial>`` carries absolute coordinates and
    is shifted with it; ``zOffset`` and ``<cornerLocal>`` are relative to the
    road surface and are left alone.

    Returns the applied shift, or ``None`` when nothing was changed.
    """
    from .opendrive.xml_utils import replace_subnormal

    elevations = list(root.iter("elevation"))
    if not elevations:
        return None

    if isinstance(baseline, str):
        if baseline == "none":
            return None
        surface = [float(e.get("a", "0")) for e in elevations]
        shift = min(surface) if baseline == "min" else sum(surface) / len(surface)
    else:
        shift = float(baseline)
    if shift == 0.0:
        return None

    for elevation in elevations:
        elevation.set(
            "a", str(replace_subnormal(float(elevation.get("a", "0")) - shift))
        )
    for position in root.iter("positionInertial"):
        if "z" in position.attrib:
            position.set("z", str(replace_subnormal(float(position.get("z")) - shift)))
    return shift


def _road_frame(geometry: ET._Element, p: float):
    """``(x, y, heading)`` on a geometry's reference line at station ``p``."""

    x = float(geometry.get("x"))
    y = float(geometry.get("y"))
    heading = float(geometry.get("hdg"))
    length = float(geometry.get("length"))
    cos_h, sin_h = math.cos(heading), math.sin(heading)
    poly = geometry.find("paramPoly3")
    if poly is None:
        return (x + cos_h * p, y + sin_h * p, heading)
    a = [float(poly.get(k, "0")) for k in ("aU", "bU", "cU", "dU")]
    b = [float(poly.get(k, "0")) for k in ("aV", "bV", "cV", "dV")]
    t = p / length if poly.get("pRange", "normalized") == "normalized" else p
    u = a[0] + a[1] * t + a[2] * t * t + a[3] * t**3
    v = b[0] + b[1] * t + b[2] * t * t + b[3] * t**3
    du = a[1] + 2 * a[2] * t + 3 * a[3] * t * t
    dv = b[1] + 2 * b[2] * t + 3 * b[3] * t * t
    return (
        x + cos_h * u - sin_h * v,
        y + sin_h * u + cos_h * v,
        heading + math.atan2(dv, du),
    )


def _lane_centre_offset(road: ET._Element, lane_id: int) -> Optional[float]:
    """Signed ``t`` of a lane's centre, from the cumulative constant widths."""
    section = road.find("lanes/laneSection")
    if section is None:
        return None
    side = "left" if lane_id > 0 else "right"
    container = section.find(side)
    if container is None:
        return None
    sign = 1 if lane_id > 0 else -1
    edge = 0.0
    for lane in sorted(
        container.findall("lane"), key=lambda e: sign * int(e.get("id"))
    ):
        width = lane.find("width")
        value = float(width.get("a")) if width is not None else 0.0
        if int(lane.get("id")) == lane_id:
            return sign * (edge + value / 2.0)
        edge += value
    return None


def _lane_point(road: ET._Element, lane_id: int, at_end: bool):
    """World position of a lane's centre at one end of the road."""

    geometries = road.findall("planView/geometry")
    if not geometries:
        return None
    geometry = geometries[-1] if at_end else geometries[0]
    station = float(geometry.get("length")) if at_end else 0.0
    x, y, heading = _road_frame(geometry, station)
    offset = _lane_centre_offset(road, lane_id)
    if offset is None:
        return None
    return (x - math.sin(heading) * offset, y + math.cos(heading) * offset)


def _translate_connector(connector: ET._Element, dx: float, dy: float) -> None:
    """Move a connecting road rigidly by ``(dx, dy)``.

    Every ``<geometry>`` origin takes the same vector, so the chain keeps its
    shape exactly: curvature, arc length and the C0 joints between segments
    are all untouched. An earlier attempt slid each segment along its *own*
    normal by a ramp, which pulled the segments apart — 0.108 m at the worst
    joint — and left the ``length`` attributes describing a curve that no
    longer existed. A rigid move cannot do either.
    """
    for geometry in connector.findall("planView/geometry"):
        geometry.set("x", str(float(geometry.get("x")) + dx))
        geometry.set("y", str(float(geometry.get("y")) + dy))


def _joint_lane_pairs(root: ET._Element, connector: ET._Element, connection):
    """``[(side, own lane id, neighbour road, neighbour lane id)]`` for a connector."""
    roads = {road.get("id"): road for road in root.iter("road")}
    link = connector.find("link")
    if link is None:
        return []
    out = []
    for side in ("predecessor", "successor"):
        end = link.find(side)
        if end is None or end.get("elementType") != "road":
            continue
        neighbour = roads.get(end.get("elementId"))
        if neighbour is None:
            continue
        if side == "predecessor":
            pairs = [
                (int(e.get("to")), int(e.get("from")))
                for e in connection.findall("laneLink")
            ]
        else:
            pairs = []
            for lane in connector.iter("lane"):
                if lane.get("type") not in _VISSIM_IMPORTED_LANE_TYPES:
                    continue
                lane_link = lane.find("link")
                far = lane_link.find("successor") if lane_link is not None else None
                if far is not None:
                    pairs.append((int(lane.get("id")), int(far.get("id"))))
        for own, other in pairs:
            out.append((side, own, neighbour, other))
    return out


def align_connector_lanes(root: ET._Element) -> List[Tuple[str, float, float]]:
    """Slide each connector so its lane meets the lane it links to.

    A road's lane centre follows from its cumulative lane widths, so
    constant-izing those widths moves it — by more the further the lane sits
    from the reference line. The connectors keep the geometry of the true lane
    centre, so the two no longer meet: on the Odaiba clip 14 of 28
    connector-to-lane joints were out by more than 0.1 m and the worst by
    0.58 m. Vissim snaps the connector end onto the link's lane, which is what
    bends the connector into a visible kink at each end.

    The connector is moved **rigidly** by the mean of the errors at its two
    ends, so its geometry keeps its shape: no segment is pulled away from its
    neighbour and no ``length`` attribute stops describing its curve. What a
    rigid move cannot remove is the difference between the two ends; that
    residual is reported.

    Returns ``(connector_id, applied_shift, residual)`` per adjusted road.
    """
    roads = {road.get("id"): road for road in root.iter("road")}
    adjusted: List[Tuple[str, float, float]] = []

    for junction in root.iter("junction"):
        for connection in junction.findall("connection"):
            connector = roads.get(connection.get("connectingRoad"))
            if connector is None:
                continue
            link = connector.find("link")
            if link is None:
                continue

            deltas = {}
            for side, at_end in (("predecessor", False), ("successor", True)):
                end = link.find(side)
                if end is None or end.get("elementType") != "road":
                    continue
                neighbour = roads.get(end.get("elementId"))
                if neighbour is None:
                    continue
                neighbour_at_end = end.get("contactPoint") != "start"

                # Which lane pair meets here.
                pairs = []
                if side == "predecessor":
                    pairs = [
                        (int(link_elem.get("to")), int(link_elem.get("from")))
                        for link_elem in connection.findall("laneLink")
                    ]
                else:
                    for lane in connector.iter("lane"):
                        if lane.get("type") != "driving":
                            continue
                        lane_link = lane.find("link")
                        far = (
                            lane_link.find("successor")
                            if lane_link is not None
                            else None
                        )
                        if far is not None:
                            pairs.append((int(lane.get("id")), int(far.get("id"))))
                errors = []
                for own_lane, other_lane in pairs:
                    ours = _lane_point(connector, own_lane, at_end)
                    theirs = _lane_point(neighbour, other_lane, neighbour_at_end)
                    if ours is None or theirs is None:
                        continue
                    # Signed along the connector's normal at that end.
                    geometries = connector.findall("planView/geometry")
                    geometry = geometries[-1] if at_end else geometries[0]
                    station = float(geometry.get("length")) if at_end else 0.0
                    _, _, heading = _road_frame(geometry, station)

                    normal = (-math.sin(heading), math.cos(heading))
                    errors.append(
                        (theirs[0] - ours[0]) * normal[0]
                        + (theirs[1] - ours[1]) * normal[1]
                    )
                if errors:
                    deltas[side] = sum(errors) / len(errors)

            if not deltas:
                continue
            start_delta = deltas.get("predecessor", deltas.get("successor", 0.0))
            end_delta = deltas.get("successor", deltas.get("predecessor", 0.0))
            shift = (start_delta + end_delta) / 2.0
            if abs(shift) < 1e-6:
                continue
            # Along the normal at the connector's start; a rigid move needs one
            # direction, and over a connector the heading varies little.
            geometries = connector.findall("planView/geometry")
            _, _, heading = _road_frame(geometries[0], 0.0)
            _translate_connector(
                connector, -math.sin(heading) * shift, math.cos(heading) * shift
            )
            adjusted.append(
                (connector.get("id"), shift, abs(start_delta - end_delta) / 2.0)
            )
    return adjusted


def _replace_geo_reference(root: ET._Element, proj: str) -> bool:
    """Set the header geoReference to ``proj``; returns True on success."""
    geo_ref = root.find("header/geoReference")
    if geo_ref is None:
        header = root.find("header")
        if header is None:
            return False
        geo_ref = ET.SubElement(header, "geoReference")
    geo_ref.text = ET.CDATA(proj)
    return True


def _mean_lane_width(width_elems: List[ET._Element], section_length: float) -> float:
    """Arc-length-weighted mean of a lane's piecewise-cubic width profile.

    Each ``<width>`` record covers ``[sOffset_i, sOffset_{i+1})`` (the last
    one runs to the end of the lane section) with
    ``w(ds) = a + b·ds + c·ds² + d·ds³``. The exact integral over a span
    ``h`` is ``a·h + b·h²/2 + c·h³/3 + d·h⁴/4``.
    """
    total_area = 0.0
    total_span = 0.0
    for i, width in enumerate(width_elems):
        s_start = float(width.get("sOffset", "0"))
        s_end = (
            float(width_elems[i + 1].get("sOffset", "0"))
            if i + 1 < len(width_elems)
            else section_length
        )
        h = max(s_end - s_start, 0.0)
        if h == 0.0:
            continue
        a = float(width.get("a", "0"))
        b = float(width.get("b", "0"))
        c = float(width.get("c", "0"))
        d = float(width.get("d", "0"))
        total_area += a * h + b * h * h / 2.0 + c * h**3 / 3.0 + d * h**4 / 4.0
        total_span += h
    if total_span == 0.0:
        # Zero-length section (degenerate stub): fall back to the first
        # record's constant term.
        return float(width_elems[0].get("a", "0"))
    return total_area / total_span


def _constantize_lane_widths(root: ET._Element) -> int:
    """Replace each lane's ``<width>`` chain with one constant record.

    Vissim defines lane width as a constant; polynomial records only feed
    its ≥ 0.25 m width-change machinery, which inserts a connector and two
    1.1 m links per variation and fragments the network. Returns the number
    of lanes whose records were collapsed.
    """
    constantized = 0
    for road in root.iter("road"):
        road_length = float(road.get("length", "0"))
        lanes_elem = road.find("lanes")
        if lanes_elem is None:
            continue
        sections = lanes_elem.findall("laneSection")
        for index, section in enumerate(sections):
            s_section = float(section.get("s", "0"))
            s_next = (
                float(sections[index + 1].get("s", "0"))
                if index + 1 < len(sections)
                else road_length
            )
            section_length = max(s_next - s_section, 0.0)
            for lane in section.iter("lane"):
                widths = lane.findall("width")
                if not widths:
                    continue
                already_constant = len(widths) == 1 and all(
                    abs(float(widths[0].get(k, "0"))) < 1e-12 for k in "bcd"
                )
                if already_constant:
                    continue
                mean = _mean_lane_width(widths, section_length)
                first = widths[0]
                first.set("sOffset", "0.0")
                first.set("a", str(mean))
                first.set("b", "0.0")
                first.set("c", "0.0")
                first.set("d", "0.0")
                for extra in widths[1:]:
                    lane.remove(extra)
                constantized += 1
    return constantized


def _sample_lane_centres(road: ET._Element, count: int = 40):
    """``[(x, y, heading, lane_id, width)]`` along every imported lane."""
    geometries = road.findall("planView/geometry")
    if not geometries:
        return []
    total = sum(float(g.get("length")) for g in geometries)
    if total <= 0.0:
        return []
    section = road.find("lanes/laneSection")
    if section is None:
        return []
    lanes = []
    for side, sign in (("left", 1), ("right", -1)):
        container = section.find(side)
        if container is None:
            continue
        edge = 0.0
        for lane in sorted(
            container.findall("lane"), key=lambda e: sign * int(e.get("id"))
        ):
            width = lane.find("width")
            value = float(width.get("a")) if width is not None else 0.0
            if lane.get("type") in _VISSIM_IMPORTED_LANE_TYPES:
                lanes.append((int(lane.get("id")), sign * (edge + value / 2.0), value))
            edge += value
    if not lanes:
        return []
    out = []
    for index in range(count):
        target = total * index / max(count - 1, 1)
        walked = 0.0
        for geometry in geometries:
            length = float(geometry.get("length"))
            if walked + length >= target - 1e-9:
                x, y, heading = _road_frame(geometry, min(target - walked, length))
                normal = (-math.sin(heading), math.cos(heading))
                for lane_id, offset, width in lanes:
                    out.append(
                        (
                            x + normal[0] * offset,
                            y + normal[1] * offset,
                            heading,
                            lane_id,
                            width,
                        )
                    )
                break
            walked += length
    return out


def _linked_road_pairs(root: ET._Element) -> set:
    """Road id pairs that name each other, directly or through a junction."""
    pairs = set()
    for road in root.iter("road"):
        link = road.find("link")
        if link is None:
            continue
        for side in ("predecessor", "successor"):
            end = link.find(side)
            if end is not None and end.get("elementType") == "road":
                pairs.add(frozenset((road.get("id"), end.get("elementId"))))
    for junction in root.iter("junction"):
        for connection in junction.findall("connection"):
            pairs.add(
                frozenset(
                    (connection.get("incomingRoad"), connection.get("connectingRoad"))
                )
            )
    return pairs


def clip_false_lane_overlaps(
    root: ET._Element,
    *,
    min_width: float = VISSIM_MIN_LANE_WIDTH,
    min_centre_distance: float = 1.5,
    max_heading_diff_deg: float = 45.0,
) -> List[Tuple[str, str, float, float]]:
    """Narrow constant widths that make neighbouring lanes overlap on paper.

    Constant-izing a width sets it to the lane's arc-length mean, so wherever
    the lane is genuinely narrower than that the emitted surface is too wide.
    Two lanes of *different* roads whose centres keep their true spacing then
    overlap for no physical reason — on the Odaiba clip connectors 63 and 64 run
    2.93 m apart carrying 3.32 m and 3.27 m of width, so their surfaces cross by
    0.36 m over 35 m. Vissim raises a conflict area wherever two link or
    connector surfaces meet, which fills the network with priorities between
    lanes no vehicle can be in at once.

    Two guards keep it from trimming a width that is not the problem. Roads
    that name each other — directly or through a junction — are skipped
    outright: they meet at a joint, where their lanes approach by design, and a
    station-sampled distance test is too coarse to tell that from an overlay
    (it cost road 15 lane 2 and road 16 lane 1 more than 1.5 m each before this
    guard existed). Of the rest, only pairs whose centres stay at least
    ``min_centre_distance`` apart are touched, which is what distinguishes
    "adjacent lanes, widths too generous" from two roads genuinely laid on top
    of one another — a topology defect that must stay visible. Widths shrink in
    proportion and never below Vissim's 1 m clamp, which would undo the change.

    Returns ``(road_a, road_b, overlap_before, overlap_after)`` per pair fixed.
    """
    roads = [road for road in root.iter("road")]
    samples = {road.get("id"): _sample_lane_centres(road) for road in roads}
    widths: dict = {}
    for road in roads:
        for lane in road.iter("lane"):
            width = lane.find("width")
            if width is not None and lane.get("type") in _VISSIM_IMPORTED_LANE_TYPES:
                widths[(road.get("id"), int(lane.get("id")))] = width

    limit = math.radians(max_heading_diff_deg)
    linked = _linked_road_pairs(root)
    fixed: List[Tuple[str, str, float, float]] = []
    for first, second in itertools.combinations(roads, 2):
        if frozenset((first.get("id"), second.get("id"))) in linked:
            continue
        worst = 0.0
        worst_key = None
        for xa, ya, ha, la, wa in samples.get(first.get("id"), []):
            for xb, yb, hb, lb, wb in samples.get(second.get("id"), []):
                delta = abs((hb - ha + math.pi) % (2 * math.pi) - math.pi)
                if delta > limit:
                    continue
                distance = math.hypot(xa - xb, ya - yb)
                if distance < min_centre_distance:
                    # Genuinely stacked: a topology defect, not a width to trim.
                    worst_key = None
                    worst = 0.0
                    break
                overlap = (wa + wb) / 2.0 - distance
                if overlap > worst:
                    worst, worst_key = overlap, (la, lb, wa, wb, distance)
            else:
                continue
            break
        if worst <= 0.0 or worst_key is None:
            continue
        la, lb, wa, wb, distance = worst_key
        # Split the excess between the two lanes, in proportion to their width.
        share = wa + wb
        new_a = wa - worst * (wa / share) * 2.0
        new_b = wb - worst * (wb / share) * 2.0
        if new_a < min_width or new_b < min_width:
            continue
        element_a = widths.get((first.get("id"), la))
        element_b = widths.get((second.get("id"), lb))
        if element_a is None or element_b is None:
            continue
        element_a.set("a", str(new_a))
        element_b.set("a", str(new_b))
        fixed.append(
            (
                first.get("id"),
                second.get("id"),
                worst,
                (new_a + new_b) / 2.0 - distance,
            )
        )
    return fixed


def _merge_overlapping_junctions(root: ET._Element) -> int:
    """Merge junctions whose connecting roads share a road attachment point.

    The divergence synthesis can emit two junctions whose connecting roads
    terminate at the *same* road endpoint (e.g. two merge branches ending
    at one road start). Vissim builds one node per ``<junction>`` and both
    node areas then cover the shared endpoint, producing "Nodes … overlap
    on link …" errors and undetermined conflict areas. Merging the
    junctions yields a single node, which cannot overlap itself.

    Returns the number of junctions dissolved into another one.
    """
    roads = {r.get("id"): r for r in root.iter("road")}
    junction_elems = {j.get("id"): j for j in root.findall("junction")}

    # Attachment point -> junctions touching it, via the road-level links
    # of each junction's connecting roads.
    attachments: dict = {}
    for jid, junction in junction_elems.items():
        for connection in junction.findall("connection"):
            connecting = roads.get(connection.get("connectingRoad"))
            if connecting is None:
                continue
            link = connecting.find("link")
            if link is None:
                continue
            for tag in ("predecessor", "successor"):
                e = link.find(tag)
                if e is not None and e.get("elementType") == "road":
                    key = (e.get("elementId"), e.get("contactPoint"))
                    attachments.setdefault(key, set()).add(jid)

    # Union-find over junctions sharing any attachment point.
    parent = {jid: jid for jid in junction_elems}

    def find(x: str) -> str:
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    for ids in attachments.values():
        ids = sorted(ids)
        for other in ids[1:]:
            ra, rb = find(ids[0]), find(other)
            if ra != rb:
                parent[rb] = ra

    groups: dict = {}
    for jid in junction_elems:
        groups.setdefault(find(jid), []).append(jid)

    merged = 0
    for group in groups.values():
        if len(group) < 2:
            continue
        # Deterministic primary: numerically smallest id when possible.
        group.sort(key=lambda s: (int(s) if s.isdigit() else 1 << 62, s))
        primary_id, rest = group[0], group[1:]
        primary = junction_elems[primary_id]
        for jid in rest:
            junction = junction_elems[jid]
            # Move connections / priority records into the primary junction.
            for child in list(junction):
                junction.remove(child)
                primary.append(child)
            # Rewrite references: connecting roads' junction attribute and
            # every road-level link that names the dissolved junction.
            for road in roads.values():
                if road.get("junction") == jid:
                    road.set("junction", primary_id)
                link = road.find("link")
                if link is None:
                    continue
                for e in link:
                    if e.get("elementType") == "junction" and e.get("elementId") == jid:
                        e.set("elementId", primary_id)
            root.remove(junction)
            merged += 1
        # Connection ids must stay unique within the merged junction.
        for index, connection in enumerate(primary.findall("connection")):
            connection.set("id", str(index))
    return merged


def _width_samples(width_elems: List[ET._Element], road_length: float) -> List[float]:
    """Sample each width polynomial at the start/middle/end of its domain."""
    samples: List[float] = []
    for i, width in enumerate(width_elems):
        s_start = float(width.get("sOffset", "0"))
        s_end = (
            float(width_elems[i + 1].get("sOffset", "0"))
            if i + 1 < len(width_elems)
            else road_length
        )
        span = max(s_end - s_start, 0.0)
        a = float(width.get("a", "0"))
        b = float(width.get("b", "0"))
        c = float(width.get("c", "0"))
        d = float(width.get("d", "0"))
        for fraction in (0.0, 0.5, 1.0):
            ds = span * fraction
            samples.append(a + b * ds + c * ds * ds + d * ds * ds * ds)
    return samples


def _collect_import_indicators(root: ET._Element, report: VissimProfileReport) -> None:
    """Fill the Vissim import risk indicators of ``report`` in place."""
    for road in root.iter("road"):
        road_id = road.get("id", "?")
        length = float(road.get("length", "0"))
        if length < VISSIM_MIN_SPLINE_SPACING:
            report.roads_below_spline_spacing.append(road_id)
        if length < VISSIM_INSERTED_LINK_LENGTH:
            report.roads_below_inserted_link_length.append(road_id)

        lanes_elem = road.find("lanes")
        if lanes_elem is None:
            continue
        for lane_section in lanes_elem.iter("laneSection"):
            for side in ("left", "right"):
                side_elem = lane_section.find(side)
                if side_elem is None:
                    continue
                for lane in side_elem.findall("lane"):
                    if lane.get("type") not in _VISSIM_IMPORTED_LANE_TYPES:
                        continue
                    widths = lane.findall("width")
                    if not widths:
                        continue
                    report.checked_lanes += 1
                    samples = _width_samples(widths, length)
                    if min(samples) < VISSIM_MIN_LANE_WIDTH:
                        report.lanes_below_min_width += 1
                    if max(samples) - min(samples) >= VISSIM_WIDTH_SWING_THRESHOLD:
                        report.lanes_above_width_swing_threshold += 1


def apply_vissim_profile(
    root: ET._Element, config: VissimConfig
) -> VissimProfileReport:
    """Apply the Vissim export profile to a serialized OpenDRIVE tree.

    Args:
        root: The ``<OpenDRIVE>`` root element (mutated in place).
        config: Profile configuration. ``config.enabled`` is not checked
            here — callers decide whether to invoke the profile.

    Returns:
        A :class:`VissimProfileReport` describing the edits and the
        remaining Vissim import risk indicators.
    """
    report = VissimProfileReport()

    if config.strip_nonstandard_attributes:
        report.rule_attributes_stripped = _strip_rule_attributes(root)

    if config.param_poly3_p_range == "normalized":
        report.param_poly3_reparameterized = _reparameterize_param_poly3(root)

    if config.local_geo_reference and config.local_geo_reference_proj:
        report.geo_reference_replaced = _replace_geo_reference(
            root, config.local_geo_reference_proj
        )

    if config.constant_lane_widths:
        report.lanes_width_constantized = _constantize_lane_widths(root)
        if config.clip_false_lane_overlaps:
            report.false_overlaps_clipped = clip_false_lane_overlaps(root)
        if config.align_connector_lanes:
            report.connectors_realigned = align_connector_lanes(root)

    report.elevation_shift = _shift_elevation(root, config.elevation_baseline)

    if config.merge_overlapping_junctions:
        report.junctions_merged = _merge_overlapping_junctions(root)

    # Indicators are collected after all edits so they describe the file
    # Vissim will actually see.
    _collect_import_indicators(root, report)
    return report


def evaluate_param_poly3(
    coefficients: Tuple[float, float, float, float], p: float
) -> float:
    """Evaluate ``a + b·p + c·p² + d·p³`` (shared by tests)."""
    a, b, c, d = coefficients
    return a + b * p + c * p * p + d * p * p * p


#: Columns of the Vissim signal placement CSV, in order.
VISSIM_SIGNAL_CSV_COLUMNS = (
    "signal_id",
    "signal_name",
    "signal_kind",
    "opendrive_type",
    "road_id",
    "vissim_link_name",
    "vissim_lane_index",
    "s_along_road_m",
    "t_offset_m",
    "z_offset_m",
    "orientation",
    "controller_ids",
)


def _signal_lane_index(road: ET._Element, t: float) -> int:
    """Which lane of the road the signal's ``t`` falls in, counted from inside.

    Vissim places a Signal Head on a lane, not at a ``t``, so the offset has to
    be resolved against the cumulative widths — the same walk that gives a lane
    its centre.
    """
    section = road.find("lanes/laneSection")
    if section is None:
        return 0
    side = "left" if t >= 0 else "right"
    container = section.find(side)
    if container is None:
        return 0
    sign = 1 if t >= 0 else -1
    edge = 0.0
    for index, lane in enumerate(
        sorted(container.findall("lane"), key=lambda e: sign * int(e.get("id"))),
        start=1,
    ):
        width = lane.find("width")
        value = float(width.get("a")) if width is not None else 0.0
        if abs(t) <= edge + value:
            return index
        edge += value
    return 0


def signal_table_rows(root: ET._Element) -> List[dict]:
    """Every signal, with the Vissim link and lane a Signal Head belongs on.

    Vissim imports no signalization at all (manual 2.6.9), so the ``<signal>``
    records travel with the file for other consumers and a modeller has to place
    Signal Heads by hand. What that costs is looking up a coordinate per head;
    this table replaces the lookup, because the road id in the label is exactly
    what Vissim reports back as the middle field of a link name.

    ``controller_ids`` lists the ``<controller>`` elements whose control record
    names the signal, which is the grouping a Vissim Signal Controller wants.
    """
    controllers: dict = {}
    for controller in root.findall("controller"):
        for control in controller.findall("control"):
            controllers.setdefault(control.get("signalId"), []).append(
                controller.get("id")
            )
    rows: List[dict] = []
    for road in root.iter("road"):
        lanes = road.find("lanes/laneSection")
        side = "Left"
        if lanes is not None and lanes.find("left") is None:
            side = "Right"
        connector = road.get("junction") not in ("-1", None)
        for signal in road.iter("signal"):
            t = float(signal.get("t") or 0.0)
            name = signal.get("name") or ""
            kind = (
                "traffic_light"
                if signal.get("dynamic") == "yes"
                else "stop_line"
                if "StopLine" in name
                else "static"
            )
            rows.append(
                {
                    "signal_id": signal.get("id"),
                    "signal_name": name,
                    "signal_kind": kind,
                    "opendrive_type": signal.get("type"),
                    "road_id": road.get("id"),
                    "vissim_link_name": (
                        f"Road_{road.get('id')}-0-{side}"
                        + (" Connector" if connector else "")
                    ),
                    "vissim_lane_index": _signal_lane_index(road, t),
                    "s_along_road_m": round(float(signal.get("s") or 0.0), 3),
                    "t_offset_m": round(t, 3),
                    "z_offset_m": round(float(signal.get("zOffset") or 0.0), 3),
                    "orientation": signal.get("orientation") or "",
                    "controller_ids": " ".join(
                        sorted(controllers.get(signal.get("id"), []))
                    ),
                }
            )
    rows.sort(key=lambda row: (row["signal_kind"], int(row["road_id"])))
    return rows


def write_vissim_signal_table(path, root: ET._Element) -> int:
    """Write :func:`signal_table_rows` to ``path`` as CSV."""
    import csv
    from pathlib import Path

    rows = signal_table_rows(root)
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open("w", newline="", encoding="utf-8") as handle:
        handle.write(
            "# Vissim imports no signalization; place Signal Heads on the link "
            "and lane below, at s_along_road_m from the link start.\n"
        )
        writer = csv.DictWriter(handle, fieldnames=list(VISSIM_SIGNAL_CSV_COLUMNS))
        writer.writeheader()
        writer.writerows(rows)
    return len(rows)
