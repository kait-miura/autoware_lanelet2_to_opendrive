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
* reports constructs Vissim is known to degrade on (roads shorter than
  its 0.5 m spline spacing / 1.1 m inserted-link length, lane widths
  below the 1 m clamp, width swings beyond the 0.25 m connector
  threshold).
"""

import logging
import re
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import lxml.etree as ET

logger = logging.getLogger(__name__)

# Vissim importer behavioral constants (PTV Vissim manual, openDRIVE import).
VISSIM_MIN_LANE_WIDTH = 1.0  # widths below this are clamped to 1 m
VISSIM_WIDTH_SWING_THRESHOLD = 0.25  # larger swings insert connector + links
VISSIM_INSERTED_LINK_LENGTH = 1.1  # length of links inserted at width changes
VISSIM_MIN_SPLINE_SPACING = 0.5  # minimum spline point spacing

_P_RANGE_MODES = ("arcLength", "normalized")

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
    """

    enabled: bool = False
    strip_nonstandard_attributes: bool = True
    param_poly3_p_range: str = "normalized"
    local_geo_reference: bool = True
    local_geo_reference_proj: Optional[str] = None

    def __post_init__(self) -> None:
        if self.param_poly3_p_range not in _P_RANGE_MODES:
            raise ValueError(
                f"param_poly3_p_range must be one of {_P_RANGE_MODES}, "
                f"got '{self.param_poly3_p_range}'"
            )


@dataclass
class VissimProfileReport:
    """Summary of profile edits and Vissim import risk indicators."""

    rule_attributes_stripped: int = 0
    param_poly3_reparameterized: int = 0
    geo_reference_replaced: bool = False
    roads_below_spline_spacing: List[str] = field(default_factory=list)
    roads_below_inserted_link_length: List[str] = field(default_factory=list)
    lanes_below_min_width: int = 0
    lanes_above_width_swing_threshold: int = 0
    checked_lanes: int = 0

    def log(self, log: logging.Logger = logger) -> None:
        """Emit the report at INFO level (WARNING for degenerate roads)."""
        log.info(
            "Vissim profile: stripped %d rule attributes, "
            "re-parameterized %d paramPoly3 segments, geoReference %s",
            self.rule_attributes_stripped,
            self.param_poly3_reparameterized,
            "replaced" if self.geo_reference_replaced else "kept",
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

    _collect_import_indicators(root, report)
    return report


def evaluate_param_poly3(
    coefficients: Tuple[float, float, float, float], p: float
) -> float:
    """Evaluate ``a + b·p + c·p² + d·p³`` (shared by tests)."""
    a, b, c, d = coefficients
    return a + b * p + c * p * p + d * p * p * p
