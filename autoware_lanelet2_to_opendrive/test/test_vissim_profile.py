"""Tests for the PTV Vissim export profile (``vissim_profile``).

Covers the three write-time transformations (rule-attribute stripping,
paramPoly3 re-parameterization, geoReference replacement), the import risk
report, and the local-frame PROJ string — including a frame-alignment check
against the actual projectors used by the converter.
"""

import math

import lxml.etree as ET
import pytest

from autoware_lanelet2_to_opendrive.vissim_profile import (
    VissimConfig,
    apply_vissim_profile,
    evaluate_param_poly3,
    local_frame_proj_string,
)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


def _sample_tree() -> ET._Element:
    """A miniature OpenDRIVE tree with every construct the profile edits."""
    xml = """
    <OpenDRIVE>
      <header revMajor="1" revMinor="4" name="t">
        <geoReference><![CDATA[+proj=utm +zone=54 +lat_0=35.0 +lon_0=139.8 +datum=WGS84 +units=m +no_defs]]></geoReference>
      </header>
      <road id="1" length="10.0" junction="-1" rule="LHT">
        <elevationProfile>
          <elevation s="0.0" a="5.0" b="0.1" c="0.0" d="0.0"/>
          <elevation s="4.0" a="5.4" b="0.05" c="0.0" d="0.0"/>
        </elevationProfile>
        <planView>
          <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="4.0">
            <paramPoly3 aU="0.0" bU="1.0" cU="0.01" dU="0.001"
                        aV="0.0" bV="0.0" cV="0.02" dV="-0.002"
                        pRange="arcLength"/>
          </geometry>
          <geometry s="4.0" x="4.0" y="0.0" hdg="0.0" length="6.0">
            <line/>
          </geometry>
        </planView>
        <lanes>
          <laneSection s="0.0">
            <left>
              <lane id="1" type="driving" level="false" rule="LHT">
                <width sOffset="0.0" a="3.0" b="0.0" c="0.0" d="0.0"/>
                <access sOffset="0.0" rule="allow" restriction="passengerCar"/>
              </lane>
              <lane id="2" type="sidewalk" level="false">
                <width sOffset="0.0" a="0.4" b="0.0" c="0.0" d="0.0"/>
              </lane>
            </left>
            <center>
              <lane id="0" type="none" level="false"/>
            </center>
          </laneSection>
        </lanes>
      </road>
      <road id="2" length="0.01" junction="1000">
        <elevationProfile>
          <elevation s="0.0" a="7.0" b="0.0" c="0.0" d="0.0"/>
        </elevationProfile>
        <planView>
          <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="0.01">
            <line/>
          </geometry>
        </planView>
        <lanes>
          <laneSection s="0.0">
            <left>
              <lane id="1" type="driving" level="false">
                <width sOffset="0.0" a="0.5" b="0.3" c="0.0" d="0.0"/>
              </lane>
            </left>
            <center><lane id="0" type="none" level="false"/></center>
          </laneSection>
        </lanes>
      </road>
      <controller id="0">
        <positionInertial x="1.0" y="2.0" z="12.0"/>
      </controller>
    </OpenDRIVE>
    """
    return ET.fromstring(xml.encode())


# ---------------------------------------------------------------------------
# VissimConfig validation
# ---------------------------------------------------------------------------


def test_config_rejects_unknown_p_range_mode():
    with pytest.raises(ValueError, match="param_poly3_p_range"):
        VissimConfig(param_poly3_p_range="bogus")


# ---------------------------------------------------------------------------
# rule attribute stripping
# ---------------------------------------------------------------------------


def test_strips_rule_attributes_from_road_lane_and_access():
    root = _sample_tree()
    report = apply_vissim_profile(root, VissimConfig(enabled=True))

    assert report.rule_attributes_stripped == 3
    assert "rule" not in root.find("road").attrib
    for lane in root.iter("lane"):
        assert "rule" not in lane.attrib
    for access in root.iter("access"):
        assert "rule" not in access.attrib
    # Non-rule attributes are untouched.
    assert root.find("road").get("junction") == "-1"
    assert root.find(".//access").get("restriction") == "passengerCar"


def test_stripping_can_be_disabled():
    root = _sample_tree()
    apply_vissim_profile(
        root,
        VissimConfig(
            enabled=True,
            strip_nonstandard_attributes=False,
            param_poly3_p_range="arcLength",
        ),
    )
    assert root.find("road").get("rule") == "LHT"


# ---------------------------------------------------------------------------
# paramPoly3 re-parameterization
# ---------------------------------------------------------------------------


def test_normalized_mode_rescales_coefficients_exactly():
    root = _sample_tree()
    geometry = root.find(".//geometry")
    length = float(geometry.get("length"))
    poly = geometry.find("paramPoly3")
    original = {k: float(poly.get(k)) for k in poly.keys() if k != "pRange"}

    report = apply_vissim_profile(root, VissimConfig(enabled=True))
    assert report.param_poly3_reparameterized == 1
    assert poly.get("pRange") is None

    # The re-parameterized polynomial evaluated at p̂ = s / L must equal the
    # original arc-length polynomial at s over the whole domain.
    for prefix in ("U", "V"):
        arc_coeffs = tuple(original[f"{c}{prefix}"] for c in "abcd")
        norm_coeffs = tuple(float(poly.get(f"{c}{prefix}")) for c in "abcd")
        for s in (0.0, 0.5, 1.7, length):
            expected = evaluate_param_poly3(arc_coeffs, s)
            actual = evaluate_param_poly3(norm_coeffs, s / length)
            assert math.isclose(expected, actual, rel_tol=0.0, abs_tol=1e-12)


def test_arc_length_mode_keeps_coefficients_and_attribute():
    root = _sample_tree()
    report = apply_vissim_profile(
        root, VissimConfig(enabled=True, param_poly3_p_range="arcLength")
    )
    poly = root.find(".//paramPoly3")
    assert report.param_poly3_reparameterized == 0
    assert poly.get("pRange") == "arcLength"
    assert float(poly.get("bU")) == 1.0


def test_line_geometries_are_untouched():
    root = _sample_tree()
    apply_vissim_profile(root, VissimConfig(enabled=True))
    lines = [g for g in root.iter("geometry") if g.find("line") is not None]
    assert len(lines) == 2
    for line_geom in lines:
        assert line_geom.find("paramPoly3") is None


# ---------------------------------------------------------------------------
# geoReference replacement
# ---------------------------------------------------------------------------


def test_geo_reference_replaced_when_proj_provided():
    root = _sample_tree()
    proj = "+proj=tmerc +lat_0=0 +lon_0=141 +k=0.9996 +x_0=1.0 +y_0=2.0"
    report = apply_vissim_profile(
        root,
        VissimConfig(enabled=True, local_geo_reference_proj=proj),
    )
    assert report.geo_reference_replaced
    assert root.find("header/geoReference").text == proj


def test_geo_reference_kept_without_proj_string():
    root = _sample_tree()
    report = apply_vissim_profile(root, VissimConfig(enabled=True))
    assert not report.geo_reference_replaced
    assert "+proj=utm" in root.find("header/geoReference").text


# ---------------------------------------------------------------------------
# import risk report
# ---------------------------------------------------------------------------


def test_report_flags_short_roads_and_width_indicators():
    root = _sample_tree()
    report = apply_vissim_profile(root, VissimConfig(enabled=True))

    # Road 2 is 0.01 m long: below both thresholds.
    assert report.roads_below_spline_spacing == ["2"]
    assert report.roads_below_inserted_link_length == ["2"]

    # Two driving lanes are checked (the 0.4 m sidewalk is ignored by
    # Vissim and therefore not counted). Road 2's lane starts at 0.5 m.
    assert report.checked_lanes == 2
    assert report.lanes_below_min_width == 1
    # After constantization no lane can swing at all.
    assert report.lanes_above_width_swing_threshold == 0
    # Only road 2's lane had a non-constant width chain.
    assert report.lanes_width_constantized == 1


# ---------------------------------------------------------------------------
# constant lane widths
# ---------------------------------------------------------------------------


def _lane_with_widths(records: str, length: float) -> ET._Element:
    xml = f"""
    <OpenDRIVE>
      <header revMajor="1" revMinor="4"/>
      <road id="1" length="{length}" junction="-1">
        <planView>
          <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="{length}"><line/></geometry>
        </planView>
        <lanes>
          <laneSection s="0.0">
            <left>
              <lane id="1" type="driving" level="false">
                {records}
              </lane>
            </left>
            <center><lane id="0" type="none" level="false"/></center>
          </laneSection>
        </lanes>
      </road>
    </OpenDRIVE>
    """
    return ET.fromstring(xml.encode())


def test_constant_lane_widths_arc_length_weighted_mean():
    # Two records over a 10 m road: 4 m of linear ramp 2→3 m
    # (a=2, b=0.25), then 6 m constant 3 m.
    # Exact mean = (∫₀⁴(2+0.25s)ds + 3·6) / 10 = (10 + 18) / 10 = 2.8.
    root = _lane_with_widths(
        '<width sOffset="0.0" a="2.0" b="0.25" c="0.0" d="0.0"/>'
        '<width sOffset="4.0" a="3.0" b="0.0" c="0.0" d="0.0"/>',
        length=10.0,
    )
    report = apply_vissim_profile(root, VissimConfig(enabled=True))
    assert report.lanes_width_constantized == 1

    widths = root.findall(".//lane[@id='1']/width")
    assert len(widths) == 1
    assert float(widths[0].get("sOffset")) == 0.0
    assert float(widths[0].get("a")) == pytest.approx(2.8, abs=1e-12)
    assert all(float(widths[0].get(k)) == 0.0 for k in "bcd")


def test_constant_lane_widths_keeps_already_constant_record():
    root = _lane_with_widths(
        '<width sOffset="0.0" a="3.5" b="0.0" c="0.0" d="0.0"/>', length=10.0
    )
    report = apply_vissim_profile(root, VissimConfig(enabled=True))
    assert report.lanes_width_constantized == 0
    assert float(root.find(".//lane[@id='1']/width").get("a")) == 3.5


def _twin_junction_tree() -> ET._Element:
    """Two junctions whose connecting roads both end at road 9's start."""
    xml = """
    <OpenDRIVE>
      <header revMajor="1" revMinor="4"/>
      <road id="31" length="75.0" junction="-1">
        <planView><geometry s="0" x="0" y="0" hdg="0" length="75.0"><line/></geometry></planView>
        <link><successor elementType="junction" elementId="1001"/></link>
        <lanes><laneSection s="0.0"><center><lane id="0" type="none" level="false"/></center></laneSection></lanes>
      </road>
      <road id="53" length="29.0" junction="1001">
        <planView><geometry s="0" x="0" y="0" hdg="0" length="29.0"><line/></geometry></planView>
        <link>
          <predecessor elementType="road" elementId="31" contactPoint="end"/>
          <successor elementType="road" elementId="9" contactPoint="start"/>
        </link>
        <lanes><laneSection s="0.0"><center><lane id="0" type="none" level="false"/></center></laneSection></lanes>
      </road>
      <road id="59" length="1.8" junction="1002">
        <planView><geometry s="0" x="0" y="0" hdg="0" length="1.8"><line/></geometry></planView>
        <link>
          <predecessor elementType="road" elementId="12" contactPoint="end"/>
          <successor elementType="road" elementId="9" contactPoint="start"/>
        </link>
        <lanes><laneSection s="0.0"><center><lane id="0" type="none" level="false"/></center></laneSection></lanes>
      </road>
      <road id="12" length="25.0" junction="-1">
        <planView><geometry s="0" x="0" y="0" hdg="0" length="25.0"><line/></geometry></planView>
        <link>
          <predecessor elementType="junction" elementId="1001"/>
          <successor elementType="junction" elementId="1002"/>
        </link>
        <lanes><laneSection s="0.0"><center><lane id="0" type="none" level="false"/></center></laneSection></lanes>
      </road>
      <road id="9" length="86.0" junction="-1">
        <planView><geometry s="0" x="0" y="0" hdg="0" length="86.0"><line/></geometry></planView>
        <link><predecessor elementType="junction" elementId="1002"/></link>
        <lanes><laneSection s="0.0"><center><lane id="0" type="none" level="false"/></center></laneSection></lanes>
      </road>
      <junction id="1001" name="junction_a">
        <connection id="0" incomingRoad="31" connectingRoad="53" contactPoint="start">
          <laneLink from="1" to="1"/>
        </connection>
      </junction>
      <junction id="1002" name="junction_b">
        <connection id="0" incomingRoad="12" connectingRoad="59" contactPoint="start">
          <laneLink from="1" to="1"/>
        </connection>
      </junction>
      <junction id="1003" name="junction_far">
        <connection id="0" incomingRoad="9" connectingRoad="31" contactPoint="start"/>
      </junction>
    </OpenDRIVE>
    """
    return ET.fromstring(xml.encode())


def test_merge_overlapping_junctions_dissolves_colocated_pair():
    root = _twin_junction_tree()
    report = apply_vissim_profile(root, VissimConfig(enabled=True))
    assert report.junctions_merged == 1

    junctions = root.findall("junction")
    ids = {j.get("id") for j in junctions}
    # 1002 dissolved into 1001; unrelated 1003 untouched.
    assert ids == {"1001", "1003"}

    merged = next(j for j in junctions if j.get("id") == "1001")
    conns = merged.findall("connection")
    assert len(conns) == 2
    # Connection ids renumbered uniquely.
    assert sorted(c.get("id") for c in conns) == ["0", "1"]
    assert {c.get("connectingRoad") for c in conns} == {"53", "59"}

    # References rewritten: connecting road 59 and every junction link.
    road_59 = root.find("road[@id='59']")
    assert road_59.get("junction") == "1001"
    road_9 = root.find("road[@id='9']")
    assert road_9.find("link/predecessor").get("elementId") == "1001"
    road_12 = root.find("road[@id='12']")
    assert road_12.find("link/successor").get("elementId") == "1001"


def test_merge_overlapping_junctions_can_be_disabled():
    root = _twin_junction_tree()
    report = apply_vissim_profile(
        root, VissimConfig(enabled=True, merge_overlapping_junctions=False)
    )
    assert report.junctions_merged == 0
    assert len(root.findall("junction")) == 3


def test_constant_lane_widths_can_be_disabled():
    root = _lane_with_widths(
        '<width sOffset="0.0" a="2.0" b="0.25" c="0.0" d="0.0"/>'
        '<width sOffset="4.0" a="3.0" b="0.0" c="0.0" d="0.0"/>',
        length=10.0,
    )
    report = apply_vissim_profile(
        root, VissimConfig(enabled=True, constant_lane_widths=False)
    )
    assert report.lanes_width_constantized == 0
    assert len(root.findall(".//lane[@id='1']/width")) == 2


# ---------------------------------------------------------------------------
# local-frame PROJ string
# ---------------------------------------------------------------------------


def test_local_frame_proj_string_structure():
    proj = local_frame_proj_string("54SUE", offset_x=92008.5, offset_y=45335.1)
    assert "+proj=tmerc" in proj
    assert "+lon_0=141" in proj  # zone 54 central meridian
    assert "+k=0.9996" in proj
    assert "+datum=WGS84" in proj
    # Northern hemisphere: y_0 = -N0 must be negative, x_0 = 500000 - E0.
    x_0 = float(proj.split("+x_0=")[1].split()[0])
    y_0 = float(proj.split("+y_0=")[1].split()[0])
    assert y_0 < 0
    # E0 within zone: x_0 + E0 == 500000 with E0 = corner + offset.
    e0 = 500000.0 - x_0
    n0 = -y_0
    assert e0 % 100000 == pytest.approx(92008.5, abs=1e-6)
    assert n0 % 100000 == pytest.approx(45335.1, abs=1e-6)


def test_local_frame_proj_string_rejects_invalid_code():
    with pytest.raises(ValueError, match="Invalid MGRS"):
        local_frame_proj_string("not-a-grid")


def test_local_frame_matches_converter_projection():
    """local(0,0) really is the point (E0, N0): the profile's frame model
    must agree with the MGRSProjector + offset pipeline the converter uses."""
    import lanelet2
    from autoware_lanelet2_extension_python.projection import MGRSProjector

    from autoware_lanelet2_to_opendrive.projection import (
        mgrs_grid_with_offset_to_lanelet2_origin,
    )

    mgrs_code = "54SUE"
    offset_x, offset_y = 92008.5, 45335.1

    proj = local_frame_proj_string(mgrs_code, offset_x, offset_y)
    x_0 = float(proj.split("+x_0=")[1].split()[0])
    y_0 = float(proj.split("+y_0=")[1].split()[0])
    e0 = 500000.0 - x_0
    n0 = -y_0

    origin = mgrs_grid_with_offset_to_lanelet2_origin(mgrs_code, offset_x, offset_y)
    mgrs_projector = MGRSProjector(origin)
    utm_projector = lanelet2.projection.UtmProjector(origin, False, False)

    # Probe points spread over a few hundred metres around the origin.
    for dlat, dlon in ((0.0, 0.0), (0.003, 0.0), (0.0, 0.004), (-0.002, 0.003)):
        gps = lanelet2.core.GPSPoint(
            origin.position.lat + dlat, origin.position.lon + dlon, 0.0
        )
        local = mgrs_projector.forward(gps)
        local_x = local.x - offset_x
        local_y = local.y - offset_y
        absolute = utm_projector.forward(gps)
        # absolute UTM == local + (E0, N0); the MGRS truncation in
        # mgrs_grid_with_offset_to_latlon rounds the origin to whole
        # metres, so allow that much slack.
        assert absolute.x - local_x == pytest.approx(e0, abs=1.0)
        assert absolute.y - local_y == pytest.approx(n0, abs=1.0)


# ---------------------------------------------------------------------------
# elevation baseline
# ---------------------------------------------------------------------------


def _surface_z(root):
    return [float(e.get("a")) for e in root.iter("elevation")]


def _gradients(root):
    return [float(e.get("b", "0")) for e in root.iter("elevation")]


def test_elevation_baseline_min_puts_the_lowest_surface_at_zero():
    """Lanelet2 stores absolute elevation, so the network floats otherwise."""
    root = _sample_tree()
    before = _surface_z(root)
    gradients = _gradients(root)
    assert min(before) == 5.0  # metres above sea level

    report = apply_vissim_profile(root, VissimConfig(enabled=True))

    assert report.elevation_shift == pytest.approx(5.0)
    assert min(_surface_z(root)) == pytest.approx(0.0)
    # Shifted by exactly one constant, so every relative height is preserved.
    assert [z - 5.0 for z in before] == pytest.approx(_surface_z(root))
    # And not a single gradient changed.
    assert _gradients(root) == pytest.approx(gradients)


def test_elevation_baseline_mean_centres_the_network():
    root = _sample_tree()
    before = _surface_z(root)
    expected = sum(before) / len(before)

    report = apply_vissim_profile(
        root, VissimConfig(enabled=True, elevation_baseline="mean")
    )

    assert report.elevation_shift == pytest.approx(expected)
    assert sum(_surface_z(root)) / len(before) == pytest.approx(0.0)


def test_elevation_baseline_explicit_value_shifts_by_that_amount():
    root = _sample_tree()
    before = _surface_z(root)
    report = apply_vissim_profile(
        root, VissimConfig(enabled=True, elevation_baseline=2.5)
    )
    assert report.elevation_shift == pytest.approx(2.5)
    assert [z - 2.5 for z in before] == pytest.approx(_surface_z(root))


def test_elevation_baseline_none_keeps_absolute_elevation():
    root = _sample_tree()
    before = _surface_z(root)
    report = apply_vissim_profile(
        root, VissimConfig(enabled=True, elevation_baseline="none")
    )
    assert report.elevation_shift is None
    assert _surface_z(root) == pytest.approx(before)


def test_elevation_shift_also_moves_absolute_inertial_positions():
    """positionInertial is absolute; zOffset is relative and must not move."""
    root = _sample_tree()
    position = root.find(".//positionInertial")
    assert float(position.get("z")) == 12.0

    apply_vissim_profile(root, VissimConfig(enabled=True))

    assert float(position.get("z")) == pytest.approx(7.0)


def test_config_rejects_unknown_elevation_baseline():
    with pytest.raises(ValueError, match="elevation_baseline"):
        VissimConfig(elevation_baseline="sea-level")


# ---------------------------------------------------------------------------
# connector lane alignment
# ---------------------------------------------------------------------------


def _junction_tree() -> ET._Element:
    """A 2-lane road, a single-lane connector onto its outer lane, and an exit.

    The road's lanes are 3.0 m and 4.0 m wide but taper, so constant-izing
    them moves the outer lane centre and the connector stops meeting it.
    """
    xml = """
    <OpenDRIVE>
      <header revMajor="1" revMinor="4"/>
      <road id="1" length="20.0" junction="-1">
        <link><successor elementType="junction" elementId="900"/></link>
        <elevationProfile><elevation s="0.0" a="0.0" b="0.0" c="0.0" d="0.0"/></elevationProfile>
        <planView><geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="20.0"><line/></geometry></planView>
        <lanes><laneSection s="0.0">
          <left>
            <lane id="1" type="driving" level="false">
              <width sOffset="0.0" a="3.0" b="0.05" c="0.0" d="0.0"/>
            </lane>
            <lane id="2" type="driving" level="false">
              <width sOffset="0.0" a="4.0" b="-0.05" c="0.0" d="0.0"/>
            </lane>
          </left>
          <center><lane id="0" type="none" level="false"/></center>
        </laneSection></lanes>
      </road>
      <road id="2" length="10.0" junction="900">
        <link>
          <predecessor elementType="road" elementId="1" contactPoint="end"/>
          <successor elementType="road" elementId="3" contactPoint="start"/>
        </link>
        <elevationProfile><elevation s="0.0" a="0.0" b="0.0" c="0.0" d="0.0"/></elevationProfile>
        <planView><geometry s="0.0" x="20.0" y="5.5" hdg="0.0" length="10.0"><line/></geometry></planView>
        <lanes><laneSection s="0.0">
          <left>
            <lane id="1" type="driving" level="false">
              <width sOffset="0.0" a="4.0" b="0.0" c="0.0" d="0.0"/>
              <link><successor id="1"/></link>
            </lane>
          </left>
          <center><lane id="0" type="none" level="false"/></center>
        </laneSection></lanes>
      </road>
      <road id="3" length="20.0" junction="-1">
        <link><predecessor elementType="junction" elementId="900"/></link>
        <elevationProfile><elevation s="0.0" a="0.0" b="0.0" c="0.0" d="0.0"/></elevationProfile>
        <planView><geometry s="0.0" x="30.0" y="5.5" hdg="0.0" length="20.0"><line/></geometry></planView>
        <lanes><laneSection s="0.0">
          <left>
            <lane id="1" type="driving" level="false">
              <width sOffset="0.0" a="4.0" b="0.0" c="0.0" d="0.0"/>
            </lane>
          </left>
          <center><lane id="0" type="none" level="false"/></center>
        </laneSection></lanes>
      </road>
      <junction id="900">
        <connection id="0" incomingRoad="1" connectingRoad="2" contactPoint="start">
          <laneLink from="2" to="1"/>
        </connection>
      </junction>
    </OpenDRIVE>
    """
    return ET.fromstring(xml.encode())


def _lane_centre(root, road_id, lane_id, at_end):
    road = root.find(f"road[@id='{road_id}']")
    geometries = road.findall("planView/geometry")
    geometry = geometries[-1] if at_end else geometries[0]
    x = float(geometry.get("x"))
    y = float(geometry.get("y"))
    hdg = float(geometry.get("hdg"))
    if at_end:
        x += math.cos(hdg) * float(geometry.get("length"))
        y += math.sin(hdg) * float(geometry.get("length"))
    edge = 0.0
    for lane in sorted(
        road.findall("lanes/laneSection/left/lane"), key=lambda e: int(e.get("id"))
    ):
        width = float(lane.find("width").get("a"))
        if int(lane.get("id")) == lane_id:
            offset = edge + width / 2.0
            return (x - math.sin(hdg) * offset, y + math.cos(hdg) * offset)
        edge += width
    return None


def test_connector_is_slid_onto_the_lane_it_links():
    root = _junction_tree()
    report = apply_vissim_profile(root, VissimConfig(enabled=True))

    assert len(report.connectors_realigned) == 1
    connector_id, start_shift, _ = report.connectors_realigned[0]
    assert connector_id == "2"
    assert abs(start_shift) > 1e-6

    # The connector's lane centre now meets road 1's lane 2 at the joint.
    ours = _lane_centre(root, "2", 1, at_end=False)
    theirs = _lane_centre(root, "1", 2, at_end=True)
    assert math.dist(ours, theirs) == pytest.approx(0.0, abs=1e-6)


def test_alignment_leaves_the_links_untouched():
    """Only connecting roads move; a link's geometry must not."""
    root = _junction_tree()
    before = [
        (g.get("x"), g.get("y"))
        for g in root.find("road[@id='1']").findall("planView/geometry")
    ]
    apply_vissim_profile(root, VissimConfig(enabled=True))
    after = [
        (g.get("x"), g.get("y"))
        for g in root.find("road[@id='1']").findall("planView/geometry")
    ]
    assert before == after


def test_alignment_can_be_disabled():
    root = _junction_tree()
    report = apply_vissim_profile(
        root, VissimConfig(enabled=True, align_connector_lanes=False)
    )
    assert report.connectors_realigned == []
    assert root.find("road[@id='2']/planView/geometry").get("y") == "5.5"
