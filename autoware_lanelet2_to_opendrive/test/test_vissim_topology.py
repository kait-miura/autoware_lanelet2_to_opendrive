"""Tests for the Vissim topology diagnostics (``vissim_topology``).

The decisive property is that coverage is measured between lane centres with
a direction gate: a connector laid along our carriageway must be reported,
while the opposing carriageway of a two-way road — whose reference line can
run within a metre of ours — must not.
"""

import math

import pytest

from autoware_lanelet2_to_opendrive.opendrive.enums import (
    ContactPoint,
    ElementType,
    LaneType,
)
from autoware_lanelet2_to_opendrive.opendrive.geometry import Line, PlanView
from autoware_lanelet2_to_opendrive.opendrive.junction import (
    Connection,
    Junction,
    LaneLink,
)
from autoware_lanelet2_to_opendrive.opendrive.lane import Lane
from autoware_lanelet2_to_opendrive.opendrive.lane_elements import (
    LaneLink as LaneElementLink,
)
from autoware_lanelet2_to_opendrive.opendrive.lane_elements import LaneWidth
from autoware_lanelet2_to_opendrive.opendrive.lane_section import LaneSection
from autoware_lanelet2_to_opendrive.opendrive.lane_sections import Lanes
from autoware_lanelet2_to_opendrive.opendrive.road import Road
from autoware_lanelet2_to_opendrive.opendrive.road_links import (
    Predecessor,
    RoadLink,
    Successor,
)
from autoware_lanelet2_to_opendrive.opendrive.elevation import (
    Elevation,
    ElevationProfile,
)
from autoware_lanelet2_to_opendrive.vissim_topology import (
    _same_stream_coverage,
    absorb_degenerate_stubs,
    align_connector_elevations,
    analyze_topology,
    dissolve_non_intersection_junctions,
)


def _road(
    road_id: int,
    *,
    x: float,
    y: float,
    hdg: float,
    length: float,
    lane_widths=(3.5,),
    junction: int = -1,
) -> Road:
    """A straight road with ``lane_widths`` left lanes (LHT convention)."""
    section = LaneSection(s_offset=0.0)
    for index, width in enumerate(lane_widths, start=1):
        lane = Lane(lane_id=index, lane_type=LaneType.DRIVING)
        lane.widths = [LaneWidth(s_offset=0.0, a=width)]
        section.left_lanes[index] = lane
    return Road(
        id=road_id,
        length=length,
        junction=junction,
        plan_view=PlanView(geometries=[Line(s=0.0, x=x, y=y, hdg=hdg, length=length)]),
        lanes=Lanes(lane_sections=[section]),
    )


def _connector(
    road_id: int,
    *,
    x: float,
    y: float,
    hdg: float,
    length: float,
    junction: int,
    predecessor: int,
    successor: int,
    lane_widths=(3.5,),
) -> Road:
    road = _road(
        road_id,
        x=x,
        y=y,
        hdg=hdg,
        length=length,
        lane_widths=lane_widths,
        junction=junction,
    )
    road.link = RoadLink(
        predecessor=Predecessor(ElementType.ROAD, predecessor, ContactPoint.END),
        successor=Successor(ElementType.ROAD, successor, ContactPoint.START),
    )
    return road


def _junction(junction_id: int, *connections) -> Junction:
    return Junction(
        id=junction_id,
        name=f"junction_{junction_id}",
        connections=[
            Connection(
                id=index,
                incoming_road=incoming,
                connecting_road=connecting,
                contact_point=ContactPoint.START,
                lane_links=[LaneLink(from_lane=1, to_lane=1)],
            )
            for index, (incoming, connecting) in enumerate(connections)
        ],
    )


# ---------------------------------------------------------------------------
# _same_stream_coverage
# ---------------------------------------------------------------------------


def test_connector_laid_along_a_road_is_full_coverage():
    """A connector on the same lane, same direction, covers the road."""
    road = _road(1, x=0.0, y=0.0, hdg=0.0, length=30.0)
    overlay = _connector(
        2,
        x=-1.0,
        y=0.0,
        hdg=0.0,
        length=32.0,
        junction=100,
        predecessor=1,
        successor=3,
    )
    assert _same_stream_coverage(road, overlay) == pytest.approx(1.0)


def test_opposing_carriageway_is_not_an_overlay():
    """The oncoming side of a two-way road must not count as an overlay.

    Both reference lines run along the same centre line, but each road's
    lanes sit to its own left, so the carriageways are side by side. A
    reference-line comparison would wrongly report a full overlay.
    """
    road = _road(1, x=0.0, y=0.0, hdg=0.0, length=30.0)
    oncoming = _connector(
        2,
        x=30.0,
        y=0.0,
        hdg=math.pi,
        length=30.0,
        junction=100,
        predecessor=5,
        successor=6,
    )
    # Reference lines coincide ...
    assert math.hypot(30.0 - 30.0, 0.0) == pytest.approx(0.0)
    # ... yet no station matches, because the tangents oppose.
    assert _same_stream_coverage(road, oncoming) == pytest.approx(0.0)


def test_laterally_distant_connector_is_not_an_overlay():
    road = _road(1, x=0.0, y=0.0, hdg=0.0, length=30.0)
    far = _connector(
        2,
        x=0.0,
        y=20.0,
        hdg=0.0,
        length=30.0,
        junction=100,
        predecessor=5,
        successor=6,
    )
    assert _same_stream_coverage(road, far) == pytest.approx(0.0)


def test_partial_overlay_is_reported_as_a_fraction():
    """A connector covering the first half of a road scores about 0.5."""
    road = _road(1, x=0.0, y=0.0, hdg=0.0, length=40.0)
    half = _connector(
        2,
        x=0.0,
        y=0.0,
        hdg=0.0,
        length=20.0,
        junction=100,
        predecessor=1,
        successor=3,
    )
    assert _same_stream_coverage(road, half) == pytest.approx(0.5, abs=0.05)


# ---------------------------------------------------------------------------
# analyze_topology — overlays
# ---------------------------------------------------------------------------


def test_analyze_reports_overlay_and_leaves_network_untouched():
    """The Odaiba shape: a connector bypasses the road it runs along.

    Connector 2 links roads 5 and 6, so road 1 — the through road it is laid
    on — is neither of its endpoints, exactly as connecting road 53 runs
    along road 12 while linking roads 31 and 9.
    """
    road = _road(1, x=0.0, y=0.0, hdg=0.0, length=30.0)
    other = _road(3, x=40.0, y=0.0, hdg=0.0, length=30.0)
    overlay = _connector(
        2,
        x=-1.0,
        y=0.0,
        hdg=0.0,
        length=32.0,
        junction=100,
        predecessor=5,
        successor=6,
    )
    roads = [road, other, overlay]
    junctions = [_junction(100, (5, 2))]

    report = analyze_topology(roads, junctions)

    assert len(report.overlaps) == 1
    overlap = report.overlaps[0]
    assert (overlap.road_id, overlap.connector_id, overlap.junction_id) == (1, 2, 100)
    assert overlap.coverage > 0.9
    # Nothing was modified.
    assert [r.id for r in roads] == [1, 3, 2]
    assert len(junctions[0].connections) == 1


def test_connector_endpoints_are_excluded_from_overlay_search():
    """A connector always touches the roads it links; that is not an overlay."""
    road = _road(1, x=0.0, y=0.0, hdg=0.0, length=30.0)
    overlay = _connector(
        2,
        x=-1.0,
        y=0.0,
        hdg=0.0,
        length=32.0,
        junction=100,
        predecessor=1,
        successor=1,
    )
    report = analyze_topology([road, overlay], [_junction(100, (1, 2))])
    assert report.overlaps == []


def test_min_link_coverage_threshold_is_honoured():
    road = _road(1, x=0.0, y=0.0, hdg=0.0, length=40.0)
    half = _connector(
        2,
        x=0.0,
        y=0.0,
        hdg=0.0,
        length=20.0,
        junction=100,
        predecessor=5,
        successor=6,
    )
    junctions = [_junction(100, (5, 2))]
    assert analyze_topology([road, half], junctions, min_link_coverage=0.4).overlaps
    assert not analyze_topology([road, half], junctions, min_link_coverage=0.9).overlaps


# ---------------------------------------------------------------------------
# analyze_topology — degenerate connectors
# ---------------------------------------------------------------------------


def test_degenerate_connector_needing_a_junction_is_flagged_as_such():
    """Two stubs ending at the same road start form a merge: junction needed."""
    left = _road(1, x=0.0, y=0.0, hdg=0.0, length=30.0)
    right = _road(2, x=0.0, y=10.0, hdg=0.0, length=30.0)
    target = _road(3, x=31.0, y=5.0, hdg=0.0, length=30.0, lane_widths=(3.5, 3.5))
    stub_a = _connector(
        4,
        x=30.0,
        y=0.0,
        hdg=0.0,
        length=0.01,
        junction=100,
        predecessor=1,
        successor=3,
    )
    stub_b = _connector(
        5,
        x=30.0,
        y=10.0,
        hdg=0.0,
        length=0.01,
        junction=100,
        predecessor=2,
        successor=3,
    )
    report = analyze_topology(
        [left, right, target, stub_a, stub_b], [_junction(100, (1, 4), (2, 5))]
    )

    assert [s.road_id for s in report.degenerate_connectors] == [4, 5]
    assert all(s.junction_required for s in report.degenerate_connectors)
    assert report.degenerate_connectors[0].length == pytest.approx(0.01)
    assert (
        report.degenerate_connectors[0].from_road,
        report.degenerate_connectors[0].to_road,
    ) == (1, 3)


def test_lone_degenerate_connector_could_become_a_direct_link():
    source = _road(1, x=0.0, y=0.0, hdg=0.0, length=30.0)
    target = _road(2, x=30.01, y=0.0, hdg=0.0, length=30.0)
    stub = _connector(
        3,
        x=30.0,
        y=0.0,
        hdg=0.0,
        length=0.01,
        junction=100,
        predecessor=1,
        successor=2,
    )
    report = analyze_topology([source, target, stub], [_junction(100, (1, 3))])

    assert len(report.degenerate_connectors) == 1
    assert not report.degenerate_connectors[0].junction_required


def test_long_connector_is_not_flagged_as_degenerate():
    source = _road(1, x=0.0, y=0.0, hdg=0.0, length=30.0)
    target = _road(2, x=60.0, y=0.0, hdg=0.0, length=30.0)
    connector = _connector(
        3,
        x=30.0,
        y=0.0,
        hdg=0.0,
        length=25.0,
        junction=100,
        predecessor=1,
        successor=2,
    )
    report = analyze_topology([source, target, connector], [_junction(100, (1, 3))])
    assert report.degenerate_connectors == []


def test_report_logs_without_error(caplog):
    road = _road(1, x=0.0, y=0.0, hdg=0.0, length=30.0)
    overlay = _connector(
        2,
        x=-1.0,
        y=0.0,
        hdg=0.0,
        length=0.01,
        junction=100,
        predecessor=5,
        successor=6,
    )
    report = analyze_topology([road, overlay], [_junction(100, (5, 2))])
    with caplog.at_level("INFO"):
        report.log()
    assert "Vissim topology" in caplog.text


# ---------------------------------------------------------------------------
# dissolve_non_intersection_junctions
# ---------------------------------------------------------------------------


def _diverge_network():
    """One approach splitting into a through road and a side road.

    Road 1 continues into road 3 via connector 5 (3 lane links) and turns off
    into road 4 via connector 6 (1 lane link) — a diverge, not an
    intersection: only one approach and no crossing movement.
    """
    approach = _road(1, x=0.0, y=0.0, hdg=0.0, length=40.0, lane_widths=(3.5, 3.5, 3.5))
    through = _road(3, x=42.0, y=0.0, hdg=0.0, length=40.0, lane_widths=(3.5, 3.5, 3.5))
    side = _road(4, x=42.0, y=-20.0, hdg=-0.6, length=40.0)
    straight = _connector(
        5,
        x=40.0,
        y=0.0,
        hdg=0.0,
        length=2.0,
        junction=900,
        predecessor=1,
        successor=3,
        lane_widths=(3.5, 3.5, 3.5),
    )
    turn = _connector(
        6,
        x=40.0,
        y=0.0,
        hdg=-0.6,
        length=2.0,
        junction=900,
        predecessor=1,
        successor=4,
    )
    junction = Junction(
        id=900,
        name="diverge_900",
        connections=[
            Connection(
                id=0,
                incoming_road=1,
                connecting_road=5,
                contact_point=ContactPoint.START,
                lane_links=[LaneLink(1, 1), LaneLink(2, 2), LaneLink(3, 3)],
            ),
            Connection(
                id=1,
                incoming_road=1,
                connecting_road=6,
                contact_point=ContactPoint.START,
                lane_links=[LaneLink(1, 1)],
            ),
        ],
    )
    approach.link = RoadLink(
        successor=Successor(ElementType.JUNCTION, 900, None),
    )
    through.link = RoadLink(
        predecessor=Predecessor(ElementType.JUNCTION, 900, None),
    )
    side.link = RoadLink(
        predecessor=Predecessor(ElementType.JUNCTION, 900, None),
    )
    return [approach, through, side, straight, turn], [junction]


def test_diverge_junction_is_dissolved():
    roads, junctions = _diverge_network()
    report = dissolve_non_intersection_junctions(roads, junctions)

    assert junctions == []
    assert [d.junction_id for d in report.dissolved_junctions] == [900]
    assert report.dissolved_junctions[0].connecting_roads == [5, 6]
    assert report.kept_junctions == []

    by_id = {road.id: road for road in roads}
    # Both former connecting roads are ordinary roads now.
    assert by_id[5].junction == -1
    assert by_id[6].junction == -1
    # The neighbours name the branch carrying the most lane links (road 5).
    assert by_id[1].link.successor.element_type == ElementType.ROAD
    assert by_id[1].link.successor.element_id == 5
    assert by_id[1].link.successor.contact_point == ContactPoint.START
    assert by_id[3].link.predecessor.element_id == 5
    assert by_id[3].link.predecessor.contact_point == ContactPoint.END
    # The side road is fed by the only connector that reaches it.
    assert by_id[4].link.predecessor.element_id == 6
    # The secondary branch keeps its own links, so the movement survives.
    assert by_id[6].link.predecessor.element_id == 1
    assert by_id[6].link.successor.element_id == 4


def test_real_intersection_is_kept():
    """Four approaches: an intersection even without detected crossings."""
    roads, junctions = _diverge_network()
    junction = junctions[0]
    junction.connections.append(
        Connection(
            id=2,
            incoming_road=7,
            connecting_road=5,
            contact_point=ContactPoint.START,
            lane_links=[LaneLink(1, 1)],
        )
    )
    junction.connections.append(
        Connection(
            id=3,
            incoming_road=8,
            connecting_road=6,
            contact_point=ContactPoint.START,
            lane_links=[LaneLink(1, 1)],
        )
    )
    report = dissolve_non_intersection_junctions(roads, junctions)

    assert [j.id for j in junctions] == [900]
    assert report.dissolved_junctions == []
    assert report.kept_junctions[0][0] == 900
    assert report.kept_junctions[0][1] == 3  # distinct approaches
    assert roads[3].junction == 900


def test_crossing_movements_keep_a_two_arm_junction():
    """Two approaches whose paths cross is still an intersection."""
    west = _road(1, x=0.0, y=0.0, hdg=0.0, length=20.0)
    south = _road(2, x=25.0, y=-25.0, hdg=math.pi / 2, length=20.0)
    east = _road(3, x=50.0, y=0.0, hdg=0.0, length=20.0)
    north = _road(4, x=25.0, y=25.0, hdg=math.pi / 2, length=20.0)
    # Straight west->east and straight south->north cross in the middle.
    across = _connector(
        5,
        x=20.0,
        y=0.0,
        hdg=0.0,
        length=30.0,
        junction=900,
        predecessor=1,
        successor=3,
    )
    up = _connector(
        6,
        x=25.0,
        y=-5.0,
        hdg=math.pi / 2,
        length=30.0,
        junction=900,
        predecessor=2,
        successor=4,
    )
    junction = Junction(
        id=900,
        name="cross_900",
        connections=[
            Connection(
                id=0,
                incoming_road=1,
                connecting_road=5,
                contact_point=ContactPoint.START,
                lane_links=[LaneLink(1, 1)],
            ),
            Connection(
                id=1,
                incoming_road=2,
                connecting_road=6,
                contact_point=ContactPoint.START,
                lane_links=[LaneLink(1, 1)],
            ),
        ],
    )
    roads = [west, south, east, north, across, up]
    junctions = [junction]

    report = dissolve_non_intersection_junctions(roads, junctions)

    assert [j.id for j in junctions] == [900]
    assert report.dissolved_junctions == []
    assert report.kept_junctions[0][2] >= 1  # crossing pairs detected


def test_dissolve_leaves_road_and_lane_ids_untouched():
    """The mapping sidecar keys on road/lane ids, so they must not change."""
    roads, junctions = _diverge_network()
    before = [
        (
            road.id,
            [lane.lane_id for lane in road.lanes.lane_sections[0].get_all_lanes()],
        )
        for road in roads
    ]
    dissolve_non_intersection_junctions(roads, junctions)
    after = [
        (
            road.id,
            [lane.lane_id for lane in road.lanes.lane_sections[0].get_all_lanes()],
        )
        for road in roads
    ]
    assert before == after


def test_dissolve_rewires_lane_links_against_the_primary_branch():
    """Lane ids were relative to the junction; they must be re-expressed.

    Leaving the junction-era ids in place is what makes a consumer lose the
    lane correspondence and connect every lane to every lane.
    """
    roads, junctions = _diverge_network()
    by_id = {road.id: road for road in roads}
    # As emitted with the junction in place: the approach's lane links carry
    # bare ids that only resolve through <connection><laneLink>.
    for lane_id, target in ((1, 1), (2, 2), (3, 3)):
        by_id[1].lanes.lane_sections[0].left_lanes[lane_id].successor = LaneElementLink(
            id=target
        )
    for lane_id in (1, 2, 3):
        by_id[5].lanes.lane_sections[0].left_lanes[
            lane_id
        ].predecessor = LaneElementLink(id=lane_id)
        by_id[5].lanes.lane_sections[0].left_lanes[lane_id].successor = LaneElementLink(
            id=lane_id
        )
        by_id[3].lanes.lane_sections[0].left_lanes[
            lane_id
        ].predecessor = LaneElementLink(id=lane_id)
    by_id[6].lanes.lane_sections[0].left_lanes[1].successor = LaneElementLink(id=1)
    by_id[4].lanes.lane_sections[0].left_lanes[1].predecessor = LaneElementLink(id=1)

    dissolve_non_intersection_junctions(roads, junctions)

    approach_lanes = by_id[1].lanes.lane_sections[0].left_lanes
    # Every approach lane now names the primary connector's matching lane.
    assert [approach_lanes[i].successor.id for i in (1, 2, 3)] == [1, 2, 3]
    # The through road resolves back through the primary connector.
    through_lanes = by_id[3].lanes.lane_sections[0].left_lanes
    assert [through_lanes[i].predecessor.id for i in (1, 2, 3)] == [1, 2, 3]
    # The side road is fed by the turn connector, which keeps its own links.
    assert by_id[4].lanes.lane_sections[0].left_lanes[1].predecessor.id == 1


def test_lane_without_a_correspondence_is_cleared_not_left_dangling():
    """An added lane fed only by a secondary branch must have no link.

    A stale id would resolve against the primary connector and duplicate a
    correspondence; clearing it leaves the lane as what it is — a lane that
    starts here and is reached by changing lanes.
    """
    roads, junctions = _diverge_network()
    by_id = {road.id: road for road in roads}
    # Road 3 gains a fourth lane fed by the secondary branch only.
    extra = Lane(lane_id=4, lane_type=LaneType.DRIVING)
    extra.widths = [LaneWidth(s_offset=0.0, a=3.5)]
    extra.predecessor = LaneElementLink(id=1)
    by_id[3].lanes.lane_sections[0].left_lanes[4] = extra

    dissolve_non_intersection_junctions(roads, junctions)

    assert by_id[3].lanes.lane_sections[0].left_lanes[4].predecessor is None


# ---------------------------------------------------------------------------
# align_connector_elevations
# ---------------------------------------------------------------------------


def _with_elevation(road: Road, *records) -> Road:
    road.elevation_profile = ElevationProfile(
        elevations=[Elevation(s=s, a=a, b=b) for s, a, b in records]
    )
    return road


def test_connector_is_lifted_onto_the_neighbouring_surface_level():
    """A cambered road leaves the connector below its lane; close the step.

    The road's profile describes its reference line while the connector's
    follows its own lane centre, so on a camber the two disagree at what is
    physically one joint.
    """
    upstream = _with_elevation(
        _road(1, x=0.0, y=0.0, hdg=0.0, length=30.0), (0.0, 5.0, 0.0)
    )
    downstream = _with_elevation(
        _road(3, x=50.0, y=0.0, hdg=0.0, length=30.0), (0.0, 6.0, 0.0)
    )
    connector = _with_elevation(
        _connector(
            2,
            x=30.0,
            y=0.0,
            hdg=0.0,
            length=20.0,
            junction=100,
            predecessor=1,
            successor=3,
        ),
        (0.0, 4.8, 0.0),
        (10.0, 4.8, 0.0),
    )
    roads = [upstream, downstream, connector]

    adjusted = align_connector_elevations(roads)

    assert [road_id for road_id, _, _ in adjusted] == [2]
    records = connector.elevation_profile.elevations
    # Start now matches road 1's end (5.0) and end matches road 3's start (6.0).
    assert records[0].a == pytest.approx(5.0)
    ramp = records[-1]
    z_end = ramp.a + ramp.b * (20.0 - ramp.s)
    assert z_end == pytest.approx(6.0)
    # A pure ramp: under 1 % extra gradient for a 1.2 m correction over 20 m.
    assert abs(records[0].b) < 0.07


def test_through_roads_are_not_touched_by_the_elevation_alignment():
    plain = _with_elevation(
        _road(1, x=0.0, y=0.0, hdg=0.0, length=30.0), (0.0, 5.0, 0.01)
    )
    other = _with_elevation(
        _road(2, x=31.0, y=0.0, hdg=0.0, length=30.0), (0.0, 9.0, 0.0)
    )
    plain.link = RoadLink(successor=Successor(ElementType.ROAD, 2, ContactPoint.START))
    assert align_connector_elevations([plain, other]) == []
    assert plain.elevation_profile.elevations[0].a == 5.0


# ---------------------------------------------------------------------------
# absorb_degenerate_stubs
# ---------------------------------------------------------------------------


def _stub_network():
    """Two upstream roads merging into one via 1 cm stubs (Odaiba 11000)."""
    left = _road(26, x=0.0, y=0.0, hdg=0.0, length=30.0)
    right = _road(25, x=0.0, y=10.0, hdg=0.0, length=30.0, lane_widths=(3.5, 3.5))
    target = _road(
        23, x=30.01, y=5.0, hdg=0.0, length=30.0, lane_widths=(3.5, 3.5, 3.5)
    )
    stubs = []
    for stub_id, up, pairs in (
        (34, 26, [(1, 1)]),
        (35, 25, [(1, 2)]),
        (36, 25, [(2, 3)]),
    ):
        stub = _connector(
            stub_id,
            x=30.0,
            y=5.0,
            hdg=0.0,
            length=0.01,
            junction=-1,
            predecessor=up,
            successor=23,
        )
        lane = stub.lanes.lane_sections[0].left_lanes[1]
        lane.predecessor = LaneElementLink(id=pairs[0][0])
        lane.successor = LaneElementLink(id=pairs[0][1])
        stubs.append(stub)
    left.link = RoadLink(successor=Successor(ElementType.ROAD, 34, ContactPoint.START))
    right.link = RoadLink(successor=Successor(ElementType.ROAD, 35, ContactPoint.START))
    target.link = RoadLink(
        predecessor=Predecessor(ElementType.ROAD, 34, ContactPoint.END)
    )
    return [left, right, target] + stubs


def test_degenerate_stubs_become_direct_links():
    roads = _stub_network()
    absorbed = absorb_degenerate_stubs(roads, [])

    assert [stub for stub, _, _, _ in absorbed] == [34, 35, 36]
    # All three 1 cm roads are gone.
    assert sorted(road.id for road in roads) == [23, 25, 26]

    by_id = {road.id: road for road in roads}
    # Road 26 -> road 23 is reciprocal: both ends were free.
    assert by_id[26].link.successor.element_id == 23
    assert by_id[23].link.predecessor.element_id == 26
    assert by_id[26].lanes.lane_sections[0].left_lanes[1].successor.id == 1
    # Road 25 -> road 23 is expressed from road 25 only: road 23's
    # predecessor was already taken, which OpenDRIVE cannot express twice.
    assert by_id[25].link.successor.element_id == 23
    right_lanes = by_id[25].lanes.lane_sections[0].left_lanes
    assert right_lanes[1].successor.id == 2
    assert right_lanes[2].successor.id == 3


def test_stub_inside_a_kept_junction_is_left_alone():
    """A real intersection keeps its connectors, degenerate or not."""
    roads = _stub_network()
    junction = _junction(1000, (26, 34))
    absorbed = absorb_degenerate_stubs(roads, [junction])
    assert 34 not in [stub for stub, _, _, _ in absorbed]
    assert 34 in [road.id for road in roads]


def test_long_connector_is_not_absorbed():
    roads = _stub_network()
    by_id = {road.id: road for road in roads}
    by_id[34].length = 25.0
    absorbed = absorb_degenerate_stubs(roads, [])
    assert 34 not in [stub for stub, _, _, _ in absorbed]


# ---------------------------------------------------------------------------
# untag_straight_turn_lanelets
# ---------------------------------------------------------------------------


class _FakeAttributes(dict):
    """Stands in for lanelet2's AttributeMap, which has no ``get``."""

    def get(self, *args, **kwargs):  # pragma: no cover - must not be used
        raise AttributeError("AttributeMap has no 'get'")


class _FakePoint:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class _FakeLanelet:
    def __init__(self, lanelet_id, direction, points):
        self.id = lanelet_id
        self.attributes = _FakeAttributes()
        if direction is not None:
            self.attributes["turn_direction"] = direction
        self.centerline = [_FakePoint(x, y) for x, y in points]


class _FakeMap:
    def __init__(self, lanelets):
        self.laneletLayer = lanelets


def _straight(n=5, length=30.0):
    return [(length * i / (n - 1), 0.0) for i in range(n)]


def _quarter_turn(n=9, radius=20.0):
    return [
        (
            radius * math.sin(math.pi / 2 * i / (n - 1)),
            radius * (1 - math.cos(math.pi / 2 * i / (n - 1))),
        )
        for i in range(n)
    ]


def test_straight_left_right_lanelets_lose_the_tag():
    """An opening turn pocket is tagged but does not turn."""
    from autoware_lanelet2_to_opendrive.vissim_topology import (
        untag_straight_turn_lanelets,
    )

    pocket = _FakeLanelet(1, "right", _straight())
    lanelet_map = _FakeMap([pocket])

    untagged = untag_straight_turn_lanelets(lanelet_map)

    assert [lid for lid, _, _ in untagged] == [1]
    assert untagged[0][1] == "right"
    assert abs(untagged[0][2]) < 1.0
    assert "turn_direction" not in pocket.attributes


def test_a_real_turn_keeps_its_tag():
    from autoware_lanelet2_to_opendrive.vissim_topology import (
        untag_straight_turn_lanelets,
    )

    turn = _FakeLanelet(2, "left", _quarter_turn())
    lanelet_map = _FakeMap([turn])

    assert untag_straight_turn_lanelets(lanelet_map) == []
    assert "turn_direction" in turn.attributes


def test_straight_tag_is_never_removed():
    """Inside an intersection, turn_direction=straight is correct."""
    from autoware_lanelet2_to_opendrive.vissim_topology import (
        untag_straight_turn_lanelets,
    )

    through = _FakeLanelet(3, "straight", _straight())
    lanelet_map = _FakeMap([through])

    assert untag_straight_turn_lanelets(lanelet_map) == []
    assert str(through.attributes["turn_direction"]) == "straight"


def test_untag_tolerance_is_configurable():
    from autoware_lanelet2_to_opendrive.vissim_topology import (
        untag_straight_turn_lanelets,
    )

    turn = _FakeLanelet(4, "left", _quarter_turn())
    assert untag_straight_turn_lanelets(_FakeMap([turn]), tolerance_deg=120.0)
    assert "turn_direction" not in turn.attributes


# ---------------------------------------------------------------------------
# merge_parallel_lane_roads
# ---------------------------------------------------------------------------


def _split_carriageway():
    """One carriageway emitted as a 2-lane and a 1-lane road (Odaiba 23/24).

    Road 24 is the inner one: road 23's reference line sits 3.26 m to its
    left, exactly the width of road 24's single lane, so the carriageways
    touch. Both share predecessor junction 1000 and successor road 26.
    """
    inner = _road(24, x=0.0, y=0.0, hdg=0.0, length=21.0, lane_widths=(3.26,))
    outer = _road(23, x=0.0, y=3.26, hdg=0.0, length=20.9, lane_widths=(3.30, 3.31))
    downstream = _road(
        26, x=25.0, y=0.0, hdg=0.0, length=30.0, lane_widths=(3.3, 3.3, 3.3)
    )
    for road in (inner, outer):
        road.link = RoadLink(
            predecessor=Predecessor(ElementType.JUNCTION, 1000, None),
            successor=Successor(ElementType.ROAD, 26, ContactPoint.START),
        )
    downstream.link = RoadLink(
        predecessor=Predecessor(ElementType.ROAD, 24, ContactPoint.END)
    )
    inner.lanes.lane_sections[0].left_lanes[1].successor = LaneElementLink(id=1)
    outer.lanes.lane_sections[0].left_lanes[1].successor = LaneElementLink(id=2)
    outer.lanes.lane_sections[0].left_lanes[2].successor = LaneElementLink(id=3)
    return [inner, outer, downstream]


def test_per_lane_roads_merge_into_one_carriageway():
    from autoware_lanelet2_to_opendrive.vissim_topology import (
        merge_parallel_lane_roads,
    )

    roads = _split_carriageway()
    mapping = {1491: (24, 1), 1490: (23, 1), 1489: (23, 2)}

    groups = merge_parallel_lane_roads(roads, [], lanelet_to_road_and_lane=mapping)

    assert len(groups) == 1
    assert groups[0].base_road_id == 24
    assert groups[0].absorbed_road_ids == [23]
    assert groups[0].lane_count == 3
    # Road 23 is gone; road 24 carries all three lanes, ordered outward.
    assert sorted(road.id for road in roads) == [24, 26]
    merged = next(road for road in roads if road.id == 24)
    lanes = merged.lanes.lane_sections[0].left_lanes
    assert sorted(lanes) == [1, 2, 3]
    assert [round(lanes[i].widths[0].a, 2) for i in (1, 2, 3)] == [3.26, 3.30, 3.31]
    # The absorbed lanes keep the correspondence they already had.
    assert [lanes[i].successor.id for i in (1, 2, 3)] == [1, 2, 3]
    # The mapping now points at the merged road's lanes.
    assert mapping == {1491: (24, 1), 1490: (24, 2), 1489: (24, 3)}


def test_roads_with_different_links_are_not_merged():
    """Differing link ends is what makes a merge unsafe, so it is refused."""
    from autoware_lanelet2_to_opendrive.vissim_topology import (
        merge_parallel_lane_roads,
    )

    roads = _split_carriageway()
    by_id = {road.id: road for road in roads}
    by_id[23].link.successor = Successor(ElementType.ROAD, 99, ContactPoint.START)

    assert merge_parallel_lane_roads(roads, []) == []
    assert sorted(road.id for road in roads) == [23, 24, 26]


def test_laterally_separated_roads_are_not_merged():
    from autoware_lanelet2_to_opendrive.vissim_topology import (
        merge_parallel_lane_roads,
    )

    roads = _split_carriageway()
    by_id = {road.id: road for road in roads}
    # Push the outer road 12 m away: no longer one carriageway.
    by_id[23].plan_view.geometries[0].y = 12.0

    assert merge_parallel_lane_roads(roads, []) == []
    assert sorted(road.id for road in roads) == [23, 24, 26]


def test_junction_connections_are_retargeted_with_the_lane_shift():
    from autoware_lanelet2_to_opendrive.vissim_topology import (
        merge_parallel_lane_roads,
    )

    roads = _split_carriageway()
    # A connector feeding the outer road's lane 1 must end up on lane 2.
    feeder = _connector(
        63,
        x=-5.0,
        y=3.26,
        hdg=0.0,
        length=5.0,
        junction=1000,
        predecessor=99,
        successor=23,
    )
    feeder.lanes.lane_sections[0].left_lanes[1].successor = LaneElementLink(id=1)
    roads.append(feeder)

    merge_parallel_lane_roads(roads, [])

    assert feeder.link.successor.element_id == 24
    assert feeder.lanes.lane_sections[0].left_lanes[1].successor.id == 2


# ---------------------------------------------------------------------------
# reciprocate_lane_links
# ---------------------------------------------------------------------------


def test_reciprocal_road_link_gets_the_lane_link_on_both_sides():
    from autoware_lanelet2_to_opendrive.vissim_topology import (
        reciprocate_lane_links,
    )

    upstream = _road(1, x=0.0, y=0.0, hdg=0.0, length=30.0, lane_widths=(3.5, 3.5))
    downstream = _road(2, x=30.0, y=0.0, hdg=0.0, length=30.0, lane_widths=(3.5, 3.5))
    upstream.link = RoadLink(
        successor=Successor(ElementType.ROAD, 2, ContactPoint.START)
    )
    downstream.link = RoadLink(
        predecessor=Predecessor(ElementType.ROAD, 1, ContactPoint.END)
    )
    upstream.lanes.lane_sections[0].left_lanes[1].successor = LaneElementLink(id=1)
    upstream.lanes.lane_sections[0].left_lanes[2].successor = LaneElementLink(id=2)

    assert reciprocate_lane_links([upstream, downstream]) == 2
    far = downstream.lanes.lane_sections[0].left_lanes
    assert [far[i].predecessor.id for i in (1, 2)] == [1, 2]


def test_reciprocation_never_overwrites_an_existing_link():
    from autoware_lanelet2_to_opendrive.vissim_topology import (
        reciprocate_lane_links,
    )

    upstream = _road(1, x=0.0, y=0.0, hdg=0.0, length=30.0)
    downstream = _road(2, x=30.0, y=0.0, hdg=0.0, length=30.0)
    upstream.link = RoadLink(
        successor=Successor(ElementType.ROAD, 2, ContactPoint.START)
    )
    downstream.link = RoadLink(
        predecessor=Predecessor(ElementType.ROAD, 1, ContactPoint.END)
    )
    upstream.lanes.lane_sections[0].left_lanes[1].successor = LaneElementLink(id=1)
    downstream.lanes.lane_sections[0].left_lanes[1].predecessor = LaneElementLink(id=7)

    assert reciprocate_lane_links([upstream, downstream]) == 0
    assert downstream.lanes.lane_sections[0].left_lanes[1].predecessor.id == 7


def test_one_sided_road_link_is_left_alone():
    """Without a reciprocal road link the lane id would be unresolvable."""
    from autoware_lanelet2_to_opendrive.vissim_topology import (
        reciprocate_lane_links,
    )

    upstream = _road(1, x=0.0, y=0.0, hdg=0.0, length=30.0)
    downstream = _road(2, x=30.0, y=0.0, hdg=0.0, length=30.0)
    upstream.link = RoadLink(
        successor=Successor(ElementType.ROAD, 2, ContactPoint.START)
    )
    downstream.link = RoadLink(
        predecessor=Predecessor(ElementType.ROAD, 9, ContactPoint.END)
    )
    upstream.lanes.lane_sections[0].left_lanes[1].successor = LaneElementLink(id=1)

    assert reciprocate_lane_links([upstream, downstream]) == 0
    assert downstream.lanes.lane_sections[0].left_lanes[1].predecessor is None
