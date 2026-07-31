"""Tests for the Vissim conformance checker CLI."""

from autoware_lanelet2_to_opendrive.vissim_conformance import _run


def _write(tmp_path, body: str):
    path = tmp_path / "net.xodr"
    path.write_text(f"<OpenDRIVE>{body}</OpenDRIVE>", encoding="utf-8")
    return str(path)


_ROAD = """
  <header revMajor="1" revMinor="4"/>
  <road id="1" length="20.0" junction="-1">
    <link/>
    <elevationProfile><elevation s="0.0" a="0.0" b="0.0" c="0.0" d="0.0"/></elevationProfile>
    <planView><geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="20.0"><line/></geometry></planView>
    <lanes><laneSection s="0.0">
      <left><lane id="1" type="driving" level="false">
        <width sOffset="0.0" a="3.5" b="0.0" c="0.0" d="0.0"/>
      </lane></left>
      <center><lane id="0" type="none" level="false"/></center>
    </laneSection></lanes>
  </road>
"""


def test_a_clean_single_road_passes_every_check(tmp_path, capsys):
    # One isolated road fails only the isolation check, so give it a neighbour.
    body = _ROAD.replace(
        "<link/>",
        '<link><successor elementType="road" elementId="2" contactPoint="start"/></link>',
    )
    body += (
        _ROAD.replace('id="1"', 'id="2"')
        .replace('x="0.0" y="0.0"', 'x="20.0" y="0.0"')
        .replace(
            "<link/>",
            '<link><predecessor elementType="road" elementId="1" contactPoint="end"/></link>',
        )
    )
    _run(_write(tmp_path, body))
    out = capsys.readouterr().out
    assert "[FAIL] header" not in out
    assert "[PASS] geometry length attribute matches its curve" in out
    assert "[PASS] no non-1.4 attributes" in out


def test_a_non_1_4_attribute_is_reported(tmp_path, capsys):
    body = _ROAD.replace('junction="-1"', 'junction="-1" rule="LHT"')
    _run(_write(tmp_path, body))
    assert "[FAIL] no non-1.4 attributes" in capsys.readouterr().out


def test_a_wrong_length_attribute_is_reported(tmp_path, capsys):
    """A curve carries its own length, so the attribute can disagree with it.

    A ``<line>`` cannot: its shape is derived *from* the attribute, so there
    is nothing to compare. The check is about paramPoly3 and arc.
    """
    curve = (
        '<geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="19.0">'
        '<paramPoly3 aU="0.0" bU="20.0" cU="0.0" dU="0.0"'
        ' aV="0.0" bV="0.0" cV="0.0" dV="0.0"/></geometry>'
    )
    body = _ROAD.replace(
        '<geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="20.0"><line/></geometry>',
        curve,
    ).replace('length="20.0" junction', 'length="19.0" junction')
    _run(_write(tmp_path, body))
    out = capsys.readouterr().out
    assert "[FAIL] geometry length attribute matches its curve" in out


def test_a_road_below_the_spline_spacing_is_reported(tmp_path, capsys):
    body = _ROAD.replace('length="20.0"', 'length="0.2"')
    _run(_write(tmp_path, body))
    assert "[FAIL] no importable road below 0.5 m" in capsys.readouterr().out


def test_a_thin_lane_is_reported(tmp_path, capsys):
    body = _ROAD.replace('a="3.5"', 'a="0.6"')
    _run(_write(tmp_path, body))
    assert "[FAIL] no lane below the 1.0 m clamp" in capsys.readouterr().out


def test_a_failing_file_exits_non_zero(tmp_path):
    """The exit code gates a conversion, so a failure must be visible to it."""
    broken = _ROAD.replace('junction="-1"', 'junction="-1" rule="LHT"')
    assert _run(_write(tmp_path, broken)) == 1
