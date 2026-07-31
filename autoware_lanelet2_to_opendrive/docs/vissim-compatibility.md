# PTV Vissim Import Compatibility

This page reviews the converter's `.xodr` output against the PTV Vissim
openDRIVE import specification (PTV Vissim manual, section 2.6.9) and
documents the `target=vissim` export profile that addresses the gaps.

The review is based on:

- the converter source on `feature/vissim-support`
- two representative outputs: an Odaiba use-case clip
  (60 roads / 7 junctions, high-fidelity emission enabled) and the
  Nishishinjuku regression map (`test/data/nishishinjuku_carla.xodr`,
  693 roads / 57 junctions)
- static analysis of every `<geometry>`, `<lane>`, `<width>`, `<link>`,
  and `<junction>` record in both files

## Executive summary — top risks

1. **`paramPoly3` p-range convention (critical, needs one Vissim
   import test).** 100 % of curved reference-line geometry is emitted as
   `<paramPoly3 pRange="arcLength">`. The `pRange` attribute does not
   exist in OpenDRIVE 1.4 (the declared version); a consumer that assumes
   the normalized `p ∈ [0, 1]` convention will collapse every curve to
   its first metre. The `target=vissim` profile can emit either
   convention (`vissim.param_poly3_p_range`); one small-map import test
   in Vissim decides the correct setting.
2. **`geoReference` does not describe the exported coordinate frame
   (severe, fixed by the profile).** The default output writes
   `+proj=utm +zone=NN +lat_0=… +lon_0=…` while the coordinates are
   *local* offsets from the map origin, not absolute UTM. Vissim uses the
   proj-string to place the network on the background map, so the network
   would land hundreds of kilometres away. The Vissim profile replaces it
   with an exact local-frame transverse-Mercator string. (Upstream issue
   #550 tracks the general fix.)
3. **Non-1.4 attributes trip schema validation (severe, fixed by the
   profile).** `road@rule`, `lane@rule` (CARLA LHT extensions) and
   `access@rule` (OpenDRIVE 1.6 syntax) are flagged by any strict 1.4
   schema check. Vissim validates on import and may abort. The profile
   strips them.
4. **LHT direction semantics (severe, needs one Vissim import test).**
   For Japanese maps all driving lanes are emitted on the *left* side
   (positive IDs) and direction is conveyed by the non-standard
   `rule="LHT"` attribute, which Vissim does not read. Vissim creates a
   link in the *opposite* direction for positive lane indices; whether
   the import-time left-hand-traffic setting compensates must be
   verified with a small map.
5. **Degenerate stub roads (moderate).** Divergence-synthesis stubs as
   short as 0.01 m exist (6–59 roads under 0.5 m per map). Vissim
   requires spline points at ≥ 0.5 m spacing and inserts 1.1 m links
   around width changes, so sub-0.5 m roads may degenerate or produce
   import errors. The profile reports them; they are not yet rewritten.

## Vissim requirement matrix

Legend: ✅ satisfied ｜ ⚠ partial / needs verification ｜ ❌ not satisfied
(before the `target=vissim` profile).

| Vissim requirement | Status | Evidence | Action |
|---|---|---|---|
| `revMajor`/`revMinor` < 1.5 | ✅ | `Header` pins `1`/`4` (`opendrive/header.py`) | none |
| No 1.5+ elements in a 1.4 file | ❌→✅ | `road@rule`, `lane@rule`, `access@rule`, `paramPoly3@pRange` found in both samples | profile strips/normalizes |
| Reference line limited to line / spiral / arc / cubic polynomial | ⚠ | Only `<line>` and `<paramPoly3>` are emitted (arc/spiral classes exist but are opt-in #466). Whether Vissim's "cubic polynomial" includes *parametric* cubics must be verified | verify; fallback documented below |
| Geometry continuity (links drawn from reference line) | ✅ | C0 gap ≤ 0.0001 m, heading jumps ≤ 1.5° across all 9 386 consecutive geometry pairs in both samples | none |
| `planView` lengths consistent | ✅ | Σ geometry length = `road@length` (< 1 mm) for all 753 roads | none |
| Elevation profile → Z | ⚠→✅ | one `<elevation>` per geometry segment, cubic in s, internally continuous to 0.000 m and never steeper than 15 %. But the values are **absolute** elevation (Odaiba: road surfaces 4.36–7.19 m, map points up to 12.71 m above sea level) because `map.offset.z` is 0, so the whole network floated above Vissim's background plane — by a varying amount where the terrain rises | `vissim.elevation_baseline` (default `min`) |
| Lane sections → links | ✅ | exactly one `<laneSection>` per road; no mid-road width discontinuities | none |
| Lane types importable (driving/biking/…) | ⚠ | `driving`, `biking` import; `shoulder`, `sidewalk` are ignored by Vissim (acceptable — pedestrian networks are separate in Vissim) | none |
| Positive/negative lane index semantics | ⚠ | all driving lanes are positive (left side) with `rule="LHT"`; Vissim builds positive-index lanes as opposite-direction links | verify with LHT import setting |
| Connectors from `link::predecessor/successor` | ✅ | connecting roads always carry explicit road→road links with `contactPoint`; incoming/outgoing roads reference the junction | none |
| `junction` / `connection` / `laneLink` well-formed | ✅ | 388 connections, 0 missing `laneLink`, 0 dangling road refs, `connectingRoad@junction` consistent | none |
| Width as constant (Vissim converts) | ❌→✅ | 98 % of `<width>` records were polynomial (mean 40, max 146 records per lane). Vissim inserts a connector + 2 × 1.1 m links at every ≥ 0.25 m variation, shattering each road into dozens of fragments — confirmed on a real import as tangled connector webs and a "Nodes … overlap on link 'Road_9-0-Left **Start**' / link segments invalid" error (the *Start* link is Vissim's inserted 1.1 m fragment). The profile now collapses each lane to a single constant width (arc-length-weighted mean) | `vissim.constant_lane_widths` (default on) |
| Spline points ≥ 0.5 m apart | ⚠ | 53–309 geometry segments < 0.5 m; roads as short as 0.01 m | profile warns; stub consolidation is future work |
| `geoReference` proj-string placement | ❌→✅ | see risk 2 | profile emits local-frame tmerc |
| No reliance on signals / markings / speed limits / lane closure | ✅ | signals, road marks, speeds are informational in the output; conversion topology never depends on them | none |
| Junction predecessor/successor data not processed by Vissim | ✅ | movements are also encoded as connecting-road links, which Vissim does read | none |
| Conflict areas auto-generated by Vissim | ⚠ | Vissim generates them at every link/connector overlap and imports no right-of-way, so priorities come up "not clearly determined" (22 on the Odaiba clip) and need review in Vissim. Count is inflated by per-lane connecting roads overlapping each other | pipeline-level multi-lane road emission (see below) |

## Findings by severity

### Critical (import fails or geometry silently wrong)

- **`paramPoly3` convention** (risk 1). The coefficients are
  arc-length-parameterized. Under a normalized-`p` interpretation the
  planView collapses; under an arc-length interpretation it is exact.
  `vissim.param_poly3_p_range: normalized` re-parameterizes each segment
  exactly (`b·L`, `c·L²`, `d·L³`) and removes the non-1.4 `pRange`
  attribute; `arcLength` keeps coefficients and the attribute.
  **Verification:** import `examples/` mini map twice, once per setting;
  the wrong one shows visibly collapsed/short curved links.
- **`geoReference` frame mismatch** (risk 2). Coordinates are offsets
  from the MGRS-grid + offset origin, axis-aligned with UTM. The profile
  computes the absolute UTM easting/northing (E₀, N₀) of the local origin
  and emits
  `+proj=tmerc +lat_0=0 +lon_0=<central meridian> +k=0.9996 +x_0=<500000−E₀> +y_0=<f_n−N₀> …`,
  which maps local (0, 0) to the true origin.

### Severe (topology or direction wrong)

- **Schema-invalid attributes** (risk 3): stripped by the profile.
- **LHT lane-side semantics** (risk 4): cannot be fixed file-side without
  restructuring lanes to the right side and reversing reference lines —
  that would break CARLA output and the Foretify preflight assumptions
  (`map_quality_preflight.py` asserts left-side travel along +s).
  Kept as an import-time verification item.

### Moderate

- **Sub-0.5 m roads / geometry segments** (risk 5): reported by the
  profile with road IDs. Divergence stubs are required by the CARLA
  loader fix (#291 series), so they are kept; a Vissim-specific stub
  consolidation pass is future work.
- **Co-located junctions → Vissim node overlap** (confirmed on a real
  Vissim import). Vissim creates one node per `<junction>`. The
  divergence synthesis can chain two junctions whose connecting roads
  terminate at the *same* road endpoint — on the Odaiba clip, junction
  1001 (connecting road 53) and junction 1002 (connecting road 59) both
  end at the start of road 9, only 3.1 m apart. Both node areas then
  cover that point and Vissim reports `Nodes "1002" and "1001" overlap
  on link "9: Road_9-0-Left" at position 0.4 m … the status of the
  conflict areas can therefore not be determined`. Fixed by
  `vissim.merge_overlapping_junctions` (default on): junctions sharing an
  attachment point are merged into one (connections moved, connection ids
  renumbered, `road@junction` and every road-level junction link
  rewritten), so one node replaces the overlapping pair. Lanelet2
  topology and all movements are preserved — only the junction grouping
  changes.
- **Polynomial width records** (confirmed on a real Vissim import):
  with dense polynomial `<width>` chains, Vissim's width-change
  machinery (connector + 2 × 1.1 m links per ≥ 0.25 m variation)
  fragmented every road, produced tangled webs of generated connectors
  ("Road_44-0-Left Connector - 1", …) and node-overlap errors on the
  inserted "… Start" fragments where two junctions sit close together.
  Fixed by `vissim.constant_lane_widths` (default on): each lane's chain
  is collapsed to its arc-length-weighted mean width. Note that after
  this change width steps between *consecutive roads* can still exceed
  0.25 m at a genuine widening/narrowing — Vissim then inserts one
  connector there, which is the intended behavior.

### Severe — non-intersections arriving as intersections (fixed)

Vissim creates **one node per `<junction>`**, and a node brings the whole
intersection machinery with it: auto-generated conflict areas, priority
rules, reduced-speed areas. The divergence synthesis wraps *every*
lane-level merge and diverge in a junction, so a plain widening, an
off-ramp or a lane drop arrives in Vissim as an intersection. On the
Odaiba clip only **one of seven junctions is a real intersection**:

| junction | approaches | crossing movements | what it really is |
|---|---|---|---|
| 1000 | 4 | 11 pairs | real intersection |
| 1001, 1002, 1003, 1004 | 1 each | none | diverge / widening |
| 11000, 11001 | 2 each | none | merge / diverge (the 0.01 m stubs) |

`vissim.dissolve_non_intersection_junctions` (default on) dissolves the
six non-intersections: their connecting roads become ordinary roads
(`junction="-1"`) and each neighbouring road is repointed at the branch
carrying the most lane links. Secondary branches keep their own
`predecessor`/`successor`, and since Vissim generates connectors from
`link::predecessor`/`link::successor` the movement survives even though
OpenDRIVE lets the neighbour name only one of them.

A junction counts as an intersection when it has ≥ 3 distinct approaches
*or* two of its movements cross geometrically. Road and lane ids are never
changed, so the `*.mapping.json` sidecar stays valid; the dissolved
connecting roads are recorded in it as `dissolved_junction_roads` so the
junction-lanelet validation knows a `turn_direction` lanelet outside a
junction is intended here (the same role `skipped_synthetic_roads` plays).

### Severe — the network floating above the background plane (fixed)

Lanelet2 maps store **absolute** elevation. On the Odaiba clip the source
points run 4.36–12.71 m above sea level (median 5.51 m) and
`map.offset.z` is `0`, so the emitted road surfaces carry 4.36–7.19 m.
Vissim's background plane is at `z = 0`, so the whole network sat 4–7 m
above it — and because the terrain rises across the clip, some stretches
floated noticeably more than others.

The elevation data itself is sound: internal continuity of every
`<elevationProfile>` is exact (largest step between consecutive
`<elevation>` records: 0.000 m over 916 joints), road-to-road boundary
jumps stay under 0.21 m, and no road exceeds a 15 % gradient. The problem
is purely the datum.

`vissim.elevation_baseline` (default `"min"`) subtracts one constant from
every `<elevation>`'s `a` coefficient — and from `<positionInertial>`,
which is absolute — so the lowest road surface lands at `z = 0`
(Odaiba: shift −4.36 m, surfaces now 0.00–2.83 m). Because it is a pure
translation, **every gradient is bit-identical**; `zOffset` and
`<cornerLocal>` are relative to the road surface and are left alone.

Note that the other map configs already set this on the input side —
`nishishinjuku` uses `offset.z: 42.49998`. Setting `map.offset.z` instead
also works, but it moves the Lanelet2 origin for *every* target;
`elevation_baseline` keeps the change inside the Vissim export.

### Must be fixed in the source map — overlapping lanelets

The Odaiba clip has **41 lanelet pairs whose centrelines run inside each
other by 25 % or more, and not one of them shares a boundary linestring**.
The pattern is uniform: a turn pocket starts at the *exact* coordinate of
the through lane it duplicates and separates to about one lane width by
its end.

| pocket | through lane | boundaries (pocket / through) | centreline gap start → end | pocket length |
|---|---|---|---|---|
| 176324 (`right`) | 176323 | 176314 / 176316 vs 176318 / 176315 | 0.00 → 3.18 m | 27.4 m |
| 1494 | 1514 | 555 / 556 vs 553 / 554 | 0.00 → 3.32 m | 54.0 m |
| 176191 (`left`) | 176185 | 176189 / 176190 vs 176184 / 175959 | 0.00 → 3.51 m | 29.9 m |
| 190217 (`left`) | 1520 | 190200 / 190202 vs 572 / 573 | 0.00 → 83.82 m | 9.0 m |

The through lanes themselves are stitched correctly — lanelet 1514 has
bounds 553 / 554 and its neighbour 1493 has 552 / **553**, sharing one
linestring, which is how Lanelet2 expresses adjacency. The pockets share
nothing, so they are independent polygons occupying the through lane's
ground rather than lanes beside it.

**Why the converter cannot repair it.** Within one OpenDRIVE road, lanes
are stacked side by side by width, so a lane cannot lie on top of another;
merging the pair is impossible. Trimming the pocket to the part that is
clear would halve a 54 m storage lane and break the mapping validation,
which compares the emitted geometry against the lanelet centreline.

**What it costs if left alone.** On the Odaiba clip 59.5 m of 3336
lane-metres (1.78 %) is duplicated, always one lane of each road:

| pair | overlap | same origin and destination |
|---|---|---|
| road 26 / 28 | 27.5 m | yes — a duplicated lane |
| road 36 / 37 | 22.8 m | yes |
| road 16 / 17 | 17.0 m | no |
| road 5 / 7 | 13.7 m | yes |
| road 16 / 33, 15 / 74, 74 / 75 | 2.8–4.5 m | no |

Flow-level results (volumes, travel times, queues) survive that: the
stretches are short and the duplicated pairs share an origin and
destination, so total demand is preserved. What does not survive is
anything lane-level — the stretch carries one lane more capacity than the
ground truth, Vissim's auto-generated conflict area there defaults to
passive so vehicles pass through each other, safety measures such as SSAM
see conflicts that do not exist, and a lane-level comparison against an
Autoware trajectory has no single answer for "which road is that lane".

`analyze_topology` reports every pair with its overlap length and whether
the two share an origin and destination, so the list can go straight to
whoever maintains the map. **The fix is to stitch each pocket into its lane
group**: give it the through lane's boundary on the shared side so the
polygons become adjacent, or start it where it has actually separated.

### Known, not fixable at write time

- **Conflict-area priority warnings** (`The priority of conflict area "N"
  could not be clearly determined`). Vissim auto-generates conflict areas
  wherever two links/connectors overlap, and it does **not** import
  right-of-way information — the manual's import section lists
  signalization, road markings, lane change, lane closure and speed limits
  as *not adopted*, and junction predecessor/successor data as *not
  processed*. The `<priority>` records this converter emits (20 on the
  Odaiba clip) are therefore ignored. Vissim picks a status and asks the
  modeller to confirm it, so these warnings are inherent to the
  OpenDRIVE → Vissim path and must be reviewed in Vissim (UI or COM).

  What *is* actionable is their **number**: 22 on the Odaiba clip. The
  converter models a multi-lane approach as one single-lane road (and one
  single-lane connecting road) per lane — roads 17/18/19 are three
  single-lane roads 3.3 m apart, one per lane of the same carriageway —
  and inside a large junction those siblings run parallel for their whole
  length (connectors 40/41/42 are 115–119 m). Vissim creates a conflict
  area for every overlapping pair, and because both members belong to the
  same traffic stream their priority is genuinely undeterminable.
  Connection 10 → 29 shows the shape that avoids this: **one** connecting
  road (49) carrying three lanes with `laneLink` 1→1, 2→2, 3→3.

  Consolidating per-lane roads into multi-lane roads would collapse most
  of these conflict areas, but it **cannot be done in the write-time
  profile**: road and lane ids are referenced by the `*.mapping.json`
  sidecar that `analyze`, the stop-line validation and the CARLA scenario
  tooling consume, so renumbering or merging roads there would silently
  invalidate them. It belongs in the conversion pipeline (multi-lane road
  emission), where the mapping is produced from the same model.

  **Measuring overlap correctly.** Comparing *reference lines* is not
  enough. On a two-way road split into per-direction roads, the two
  reference lines can run within a metre of each other while the
  carriageways sit side by side, because each road's lanes extend to its
  own left. A reference-line metric reports that as a 100 % overlay — the
  Odaiba connectors 40/41/42 look like they cover roads 20 and 22
  completely, but their headings differ by 180° and the driveable surfaces
  do not overlap at all. `vissim_topology.py` therefore samples **lane
  centres** and only counts stations whose tangents agree within 45°.
  Under that measure the only genuine overlay on the clip is connector 53
  running along 48 % of road 12.

  **Why it is reported, not repaired.** Dropping an overlapping connector
  is unsafe at road level: connectors 43, 47 and 50 all run from road 11
  to road 29 and look like mutual duplicates, yet each serves a
  *different destination lane* (road 29 lanes 1, 2 and 3), so removing one
  silently deletes a movement. The diagnostics list the constructs; the
  repair belongs in the junction/divergence geometry generation.

### Minor / informational

- `sidewalk`, `shoulder` lanes and all `<object>`, `<signal>`,
  `<controller>` records are ignored by Vissim. They are retained (they
  do not affect import).
- `<lane><speed>` and `<roadMark>` are ignored by Vissim (speed limits
  and markings are not imported).
- Lane widths < 1 m are clamped to 1 m by Vissim (2–4 lanes per sample
  map, all `shoulder`-adjacent geometry).

## The `target=vissim` profile

```bash
uv run python -m autoware_lanelet2_to_opendrive.main \
    map=<map_name> target=vissim \
    input_map_path=<map.osm> output_map_path=<out.xodr>
```

The profile is a **write-time post-processing pass**
(`vissim_profile.py`) applied to the final XML tree, so the conversion
pipeline — and therefore CARLA/Foretify behavior under other targets —
is untouched. Settings (see `conf/target/vissim.yaml`):

| Key | Default | Effect |
|---|---|---|
| `vissim.enabled` | `true` (in the vissim target) | master switch for the post-pass |
| `vissim.strip_nonstandard_attributes` | `true` | remove `road@rule`, `lane@rule`, `access@rule` |
| `vissim.param_poly3_p_range` | `normalized` | `normalized`: exact re-parameterization to `p ∈ [0,1]`, `pRange` attribute removed (schema-clean). `arcLength`: keep coefficients and attribute |
| `vissim.local_geo_reference` | `true` | replace the header proj-string with the exact local-frame tmerc string |
| `vissim.constant_lane_widths` | `true` | collapse each lane's `<width>` chain to a single constant record (arc-length-weighted mean) — prevents Vissim's per-variation connector/1.1 m-link insertion from fragmenting the network |
| `vissim.elevation_baseline` | `"min"` | shift every elevation so the lowest road surface sits at `z = 0` (`"mean"` centres it, `"none"` keeps absolute, a number shifts by that much). Only a constant offset, so gradients are preserved |
| `vissim.merge_overlapping_junctions` | `true` | merge junctions whose connecting roads terminate at the same road endpoint into one junction — Vissim builds one node per junction, and co-located junctions produce "Nodes … overlap on link …" errors with undetermined conflict areas |
| `vissim.dissolve_non_intersection_junctions` | `true` | turn junctions that carry no crossing movement (pure merges and diverges) into ordinary road links, so Vissim does not place a node — and therefore no intersection — where the road merely widens or forks. Odaiba: 7 junctions → the 1 real intersection |

The pass logs a **Vissim import report**: counts of roads shorter than
1.1 m / 0.5 m, lanes whose width falls below 1.0 m, and lanes whose
width swing exceeds 0.25 m.

Alongside it, `vissim_topology.analyze_topology` runs inside the pipeline
(read-only) and reports the two constructs Vissim degrades on:

- **connectors running along a through road** in the same direction, with
  the covered fraction — Vissim puts an undetermined conflict area over
  that stretch. Odaiba: `road 12: connector 53 (junction 1001) runs along
  48% of it`.
- **connectors below Vissim's 0.5 m minimum spline spacing**, with the
  roads they join and whether the junction is structurally required.
  Odaiba: six 0.01 m stubs (roads 34–39), all marked *junction needed*
  because they express merges/diverges that a single predecessor and
  successor cannot.

## Interaction with CARLA and Foretify targets

| Area | CARLA (`target=carla`) | Foretify preflight | Vissim profile decision |
|---|---|---|---|
| `road@rule` / `lane@rule` LHT extension | **required** for LHT maps | tolerated (`analyze` suppresses the schema false positive) | stripped only in Vissim output; other targets unchanged |
| `paramPoly3 pRange="arcLength"` | required (CARLA parses it) | assumed by `map_quality_preflight` evaluators | re-parameterized only in Vissim output at write time; internal geometry untouched |
| `geoReference` UTM string | read but placement-insensitive | not evaluated | replaced only in Vissim output; upstream #550 will later fix all targets — expect a merge point there |
| Divergence stub roads (0.01 m) | **required** — degenerate-stub fix keeps CARLA loading | checked by preflight | kept as-is; reported in the Vissim import log; consolidation is future work |
| Signals / Stencil_STOP objects | required | evaluated | retained (Vissim ignores them harmlessly) |
| Lane side (left/positive, LHT) | required convention | assumed by preflight (+s travel) | unchanged; verify Vissim's import-time LHT handling |

## Open items / verification checklist

First-import findings (Odaiba clip, 2026-07-31): the network landed on the
background map at the correct position and curves rendered with plausible
shapes — the normalized `paramPoly3` interpretation (item 1), parametric
cubic support (item 3), and the local-frame geoReference (item 4) all look
correct. The polynomial-width fragmentation this import exposed is fixed by
`constant_lane_widths` (see above).

1. ~~normalized vs arcLength import test~~ — normalized appears correct;
   re-confirm after the constant-width re-export.
2. Confirm Vissim's left-side-traffic import option yields links in the
   travel direction for positive-index lanes (LHT Japanese maps).
3. ~~paramPoly3 support~~ — appears supported (curved links imported).
4. ~~Background-map placement~~ — appears correct; re-confirm visually.
5. Re-import after the constant-width and junction-merge fixes: the
   "Nodes … overlap on link …" error and the dense generated-connector
   webs should be gone, and conflict areas should now be determinable.
6. Review roads listed in the import report (< 0.5 m) inside Vissim; if
   they degenerate, plan the stub-consolidation pass.
7. Vissim renders one constant width per link, so lane edges step at road
   boundaries (median 0.10 m, 63 of 83 joints below Vissim's 0.25 m
   threshold and therefore absorbed by its own width harmonization). If a
   smoother appearance is wanted in generic OpenDRIVE viewers, a
   chain-level width harmonization (one width per connected lane chain
   below the 0.25 m threshold) is the next step.

---

[← Back to Limitations Overview](limitations/index.md)
