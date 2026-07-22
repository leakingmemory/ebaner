# ebaner

A small Vulkan/C++ viewer for the `terrainmapper` game-export data
(`../norway-rails`). It stitches the elevation tiles around **Bodø station** into
a continuous 3D surface and draws the railway, roads and buildings on top, with a
first physics-driven rail vehicle. The camera starts at the buffer-stop end of the
Bodø main track ("track 1 end"), resolved from the `tracks.bin` geometry.

## What it renders

- **Terrain** — heightmap tiles shaded with hillshade and coloured by **AR50 land
  cover** when the export provides it (forest, agriculture, open land, bog,
  glacier, water, built-up …), falling back to an elevation ramp otherwise. The
  overlapping LOD quadtree is de-overlapped (finest tile per area) and
  watertight-stitched so there are no cracks between resolutions
  (`src/TerrainMesh.cpp`). Where a **surface** railway falls below the terrain, the
  ground is **cut into a trench** to fit the track and ballast — a flat floor at the
  ballast base with side walls sloping up at ~20° until they meet the original
  ground — so the rails sit in a cutting instead of being buried. Tunnels and
  bridges are left alone, and where a surface track enters a tunnel in a cutting the
  trench stops at the mouth, leaving a vertical portal wall of terrain. The carve
  edits the height grid as a world-space function (so LOD seams stay watertight) at
  load time (`src/TerrainCarve.cpp`; set `EBANER_NOCARVE=1` to disable).
- **Railway** — a realistic cross-section: a ballast bed, concrete sleepers and
  two rusty rails (`src/TrackMesh.cpp`). The export splits each line into separate
  segments at medium transitions (surface / tunnel / bridge); `buildTrackPaths`
  (`src/TrackPath.cpp`) **joins segments that meet end-to-end** (through degree-2
  nodes of the same track type) into continuous routes, so the line is unbroken
  through tunnels and the train can run straight through instead of derailing at a
  portal. Centrelines are then smoothed through the coarse surveyed points with a
  **centripetal Catmull-Rom** spline, so curves are continuous. Sleepers are real
  3-D boxes near the camera and a repeating texture at distance (distance LOD).
  Track is **superelevated (banked)** on curves from the OSM speed limit + curvature.
- **Roads** — category-coloured asphalt ribbons (`src/RoadMesh.cpp`): the public
  network (Europavei/Riksvei/Fylkesvei/Kommunal) prominent, private tracks thin
  and faint.
- **Buildings** — OSM footprints extruded into lit prisms coloured by type, with
  **pitched roofs** (flat / gabled / hipped / pyramidal / skillion) from the OSM
  `roof_shape` tag (`src/BuildingMesh.cpp`).
- **Station platforms** — OSM `railway=platform` footprints (area platforms and
  buffered platform edges) extruded into low lit slabs beside the tracks
  (`src/PlatformMesh.cpp`). The slab top is set a standard 0.76 m above the
  nearest **rail head** (not the raw DTM), so long platforms sit flat at rail
  level instead of sinking into rising ground. The surface is styled to the
  Norwegian standard: an asphalt centre with light **concrete edge slabs** and a
  painted **yellow safety line** where the slabs begin, on the track-facing
  long edges.
- **Rail vehicle** — chosen on an in-window **start screen** (a text menu): a
  single-axle wheelset, a dual-axle bogie (two wheelsets + frame), a full-length
  **carriage underframe** on two dual-axle bogies (one at each end), or a longer
  **articulated module** on three bogies whose two underframe sections hinge over
  the shared middle (Jacobs) bogie so it flexes on curves, or a liveried **NSB
  Class 93** (Bombardier Talent) DMU — a two-section body with raked cab noses, a
  modelled saloon (low-floor gangway, raised vestibules, doors, boxed tech/WC
  areas, wall/ceiling lining and oriented seats) and a basic **driver's cab**
  (seat + desk, a combined power/brake lever on the driver's right and a smaller
  **R/N/F reverser** on the left) at each end
  (`src/VehicleMesh.cpp`). A small 1-DOF physics model (`src/Vehicle.cpp`) gives it
  mass, gravity resolved into along-track acceleration + weight-on-rails, Davis
  running resistance, box inertia and a curve overturning limit. It rolls under
  gravity on grades, can be **hand-pushed** (Up/Down), coasts to a stop via rolling
  resistance, and if it runs off the end of its track it derails and is slowed to a
  stop by ground friction proportional to its weight. A single **combined
  power/brake lever** per cab (`,` toward power N → P1–P5, `.` toward brake N → B1–B4
  → Emergency, Space slams emergency) drives both a simple **air-brake** and the
  traction. On the brake side a main reservoir feeds a notched direct brake: each
  notch laps the brake-cylinder pressure to a target from the reservoir, producing a
  braking force capped by wheel–rail adhesion that also holds the vehicle at rest.
  The reservoir holds enough air for many applications; cycling the brakes slowly
  draws it down and, once depleted, the cylinders can no longer fully charge and the
  brakes fade. On the power side the vehicle **drives** through a modelled
  **diesel-hydraulic transmission** (per the real Di 93): a **torque converter** for
  launch (torque multiplication while it slips, so the revs flare then couple) feeding
  a **5-speed automatic gearbox** that shifts on road speed (the engine steps down at
  each upshift); tractive effort is the least of the geared/converter limit, the
  constant-power hyperbola and the adhesion cap, so it launches hard then tapers toward
  a level top speed. The gearbox is modelled but not shown — only speed and rpm are on
  the HUD. Each cab has its **own** power/brake lever and reverser, operating
  independently; the keyboard drives the cab you are seated in (driver view `V`, else
  the front cab). The **reverser** (`F` / `N` / `R`) gates both across both cabs: with
  both handles in Neutral, or both out of Neutral, the brakes go to emergency and no
  power is available; only when **exactly one** cab is out of Neutral does that cab's
  lever take control, with power applied in its direction (reverse is capped to a
  shunting speed). Power and the recharge come from the **diesel engines** (one per cab
  end, 2 × 306 kW): start them with `I` (both crank to idle ~700 rpm together, shown as
  a rev-counter bar per engine on the left cab LCD; they are two Cummins N14E-R, 14 L,
  full power at 1500 rpm), they rev up under power, and their compressors refill the
  reservoir at idle — which also lifts it back above the low-air safety trip. It starts
  held in **emergency with the reservoir full and the engines off**; the cab's speed and
  duplex air gauges and the combined lever animate with the sim, mirrored on a HUD.
  With an audio backend
  (PulseAudio or PortAudio), the brake air is **synthesized** in real time: a hiss
  whose loudness tracks the airflow — a subdued charge on apply and a prominent,
  brighter vent on release — fading as the pressure equalizes and with camera
  distance to the bogies, plus a valve click at each change of the handle or the
  safety. The two diesels get a **muffled idle drone** (a firing thrum at the ~35 Hz
  idle firing rate with a soft combustion knock, heavily low-passed for the
  insulated character, the two ends slightly detuned so they beat), following each
  engine's rpm and faded by distance to its end. While a compressor charges the
  reservoir it adds a higher, muffled **pump hum** and loads its engine down a touch
  (a small, audible idle droop) (`src/Audio.cpp`).

## Requirements

System packages (all found via CMake / pkg-config):

- Vulkan SDK / loader + headers
- GLFW 3
- GLM
- `glslc` (shader compiler, from shaderc)
- PulseAudio (`libpulse-simple`) or PortAudio (**optional** — either enables the
  synthesized brake sound, PulseAudio preferred; the build is silent without both)
- CMake ≥ 3.20, a C++20 compiler

## Build

```sh
cmake -S . -B build
cmake --build build -j
```

This builds two executables that share the same rendering/loading engine
(`libebaner_engine`): the **`ebaner`** viewer above, and **`ebaner-trackedit`**
below.

## Run

```sh
./build/ebaner ../norway-rails
```

The dataset path defaults to `../norway-rails` if omitted. On startup the console
prints the resolved start point (UTM 33N), the look direction, tile/triangle
counts, and vehicle physics (mass, inertia, tipping limit).

## Controls

| Input        | Action                          |
|--------------|---------------------------------|
| W / A / S / D| Move horizontally               |
| Q / E        | Move down / up                  |
| Mouse        | Look                            |
| Left Shift   | Move faster (×8)                |
| C            | Toggle chase camera (the vehicle) |
| V            | Driver's-seat view; press again to switch cab, again to exit |
| I            | Start / stop the diesel engines (both together) |
| Up / Down    | Hand-push the vehicle fwd / back |
| , / .        | Combined lever: step toward power / toward brake (the viewed cab) |
| Space        | Emergency brake                 |
| F / N / R    | Reverser: Forward / Neutral / Reverse (the viewed cab) |
| M            | Mute / unmute sound             |
| Tab          | Release/grab cursor             |
| Esc          | Quit                            |

## Track editor (`ebaner-trackedit`)

```sh
./build/ebaner-trackedit ../norway-rails
```

A **WYSIWYG track-network editor** that reuses the same engine: it renders the
identical scene (terrain, roads, buildings, rails) and, on top, overlays the **raw
rail geo-points** as round markers and the **links between consecutive points** of
each track as lines — the surveyed geometry behind the smoothed rails. Markers/links
are coloured by track type (**amber** main line, **cyan** siding, **magenta** yard),
and **dead ends** (loose ends of a broken link) are marked in **red**. The terrain,
rails and buildings are drawn **ghosted** (half-transparent, editor only) and the
geo-point network is drawn **on top** (x-ray, sized so near points read clearly), so
no point is buried in the terrain or rails. Walk around with the same free-fly
controls (W/A/S/D, Q/E, mouse, Shift, Tab, Esc); a HUD shows the track / geo-point /
dead-end counts and the camera position.

**Editing model.** Edits **preview immediately** in the editor (applied to the
in-memory geometry the same way the viewer applies them at load) and are written to a
**drop-in overlay**, `<dataset>/overlay/track-edits.txt`, only when you press
**Ctrl+S** — a separate directory the generator never touches, so regenerated base
tiles can be dropped in without losing manual edits. On the next load, `TerrainData`
applies the overlay over the generated tiles for **both** the viewer and the editor.
(`EBANER_NOOVERLAY=1` ignores the overlay; unsaved edits are discarded on quit — the
HUD shows the unsaved count.)

**Selecting points.** Press **Tab** to free the cursor, then **left-click** a
geo-point to select it (it turns **white**); **Ctrl+click** toggles points for a
multi-selection; clicking empty space clears it. Picking is screen-space from the
cursor. The **elevation** of the point under the cursor is labelled beside it, and the
HUD shows the selected point's elevation (or the `min..max` range for a group).

**Raising / lowering.** With points selected, **Up** / **Down** nudge their elevation
by 0.1 m (auto-repeating while held, so bigger moves just hold the key). Each is an
`elev` override, previewed live and saved with Ctrl+S — handy for fixing a stray point.

**Straightening a grade (`G`).** Where a track profile has a vertical bump that
shouldn't be there, select points on that track (at least the two ends of the run)
and press **G**: every point from the first to the last selected is snapped onto a
straight, **endpoint-anchored** grade between them (a `elev x y z` overlay line per
point — only the elevation changes). The bump flattens for the rails, the smoothed
path the train rides, and the terrain carve.

**Connecting a siding to the track it crosses.** Sidings often overshoot the track
they should join — the end pokes past it with a red dead-end on the far side. Select
that dead-end and press **J**: its end snaps onto the nearest track its trajectory
crosses (a `move` overlay edit), trimming the overshoot to a clean turnout point; once
the end lies on the track it's no longer flagged red. (Only the geometry is moved — the
crossed track isn't split, so the vehicle's through-routes are unaffected.)

**Fixing broken links.** Some exports leave a line disconnected across a gap (e.g. a
tunnel approach), which derails the train at the loose end. Aim the centre crosshair
at a red dead-end and **Enter** to pick end **A**, aim at the other loose end and
**Enter** again for **B**, then **L** to link them (a `link …` overlay edit; **X**
clears the pick). The two segments join into one continuous route and the train runs
across the former gap.

## Data format

See `../terrainmapper/doc/game-export-format.md`. In short: 256×256 little-endian
`float32` heightmaps (`terrain.hm32`, row 0 = north), EPSG:25833 (UTM 33N) metres,
tiled across four fully-overlapping LOD levels. World coordinates are rendered
relative to the start point to preserve float precision.

Per tile the viewer also reads, when present:

- `landcover.u8` — 256×256 uint8 AR50 `artype` codes; the terrain is textured by
  land type (procedural per-class surfaces in a Vulkan texture array, sampled in
  world space — `src/Textures.cpp`, `shaders/terrain.frag`). Requires exporting
  terrainmapper **with an AR50 dataset loaded**.
- `tracks.bin` — railway polylines (deduped by `trackId`), including a per-vertex
  OSM **speed limit** used for banking.
- `roads.bin` — road polylines (deduped by geometry), category + number.
- `buildings.bin` — OSM building footprints with kind, roof shape, base
  elevation and height.
- `platforms.bin` — OSM station-platform footprints with base elevation and slab
  height.

The track/road/building geometry appears only when the export was produced with
the corresponding sources (national rail register + NVDB roads + OSM enrichment).

## Debug environment variables

| Variable            | Effect                                                        |
|---------------------|--------------------------------------------------------------|
| `EBANER_SCREENSHOT` | Render ~20 frames, write that frame to the given PPM, exit.   |
| `EBANER_CAM`        | Scripted camera `"x,y,z,yawDeg,pitchDeg"` (scene-relative m). |
| `EBANER_NOSTITCH`   | Skip the seam-stitching pass (to inspect raw tile seams).     |
| `EBANER_NOCARVE`    | Skip carving railway cuttings into the terrain.               |
| `EBANER_NOOVERLAY`  | Ignore the `overlay/` track edits (link fixes).               |
| `EBANER_VEHICLE`    | Skip the start screen and preselect a vehicle (`0` or `1`).   |
| `EBANER_AUDIO_DUMP` | Render a scripted brake sequence to the given WAV and exit.   |
| `EBANER_AUDIO_DUMP_ENGINE` | Render an engine start/idle/stop to the given WAV, exit. |

## Not yet implemented

A per-engine/per-bogie driveline split, hydrodynamic (retarder) braking through the
converter, wheelslip/slip-control, a detented reverser gate / key interlock (no
reversing above zero speed), brake-pipe/triple-valve propagation and
multi-unit consist braking, per-wheel grip-vs-slip and overturn at speed,
terrain-grounded derailment, carriage bodies and multi-car consists (couplers),
and streamed/dynamic tile loading.

## License

ebaner is licensed under the **GNU General Public License, version 3 (GPLv3)** —
see [`LICENSE`](LICENSE).

Copyright © Jan-Espen Oversand &lt;sigsegv@radiotube.org&gt;.

## Contributing

Contributions are welcome under the terms in [`CONTRIBUTING.md`](CONTRIBUTING.md).
Note that, in addition to the GPLv3, contributors grant the maintainer
(Jan-Espen Oversand) the right to relicense the project and to offer it under
additional licenses.
