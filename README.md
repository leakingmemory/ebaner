# ebaner

A small Vulkan/C++ viewer for the `terrainmapper` game-export data
(`../norway-rails`). It stitches the elevation tiles around a **chosen station** into
a continuous 3D surface and draws the railway, roads and buildings on top, with a
first physics-driven rail vehicle. The ground streams in as you travel. The camera
starts on the running line at that station — at a terminus like Bodø that is the
buffer-stop end of track 1, resolved from the track geometry rather than named here.

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
- **Switch stands** — a classic Norwegian manual switch stand (sporveksel) at
  every turnout, i.e. wherever one track's end joins another track's line
  (`src/SwitchMesh.cpp`, `src/SwitchNetwork.cpp`). Each stand is a weighted lever
  (a bar ending in a low **cylindrical counterweight**) driving a throw rod to the
  movable rail, plus a tall post carrying a rotating **indicator target**. The
  target is one mechanical two-faced plate — an **arrow** on one face, a **filled
  circle** on the other, and edge-on it reads as a **vertical line**. Set straight
  it is edge-on to the track so both ends see the line; thrown to diverging it
  turns 90° so the toe/common approach sees the arrow (a facing train will be
  diverted to the siding) and the branch approach sees the filled circle (a train
  may run from the siding onto the main, but not the reverse). A broken switch
  sits part-turned with the lever centred.
- **Movable switches** — the switches actually route the train. Aim the centre
  crosshair at a stand and press **T** to throw it (straight ⇄ diverging); the
  lever and target animate and a train reaching the turnout is diverted to the
  set track. Enter a switch **facing** (from the common/toe side) and it takes you
  to the track it is set for — into a **broken** (neutral) switch it **derails**.
  Enter it **trailing** (from one of the two branches toward the common track)
  with the switch set against you and the train forces it through: it continues
  onto the common track but the switch is left **broken** in the neutral position
  (throw it again to repair it). Trailing through an already-broken switch just
  continues onto the common track.
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

## Test

```sh
ctest --test-dir build --output-on-failure
```

The tests run against a real terrainmapper export, since what the code has to cope
with is decided by what the data is actually like — there is no synthetic dataset to
check against. They look for one at `../norway-rails`; point them elsewhere with
`cmake -S . -B build -DEBANER_TEST_DATASET=/path/to/export`. Without an export they
report as skipped rather than failing, so a build without one is still clean.

- **`track-edit`** — an edit made in the track editor reaches every store that holds
  track geometry: the windowed tiles the terrain carve reads, and the resident network
  the paths, junction graph and turnouts are built from. Editing only one of them fails
  silently — applied to the tiles alone, raising a rail re-cuts the ground and leaves
  the rail where it was.

## Run

```sh
./build/ebaner ../norway-rails            # pick the station on the start screen
./build/ebaner ../norway-rails Fauske     # or name it, and skip straight past
```

### Where to start

Both binaries take an optional station name after the dataset. The stations come from
the export itself — every tile's `meta.json` lists the ones inside it, 720 in all — so
any of them works, not a list kept here. The name is matched ignoring case and the
Norwegian letters, so `Bodo` finds `Bodø` and `oteraga` finds `Oteråga`; an unknown name
prints the near misses rather than guessing. Omitted, both ask on screen before loading
anything (arrows, PgUp/PgDn, Enter; Esc quits).

Starting far afield is free — the scene is built around whichever station you pick.
Driving a long way from it is a different matter, and is what the floating-origin work
still outstanding is for.

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
| T            | Throw the switch under the crosshair (straight ⇄ diverging) |
| M            | Mute / unmute sound             |
| Tab          | Release/grab cursor             |
| Esc          | Open menu (Exit); press again to resume |

## Track editor (`ebaner-trackedit`)

```sh
./build/ebaner-trackedit ../norway-rails          # or: ... ../norway-rails Rognan
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

**Selecting a stretch (`Left` / `Right`).** Clicking every point of a kilometre of
line is not on, so the arrows run the selection along it: **Right** extends it forward,
**Left** backward, one point at a time, **Shift** ten, **Ctrl** fifty (auto-repeating
while held). It follows the route rather than one track — the main line through a
station is several tracks end to end — and at a junction it takes the straightest
continuation, the same rule the path builder chains by, so it does not wander off down
a siding. The HUD shows how many points and how many metres are selected.

**Raising / lowering.** With points selected, **Up** / **Down** nudge their elevation
by 0.1 m; **Shift** makes it 1 m and **Ctrl** 10 m, for dropping a long stretch onto
the terrain rather than fixing a stray point. All auto-repeat while held, the coarser
steps more slowly. Every selected point moves by the *same* amount, so the grade
through the selection is preserved — it is a shift, not a flattening (for that, see
`G` below). Each is an `elev` override, previewed live and saved with Ctrl+S.

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

**Adding a connecting rail (building a switch).** The export sometimes omits the
short connecting rails at a crossover or slip, so two tracks that should be joined
by switches only cross. Select two geo-points on **different** tracks (click one,
Ctrl+click the other) and press **R**: a straight connecting rail is added between
them (a `rail …` overlay edit). Its ends sit on the two tracks, so the switch
detection makes a **switch at each end** — a train can then divert across it (main ⇄
siding). Keep the meeting angle shallow, as a real turnout does; too steep and it is
treated as a crossing rather than a switch. Live preview; Ctrl+S saves; the switch
stands appear on reload / in the viewer.

**Scissors crossover (`C`).** For two roughly-parallel tracks (e.g. the main and a
loop), select one geo-point on **each** track, opposite each other, and press **C**: it
lays a **double crossover** — two short diagonal rails that cross in the middle (the
diamond), a switch on each track at each end — so trains can cross between the two lines
either way as well as run straight, like a station-throat scissors. Pick a spot where
both tracks are at grade (it uses the overlay-applied elevations). Live preview; Ctrl+S
saves; switch stands appear on reload / in the viewer.

**Auto-slip at a diamond (`K`).** For an actual diamond crossing (two tracks crossing at
a clear angle), select a single geo-point in the **middle of the crossing** and press
**K**: it finds the two crossing tracks and adds the two diagonal connecting rails (a
slip), a switch on each track at each end. Uses the overlay-applied elevations; skipped
if the crossing is nearly parallel (use `C`) or too steep to be a switch.

**Fixing broken links.** Some exports leave a line disconnected across a gap (e.g. a
tunnel approach), which derails the train at the loose end. Aim the centre crosshair
at a red dead-end and **Enter** to pick end **A**, aim at the other loose end and
**Enter** again for **B**, then **L** to link them (a `link …` overlay edit; **X**
clears the pick). The two segments join into one continuous route and the train runs
across the former gap.

**Simple entry signals.** The entry signals proper are routes between track-circuit
borders, with C1/C2 authorities and full interlocking — which needs circuits drawn
through the station, so they exist at Bodø and nowhere else. These are the plain
alternative: a short head with two steady lamps, red over green, and no circuits or
routes behind them. Pick **Simple entry signals** from the
Esc menu, then **click any point on any track** to place one (as with a distant signal —
no border needed), **right-click** to select, **F** to turn it round, **F2** to name,
**X** to delete, **Ctrl+S** to save to `overlay/simple-entry-signals.txt`.

Each attaches to the **nearest station** in the export, and the station is what they are
interlocked by. A station is either **off** — unmanned, its signals dark, trains running
through without reference to them — or **on**, when they show red and **one of them may
be cleared at a time**. A typical station has two, one at each end, and clearing one puts
the other back to red. Stations start off, so placing signals on a line does not stop the
traffic already running over it; switching a station off again clears any green with it,
so it comes back on in a known state.

The editor HUD names the station a selected signal attached to and how far off it is,
flagging anything beyond 4 km as suspect — that is almost certainly not the station it
serves. An explicit station name can be written into the overlay line to overrule the
nearest-station rule where two stations sit close together.

In the viewer they are worked from the traffic manager (`O`), where **E** opens the
station being worked: the first line switches the station on or off, then **All red**,
then its signals. A signal holds whatever it was last given — nothing else resets it, no
train passing and no timer — which is why the state of each is on the line beside it.

**Which station** the traffic manager is working is named in its title, and everything it
does — the entry-signal panel, the flag and TXP lines under it, and the exit routes `R`
offers — acts on that one station. Opening the map lands on the station nearest the train,
which is the one meant nine times out of ten; **N** and **B** then walk to the next and
previous station along the line, bringing the view with them. The list is the stations of
the line the run started on, in the order they come in, whether or not anything is
authored at them yet — that is what makes it possible to go and author something at the
next one. `EBANER_MAP=1` opens the traffic manager straight away, and `EBANER_MAP=<name>`
opens it at a named station, which is how it is screenshotted headlessly.

**Level crossings.** A crossing secured by lights alone — no barriers. Pick **Level
crossings** from the Esc menu and **click a point on any track**; **right-click** selects,
**F2** names, **X** deletes, **Ctrl+S** saves to `overlay/level-crossings.txt`. There is
no flip: a crossing faces both ways.

Four heads, each **red over white** and flashing: two facing the train, one each way along
the track, and two facing the road across it. Three detection circuits come with the
crossing rather than being drawn — an **inner** one over the crossing and out past the
train signals either side, and an **approach** circuit each way. The approach distance is
derived from the line speed there (braking distance plus the time the sequence needs), so
a 130 km/h main line arms about 1.5 km out and a 40 km/h branch a couple of hundred
metres; the editor HUD shows what it worked out, and a distance written into the overlay
line overrules it.

The sequence, with the road's lights first and the train's second:

| | road | train |
|---|---|---|
| **idle** | white, slow | red, slow |
| **closing** (5 s) | red, fast | red, fast |
| **secured** | red, fast | white, fast |
| **opening** (5 s) | red, fast | red, fast |

An approach circuit arms it on the **edge** — clear to occupied — and only from idle,
which is what stops a departing train re-arming the crossing from the far circuit. The
inner circuit arms it whenever it is occupied, no edge and no gate, because it is the
fallback for an approach circuit that has failed. The release runs on the inner circuit
clearing and on nothing else, and mirrors the delay: the train's signal drops to red at
once, the road opens 5 s later. A crossing armed by a movement that then turned back
releases itself after a minute of everything being clear.

Each crossing runs its **own** blink, phased off its id, so two within sight of one
another are visibly out of step — as real ones are, each having its own oscillator.

**Flag posts.** The hand signal a station's TXP hangs out: **red for stop, green for
pass through, and an empty fixture** when the station is unmanned or neither applies. Pick
**Flag posts** from the Esc menu and **click a point on any track**; **right-click**
selects, **F** swaps which side of the track the post stands on, **F2** names, **X**
deletes, **Ctrl+S** saves to `overlay/flag-posts.txt`.

The flag is not carried. It is on a stick that slots horizontally into a fixture near the
top of the post, so it hangs down toward the track and stays there. The post and its
fixture stand whether or not anything is in them — an empty fixture is itself an
indication. Because the flag hangs it reads from either direction, so there is no facing
to author; `F` is placement, not aim.

Each post attaches to the **nearest station**, which is only how the panel groups them:
`O` for the traffic manager, then **E**, gives one line per post under the worked
station's signals, and **Enter cycles it** — no flag, red, green, no flag. Every post is
independent. Setting one says nothing about any other, several may show different things
at once, and none of them is tied to the manned switch: a manned station has no flag out
most of the time.

Nothing constrains the flag against the fixed signalling either — a green flag against a
red entry signal is allowed, because part of why a hand signal exists is to say what the
signalling cannot.

**TXP positions (permission to leave).** The one hand signal that hangs on nothing: the
TXP walks out to a spot beside the track where the driver can see them and holds up a
round sign — **a green ring around a white centre**, a European prohibitory sign with the
red swapped for green — giving a stopped train permission to leave. Pick **TXP
positions** from the Esc menu and **click a point on any track**; **right-click** selects,
**F** turns the signal round, **B** swaps which side of the track they stand, **F2** names,
**X** deletes, **Ctrl+S** saves to `overlay/txp-positions.txt`.

The direction is where the train departs to — the driver is looking that way, so the TXP
faces back against it. The side is absolute, so turning the signal round does not walk
them across the track.

A station is meant to have **as many positions as it needs**, and most need several: a TXP
standing at one end of Fauske cannot be seen from the other, so anywhere a train might be
stopped wants its own spot. **The editor draws a figure at every position**, which is the
only way to see what a station actually covers while authoring it.

Worked from the station panel (`O`, then **E**), one line per position which Enter shows
or takes down. **Only one shows at a time per station**: showing one stands the TXP down
wherever they were. That is the one place the "one person" argument really holds — unlike
the flags, which sit in fixtures and can all be out at once. In the sim nothing is drawn
at a position that is not showing, and nothing depends on the manned switch.

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
