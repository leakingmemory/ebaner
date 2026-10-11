# ebaner

A train simulator for the Norwegian railways, in Vulkan and C++.

Trains are driven over the line as it actually is. The terrain, the track and the
gradients come from a `terrainmapper` export of the national data, and what the train
does on them is modelled rather than scripted: a continuous air brake that has to be
kept charged and that feels how long the train is, traction worked out from each
machine's own transmission, couplers that can be parted and run round, track circuits,
signals, interlocked routes and line blocks, and a driver the computer can take over
from. Every train in the world is simulated, not only the one being driven.

It also draws the world it runs through — the elevation tiles around a **chosen
station** stitched into a continuous 3D surface, with the railway, roads, buildings
and platforms on top, streaming in as you travel. The camera starts on the running
line at that station: at a terminus like Bodø that is the buffer-stop end of track 1,
resolved from the track geometry rather than named here.

A second program, **`ebaner-trackedit`**, is the authoring side — the track,
signalling and interlocking editor the dataset's overlay is written with.

## What it does

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
- **Only what is on screen** — the ground, the buildings and the ballast-and-rails run
  are each cut into pieces with a bounding sphere, and a piece outside the view frustum
  is not submitted (`Frustum` in `src/VulkanRenderer.cpp`). The world streams for twenty
  kilometres around the camera and a view holds a wedge of that, so most of it was being
  sent to the card every frame to be thrown away behind the eye. The pieces come off
  what the geometry already is — a terrain tile, the run of one map tile's buildings, a
  200 m length of line — and the bounds are measured from the vertices actually emitted
  rather than from the footprints they were derived from, since a roof stands several
  metres above the wall top it was built on. Nothing is reordered to achieve it: which of
  two roofs at the same height is the one drawn depends on the order they are submitted
  in, so grouping them by anything other than the order they already arrive in would
  quietly change the picture. Worth 39-51% of the frame time on the machine it was
  measured on, and it renders pixel for pixel what it did before.
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
- **Rolling stock** — the NSB Class 93 diesel railcar (single, or two and three sets in
  multiple), the NSB Di 4 and CargoNet CD 312 diesel-electric locomotives, NSB Type 5
  carriages and a WLAB-2 sleeper, and pocket and container wagons, made up into a
  passenger train, a night train and a 600 m freight. Each machine's transmission,
  traction curve and cab are its own. See [Rolling stock](docs/rolling-stock.md).
- **Brakes** — an automatic air brake with a distributor, auxiliary reservoir and
  cylinder on every bogie, so an application travels down the train and a parted pipe
  stops it. EP on the Class 93, an engine brake blended in on the railcar, and electric
  and independent brakes on the locomotives. See [Braking](docs/braking.md).
- **Coupling and more than one train** — every train in the world is simulated. Trains
  couple when driven together gently, and derail when they meet too fast. Esc places
  another train or switches to driving one.
- **Auto-driver** — `P` hands the train over. It drives to the line speed and stops at
  signals, stations and the end of the line. See [The auto-driver](docs/auto-driver.md).
- **Sound** — brake air, each diesel's own engine voice, the Di 4's grid blower,
  coupling and collision, and rolling noise, all synthesised from the simulation. See
  [Sound](docs/audio.md).

## Requirements

System packages (all found via CMake / pkg-config):

- Vulkan SDK / loader + headers
- GLFW 3
- GLM
- `glslc` (shader compiler, from shaderc)
- PulseAudio (`libpulse-simple`) or PortAudio (**optional** — either enables the
  synthesized brake sound, PulseAudio preferred; the build is silent without both)
- Lua 5.4 (**optional** — enables the dataset's `overlay.lua`; see *Scripting* below.
  Found as `lua5.4` / `lua-5.4` / `lua54` via pkg-config. LuaJIT is deliberately not
  accepted: it would answer to "some Lua" and quietly give 5.1 semantics)
- CMake ≥ 3.20, a C++20 compiler

## Build

```sh
cmake -S . -B build
cmake --build build -j
```

This builds three executables that share the same rendering/loading engine
(`libebaner_engine`): the **`ebaner`** simulator above, **`ebaner-trackedit`**
below, and **`ebaner-dumptrack`**, which answers questions about the network the
way the sim sees it - after `track-edits.txt` is applied, which is the only form
worth deriving anything from. `--near` lists what is around a point, a track id
prints its vertices, `--switches` shows the turnouts and which overrides reached
them, `--route` asks the editor's own search whether a road exists, and `--gaps`
lists the loose ends: endpoints with nothing inside the 1 m the path builder
joins at, each with the nearest other loose end, which is what a `link` edit
would join. A buffer stop is a loose end too, so the distance is the tell - a
real break is two ends facing each other a few tens of metres apart.

### Measuring a train that is moving

Every profile before this one was of a train **standing still**, which is a profile of a
different program: nothing crosses a section boundary, so occupancy never changes, aspects
are never recomputed, the signal mesh is never rebuilt and no crossing steps a phase. All
four of those reported **zero calls a second** while a 600 m freight sat at Rognan. Moving
at 72 km/h past the same crossings the frame was **61–64 ms**, not the 20 that standing
still had suggested.

`EBANER_ROLL=<m/s>` is what makes that measurable without a keyboard: the train spawns at
that speed with the brake released and coasts. 1797 t against Davis resistance rolls a long
way, which is the long window a profile wants.

What the moving profile then said, at Rognan:

| block | mean |
|---|---|
| **vehicle mesh** | **19–21 ms** |
| crossings | 4.9 ms |
| signal mesh rebuild + upload | 3.9 ms |
| sim step | 3.0 ms |
| **occupancy — the track circuits** | **1.6–2.2 ms** |
| `drawFrame` | 1.5 ms |
| HUD text | 1.0 ms |

Two things follow. The **track circuits are not the cost** — threading them could buy 2 ms
of 62 and would leave the interlocking acting on a frame-old world. And the frame is
**CPU-bound**: `drawFrame` is 1.5 ms, and running the same scene in map mode, which skips
every 3-D mesh, saves only ~6 ms.

### Not building what cannot be seen

The vehicle mesh was the whole top of that list, and measuring *inside* it settled how to
cut it: of the per-frame cost, the network walks are **4 %** and emitting the 83 388
vertices is the other 96 %. So the only way to make it cheaper is to emit fewer of them.

`refresh` now takes the view-projection and builds a unit only if its bounding sphere is in
frustum; the rest have their slice of the buffer filled with **one repeated point**, which
leaves every triangle in it degenerate. Nothing is rasterised, the vertex count does not
move, and the index buffer on the GPU stays exactly as valid as it was. The `Frustum` is
the one the terrain and building draws already use, lifted out of `VulkanRenderer.cpp` into
`Frustum.h` rather than copied.

| at Rognan, 600 m freight, **moving** | before | after |
|---|---|---|
| **cab view** (what you drive from) | 23.4 ms (43 fps) | **18.5 ms (54 fps)** |
| of which vehicle mesh | 8.4 ms | **2.0 ms** — 1 unit of 23 built |
| chase view | 23.4 ms | 23.9 ms — nothing to cull, whole train in shot |

Proved rather than eyeballed, because the failure mode is a wagon quietly missing: over
**192 camera positions** round a curved train, near and far, high and low, every vertex the
cull dropped was checked against the view volume — **5 496 156 dropped, 0 of them on
screen**.

One incidental find worth the note: calling `glfwGetFramebufferSize` a second time in the
frame, to get the aspect for that view-projection, cost **~6 ms a frame** on this machine.
It asks the window system and can take a round trip. The numbers were already in hand from
the top of the frame.

### What a crossing asks an axle

A level crossing has to know which of its roads each wheel is on, and it used to find out
by **searching**: walk the approach in 5 m steps calling `poseAt` at each one, from
`s - outerM - 200` to `s + outerM + 200`. With approaches of 968–1473 m that is up to
**670 samples, per axle, per crossing track, per frame**. A cheap straight-line reject
keeps distant crossings free, which is why it never mattered: a 6-axle railcar costs 4 000
samples at the crossing it is standing on and nothing anywhere else.

The 600 m freight has **112 axles**, and every one of them is inside the reject radius of
the crossing it is standing at: **75 000 `poseAt` calls in one frame**, for one crossing.
That is the whole of "it gets terrible near a level crossing with a long train".

The answer was already known and thrown away. Every axle's position comes from `walkTo`,
which produces the path index and the arc position along it — exactly what those 670
samples reconstruct — and `railFrame` kept only the world point. Axles now also come as
**`AxlePlace { pos, path, s }`**, and a crossing answers by subtraction: same path, and
within reach along it. The search stays as the **fallback**, because a track can belong to
more than one `TrackPath` and a train walking a different one over the same rails still has
to be seen; the fast form says whether its answer is *final*, so the fallback runs only for
that case.

The same block walked **all 3388 turnouts** for every axle standing on a crossing, to find
the points between it and the crossing. Turnouts do not move — only their states do — so
that list is built once per path at load.

Measured at Rognan, which has two crossings, with the 600 m freight:

| | before | after |
|---|---|---|
| whole frame | 43.4 ms (23 fps) | **20.6 ms (49 fps)** |
| of which crossings | dominant | **1.8 ms** |
| one railcar, for comparison | 8.4 ms | 7.7 ms |

Checked rather than assumed, because a crossing that stops seeing a train does not fail
loudly — it just fails to close. Running both forms side by side over the real network at
four stations agreed **1 635 000 times out of 1 635 000**, with no answer more than 5 m
apart.

### What a long train costs, and what it need not

A 600 m freight is 23 units and **83 388 vertices** — the longest train before it was the
8-unit night train — and the per-frame path was rebuilding all of it, indices and all, and
then copying 3 MB twice. On an Intel HD 510 that was **8.4 ms of a 20.5 ms frame**.

Three of those things cannot change between frames and are no longer redone. The **index
buffer** is fixed from the moment it is attached (that is why changing a train's
composition has to go through `attachVehicle`), so the 144 000 index pushes were building
exactly the buffer the GPU already held. The **opaque/glass split** is fixed for the same
reason, so `sortGlass`'s colour sort over 48 000 triangles was finding the same answer
every frame; only the `texLayer` stamp has to be put back, and which vertices carry it is
recorded once. And `updateVehicleVertices` **swaps** rather than copies, since the
deferral past the frame fence is what it is for, not the copy.

| | before | after |
|---|---|---|
| 600 m freight, vehicle mesh | 8.37 ms | **6.30 ms** |
| 600 m freight, upload | 0.62 ms | **0.00 ms** |
| 600 m freight, whole frame | 20.51 ms (49 fps) | **17.04 ms (59 fps)** |
| one railcar, vehicle mesh | 3.01 ms | **2.03 ms** |

`refresh()` is checked against `build()` offline rather than by eye: for every vehicle in
the table it reproduces the buffer **bit for bit**, vertices, glazing tags and indices
alike. The failure it guards against is silent — a train drawn from the wrong vertices
looks like a heap of triangles — so it also warns if a refresh ever emits a different
vertex count, which is what a composition change without an `attachVehicle` would do.

What is left of the 6.3 ms is the geometry itself, and the way to cut that is to stop
making geometry nobody can see: the draw has no **culling** at all, so all 23 units are
submitted whether they are in front of the camera or half a kilometre behind it. The
machinery is already here — `drawFrame` builds a `Frustum` and the terrain and building
draws skip chunks with `frustum.sees(centre, radius)` — and the vehicle is the one that
does not use it.

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
./build/ebaner "" Fauske                  # dataset from $EBANER_DATA or an install
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

On startup the console prints the resolved start point (UTM 33N), the look direction,
tile/triangle counts, and vehicle physics (mass, inertia, tipping limit) — and, first of
all, two `[Paths]` lines saying which dataset and which shaders it settled on, which is
the first question to ask of a build that will not start.

### Where the dataset is looked for

The export is **installed separately**. The tiles run to tens of gigabytes, so they are
in neither this repository nor any package built from it, and nothing here installs
them; this is the reading side only. Named on the command line it is taken as given —
a wrong path is reported as the wrong path rather than quietly replaced. Otherwise the
first of these that holds a `tiles/` directory wins:

| | where |
|---|---|
| 1 | `$EBANER_DATA` |
| 2 | `$XDG_DATA_HOME/ebaner`, or `~/.local/share/ebaner` *(Unix)* |
| 3 | `<directory of the binary>/data` |
| 4 | `<directory of the binary>/../share/ebaner` |
| 5 | each of `$XDG_DATA_DIRS` + `/ebaner`, by default `/usr/local/share` then `/usr/share` *(Unix)* |
| 6 | the `datadir` this build was configured with, `$prefix/share/ebaner` |

**Rows 2 to 6 are each tried twice**: the directory itself, and `norway-rails` inside
it. So `/usr/share/ebaner/norway-rails` is found, which is where a package installs the
export — `share/ebaner` already holds the program's own `shaders/`, and emptying tens of
thousands of terrain tiles in beside them is no way to lay out a prefix. An export
unpacked so that its `tiles/` sit directly in `share/ebaner` is found too; neither
layout has to know about the other.

Rows 3 and 4 are what a Windows build will have, where there is no XDG and the binary's
own directory is the only thing to go on. Found nothing, the program lists every place
it looked and stops.

**No development path is searched and none is compiled in.** Working on the program
means saying where the data is — on the command line, or in `EBANER_DATA`:

```sh
./build/ebaner ../norway-rails Fauske
EBANER_DATA=../norway-rails ./build/ebaner "" Fauske
```

The compiled shaders need no such help and are found the same way: `$EBANER_SHADERS`,
then `<binary>/shaders`, then `<binary>/../share/ebaner/shaders`, then the configured
`datadir`. The second of those is the Windows install layout and the build tree both —
CMake puts the `.spv` files in `build/shaders` beside `build/ebaner` — so nothing has to
know it is running from a build, and no build path is baked into the binary.

"<binary>" is the directory the running executable is in. It comes from the OS where
there is a way to ask (`/proc/self/exe`, `GetModuleFileName`); failing that, from
`argv[0]`, which is used as given when it is a path and looked up on `$PATH` when it is
a bare name — the same search the shell did, against the same `PATH`, finding the same
file.

## Install

Step-by-step instructions for Linux — distribution packages, fetching the dataset and
its overlay, and an unofficial Gentoo overlay — are in [`INSTALL.md`](INSTALL.md).

```sh
cmake --install build --prefix /usr/local     # or DESTDIR=... for packaging
```

A plain GNU layout, through `GNUInstallDirs`, so `-DCMAKE_INSTALL_LIBDIR=...` and the
rest work as expected:

| | |
|---|---|
| `$prefix/bin` | `ebaner`, `ebaner-trackedit`, `ebaner-dumptrack` |
| `$prefix/share/ebaner/shaders` | the compiled `.spv` files |
| `$prefix/share/doc/ebaner` | `README.md`, `LICENSE` |

Put the export at `$prefix/share/ebaner/norway-rails` (beside `shaders/`, not in among
it), or anywhere else in the list above.

### Windows

`.github/workflows/windows.yml` builds x64 executables with MSVC and vcpkg. It is
**dispatched by hand, and runs on a pushed `v*` tag** — nothing else. A Windows runner
bills at twice the Linux rate, and the project is developed and tested here.

It installs the Vulkan SDK from LunarG (headers, `vulkan-1.lib`, and `glslc`), takes
glfw3, glm, lua and portaudio from `vcpkg.json`, builds against the
**`x64-windows-static`** triplet, runs `ctest`, and packs `cmake --install` output into
`ebaner-windows-x64.zip` — the same `bin/` and `share/ebaner/shaders/` layout as above,
which the path search finds with no help. On a tag the zip is attached to the release.

Static on purpose: the zip needs no DLLs beside it and no MSVC redistributable. The one
thing loaded at run time is `vulkan-1.dll`, which comes with the graphics driver and is
not ours to ship. The dataset is still installed separately — put it in `data\` beside
the executables, or point `EBANER_DATA` at it.

Of the 18 tests, 11 run on the runner and 7 report skipped, having found no export.

`vcpkg.json` is inert for a Linux build: manifest mode engages only when the vcpkg
toolchain file is passed, which it never is here.

**Before spending a run on it**, the whole tree can be compiled for Windows locally with
the mingw-w64 cross compiler. It is not MSVC and will not find everything MSVC objects
to, but it catches anything POSIX that has crept in, and it costs nothing:

```sh
mkdir -p /tmp/wininc && ln -sfn /usr/include/{vulkan,vk_video,GLFW,glm} /tmp/wininc/
for f in src/*.cpp tools/*.cpp tests/*.cpp; do
    x86_64-w64-mingw32-g++ -std=c++20 -fsyntax-only -I src -I /tmp/wininc \
        -DEBANER_DATADIR='"C:/x"' "$f" || echo "FAILED: $f"
done
```

All 70 translation units pass it today. `src/Paths.cpp` goes further than that and has
been run: cross-compiled and executed under wine, it resolves its own directory through
`GetModuleFileNameW` and finds both the installed layout (`bin\..\share\ebaner`) and
the portable one (`shaders\` and `data\` beside the executable).

## Controls

For a walk-through of driving a train from cold, see
[Driving the Class 93](docs/driving-class-93.md),
[Driving the Di 4](docs/driving-di4.md) and
[Driving the CD 312](docs/driving-cd312.md).

The driving keys work the cab you are sitting in (`V`), or the front cab in any other
view. What `,` and `.` move depends on the machine: the Class 93 has one combined
power/brake lever, while the Di 4 and CD 312 have a power controller with a separate
train brake (`K`/`L`) and locomotive brake (`H`/`J`).

| Input        | Action                          |
|--------------|---------------------------------|
| W / A / S / D| Move horizontally               |
| Q / E        | Move down / up                  |
| Mouse        | Look                            |
| Left Shift   | Move faster (×8)                |
| C            | Toggle chase camera (the vehicle) |
| V            | Driver's-seat view; press again to switch cab, again to exit |
| I            | Start / stop the diesel engines (all of the set's engines together) |
| Up / Down    | Hand-push the vehicle fwd / back |
| , / .        | Class 93: combined lever toward power / toward brake (N → P1–P5, N → B1–B4 → EMERG). Di 4, CD 312: power controller up / down (P1–P5 above N, the electric brake E1–E5 below it) |
| K / L        | Train brake: release / apply a notch (Di 4, CD 312) |
| H / J        | Locomotive (independent) brake: release / apply (Di 4, CD 312) |
| Space        | Emergency brake                 |
| F / N / R    | Reverser: Forward / Neutral / Reverse. Putting a cab in gear centres every other cab |
| T            | Throw the switch under the crosshair (straight ⇄ diverging) |
| U            | Uncouple the coupler nearest the camera (within 30 m, train stopped): press once to select it, again within 5 s to part it |
| P            | Hand the train to the **auto-driver**, or take it back |
| O            | Open / close the traffic-manager map |
| M            | Mute / unmute sound             |
| Tab          | Release/grab cursor             |
| Esc          | Open menu (Exit); press again to resume. In a picker, backs out one step |

In the traffic-manager map (`O`), the cab keys are replaced by the map's own. See
[Using the traffic manager](docs/traffic-manager.md) for how to work it, and
[Working a TXP station](docs/txp-station.md) for a station worked by hand, or
[without line block](docs/txp-station-without-line-block.md) for one with its own
interlocking but none on the line, and
[Working a remote-controlled station](docs/remote-controlled-station.md) for a stretch
with line block:

| Input        | Action                          |
|--------------|---------------------------------|
| W / A / S / D| Pan                             |
| Mouse wheel, Z / X | Zoom in / out             |
| Left click on a switch | Throw it (motor switches only, and only when its locking allows) |
| Left click on a signal | Arm it, then click a ringed destination to set the route. Click a signal with a route set to cancel it |
| R            | Pick an exit route at the current station |
| E            | Pick an entry signal at the current station |
| N / B, Right / Left | Next / previous station |
| Up / Down, Enter | Move and choose in a picker |
| Q            | In the `R` route list: arm the highlighted route, to set itself once it can |

## Track editor (`ebaner-trackedit`)

```sh
./build/ebaner-trackedit ../norway-rails          # or: ... ../norway-rails Rognan
```

The editor shows the surveyed track points over the ghosted scene. In it you fix the
geometry, draw sidings, and lay out track circuits, main, block and simple signals,
level crossings, flag posts and TXP positions. Everything is saved to the dataset's
`overlay/` directory, which the simulator applies at load. See
[The track editor](docs/track-editor.md).

## Scripting

Everything in `overlay/` so far is a *fact*: a crossing is a line in a file, a TXP position
is a line in a file. Some of what a railway needs is not a fact but a **rule** — a
timetable, a train that runs itself, when a station is worked — and there was nowhere to
write a rule down. `<dataset>/overlay/overlay.lua` is that place. The simulator runs it once at
startup, after the world, the paths, the switches and every other overlay are loaded, so
that when it is eventually given an API there is already something for it to look at.

```lua
print("God dag frå Nordlandsbanen!")
```

| | |
|---|---|
| no file | nothing said, like every other overlay |
| ran | `[Script] ran overlay.lua` |
| it failed | `[Script] overlay.lua: <the Lua message>`, and the simulator carries on |
| file present, built without Lua | `script: overlay.lua present but built without Lua; not run` |

A bad script does not stop the simulator: it is authored content, and authored content that
does not resolve is reported and stepped over. But a script that is *never run* is told
about — that is the one place this differs from the optional audio, where silence is a fair
thing to leave unsaid.

The interpreter **stays open** after the chunk returns, so what a script defines is still
defined afterwards. That is what makes hooks possible later without a rewrite. The full
standard library is loaded, `io` and `os` included: the script is in the reader's own
dataset and written by them, so a sandbox would keep nobody out and would rule out the file
work a scripted authoring task wants to do.

Nothing of the simulator is exposed yet — no trains, no signals, no stations, no TXP
network, and no `init`/`tick`. Those want designing against a real first script. The editor
does not run scripts; the host is in the engine library so it can be given them later.

## Data format

See `../terrainmapper/doc/game-export-format.md`. In short: 256×256 little-endian
`float32` heightmaps (`terrain.hm32`, row 0 = north), EPSG:25833 (UTM 33N) metres,
tiled across four fully-overlapping LOD levels. World coordinates are rendered
relative to the start point to preserve float precision.

Per tile the simulator also reads, when present:

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
| `EBANER_DATA`       | The terrainmapper export to use, ahead of every other place searched (a path given on the command line still wins). |
| `EBANER_SHADERS`    | The directory holding the compiled `.spv` files, ahead of every other place searched. |
| `EBANER_SCREENSHOT` | Render ~20 frames, write that frame to the given PPM, exit.   |
| `EBANER_CAM`        | Scripted camera `"x,y,z,yawDeg,pitchDeg"` (scene-relative m). |
| `EBANER_NOSTITCH`   | Skip the seam-stitching pass (to inspect raw tile seams).     |
| `EBANER_NOCARVE`    | Skip carving railway cuttings into the terrain.               |
| `EBANER_NOOVERLAY`  | Ignore the `overlay/` track edits (link fixes).               |
| `EBANER_EDMODE`     | `ebaner-trackedit` only: start in this mode, by its menu name. |
| `EBANER_VEHICLE`    | Skip the start screen and preselect a vehicle, by its index in `kVehicleSpecs` (`0` = a single Class 93, `1`/`2` = two and three coupled, `3` = Di 4 + 5, `4` = the night train, `5` = the 600 m freight, `6` = a light Di 4, `7` = a CD 312, `14` = a pocket wagon, `15` = a container flat). The table is ordered for the pickers, so these move when a vehicle is added - the start screen numbers the list. |
| `EBANER_PROFILE`    | Print per-frame timings once a second: whole frame, the sim step, occupancy, crossings, signal aspects, distant walks, the signal mesh, and the vehicle mesh and its upload. With `EBANER_SCREENSHOT=/dev/null EBANER_SHOTFRAME=900` the simulator exits by itself with the reports flushed, which is how a profile gets taken without a file or a keypress. |
| `EBANER_RADIUS`     | Half-window of world to load, in metres (default 20000). Two kilometres loads in seconds where the full 40 km square takes minutes, which is what makes a profile or a frame-time measurement affordable at all. Inspection only: drive out of it and the world ends. |
| `EBANER_ROLL`       | Spawn the train at this speed in m/s with the brake released, so it coasts. A profile of a standing train is a profile of a different program - nothing changes occupancy, so nothing downstream of it runs. |
| `EBANER_GREEN`      | `"OTR 1"` — clear one simple entry signal by name before the first frame, manning its station. The other half of setting up a crossing without a keyboard: with every signal at danger a train stops at the first one, and nothing that happens inside a station could be watched. |
| `EBANER_THROW`      | `"x,y;x,y"` — throw the turnout nearest each world point to diverging before the first frame. A crossing move is a train taking the loop rather than the main, and until this there was no way to set one up without a keyboard, so every bug in what the driver makes of a loop had to be found by hand. |
| `EBANER_AUTO`       | Hand the train to the auto-driver at startup and log what it is doing once a second. A train that drives a real line with no keyboard, which is what makes the signalling and the crossings testable without a driver sitting in the cab. |
| `EBANER_CHASE`      | `1` rides the vehicle from the start (the chase camera is otherwise only reachable by pressing C, so the one view that shows the machine could not be screenshotted). |
| `EBANER_AUDIO_DUMP` | Render a scripted brake sequence to the given WAV and exit.   |
| `EBANER_AUDIO_DUMP_ENGINE` | Render an engine start/idle/stop to the given WAV, exit. |
| `EBANER_AUDIO_DUMP_CROSSING` | Render a crossing bell activating/falling silent, exit. |
| `EBANER_AUDIO_DUMP_ROLLING` | Render rolling noise sweeping speed, weight, curve and brake, exit. |

## Not yet implemented

A per-engine/per-bogie driveline split, hydrodynamic (retarder) braking through the
converter, wheelslip/slip-control, a detented reverser gate / key interlock (no
reversing above zero speed), per-wheel grip-vs-slip and overturn at speed, and
terrain-grounded derailment.

## License

ebaner is licensed under the **GNU General Public License, version 3 (GPLv3)** —
see [`LICENSE`](LICENSE).

Copyright © Jan-Espen Oversand &lt;sigsegv@radiotube.org&gt;.

## Contributing

Contributions are welcome under the terms in [`CONTRIBUTING.md`](CONTRIBUTING.md).
Note that, in addition to the GPLv3, contributors grant the maintainer
(Jan-Espen Oversand) the right to relicense the project and to offer it under
additional licenses.
