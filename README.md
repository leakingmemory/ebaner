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

- **The world** — the terrain from the national elevation data, coloured by land cover
  and cut for the railway, with the track, roads, buildings and platforms on it,
  streaming in around the camera. See
  [The world and its rendering](docs/world-and-rendering.md).
- **Switches** — a hand switch stand with its indicator target at every turnout. `T`
  throws one; a train runs the way it is set, forces a trailing switch set against it,
  and derails at a broken one.
- **Signalling** — track circuits, interlocked entry and exit routes, line block with
  block signals, level crossings, and stations worked by a TXP with simple signals,
  flags and train orders, all worked from the traffic manager (`O`). See
  [Using the traffic manager](docs/traffic-manager.md) and the station guides under
  Controls.
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

This builds three executables sharing one engine (`libebaner_engine`): the
**`ebaner`** simulator, the **`ebaner-trackedit`** editor, and **`ebaner-dumptrack`**,
which answers questions about the network as the simulator sees it, with the overlay's
track edits applied:

| | |
|---|---|
| `--near` | what is around a point |
| a track id | that track's vertices |
| `--switches` | the turnouts, and which overrides reached them |
| `--route` | whether a road exists, by the editor's search |
| `--gaps` | the loose ends, each with the nearest other loose end: what a `link` edit would join |

How the world is built and drawn, and how to profile it, is in
[The world and its rendering](docs/world-and-rendering.md).

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

ebaner reads a `terrainmapper` export, with an overlay of authored edits on top. See
[Data format](docs/data-format.md).

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
