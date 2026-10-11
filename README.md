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
- **Signalling** — track circuits, interlocked entry and exit routes, line block and
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
- PulseAudio (`libpulse-simple`) or PortAudio (**optional**: without either the
  simulator is silent)
- Lua 5.4 (**optional**: runs the dataset's `overlay.lua`, see
  [Scripting](docs/scripting.md). LuaJIT isn't accepted)
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

The tests run against a real export, looked for at `../norway-rails`; point them
elsewhere with `-DEBANER_TEST_DATASET=/path/to/export`. Without one they report as
skipped.

## Run

```sh
./build/ebaner ../norway-rails            # pick the station on the start screen
./build/ebaner ../norway-rails Fauske     # or name it
./build/ebaner "" Fauske                  # dataset from $EBANER_DATA or an install
```

The dataset is installed separately; where it is looked for is in
[INSTALL.md](INSTALL.md#where-the-program-looks). Any of the export's 720 stations can be
named, ignoring case and Norwegian letters (`Bodo` finds `Bodø`). The first console
lines, `[Paths]`, say which dataset and shaders were found.

## Install

See [INSTALL.md](INSTALL.md) for Linux: dependencies, building and installing, the
dataset, and an unofficial Gentoo overlay. A Windows build is made by CI; see
[Windows](docs/windows.md).

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

An optional `overlay/overlay.lua` in the dataset is run at startup. Nothing of the
simulator is exposed to it yet. See [Scripting](docs/scripting.md).

## Data format

ebaner reads a `terrainmapper` export, with an overlay of authored edits on top. See
[Data format](docs/data-format.md).

## Environment variables

`EBANER_DATA` and `EBANER_SHADERS` set where the dataset and shaders are. The rest set up
scenarios, screenshots and profiling without a keyboard; see
[Environment variables](docs/debug-variables.md).

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
