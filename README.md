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
  (`src/TerrainMesh.cpp`).
- **Railway** — a realistic cross-section: a ballast bed, concrete sleepers and
  two rusty rails (`src/TrackMesh.cpp`). Centrelines are smoothed through the
  coarse surveyed points with a **centripetal Catmull-Rom** spline
  (`src/TrackPath.cpp`), so curves are continuous. Sleepers are real 3-D boxes
  near the camera and a repeating texture at distance (distance LOD). Track is
  **superelevated (banked)** on curves from the OSM speed limit + curvature.
- **Roads** — category-coloured asphalt ribbons (`src/RoadMesh.cpp`): the public
  network (Europavei/Riksvei/Fylkesvei/Kommunal) prominent, private tracks thin
  and faint.
- **Buildings** — OSM footprints extruded into lit prisms coloured by type, with
  **pitched roofs** (flat / gabled / hipped / pyramidal / skillion) from the OSM
  `roof_shape` tag (`src/BuildingMesh.cpp`).
- **Station platforms** — OSM `railway=platform` footprints (area platforms and
  buffered platform edges) extruded into low lit concrete slabs seated on the DTM
  beside the tracks (`src/PlatformMesh.cpp`).
- **Rail vehicle** — chosen on an in-window **start screen** (a text menu): a
  single-axle wheelset or a dual-axle bogie (two wheelsets + frame), both unpowered
  (`src/VehicleMesh.cpp`). A small 1-DOF physics model (`src/Vehicle.cpp`) gives it
  mass, gravity resolved into along-track acceleration + weight-on-rails, Davis
  running resistance, box inertia and a curve overturning limit. It rolls under
  gravity on grades, can be **hand-pushed** (Up/Down), coasts to a stop via rolling
  resistance, and if it runs off the end of its track it derails and is slowed to a
  stop by ground friction proportional to its weight.

## Requirements

System packages (all found via CMake / pkg-config):

- Vulkan SDK / loader + headers
- GLFW 3
- GLM
- `glslc` (shader compiler, from shaderc)
- CMake ≥ 3.20, a C++20 compiler

## Build

```sh
cmake -S . -B build
cmake --build build -j
```

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
| Up / Down    | Hand-push the vehicle fwd / back |
| Tab          | Release/grab cursor             |
| Esc          | Quit                            |

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
| `EBANER_VEHICLE`    | Skip the start screen and preselect a vehicle (`0` or `1`).   |

## Not yet implemented

Traction/braking and rolling resistance, per-wheel grip-vs-slip and overturn at
speed, terrain-grounded derailment, multi-axle bodies, and streamed/dynamic tile
loading.

## License

ebaner is licensed under the **GNU General Public License, version 3 (GPLv3)** —
see [`LICENSE`](LICENSE).

Copyright © Jan-Espen Oversand &lt;sigsegv@radiotube.org&gt;.

## Contributing

Contributions are welcome under the terms in [`CONTRIBUTING.md`](CONTRIBUTING.md).
Note that, in addition to the GPLv3, contributors grant the maintainer
(Jan-Espen Oversand) the right to relicense the project and to offer it under
additional licenses.
