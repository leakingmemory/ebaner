# ebaner

A small Vulkan/C++ terrain viewer for the `terrainmapper` game-export data
(`../norway-rails`). It stitches the elevation tiles around **Bodø station** into
a continuous 3D surface and lets you fly around it. The camera starts at the
buffer-stop end of the Bodø main track ("track 1 end"), resolved from the
`tracks.bin` geometry.

It renders **terrain only** (railway tracks and roads are not drawn yet), shaded
with hillshade and coloured by **AR50 land cover** when the export provides it
(forest, agriculture, open land, bog, glacier, water …), falling back to an
elevation ramp otherwise.

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
prints the resolved start point (UTM 33N), the look direction, and how many tiles
and triangles were loaded.

## Controls

| Input        | Action              |
|--------------|---------------------|
| W / A / S / D| Move horizontally   |
| Q / E        | Move down / up      |
| Mouse        | Look                |
| Left Shift   | Move faster (×8)    |
| Tab          | Release/grab cursor |
| Esc          | Quit                |

## Data format

See `../terrainmapper/doc/game-export-format.md`. In short: 256×256 little-endian
`float32` heightmaps (`terrain.hm32`, row 0 = north), EPSG:25833 (UTM 33N) metres,
tiled across four LOD levels. World coordinates are stored relative to the start
point to preserve float precision.

Land cover: if a tile has a `landcover.u8` file (256×256 uint8 AR50 `artype`
codes, same grid as `terrain.hm32`), the terrain is coloured by land type
(elevation ramp blended 60/40 with a per-class tint, water/glacier overriding).
This requires exporting terrainmapper **with an AR50 dataset loaded**; plain
exports omit the file and the viewer falls back to the elevation ramp.

Note: contrary to the format doc, the LOD levels in this export **fully overlap**
(the same ground is present at every LOD as a power-of-two quadtree). The renderer
therefore de-overlaps — it keeps only the finest tile per ground area — and then
watertight-stitches the resulting seams (edge bridges + corner fills) so there are
no cracks or T-junctions between differing resolutions (see `src/TerrainMesh.cpp`).

## Debug environment variables

| Variable            | Effect                                                        |
|---------------------|--------------------------------------------------------------|
| `EBANER_SCREENSHOT` | Render ~20 frames, write that frame to the given PPM, exit.   |
| `EBANER_CAM`        | Scripted camera `"x,y,z,yawDeg,pitchDeg"` (scene-relative m). |
| `EBANER_NOSTITCH`   | Skip the seam-stitching pass (to inspect raw tile seams).     |

## Not yet implemented

Track/road rendering and streamed/dynamic tile loading.
