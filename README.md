# ebaner

A small Vulkan/C++ terrain viewer for the `terrainmapper` game-export data
(`../norway-rails`). It stitches the elevation tiles around **Bodø station** into
a continuous 3D surface and lets you fly around it. The camera starts at the
buffer-stop end of the Bodø main track ("track 1 end"), resolved from the
`tracks.bin` geometry.

This first version renders **terrain only** (hypsometric colouring + hillshade).
Railway tracks and roads are not drawn yet.

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
tiled across four LOD levels that don't overlap in coverage. World coordinates are
stored relative to the start point to preserve float precision.

## Not yet implemented

Track/road rendering, LOD-seam crack stitching, and streamed/dynamic tile loading.
