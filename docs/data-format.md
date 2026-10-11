# Data format

ebaner reads a `terrainmapper` export; the format is specified in
`../terrainmapper/doc/game-export-format.md`. How to install one is in
[INSTALL.md](../INSTALL.md).

## Tiles

Heightmaps are 256×256 little-endian `float32` (`terrain.hm32`, row 0 = north), in
EPSG:25833 (UTM 33N) metres, tiled across four fully overlapping LOD levels. The world is
rendered relative to the start point to keep float precision.

Each tile may also have:

| File | |
|---|---|
| `landcover.u8` | 256×256 `uint8` AR50 `artype` codes, which the terrain is textured by. Needs terrainmapper to export with an AR50 dataset loaded |
| `tracks.bin` | railway polylines, deduplicated by `trackId`, with an OSM speed limit per vertex (used for superelevation) |
| `roads.bin` | road polylines, deduplicated by geometry, with category and number |
| `buildings.bin` | OSM building footprints with kind, roof shape, base elevation and height |
| `platforms.bin` | OSM platform footprints with base elevation and slab height |

Tracks, roads and buildings appear only when the export was made with those sources (the
national rail register, NVDB roads and OSM enrichment).

## The overlay

Everything authored on top of the export, such as track fixes, signalling and lineside
equipment, lives in `<dataset>/overlay/`, which the export never touches. The files are
listed in [The track editor](track-editor.md#saving-the-overlay). An optional
`overlay/overlay.lua` is run at startup; see [Scripting](../README.md#scripting).
