# The world and its rendering

How ebaner builds the world from the dataset and draws it. The renderer is Vulkan, and
the simulator and the track editor share it (`libebaner_engine`).

The world comes from a `terrainmapper` export: elevation tiles, land cover, the railway,
roads, buildings and platforms (see [Data format](data-format.md)). It is loaded around
the chosen station and **streams** as the camera moves, for 20 km around it.

## Terrain

- Heightmap tiles, hillshaded and coloured by **AR50 land cover** (forest, farmland, open
  land, bog, glacier, water, built-up …) where the export has it, or by an elevation ramp
  where it doesn't. The land types are procedural textures in a texture array, sampled in
  world space (`src/Textures.cpp`, `shaders/terrain.frag`).
- The export's overlapping LOD levels are reduced to the finest tile for each area and
  **stitched** so there are no cracks between resolutions (`src/TerrainMesh.cpp`).
- **Cuttings.** Where a surface railway runs below the ground, the terrain is cut into a
  trench for it: a flat floor at the ballast base and sides sloping up at about 20° to
  the original ground. Tunnels and bridges are left alone, so a trench ends at a tunnel
  mouth in a vertical portal. The cut is applied to the height grid at load
  (`src/TerrainCarve.cpp`). `EBANER_NOCARVE=1` turns it off.

## Track

- A ballast bed, concrete sleepers and two rails (`src/TrackMesh.cpp`). Sleepers are
  modelled near the camera and a texture further away.
- The export splits a line wherever it changes between surface, tunnel and bridge.
  `buildTrackPaths` (`src/TrackPath.cpp`) joins the pieces that meet end to end into
  continuous paths, so trains run straight through tunnel portals.
- Centrelines are smoothed through the surveyed points with a **centripetal Catmull-Rom**
  spline.
- Curves are **superelevated** from the OSM speed limit and the curvature.

## Roads, buildings and platforms

- **Roads** are asphalt ribbons coloured by category: the public network (European,
  national, county and municipal roads) prominent, private tracks thin and faint
  (`src/RoadMesh.cpp`).
- **Buildings** are OSM footprints extruded and coloured by type, with flat, gabled,
  hipped, pyramidal or skillion roofs from the OSM `roof_shape` tag
  (`src/BuildingMesh.cpp`).
- **Platforms** are OSM platform footprints built as slabs with their top 0.76 m above the
  nearest **rail head**, so a long platform stays level with the track. The surface is
  asphalt with light concrete edge slabs and a yellow safety line along the edges facing
  the track (`src/PlatformMesh.cpp`).

## Switches

A switch is detected wherever one track's end joins another track's line
(`src/SwitchNetwork.cpp`). Each has a Norwegian hand switch stand
(`src/SwitchMesh.cpp`): a weighted lever with a cylindrical counterweight, a throw rod to
the blades, and a post carrying the **indicator target**, a two-faced plate.

| Switch | What a driver sees |
|---|---|
| **Straight** | the plate edge-on from both ends: a vertical line |
| **Diverging** | turned 90°. From the toe: the **arrow** (a facing train will be diverted). From the branch: the **filled circle** (a train may run from the siding onto the main) |
| **Broken** | part-turned, with the lever centred |

Switches route the trains:

- `T` with the crosshair on a stand throws it. Motor switches are also thrown from the
  traffic manager and by setting routes.
- A train entering a switch **facing** takes the road it is set for. A **broken** switch
  derails it.
- A train entering **trailing** with the switch set against it forces it through and
  leaves it **broken**. Throwing it again repairs it.

## What is drawn

Each frame draws only what is in the view frustum (`Frustum.h`). The ground, the roads,
the buildings and the track are split into pieces with a bounding sphere each: a terrain
tile, the roads or the buildings of one square kilometre, a 200 m length of track. The
bounds are measured from the vertices actually emitted. Pieces are drawn in the order they
arrive, because which of two surfaces at the same depth wins depends on that order: the
private roads go first so the public network lands on top.

Buildings are also drawn only **out to 4 km**. Further away a house is a pixel or less,
and a town in view is tens of thousands of such triangles.

Vehicles are culled per unit: a unit outside the frustum has its part of the vertex
buffer filled with one repeated point, so its triangles are degenerate while the buffer
and its indices stay valid.

## The vehicle mesh

A train's mesh is rebuilt every frame as it moves. What can't change between frames is
done once, when the train is attached (`attachVehicle`):

- the **index buffer**;
- which triangles are **glass**, drawn after the opaque ones.

So a change of a train's make-up has to go through `attachVehicle`. `refresh()` must
produce exactly what `build()` would; it is checked bit for bit for every vehicle in the
table, and warns if a refresh ever emits a different vertex count. The updated vertices
are swapped in after the frame fence, not copied.

## How a crossing finds the axles

A level crossing needs to know which of its tracks each axle is on. Every axle's position
comes with its path and its arc position along that path (`AxlePlace { pos, path, s }`),
so a crossing answers by comparing positions along the same path. Where a track belongs to
more than one path, it falls back to searching along the approach. The turnouts between
each path and its crossings are listed once at load.

## Profiling

| Variable | |
|---|---|
| `EBANER_PROFILE=1` | print per-frame timings once a second: the frame, the sim step, occupancy, crossings, signal aspects, distant walks, the signal mesh, the vehicle mesh and its upload |
| `EBANER_ROLL=<m/s>` | start the train at that speed with the brake released, so it coasts. Profile a moving train: a standing one changes no occupancy, so nothing downstream of it runs |
| `EBANER_RADIUS=<m>` | load only this half-width of the world (default 20000), for a quick start |
| `EBANER_SCREENSHOT=/dev/null EBANER_SHOTFRAME=900` | exit by itself after 900 frames with the reports flushed |
