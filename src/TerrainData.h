// ebaner - a Vulkan viewer for terrainmapper rail/terrain exports.
// Copyright (C) 2026 Jan-Espen Oversand <sigsegv@radiotube.org>
//
// This file is part of ebaner. ebaner is free software: you can redistribute it
// and/or modify it under the terms of version 3 of the GNU General Public License
// as published by the Free Software Foundation.
//
// ebaner is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
// PARTICULAR PURPOSE. See the GNU General Public License for more details. You
// should have received a copy of the license along with ebaner; if not, see
// <https://www.gnu.org/licenses/>.

#pragma once

#include <glm/glm.hpp>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

struct TrackEdit;

// One railway track segment: a polyline in world coords (EPSG:25833, metres).
// trackId is globally unique and consistent across tiles (a segment crossing a
// tile boundary appears in full in every tile it touches).
struct TrackSegment {
    std::uint32_t trackId = 0;
    std::uint8_t trackType = 0; // 0=main line, 1=siding, 2=yard
    std::uint8_t medium = 0x20; // 0x20 surface, 0x55 tunnel, 0x54 tube, 0x4C/0x42 bridge
    std::vector<glm::dvec3> pts;
    std::vector<std::uint16_t> speed; // per-vertex OSM speed km/h (0=unknown)
};

// One road segment: a polyline in world coords (EPSG:25833, metres). Roads carry
// no unique id, so a segment crossing tile boundaries (included in full in every
// tile it touches) must be deduplicated by geometry.
struct RoadSegment {
    std::uint8_t kategori = 0; // 'E','R','F','K','P' (road class)
    std::uint32_t nummer = 0;  // road number (e.g. 6 for E6)
    std::vector<glm::dvec3> pts;
};

// One OSM building: an exterior-ring footprint (world coords, EPSG:25833) with a
// ground base elevation and extrusion height. Buildings carry no id, so a
// footprint straddling tile boundaries must be deduplicated by geometry.
struct BuildingSegment {
    std::uint8_t kind = 0;      // 0=other,1=residential,2=commercial,3=industrial
    std::uint8_t roofShape = 0; // 0=flat,1=gabled,2=hipped,3=pyramidal,4=skillion
    float baseZ = 0.0f;         // ground elevation (m)
    float height = 0.0f;        // wall/eaves height (m)
    std::vector<glm::dvec2> footprint; // exterior ring, not closed
};

// One OSM station platform: an exterior-ring footprint (world coords,
// EPSG:25833) with a ground base elevation and slab height. Platforms carry no
// id, so a footprint straddling tile boundaries must be deduplicated by geometry.
struct PlatformSegment {
    float baseZ = 0.0f;  // ground elevation (m)
    float height = 0.0f; // slab height (m)
    std::vector<glm::dvec2> footprint; // exterior ring, not closed
};

// One loaded terrain tile: a 256x256 grid of float32 elevations plus the geometry
// needed to place it in the world (EPSG:25833, metres). Tiles carry the ground and
// what is scattered on it; the railway is not here but in TerrainData::networkTracks,
// which covers the whole dataset rather than the loaded window.
struct Tile {
    int lod = 0;
    int col = 0;
    int row = 0;
    double originX = 0.0;   // world easting of SW corner
    double originY = 0.0;   // world northing of SW corner
    double resolution = 0.0; // metres per pixel
    double extent = 0.0;     // tile size in metres (256 * resolution)
    std::vector<float> heights;      // 256*256, row 0 = north edge
    // The heights as loaded, before carveTrackCuttings. Kept per tile so a re-carve
    // after an edit starts from the ground rather than stacking on the last cutting.
    std::vector<float> pristine;
    std::vector<std::uint8_t> landcover; // 256*256 AR50 artype, empty if absent
    std::vector<RoadSegment> roads;      // road segments intersecting this tile
    std::vector<BuildingSegment> buildings; // buildings intersecting this tile
    std::vector<PlatformSegment> platforms; // platforms intersecting this tile
};

// Loads terrainmapper export tiles around a start location and resolves the
// camera start point from the railway track geometry.
class TerrainData {
public:
    static constexpr int PIXELS = 256;
    static constexpr float NODATA = -9999.0f;

    // Loads tiles within `halfWindowMetres` of where the railway passes `startSeed` -
    // a station node, from Stations.h. The scene origin becomes that point, so starting
    // anywhere in the dataset costs nothing; it is only travelling far from it that
    // wants more than float32 can carry.
    // Throws std::runtime_error on fatal problems (missing dataset root).
    void load(const std::string& datasetRoot, const glm::dvec3& startSeed,
              double halfWindowMetres = 20000.0);
    // Bodo, for callers that do not care where they start.
    void load(const std::string& datasetRoot, double halfWindowMetres = 20000.0);

    // Loaded tiles, keyed by (lod, col, row). Node-based and held by pointer because
    // tiles come and go while the program runs and other code holds references into
    // them across a load: a vector would rehome every tile the moment one more arrived.
    using TileMap = std::unordered_map<std::uint64_t, std::unique_ptr<Tile>>;
    static std::uint64_t tileKey(int lod, int col, int row) {
        return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(lod)) << 40) |
               (static_cast<std::uint64_t>(static_cast<std::uint32_t>(col) & 0xFFFFF) << 20) |
               (static_cast<std::uint64_t>(static_cast<std::uint32_t>(row) & 0xFFFFF));
    }
    const TileMap& tiles() const { return tiles_; }
    const Tile* tileAt(int lod, int col, int row) const {
        const auto it = tiles_.find(tileKey(lod, col, row));
        return it == tiles_.end() ? nullptr : it->second.get();
    }
    // Metres per pixel at each LOD; a tile spans PIXELS * this.
    static double lodResolution(int lod);

    // Every railway segment in the dataset, one copy per trackId (finest LOD wins),
    // in world coords. The terrain is windowed and streams, but the rail network is
    // four orders of magnitude smaller than the ground under it - the whole thing is
    // ~10 MB and under a second to read - so it is loaded once and stays. Anything
    // that has to follow the rails rather than draw them reads this and never has to
    // ask whether the track it wants is loaded: signalling and routing far ahead of
    // the train work the same as under it.
    const std::vector<TrackSegment>& networkTracks() const { return networkTracks_; }

    // What a window move did, so a caller can rebuild only what it has to.
    struct WindowChange {
        std::vector<std::uint64_t> added;
        std::vector<std::uint64_t> removed;
        // Footprints (x0, y0, x1, y1) of the tiles involved, added and removed alike.
        // A tile's ground is built with the seams it shares with its neighbours, so a
        // tile appearing or vanishing makes the ring around it stale too.
        std::vector<glm::dvec4> touched;
        bool any() const { return !added.empty() || !removed.empty(); }
    };

    // Move the loaded window to `centre` (world x,y): read whatever tiles are now in
    // range and drop whatever has fallen out of it. Newly read tiles are carved before
    // they are published.
    //
    // Not safe to call while another thread is reading the tiles.
    WindowChange updateWindow(double centreX, double centreY);

    // Apply track-edit overlays to the rail network in-session (mutating the
    // geometry), so the editor can preview edits before they are saved. The saved
    // overlay file is already applied during load().
    void applyTrackEdits(const std::vector<TrackEdit>& edits);

    // Re-carve the terrain cuttings from the pristine (pre-carve) heightfield using
    // the current (possibly edited) track geometry. Lets the editor preview how an
    // edit re-shapes the terrain without double-carving the already-carved heights.
    void recarve();

    // Samples ground elevation (m) at world (x,y), preferring the finest LOD
    // tile that covers the point with valid data. Returns false if no loaded
    // tile has valid (non-NODATA) coverage there.
    bool sampleGround(double worldX, double worldY, float& elevation) const;

    // Scene origin (world coords) that all rendered geometry is relative to.
    glm::dvec3 sceneOrigin() const { return sceneOrigin_; }

    // How far out the terrain was loaded, scene-relative. Visual geometry that would
    // otherwise follow the whole network - the rails above all - is built to this, so
    // nothing is drawn standing over ground that was never loaded.
    float loadedRadius() const { return static_cast<float>(halfWindow_); }

    // Camera start, scene-relative (metres). z is terrain/track elevation.
    glm::vec3 startPos() const { return startPos_; }

    // Horizontal look direction down the line, scene-relative & normalised.
    glm::vec3 startDir() const { return startDir_; }

    // Elevation range over loaded non-nodata samples (for the colour ramp).
    float minElevation() const { return minElev_; }
    float maxElevation() const { return maxElev_; }

    // True if any loaded tile carried land-cover data.
    bool hasLandCover() const { return hasLandCover_; }

private:
    // Resolves startWorld_/startDir_ onto the railway near `seed`. Needs the network
    // loaded, and does not care where the scene origin is (there isn't one yet).
    void resolveStartPoint(const glm::dvec3& seed);

    // Fills networkTracks_ from every tile in the dataset, finest LOD first.
    void loadNetworkTracks(const std::string& datasetRoot);

    // Reads one tile into the map. False if it has no heightfield on disk.
    bool loadTile(int lod, int col, int row);

    // The loaded tiles as a plain list, for passes that walk all of them.
    std::vector<Tile*> tileList();

    // Reads a single terrain.hm32 into `out`; returns false if unreadable.
    bool loadHeightmap(const std::string& path, std::vector<float>& out) const;

    // Reads a single landcover.u8 into `out`; returns false if unreadable.
    bool loadLandCover(const std::string& path,
                       std::vector<std::uint8_t>& out) const;

    // Parses a tracks.bin into `out` (world coords); false if unreadable/empty.
    static bool parseTracksBin(const std::string& path,
                               std::vector<TrackSegment>& out);

    // Parses a roads.bin into `out` (world coords); false if unreadable/empty.
    static bool parseRoadsBin(const std::string& path,
                              std::vector<RoadSegment>& out);

    // Parses a buildings.bin into `out` (world coords); false if unreadable/empty.
    static bool parseBuildingsBin(const std::string& path,
                                  std::vector<BuildingSegment>& out);

    // Parses a platforms.bin into `out` (world coords); false if unreadable/empty.
    static bool parsePlatformsBin(const std::string& path,
                                  std::vector<PlatformSegment>& out);

    TileMap tiles_;
    std::vector<TrackSegment> networkTracks_;
    std::string root_;          // dataset root, for reading tiles after load()
    glm::dvec2 windowCentre_{0.0};
    glm::dvec3 sceneOrigin_{0.0};
    glm::dvec3 startWorld_{0.0};  // world coords of track-1 terminus
    glm::vec3 startPos_{0.0f};
    glm::vec3 startDir_{1.0f, 0.0f, 0.0f};
    double halfWindow_ = 0.0;
    float minElev_ = 0.0f;
    float maxElev_ = 1.0f;
    bool hasLandCover_ = false;
};
