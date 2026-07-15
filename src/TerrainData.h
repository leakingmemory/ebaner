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
#include <string>
#include <vector>

// One railway track segment: a polyline in world coords (EPSG:25833, metres).
// trackId is globally unique and consistent across tiles (a segment crossing a
// tile boundary appears in full in every tile it touches).
struct TrackSegment {
    std::uint32_t trackId = 0;
    std::uint8_t trackType = 0; // 0=main line, 1=siding, 2=yard
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

// One loaded terrain tile: a 256x256 grid of float32 elevations plus the
// geometry needed to place it in the world (EPSG:25833, metres).
struct Tile {
    int lod = 0;
    int col = 0;
    int row = 0;
    double originX = 0.0;   // world easting of SW corner
    double originY = 0.0;   // world northing of SW corner
    double resolution = 0.0; // metres per pixel
    double extent = 0.0;     // tile size in metres (256 * resolution)
    std::vector<float> heights;      // 256*256, row 0 = north edge
    std::vector<std::uint8_t> landcover; // 256*256 AR50 artype, empty if absent
    std::vector<TrackSegment> tracks;    // railway segments intersecting this tile
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

    // Loads tiles within `halfWindowMetres` of the Bodo track-1 terminus.
    // Throws std::runtime_error on fatal problems (missing dataset root).
    void load(const std::string& datasetRoot, double halfWindowMetres = 20000.0);

    const std::vector<Tile>& tiles() const { return tiles_; }

    // Samples ground elevation (m) at world (x,y), preferring the finest LOD
    // tile that covers the point with valid data. Returns false if no loaded
    // tile has valid (non-NODATA) coverage there.
    bool sampleGround(double worldX, double worldY, float& elevation) const;

    // Scene origin (world coords) that all rendered geometry is relative to.
    glm::dvec3 sceneOrigin() const { return sceneOrigin_; }

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
    // Resolves startWorld_/startDir_ by parsing the Bodo tile's tracks.bin.
    void resolveStartPoint(const std::string& datasetRoot);

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

    std::vector<Tile> tiles_;
    glm::dvec3 sceneOrigin_{0.0};
    glm::dvec3 startWorld_{0.0};  // world coords of track-1 terminus
    glm::vec3 startPos_{0.0f};
    glm::vec3 startDir_{1.0f, 0.0f, 0.0f};
    float minElev_ = 0.0f;
    float maxElev_ = 1.0f;
    bool hasLandCover_ = false;
};
