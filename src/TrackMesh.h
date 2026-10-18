// ebaner - a Norwegian railway simulator.
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
#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

class TerrainData;
class TrackPath;

// Vertex for the detailed railway geometry (ballast, sleepers, rails).
struct TrackVertex {
    glm::vec3 pos;      // scene-origin-relative metres (z up)
    glm::vec3 normal;   // for simple sun lighting
    glm::vec3 color;    // solid tint (rails, sleepers, ballast sides)
    glm::vec2 uv;       // ballast top only: (across 0..1, alongDist / spacing)
    float texLayer;     // >=0 -> sample uLand[texLayer]*color; <0 -> solid color
};

// Where one sleeper lies: its centre and the track's frame there. Every sleeper is the
// same concrete box, so the box is held once (TrackMesh::sleeperBox) and drawn at each of
// these, rather than built again as twenty vertices for every 0.6 m of line.
struct SleeperInstance {
    glm::vec3 pos;     // scene-relative metres, on the centreline
    glm::vec3 right;   // across the track, banked with the cant
    glm::vec3 tangent; // along the track
    glm::vec3 up;
};

// A contiguous run of indices to draw as one piece. For the sleepers it is a run of
// instances instead, drawn only when the camera is near its centroid (distance
// level-of-detail).
struct TrackDrawChunk {
    glm::vec3 centroid;      // scene-relative metres
    std::uint32_t firstIndex = 0;
    std::uint32_t indexCount = 0;
    // Bounding radius about the centroid, for testing the whole chunk against the view
    // at once. Zero means "not known", and such a chunk is always drawn.
    float radius = 0.0f;
};

// The square of the ground that roads and buildings are grouped by for drawing, so a
// view draws the squares it can see rather than the whole 20 km around it. A kilometre:
// a town is several of them and a farm one, and a hundred-odd draws a frame is nothing.
constexpr double kDrawCellMetres = 1000.0;
// Which cell a world position (x, y) falls in, as one sortable key: rows south to north,
// each west to east.
inline std::int64_t drawCell(double x, double y) {
    const auto cx = static_cast<std::int64_t>(std::floor(x / kDrawCellMetres));
    const auto cy = static_cast<std::int64_t>(std::floor(y / kDrawCellMetres));
    return cy * (std::int64_t(1) << 32) + cx;
}

// Builds a realistic railway cross-section (ballast bed, sleepers, two rails) for
// the tracks near a point.
//
// The paths cover the whole line, but the rails are expensive geometry - a few tens
// of thousands of vertices per kilometre - so only the ones within `radius` of
// `centre` are built. The network stays resident as data and streams as pictures.
//
// The mesh is the ballast and the rails, always drawn. The sleepers are a list of
// places to draw the one sleeper box, in chunks submitted only close to the camera, with
// a sleeper-stripe texture on the ballast standing in for them at distance.
class TrackMesh {
public:
    // `radius` <= 0 means every path, which is only safe for a bounded network.
    void build(const std::vector<TrackPath>& paths, const glm::vec3& centre,
               float radius);

    const std::vector<TrackVertex>& vertices() const { return vertices_; }
    const std::vector<std::uint32_t>& indices() const { return indices_; }
    // The built mesh, handed over: for when it is only being passed on, so a world's worth
    // of geometry is not held twice.
    std::vector<TrackVertex> takeVertices() { return std::move(vertices_); }
    std::vector<std::uint32_t> takeIndices() { return std::move(indices_); }
    std::uint32_t alwaysIndexCount() const { return alwaysIndexCount_; }
    // The ballast-and-rails run cut into pieces, so the ones the camera is not looking
    // at can be left out. Always drawn is not the same as always in view.
    const std::vector<TrackDrawChunk>& alwaysChunks() const { return alwaysChunks_; }
    // The sleepers, and the runs of them that are drawn or left out together
    // (firstIndex/indexCount count instances).
    const std::vector<SleeperInstance>& sleepers() const { return sleepers_; }
    std::vector<SleeperInstance> takeSleepers() { return std::move(sleepers_); }
    const std::vector<TrackDrawChunk>& sleeperChunks() const { return chunks_; }

    // The sleeper box every SleeperInstance draws, in its own frame: x across the track,
    // y along it, z up from the centreline.
    static void sleeperBox(std::vector<TrackVertex>& vertices,
                           std::vector<std::uint32_t>& indices);

private:
    std::vector<TrackVertex> vertices_;
    std::vector<std::uint32_t> indices_;
    std::uint32_t alwaysIndexCount_ = 0; // ballast + rails, drawn every frame
    std::vector<TrackDrawChunk> alwaysChunks_; // ...cut up, frustum-culled
    std::vector<SleeperInstance> sleepers_;
    std::vector<TrackDrawChunk> chunks_; // sleeper runs, distance-culled
};
