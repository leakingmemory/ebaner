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

// A contiguous run of sleeper-box indices, drawn only when the camera is near
// its centroid (distance level-of-detail).
struct TrackDrawChunk {
    glm::vec3 centroid;      // scene-relative metres
    std::uint32_t firstIndex = 0;
    std::uint32_t indexCount = 0;
};

// Builds a realistic railway cross-section (ballast bed, sleepers, two rails) for
// all tracks in the loaded tiles, deduped by trackId.
//
// The index buffer is laid out as [ballast + rails ("always")] followed by
// [sleeper chunks]. Ballast and rails are always drawn; sleeper boxes are only
// submitted for chunks close to the camera, with a sleeper-stripe texture on the
// ballast standing in for them at distance.
class TrackMesh {
public:
    void build(const std::vector<TrackPath>& paths);

    const std::vector<TrackVertex>& vertices() const { return vertices_; }
    const std::vector<std::uint32_t>& indices() const { return indices_; }
    std::uint32_t alwaysIndexCount() const { return alwaysIndexCount_; }
    const std::vector<TrackDrawChunk>& sleeperChunks() const { return chunks_; }

private:
    std::vector<TrackVertex> vertices_;
    std::vector<std::uint32_t> indices_;
    std::uint32_t alwaysIndexCount_ = 0; // ballast + rails, drawn every frame
    std::vector<TrackDrawChunk> chunks_; // sleeper runs, distance-culled
};
