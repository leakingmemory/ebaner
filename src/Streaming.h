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

#include "TerrainMesh.h" // Vertex
#include "TrackMesh.h"   // TrackVertex, TrackDrawChunk

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <glm/glm.hpp>
#include <mutex>
#include <thread>
#include <vector>

class SwitchNetwork;
class TerrainData;
class TrackPath;

// Keeps the world built around the camera.
//
// The rail network is resident and never rebuilt - the paths, the turnouts and the
// signalling all cover the whole dataset from the start, so nothing here can invalidate
// a `TrackPath*` a vehicle is standing on. What moves is the *scenery*: the ground, the
// roads, the buildings, and the rails as geometry rather than as a route.
//
// The ground is rebuilt a tile at a time: a window move touches a ring of tiles, not the
// whole world, so only those chunks are built and uploaded. What makes that possible is
// that every terrain triangle belongs to exactly one tile, seams included.
//
// The rest - rails, roads, buildings - is still rebuilt whole and swapped in one go. It is
// the smaller half, and chunking it means the same treatment for four more meshes.
//
// Threading contract: while a build is in flight this thread owns the TerrainData
// exclusively. The render thread must read nothing from it but the values that do not
// move (the scene origin, the window radius).
class WorldStreamer {
public:
    // One tile's ground, ready to upload.
    struct Chunk {
        std::uint64_t key = 0;
        std::vector<Vertex> vertices;
        std::vector<std::uint32_t> indices;
    };

    // One finished world, ready to be handed to the renderer.
    struct Build {
        glm::vec3 centre{0.0f}; // scene-relative, what this was built around
        std::vector<Chunk> terrainChunks; // built or rebuilt
        std::vector<std::uint64_t> terrainDrop; // no longer resident
        std::vector<TrackVertex> trackV;
        std::vector<std::uint32_t> trackI;
        std::uint32_t trackAlways = 0;
        std::vector<TrackDrawChunk> trackAlwaysChunks;
        std::vector<TrackDrawChunk> sleeperChunks;
        std::vector<TrackVertex> roadV;
        std::vector<std::uint32_t> roadI;
        std::vector<TrackVertex> structV;
        std::vector<std::uint32_t> structI;
        // The buildings grouped by locality, covering the first structChunked indices;
        // whatever is merged in after them (platforms, signs, bores) is small, sits by
        // the line and is always drawn.
        std::vector<TrackDrawChunk> structChunks;
        std::uint32_t structChunked = 0;
        float minElev = 0.0f, maxElev = 1.0f;
    };

    ~WorldStreamer();

    // Everything referenced must outlive the streamer. `net` is read for switch stands
    // and is not modified.
    void start(TerrainData& data, const std::vector<TrackPath>& paths,
               const SwitchNetwork& net);
    void stop();

    // Call once a frame with the camera's scene position. Kicks a rebuild when the
    // camera has moved far enough from what is currently built and none is in flight.
    void update(const glm::vec3& camScene);

    // Non-blocking. True if a finished build was moved into `out`.
    bool take(Build& out);

    // True while the worker owns the TerrainData.
    bool building() const { return building_.load(std::memory_order_acquire); }

private:
    void run();

    TerrainData* data_ = nullptr;
    const std::vector<TrackPath>* paths_ = nullptr;
    const SwitchNetwork* net_ = nullptr;

    std::thread worker_;
    std::mutex m_;
    std::condition_variable cv_;
    bool quit_ = false;
    bool requested_ = false;
    glm::vec3 requestCentre_{0.0f};
    std::atomic<bool> building_{false};

    glm::vec3 builtCentre_{0.0f}; // what the live world was built around
    bool haveResult_ = false;
    Build result_;
};
