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

#include "Streaming.h"

#include "BuildingMesh.h"
#include "PlatformMesh.h"
#include "RoadMesh.h"
#include "SpeedLimits.h"
#include "SpeedSignMesh.h"
#include "SwitchNetwork.h"
#include "TerrainData.h"
#include "TrackPath.h"
#include "TunnelMesh.h"

#include <cstdio>
#include <unordered_set>
#include <cstdlib>

namespace {
// How far the camera may drift from what is built before the world is rebuilt. Well
// inside the loaded radius, so the edge of the terrain is never in sight: at 20 km loaded
// this rebuilds after 2 km, leaving 18 km of ground beyond the camera at all times.
constexpr float kRebuildStepM = 2000.0f;
} // namespace

WorldStreamer::~WorldStreamer() { stop(); }

void WorldStreamer::start(TerrainData& data, const std::vector<TrackPath>& paths,
                          const SwitchNetwork& net) {
    data_ = &data;
    paths_ = &paths;
    net_ = &net;
    builtCentre_ = glm::vec3(0.0f); // load() built the world about the scene origin
    worker_ = std::thread([this] { run(); });
}

void WorldStreamer::stop() {
    if (!worker_.joinable()) return;
    {
        std::lock_guard<std::mutex> lk(m_);
        quit_ = true;
    }
    cv_.notify_all();
    worker_.join();
}

void WorldStreamer::update(const glm::vec3& camScene) {
    if (!worker_.joinable()) return;
    std::lock_guard<std::mutex> lk(m_);
    if (requested_ || haveResult_ || building_.load(std::memory_order_acquire)) return;
    if (glm::distance(glm::vec2(camScene), glm::vec2(builtCentre_)) < kRebuildStepM)
        return;
    requestCentre_ = camScene;
    requested_ = true;
    cv_.notify_one();
}

bool WorldStreamer::take(Build& out) {
    std::lock_guard<std::mutex> lk(m_);
    if (!haveResult_) return false;
    out = std::move(result_);
    result_ = Build{};
    haveResult_ = false;
    builtCentre_ = out.centre;
    return true;
}

void WorldStreamer::run() {
    for (;;) {
        glm::vec3 centre;
        {
            std::unique_lock<std::mutex> lk(m_);
            cv_.wait(lk, [this] { return quit_ || requested_; });
            if (quit_) return;
            requested_ = false;
            centre = requestCentre_;
            building_.store(true, std::memory_order_release);
        }

        const glm::dvec3 origin = data_->sceneOrigin();
        const float radius = data_->loadedRadius();

        // From here until `building_` clears, this thread owns the terrain.
        const TerrainData::WindowChange ch =
            data_->updateWindow(origin.x + centre.x, origin.y + centre.y);

        Build b;
        b.centre = centre;
        b.terrainDrop = ch.removed;

        // The bores first: the terrain has to know which of its triangles stand in a
        // tunnel mouth. Both depend on the tiles that just arrived.
        TunnelMesh tunnels;
        tunnels.build(*data_);

        // Which ground to rebuild: the tiles that arrived, and every resident tile whose
        // footprint touches one that arrived or left. A tile builds the seams it shares
        // with its neighbours, and builds none where a neighbour is missing - so a tile
        // appearing or vanishing leaves the ring around it holding a stale edge.
        std::unordered_set<std::uint64_t> dirty(ch.added.begin(), ch.added.end());
        for (const glm::dvec4& box : ch.touched) {
            constexpr double kAdj = 1.0; // footprints share an edge exactly
            for (const auto& [key, t] : data_->tiles()) {
                if (t->originX > box.z + kAdj || t->originX + t->extent < box.x - kAdj ||
                    t->originY > box.w + kAdj || t->originY + t->extent < box.y - kAdj)
                    continue;
                dirty.insert(key);
            }
        }

        TerrainMesh terrain;
        for (const std::uint64_t key : dirty) {
            const auto it = data_->tiles().find(key);
            if (it == data_->tiles().end()) continue;
            terrain.buildTile(*data_, *it->second, &tunnels);
            b.terrainChunks.push_back({key, terrain.vertices(), terrain.indices()});
        }

        TrackMesh tracks;
        tracks.build(*paths_, centre, radius);
        b.trackV = tracks.vertices();
        b.trackI = tracks.indices();
        b.trackAlways = tracks.alwaysIndexCount();
        b.sleeperChunks = tracks.sleeperChunks();
        b.trackAlwaysChunks = tracks.alwaysChunks();

        RoadMesh roads;
        roads.build(*data_);
        b.roadV = roads.vertices();
        b.roadI = roads.indices();

        // The static bucket: buildings, platforms, speed signs and bores share one
        // buffer, in the same order main.cpp assembles them at startup.
        BuildingMesh buildings;
        buildings.build(*data_);
        b.structV = buildings.vertices();
        b.structI = buildings.indices();
        b.structChunks = buildings.chunks();
        b.structChunked = static_cast<std::uint32_t>(b.structI.size());
        auto merge = [&](const std::vector<TrackVertex>& v,
                         const std::vector<std::uint32_t>& idx) {
            const std::uint32_t base = static_cast<std::uint32_t>(b.structV.size());
            b.structV.insert(b.structV.end(), v.begin(), v.end());
            b.structI.reserve(b.structI.size() + idx.size());
            for (const std::uint32_t i : idx) b.structI.push_back(i + base);
        };
        PlatformMesh platforms;
        platforms.build(*data_, *paths_);
        merge(platforms.vertices(), platforms.indices());
        SpeedSignMesh signs;
        signs.build(speedSigns(*paths_, centre, radius));
        merge(signs.vertices(), signs.indices());
        merge(tunnels.vertices(), tunnels.indices());

        b.minElev = data_->minElevation();
        b.maxElev = data_->maxElevation();

        {
            std::lock_guard<std::mutex> lk(m_);
            result_ = std::move(b);
            haveResult_ = true;
            building_.store(false, std::memory_order_release);
        }
    }
}
