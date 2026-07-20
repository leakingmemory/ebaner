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

#include "TrackOverlay.h"

#include "TerrainData.h"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

namespace fs = std::filesystem;

namespace {
constexpr double kSnapTol = 8.0;             // m; snap an edit end to a real endpoint
std::uint32_t kConnectorIdBase = 0xF0000000; // synthetic connector trackIds

std::string overlayFile(const std::string& root) {
    return root + "/overlay/track-edits.txt";
}

// Nearest track-segment endpoint to a world point across all tiles; returns the
// endpoint, its track type, and the owning tile index, or false if none within tol.
bool nearestEndpoint(const std::vector<Tile>& tiles, const glm::dvec3& q, double tol,
                     glm::dvec3& out, std::uint8_t& trackType, int& tileIdx) {
    double best = tol;
    bool found = false;
    for (int ti = 0; ti < static_cast<int>(tiles.size()); ++ti) {
        for (const TrackSegment& s : tiles[ti].tracks) {
            if (s.pts.size() < 2) continue;
            for (const glm::dvec3& e : {s.pts.front(), s.pts.back()}) {
                const double d = std::hypot(e.x - q.x, e.y - q.y);
                if (d < best) {
                    best = d;
                    out = e;
                    trackType = s.trackType;
                    tileIdx = ti;
                    found = true;
                }
            }
        }
    }
    return found;
}
} // namespace

std::vector<TrackEdit> loadTrackOverlay(const std::string& datasetRoot) {
    std::vector<TrackEdit> edits;
    std::ifstream f(overlayFile(datasetRoot));
    if (!f) return edits;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        char kind[32] = {};
        TrackEdit e;
        if (std::sscanf(line.c_str(), "%31s %lf %lf %lf %lf %lf %lf", kind, &e.a.x,
                        &e.a.y, &e.a.z, &e.b.x, &e.b.y, &e.b.z) == 7 &&
            std::string(kind) == "link")
            edits.push_back(e);
    }
    return edits;
}

void applyTrackOverlay(std::vector<Tile>& tiles, const std::vector<TrackEdit>& edits) {
    if (tiles.empty() || edits.empty()) return;
    int applied = 0;
    for (std::size_t i = 0; i < edits.size(); ++i) {
        glm::dvec3 ea, eb;
        std::uint8_t ta = 0, tb = 0;
        int tia = 0, tib = 0;
        if (!nearestEndpoint(tiles, edits[i].a, kSnapTol, ea, ta, tia)) continue;
        if (!nearestEndpoint(tiles, edits[i].b, kSnapTol, eb, tb, tib)) continue;
        if (std::hypot(ea.x - eb.x, ea.y - eb.y) < 1.0) continue; // same point

        // A synthetic connector segment joining the two ends. medium = tunnel so the
        // terrain carve leaves it (the gap is typically inside a mountain anyway),
        // and a unique high trackId so it survives buildTrackPaths' dedup.
        TrackSegment c;
        c.trackId = kConnectorIdBase + static_cast<std::uint32_t>(i);
        c.trackType = ta;      // both ends share type (else the join won't chain)
        c.medium = 0x55;       // tunnel
        c.pts = {ea, eb};
        tiles[tia].tracks.push_back(std::move(c));
        ++applied;
    }
    if (applied > 0)
        std::printf("[TrackOverlay] applied %d/%zu link edit(s)\n", applied,
                    edits.size());
}

bool appendTrackEdit(const std::string& datasetRoot, const TrackEdit& edit) {
    std::error_code ec;
    fs::create_directories(datasetRoot + "/overlay", ec);
    std::ofstream f(overlayFile(datasetRoot), std::ios::app);
    if (!f) return false;
    f << "link " << edit.a.x << ' ' << edit.a.y << ' ' << edit.a.z << ' ' << edit.b.x
      << ' ' << edit.b.y << ' ' << edit.b.z << '\n';
    return static_cast<bool>(f);
}
