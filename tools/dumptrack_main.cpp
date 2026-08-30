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

// ebaner-dumptrack: print track geometry *as the game sees it*, after the overlay.
//
// This exists because reading tracks.bin is a trap. The export's heights are only
// a starting point - track-edits.txt regrades them, sometimes by tens of metres,
// and a main line that reads 80 m on disk can be at 20 m once the overlay is on.
// Anything derived from the raw file (a siding regraded to match its main, a
// gradient, a comparison against the terrain) is then wrong by whatever the
// overlay moved, and wrong consistently enough to look right.
//
// So: load the dataset the way the game does and print what comes out.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

#include "TerrainData.h"
#include "TrackCircuits.h"
#include "TrackGraph.h"

namespace {

void usage() {
    std::puts(
        "usage: ebaner-dumptrack <datasetRoot> [--near <x> <y> <radius>] [<trackIdHex> ...]\n"
        "\n"
        "  --near x y radius     list the tracks whose geometry comes within radius\n"
        "                        metres of a point, nearest first\n"
        "  <trackIdHex> ...      print every vertex of those tracks, in order, as\n"
        "                        \"PT <x> <y> <z>\"\n"
        "  --route <a> <b>       ask the editor's own search for a route between two\n"
        "                        borders, each written <trackIdHex>:<frac>. Prints the\n"
        "                        count it found - 0 means the editor will refuse to\n"
        "                        build it, more than 1 means it wants a via - and the\n"
        "                        intervals when there is exactly one.\n"
        "\n"
        "Heights are after track-edits.txt is applied, which is the only form worth\n"
        "deriving anything from.");
}

double distToSeg(double px, double py, const glm::dvec3& a, const glm::dvec3& b) {
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double l2 = dx * dx + dy * dy;
    double t = 0.0;
    if (l2 > 0.0) t = std::fmax(0.0, std::fmin(1.0, ((px - a.x) * dx + (py - a.y) * dy) / l2));
    return std::hypot(px - (a.x + t * dx), py - (a.y + t * dy));
}

const char* mediumName(std::uint8_t m) {
    switch (m) {
        case 0x55: return "tunnel";
        case 0x54: return "tube";
        case 0x4C: case 0x42: return "bridge";
        default: return "surface";
    }
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) { usage(); return 2; }
    const std::string root = argv[1];
    if (!std::filesystem::exists(root)) {
        std::fprintf(stderr, "no dataset at %s\n", root.c_str());
        return 1;
    }

    std::vector<std::pair<Border, Border>> routeQs;
    bool haveNear = false;
    double nx = 0.0, ny = 0.0, nr = 0.0;
    std::vector<std::uint32_t> ids;
    for (int i = 2; i < argc; ++i) {
        if (std::strcmp(argv[i], "--near") == 0 && i + 3 < argc) {
            // The radius is required rather than optional: an optional one cannot
            // be told from a track id, since "300" is a perfectly good hex id.
            haveNear = true;
            nx = std::atof(argv[++i]);
            ny = std::atof(argv[++i]);
            nr = std::atof(argv[++i]);
        } else if (std::strcmp(argv[i], "--route") == 0 && i + 2 < argc) {
            auto parse = [](const char* t) {
                Border b;
                const char* c = std::strchr(t, ':');
                b.trackId = static_cast<std::uint32_t>(std::strtoul(t, nullptr, 16));
                b.frac = c ? std::atof(c + 1) : 0.0;
                return b;
            };
            const Border a = parse(argv[++i]);
            const Border b = parse(argv[++i]);
            routeQs.push_back({a, b});
        } else if (std::strcmp(argv[i], "-h") == 0 ||
                   std::strcmp(argv[i], "--help") == 0) {
            usage();
            return 0;
        } else {
            ids.push_back(static_cast<std::uint32_t>(std::strtoul(argv[i], nullptr, 16)));
        }
    }
    if (!haveNear && ids.empty() && routeQs.empty()) { usage(); return 2; }

    // The terrain window is small on purpose: the rail network is read whole
    // whatever ground is loaded, and this asks only about rail.
    TerrainData data;
    data.load(root, 4000.0);
    const std::vector<TrackSegment>& tracks = data.networkTracks();
    std::fprintf(stderr, "[dumptrack] %zu tracks after the overlay\n", tracks.size());

    if (haveNear) {
        std::vector<std::pair<double, const TrackSegment*>> hits;
        for (const TrackSegment& s : tracks) {
            double best = 1e30;
            for (std::size_t i = 1; i < s.pts.size(); ++i)
                best = std::fmin(best, distToSeg(nx, ny, s.pts[i - 1], s.pts[i]));
            if (s.pts.size() == 1)
                best = std::hypot(s.pts[0].x - nx, s.pts[0].y - ny);
            if (best <= nr) hits.push_back({best, &s});
        }
        std::sort(hits.begin(), hits.end(),
                  [](const auto& a, const auto& b) { return a.first < b.first; });
        std::printf("%8s %10s %5s %8s %9s  %s\n",
                    "dist", "id", "type", "verts", "medium", "z range");
        for (const auto& [d, s] : hits) {
            double z0 = 1e30, z1 = -1e30;
            for (const glm::dvec3& p : s->pts) { z0 = std::fmin(z0, p.z); z1 = std::fmax(z1, p.z); }
            std::printf("%8.2f %10x %5u %8zu %9s  %.2f .. %.2f\n",
                        d, s->trackId, s->trackType, s->pts.size(),
                        mediumName(s->medium), z0, z1);
        }
    }

    if (!routeQs.empty()) {
        // The search wants polylines in the graph's order, the way the editor builds
        // them - not the raw track list, which is why this goes through TrackGraph.
        const TrackGraph graph = buildTrackGraph(data);
        std::vector<TrackPoly> polys;
        for (std::size_t i = 0; i < graph.pointWorld.size(); ++i) {
            if (polys.empty() || polys.back().id != graph.pointTrack[i])
                polys.push_back({graph.pointTrack[i], {}});
            polys.back().pts.push_back(graph.pointWorld[i]);
        }
        for (const auto& [a, b] : routeQs) {
            std::vector<SectionInterval> r;
            const int n = findSignalRoute(polys, a, b, r, {});
            std::printf("ROUTE %x:%g -> %x:%g  found %d",
                        a.trackId, a.frac, b.trackId, b.frac, n);
            for (const SectionInterval& iv : r)
                std::printf(" %x:%g:%g", iv.trackId, iv.from, iv.to);
            std::printf("\n");
        }
    }

    for (std::uint32_t id : ids) {
        bool found = false;
        for (const TrackSegment& s : tracks) {
            if (s.trackId != id) continue;
            found = true;
            for (const glm::dvec3& p : s.pts)
                std::printf("PT %.3f %.3f %.3f\n", p.x, p.y, p.z);
        }
        if (!found) std::fprintf(stderr, "[dumptrack] no track %x\n", id);
    }
    return 0;
}
