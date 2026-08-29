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

// A route the length of a block section has to be findable.
//
// findSignalRoute walks the junction graph, and it used to give up after eight
// tracks - a bound written for the short paths inside a station, where a route
// crosses a throat and stops. A line block is not that. The plain line between
// two stations is cut wherever the medium changes, so the run from Trofors to
// Eiterstraum is seventeen tracks of tunnel, bridge and open ground for a single
// unbroken road with not one turnout on it. Under the old bound the search ran
// out of depth halfway and reported no route at all, which reads in the editor
// as "these borders cannot be connected" rather than as a limit being hit.
//
// So this walks the real thing, both ways round, and asks for exactly one route.
// The count matters as much as the success: more than one would mean the search
// had started wandering into sidings and calling them alternatives.
//
// Usage: SignalRouteTest <datasetRoot>   (exit 77 = skipped, no dataset)

#include "TerrainData.h"
#include "TrackCircuits.h"
#include "TrackGraph.h"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

namespace {

int failures = 0;

void check(bool ok, const char* what, long got, long want) {
    std::printf("  %-46s %4ld (want %4ld) %s\n", what, got, want, ok ? "ok" : "FAILED");
    if (!ok) ++failures;
}

// The outer borders of the TRO-EIT circuit, as authored in track-circuits.txt.
// Named by track and fraction rather than by index: the border list is appended
// to as stations are signalled, so an index would not survive the next session.
constexpr std::uint32_t kSouthTrack = 0x5a0;
constexpr double kSouthFrac = 0.143575;
constexpr std::uint32_t kNorthTrack = 0x5b2;
constexpr double kNorthFrac = 0.969712;

const Border* findBorder(const TrackCircuits& tc, std::uint32_t track, double frac) {
    for (const Border& b : tc.borders)
        if (b.trackId == track && std::abs(b.frac - frac) < 1e-6) return &b;
    return nullptr;
}

} // namespace

int main(int argc, char** argv) {
    const std::string root = argc > 1 ? argv[1] : "../norway-rails";
    if (!std::filesystem::exists(root)) {
        std::printf("no dataset at %s - skipping\n", root.c_str());
        return 77;
    }

    TerrainData data;
    data.load(root, 4000.0);
    const TrackGraph graph = buildTrackGraph(data);
    if (graph.pointWorld.empty()) {
        std::puts("no track points loaded - cannot test");
        return 1;
    }
    std::vector<TrackPoly> polys;
    for (std::size_t i = 0; i < graph.pointWorld.size(); ++i) {
        if (polys.empty() || polys.back().id != graph.pointTrack[i])
            polys.push_back({graph.pointTrack[i], {}});
        polys.back().pts.push_back(graph.pointWorld[i]);
    }

    const TrackCircuits tc = loadTrackCircuits(root);
    const Border* south = findBorder(tc, kSouthTrack, kSouthFrac);
    const Border* north = findBorder(tc, kNorthTrack, kNorthFrac);
    if (south == nullptr || north == nullptr) {
        // The overlay has not been signalled this far yet; nothing to check.
        std::puts("TRO-EIT outer borders not in track-circuits.txt - skipping");
        return 77;
    }

    std::puts("\nfull line block, Trofors - Eiterstraum:");
    std::vector<SectionInterval> up;
    const int nUp = findSignalRoute(polys, *south, *north, up, {});
    check(nUp == 1, "routes found southbound border -> northbound", nUp, 1);
    std::vector<SectionInterval> down;
    const int nDown = findSignalRoute(polys, *north, *south, down, {});
    check(nDown == 1, "routes found northbound border -> southbound", nDown, 1);

    // The road is the same one either way, so the two must cover the same tracks.
    if (nUp == 1 && nDown == 1) {
        check(up.size() == down.size(), "same interval count both directions",
              static_cast<long>(up.size()), static_cast<long>(down.size()));
        // Long enough to be the block and not some stub the search settled for.
        check(up.size() >= 15, "intervals spanning the block", static_cast<long>(up.size()), 15);
        double metres = 0.0;
        for (const SectionInterval& iv : up) {
            for (const TrackPoly& p : polys) {
                if (p.id != iv.trackId) continue;
                double len = 0.0;
                for (std::size_t i = 1; i < p.pts.size(); ++i)
                    len += std::hypot(p.pts[i].x - p.pts[i - 1].x, p.pts[i].y - p.pts[i - 1].y);
                metres += len * std::abs(iv.to - iv.from);
            }
        }
        check(metres > 15000.0, "route length over 15 km (m)", static_cast<long>(metres), 15000);
    }

    std::printf("\n%s\n", failures == 0 ? "all ok" : "FAILURES");
    return failures == 0 ? 0 : 1;
}
