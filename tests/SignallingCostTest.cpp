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

// What it costs to decide what every signal is showing.
//
// This runs once per track-circuit transition, so it lands inside a frame and the driver
// sees it. It used to derive, from scratch and for all 137 signal paths, which turnouts each
// route crosses - a question answered entirely by the geometry and therefore the same every
// time. Each derivation is a linear scan of all 6100 tracks per turnout per leg, so the pass
// ran tens of milliseconds and a train entering a circuit visibly stalled the picture.
//
// Two things are checked here. That the cache still agrees with the function it replaced -
// exactly, for every route, since a cache that has quietly drifted from the geometry would
// set the wrong roads. And that the pass stays cheap, with a budget generous enough to be
// about catching a return to walking the whole network rather than about micro-timing.
//
// Usage: SignallingCostTest <datasetRoot>   (exit 77 = skipped, no dataset)

#include "SignalPaths.h"
#include "SwitchNetwork.h"
#include "SwitchTypes.h"
#include "TerrainData.h"
#include "TrackCircuits.h"
#include "TrackGraph.h"
#include "TrackPath.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

namespace {

int failures = 0;

void check(bool ok, const char* what, double got, double want) {
    std::printf("  %-50s %9.3f (want %8.3f) %s\n", what, got, want, ok ? "ok" : "FAILED");
    if (!ok) ++failures;
}

double msSince(std::chrono::steady_clock::time_point t0) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0)
        .count();
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
    const std::vector<TrackPath> tpaths = buildTrackPaths(data);
    const TrackGraph graph = buildTrackGraph(data);
    if (tpaths.empty() || graph.pointWorld.empty()) {
        std::puts("no track geometry - cannot test");
        return 1;
    }
    std::vector<TrackPoly> polys;
    for (std::size_t i = 0; i < graph.pointWorld.size(); ++i) {
        if (polys.empty() || polys.back().id != graph.pointTrack[i])
            polys.push_back({graph.pointTrack[i], {}});
        polys.back().pts.push_back(graph.pointWorld[i]);
    }
    SwitchNetwork net;
    net.build(data, tpaths, loadSwitchSuppressions(root));
    applySwitchTypes(net, loadSwitchTypes(root));

    const TrackCircuits circuits = loadTrackCircuits(root);
    const std::vector<SignalPath> signalPaths = loadSignalPaths(root);
    const std::vector<SignalPath> exitSignals = loadExitSignals(root);
    const std::vector<SignalPath> entrySignals = loadEntrySignals(root);
    if (signalPaths.empty()) {
        std::puts("no signal paths authored - nothing to measure");
        return 77;
    }

    std::printf("\n%zu mini path(s), %zu turnout(s), %zu track(s), %zu section(s)\n",
                signalPaths.size(), net.turnouts().size(), polys.size(),
                circuits.sections.size());

    // The placements, built as the viewer builds them.
    std::vector<SignalPlacement> mains = signalPlacements(exitSignals, polys, SignalKind::Exit);
    {
        const std::vector<SignalPlacement> e =
            signalPlacements(entrySignals, polys, SignalKind::Entry);
        mains.insert(mains.end(), e.begin(), e.end());
    }
    std::vector<SignalPlacement> placements =
        mergeSignals(signalPlacements(signalPaths, polys), mains);
    // The free-standing distants, appended after the merge as the viewer appends them.
    // Without these there is nothing for the walk below to walk, and the measurement would
    // say the distant pass is free when it is the one thing here that cannot be cached.
    const std::vector<DistantSignal> distants = loadDistantSignals(root);
    for (std::size_t i = 0; i < distants.size(); ++i) {
        const DistantSignal& d = distants[i];
        const glm::dvec3 w = fracToWorld(polys, d.trackId, d.frac);
        if (w.x == 0.0 && w.y == 0.0) continue;
        SignalPlacement sp;
        sp.kind = SignalKind::Distant;
        sp.world = w;
        sp.forward = trackTangent(polys, d.trackId, d.frac, d.dir);
        sp.at = {d.trackId, d.frac};
        sp.side = d.side;
        sp.paths.push_back(static_cast<int>(i));
        placements.push_back(std::move(sp));
    }
    const TrackJunctions junctions = trackJunctions(polys);

    std::puts("\nworking out what cannot change:");
    const auto tCache = std::chrono::steady_clock::now();
    const RouteStatics statics = routeStatics(signalPaths, net, polys, circuits);
    const double cacheMs = msSince(tCache);
    check(statics.size() == signalPaths.size(), "one entry per path",
          static_cast<double>(statics.size()), static_cast<double>(signalPaths.size()));
    std::printf("    took %.1f ms, once, at load\n", cacheMs);

    // The cache must say exactly what the function it replaced says. It is only valid
    // because geometry cannot change while the viewer runs - the editor is another matter -
    // so this is the assertion that stands in for that argument.
    std::puts("\nand it agrees with deriving it live:");
    {
        int wrong = 0;
        std::size_t reqs = 0;
        for (std::size_t i = 0; i < signalPaths.size(); ++i) {
            const std::vector<PathSwitch> live =
                pathSwitchRequirements(signalPaths[i], net, polys);
            reqs += live.size();
            if (live.size() != statics.switches[i].size()) { ++wrong; continue; }
            for (std::size_t k = 0; k < live.size(); ++k)
                if (live[k].turnout != statics.switches[i][k].turnout ||
                    live[k].need != statics.switches[i][k].need) {
                    ++wrong;
                    break;
                }
        }
        check(wrong == 0, "paths whose cached requirements differ", wrong, 0);
        std::printf("    %zu requirement(s) over %zu path(s)\n", reqs, signalPaths.size());
    }
    // Likewise the circuits: cached as indices, so reading occupancy is a subscript.
    {
        int wrong = 0;
        for (std::size_t i = 0; i < signalPaths.size(); ++i) {
            const std::vector<int> ids = pathSections(signalPaths[i], circuits);
            if (ids.size() != statics.sections[i].size()) { ++wrong; continue; }
            for (const int si : statics.sections[i]) {
                if (si < 0 || si >= static_cast<int>(circuits.sections.size())) { ++wrong; break; }
                const int id = circuits.sections[si].id;
                if (std::find(ids.begin(), ids.end(), id) == ids.end()) { ++wrong; break; }
            }
        }
        check(wrong == 0, "paths whose cached circuits differ", wrong, 0);
    }

    // What the pass used to cost: deriving every route's requirements from scratch, which
    // is what updateSignalAspects did for each of the paths it looks at.
    std::puts("\nwhat one pass costs:");
    {
        const auto t0 = std::chrono::steady_clock::now();
        std::size_t sink = 0;
        for (const SignalPath& p : signalPaths)
            sink += pathSwitchRequirements(p, net, polys).size();
        const double beforeMs = msSince(t0);
        std::printf("    deriving every path's turnouts from scratch: %7.2f ms  (%zu)\n",
                    beforeMs, sink);
    }

    const std::vector<char> secOccupied(circuits.sections.size(), 0);
    const std::vector<char> routeSet(signalPaths.size(), 0);
    const std::vector<SignalAspect> exitAspects(placements.size(), SignalAspect::Stop);
    double aspectMs = 0.0;
    {
        // Twice, and the second is the one reported: the first walks cold caches.
        for (int rep = 0; rep < 2; ++rep) {
            const auto t0 = std::chrono::steady_clock::now();
            updateSignalAspects(placements, statics, net, secOccupied, routeSet, exitAspects);
            aspectMs = msSince(t0);
        }
        std::printf("    signal aspects, from the cache:              %7.2f ms\n", aspectMs);
    }
    double distantMs = 0.0;
    {
        const auto t0 = std::chrono::steady_clock::now();
        updateDistantAspects(placements, polys, junctions, net);
        distantMs = msSince(t0);
        std::printf("    distant walks:                               %7.2f ms\n", distantMs);
    }
    std::printf("    %zu placement(s), %zu of them distants\n", placements.size(),
                distants.size());

    // Generous on purpose. This is not about a millisecond either way - it is about
    // catching a return to deriving the geometry every pass, which was two orders of
    // magnitude worse and put the cost inside the frame the driver was looking at.
    check(aspectMs < 5.0, "signal aspects within budget (ms)", aspectMs, 5.0);
    check(distantMs < 25.0, "distant walks within budget (ms)", distantMs, 25.0);

    std::printf("\n%s\n", failures == 0 ? "all ok" : "FAILURES");
    return failures == 0 ? 0 : 1;
}
