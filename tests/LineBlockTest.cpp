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

// The line block between Majavatn and Svenningdal, driven without a simulator.
//
// The interlocking in main.cpp is lambdas over live state and cannot be tested at all,
// which is why the line-block rule is not there: it takes occupancy and route-existence
// as callables, so a test can put a train anywhere it likes and read what the signals do.
//
// The geometry is real - 31 km of line cut in two by the border at 586:0.425653, and the
// four signal paths authored over it. The two block signals are *synthesised here* rather
// than read from overlay/block-signals.txt: the overlay is a separate repository, and a
// test that only runs once someone has committed a file there is a test that does not run.
//
// Usage: LineBlockTest <datasetRoot>   (exit 77 = skipped, no dataset)

#include "LineBlock.h"
#include "SignalPaths.h"
#include "SwitchNetwork.h"
#include "SwitchTypes.h"
#include "TerrainData.h"
#include "TrackCircuits.h"
#include "TrackGraph.h"
#include "TrackPath.h"

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

namespace {

int failures = 0;

void check(bool ok, const char* what, long got, long want) {
    std::printf("  %-52s %5ld (want %5ld) %s\n", what, got, want, ok ? "ok" : "FAILED");
    if (!ok) ++failures;
}

void checkStr(bool ok, const char* what, const std::string& got) {
    std::printf("  %-52s %-24s %s\n", what, ('"' + got + '"').c_str(), ok ? "ok" : "FAILED");
    if (!ok) ++failures;
}

// The border the two block signals stand on, named by track and fraction rather than by
// index: the border list grows as stations are signalled and an index would not survive.
constexpr std::uint32_t kBlockTrack = 0x586;
constexpr double kBlockFrac = 0.425653;

int pathNamed(const std::vector<SignalPath>& ps, const char* name) {
    for (std::size_t i = 0; i < ps.size(); ++i)
        if (ps[i].name == name) return static_cast<int>(i);
    return -1;
}

const char* aspectName(SignalAspect a) {
    switch (a) {
        case SignalAspect::Stop: return "Stop";
        case SignalAspect::TrainOnTrack: return "TrainOnTrack";
        case SignalAspect::Clear: return "Clear";
        case SignalAspect::ClearReduced: return "ClearReduced";
        case SignalAspect::Dark: return "Dark";
    }
    return "?";
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
    const TrackCircuits circuits = loadTrackCircuits(root);
    const std::vector<SignalPath> signalPaths = loadSignalPaths(root);

    // The four roads over this line. Two are the block signals' own; two are the roads the
    // station exit signals will run over, and stand in for those departures here.
    const int northRoad = pathNamed(signalPaths, "MAJ-SGD N"); // block signal -> SGD
    const int southRoad = pathNamed(signalPaths, "SGD-MAJ S"); // block signal -> MAJ
    const int fromMaj = pathNamed(signalPaths, "MAJ-SGD S");   // MAJ -> block signal
    const int fromSgd = pathNamed(signalPaths, "SGD-MAJ N");   // SGD -> block signal
    if (northRoad < 0 || southRoad < 0 || fromMaj < 0 || fromSgd < 0) {
        std::puts("the MAJ-SGD signal paths are not in the overlay - skipping");
        return 77;
    }

    const std::vector<TrackPath> tpaths = buildTrackPaths(data);
    SwitchNetwork net;
    net.build(data, tpaths, loadSwitchSuppressions(root));
    applySwitchTypes(net, loadSwitchTypes(root));

    // Two masts on one border, one facing each way - which is what a block signal is.
    std::vector<BlockSignal> bs(2);
    bs[0] = {1, "MAJ-SGD N", {kBlockTrack, kBlockFrac}, +1, 1};
    bs[1] = {2, "MAJ-SGD S", {kBlockTrack, kBlockFrac}, -1, 1};

    std::puts("\nresolving the line block:");
    const LineBlocks lb = resolveLineBlocks(bs, signalPaths, polys, circuits);
    for (const std::string& n : lb.notes) std::printf("  note: %s\n", n.c_str());
    check(lb.lines.size() == 1, "one line block", static_cast<long>(lb.lines.size()), 1);
    if (lb.lines.size() != 1) {
        std::puts("\nFAILURES");
        return 1;
    }
    const LineBlock& L = lb.lines.front();
    check(L.sound, "and it makes sense", L.sound ? 1 : 0, 1);
    check(L.signals.size() == 2, "two signals on it", static_cast<long>(L.signals.size()), 2);
    check(L.sections.size() == 2, "two block sections",
          static_cast<long>(L.sections.size()), 2);
    check(L.ends.size() == 2, "bounded by two borders", static_cast<long>(L.ends.size()), 2);
    checkStr(L.name == "MAJ-SGD", "named after its roads", L.name);

    // Each signal takes the road leaving its border the way it faces, and the two roads are
    // different: that is what makes them the two halves of the line rather than one road
    // described twice.
    check(lb.roads[0].road == northRoad, "the + signal governs MAJ-SGD N",
          lb.roads[0].road, northRoad);
    check(lb.roads[1].road == southRoad, "the - signal governs SGD-MAJ S",
          lb.roads[1].road, southRoad);
    check(lb.roads[0].sections.size() == 1, "one block ahead of the + signal",
          static_cast<long>(lb.roads[0].sections.size()), 1);
    check(lb.roads[1].sections.size() == 1, "one block ahead of the - signal",
          static_cast<long>(lb.roads[1].sections.size()), 1);
    check(lb.roads[0].sections != lb.roads[1].sections, "and they are different blocks",
          lb.roads[0].sections == lb.roads[1].sections ? 0 : 1, 1);
    check(lb.roads[0].align == -lb.roads[1].align, "the two face opposite ways",
          lb.roads[0].align == -lb.roads[1].align ? 1 : 0, 1);

    const int secN = lb.roads[0].sections.front(); // the block beyond the + signal
    const int secS = lb.roads[1].sections.front(); // the block beyond the - signal

    // A departure out of Majavatn runs the way the + signal faces: it is running toward
    // that signal's back, over the road the - signal governs.
    std::puts("\nwhich way a movement runs:");
    const int dirMaj = lineBlockDirection(lb, 0, signalPaths[fromMaj], signalPaths);
    const int dirSgd = lineBlockDirection(lb, 0, signalPaths[fromSgd], signalPaths);
    check(dirMaj == lb.roads[0].align, "a departure from MAJ runs the + signal's way",
          dirMaj, lb.roads[0].align);
    check(dirSgd == lb.roads[1].align, "a departure from SGD runs the - signal's way",
          dirSgd, lb.roads[1].align);
    check(dirMaj == -dirSgd, "and the two oppose each other", dirMaj == -dirSgd ? 1 : 0, 1);

    // Occupancy, as the sim would supply it.
    std::vector<int> occ;
    auto occupied = [&](int id) {
        return std::find(occ.begin(), occ.end(), id) != occ.end();
    };
    bool haveRoute = false;
    auto routeInto = [&](int) { return haveRoute; };

    std::puts("\nthe direction lock:");
    std::vector<LineBlockState> state(lb.lines.size());
    std::string brief;
    checkStr(lineBlockRefusal(lb, state, signalPaths[fromMaj], signalPaths, occupied,
                              &brief).empty(),
             "a clear line refuses nothing", brief);
    claimLineBlocks(lb, state, signalPaths[fromMaj], signalPaths, "MAJ NB T1");
    check(state[0].claimed == dirMaj, "MAJ's departure claims the line",
          state[0].claimed, dirMaj);
    brief.clear();
    const std::string refused =
        lineBlockRefusal(lb, state, signalPaths[fromSgd], signalPaths, occupied, &brief);
    check(!refused.empty(), "SGD is now refused the other way", refused.empty() ? 0 : 1, 1);
    checkStr(brief == "line set", "and the picker is told why", brief);
    brief.clear();
    checkStr(lineBlockRefusal(lb, state, signalPaths[fromMaj], signalPaths, occupied,
                              &brief).empty(),
             "while a second MAJ departure is allowed", brief);

    // A train on a line nobody claimed has no direction on record, so nothing is let out.
    std::puts("\na train nothing claimed:");
    std::vector<LineBlockState> loose(lb.lines.size());
    occ = {secN};
    brief.clear();
    check(!lineBlockRefusal(lb, loose, signalPaths[fromMaj], signalPaths, occupied,
                            &brief).empty(),
          "an unclaimed line with a train on it refuses", 1, 1);
    checkStr(brief == "line occupied", "and says so", brief);
    occ.clear();

    // The sequence the whole thing exists for: two trains, one line, one behind the other.
    std::puts("\ntwo trains following each other over the line:");
    BlockOutcome out;
    auto step = [&]() { stepLineBlocks(lb, state, signalPaths, net, polys, occupied,
                                       routeInto, out); };
    auto shows = [&](int i) { return out.aspect[i]; };
    auto roadOpen = [&](int i) { return out.roadSet[lb.roads[i].road] != 0; };

    haveRoute = true; // MAJ's route is set and still held
    step();
    check(shows(0) == SignalAspect::Clear, "line claimed and empty: the + signal clears",
          shows(0) == SignalAspect::Clear, 1);
    check(roadOpen(0), "with its road opened under it", roadOpen(0) ? 1 : 0, 1);
    check(shows(1) == SignalAspect::Stop, "the - signal stays at danger",
          shows(1) == SignalAspect::Stop, 1);

    occ = {secN}; // train 1 has passed the block signal
    haveRoute = false; // and its route has been given up behind it
    step();
    check(shows(0) == SignalAspect::Stop, "train 1 past it: back to danger behind him",
          shows(0) == SignalAspect::Stop, 1);
    check(!roadOpen(0), "and its road closed", roadOpen(0) ? 1 : 0, 0);
    check(state[0].claimed == dirMaj, "but the claim outlives the route",
          state[0].claimed, dirMaj);
    brief.clear();
    check(!lineBlockRefusal(lb, state, signalPaths[fromSgd], signalPaths, occupied,
                            &brief).empty(),
          "so SGD is still refused", 1, 1);

    occ = {secS, secN}; // train 2 has left MAJ and stands at the block signal
    step();
    check(shows(0) == SignalAspect::Stop, "train 2 held at the signal behind him",
          shows(0) == SignalAspect::Stop, 1);

    occ = {secS}; // train 1 has reached Svenningdal
    step();
    check(shows(0) == SignalAspect::Clear, "train 1 clear of the block: green again",
          shows(0) == SignalAspect::Clear, 1);
    check(roadOpen(0), "road open again", roadOpen(0) ? 1 : 0, 1);
    check(state[0].claimed == dirMaj, "the line is still MAJ's", state[0].claimed, dirMaj);

    occ.clear(); // train 2 has reached Svenningdal too
    step();
    check(state[0].claimed == 0, "line clear: the claim is released", state[0].claimed, 0);
    check(shows(0) == SignalAspect::Stop, "and both signals rest at danger",
          shows(0) == SignalAspect::Stop, 1);
    brief.clear();
    checkStr(lineBlockRefusal(lb, state, signalPaths[fromSgd], signalPaths, occupied,
                              &brief).empty(),
             "SGD may now have the line", brief);

    // Requirement, not decoration: the distant on the approach has to read the block signal
    // exactly as it reads a station's. Nothing in firstMainSignalAhead knows what a block
    // signal is - it counts everything that is not a dwarf or a distant - so this asks
    // whether that silence is really enough.
    std::puts("\nwhat a distant on the approach can see:");
    std::vector<SignalPlacement> pl;
    for (std::size_t i = 0; i < bs.size(); ++i) {
        const SignalPath& road = signalPaths[lb.roads[i].road];
        SignalPlacement sp;
        if (!routeStartPose(road, polys, sp.world, sp.forward)) continue;
        sp.kind = SignalKind::Block;
        sp.at = bs[i].at;
        sp.side = bs[i].side;
        sp.paths.push_back(lb.roads[i].road);
        pl.push_back(std::move(sp));
    }
    check(pl.size() == 2, "two block masts placed", static_cast<long>(pl.size()), 2);
    const TrackJunctions junctions = trackJunctions(polys);
    // Stand a little way back along the road running up to the + signal and look ahead.
    const SectionInterval& up = signalPaths[fromMaj].parts.back();
    const int dir = up.to >= up.from ? +1 : -1;
    const double from = up.from + 0.5 * (up.to - up.from);
    const int seen = firstMainSignalAhead(polys, junctions, net, pl, up.trackId, from, dir,
                                          kDistantReach);
    check(seen == 0, "a distant on the approach reads the + block signal", seen, 0);
    if (seen >= 0)
        std::printf("    it is showing %s\n", aspectName(out.aspect[seen]));

    // With nothing authored the whole feature has to vanish, not merely behave.
    std::puts("\nwith no block signals authored:");
    const LineBlocks none = resolveLineBlocks({}, signalPaths, polys, circuits);
    check(none.empty(), "no lines", static_cast<long>(none.lines.size()), 0);
    check(none.notes.empty(), "and nothing to complain about",
          static_cast<long>(none.notes.size()), 0);
    std::vector<LineBlockState> noState;
    BlockOutcome noOut;
    check(!stepLineBlocks(none, noState, signalPaths, net, polys, occupied, routeInto,
                          noOut),
          "a step changes nothing", 0, 0);
    brief.clear();
    checkStr(lineBlockRefusal(none, noState, signalPaths[fromMaj], signalPaths, occupied,
                              &brief).empty(),
             "and refuses nothing", brief);

    std::printf("\n%s\n", failures == 0 ? "all ok" : "FAILURES");
    return failures == 0 ? 0 : 1;
}
