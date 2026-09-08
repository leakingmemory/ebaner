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

// Track circuits, decided by where a train is rather than by how near it looks.
//
// A section is a stretch of one road and a train is a stretch of one road, so occupancy is
// an overlap and nothing else - no tolerance, no sampling, and no direction. This drives
// that against the real Majavatn-Svenningdal geometry, at the places the old geometric
// version got wrong: the border between two blocks, the mast of the block signal half a
// metre short of it, and a siding beside its main where the two run close enough that a
// lateral tolerance could not tell them apart.
//
// Trains are represented here by their spans directly rather than by building a Consist:
// what is under test is the rule, and a span is exactly what a vehicle will hand it.
//
// Usage: OccupancyTest <datasetRoot>   (exit 77 = skipped, no dataset)

#include "Occupancy.h"
#include "TerrainData.h"
#include "TrackCircuits.h"
#include "TrackPath.h"

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

namespace {

int failures = 0;

void check(bool ok, const char* what, long got, long want) {
    std::printf("  %-54s %5ld (want %4ld) %s\n", what, got, want, ok ? "ok" : "FAILED");
    if (!ok) ++failures;
}

void checkStr(bool ok, const char* what, const std::string& got) {
    std::printf("  %-54s %-18s %s\n", what, ('"' + got + '"').c_str(), ok ? "ok" : "FAILED");
    if (!ok) ++failures;
}

// The border the two Majavatn-Svenningdal blocks meet at, and the two sections either side.
constexpr std::uint32_t kBlockTrack = 0x586;
constexpr double kBlockFrac = 0.425653;

int sectionNamed(const TrackCircuits& tc, const char* name) {
    for (std::size_t i = 0; i < tc.sections.size(); ++i)
        if (tc.sections[i].name == name) return static_cast<int>(i);
    return -1;
}

// Every section the given spans hold, by name, for a legible failure.
std::string held(const TrackCircuits& tc, const std::vector<char>& occ) {
    std::string s;
    for (std::size_t i = 0; i < occ.size() && i < tc.sections.size(); ++i)
        if (occ[i]) s += (s.empty() ? "" : ", ") + tc.sections[i].name;
    return s.empty() ? "(none)" : s;
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
    const std::vector<TrackPath> paths = buildTrackPaths(data);
    const TrackCircuits circuits = loadTrackCircuits(root);
    if (paths.empty() || circuits.sections.empty()) {
        std::puts("no geometry or no circuits - cannot test");
        return 1;
    }

    std::puts("\nresolving every section onto the roads:");
    const SectionSpans secs = resolveSectionSpans(circuits, paths);
    for (const std::string& n : secs.notes) std::printf("    note: %s\n", n.c_str());
    long intervals = 0;
    for (const Section& s : circuits.sections) intervals += static_cast<long>(s.parts.size());
    check(secs.sectionCount == static_cast<int>(circuits.sections.size()),
          "every section accounted for", secs.sectionCount,
          static_cast<long>(circuits.sections.size()));
    check(static_cast<long>(secs.entries.size()) + static_cast<long>(secs.notes.size()) ==
              intervals,
          "every interval either placed or reported",
          static_cast<long>(secs.entries.size() + secs.notes.size()), intervals);
    // Three intervals name tracks the export no longer has. That number is asserted rather
    // than tolerated: it going up means the overlay has drifted from the export again, and
    // a section with a hole in it releases routes it should be holding.
    check(static_cast<long>(secs.notes.size()) == 3, "intervals that would not resolve",
          static_cast<long>(secs.notes.size()), 3);
    std::printf("    %zu interval(s) placed over %d section(s)\n", secs.entries.size(),
                secs.sectionCount);

    // Every section's span must be as long as the interval it came from. This is what
    // catches a fraction placed on the wrong part of a path: a wrong answer is a whole
    // track or a whole span out, and the length gives it away at once.
    std::puts("\nand each of them is the length the overlay says it is:");
    {
        double worst = 0.0;
        int worstSection = -1;
        for (const SectionSpans::Entry& e : secs.entries) {
            const Section& sec = circuits.sections[e.section];
            // Which interval this entry came from: match on the path's own extent.
            double want = 0.0;
            for (const SectionInterval& iv : sec.parts) {
                const TrackPath& p = paths[e.span.pathIdx];
                float a = 0.0f, b = 0.0f;
                if (!p.fracToS(iv.trackId, iv.from, a) || !p.fracToS(iv.trackId, iv.to, b))
                    continue;
                if (std::abs(std::min(a, b) - e.span.s0) > 1e-3) continue;
                // The overlay's own measure of the same stretch.
                want = 0.0;
                for (const TrackPath::TrackRun& r : p.trackRuns())
                    if (r.trackId == iv.trackId)
                        want = std::abs(static_cast<double>(b) - a);
                break;
            }
            const double got = e.span.s1 - e.span.s0;
            const double d = std::abs(got - want);
            if (d > worst) { worst = d; worstSection = e.section; }
        }
        check(worst < 0.01, "worst length disagreement (mm)",
              static_cast<long>(worst * 1000.0), 10);
        if (worstSection >= 0 && worst >= 0.01)
            std::printf("    worst was %s\n", circuits.sections[worstSection].name.c_str());
    }

    // The two blocks either side of the border, and where that border is on the path.
    const int secS = sectionNamed(circuits, "MAJ-SGD S");
    const int secN = sectionNamed(circuits, "MAJ-SGD N");
    if (secS < 0 || secN < 0) {
        std::puts("\nthe MAJ-SGD blocks are not in the overlay - skipping the rest");
        return failures == 0 ? 77 : 1;
    }
    int blockPath = -1;
    float blockS = 0.0f;
    for (std::size_t pi = 0; pi < paths.size() && blockPath < 0; ++pi)
        if (paths[pi].fracToS(kBlockTrack, kBlockFrac, blockS))
            blockPath = static_cast<int>(pi);
    check(blockPath >= 0, "the block border placed on a path", blockPath >= 0 ? 1 : 0, 1);
    if (blockPath < 0) {
        std::puts("\nFAILURES");
        return 1;
    }

    std::vector<char> occ;
    // A train as the vehicle will hand it over: one stretch of one road.
    auto atBorder = [&](double from, double to) {
        return std::vector<PathSpan>{
            {blockPath, blockS + static_cast<float>(from), blockS + static_cast<float>(to)}};
    };

    std::puts("\na train either side of the border between two blocks:");
    // Which of the two blocks lies which way along the path - the path's arc length has its
    // own direction and it is not the timetable's.
    computeOccupancy(secs, atBorder(20.0, 60.0), occ);
    const bool aheadIsN = occ[secN] != 0;
    const int ahead = aheadIsN ? secN : secS;
    const int behind = aheadIsN ? secS : secN;
    checkStr(occ[ahead] && !occ[behind], "40 m clear of it: one block only",
             held(circuits, occ));
    computeOccupancy(secs, atBorder(-60.0, -20.0), occ);
    checkStr(occ[behind] && !occ[ahead], "40 m the other side: the other block only",
             held(circuits, occ));
    computeOccupancy(secs, atBorder(-20.0, 20.0), occ);
    checkStr(occ[ahead] && occ[behind], "straddling it: both, never neither",
             held(circuits, occ));
    // A train exactly on a border is in the sections either side. The alternative - in
    // neither for an instant - is what releases a route out from under a train.
    computeOccupancy(secs, atBorder(0.0, 0.0), occ);
    checkStr(occ[ahead] && occ[behind], "standing exactly on it: both", held(circuits, occ));

    // The fault that started all of this. A block signal stands half a metre before the
    // border it protects, so a driver pulling up to it has his leading axle just short of
    // the border. Under the old lateral tolerance that put him 2.5 m *into* the block
    // beyond, which was his own signal's section - and it dropped to danger in his face.
    std::puts("\na train drawn up at the block signal, half a metre short of the border:");
    computeOccupancy(secs, atBorder(-80.0, -0.5), occ);
    checkStr(occ[behind] && !occ[ahead], "in the block behind it and not the one ahead",
             held(circuits, occ));

    // Nothing here reads a speed or a heading, so a train at a stand is not a special case
    // and cannot become one. Absence cannot be asserted directly; what can be asserted is
    // that the answer depends on nothing but which rails are covered - not on how many
    // pieces they arrive in, nor in what order.
    std::puts("\nthe same rails, described differently:");
    {
        std::vector<char> once, twice, shuffled;
        computeOccupancy(secs, {{blockPath, blockS - 30.0f, blockS + 10.0f}}, once);
        computeOccupancy(secs,
                         {{blockPath, blockS - 30.0f, blockS - 5.0f},
                          {blockPath, blockS - 5.0f, blockS + 10.0f}},
                         twice);
        computeOccupancy(secs,
                         {{blockPath, blockS - 5.0f, blockS + 10.0f},
                          {blockPath, blockS - 30.0f, blockS - 5.0f}},
                         shuffled);
        check(once == twice, "one span or two covering the same rails", once == twice, 1);
        check(twice == shuffled, "and in either order", twice == shuffled, 1);
        // A train of no length at all - which is what a stationary reading degenerates to
        // if the ends ever coincide - still senses, and senses both sides of the joint.
        std::vector<char> point;
        computeOccupancy(secs, {{blockPath, blockS, blockS}}, point);
        checkStr(point[ahead] && point[behind], "a train of no length still holds both",
                 held(circuits, point));
    }

    // A split changes nothing, because nothing is carried across it: occupancy is derived
    // from where each vehicle is, and parting a train moves no vehicle.
    std::puts("\nparting a train that straddles the border:");
    {
        std::vector<char> whole, parted;
        computeOccupancy(secs, atBorder(-60.0, 60.0), whole);
        // The same rails, described as two trains rather than one - which is exactly what
        // uncoupling produces, since each Vehicle keeps its own path and arc length.
        computeOccupancy(secs,
                         {{blockPath, blockS - 60.0f, blockS - 0.3f},
                          {blockPath, blockS + 0.3f, blockS + 60.0f}},
                         parted);
        check(whole == parted, "the same sections held, before and after",
              whole == parted ? 1 : 0, 1);
        checkStr(whole[ahead] && whole[behind], "and it is both blocks", held(circuits, whole));
    }

    // A siding beside its main. At Svenningdal the loop is 6.1 m from the main at its widest
    // and converges to nothing at the turnouts, so no lateral tolerance can separate them -
    // but they are different roads, and a vehicle knows which one it is on.
    const int sgdT1 = sectionNamed(circuits, "SGD T1");
    const int sgdT2 = sectionNamed(circuits, "SGD T2");
    if (sgdT1 >= 0 && sgdT2 >= 0) {
        std::puts("\na train on the Svenningdal loop, alongside the main:");
        // Somewhere in the middle of the loop's own section.
        PathSpan onLoop;
        for (const SectionSpans::Entry& e : secs.entries)
            if (e.section == sgdT2 && e.span.s1 - e.span.s0 > 100.0f) {
                const float mid = 0.5f * (e.span.s0 + e.span.s1);
                onLoop = {e.span.pathIdx, mid - 40.0f, mid + 40.0f};
            }
        check(onLoop.pathIdx >= 0, "found a stretch of the loop to stand on",
              onLoop.pathIdx >= 0 ? 1 : 0, 1);
        if (onLoop.pathIdx >= 0) {
            computeOccupancy(secs, {onLoop}, occ);
            checkStr(occ[sgdT2] && !occ[sgdT1], "holds the loop and not the main",
                     held(circuits, occ));
        }
    }

    // The successor to the old "no blind spots" check. There is no sampling now and so no
    // way for a section to be blind by construction - but the property is worth asserting
    // directly rather than by argument, because it is the one whose failure lost a train
    // out on the line: the circuit read clear, the line block's claim was released behind
    // him, and the signal he was running toward went back to danger.
    std::puts("\na train anywhere inside a section registers in it:");
    {
        int missed = 0;
        long placements = 0;
        std::string worstWhere;
        for (const SectionSpans::Entry& e : secs.entries) {
            const float len = e.span.s1 - e.span.s0;
            if (len < 1.0f) continue; // a sliver at a throat; nothing stands wholly in it
            // A short train stepped along the whole length of the section.
            for (int k = 0; k <= 20; ++k) {
                const float at = e.span.s0 + len * static_cast<float>(k) / 20.0f;
                const float half = std::min(5.0f, 0.4f * len);
                computeOccupancy(secs, {{e.span.pathIdx, at - half, at + half}}, occ);
                ++placements;
                if (!occ[e.section]) {
                    ++missed;
                    worstWhere = circuits.sections[e.section].name;
                }
            }
        }
        std::printf("    %ld placement(s) over %zu stretch(es) of section\n", placements,
                    secs.entries.size());
        check(missed == 0, "placements the section failed to sense", missed, 0);
        if (missed > 0) std::printf("    e.g. in \"%s\"\n", worstWhere.c_str());
    }

    std::puts("\nwith no trains at all:");
    computeOccupancy(secs, {}, occ);
    {
        int anyHeld = 0;
        for (const char c : occ) anyHeld += c ? 1 : 0;
        check(anyHeld == 0, "nothing is occupied", anyHeld, 0);
        check(static_cast<long>(occ.size()) == secs.sectionCount, "and every section answers",
              static_cast<long>(occ.size()), secs.sectionCount);
    }

    std::printf("\n%s\n", failures == 0 ? "all ok" : "FAILURES");
    return failures == 0 ? 0 : 1;
}
