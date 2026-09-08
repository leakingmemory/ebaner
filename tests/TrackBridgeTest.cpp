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

// The bridge between the two coordinate systems this program keeps.
//
// Everything authored in the overlay - borders, section intervals, signal anchors - is
// written against one exported track and a fraction along it, where that fraction is planar
// chord length over the track's own polyline. A train runs in arc length along a TrackPath,
// which is a centripetal spline in three dimensions chained through many tracks head to
// tail. The two are different metrics over different extents, so `s = frac * length` is
// wrong twice over and drifts by metres over a kilometre; every other consumer in the tree
// bridges the gap by proximity instead, with tolerances of 2.5 m to 25 m.
//
// TrackPath::fracToS and ::trackAt convert between them. Some of the checks below are exact
// and some are not, and which is which is the whole point:
//
//   * At a surveyed point the polyline and the spline are built from the same coordinate
//     and both pass through it, so they must agree to nothing at all. Any disagreement
//     there is the bridge's own error. That is the test that proves it.
//   * Between two surveyed points they genuinely differ: the overlay interpolates along the
//     chord, the rails follow the spline, and on a curve with points 50 m apart that is
//     about a metre at mid-span. Neither is wrong, and the spline is the one a train runs
//     on, so the assertion is relative - the bridge must add nothing beyond it.
//   * At the seam between two chained tracks the shared point is dropped as a duplicate, so
//     a track's own end is represented by its neighbour's, up to the metre within which the
//     chaining joins ends. There is no rail at the dropped position at all.
//
// Usage: TrackBridgeTest <datasetRoot>   (exit 77 = skipped, no dataset)

#include "TerrainData.h"
#include "TrackCircuits.h"
#include "TrackGraph.h"
#include "TrackPath.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

int failures = 0;

void check(bool ok, const char* what, double got, double want) {
    std::printf("  %-52s %9.3f (want %8.3f) %s\n", what, got, want, ok ? "ok" : "FAILED");
    if (!ok) ++failures;
}

// How near two track ends must be for the chaining to join them (kJoinTol in
// buildTrackPaths). At a seam one of the two coincident points is dropped, so this is how
// far a chained track's own end can be from where the path represents it.
constexpr double kJoinTolM = 1.0;

// Which path carries each track. One path only: networkTracks() is deduped to one segment
// per id and buildTrackPaths' `used[]` makes the chaining a strict partition, so a track is
// on exactly one path, contiguously, once. That is what makes the bridge a function rather
// than a search - and it is worth asserting rather than assuming.
struct TrackIndex {
    std::unordered_map<std::uint32_t, int> path; // trackId -> path index
    std::unordered_map<std::uint32_t, int> runs; // trackId -> runs claiming it
};

TrackIndex indexTracks(const std::vector<TrackPath>& paths) {
    TrackIndex ix;
    for (std::size_t pi = 0; pi < paths.size(); ++pi)
        for (const TrackPath::TrackRun& r : paths[pi].trackRuns()) {
            ++ix.runs[r.trackId];
            ix.path.emplace(r.trackId, static_cast<int>(pi));
        }
    return ix;
}

int pathOf(const TrackIndex& ix, std::uint32_t trackId) {
    const auto it = ix.path.find(trackId);
    return it == ix.path.end() ? -1 : it->second;
}

// Each surveyed point of a track as its own planar-chord fraction - the rule the overlay
// measures a fraction by, so these are the fractions that must land exactly.
std::vector<double> pointFractions(const TrackPoly& tp) {
    std::vector<double> out{0.0};
    double total = 0.0;
    for (std::size_t k = 1; k < tp.pts.size(); ++k)
        total += std::hypot(tp.pts[k].x - tp.pts[k - 1].x, tp.pts[k].y - tp.pts[k - 1].y);
    if (total <= 0.0) return out;
    double cum = 0.0;
    for (std::size_t k = 1; k < tp.pts.size(); ++k) {
        cum += std::hypot(tp.pts[k].x - tp.pts[k - 1].x, tp.pts[k].y - tp.pts[k - 1].y);
        out.push_back(cum / total);
    }
    return out;
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
    const TrackGraph graph = buildTrackGraph(data);
    if (paths.empty() || graph.pointWorld.empty()) {
        std::puts("no track geometry loaded - cannot test");
        return 1;
    }
    // The overlay's own view of the same rails, which is what a fraction means.
    std::vector<TrackPoly> polys;
    for (std::size_t i = 0; i < graph.pointWorld.size(); ++i) {
        if (polys.empty() || polys.back().id != graph.pointTrack[i])
            polys.push_back({graph.pointTrack[i], {}});
        polys.back().pts.push_back(graph.pointWorld[i]);
    }
    const glm::dvec3 org = data.sceneOrigin();
    const TrackCircuits circuits = loadTrackCircuits(root);
    const TrackIndex ix = indexTracks(paths);

    // Where the path puts (track, frac), in world coordinates.
    auto onPath = [&](int pi, std::uint32_t id, double frac, glm::dvec2& out) {
        float s = 0.0f;
        if (!paths[pi].fracToS(id, frac, s)) return false;
        const glm::vec3 p = paths[pi].poseAt(s).pos;
        out = {double(p.x) + org.x, double(p.y) + org.y};
        return true;
    };

    std::puts("\nthe bridge is built at all:");
    std::size_t runs = 0, descending = 0;
    for (const TrackPath& p : paths) {
        runs += p.trackRuns().size();
        for (const TrackPath::TrackRun& r : p.trackRuns())
            if (r.descending) ++descending;
    }
    check(runs > 0, "track runs resolved", static_cast<double>(runs), 1);
    // A chain takes a segment backwards whenever its export order runs against whichever
    // segment happened to seed it, and that run's fractions then descend. If the dataset had
    // none of those, everything below would prove nothing about the case - and it is the
    // case that put a border 43 km out when the search was written for one direction only.
    check(descending > 0, "of which some run backwards", static_cast<double>(descending), 1);
    std::printf("    %zu run(s) over %zu path(s), %zu of them descending\n", runs,
                paths.size(), descending);

    std::puts("\neach track is on exactly one path:");
    {
        int shared = 0, missing = 0;
        for (const TrackPoly& tp : polys) {
            const auto it = ix.runs.find(tp.id);
            if (it == ix.runs.end()) ++missing;
            else if (it->second > 1) ++shared;
        }
        check(shared == 0, "no track claimed by two runs", shared, 0);
        check(missing == 0, "no track missing from every path", missing, 0);
    }

    // The sharp one. Both representations are built from the same surveyed coordinates and
    // both pass through every one of them, so away from the seams they must agree - to the
    // limit of what the geometry can even represent.
    //
    // That limit is not a constant. TrackPath holds its control points as scene-relative
    // *floats*, and this dataset runs 733 km from the scene origin at its far end, where a
    // float32 step is 87 mm. So the residual is judged in units of the local float
    // resolution rather than in metres: within one of those the path has reproduced the
    // point as exactly as it is able to, and a real error in the bridge - which would be a
    // whole span or a whole track out - could not hide under it.
    std::puts("\nat every surveyed point in the body of a track:");
    {
        double worstUlps = 0.0, worstM = 0.0;
        std::uint32_t worstTrack = 0;
        long checked = 0;
        for (const TrackPoly& tp : polys) {
            const int pi = pathOf(ix, tp.id);
            if (pi < 0 || tp.pts.size() < 3) continue;
            const std::vector<double> fr = pointFractions(tp);
            // Interior only: the two ends are seams, checked on their own terms below.
            for (std::size_t k = 1; k + 1 < fr.size() && k + 1 < tp.pts.size(); ++k) {
                glm::dvec2 got;
                if (!onPath(pi, tp.id, fr[k], got)) continue;
                const double d = std::hypot(got.x - tp.pts[k].x, got.y - tp.pts[k].y);
                // One float step at this point's own distance from the scene origin.
                const double mag = std::max(std::abs(tp.pts[k].x - org.x),
                                            std::abs(tp.pts[k].y - org.y));
                const double ulp = std::max(mag * 1.1920929e-7, 1e-4); // 2^-23
                if (d / ulp > worstUlps) {
                    worstUlps = d / ulp;
                    worstM = d;
                    worstTrack = tp.id;
                }
                ++checked;
            }
        }
        std::printf("    %ld point(s) over %zu track(s)\n", checked, polys.size());
        check(worstUlps < 2.0, "worst offset, in float steps of the scene", worstUlps, 2.0);
        std::printf("    worst was %.4f m on track %x, where one float step is %.4f m\n",
                    worstM, worstTrack, worstUlps > 0 ? worstM / worstUlps : 0.0);
    }

    // The ends. A chained track's own end is dropped as a duplicate of the node before it,
    // so the path represents it by the neighbour's point - which the chaining will have
    // joined from as far as kJoinTol away. There is no rail at the dropped position, so
    // this is not an error to be fixed but a limit to be known.
    std::puts("\nat the two extreme ends of every track:");
    {
        double worst = 0.0;
        int ends = 0, over = 0;
        for (const TrackPoly& tp : polys) {
            const int pi = pathOf(ix, tp.id);
            if (pi < 0 || tp.pts.size() < 2) continue;
            for (int e = 0; e < 2; ++e) {
                glm::dvec2 got;
                if (!onPath(pi, tp.id, e ? 1.0 : 0.0, got)) continue;
                const glm::dvec3& w = e ? tp.pts.back() : tp.pts.front();
                const double d = std::hypot(got.x - w.x, got.y - w.y);
                worst = std::max(worst, d);
                if (d > 0.1) ++over;
                ++ends;
            }
        }
        check(ends > 0, "track ends placed", ends, 1);
        check(worst <= kJoinTolM, "worst end offset (m)", worst, kJoinTolM);
        std::printf("    %d of %d end(s) further out than 0.1 m - the seams where two "
                    "exported tracks do not quite meet\n", over, ends);
    }

    // Borders are authored by eye and mostly land between two surveyed points, where the
    // chord and the spline genuinely differ. There is no absolute number to assert, so
    // assert the relative thing: the bridge must add nothing beyond what the two
    // representations already differ by on that same track.
    std::puts("\nevery authored border, against its own track's chord/spline gap:");
    {
        double worst = 0.0, worstAllowed = 0.0;
        std::uint32_t worstTrack = 0;
        int placed = 0, unplaced = 0, overGap = 0;
        for (const Border& b : circuits.borders) {
            const int pi = pathOf(ix, b.trackId);
            glm::dvec2 got;
            if (pi < 0 || !onPath(pi, b.trackId, b.frac, got)) { ++unplaced; continue; }
            const glm::dvec3 want = fracToWorld(polys, b.trackId, b.frac);
            if (want.x == 0.0 && want.y == 0.0) { ++unplaced; continue; }
            const double d = std::hypot(got.x - want.x, got.y - want.y);

            // The most the two differ anywhere on this track: sampled at the middle of
            // every span, which is where the chord is furthest from the spline. A border
            // may also sit at a seam, so the join tolerance is allowed as well.
            const TrackPoly* tp = nullptr;
            for (const TrackPoly& q : polys)
                if (q.id == b.trackId) tp = &q;
            double gap = kJoinTolM;
            if (tp != nullptr) {
                const std::vector<double> fr = pointFractions(*tp);
                for (std::size_t k = 1; k < fr.size(); ++k) {
                    glm::dvec2 mg;
                    const double mid = 0.5 * (fr[k - 1] + fr[k]);
                    if (!onPath(pi, b.trackId, mid, mg)) continue;
                    const glm::dvec3 mw = fracToWorld(polys, b.trackId, mid);
                    gap = std::max(gap, std::hypot(mg.x - mw.x, mg.y - mw.y));
                }
            }
            if (d > gap + 0.01) ++overGap;
            if (d > worst) { worst = d; worstAllowed = gap; worstTrack = b.trackId; }
            ++placed;
        }
        check(placed > 0, "borders placed on a path", placed, 1);
        check(unplaced == 0, "borders the bridge could not place", unplaced, 0);
        check(overGap == 0, "borders further out than that gap", overGap, 0);
        std::printf("    worst was %.3f m on track %x, where the two representations "
                    "differ by up to %.3f m anyway\n", worst, worstTrack, worstAllowed);
    }

    // And back again, which is the direction occupancy reads: a train's arc length has to
    // come back as the track and fraction the overlay was authored in.
    std::puts("\nand the same borders, converted back off the path:");
    {
        double worstFrac = 0.0, worstM = 0.0;
        int wrongTrack = 0;
        for (const Border& b : circuits.borders) {
            const int pi = pathOf(ix, b.trackId);
            if (pi < 0) continue;
            float s = 0.0f;
            if (!paths[pi].fracToS(b.trackId, b.frac, s)) continue;
            std::uint32_t backTrack = 0;
            double backFrac = 0.0;
            if (!paths[pi].trackAt(s, backTrack, backFrac) || backTrack != b.trackId) {
                ++wrongTrack;
                continue;
            }
            // Judged in metres, so a 19 km track and a 50 m siding are held to the same
            // standard rather than to the same number.
            double len = 0.0;
            for (const TrackPoly& q : polys)
                if (q.id == b.trackId) len = polyLength(q.pts);
            const double errM = std::abs(backFrac - b.frac) * len;
            if (errM > worstM) { worstM = errM; worstFrac = std::abs(backFrac - b.frac); }
        }
        check(wrongTrack == 0, "borders that came back on another track", wrongTrack, 0);
        check(worstM < 0.05, "worst round-trip error (m)", worstM, 0.05);
        std::printf("    that is %.2e in fraction\n", worstFrac);
    }

    std::printf("\n%s\n", failures == 0 ? "all ok" : "FAILURES");
    return failures == 0 ? 0 : 1;
}
