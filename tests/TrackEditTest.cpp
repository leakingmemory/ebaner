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

// A track edit has to reach everything built from track geometry.
//
// This was written for a regression where it did not. The rail network had just been
// made resident and global - the terrain is a window that streams, the railway is small
// enough to keep whole - which briefly left two stores of track geometry: the tiles the
// carve read, and the network the paths and the junction graph were built from. The
// editor's apply path still wrote only to the tiles, so raising a rail re-cut the ground
// and left the rail where it was. Nothing crashed and nothing warned.
//
// The stores have since been made one, which removes that failure by construction. What
// is left to protect is the step after it: that an edit applied to the network is
// actually seen by the things rebuilt from it. A cache, a stale copy or an apply pointed
// at the wrong list would all look exactly like the original bug.
//
// Usage: TrackEditTest <datasetRoot>   (exit 77 = skipped, no dataset)

#include "PlatformMesh.h"
#include "TerrainData.h"
#include "TrackCircuits.h" // TrackPoly
#include "TrackGraph.h"
#include "TrackOverlay.h"
#include "TrackPath.h"
#include "TxpMesh.h"
#include "TxpPositions.h"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

namespace {

constexpr double kNudgeM = 5.0;  // a rise far larger than any smoothing or rounding
constexpr double kSameXY = 0.5;  // m, matching a vertex by position
int failures = 0;

void check(bool ok, const char* what, double got, double want) {
    std::printf("  %-34s %8.3f (want %.3f) %s\n", what, got, want, ok ? "ok" : "FAILED");
    if (!ok) ++failures;
}

// z of the vertex at `xy` on track `tid` in the rail network - the one store, and what
// the carve, the paths and the graph all read. False if the network has no such vertex.
bool networkVertexZ(const TerrainData& data, std::uint32_t tid, const glm::dvec2& xy,
                    double& z) {
    for (const TrackSegment& s : data.networkTracks()) {
        if (s.trackId != tid) continue;
        for (const glm::dvec3& p : s.pts)
            if (std::hypot(p.x - xy.x, p.y - xy.y) <= kSameXY) {
                z = p.z;
                return true;
            }
    }
    return false;
}

// z of the rail as it would be built, taken from the rebuilt paths at the nearest point
// to `xy`. This is what the editor's preview actually draws.
bool railZ(const std::vector<TrackPath>& paths, const glm::dvec3& origin,
           const glm::dvec2& xy, double& z) {
    const glm::vec2 tgt(static_cast<float>(xy.x - origin.x),
                        static_cast<float>(xy.y - origin.y));
    double best = 1e30;
    for (const TrackPath& p : paths) {
        if (!p.nearXY(tgt, 50.0f)) continue;
        for (float s = 0.0f; s <= p.length(); s += 1.0f) {
            const glm::vec3 q = p.poseAt(s).pos;
            const double d = std::hypot(q.x - tgt.x, q.y - tgt.y);
            if (d < best) {
                best = d;
                z = q.z;
            }
        }
    }
    return best < kSameXY;
}

} // namespace

int main(int argc, char** argv) {
    const std::string root = argc > 1 ? argv[1] : "../norway-rails";
    if (!std::filesystem::exists(root)) {
        std::printf("no dataset at %s - skipping\n", root.c_str());
        return 77; // CTest SKIP_RETURN_CODE
    }

    // A small terrain window: the test is about the rail stores, and the network is read
    // whole regardless of how much ground is loaded.
    TerrainData data;
    data.load(root, 4000.0);

    TrackGraph g0 = buildTrackGraph(data);
    std::vector<TrackPath> paths0 = buildTrackPaths(data);
    if (g0.pointWorld.empty()) {
        std::puts("no track points loaded - cannot test");
        return 1;
    }

    // Edit a vertex no other track shares. At a junction several tracks have a vertex at
    // the same place; an edit names one of them, so the rail nearest that point afterwards
    // may be a different track's that was never asked to move.
    int idx = -1;
    double netZ0 = 0.0;
    for (std::size_t i = 0; i < g0.pointWorld.size() && idx < 0; ++i) {
        const glm::dvec3& w = g0.pointWorld[i];
        if (!networkVertexZ(data, g0.pointTrack[i], glm::dvec2(w.x, w.y), netZ0)) continue;
        bool shared = false;
        for (std::size_t j = 0; j < g0.pointWorld.size() && !shared; ++j)
            shared = j != i && std::hypot(g0.pointWorld[j].x - w.x,
                                          g0.pointWorld[j].y - w.y) <= kSameXY;
        if (!shared) idx = static_cast<int>(i);
    }
    if (idx < 0) {
        std::puts("no unshared track vertex found - cannot test");
        return 1;
    }

    const glm::dvec3 at = g0.pointWorld[idx];
    const std::uint32_t tid = g0.pointTrack[idx];
    const double graphZ0 = at.z;
    double rail0 = 0.0;
    const bool haveRail = railZ(paths0, data.sceneOrigin(), glm::dvec2(at.x, at.y), rail0);

    // Exactly what the editor's doElevStep stages for a raise: this track's vertex, at
    // its own (x, y), with a new z.
    TrackEdit e;
    e.kind = TrackEdit::Elev;
    e.a = glm::dvec3(at.x, at.y, graphZ0 + kNudgeM);
    e.track = tid;
    data.applyTrackEdits({e});

    const TrackGraph g1 = buildTrackGraph(data);
    const std::vector<TrackPath> paths1 = buildTrackPaths(data);
    double netZ1 = 0.0;
    networkVertexZ(data, tid, glm::dvec2(at.x, at.y), netZ1);
    double rail1 = 0.0;
    if (haveRail) railZ(paths1, data.sceneOrigin(), glm::dvec2(at.x, at.y), rail1);

    std::printf("\ntrack %u vertex at (%.1f, %.1f), raised %.1f m\n", tid, at.x, at.y,
                kNudgeM);
    check(std::abs((netZ1 - netZ0) - kNudgeM) < 0.001, "rail network (the one store)",
          netZ1 - netZ0, kNudgeM);
    check(std::abs((g1.pointWorld[idx].z - graphZ0) - kNudgeM) < 0.001,
          "rebuilt junction graph", g1.pointWorld[idx].z - graphZ0, kNudgeM);
    // The rails are a spline through the control points, so raising one point lifts the
    // curve by very slightly less than the nudge. Only that it clearly moved matters.
    if (haveRail)
        check(rail1 - rail0 > kNudgeM * 0.5, "rebuilt rails", rail1 - rail0, kNudgeM);

    // The TXP stands on the platform, not in it.
    //
    // The track z is the rail head and a Norwegian platform is a step above that, so a
    // figure placed at track level is buried to the waist in the slab. It has to be
    // measured at the spot they actually stand, a few metres to the side, which is often
    // exactly the difference between the ballast and the platform. This lives here rather
    // than in the pure-logic test because it needs real platform footprints to stand on.
    {
        std::vector<TrackPoly> polys;
        for (std::size_t i = 0; i < g1.pointWorld.size(); ++i) {
            if (polys.empty() || polys.back().id != g1.pointTrack[i])
                polys.push_back({g1.pointTrack[i], {}});
            polys.back().pts.push_back(g1.pointWorld[i]);
        }
        const glm::dvec3 origin = data.sceneOrigin();

        // Hunt for spots whose standing position lands on a platform, and check each one
        // puts the figure's feet on the slab rather than through it.
        int found = 0;
        for (const TrackPoly& poly : polys) {
            if (found >= 3) break;
            for (double frac : {0.25, 0.5, 0.75}) {
                if (found >= 3) break;
                for (int side : {1, -1}) {
                    std::vector<TxpPosition> t(1);
                    t[0].id = 1;
                    t[0].trackId = poly.id;
                    t[0].frac = frac;
                    t[0].side = side;
                    const std::vector<float> lift = txpStandLift(t, polys, data, paths1);
                    if (std::abs(lift[0]) < 1e-4f) continue; // not on a platform

                    TxpMesh m;
                    m.build(t, std::vector<char>{1}, polys, origin, lift);
                    if (m.vertices().empty()) continue;
                    float feet = 1e30f;
                    for (const TrackVertex& v : m.vertices())
                        feet = std::min(feet, v.pos.z);

                    // The slab top at the spot they stand on, from the same place the
                    // drawn surface comes from.
                    const glm::dvec3 w = fracToWorld(polys, poly.id, frac);
                    const glm::dvec2 tg = trackTangent(polys, poly.id, frac, 1);
                    const double l = std::hypot(tg.x, tg.y);
                    if (l < 1e-9) continue;
                    const double sx = w.x + (tg.y / l) * 3.2 * side;
                    const double sy = w.y + (-tg.x / l) * 3.2 * side;
                    float top = 0.0f;
                    if (!platformTopAt(data, paths1, sx, sy, top)) continue;

                    check(std::abs(feet - top) < 0.01, "TXP feet on the platform slab",
                          feet - top, 0.0);
                    ++found;
                }
            }
        }
        if (found == 0)
            std::puts("  no track passes a platform in this window - nothing to check");
    }

    std::printf("%s\n", failures ? "FAILED" : "PASSED");
    return failures ? 1 : 0;
}
