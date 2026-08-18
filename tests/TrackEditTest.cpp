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
#include "SignalPaths.h"
#include "SwitchNetwork.h"
#include "TrackPath.h"
#include "TxpMesh.h"
#include "TxpPositions.h"
#include "Vehicle.h"

#include <cmath>
#include <algorithm>
#include <chrono>
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

    // Which turnouts a route needs, and how quickly.
    //
    // This is asked again every time a train enters or leaves a track circuit, for every
    // route in the station, so it has to be quick - it once walked all 3350 turnouts per
    // route with a full track-list scan inside, and a train crossing a circuit border
    // stopped the sim for the best part of a second. It is fast because a turnout outside
    // the route's own bounding box cannot be standing on it; the first check here is that
    // invariant, since everything else rests on it.
    {
        std::vector<TrackPoly> polys;
        for (std::size_t i = 0; i < g1.pointWorld.size(); ++i) {
            if (polys.empty() || polys.back().id != g1.pointTrack[i])
                polys.push_back({g1.pointTrack[i], {}});
            polys.back().pts.push_back(g1.pointWorld[i]);
        }
        SwitchNetwork net;
        net.build(data, paths1);
        std::vector<SignalPath> routes = loadSignalPaths(root);
        for (const std::vector<SignalPath>& more :
             {loadExitSignals(root), loadEntrySignals(root), loadExitRoutes(root)})
            routes.insert(routes.end(), more.begin(), more.end());

        if (routes.empty() || net.size() == 0) {
            std::puts("  no routes or no turnouts here - nothing to check");
        } else {
            // Timed on its own, since checking the answers costs more than deriving them.
            const auto t0 = std::chrono::steady_clock::now();
            std::vector<std::vector<PathSwitch>> all;
            all.reserve(routes.size());
            for (const SignalPath& p : routes)
                all.push_back(pathSwitchRequirements(p, net, polys));
            const double ms = std::chrono::duration<double, std::milli>(
                                  std::chrono::steady_clock::now() - t0)
                                  .count();

            std::size_t reqs = 0;
            double worstOff = 0.0;
            for (std::size_t r = 0; r < routes.size(); ++r) {
                for (const PathSwitch& ps : all[r]) {
                    ++reqs;
                    // How far the named turnout stands from the nearest point of the
                    // route. A requirement further off than kAtTurnout would mean the
                    // box had let through something the geometry then accepted wrongly.
                    const glm::dvec3& w = net.turnouts()[ps.turnout].world;
                    double best = 1e30;
                    for (const SectionInterval& iv : routes[r].parts) {
                        double frac = 0.0, dist = 0.0;
                        if (!projectOnTrack(polys, iv.trackId, glm::dvec2(w.x, w.y), frac,
                                            dist))
                            continue;
                        best = std::min(best, dist);
                    }
                    worstOff = std::max(worstOff, best);
                }
            }
            std::printf("  %zu route(s), %zu turnout(s), %zu requirement(s)\n",
                        routes.size(), net.size(), reqs);
            if (reqs > 0)
                check(worstOff <= 3.0, "requirement stands at its turnout", worstOff, 0.0);
            // Generous: it measures a few milliseconds, and the point is to catch a
            // return to walking the whole network, which is two orders of magnitude away.
            check(ms < 100.0, "derived for every route (ms)", ms, 0.0);
        }
    }

    // A train running over a turnout knocks once per axle. Modern rail is welded and
    // has no joints to beat against, so the points are the only place a wheel meets a
    // gap - and how many knocks there are is an integer the sim can be asked for
    // directly, rather than something to be picked back out of a waveform.
    {
        std::puts("\n  Wheels over the points:");
        SwitchNetwork net;
        net.build(data, paths1);
        // A turnout with room either side of it on its own path, so the whole train can
        // be run past it without reaching either end and derailing.
        int found = -1;
        for (std::size_t i = 0; i < net.turnouts().size() && found < 0; ++i) {
            const Turnout& to = net.turnouts()[i];
            if (to.mainPath < 0) continue;
            const float len = paths1[to.mainPath].length();
            if (to.sMain > 200.0f && to.sMain < len - 200.0f) found = static_cast<int>(i);
        }
        if (found < 0) {
            std::puts("  no turnout with room to run past - nothing to check");
        } else {
            const Turnout& to = net.turnouts()[found];
            // The Class 93: three bogies, six axles.
            const VehicleSpec& spec = kVehicleSpecs[4];
            Vehicle v(&paths1[to.mainPath], spec, to.sMain - 120.0f, 20.0f);
            v.attachNetwork(&paths1, &net);
            v.setReverser(0, 1);      // one cab in gear, so the handle rules...
            v.setBrakeNotch(0, 0);    // ...and release the brakes it starts held with
            const std::size_t axles = v.axleOffsets().size();
            const float from = v.s();
            // Long enough to carry every axle past: 120 m up to it plus the train's own
            // length, at 20 m/s, with the rolling resistance shaving a little off.
            for (int step = 0; step < 1200; ++step) v.update(1.0f / 60.0f);
            const float ran = std::abs(v.s() - from);
            std::printf("  %zu axles, ran %.0f m over turnout %d, %u knock(s)\n", axles,
                        ran, found, v.railImpacts());
            // The count only means anything if the train really did run the whole way
            // past on the rails: derailed or stalled short of it, zero knocks would
            // "pass" a test that had checked nothing.
            check(v.state() == VehicleState::OnRail, "still on the rails",
                  double(static_cast<int>(v.state())), 0.0);
            check(ran > 120.0f + spec.bogieSpacing + spec.wheelbase,
                  "and ran the whole train past it (m)", double(ran), 120.0);
            check(v.railImpacts() == axles, "one knock per axle over one turnout",
                  double(v.railImpacts()), double(axles));
            // And none at all standing still on plain line: welded rail is silent.
            Vehicle still(&paths1[to.mainPath], spec, 20.0f, 0.0f);
            still.attachNetwork(&paths1, &net);
            for (int step = 0; step < 120; ++step) still.update(1.0f / 60.0f);
            check(still.railImpacts() == 0, "and none standing on plain line",
                  double(still.railImpacts()), 0.0);
        }
    }

    // Moving a drawn siding's points about in plan.
    //
    // A drawn road *is* its overlay record, so the editor moves a point by editing that
    // record and laying the road down again - there is no `move` edit and nothing to
    // reconcile. What has to hold is that the move reaches the network, that it changes
    // the plan and nothing else (the height is geometry mode's business), and that
    // dragging an end onto or off another track makes or breaks its switch, which is the
    // only reason to move an end at all.
    {
        std::puts("\n  Moving a drawn siding's points:");
        // A vertex in the middle of a real main-line segment to branch from, so the road
        // starts on the line rather than merely near it.
        const TrackSegment* host = nullptr;
        std::size_t vi = 0;
        for (const TrackSegment& s : data.networkTracks()) {
            if (s.trackType != 0 || s.pts.size() < 9) continue;
            host = &s;
            vi = s.pts.size() / 2;
            break;
        }
        if (!host) {
            std::puts("  no main-line segment to branch from - nothing to check");
        } else {
            const glm::dvec3 root = host->pts[vi];
            glm::dvec2 t(host->pts[vi + 1].x - host->pts[vi - 1].x,
                         host->pts[vi + 1].y - host->pts[vi - 1].y);
            t /= std::max(glm::length(t), 1e-9);
            // 15 degrees off the running line: inside the 8-35 window a turnout needs.
            const double c = std::cos(0.2618), sn = std::sin(0.2618);
            const glm::dvec2 d(t.x * c - t.y * sn, t.x * sn + t.y * c);

            TrackEdit rec;
            rec.kind = TrackEdit::Track;
            rec.track = kNewTrackIdBase + 900;
            rec.trackType = 1;
            rec.pts = {root,
                       {root.x + d.x * 60.0, root.y + d.y * 60.0, root.z},
                       {root.x + d.x * 130.0, root.y + d.y * 130.0, root.z}};

            auto turnoutsOnIt = [&](const TrackEdit& r) {
                std::vector<TrackPath> ps = buildTrackPaths(data);
                SwitchNetwork n;
                n.build(data, ps);
                int hits = 0;
                for (const Turnout& to : n.turnouts())
                    if (to.sidingTrack == r.track) ++hits;
                return hits;
            };

            data.applyTrackEdits({rec});
            const int joined = turnoutsOnIt(rec);
            std::printf("  drawn at the root: %d turnout(s)\n", joined);
            check(joined == 1, "a road drawn onto the line has its switch",
                  double(joined), 1.0);

            // Drag the middle point sideways to tweak the alignment: the plan changes,
            // the heights do not, and the switch survives - a 4 m nudge 60 m out swings
            // the angle the road leaves at by under 4 degrees, well inside the window a
            // turnout needs. (A big enough nudge would take the switch off the root
            // without the root having moved at all, which is why the mode reports the
            // angle at both ends whichever point is being dragged.)
            const glm::dvec2 across(-d.y, d.x);
            const double z0 = rec.pts[1].z;
            rec.pts[1].x += across.x * 4.0;
            rec.pts[1].y += across.y * 4.0;
            data.removeTrack(rec.track);
            data.applyTrackEdits({rec});
            const TrackSegment* laid = nullptr;
            for (const TrackSegment& s : data.networkTracks())
                if (s.trackId == rec.track) laid = &s;
            check(laid != nullptr, "the moved road is in the network", laid ? 1.0 : 0.0,
                  1.0);
            if (laid) {
                const double moved = std::hypot(laid->pts[1].x - (root.x + d.x * 60.0),
                                                laid->pts[1].y - (root.y + d.y * 60.0));
                check(std::abs(moved - 4.0) < 1e-6, "the middle point moved in plan (m)",
                      moved, 4.0);
                check(std::abs(laid->pts[1].z - z0) < 1e-9,
                      "and not a millimetre in height", laid->pts[1].z - z0, 0.0);
            }
            check(turnoutsOnIt(rec) == 1, "its switch is where it was",
                  double(turnoutsOnIt(rec)), 1.0);

            // Now drag the *root* off the line. Nothing joins there any more, and the
            // switch has to go with it - which is the thing an end move is really for.
            rec.pts[0].x += d.x * 40.0;
            rec.pts[0].y += d.y * 40.0;
            data.removeTrack(rec.track);
            data.applyTrackEdits({rec});
            const int adrift = turnoutsOnIt(rec);
            std::printf("  root dragged 40 m clear: %d turnout(s)\n", adrift);
            check(adrift == 0, "an end moved off the line loses its switch",
                  double(adrift), 0.0);

            // And back onto it: the switch returns. A move is not one-way.
            rec.pts[0] = root;
            data.removeTrack(rec.track);
            data.applyTrackEdits({rec});
            check(turnoutsOnIt(rec) == 1, "and gets it back when put back",
                  double(turnoutsOnIt(rec)), 1.0);

            // How near counts as "on the line". The editor snaps an end onto a track and
            // reports it as joined within a tolerance of its own, and that tolerance has
            // to be the detector's or it promises switches that never appear. This is
            // where the two are held together: 2 m is SwitchNetwork's touch tolerance.
            auto atOffset = [&](double off) {
                const glm::dvec3 a(root.x + across.x * off, root.y + across.y * off, root.z);
                rec.pts[0] = a;
                rec.pts[1] = {a.x + d.x * 60.0, a.y + d.y * 60.0, a.z};
                rec.pts[2] = {a.x + d.x * 130.0, a.y + d.y * 130.0, a.z};
                data.removeTrack(rec.track);
                data.applyTrackEdits({rec});
                return turnoutsOnIt(rec);
            };
            const int near = atOffset(1.5), far = atOffset(3.0);
            std::printf("  1.5 m off the line: %d turnout(s); 3.0 m off: %d\n", near, far);
            check(near == 1, "an end just off the line still switches", double(near), 1.0);
            check(far == 0, "and one well off it does not", double(far), 0.0);
            rec.pts[0] = root; // put it back for the round-trip below
            rec.pts[1] = {root.x + d.x * 60.0, root.y + d.y * 60.0, root.z};
            rec.pts[2] = {root.x + d.x * 130.0, root.y + d.y * 130.0, root.z};

            // The record is what is saved, so it has to survive the file with the moves
            // in it. Written to the build tree, never to the dataset.
            const std::string scratch =
                (std::filesystem::temp_directory_path() / "ebaner-move-test").string();
            std::error_code ec;
            std::filesystem::create_directories(scratch + "/overlay", ec);
            if (writeTrackOverlay(scratch, {rec})) {
                const std::vector<TrackEdit> back = loadTrackOverlay(scratch);
                double worst = 0.0;
                if (back.size() == 1 && back[0].pts.size() == rec.pts.size())
                    for (std::size_t i = 0; i < rec.pts.size(); ++i)
                        worst = std::max(worst, glm::length(back[0].pts[i] - rec.pts[i]));
                else
                    worst = 1e9;
                check(worst < 0.002, "the moved road round-trips through the overlay (m)",
                      worst, 0.0);
            }
            std::filesystem::remove_all(scratch, ec);
            data.removeTrack(rec.track); // leave the network as it was found
        }
    }

    std::printf("%s\n", failures ? "FAILED" : "PASSED");
    return failures ? 1 : 0;
}
