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

#include "SwitchNetwork.h"

#include "TerrainData.h"
#include "TrackPath.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <unordered_set>

namespace {
// 2-D distance from p to segment a-b, plus the clamped parameter t along a->b.
struct SegHit { float dist; float t; };
SegHit distToSeg(const glm::dvec2& p, const glm::dvec2& a, const glm::dvec2& b) {
    const glm::dvec2 ab = b - a;
    const double l2 = glm::dot(ab, ab);
    const double t = l2 > 1e-9 ? glm::clamp(glm::dot(p - a, ab) / l2, 0.0, 1.0) : 0.0;
    return {static_cast<float>(glm::length(p - (a + ab * t))), static_cast<float>(t)};
}

// Nearest point on a path to scene point X: fills distance, arc-length s and the
// unit 2-D tangent there. Sampled at ~1 m, adequate for locating a turnout.
void nearestOnPath(const TrackPath& p, const glm::dvec2& X, double& outDist,
                   float& outS, glm::dvec2& outTan) {
    outDist = 1e30;
    const float len = p.length();
    for (float s = 0.0f; s <= len + 0.5f; s += 1.0f) {
        const float ss = std::min(s, len);
        const glm::vec3 q = p.poseAt(ss).pos;
        const double d = std::hypot(q.x - X.x, q.y - X.y);
        if (d < outDist) {
            outDist = d;
            outS = ss;
            const glm::vec3 tg = p.poseAt(ss).tangent;
            const double tl = std::hypot(tg.x, tg.y);
            outTan = tl > 1e-9 ? glm::dvec2(tg.x / tl, tg.y / tl) : glm::dvec2(1, 0);
        }
    }
}
} // namespace

void SwitchNetwork::build(const TerrainData& data,
                          const std::vector<TrackPath>& paths) {
    turnouts_.clear();
    state_.clear();
    origin_ = data.sceneOrigin();

    // --- Detect turnouts (endpoint-on-another-track, minus straight stitches) ---
    std::unordered_set<std::uint32_t> seen;
    std::vector<const TrackSegment*> segs;
    for (const Tile& t : data.tiles())
        for (const TrackSegment& s : t.tracks)
            if (s.pts.size() >= 2 && seen.insert(s.trackId).second) segs.push_back(&s);

    std::vector<Turnout> raw;
    for (const TrackSegment* sp : segs) {
        const std::vector<glm::dvec3>& p = sp->pts;
        const std::pair<glm::dvec3, glm::dvec3> endsWithNb[2] = {
            {p.front(), p[1]}, {p.back(), p[p.size() - 2]}};
        for (const auto& [end, nb] : endsWithNb) {
            const glm::dvec2 e2(end.x, end.y);
            float best = 2.0f; // metres; the touch tolerance
            glm::dvec3 hitX{0.0};
            glm::dvec2 hitThru{0.0};
            const TrackSegment* hitSeg = nullptr;
            for (const TrackSegment* op : segs) {
                if (op == sp) continue;
                const std::vector<glm::dvec3>& q = op->pts;
                for (std::size_t k = 0; k + 1 < q.size(); ++k) {
                    const glm::dvec2 a(q[k].x, q[k].y), b(q[k + 1].x, q[k + 1].y);
                    const SegHit h = distToSeg(e2, a, b);
                    if (h.dist < best) {
                        best = h.dist;
                        const double z = q[k].z + (q[k + 1].z - q[k].z) * h.t;
                        hitX = glm::dvec3(e2.x, e2.y, z);
                        hitThru = b - a;
                        hitSeg = op;
                    }
                }
            }
            if (!hitSeg) continue;
            const double tl = glm::length(hitThru);
            const glm::dvec2 dv(nb.x - end.x, nb.y - end.y);
            const double dl = glm::length(dv);
            if (tl < 1e-6 || dl < 1e-6) continue;
            const glm::dvec2 T = hitThru / tl, D = dv / dl;

            // Reject a straight end-to-end stitch (two ends joined collinearly, e.g. a
            // tunnel break rejoined into a continuous route): keep the junction only if
            // the through track passes *through* it or the branch clearly diverges.
            constexpr double kEndMargin = 10.0;    // m
            constexpr double kMinDivergeDeg = 8.0; // deg
            const double endDist =
                std::min(std::hypot(e2.x - hitSeg->pts.front().x, e2.y - hitSeg->pts.front().y),
                         std::hypot(e2.x - hitSeg->pts.back().x, e2.y - hitSeg->pts.back().y));
            const double angDeg =
                glm::degrees(std::acos(std::clamp(std::abs(T.x * D.x + T.y * D.y), 0.0, 1.0)));
            if (endDist < kEndMargin && angDeg < kMinDivergeDeg) continue;

            Turnout to;
            to.world = hitX;
            to.thru = T;
            to.div = D;
            raw.push_back(to);
        }
    }

    // Collapse turnouts sharing a point (both tracks' ends meet there) to one.
    for (const Turnout& t : raw) {
        bool dup = false;
        for (const Turnout& u : turnouts_)
            if (std::hypot(t.world.x - u.world.x, t.world.y - u.world.y) < 3.0) {
                dup = true;
                break;
            }
        if (!dup) turnouts_.push_back(t);
    }

    // --- Resolve each turnout onto the built paths (main + siding) ---
    int resolved = 0;
    for (Turnout& t : turnouts_) {
        const glm::dvec2 X(t.world.x - origin_.x, t.world.y - origin_.y);

        int siding = -1, main = -1;
        double bestSide = 0.3, bestMain = 0.7; // required direction agreement
        float sMain = 0.0f, sSiding = 0.0f;
        glm::dvec2 mainTan(1, 0);
        for (int pi = 0; pi < static_cast<int>(paths.size()); ++pi) {
            const TrackPath& p = paths[pi];
            double d;
            float s;
            glm::dvec2 tan;
            nearestOnPath(p, X, d, s, tan);
            if (d > 2.5) continue; // not at this turnout

            // Siding: a branch whose *end* is at X, leaving along `div`.
            const float len = p.length();
            const bool nearFront = std::hypot(p.poseAt(0.0f).pos.x - X.x,
                                              p.poseAt(0.0f).pos.y - X.y) < 2.5;
            const bool nearBack = std::hypot(p.poseAt(len).pos.x - X.x,
                                             p.poseAt(len).pos.y - X.y) < 2.5;
            if (nearFront || nearBack) {
                // Leaving direction into the path from whichever end is at X.
                const float es = nearFront ? 0.0f : len;
                glm::vec3 tg = p.poseAt(es).tangent;
                glm::dvec2 leave(tg.x, tg.y);
                if (!nearFront) leave = -leave;
                const double ll = glm::length(leave);
                if (ll > 1e-9) leave /= ll;
                const double a = glm::dot(leave, t.div);
                if (a > bestSide) {
                    bestSide = a;
                    siding = pi;
                    sSiding = es;
                }
            }
            // Main: the through track, tangent parallel to `thru` at X.
            const double m = std::abs(glm::dot(tan, t.thru));
            if (m > bestMain) {
                bestMain = m;
                main = pi;
                sMain = s;
                mainTan = tan;
            }
        }
        // Main and siding must differ; if the same path won both, drop it as main.
        if (main == siding) main = -1;
        if (main < 0 || siding < 0) continue;
        // The main must genuinely pass *through* the point (a toe side and a frog
        // side). If it merely ends here (a stub-to-stub yard junction), there is no
        // common track to route through, so leave this switch inert.
        const float mlen = paths[main].length();
        if (std::min(sMain, mlen - sMain) < 5.0f) continue;

        t.mainPath = main;
        t.sMain = sMain;
        t.sidingPath = siding;
        t.sSiding = sSiding;
        // Facing = approaching from the toe (moving toward the frog). The frog
        // direction on the main is `thru` toward where the branch heads.
        const glm::dvec2 frog = t.thru * (glm::dot(t.div, t.thru) >= 0 ? 1.0 : -1.0);
        t.facingS = glm::dot(mainTan, frog) >= 0.0 ? 1 : -1;
        ++resolved;
    }

    state_.assign(turnouts_.size(), SwitchState::Straight);
    std::printf("[SwitchNetwork] %zu turnouts (%d routed)\n", turnouts_.size(), resolved);
}

void SwitchNetwork::toggle(int i) {
    if (i < 0 || i >= static_cast<int>(state_.size())) return;
    state_[i] = state_[i] == SwitchState::Straight ? SwitchState::Diverging
                                                   : SwitchState::Straight;
}
