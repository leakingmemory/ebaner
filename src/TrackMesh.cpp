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

#include <algorithm>
#include "TrackMesh.h"

#include "TrackPath.h"

#include <cstdio>
#include <vector>

namespace {

// --- Cross-section dimensions (metres) -------------------------------------
constexpr float kSleeperSpacing = 0.6f;  // tie pitch
constexpr int kSleepersPerChunk = 48;    // ~29 m runs for distance LOD
constexpr float kRailSampleStep = 3.0f;  // ballast/rail sweep step along the curve
constexpr float kRailHalf = 0.7175f;     // half of 1.435 m standard gauge

// Ballast trapezoid.
constexpr float kBallastTopHalf = 2.0f;
constexpr float kBallastBotHalf = 2.8f;
constexpr float kBallastHeight = 0.65f; // tall enough that ties sit down in it
constexpr float kBallastSink = 0.1f;    // seat the bottom slightly into the DTM

// Sleeper box.
constexpr float kSleeperHalfLen = 1.3f;  // across the track (2.6 m)
constexpr float kSleeperHalfWid = 0.13f; // along the track (0.26 m)
constexpr float kSleeperHeight = 0.22f;
constexpr float kSleeperExposed = 0.06f; // how much of the tie stands proud

// Rail prism.
constexpr float kRailHalfWidth = 0.0375f; // 0.075 m
constexpr float kRailHeight = 0.15f;

// Vertical offsets from the rail-bed centreline z (which already carries the
// +0.6 m bed offset baked into the export). Sleepers are mostly submerged in the
// ballast; only ~kSleeperExposed of the tie shows above the bed, with the rails
// on top.
constexpr float kBallastBotZ = -kBallastSink;                 // -0.10
constexpr float kBallastTopZ = kBallastHeight - kBallastSink; //  0.55
constexpr float kSleeperTopZ = kBallastTopZ + kSleeperExposed;   // 0.61
constexpr float kSleeperBotZ = kSleeperTopZ - kSleeperHeight;    // 0.39
constexpr float kRailBotZ = kSleeperTopZ;                     //  0.61
constexpr float kRailTopZ = kRailBotZ + kRailHeight;          //  0.76

// Colours.
const glm::vec3 kBallastTint(1.0f, 1.0f, 1.0f); // multiplies the ballast texture
const glm::vec3 kBallastSide(0.42f, 0.40f, 0.36f);
const glm::vec3 kSleeperCol(0.58f, 0.58f, 0.56f); // concrete
const glm::vec3 kRailCol(0.40f, 0.25f, 0.18f);    // rusty steel

// Sample distances along a path: 0, step, 2*step, ... , length (endpoint always
// included so the sweep closes exactly).
std::vector<float> sampleDistances(float from, float to, float step) {
    std::vector<float> ss;
    for (float s = from; s < to; s += step) ss.push_back(s);
    ss.push_back(to);
    return ss;
}

} // namespace

void TrackMesh::build(const std::vector<TrackPath>& paths, const glm::vec3& centre,
                      float radius) {
    // Reset accumulators so build() is idempotent (the editor rebuilds to re-preview).
    vertices_.clear();
    indices_.clear();
    chunks_.clear();
    alwaysIndexCount_ = 0;
    alwaysChunks_.clear();

    // Which stretch of each path to draw. A path is not near or far as a whole - the
    // main line runs the length of the country - so this clips it to the arc length
    // that is actually in reach. A bounding box only answers the first, cheap half of
    // that question: a long path's box covers everything, so it must not be the last
    // word or half the network comes out "near".
    struct Run { const TrackPath* path; float from, to; };
    std::vector<Run> near;
    constexpr float kProbeM = 100.0f; // how finely the reach is bracketed
    for (const TrackPath& p : paths) {
        if (radius <= 0.0f) { near.push_back({&p, 0.0f, p.length()}); continue; }
        if (!p.nearXY(glm::vec2(centre), radius)) continue;
        // Every contiguous stretch in range, not the span from the first to the last:
        // a path can run in, leave, and come back hundreds of kilometres later, and
        // bridging that gap would build the whole line.
        float lo = -1.0f, hi = -1.0f;
        auto flushRun = [&]() {
            if (lo < 0.0f) return;
            // Out to the probe either side, so a run is not cut short of the radius by
            // where the probe happened to land.
            near.push_back({&p, std::max(0.0f, lo - kProbeM),
                            std::min(p.length(), hi + kProbeM)});
            lo = hi = -1.0f;
        };
        for (float s = 0.0f;; s += kProbeM) {
            const float ss = std::min(s, p.length());
            if (glm::distance(p.poseAt(ss).pos, centre) < radius) {
                if (lo < 0.0f) lo = ss;
                hi = ss;
            } else {
                flushRun();
            }
            if (ss >= p.length()) break;
        }
        flushRun();
    }

    // Emit one quad (two triangles) with an outward-facing normal. The normal is
    // the geometric normal, flipped to point away from `inside` so lighting is
    // correct regardless of winding (the pipeline draws with culling off).
    auto emitQuad = [&](const glm::vec3& p0, const glm::vec3& p1,
                        const glm::vec3& p2, const glm::vec3& p3,
                        const glm::vec3& inside, const glm::vec3& color,
                        float texLayer, glm::vec2 uv0, glm::vec2 uv1,
                        glm::vec2 uv2, glm::vec2 uv3) {
        glm::vec3 n = glm::cross(p1 - p0, p3 - p0);
        const float nl = glm::length(n);
        n = (nl > 1e-12f) ? n / nl : glm::vec3(0.0f, 0.0f, 1.0f);
        const glm::vec3 cen = (p0 + p1 + p2 + p3) * 0.25f;
        if (glm::dot(n, cen - inside) < 0.0f) n = -n;
        const std::uint32_t base = static_cast<std::uint32_t>(vertices_.size());
        vertices_.push_back({p0, n, color, uv0, texLayer});
        vertices_.push_back({p1, n, color, uv1, texLayer});
        vertices_.push_back({p2, n, color, uv2, texLayer});
        vertices_.push_back({p3, n, color, uv3, texLayer});
        indices_.push_back(base + 0);
        indices_.push_back(base + 1);
        indices_.push_back(base + 2);
        indices_.push_back(base + 0);
        indices_.push_back(base + 2);
        indices_.push_back(base + 3);
    };
    const glm::vec2 z2(0.0f);
    auto solidQuad = [&](const glm::vec3& p0, const glm::vec3& p1,
                         const glm::vec3& p2, const glm::vec3& p3,
                         const glm::vec3& inside, const glm::vec3& color) {
        emitQuad(p0, p1, p2, p3, inside, color, -1.0f, z2, z2, z2, z2);
    };
    // Point at centreline P, offset `across` along cross-track right R and `z`
    // along cross-track up U (both banked by cant).
    auto pt = [](const glm::vec3& P, const glm::vec3& R, const glm::vec3& U,
                 float across, float z) {
        return P + R * across + U * z;
    };

    // --- Pass 1: ballast bed + rails (always drawn) ------------------------
    //
    // Cut into pieces as it goes, so that what is behind the camera need not be drawn.
    // A run can be kilometres long and one bounding sphere around it would never be
    // rejected, so the piece is a fixed length of line rather than a whole run.
    constexpr float kAlwaysChunkM = 200.0f;
    std::uint32_t runFirst = 0, runVertFirst = 0;
    bool runOpen = false;
    // The piece's extent comes off the vertices it produced, so it is exactly what was
    // drawn rather than an estimate of how far the formation stands off the centre line.
    auto closeAlways = [&]() {
        if (!runOpen) return;
        const std::uint32_t n = static_cast<std::uint32_t>(indices_.size()) - runFirst;
        if (n > 0 && runVertFirst < vertices_.size()) {
            glm::vec3 lo(1e30f), hi(-1e30f);
            for (std::size_t k = runVertFirst; k < vertices_.size(); ++k) {
                lo = glm::min(lo, vertices_[k].pos);
                hi = glm::max(hi, vertices_[k].pos);
            }
            TrackDrawChunk c;
            c.firstIndex = runFirst;
            c.indexCount = n;
            c.centroid = 0.5f * (lo + hi);
            c.radius = 0.5f * glm::length(hi - lo);
            alwaysChunks_.push_back(c);
        }
        runOpen = false;
    };
    auto openAlways = [&]() {
        closeAlways();
        runFirst = static_cast<std::uint32_t>(indices_.size());
        runVertFirst = static_cast<std::uint32_t>(vertices_.size());
        runOpen = true;
    };
    for (const Run& run : near) {
        const TrackPath& path = *run.path;
        const std::vector<float> ss = sampleDistances(run.from, run.to, kRailSampleStep);
        float chunkFrom = run.from - kAlwaysChunkM; // force one open on the first sample
        for (std::size_t i = 0; i + 1 < ss.size(); ++i) {
            if (ss[i] - chunkFrom >= kAlwaysChunkM) { openAlways(); chunkFrom = ss[i]; }
            const TrackPose pa = path.poseAt(ss[i]);
            const TrackPose pb = path.poseAt(ss[i + 1]);
            const glm::vec3 A = pa.pos, B = pb.pos;
            const glm::vec3 Ra = pa.right, Rb = pb.right;
            const glm::vec3 Ua = pa.up, Ub = pb.up;
            const glm::vec3 mid = (A + B) * 0.5f;
            const float vA = ss[i] / kSleeperSpacing;
            const float vB = ss[i + 1] / kSleeperSpacing;

            // Ballast top (textured with the ballast/sleeper layer).
            emitQuad(pt(A, Ra, Ua,-kBallastTopHalf, kBallastTopZ),
                     pt(A, Ra, Ua,kBallastTopHalf, kBallastTopZ),
                     pt(B, Rb, Ub,kBallastTopHalf, kBallastTopZ),
                     pt(B, Rb, Ub,-kBallastTopHalf, kBallastTopZ), mid, kBallastTint,
                     0.0f, glm::vec2(0.0f, vA), glm::vec2(1.0f, vA),
                     glm::vec2(1.0f, vB), glm::vec2(0.0f, vB));

            // Ballast side slopes.
            solidQuad(pt(A, Ra, Ua,-kBallastTopHalf, kBallastTopZ),
                      pt(A, Ra, Ua,-kBallastBotHalf, kBallastBotZ),
                      pt(B, Rb, Ub,-kBallastBotHalf, kBallastBotZ),
                      pt(B, Rb, Ub,-kBallastTopHalf, kBallastTopZ), mid, kBallastSide);
            solidQuad(pt(A, Ra, Ua,kBallastTopHalf, kBallastTopZ),
                      pt(A, Ra, Ua,kBallastBotHalf, kBallastBotZ),
                      pt(B, Rb, Ub,kBallastBotHalf, kBallastBotZ),
                      pt(B, Rb, Ub,kBallastTopHalf, kBallastTopZ), mid, kBallastSide);

            // Two rails: top + two vertical sides each.
            for (float sign : {-1.0f, 1.0f}) {
                const float rc = sign * kRailHalf;
                const glm::vec3 rmid =
                    (pt(A, Ra, Ua,rc, (kRailBotZ + kRailTopZ) * 0.5f) +
                     pt(B, Rb, Ub,rc, (kRailBotZ + kRailTopZ) * 0.5f)) *
                    0.5f;
                solidQuad(pt(A, Ra, Ua,rc - kRailHalfWidth, kRailTopZ),
                          pt(A, Ra, Ua,rc + kRailHalfWidth, kRailTopZ),
                          pt(B, Rb, Ub,rc + kRailHalfWidth, kRailTopZ),
                          pt(B, Rb, Ub,rc - kRailHalfWidth, kRailTopZ), rmid, kRailCol);
                solidQuad(pt(A, Ra, Ua,rc - kRailHalfWidth, kRailTopZ),
                          pt(A, Ra, Ua,rc - kRailHalfWidth, kRailBotZ),
                          pt(B, Rb, Ub,rc - kRailHalfWidth, kRailBotZ),
                          pt(B, Rb, Ub,rc - kRailHalfWidth, kRailTopZ), rmid, kRailCol);
                solidQuad(pt(A, Ra, Ua,rc + kRailHalfWidth, kRailTopZ),
                          pt(A, Ra, Ua,rc + kRailHalfWidth, kRailBotZ),
                          pt(B, Rb, Ub,rc + kRailHalfWidth, kRailBotZ),
                          pt(B, Rb, Ub,rc + kRailHalfWidth, kRailTopZ), rmid, kRailCol);
            }
        }
    }
    closeAlways(); // the last piece has no successor to close it
    alwaysIndexCount_ = static_cast<std::uint32_t>(indices_.size());

    // --- Pass 2: sleeper boxes, grouped into distance-culled chunks --------
    auto emitSleeper = [&](const glm::vec3& P, const glm::vec3& R,
                           const glm::vec3& U, const glm::vec3& T) {
        const float L = kSleeperHalfLen, W = kSleeperHalfWid;
        const float zb = kSleeperBotZ, zt = kSleeperTopZ;
        const glm::vec3 inside = P + U * ((zb + zt) * 0.5f);
        auto s = [&](float a, float b, float z) {
            return P + R * a + T * b + U * z;
        };
        solidQuad(s(-L, -W, zt), s(L, -W, zt), s(L, W, zt), s(-L, W, zt), inside,
                  kSleeperCol);
        solidQuad(s(L, -W, zt), s(L, -W, zb), s(L, W, zb), s(L, W, zt), inside,
                  kSleeperCol);
        solidQuad(s(-L, -W, zt), s(-L, -W, zb), s(-L, W, zb), s(-L, W, zt), inside,
                  kSleeperCol);
        solidQuad(s(-L, W, zt), s(-L, W, zb), s(L, W, zb), s(L, W, zt), inside,
                  kSleeperCol);
        solidQuad(s(-L, -W, zt), s(-L, -W, zb), s(L, -W, zb), s(L, -W, zt), inside,
                  kSleeperCol);
    };

    for (const Run& run : near) {
        const TrackPath& path = *run.path;
        const float total = run.to;
        std::uint32_t chunkFirst = static_cast<std::uint32_t>(indices_.size());
        glm::vec3 accum(0.0f);
        int accumN = 0;
        auto flush = [&]() {
            if (accumN == 0) return;
            TrackDrawChunk ch;
            ch.firstIndex = chunkFirst;
            ch.indexCount =
                static_cast<std::uint32_t>(indices_.size()) - chunkFirst;
            ch.centroid = accum / static_cast<float>(accumN);
            chunks_.push_back(ch);
            chunkFirst = static_cast<std::uint32_t>(indices_.size());
            accum = glm::vec3(0.0f);
            accumN = 0;
        };

        for (float dist = run.from; dist <= total + 1e-3f; dist += kSleeperSpacing) {
            const TrackPose p = path.poseAt(dist);
            emitSleeper(p.pos, p.right, p.up, p.tangent);
            accum += p.pos;
            ++accumN;
            if (accumN >= kSleepersPerChunk) flush();
        }
        flush(); // end of track
    }

    std::printf(
        "[TrackMesh] %zu track runs, %zu vertices, %zu triangles, %zu sleeper chunks\n",
        near.size(), vertices_.size(), indices_.size() / 3, chunks_.size());
}
