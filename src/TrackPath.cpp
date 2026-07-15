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

#include "TrackPath.h"

#include "TerrainData.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <unordered_map>
#include <unordered_set>

namespace {
constexpr int kSubStepsPerSpan = 20; // arc-length table resolution per span
constexpr float kCantFactor = 0.6f;  // design fraction (cant deficiency)
constexpr float kMaxCant = 0.105f;   // ~6 deg (150 mm over 1.435 m gauge)
constexpr float kGravity = 9.81f;
constexpr float kCantSmoothM = 22.0f; // cant-transition smoothing window (m)
} // namespace

TrackPath::TrackPath(std::uint32_t trackId, std::uint8_t trackType,
                     const std::vector<glm::vec3>& pts,
                     const std::vector<std::uint16_t>& speed)
    : speed_(speed), trackId_(trackId), trackType_(trackType) {
    const int n = static_cast<int>(pts.size());
    // Control points with reflected phantom endpoints, so span i interpolates
    // ctrl_[i+1]..ctrl_[i+2] using neighbours ctrl_[i] and ctrl_[i+3].
    ctrl_.reserve(n + 2);
    ctrl_.push_back(2.0f * pts.front() - pts[1]); // phantom before start
    for (const glm::vec3& p : pts) ctrl_.push_back(p);
    ctrl_.push_back(2.0f * pts.back() - pts[n - 2]); // phantom after end

    // Centripetal knots (alpha = 0.5).
    knot_.resize(ctrl_.size());
    knot_[0] = 0.0f;
    for (std::size_t i = 1; i < ctrl_.size(); ++i) {
        const float d = glm::distance(ctrl_[i], ctrl_[i - 1]);
        knot_[i] = knot_[i - 1] + std::sqrt(std::max(d, 1e-4f));
    }

    // Arc-length table: g = span + local-u in [0, numSpans]; s = cumulative length.
    const int numSpans = static_cast<int>(ctrl_.size()) - 3;
    table_.push_back({0.0f, 0.0f, 0.0f});
    glm::vec3 prev;
    eval(0, 0.0f, &prev, nullptr, nullptr);
    float s = 0.0f;
    for (int span = 0; span < numSpans; ++span) {
        for (int k = 1; k <= kSubStepsPerSpan; ++k) {
            const float u = static_cast<float>(k) / kSubStepsPerSpan;
            glm::vec3 p;
            eval(span, u, &p, nullptr, nullptr);
            s += glm::distance(p, prev);
            prev = p;
            table_.push_back({s, static_cast<float>(span) + u, 0.0f});
        }
    }
    length_ = table_.empty() ? 0.0f : table_.back().s;

    // Precompute a smoothed superelevation (cant) per sample from the design
    // speed and curvature: theta = sign(k) * min(factor*atan(v^2*|k|/g), maxCant).
    std::vector<float> raw(table_.size(), 0.0f);
    const int npts = static_cast<int>(speed_.size());
    for (std::size_t i = 0; i < table_.size(); ++i) {
        const float g = table_[i].g;
        const int span = std::clamp(static_cast<int>(std::floor(g)), 0,
                                    std::max(numSpans - 1, 0));
        const float u = std::clamp(g - static_cast<float>(span), 0.0f, 1.0f);
        glm::vec3 d1, d2;
        eval(span, u, nullptr, &d1, &d2);
        const float denom = std::pow(d1.x * d1.x + d1.y * d1.y, 1.5f);
        const float kappa =
            (denom > 1e-8f) ? (d1.x * d2.y - d1.y * d2.x) / denom : 0.0f;
        float v = 0.0f; // m/s at the nearer surveyed point's speed
        if (npts > 0) {
            int idx = std::clamp((u < 0.5f) ? span : span + 1, 0, npts - 1);
            std::uint16_t sp = speed_[idx];
            if (sp == 0) {
                const int j = std::clamp((idx == span) ? span + 1 : span, 0, npts - 1);
                sp = speed_[j];
            }
            v = static_cast<float>(sp) / 3.6f;
        }
        if (v > 0.0f) {
            const float mag =
                std::atan(kCantFactor * v * v * std::abs(kappa) / kGravity);
            raw[i] = std::copysign(std::min(mag, kMaxCant), kappa);
        }
    }
    // Box-smooth over ~kCantSmoothM metres so cant ramps in/out (table sorted by s).
    for (std::size_t i = 0; i < table_.size(); ++i) {
        const float s0 = table_[i].s;
        float sum = 0.0f;
        int cnt = 0;
        for (int j = static_cast<int>(i); j >= 0 && s0 - table_[j].s <= kCantSmoothM; --j) {
            sum += raw[j];
            ++cnt;
        }
        for (std::size_t j = i + 1; j < table_.size() && table_[j].s - s0 <= kCantSmoothM; ++j) {
            sum += raw[j];
            ++cnt;
        }
        table_[i].cant = (cnt > 0) ? sum / static_cast<float>(cnt) : raw[i];
    }
}

void TrackPath::eval(int span, float u, glm::vec3* p, glm::vec3* d1,
                     glm::vec3* d2) const {
    const glm::vec3 P0 = ctrl_[span], P1 = ctrl_[span + 1];
    const glm::vec3 P2 = ctrl_[span + 2], P3 = ctrl_[span + 3];
    const float t0 = knot_[span], t1 = knot_[span + 1];
    const float t2 = knot_[span + 2], t3 = knot_[span + 3];

    // Non-uniform Catmull-Rom tangents (in global knot space) at P1 and P2.
    const glm::vec3 m1 =
        (P1 - P0) / (t1 - t0) - (P2 - P0) / (t2 - t0) + (P2 - P1) / (t2 - t1);
    const glm::vec3 m2 =
        (P2 - P1) / (t2 - t1) - (P3 - P1) / (t3 - t1) + (P3 - P2) / (t3 - t2);
    const float dt = t2 - t1;
    const glm::vec3 M1 = m1 * dt, M2 = m2 * dt; // scaled to local u in [0,1]

    const float u2 = u * u, u3 = u2 * u;
    if (p) {
        const float h00 = 2 * u3 - 3 * u2 + 1, h10 = u3 - 2 * u2 + u;
        const float h01 = -2 * u3 + 3 * u2, h11 = u3 - u2;
        *p = h00 * P1 + h10 * M1 + h01 * P2 + h11 * M2;
    }
    if (d1) {
        const float g00 = 6 * u2 - 6 * u, g10 = 3 * u2 - 4 * u + 1;
        const float g01 = -6 * u2 + 6 * u, g11 = 3 * u2 - 2 * u;
        *d1 = g00 * P1 + g10 * M1 + g01 * P2 + g11 * M2;
    }
    if (d2) {
        const float k00 = 12 * u - 6, k10 = 6 * u - 4;
        const float k01 = -12 * u + 6, k11 = 6 * u - 2;
        *d2 = k00 * P1 + k10 * M1 + k01 * P2 + k11 * M2;
    }
}

void TrackPath::locate(float s, int& span, float& u, float& cant) const {
    const int numSpans = static_cast<int>(ctrl_.size()) - 3;
    cant = 0.0f;
    if (numSpans < 1 || table_.size() < 2 || length_ <= 0.0f) {
        span = -1;
        u = 0.0f;
        return;
    }
    s = std::clamp(s, 0.0f, length_);
    // Binary search the arc-length table for the bracket around s.
    int lo = 0, hi = static_cast<int>(table_.size()) - 1;
    while (hi - lo > 1) {
        const int mid = (lo + hi) / 2;
        if (table_[mid].s <= s)
            lo = mid;
        else
            hi = mid;
    }
    const Sample& a = table_[lo];
    const Sample& b = table_[hi];
    const float frac = (b.s > a.s) ? (s - a.s) / (b.s - a.s) : 0.0f;
    const float g = a.g + frac * (b.g - a.g);
    span = std::clamp(static_cast<int>(std::floor(g)), 0, numSpans - 1);
    u = std::clamp(g - static_cast<float>(span), 0.0f, 1.0f);
    cant = a.cant + frac * (b.cant - a.cant);
}

TrackPose TrackPath::poseAt(float s) const {
    TrackPose pose;
    int span;
    float u, cant;
    locate(s, span, u, cant);
    if (span < 0) {
        pose.pos = ctrl_.size() > 1 ? ctrl_[1] : glm::vec3(0.0f);
        pose.tangent = glm::vec3(1.0f, 0.0f, 0.0f);
        pose.right = glm::vec3(0.0f, 1.0f, 0.0f);
        pose.up = glm::vec3(0.0f, 0.0f, 1.0f);
        pose.curvature = 0.0f;
        pose.cant = 0.0f;
        return pose;
    }

    glm::vec3 P, D1, D2;
    eval(span, u, &P, &D1, &D2);
    pose.pos = P;

    const float tl = glm::length(D1);
    pose.tangent = (tl > 1e-8f) ? D1 / tl : glm::vec3(1.0f, 0.0f, 0.0f);

    const glm::vec2 th(pose.tangent.x, pose.tangent.y);
    const float thl = glm::length(th);
    const glm::vec2 r =
        (thl > 1e-8f) ? glm::vec2(-th.y, th.x) / thl : glm::vec2(0.0f, 1.0f);
    const glm::vec3 r0(r.x, r.y, 0.0f); // horizontal cross-track right

    // Track-normal up (perpendicular to tangent and right), oriented +z.
    glm::vec3 u0 = glm::cross(r0, pose.tangent);
    if (glm::length(u0) < 1e-8f) u0 = glm::vec3(0.0f, 0.0f, 1.0f);
    u0 = glm::normalize(u0);
    if (u0.z < 0.0f) u0 = -u0;

    // Roll the cross-track frame about the tangent by the cant angle (banks into
    // the curve: outer rail rises).
    const float c = std::cos(cant), sn = std::sin(cant);
    pose.right = c * r0 + sn * u0;
    pose.up = -sn * r0 + c * u0;
    pose.cant = cant;

    // Horizontal signed curvature (x'y'' - y'x'') / |xy'|^3.
    const float xp = D1.x, yp = D1.y, xpp = D2.x, ypp = D2.y;
    const float denom = std::pow(xp * xp + yp * yp, 1.5f);
    pose.curvature = (denom > 1e-8f) ? (xp * ypp - yp * xpp) / denom : 0.0f;
    return pose;
}

float TrackPath::speedLimitAt(float s) const {
    if (speed_.empty()) return 0.0f;
    int span;
    float u, cant;
    locate(s, span, u, cant);
    const int n = static_cast<int>(speed_.size());
    if (span < 0) return static_cast<float>(speed_.front());
    // speed_ is per surveyed point; a span runs point[span]..point[span+1].
    // Line speed is piecewise-constant, so snap to the nearer endpoint.
    int i = std::clamp((u < 0.5f) ? span : span + 1, 0, n - 1);
    if (speed_[i] != 0) return static_cast<float>(speed_[i]);
    // Unknown here: fall back to the other endpoint of the span.
    int j = std::clamp((i == span) ? span + 1 : span, 0, n - 1);
    return static_cast<float>(speed_[j]);
}

std::vector<TrackPath> buildTrackPaths(const TerrainData& data) {
    const glm::dvec3 origin = data.sceneOrigin();
    std::unordered_set<std::uint32_t> seen;
    std::vector<TrackPath> paths;
    std::size_t mainVerts = 0, knownVerts = 0; // speed-plumbing summary
    int minSpeed = 0, maxSpeed = 0;
    std::unordered_map<int, int> speedHist;
    for (const Tile& t : data.tiles()) {
        for (const TrackSegment& seg : t.tracks) {
            if (!seen.insert(seg.trackId).second) continue; // one per through-track
            if (seg.pts.size() < 2) continue;
            std::vector<glm::vec3> pts;
            std::vector<std::uint16_t> speed;
            pts.reserve(seg.pts.size());
            speed.reserve(seg.pts.size());
            for (std::size_t k = 0; k < seg.pts.size(); ++k) {
                const glm::dvec3& w = seg.pts[k];
                const glm::vec3 p(static_cast<float>(w.x - origin.x),
                                  static_cast<float>(w.y - origin.y),
                                  static_cast<float>(w.z - origin.z));
                if (pts.empty() || glm::distance(pts.back(), p) > 1e-3f) {
                    pts.push_back(p); // drop coincident points (and their speed)
                    speed.push_back(k < seg.speed.size() ? seg.speed[k] : 0);
                }
            }
            if (pts.size() < 2) continue;
            if (seg.trackType == 0) { // main line: tally speed coverage
                for (std::uint16_t sp : speed) {
                    ++mainVerts;
                    if (sp > 0) {
                        ++knownVerts;
                        ++speedHist[sp];
                        if (minSpeed == 0 || sp < minSpeed) minSpeed = sp;
                        if (sp > maxSpeed) maxSpeed = sp;
                    }
                }
            }
            paths.emplace_back(seg.trackId, seg.trackType, pts, speed);
        }
    }

    int modal = 0, modalCount = 0;
    for (const auto& [sp, c] : speedHist)
        if (c > modalCount) { modalCount = c; modal = sp; }
    const double pct =
        mainVerts ? 100.0 * static_cast<double>(knownVerts) / mainVerts : 0.0;
    std::printf("[TrackPath] %zu paths; main-line speed known on %.0f%% of "
                "vertices (%zu/%zu); range %d-%d km/h, modal %d\n",
                paths.size(), pct, knownVerts, mainVerts, minSpeed, maxSpeed,
                modal);
    return paths;
}
