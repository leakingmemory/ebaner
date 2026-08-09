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
#include <cstdint>
#include <cstdio>
#include <deque>
#include <unordered_map>
#include <unordered_set>
#include <vector>

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

    bmin_ = bmax_ = ctrl_.front();
    for (const glm::vec3& c : ctrl_) {
        bmin_ = glm::min(bmin_, c);
        bmax_ = glm::max(bmax_, c);
    }

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
    // Cross-track, and *left* of travel despite the name the frame carries: rotating the
    // tangent a quarter turn this way is counter-clockwise, and with z up that is the left
    // hand. Everything downstream is symmetric about it, so the naming costs nothing there -
    // but the roll below is not symmetric, and has to be applied knowing which side this is.
    const glm::vec3 r0(r.x, r.y, 0.0f);

    // Track-normal up (perpendicular to tangent and right), oriented +z.
    glm::vec3 u0 = glm::cross(r0, pose.tangent);
    if (glm::length(u0) < 1e-8f) u0 = glm::vec3(0.0f, 0.0f, 1.0f);
    u0 = glm::normalize(u0);
    if (u0.z < 0.0f) u0 = -u0;

    // Roll the cross-track frame about the tangent by the cant angle, so that the track
    // banks *into* the curve: the outer rail rises. Cant carries the sign of the curvature
    // it was derived from - positive curving left - and on a left-hand curve r0 is the inner
    // side, so a positive cant has to take r0 down rather than up.
    const float c = std::cos(cant), sn = std::sin(cant);
    pose.right = c * r0 - sn * u0;
    pose.up = sn * r0 + c * u0;
    pose.cant = cant;

    // Horizontal signed curvature (x'y'' - y'x'') / |xy'|^3.
    const float xp = D1.x, yp = D1.y, xpp = D2.x, ypp = D2.y;
    const float denom = std::pow(xp * xp + yp * yp, 1.5f);
    pose.curvature = (denom > 1e-8f) ? (xp * ypp - yp * xpp) / denom : 0.0f;
    return pose;
}

std::vector<TrackPath::SpeedPoint> TrackPath::speedPoints() const {
    std::vector<SpeedPoint> out;
    const int numSpans = static_cast<int>(ctrl_.size()) - 3;
    if (speed_.empty() || numSpans < 1 || table_.size() < 2) return out;
    // Surveyed point i is the start of span i, so it sits at global parameter g = i. The
    // arc-length table runs the other way (s -> g), so walk it once and read off the s
    // where g crosses each whole number.
    out.reserve(speed_.size());
    std::size_t t = 1;
    for (std::size_t i = 0; i < speed_.size(); ++i) {
        const float g = static_cast<float>(i);
        while (t + 1 < table_.size() && table_[t].g < g) ++t;
        const Sample& a = table_[t - 1];
        const Sample& b = table_[t];
        const float frac = (b.g > a.g) ? (g - a.g) / (b.g - a.g) : 0.0f;
        out.push_back({std::clamp(a.s + frac * (b.s - a.s), 0.0f, length_), speed_[i]});
    }
    return out;
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

namespace {
// One exported track segment (scene-relative), before chaining.
struct Seg {
    std::vector<glm::vec3> pts;
    std::vector<std::uint16_t> speed;
    std::uint8_t trackType = 0;
    std::uint32_t trackId = 0;
};

// The exporter splits each line into separate segments at medium transitions
// (surface / tunnel / bridge), each with its own trackId. Join segments that meet
// end-to-end at a shared node (within kJoinTol) into continuous routes, so the
// vehicle can run through tunnels instead of derailing off the end of a segment.
constexpr float kJoinTol = 1.0f; // m; conservative so only coincident ends join

// Union-find over segment endpoints.
struct DSU {
    std::vector<int> p;
    void init(int n) { p.resize(n); for (int i = 0; i < n; ++i) p[i] = i; }
    int find(int x) { while (p[x] != x) x = p[x] = p[p[x]]; return x; }
    void unite(int a, int b) { p[find(a)] = find(b); }
};
} // namespace

std::vector<TrackPath> buildTrackPaths(const TerrainData& data) {
    const glm::dvec3 origin = data.sceneOrigin();
    std::size_t mainVerts = 0, knownVerts = 0; // speed-plumbing summary
    int minSpeed = 0, maxSpeed = 0;
    std::unordered_map<int, int> speedHist;

    // --- Collect the segments, scene-relative. ---
    // The network is the whole dataset's track, already one copy per trackId, so the
    // paths span the line rather than whatever terrain happens to be loaded.
    std::vector<Seg> segs;
    {
        for (const TrackSegment& seg : data.networkTracks()) {
            if (seg.pts.size() < 2) continue;
            Seg s;
            s.trackType = seg.trackType;
            s.trackId = seg.trackId;
            s.pts.reserve(seg.pts.size());
            s.speed.reserve(seg.pts.size());
            for (std::size_t k = 0; k < seg.pts.size(); ++k) {
                const glm::dvec3& w = seg.pts[k];
                const glm::vec3 p(static_cast<float>(w.x - origin.x),
                                  static_cast<float>(w.y - origin.y),
                                  static_cast<float>(w.z - origin.z));
                if (s.pts.empty() || glm::distance(s.pts.back(), p) > 1e-3f) {
                    s.pts.push_back(p); // drop coincident points (and their speed)
                    s.speed.push_back(k < seg.speed.size() ? seg.speed[k] : 0);
                }
            }
            if (s.pts.size() < 2) continue;
            if (s.trackType == 0) { // main line: tally speed coverage
                for (std::uint16_t sp : s.speed) {
                    ++mainVerts;
                    if (sp > 0) {
                        ++knownVerts;
                        ++speedHist[sp];
                        if (minSpeed == 0 || sp < minSpeed) minSpeed = sp;
                        if (sp > maxSpeed) maxSpeed = sp;
                    }
                }
            }
            segs.push_back(std::move(s));
        }
    }

    // --- Group endpoints into shared nodes (union endpoints within kJoinTol). ---
    // Endpoint index e = 2*seg + which (which: 0 = front, 1 = back).
    const int nEnds = 2 * static_cast<int>(segs.size());
    auto endPt = [&](int e) -> const glm::vec3& {
        return (e & 1) ? segs[e >> 1].pts.back() : segs[e >> 1].pts.front();
    };
    DSU dsu;
    dsu.init(nEnds);
    {
        std::unordered_map<std::int64_t, std::vector<int>> cell;
        auto key = [](int cx, int cy) {
            return (std::int64_t(cx) << 32) ^ std::int64_t(std::uint32_t(cy));
        };
        for (int e = 0; e < nEnds; ++e) {
            const glm::vec3& q = endPt(e);
            const int cx = static_cast<int>(std::floor(q.x / kJoinTol));
            const int cy = static_cast<int>(std::floor(q.y / kJoinTol));
            for (int dx = -1; dx <= 1; ++dx)
                for (int dy = -1; dy <= 1; ++dy) {
                    auto it = cell.find(key(cx + dx, cy + dy));
                    if (it == cell.end()) continue;
                    for (int f : it->second)
                        if (std::hypot(q.x - endPt(f).x, q.y - endPt(f).y) <= kJoinTol)
                            dsu.unite(e, f);
                }
            cell[key(cx, cy)].push_back(e);
        }
    }
    // node root -> endpoint indices meeting there.
    std::unordered_map<int, std::vector<int>> node;
    for (int e = 0; e < nEnds; ++e) node[dsu.find(e)].push_back(e);

    // --- Partner links: chain segments head-to-tail through a node. ---
    // At a plain degree-2 join the two ends partner (a track continuing, possibly
    // curving). At a multi-way junction (>=3 ends, a turnout) the through route is the
    // straightest same-type pair; partnering it lets a siding continue across the
    // junction instead of dead-ending there (the others stay as diverging branches,
    // which SwitchNetwork then picks up as switches).
    // Unit direction from an endpoint into its own segment (i.e. away from the node).
    auto dirOut = [&](int e) -> glm::vec2 {
        const Seg& s = segs[e >> 1];
        const glm::vec3& tip = (e & 1) ? s.pts.back() : s.pts.front();
        const glm::vec3& nb = (e & 1) ? s.pts[s.pts.size() - 2] : s.pts[1];
        glm::vec2 d(nb.x - tip.x, nb.y - tip.y);
        const float L = glm::length(d);
        return L > 1e-6f ? d / L : glm::vec2(1.0f, 0.0f);
    };
    auto joinable = [&](int a, int b) {
        return (a >> 1) != (b >> 1) &&                 // not the same segment (tiny loop)
               segs[a >> 1].trackType == segs[b >> 1].trackType;
    };
    std::vector<int> partner(nEnds, -1);
    for (auto& [root, ends] : node) {
        if (ends.size() < 2) continue;                 // dead end
        if (ends.size() == 2) {
            const int a = ends[0], b = ends[1];
            if (joinable(a, b)) { partner[a] = b; partner[b] = a; }
            continue;
        }
        // Multi-way: chain the straightest same-type pair (dirOut nearly opposite, so
        // the track runs through). Require it to be reasonably straight so a sharp branch
        // is never taken for the through route.
        int ba = -1, bb = -1;
        float best = -0.7f;                            // <= this => through (<~45 deg bend)
        for (std::size_t p = 0; p < ends.size(); ++p)
            for (std::size_t q = p + 1; q < ends.size(); ++q) {
                const int a = ends[p], b = ends[q];
                if (!joinable(a, b)) continue;
                const float d = glm::dot(dirOut(a), dirOut(b));
                if (d < best) { best = d; ba = a; bb = b; }
            }
        if (ba >= 0) { partner[ba] = bb; partner[bb] = ba; }
    }

    // --- Chain segments head-to-tail through partner links into routes. ---
    std::vector<TrackPath> paths;
    std::vector<char> used(segs.size(), 0);
    for (int i = 0; i < static_cast<int>(segs.size()); ++i) {
        if (used[i]) continue;
        std::deque<std::pair<int, bool>> chain; // (segment, reversed)
        chain.push_back({i, false});
        used[i] = 1;
        // Extend forward from the tail.
        for (;;) {
            auto [sg, flip] = chain.back();
            const int tail = flip ? (2 * sg) : (2 * sg + 1);
            const int p = partner[tail];
            if (p < 0 || used[p >> 1]) break;
            const int j = p >> 1;
            chain.push_back({j, (p & 1) == 1}); // reverse j if its back connects
            used[j] = 1;
        }
        // Extend backward from the head.
        for (;;) {
            auto [sg, flip] = chain.front();
            const int head = flip ? (2 * sg + 1) : (2 * sg);
            const int p = partner[head];
            if (p < 0 || used[p >> 1]) break;
            const int j = p >> 1;
            chain.push_front({j, (p & 1) == 0}); // reverse j if its front connects
            used[j] = 1;
        }

        // Concatenate (dropping the duplicated join vertex, then any coincident).
        std::vector<glm::vec3> pts;
        std::vector<std::uint16_t> speed;
        for (std::size_t c = 0; c < chain.size(); ++c) {
            const auto [sg, flip] = chain[c];
            const Seg& s = segs[sg];
            const int n = static_cast<int>(s.pts.size());
            for (int k = 0; k < n; ++k) {
                if (c > 0 && k == 0) continue; // shared node with previous segment
                const int kk = flip ? (n - 1 - k) : k;
                const glm::vec3& P = s.pts[kk];
                if (!pts.empty() && glm::distance(pts.back(), P) <= 1e-3f) continue;
                pts.push_back(P);
                speed.push_back(kk < static_cast<int>(s.speed.size()) ? s.speed[kk] : 0);
            }
        }
        if (pts.size() < 2) continue;
        paths.emplace_back(segs[i].trackId, segs[i].trackType, pts, speed);
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
