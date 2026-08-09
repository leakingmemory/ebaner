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

#include "SpeedLimits.h"

#include <algorithm>

std::vector<SpeedStretch> resolveSpeeds(const TrackPath& p, int dir) {
    std::vector<SpeedStretch> out;
    std::vector<TrackPath::SpeedPoint> pts = p.speedPoints();
    if (pts.empty()) return out;
    if (dir < 0) std::reverse(pts.begin(), pts.end()); // read the path the other way

    int cur = 0;         // the last limit seen; 0 until one is
    float prevS = 0.0f;  // where the previous point sat
    for (const TrackPath::SpeedPoint& sp : pts) {
        // A point that says nothing keeps what was in force. Nothing in force yet means the
        // path began undefined, which is what the default is for.
        const int kmh = sp.kmh != 0 ? static_cast<int>(sp.kmh) : (cur != 0 ? cur : kDefaultKmh);
        if (out.empty()) {
            out.push_back({sp.s, kmh}); // the path starts here, whatever it starts at
        } else if (kmh != out.back().kmh) {
            // The limit changes somewhere *between* the two points, not at either of them -
            // the data attaches a value to a point, not to a span. Splitting the difference
            // is what makes the two directions agree: taking the point that carries the new
            // value instead would put the boundary a whole span apart each way, so a rise
            // and the opposing drop could never share a post. Where a gap intervenes they
            // still differ, which is the carry-forward rule working rather than an artefact.
            out.push_back({0.5f * (prevS + sp.s), kmh});
        }
        cur = kmh;
        prevS = sp.s;
    }
    return out;
}

float warningDistance(int fromKmh, int toKmh) {
    const float v1 = static_cast<float>(fromKmh) / 3.6f;
    const float v2 = static_cast<float>(toKmh) / 3.6f;
    const float brake = std::max(0.0f, (v1 * v1 - v2 * v2) / (2.0f * kServiceDecel));
    const float need = kSightingM + brake;
    for (const float band : kWarnBands)
        if (need <= band) return band;
    return kWarnBands[std::size(kWarnBands) - 1]; // the longest band is also the cap
}

std::vector<SpeedSign> speedSigns(const std::vector<TrackPath>& paths,
                                  const glm::vec3& centre, float radius) {
    std::vector<SpeedSign> out;
    for (const TrackPath& p : paths) {
        if (radius > 0.0f && !p.nearXY(glm::vec2(centre), radius)) continue;
        for (int dir = 1; dir >= -1; dir -= 2) {
            const std::vector<SpeedStretch> st = resolveSpeeds(p, dir);
            const float d = static_cast<float>(dir);
            auto place = [&](float s, int kmh, SpeedSignKind kind) {
                const TrackPose pose = p.poseAt(s);
                SpeedSign sg;
                sg.pos = pose.pos;
                sg.tangent = pose.tangent * d;
                // Right of travel, derived from the heading rather than taken from
                // TrackPose::right - that one is banked by cant and runs the other way
                // round, which would stand every sign on the wrong side of the line.
                sg.right = glm::vec3(sg.tangent.y, -sg.tangent.x, 0.0f);
                if (const float L = glm::length(sg.right); L > 1e-6f) sg.right /= L;
                sg.kmh = kmh;
                sg.kind = kind;
                out.push_back(sg);
            };
            // From index 1: the first stretch is where the path starts, not a change.
            for (std::size_t i = 1; i < st.size(); ++i) {
                if (st[i].kmh > st[i - 1].kmh) {
                    place(st[i].s, st[i].kmh, SpeedSignKind::Increase);
                    continue;
                }
                if (st[i].kmh >= st[i - 1].kmh) continue; // unchanged
                place(st[i].s, st[i].kmh, SpeedSignKind::ReductionMarker);
                // The warning stands a braking distance back along the way we came. Where
                // that runs off the start of the path it is clamped rather than dropped: one
                // standing a little close is more use than none, and the path boundary is an
                // import artefact anyway.
                const float back = warningDistance(st[i - 1].kmh, st[i].kmh);
                const float ws = std::clamp(st[i].s - d * back, 0.0f, p.length());
                place(ws, st[i].kmh, SpeedSignKind::ReductionWarning);
            }
        }
    }
    // A point where the limit changes is a rise one way and a drop the other, so a marker and
    // an opposing increase often stand at the same spot - one post, a face each. They only
    // coincide when the transition sits between two adjacent defined points; with a gap in
    // between, the two directions resolve it at different places and each keeps its own post.
    constexpr float kSamePost = 3.0f; // m
    for (SpeedSign& inc : out) {
        if (inc.kind != SpeedSignKind::Increase || inc.backMarker) continue;
        for (SpeedSign& mk : out) {
            if (mk.kind != SpeedSignKind::ReductionMarker || mk.kmh < 0) continue;
            if (glm::length(mk.pos - inc.pos) > kSamePost) continue;
            if (glm::dot(mk.tangent, inc.tangent) > -0.9f) continue; // not the opposing way
            inc.backMarker = true;
            mk.kmh = -1; // fold it onto the increase and drop it below
            break;
        }
    }
    out.erase(std::remove_if(out.begin(), out.end(),
                             [](const SpeedSign& s) {
                                 return s.kind == SpeedSignKind::ReductionMarker && s.kmh < 0;
                             }),
              out.end());
    // Clip by where each sign stands, not by which path it belongs to: the main line is
    // one path running the length of the country, so a path that comes near here still
    // has limit changes hundreds of kilometres away, and each would post a sign.
    if (radius > 0.0f)
        out.erase(std::remove_if(out.begin(), out.end(),
                                 [&](const SpeedSign& s) {
                                     return std::hypot(s.pos.x - centre.x,
                                                       s.pos.y - centre.y) > radius;
                                 }),
                  out.end());
    return out;
}
