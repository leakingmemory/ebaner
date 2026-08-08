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

    int cur = 0; // the last limit seen; 0 until one is
    for (const TrackPath::SpeedPoint& sp : pts) {
        // A point that says nothing keeps what was in force. Nothing in force yet means the
        // path began undefined, which is what the default is for.
        const int kmh = sp.kmh != 0 ? static_cast<int>(sp.kmh) : (cur != 0 ? cur : kDefaultKmh);
        if (out.empty() || kmh != out.back().kmh) out.push_back({sp.s, kmh});
        cur = kmh;
    }
    return out;
}

std::vector<SpeedSign> speedIncreaseSigns(const std::vector<TrackPath>& paths) {
    std::vector<SpeedSign> out;
    for (const TrackPath& p : paths) {
        for (int dir = 1; dir >= -1; dir -= 2) {
            const std::vector<SpeedStretch> st = resolveSpeeds(p, dir);
            // From index 1: the first stretch is where the path starts, not a rise.
            for (std::size_t i = 1; i < st.size(); ++i) {
                if (st[i].kmh <= st[i - 1].kmh) continue; // unchanged, or a drop
                const TrackPose pose = p.poseAt(st[i].s);
                SpeedSign s;
                s.pos = pose.pos;
                s.tangent = pose.tangent * static_cast<float>(dir);
                // Right of travel, derived from the heading rather than taken from
                // TrackPose::right - that one is banked by cant and runs the other way
                // round, which would stand every sign on the wrong side of the line.
                s.right = glm::vec3(s.tangent.y, -s.tangent.x, 0.0f);
                if (const float L = glm::length(s.right); L > 1e-6f) s.right /= L;
                s.kmh = st[i].kmh;
                out.push_back(s);
            }
        }
    }
    return out;
}
