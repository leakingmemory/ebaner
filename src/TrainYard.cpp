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

#include "TrainYard.h"

#include "Consist.h"
#include "Occupancy.h"
#include "TrackPath.h"
#include "Vehicle.h"

#include <algorithm>
#include <limits>

namespace {

// Half the train measured over its outermost axles - which is what has to be on the rail.
// The body overhangs beyond them and may hang over a buffer stop without anything caring,
// but an axle past the end of the path is pinned by the arc-length clamp.
float outerHalf(const VehicleSpec& spec) {
    return 0.5f * (spec.bogieSpacing + spec.wheelbase) +
           static_cast<float>(std::max(0, spec.units - 1)) * 0.5f *
               (spec.length + Consist::kCouplerGap);
}

} // namespace

Placement placeTrainNear(const std::vector<TrackPath>& paths, glm::vec2 targetXY,
                         const VehicleSpec& spec, float reach, float maxDist) {
    Placement out;
    out.half = outerHalf(spec);
    const float margin = out.half + 1.0f;

    // Two passes, near then everywhere, for the same reason the start-up spawn does it:
    // sampling every path in the country at 5 m is the better part of a million poseAt
    // calls, and the answer is nearly always within sight of where it was asked for.
    bool sawAnyMain = false;
    float best = std::numeric_limits<float>::max();
    for (const float r : {reach, 0.0f}) {
        for (std::size_t i = 0; i < paths.size(); ++i) {
            const TrackPath& p = paths[i];
            if (p.trackType() != 0) continue; // main line: a siding is not where you park
            sawAnyMain = true;
            if (r > 0.0f && !p.nearXY(targetXY, r)) continue;
            const float L = p.length();
            if (L <= 2.0f * margin) continue; // no room on this one for a train this long
            for (float s = margin; s <= L - margin; s += 5.0f) {
                const glm::vec3 q = p.poseAt(s).pos;
                const float dx = q.x - targetXY.x, dy = q.y - targetXY.y;
                const float d2 = dx * dx + dy * dy;
                if (d2 >= best) continue;
                best = d2;
                out.path = &p;
                out.pathIdx = static_cast<int>(i);
                out.s = s;
            }
        }
        if (out.path != nullptr || r == 0.0f) break;
    }

    if (out.path != nullptr && best > maxDist * maxDist) {
        // Something was found, but not where it was asked for. This is what asking for a
        // station on a line the export does not carry looks like.
        out = Placement{};
        out.half = outerHalf(spec);
        out.why = "no line near there";
    } else if (out.path == nullptr) {
        // Which of the two it is matters to whoever asked: a train too long for the line is
        // a different problem from being nowhere near one.
        out.why = sawAnyMain ? "no main line long enough for this train"
                             : "no main line in this dataset";
    }
    return out;
}

bool placementClear(const std::deque<Consist>& trains, const Placement& p) {
    if (p.path == nullptr) return false;
    // The road the train would stand on, in the coordinates a train runs in. Compared
    // against what is already standing there rather than against distances in the world:
    // two roads can run within metres of each other and be entirely separate places.
    PathSpan want;
    want.pathIdx = p.pathIdx;
    want.s0 = p.s - p.half;
    want.s1 = p.s + p.half;

    std::vector<PathSpan> spans;
    for (const Consist& t : trains) {
        spans.clear();
        // A train whose road could not be worked out at all counts as being in the way.
        // It is somewhere, and "I could not tell" is not a reason to drop another train on
        // it - the wrong answer here is two trains inside each other, which derails both
        // and reads to the circuits as one.
        if (!t.occupiedSpans(spans)) return false;
        for (const PathSpan& s : spans)
            if (s.overlaps(want)) return false;
    }
    return true;
}

const Station* nearestStation(const std::vector<Station>& all, glm::dvec3 world) {
    const Station* best = nullptr;
    double bestD2 = std::numeric_limits<double>::max();
    for (const Station& s : all) {
        const double dx = s.world.x - world.x, dy = s.world.y - world.y;
        const double d2 = dx * dx + dy * dy;
        if (d2 < bestD2) { bestD2 = d2; best = &s; }
    }
    return best;
}
