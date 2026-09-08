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

#pragma once

// Which track circuits hold a train.
//
// Occupancy used to be decided in world space: every axle's position was matched against a
// sampled polyline of every section, and an axle within 2.5 m of one counted as standing in
// it. That tolerance is lateral - it is there to keep an axle on its own road rather than a
// parallel one - and every fault it produced came of asking it to do a job it was not.
// Sampled coarsely the polyline cut the curves and a train on the plain line was invisible
// to its own circuit; clamped to a segment the tolerance wrapped round the ends of a run and
// a train standing 2.5 m outside a section occupied it; and it could never tell two roads
// apart where they converge to under 4 m at a turnout throat.
//
// So it is decided topologically instead, which is what an axle counter does: a section is
// a stretch of one road, a train is a stretch of one road, and the section is occupied when
// the two overlap. There is no tolerance because there is nothing to be tolerant about -
// both are arc-length ranges on the same TrackPath, and the vehicle knows which path it is
// on without anybody having to guess from where it is standing.
//
// Direction never enters it. A span has two ends and it does not matter which is the front,
// so a train creeping at a millimetre a second, or standing, is no different from one at
// line speed - the case that makes a real counter's direction logic delicate simply does not
// arise. Nor does a split need anything undone: occupancy is derived afresh from where each
// vehicle *is*, so parting a train leaves two vehicles in exactly the places the one train's
// vehicles were, and the same sections come out occupied.

#include "TrackCircuits.h"
#include "TrackPath.h"

#include <string>
#include <vector>

// A stretch of one road: `s0` to `s1` metres along path `pathIdx`, with `s0 <= s1`.
// Sections and trains are both expressed this way, which is what makes the test between
// them an overlap and nothing more.
struct PathSpan {
    int pathIdx = -1;
    float s0 = 0.0f, s1 = 0.0f;
    bool overlaps(const PathSpan& o) const {
        // Closed at both ends on purpose. Neighbouring sections meet at a border, so a train
        // exactly on one is in the sections either side of it - which is right, and far
        // safer than the alternative of it being in neither for an instant.
        return pathIdx == o.pathIdx && s0 <= o.s1 && o.s0 <= s1;
    }
};

// Where every authored section lies, in the coordinates a train runs in.
struct SectionSpans {
    // One entry per section interval that resolved, in no particular order. Flat rather
    // than grouped by section: the per-frame loop walks all of them once, and there are a
    // couple of hundred over the whole line.
    struct Entry {
        int section = -1; // index into TrackCircuits::sections
        PathSpan span;
    };
    std::vector<Entry> entries;
    std::vector<std::string> notes; // intervals that would not resolve; the caller prints
    int sectionCount = 0;
    bool empty() const { return entries.empty(); }
};

// Resolve every section against the paths, once, at load.
//
// An interval that cannot be placed - its track is not in the export any more, which three
// of them in this dataset are - contributes nothing and is named in `notes`. That is the
// same thing the old geometric version did, silently: the section simply has one fewer
// piece and can no longer sense a train on that piece. It is reported here because a
// circuit with a hole in it releases routes it should be holding, and nobody should have to
// find that out from a signal.
SectionSpans resolveSectionSpans(const TrackCircuits& circuits,
                                 const std::vector<TrackPath>& paths);

// Fill `occ` - one entry per section, indexed as TrackCircuits::sections is - from the
// stretches of road the trains cover. Cleared first, so it is the whole answer and not an
// addition to a previous one.
void computeOccupancy(const SectionSpans& secs, const std::vector<PathSpan>& trainSpans,
                      std::vector<char>& occ);
