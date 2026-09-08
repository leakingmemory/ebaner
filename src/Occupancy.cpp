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

#include "Occupancy.h"

#include <algorithm>
#include <cstdio>
#include <unordered_map>

namespace {

// Which path carries each track. A track is on exactly one, contiguously and once: the
// export is deduped to one segment per id and buildTrackPaths' chaining is a strict
// partition of those segments. So this is a lookup and not a search - which is the whole
// point of the bridge, and why nothing here has a tolerance in it.
std::unordered_map<std::uint32_t, int> indexTracks(const std::vector<TrackPath>& paths) {
    std::unordered_map<std::uint32_t, int> out;
    for (std::size_t pi = 0; pi < paths.size(); ++pi)
        for (const TrackPath::TrackRun& r : paths[pi].trackRuns())
            out.emplace(r.trackId, static_cast<int>(pi));
    return out;
}

} // namespace

SectionSpans resolveSectionSpans(const TrackCircuits& circuits,
                                 const std::vector<TrackPath>& paths) {
    SectionSpans out;
    out.sectionCount = static_cast<int>(circuits.sections.size());
    const std::unordered_map<std::uint32_t, int> onPath = indexTracks(paths);

    char note[192];
    for (std::size_t si = 0; si < circuits.sections.size(); ++si) {
        const Section& sec = circuits.sections[si];
        for (const SectionInterval& iv : sec.parts) {
            const auto it = onPath.find(iv.trackId);
            if (it == onPath.end()) {
                std::snprintf(note, sizeof(note),
                              "section %d \"%s\" covers track %x, which is not in the "
                              "export - that stretch cannot sense a train",
                              sec.id, sec.name.c_str(), iv.trackId);
                out.notes.push_back(note);
                continue;
            }
            const TrackPath& p = paths[it->second];
            float a = 0.0f, b = 0.0f;
            if (!p.fracToS(iv.trackId, iv.from, a) || !p.fracToS(iv.trackId, iv.to, b)) {
                std::snprintf(note, sizeof(note),
                              "section %d \"%s\": track %x is on path %d but %g..%g will "
                              "not place on it",
                              sec.id, sec.name.c_str(), iv.trackId, it->second, iv.from,
                              iv.to);
                out.notes.push_back(note);
                continue;
            }
            // A section's interval is undirected - `from` may exceed `to` and means the
            // same stretch either way - so it is normalised here and the direction it was
            // written in is forgotten. It matters in a SignalPath, where the sign is the
            // way a train runs; it means nothing in a track circuit, which senses a train
            // whichever way it is going.
            SectionSpans::Entry e;
            e.section = static_cast<int>(si);
            e.span.pathIdx = it->second;
            e.span.s0 = std::min(a, b);
            e.span.s1 = std::max(a, b);
            out.entries.push_back(e);
        }
    }
    return out;
}

void computeOccupancy(const SectionSpans& secs, const std::vector<PathSpan>& trainSpans,
                      std::vector<char>& occ) {
    occ.assign(static_cast<std::size_t>(std::max(0, secs.sectionCount)), 0);
    if (trainSpans.empty()) return;
    for (const SectionSpans::Entry& e : secs.entries) {
        if (e.section < 0 || e.section >= static_cast<int>(occ.size())) continue;
        if (occ[e.section]) continue; // already held by something else
        for (const PathSpan& t : trainSpans)
            if (t.overlaps(e.span)) {
                occ[e.section] = 1;
                break;
            }
    }
}
