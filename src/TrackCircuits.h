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

#include <glm/glm.hpp>

#include <cstdint>
#include <string>
#include <vector>

// Track-circuit (train-present sensing) authoring data, kept as a drop-in overlay
// separate from the geometry edits so the signaling layer can consume it and a
// regenerated import doesn't lose it. Everything is anchored by track id + a
// fractional arc-length position, so a border/section follows small geometry changes
// and survives re-imports as long as the track keeps its id and rough path.
//
// File `<datasetRoot>/overlay/track-circuits.txt` (frac is authoritative; the world
// x/y on a border line is a cached hint for readability + staleness only):
//   border  <trackIdHex> <frac> <x> <y>
//   section <id> <name> <trackIdHex>:<from>:<to> <trackIdHex>:<from>:<to> ...

// A border point (an insulated joint): a position along one track.
struct Border {
    std::uint32_t trackId = 0;
    double frac = 0.0; // arc-length fraction 0..1 along the track's polyline
};

// One track's contribution to a section: the [from,to] fraction range it covers.
struct SectionInterval {
    std::uint32_t trackId = 0;
    double from = 0.0, to = 1.0;
};

// A sensing section: a border-bounded connected block of track (may span turnouts).
struct Section {
    int id = 0;
    std::string name;
    std::vector<SectionInterval> parts;
};

struct TrackCircuits {
    std::vector<Border> borders;
    std::vector<Section> sections;
};

// One track's ordered world polyline (as grouped from the editor's graph by trackId).
struct TrackPoly {
    std::uint32_t id = 0;
    std::vector<glm::dvec3> pts;
};

// --- File IO (mirrors loadTrackOverlay/writeTrackOverlay) ---
TrackCircuits loadTrackCircuits(const std::string& datasetRoot);
bool writeTrackCircuits(const std::string& datasetRoot, const TrackCircuits& tc);

// --- Geometry helpers ---
// Total planar (x,y) length of a polyline.
double polyLength(const std::vector<glm::dvec3>& pts);
// World position at `frac` along a track (interpolated). Returns {0} if id absent.
glm::dvec3 fracToWorld(const std::vector<TrackPoly>& polys, std::uint32_t trackId,
                       double frac);
// Nearest point on a track's polyline to the world (x,y) point `p`: fills its arc-length
// fraction and planar distance. False if the track is absent (outputs untouched).
bool projectOnTrack(const std::vector<TrackPoly>& polys, std::uint32_t trackId,
                    glm::dvec2 p, double& frac, double& dist);
// How close two fractions on `trackId` must be to count as the same point (a metric
// tolerance expressed as a fraction, so it is independent of track length).
double sameFracTol(const std::vector<TrackPoly>& polys, std::uint32_t trackId);

// --- Moving a border ---
// A border's fraction is copied into everything anchored to it (section intervals, and the
// signal paths in SignalPaths.h), so a move has to rewrite them all together.
//
// Whether border `borderIdx` may move to (newTrack, newFrac): the target must be on the
// border's own track, must not sit at or beyond the neighbouring border in either
// direction (which would invert or collapse the section between them), and must actually
// differ from where it already is. `why` gets a human-readable reason on refusal.
bool canMoveBorder(const TrackCircuits& tc, const std::vector<TrackPoly>& polys,
                   int borderIdx, std::uint32_t newTrack, double newFrac, std::string& why);
// Rewrite the border itself plus every section-interval endpoint anchored to it. Matches on
// track id *and* fraction, since not every fraction on a track is a border. Returns how
// many values changed.
int moveBorderFrac(TrackCircuits& tc, const std::vector<TrackPoly>& polys,
                   std::uint32_t trackId, double oldFrac, double newFrac);

// --- Section flood-fill ---
// The connected block of track containing the seed, bounded by border points and real
// track dead-ends. `enclosed` is true if at least one border bounds it (i.e. the user
// has carved it out of the network rather than grabbing the whole thing).
struct SectionResult {
    std::vector<SectionInterval> parts;
    bool enclosed = false;
    int borderEnds = 0;   // interval ends stopped at a border (insulated joint)
    int deadEnds = 0;     // interval ends stopped at a real track terminus (buffer)
    double lengthM = 0.0; // total planar length of the block
};
SectionResult floodSection(const std::vector<TrackPoly>& polys,
                           const std::vector<Border>& borders,
                           std::uint32_t seedTrack, double seedFrac);

// --- Directed route finder (for mini signal paths) ---
// Enumerate directed routes from `start` to `end` (both trackId + arc-length anchors)
// through the connected network, taking only forward (non-reversing) moves at junctions
// so turnout legality (toe<->through / toe<->diverging, never through<->diverging) falls
// out of the geometry. Returns the number of distinct routes found (capped at 2); when
// exactly one exists, fills `out` with its ordered directed intervals (from -> to is the
// travel direction, so `from` may exceed `to`). 0 = no route, >=2 = ambiguous.
int findSignalRoute(const std::vector<TrackPoly>& polys, const Border& start,
                    const Border& end, std::vector<SectionInterval>& out);
