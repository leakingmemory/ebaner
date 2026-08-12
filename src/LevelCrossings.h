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

#include "TrackCircuits.h" // TrackPoly, fracToWorld

#include <cstdint>
#include <string>
#include <vector>

class TrackPath;

// A level crossing secured by lights alone - no barriers.
//
// Four heads, each red over white and flashing: two facing the train, one each way along
// the track, and two facing the road, one each way across it. Three detection circuits,
// all belonging to the crossing rather than to the authored track circuits in
// TrackCircuits.h - nothing has to be drawn for one of these to work.
//
// Some crossings also carry half-barriers - see `barriers` below. Both variants light and
// ring identically; the barriers are the only difference.
//
// File `<datasetRoot>/overlay/level-crossings.txt`:
//   crossing <id> "<name>" <trackHex>:<frac> [<outerM>] [barriers]
// Both trailing fields are optional and may come in either order.
struct LevelCrossing {
    int id = 0;
    std::string name;
    std::uint32_t trackId = 0;
    double frac = 0.0;
    // How far out the approach circuits reach. 0 means derive it from the line speed
    // here, which is what nearly every crossing should do; a value overrules that where
    // the derivation comes out wrong.
    double outerM = 0.0;
    // Half-barriers as well as lights: one boom each side, each covering the lane its
    // traffic arrives on. Off by default, and only written when set - a crossing secured
    // by lights alone is the ordinary kind and its overlay line stays as it was.
    bool barriers = false;
};

std::vector<LevelCrossing> loadLevelCrossings(const std::string& datasetRoot);
bool writeLevelCrossings(const std::string& datasetRoot,
                         const std::vector<LevelCrossing>& xs);

// --- Geometry -------------------------------------------------------------------
// Where the train heads stand either side of the crossing, and how far past them the
// inner circuit reaches. A train that has pulled up to a signal must still be detected
// and must still be able to see it, so the circuit has to outlast the signal.
constexpr double kSignalOffsetM = 25.0;
constexpr double kInnerMarginM = 25.0;
inline double innerHalfM() { return kSignalOffsetM + kInnerMarginM; }

// The approach distance derived from the line speed: braking at a service rate, plus the
// time the crossing needs to run its own sequence. At 130 km/h that is around 1.5 km; at
// 40 km/h a couple of hundred metres, so a yard crossing does not arm a kilometre out.
// Not SpeedLimits.h's kServiceDecel, which is deliberately conservative because warning
// a driver early about a speed change costs nothing. Arming a crossing early does cost
// something - the road is shut for longer than it needs to be - so this is the rate a
// train actually brakes at rather than a pessimistic bound.
constexpr double kCrossingBrakeDecel = 0.7; // m/s^2
constexpr double kSequenceS = 15.0;    // s of running the crossing wants in hand
constexpr double kOuterMinM = 200.0;
constexpr double kOuterMaxM = 2000.0;
double approachDistance(double lineSpeedKmh);

// A crossing resolved onto the built paths: which path it sits on and where, plus the
// circuit extents in that path's arc length. Paths are built once and never rebuilt, so
// this is worked out once at load.
struct CrossingSite {
    int path = -1;      // index into the path list, -1 = not on any (stale overlay)
    float s = 0.0f;     // arc length of the crossing along that path
    float innerM = 0.0f;
    float outerM = 0.0f;
    float lineSpeedKmh = 0.0f; // what the approach distance was derived from
};
// `origin` is the scene origin: the polys are in world coordinates and the paths are
// scene-relative, so resolving one onto the other has to cross between them.
std::vector<CrossingSite> resolveCrossings(const std::vector<LevelCrossing>& xs,
                                           const std::vector<TrackPath>& paths,
                                           const std::vector<TrackPoly>& polys,
                                           const glm::dvec3& origin);

// --- The sequence ---------------------------------------------------------------
// Closing and Opening light identically - the crossing is shut to the road in both - and
// differ only in where they lead. That is the 5 s reverse falling out as the same state
// entered from the other end.
enum class CrossingPhase { Idle, Closing, Secured, Opening };

// How long the train's red is held before it goes to white, and the same delay again on
// the way back out.
constexpr double kTrainDelayS = 5.0;
// A train can arm an approach circuit and then reverse away without ever arriving. As
// described the crossing would stay shut for good, so an all-clear this long releases it.
constexpr double kStuckTimeoutS = 60.0;
// How long the warning bell rings for, measured from the moment the sequence starts.
//
// It stops well before the crossing opens, and the lights carry the warning from there.
// A crossing on a long approach can stay shut for minutes, and a bell that rang the whole
// time would be unbearable for whoever lives beside it - the bell is there to catch the
// attention of someone already at the crossing, which it has either done by now or will
// not do at all.
constexpr double kBellS = 30.0;
// The barriers, on a clock of their own.
//
// They start down well after the lights start flashing - the road is warned first and the
// boom falls into a gap that has already begun to clear - and they are fully down at
// kBarrierDelayS + kBarrierTravelS = 15 s, which is exactly the kSequenceS the approach
// distance already reserves for the crossing to run in. So barriers cost no extra warning
// distance; the circuits do not move.
constexpr double kBarrierDelayS = 7.0;   // after activating, before the boom starts to fall
constexpr double kBarrierTravelS = 8.0;  // to travel either way, upright to horizontal

struct CrossingState {
    CrossingPhase phase = CrossingPhase::Idle;
    double phaseSince = 0.0; // when the current phase began
    // When the crossing last left Idle. Distinct from phaseSince, which restarts at every
    // phase change: the bell has to run across Closing into Secured without being
    // retriggered halfway by the change between them.
    double activeSince = 0.0;
    double allClearSince = 0.0; // when every circuit last became clear (0 = not clear)
    bool prevOuterA = false, prevOuterB = false; // for the edge gate
    // Whether the train has actually reached the crossing this cycle. Without it,
    // Secured would release the instant it was entered: the sequence starts while the
    // train is still out on the approach, so an unoccupied inner circuit at that moment
    // means "not here yet", not "gone past".
    bool innerSeen = false;
    // Where the booms are: 0 fully up, 1 fully down. Continuous state of its own rather
    // than something read off the phase, because it outlives the phases at both ends -
    // it is still up for the first 2 s of Secured, and still coming up for 3 s after
    // Opening has handed back to Idle.
    float barrier = 0.0f;
    // When this crossing was last stepped, so the barrier can be moved at a rate. 0 means
    // never, and the first step moves nothing.
    double lastStepS = 0.0;

    // Whether the booms are in motion - what tells the renderer it must keep rebuilding.
    bool barrierMoving() const { return barrier > 0.0f && barrier < 1.0f; }
};

// What the three circuits see this instant.
struct CrossingOccupancy {
    bool outerA = false; // approach on the -s side
    bool inner = false;
    bool outerB = false; // approach on the +s side
};

// Advance one crossing. `now` is a monotonic clock in seconds.
//
// Two rules carry the safety argument here:
//  - the approach circuits arm the sequence on their *edge* (clear -> occupied) and only
//    from Idle, which is what stops a departing train re-arming the crossing from the far
//    approach circuit as it leaves;
//  - the inner circuit arms it whenever it is occupied, no edge and no gate, because it is
//    the fallback for an approach circuit that has failed.
//
// Takes the crossing itself as well as its state because the barriers are authored, not
// runtime: whether there are booms to move belongs to the record, and copying it into the
// state would be two places to keep in step.
void stepCrossing(const LevelCrossing& x, CrossingState& st, const CrossingOccupancy& occ,
                  double now);

// What each head shows. Only one lamp of a head is ever lit.
struct CrossingLights {
    bool trainRed = false, trainWhite = false;
    bool roadRed = false, roadWhite = false;
    bool fast = false; // the active pulse; idle is the slow one
};
CrossingLights crossingLights(CrossingPhase phase);

// Whether the warning bell is sounding. Silent while idle, and silent again kBellS after
// the sequence started even though the crossing is still shut and still flashing.
bool crossingBell(const CrossingState& st, double now);
