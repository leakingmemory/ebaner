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

#include "SwitchNetwork.h"  // SwitchNetwork (which road the points lead to)
#include "TrackCircuits.h"  // TrackPoly, TrackJunctions, fracToWorld

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
// A crossing may span more than one track. Inside a station the road crosses both roads at
// once, and that is one crossing - one road shut, one bell - with a set of circuits per
// track, so that a train on one of them runs the sequence while the other is left alone and
// two trains, one on each, are each seen for themselves.
struct CrossingTrack {
    std::uint32_t trackId = 0;
    double frac = 0.0;
};
// File `<datasetRoot>/overlay/level-crossings.txt`:
//   crossing <id> "<name>" <trackHex>:<frac> [also <trackHex>:<frac>]... [<outerM>] [barriers]
// Every trailing field is optional and they may come in any order.
struct LevelCrossing {
    int id = 0;
    std::string name;
    // One entry for an ordinary crossing, two where it spans both roads of a station. The
    // first is the one the record leads with; the rest arrive as `also`.
    std::vector<CrossingTrack> tracks;
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

// How far out the crossing's distant signals stand, each side.
//
// A driver who first sees the crossing's own head at kSignalOffsetM has no room left to
// do anything about it, so the same indication is repeated far enough back to stop from:
// four fifths of the braking distance at the line speed. Four fifths and not the whole of
// it because a distant is a warning to start braking, not the last possible moment - the
// remaining fifth is the margin.
//
// It has to sit *inside* the approach circuit, and by a margin: the train must already
// have armed the crossing before it can read the repeat, or the signal would show idle to
// a driver who is the reason it is about to close. So the braking figure is capped
// against the approach distance rather than trusted on its own: the approach is clamped
// at kOuterMaxM and the braking distance is not, so past about 160 km/h the raw figure
// starts eating the margin and past about 213 km/h it leaves the circuit altogether.
constexpr double kDistantOfBraking = 0.8;  // of the braking distance
constexpr double kDistantOfApproach = 0.8; // but never further out than this of the approach
double distantDistance(double lineSpeedKmh, double outerM);

// A crossing resolved onto the built paths: which path each of its tracks sits on and
// where, plus the circuit extents in that path's arc length. Paths are built once and never
// rebuilt, so this is worked out once at load.
//
// The distances belong to the crossing rather than to a track. One crossing arming its two
// tracks at two different distances would be two crossings sharing a road, so they are
// derived once from the fastest track it carries - the warning time a crossing needs is set
// by the fastest train that can reach it.
struct CrossingSite {
    // Parallel to LevelCrossing::tracks; `path < 0` is one that did not resolve.
    struct On {
        int path = -1;  // index into the path list, -1 = not on any (stale overlay)
        float s = 0.0f; // arc length of the crossing along that path
    };
    std::vector<On> tracks;
    float innerM = 0.0f;
    float outerM = 0.0f;
    float distantM = 0.0f;     // where the repeats stand, each side
    float lineSpeedKmh = 0.0f; // what the approach distance was derived from

    // Whether anything resolved at all - what the single `path < 0` guard used to ask.
    bool resolved() const {
        for (const On& o : tracks)
            if (o.path >= 0) return true;
        return false;
    }
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

// The sequence one track of a crossing is running. A crossing with two tracks runs two of
// these, independently: the whole point is that a train on one road does not make the other
// road's circuits say anything.
struct CrossingTrackState {
    CrossingPhase phase = CrossingPhase::Idle;
    double phaseSince = 0.0;    // when the current phase began
    double allClearSince = 0.0; // when this track's circuits last became clear (0 = not)
    bool prevOuterA = false, prevOuterB = false; // for the edge gate
    // Whether the train has actually reached the crossing this cycle. Without it,
    // Secured would release the instant it was entered: the sequence starts while the
    // train is still out on the approach, so an unoccupied inner circuit at that moment
    // means "not here yet", not "gone past".
    bool innerSeen = false;
};

struct CrossingState {
    // One per track of the crossing, parallel to LevelCrossing::tracks.
    std::vector<CrossingTrackState> tracks;
    // When the crossing as a whole last left Idle. Distinct from a track's phaseSince,
    // which restarts at every phase change: the bell has to run across Closing into
    // Secured without being retriggered halfway by the change between them - and, now,
    // without being restarted by the second track arming. One crossing, one bell.
    double activeSince = 0.0;
    // Where the booms are: 0 fully up, 1 fully down. Continuous state of its own rather
    // than something read off the phase, because it outlives the phases at both ends -
    // it is still up for the first 2 s of Secured, and still coming up for 3 s after
    // Opening has handed back to Idle. One road, one pair of booms, however many tracks.
    float barrier = 0.0f;
    // When this crossing was last stepped, so the barrier can be moved at a rate. 0 means
    // never, and the first step moves nothing.
    double lastStepS = 0.0;

    // Shut to the road while *any* track is running its sequence.
    bool shut() const {
        for (const CrossingTrackState& t : tracks)
            if (t.phase != CrossingPhase::Idle) return true;
        return false;
    }
    // What one track is showing. Idle for a track that does not exist, so a caller need
    // not carry the count around.
    CrossingPhase phase(std::size_t track) const {
        return track < tracks.size() ? tracks[track].phase : CrossingPhase::Idle;
    }
    // Whether the booms are in motion - what tells the renderer it must keep rebuilding.
    bool barrierMoving() const { return barrier > 0.0f && barrier < 1.0f; }
};

// Which of the crossing's tracks a point on the rails is on: an index into its tracks, or
// -1 for a point that is on none of them. `s` gets the arc length along that track's path.
//
// Decided once, for the point, rather than by each track asking whether the point is near
// enough to be its own. The two roads of a station converge at their turnouts and run about
// a metre apart there, so both would answer yes to that - and a train leaving on the main
// line armed the loop's circuits as it passed the points. A train is on one road.
//
int crossingTrackUnder(const CrossingSite& site, const std::vector<TrackPath>& paths,
                       const glm::vec2& at, float& s);

// Which road a train standing at `s` on the crossing's road `on` is actually *for*: `on`
// itself, or another of the crossing's roads if a turnout between it and the crossing is
// facing it and set to that road.
//
// Out on the approach the roads have not divided yet, so geometry cannot answer this: a
// train on the rails that lead to both is on the main line by position and bound for the
// loop by the points. Arming the road it will not take would run the sequence on the wrong
// side - clearing that road's heads to white and leaving the train to pass the other's at
// red. Between the turnouts nothing is in the way and this changes nothing.
int crossingRoadAtPoints(const CrossingSite& site, const SwitchNetwork& net, int on,
                         float s);

// A signal standing in one of a crossing's approach circuits, facing it.
//
// Where a signal stands never changes; only what it is showing does. So these are worked
// out once, at load, and read every frame against the aspects.
struct CrossingGuard {
    int track = 0;      // which of the crossing's roads it stands on
    int placement = -1; // into the caller's signal placements
    float atM = 0.0f;   // where it stands, signed from the crossing as an axle's rel is
};

// How far each approach circuit reaches this instant: its full length, or the nearest
// signal at danger facing the crossing on that side. Indexed [2 * track + side], side 0
// being the approach on the -s side.
//
// A signal at danger breaks the circuit at itself, because nothing beyond one can reach the
// crossing without first passing it. Inside a station that is the difference between a
// crossing that shuts for the traffic that is coming and one that shuts for a train
// standing at a red signal for as long as it stands there.
//
// `open` says which placements are giving an authority to move, parallel to whatever list
// the guards index - filled by the caller, which is what keeps signals out of this header.
// Never shorter than the inner circuit: that one is at the crossing and belongs to no
// signal.
std::vector<float> crossingReach(const CrossingSite& site,
                                 const std::vector<CrossingGuard>& guards,
                                 const std::vector<char>& open);

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
//
// `occ` carries one reading per track, in the record's order; a short list leaves the rest
// clear. The tracks are stepped independently and only then do the things that belong to
// the road - the bell's clock and the booms - look at all of them together.
void stepCrossing(const LevelCrossing& x, CrossingState& st,
                  const std::vector<CrossingOccupancy>& occ, double now);

// What each head shows. Only one lamp of a head is ever lit.
//
// `fast` is the crossing's own pulse, not this phase's: every head on a crossing flashes
// together, so a track sitting idle beside one that is arming shows red on the *fast*
// pulse. Its lamps are still its own - only the track the train is on goes white.
struct CrossingLights {
    bool trainRed = false, trainWhite = false;
    bool roadRed = false, roadWhite = false;
    bool fast = false; // the active pulse; idle is the slow one
};
CrossingLights crossingLights(CrossingPhase phase, bool fast);

// Which of the crossing's tracks a train at (`trackId`, `frac`) running in `dir` would be
// carried onto, taking every turnout the way it is currently set: an index into
// `x.tracks`, or -1.
//
// -1 is not a failure to be papered over. It means the points cannot say which road the
// train will take - a broken switch, or a fan the interlocking does not know - and a repeat
// that cannot know what it is repeating warns rather than guesses.
//
// This is what lets one mast out on the single track stand for both roads of a station: the
// mast does not move when the points are thrown, but what it is warning about does.
int crossingTrackAhead(const LevelCrossing& x, const std::vector<TrackPoly>& polys,
                       const TrackJunctions& junctions, const SwitchNetwork& net,
                       std::uint32_t trackId, double frac, int dir, double maxM);

// Whether the warning bell is sounding. Silent while idle, and silent again kBellS after
// the sequence started even though the crossing is still shut and still flashing.
bool crossingBell(const CrossingState& st, double now);
