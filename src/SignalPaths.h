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

#include <functional>
#include <string>
#include <vector>

#include "SwitchNetwork.h" // SwitchNetwork, SwitchState (path alignment)
#include "TrackCircuits.h" // Border, SectionInterval

// A mini signal path: a directional route from one track-circuit border to another,
// running through one or more circuits and switches. Stored as its two endpoint borders
// plus the ordered directed track intervals of the route (from -> to is the travel
// direction, so `from` may exceed `to`); the switch legs it takes and the circuits it
// spans are derivable from these intervals. Anchored by trackId + arc-length like the
// rest of the overlay, so it survives re-imports.
//
// File `<datasetRoot>/overlay/signal-paths.txt`:
//   path <id> <name> <startTrackHex>:<startFrac> <endTrackHex>:<endFrac> \
//        [via <trackHex>:<frac>]... <trackHex>:<from>:<to> <trackHex>:<from>:<to> ...
// The `via` entries are optional and come before the intervals. A reader that predates
// them skips both tokens (neither parses as an interval), so the file stays loadable.

// What a set route authorises, which is what the signal governing it then displays. In
// Norway an unrestricted proceed is two greens and a proceed over a deviation is one.
enum class RouteType { C1, C2 };

struct SignalPath {
    int id = 0;
    std::string name;
    Border start;
    Border end;
    // Borders the route must pass through, in order. Used to pick one road where several
    // connect the same two borders; empty when the route is unambiguous on its own.
    std::vector<Border> vias;
    std::vector<SectionInterval> parts; // ordered, directed route intervals
    // --- exit routes only (left at these defaults by mini paths and exit signals) ---
    int exitId = 0;                  // >0: the exit signal this route leads up to
    RouteType type = RouteType::C1;  // the authority it grants
    // --- main signals only (entry and exit signals) ---
    // A distant hangs on this signal's mast, repeating the next main signal ahead. It is a
    // fact about the *mast*, not about this one route: several records sharing a start
    // border are one signal, so any of them carrying the flag means the mast carries the
    // distant. The editor sets it on all of them at once; a file edited by hand need not.
    bool distant = false;
    // This signal is built with two lamps rather than three - red over green, the head a
    // siding's own signal carries. A fact about the mast in the same way `distant` is.
    bool twoLamp = false;
    // --- anything that stands beside a track (mini paths and main signals alike) ---
    // Which side of the track the mast stands on: +1 to the right of the direction the
    // signal is read from, -1 to the left. Right is the Norwegian convention and the
    // default, so a file that says nothing means right and stays as it was written.
    //
    // Kept apart from the facing on purpose, exactly as an avalanche signal's is: turning
    // a head round should not walk its post across the line. Like `distant` and `twoLamp`
    // this is a fact about the *mast*, so several records sharing a start border settle it
    // between them - any one of them saying left puts the mast on the left.
    int side = 1;
    // Which station works this route, named outright, overruling the geometry.
    //
    // The traffic manager groups routes into places by how close their in-station ends
    // lie, which is right until a route runs somewhere far from the platforms - a branch
    // to an industrial siding a couple of kilometres out. Those cluster as a place of
    // their own, and since the picker offers the cluster nearest the station being
    // worked, a cluster with no station near it can never be offered at all: the route
    // is built, has a mast, and is unreachable. Naming the station settles it.
    //
    // Authored on either half of a movement - the approach or the signal - since either
    // may be the one that reads oddly; if both name a station they must agree.
    std::string station;
};

// --- Moving a border (see canMoveBorder/moveBorderFrac in TrackCircuits.h) ---
// Would moving the border at (trackId, oldFrac) to `newFrac` break a route? A path may
// legitimately end at a non-border point (a junction or a track end), which the
// border-neighbour check knows nothing about, so this catches both a route interval
// collapsing to nothing and one *reversing* - the latter happens when the border is
// dragged past a junction the route uses, which would silently turn the route (and its
// signal) around. `why` gets a human-readable reason.
bool borderMoveBreaksPath(const std::vector<SignalPath>& paths,
                          const std::vector<TrackPoly>& polys, std::uint32_t trackId,
                          double oldFrac, double newFrac, std::string& why);
// Rewrite every path endpoint and route-interval endpoint anchored to that border. Matches
// on track id *and* fraction. Returns how many values changed.
int moveBorderFrac(std::vector<SignalPath>& paths, const std::vector<TrackPoly>& polys,
                   std::uint32_t trackId, double oldFrac, double newFrac);

// --- File IO (mirrors loadTrackCircuits/writeTrackCircuits) ---
std::vector<SignalPath> loadSignalPaths(const std::string& datasetRoot);
bool writeSignalPaths(const std::string& datasetRoot,
                      const std::vector<SignalPath>& paths);

// Exit signals (main signals) share the SignalPath shape: the signal stands on `start`
// and protects the route to `end`. Stored separately in `overlay/exit-signals.txt`.
std::vector<SignalPath> loadExitSignals(const std::string& datasetRoot);
bool writeExitSignals(const std::string& datasetRoot,
                      const std::vector<SignalPath>& exits);

// Entry signals: the signal stands on `start` and the record is the whole route into the
// station, ending at `end`. Unlike an exit signal the authority begins at the mast, so there
// is no separate approach record - several entries sharing a start border are one signal.
// Stored in `overlay/entry-signals.txt`.
std::vector<SignalPath> loadEntrySignals(const std::string& datasetRoot);
bool writeEntrySignals(const std::string& datasetRoot,
                       const std::vector<SignalPath>& entries);

// Exit routes: the authority to move from a border inside the station up to an exit signal
// (`start` -> `end`, where `end` is that signal's own border and `exitId` names it). A main
// signal's authority begins back at the platform road, not at the signal, and one signal
// commonly serves several roads - so these are many-to-one onto an exit signal.
// Stored in `overlay/exit-routes.txt`.
std::vector<SignalPath> loadExitRoutes(const std::string& datasetRoot);
bool writeExitRoutes(const std::string& datasetRoot,
                     const std::vector<SignalPath>& routes);

// Entry approaches: the road leading *up to* an entry signal, which is the other half of
// what an exit route is. An entry signal's authority is taken to begin at its mast, and for
// nearly every one of them that is right; where several roads lead up to one mast, though,
// which of them a train is coming in on is a choice, and this is where those roads are
// written down. Stored in `overlay/entry-approaches.txt`.
//
// Called an approach rather than an entry route because this codebase already calls the
// entry signal's own record an entry route - in the file, in the editor and in the load
// report - and one word for two things in an interlocking is how a mistake gets made.
//
// It names no signal. An exit route can name one because an exit signal is a single record;
// an entry mast is several records sharing a start border, so there is no id to name. The
// approach identifies its mast geometrically, as `routeTargetSignal` reads it: its `end` is
// the mast's border, and it must arrive facing the way the mast faces.
//
// An entry signal with no approach behaves exactly as it did before there were any - the
// authority begins at the mast - which is what every one of them on the line does.
std::vector<SignalPath> loadEntryApproaches(const std::string& datasetRoot);
bool writeEntryApproaches(const std::string& datasetRoot,
                          const std::vector<SignalPath>& approaches);

// A distant signal: a point anywhere along a track, facing one way, that shows what the
// first main signal ahead is displaying. It sits on no border, belongs to no route and is
// not interlocked - it only looks. Stored in `overlay/distant-signals.txt`:
//   distant <id> "<name>" <trackHex>:<frac> <+|->
struct DistantSignal {
    int id = 0;
    std::string name;
    std::uint32_t trackId = 0;
    double frac = 0.0;
    int dir = 1; // +1 reads toward increasing frac along the track, -1 the other way
    int side = 1; // +1 right of that direction, -1 left; independent of it
};
std::vector<DistantSignal> loadDistantSignals(const std::string& datasetRoot);
bool writeDistantSignals(const std::string& datasetRoot,
                         const std::vector<DistantSignal>& ds);

// What a signal displays. A dwarf uses the fixed reference lamp plus one lamp on the arc:
// Stop = horizontal pair, TrainOnTrack = 45 deg (a train stands in the route's circuits),
// Clear = vertical. A main signal reads the same values as its Norwegian aspects: Stop is
// the red, Clear is C1 (two greens, no restriction) and ClearReduced is C2 (one green, over
// a deviation). A dwarf never shows ClearReduced. A distant carries the same three values
// meaning *expect* that - there is nothing it can say that a main cannot, so a parallel
// enum would only be one more thing to keep in step.
// Dark is a signal showing nothing at all, which only the simple station signals do: an
// unmanned station switches its signals off entirely and trains run through without
// reference to them. It is not "stop" and it is not a fault - an unlit mast is a real
// indication with its own meaning, so it is a value here rather than an absence.
enum class SignalAspect { Stop, TrainOnTrack, Clear, ClearReduced, Dark };

// Which signal stands at a placement: the low dwarf (dvergsignal) that governs shunting
// moves, one of the two tall main signals - the exit protecting a route out of the station,
// the entry authorising one in - or the distant that repeats, from braking distance, what
// the first main signal ahead is showing. Both mains carry the same three-lamp head; only
// the entry's danger aspect flashes, while every distant lamp does.
//
// StationEntry is the simple entry signal (SimpleEntrySignals.h): a main signal's mast
// under a short head carrying two steady lamps, red over green - stop or go, and dark
// when its station is unmanned. Nothing flashes: there is no third indication for a
// flash to tell apart. It is a kind of its own rather than reusing Entry because `paths`
// below indexes whichever collection `kind` names, and those are different collections -
// which is exactly the confusion `kind` is here to prevent.
enum class SignalKind { Dwarf, Exit, Entry, Distant, StationEntry };

// Where a signal sits: the on-track start point of a route and its initial travel
// direction (a signal governs movements leaving that point in that direction).
struct SignalPlacement {
    glm::dvec3 world{0.0};   // start-border world position (on the track)
    glm::dvec2 forward{0.0}; // unit travel direction leaving the border
    // Indices of the routes this signal governs, into the collection matching `kind`:
    // mini paths for a Dwarf, exit signals for an Exit, entry signals for an Entry. Reading
    // them against the wrong collection is the mistake `kind` exists to prevent.
    std::vector<int> paths;
    // Where the signal stands. For a main or a dwarf this is its routes' shared start
    // border; for a distant, the point it was placed at. What lets a forward walk say "this
    // signal is on the road ahead" exactly, rather than by hunting for it in world space.
    Border at;
    SignalAspect aspect = SignalAspect::Stop;
    SignalKind kind = SignalKind::Dwarf;
    // Which side of the track the post stands on, +1 right of `forward` and -1 left. The
    // one thing here the drawing reads that the interlocking does not.
    int side = 1;
    // An exit signal placed where a dwarf also stands shares its pole, the dwarf lower to
    // the ground. `dwarfAspect` is that dwarf's own indication, kept separate so each head
    // still shows what it means.
    bool withDwarf = false;
    SignalAspect dwarfAspect = SignalAspect::Stop;
    std::vector<int> dwarfPaths; // the mini paths of the dwarf sharing this pole
    // A main signal whose routes are flagged `distant` carries one on its own mast, on an
    // adapter in front of and below the main head. Its indication is kept apart from the
    // main's for the same reason the dwarf's is: two signals, one pole.
    bool withDistant = false;
    SignalAspect distantAspect = SignalAspect::Stop;
    // Two lamps on the head instead of three - red over green - which is what a siding's
    // own signal carries. It changes nothing but the drawing: the aspect here is still
    // whatever the interlocking granted, and a head with one green lights it for any
    // clearance, C1 and C2 alike, because it has no way to tell them apart.
    bool twoLamp = false;
};

// Whether this signal is giving an authority to move: a proceed aspect on its own head, or
// on the dwarf sharing its pole - either of them clearing is authority to pass.
//
// A dwarf showing that a train stands in the road ahead is not one. Nor is a dark simple
// station signal, which is a station switched off rather than a road offered: an unmanned
// station's trains run past its signals without reference to them, and nothing here should
// read that as permission.
bool signalGivesAuthority(const SignalPlacement& sp);

// Where a route begins and which way it sets off: the world point of its first interval's
// `from` and the unit direction leaving it. False if the track is missing or degenerate.
bool routeStartPose(const SignalPath& p, const std::vector<TrackPoly>& polys,
                    glm::dvec3& world, glm::dvec2& fwd);

// One placement per distinct (start border, travel direction) over all paths - so paths
// sharing a start collapse to a single signal governing all of them. That is what makes a
// station's several entry routes one mast, exactly as several mini paths make one dwarf.
// Only give this exit *signals*, never exit routes: it puts a signal at every route start,
// so exit routes would sprout masts on the platform roads.
std::vector<SignalPlacement> signalPlacements(const std::vector<SignalPath>& paths,
                                              const std::vector<TrackPoly>& polys,
                                              SignalKind kind = SignalKind::Dwarf);

// Which signal record a route leads up to (an index into `signals`, else -1). The route must
// end on that signal's own border *and* arrive facing the way the signal faces: findSignalRoute
// only knows the route reached the border, not which side of the signal it came from, and
// arriving from behind is not authority to pass it.
//
// Asked of the exit signals for an exit route, and of the entry signals for an entry
// approach. Where several records share a border - as an entry mast's do - it answers with
// the first, which is enough to say "this road leads to that mast".
int routeTargetSignal(const SignalPath& route, const std::vector<SignalPath>& signals,
                      const std::vector<TrackPoly>& polys);

// One list holding dwarfs and main signals. Where a main signal and a dwarf stand at the
// same border facing the same way they are folded into a single placement (`withDwarf`), so
// the two heads share one pole. `mains` is the already-tagged exit and entry placements.
std::vector<SignalPlacement> mergeSignals(const std::vector<SignalPlacement>& dwarfs,
                                          const std::vector<SignalPlacement>& mains);

// A turnout the path traverses and the position it needs there: straight where the path
// runs through the turnout, diverging where it crosses to or from the branch.
struct PathSwitch {
    int turnout = -1;
    SwitchState need = SwitchState::Straight;
};

// Every turnout the path traverses, with the position that path needs.
std::vector<PathSwitch> pathSwitchRequirements(const SignalPath& p, const SwitchNetwork& net,
                                               const std::vector<TrackPoly>& polys);

// True if every turnout the path traverses currently sits in the position that path needs.
bool pathSwitchesAligned(const SignalPath& p, const SwitchNetwork& net,
                         const std::vector<TrackPoly>& polys);

// The type a new exit route should start out as: C2 if any turnout on the departure needs
// its diverging leg, else C1. Both the route up to the signal *and* the signal's own route
// beyond it count - the driver is being told about the whole departure, and the merge onto
// the line commonly sits past the signal. A default only; the editor can override it.
RouteType defaultRouteType(const SignalPath& route, const SignalPath& exit,
                           const SwitchNetwork& net, const std::vector<TrackPoly>& polys);
// The same for a route that is the whole movement on its own - an entry signal's authority
// starts at the mast, so there is no onward leg to take into account.
RouteType defaultRouteType(const SignalPath& route, const SwitchNetwork& net,
                           const std::vector<TrackPoly>& polys);

// --- What a distant signal can see ---
// How far a distant looks. Well past any braking distance, so the search ends by finding a
// signal rather than by running out of range.
inline constexpr double kDistantReach = 4000.0; // m

// Walk the road ahead, taking each turnout the way it is *currently* set.
//
// `onSpan(track, from, to, dir)` is handed each stretch traversed, in order, and returns
// true to stop the walk - it has found whatever it was looking for. The walk also stops at
// `maxM`, at a dead end, and at any junction where the switches cannot say which way the
// road lies: if the interlocking does not know, nothing reading down the line may guess.
//
// Two quite different questions are asked through this - which signal a distant repeats,
// and which road of a station a crossing's repeat is warning about - and the part they
// share is the awkward part: which leg of a turnout is the one the points are set to.
using WalkSpan = std::function<bool(std::uint32_t track, double from, double to, int dir)>;
void walkAhead(const std::vector<TrackPoly>& polys, const TrackJunctions& junctions,
               const SwitchNetwork& net, std::uint32_t trackId, double frac, int dir,
               double maxM, const WalkSpan& onSpan);

// The first main signal facing the same way within `maxM` (an index into `placements`, else
// -1). `walked` optionally collects the road taken, so the editor can draw what a signal
// sees.
int firstMainSignalAhead(const std::vector<TrackPoly>& polys, const TrackJunctions& junctions,
                         const SwitchNetwork& net,
                         const std::vector<SignalPlacement>& placements,
                         std::uint32_t trackId, double frac, int dir, double maxM,
                         std::vector<SectionInterval>* walked = nullptr);

// Point every distant at what it can see - the free-standing ones out on the line, and the
// ones hanging on a main signal's mast. Nothing reachable reads as Stop - the same warning
// as a main at danger, which is the whole point of the rule. Returns true if any aspect
// changed. Call it after the main aspects have settled.
//
// A mounted one is switched off entirely (Dark) whenever the main under it is at danger:
// there is nothing to warn about ahead of a signal you have to stop at. Otherwise it looks
// forward from its own mast exactly as a free-standing one does, and steps over the main it
// hangs on - that signal stands at the point the walk starts from, which the walk excludes.
bool updateDistantAspects(std::vector<SignalPlacement>& placements,
                          const std::vector<TrackPoly>& polys,
                          const TrackJunctions& junctions, const SwitchNetwork& net,
                          double maxM = kDistantReach);

// The ids of the track-circuit sections the path actually runs through (a shared end
// point with a neighbouring section does not count as running through it).
std::vector<int> pathSections(const SignalPath& p, const TrackCircuits& circuits);

// An exit route and the exit signal's own onward route are two halves of one movement.
// Joining them into a single SignalPath lets pathSections and pathSwitchRequirements
// describe the whole departure - which is what has to be clear, locked and aligned.
// Intervals that meet at the signal's border on one track are merged.
SignalPath departureRoute(const SignalPath& exitRoute, const SignalPath& exitSignal);

// True if `part` runs entirely inside `whole`, in the same direction: which dwarf paths lie
// along a departure and so should be opened with it. Compares anchors only.
bool routeContains(const SignalPath& whole, const SignalPath& part);

// True if the two routes ever run over the same rails in opposite directions.
//
// Two authorities overlapping is not by itself a conflict. A train let in to a platform
// road and then let out of it again is one movement over one road, which is how a station
// is passed through rather than stopped at, and both routes are set at once to do it. Two
// routes *facing* each other over the same rails is another matter entirely, and that is
// what this picks out. Merely meeting at a shared border does not count as running over
// the same rails.
bool routesOppose(const SignalPath& a, const SignalPath& b);

// Recompute each signal's aspect: Clear when one of its paths has a route set, else
// TrainOnTrack when one of its paths has its switches aligned and a train standing in that
// path's circuits, else Stop. `secOccupied` is indexed like `circuits.sections` and
// `routeSet` like `paths`. Returns true if any aspect changed.
//
// `exitAspects` (indexed like `placements`, empty = every main signal at danger) says what
// each exit signal should display. That is a question about which routes are locked, which
// only the interlocking knows, so it is passed in rather than decided here; the dwarf rule
// above stays owned by this function.
bool updateSignalAspects(std::vector<SignalPlacement>& placements,
                         const std::vector<SignalPath>& paths, const SwitchNetwork& net,
                         const std::vector<TrackPoly>& polys,
                         const TrackCircuits& circuits,
                         const std::vector<char>& secOccupied,
                         const std::vector<char>& routeSet,
                         const std::vector<SignalAspect>& exitAspects = {});
