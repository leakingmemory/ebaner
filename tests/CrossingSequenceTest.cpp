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

// The level-crossing sequence.
//
// This is the part of a crossing that is a safety argument rather than a picture, and
// none of it is visible in a screenshot: whether the approach circuits are edge-gated,
// whether the inner circuit can arm the sequence on its own, and whether a crossing can
// be left shut by a train that turned back. So it is driven here directly, with the
// occupancy dictated rather than simulated, and no dataset in the way.

#include "FlagPosts.h"
#include "TxpGraph.h"
#include "TxpNetwork.h"
#include "TxpMesh.h"
#include "Script.h"
#include "SignalPaths.h"
#include "TerrainData.h"
#include "TrackOverlay.h"
#include "TrackPath.h"
#include "TxpPositions.h"
#include "LevelCrossings.h"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <string>

namespace {

int failures = 0;

void check(bool ok, const std::string& what) {
    std::printf("  %-56s %s\n", what.c_str(), ok ? "ok" : "FAILED");
    if (!ok) ++failures;
}

const char* phaseName(CrossingPhase p) {
    switch (p) {
        case CrossingPhase::Idle: return "Idle";
        case CrossingPhase::Closing: return "Closing";
        case CrossingPhase::Secured: return "Secured";
        case CrossingPhase::Opening: return "Opening";
    }
    return "?";
}

// A crossing driven by a clock, with the circuits set by hand.
struct Rig {
    LevelCrossing x; // the crossing itself; set x.barriers to test the barrier variant
    CrossingState st;
    CrossingOccupancy occ;
    double now = 0.0;

    // Advance `seconds`, stepping finely enough that a 5 s phase cannot be stepped over.
    void run(double seconds) {
        const double dt = 0.1;
        for (double t = 0.0; t < seconds - 1e-9; t += dt) {
            now += dt;
            stepCrossing(x, st, occ, now);
        }
    }
    // Apply a circuit change and let it be seen, without meaningfully advancing time.
    void set(bool a, bool inner, bool b) {
        occ = {a, inner, b};
        now += 0.01;
        stepCrossing(x, st, occ, now);
    }
    CrossingPhase phase() const { return st.phase; }
};

void expectPhase(Rig& r, CrossingPhase want, const std::string& what) {
    const bool ok = r.phase() == want;
    std::printf("  %-56s %s%s\n", what.c_str(), ok ? "ok" : "FAILED",
                ok ? "" : (std::string("  (was ") + phaseName(r.phase()) + ", wanted " +
                           phaseName(want) + ")")
                              .c_str());
    if (!ok) ++failures;
}

} // namespace

int main(int argc, char** argv) {
    std::puts("\nA train approaching, passing and clearing:");
    {
        Rig r;
        expectPhase(r, CrossingPhase::Idle, "starts idle");

        r.set(true, false, false); // enters the approach circuit
        expectPhase(r, CrossingPhase::Closing, "approach circuit arms the sequence");

        r.run(4.0);
        expectPhase(r, CrossingPhase::Closing, "still closing before the delay is up");
        {
            const CrossingLights l = crossingLights(r.phase());
            check(l.roadRed && !l.roadWhite, "road is red at once, from the first moment");
            check(l.trainRed && !l.trainWhite, "train is still red through the delay");
            check(l.fast, "and on the fast pulse");
        }

        r.run(2.0); // past 5 s
        expectPhase(r, CrossingPhase::Secured, "goes secured after the delay");
        {
            const CrossingLights l = crossingLights(r.phase());
            check(l.trainWhite && !l.trainRed, "train goes white");
            check(l.roadRed, "road stays red");
        }

        r.set(false, true, false); // now over the crossing
        r.run(3.0);
        expectPhase(r, CrossingPhase::Secured, "holds secured while the train is on it");

        r.set(false, false, true); // off the crossing, into the far approach circuit
        expectPhase(r, CrossingPhase::Opening, "inner clearing starts the release");
        {
            const CrossingLights l = crossingLights(r.phase());
            check(l.trainRed && !l.trainWhite, "train drops to red at once");
            check(l.roadRed && !l.roadWhite, "road is still held for the delay");
        }

        r.run(4.0);
        expectPhase(r, CrossingPhase::Opening, "road still held before the delay is up");
        r.run(2.0);
        expectPhase(r, CrossingPhase::Idle, "back to idle after the mirrored delay");
        {
            const CrossingLights l = crossingLights(r.phase());
            check(l.roadWhite && !l.roadRed, "road opens white");
            check(l.trainRed, "train shows red");
            check(!l.fast, "and everything is back on the slow pulse");
        }

        // The whole point of the edge gate: the train is *still standing* in the far
        // approach circuit, and that must not start the sequence over again.
        r.run(10.0);
        expectPhase(r, CrossingPhase::Idle,
                    "a train sitting in the far circuit does not re-arm");

        r.set(false, false, false); // finally leaves
        r.run(1.0);
        expectPhase(r, CrossingPhase::Idle, "and stays idle once it has gone");
    }

    std::puts("\nThe approach circuits have failed - the inner one must still work:");
    {
        Rig r;
        r.run(1.0);
        // Both approach circuits dead: they never report anything.
        r.set(false, true, false);
        expectPhase(r, CrossingPhase::Closing,
                    "inner circuit alone arms the sequence");
        r.run(6.0);
        expectPhase(r, CrossingPhase::Secured, "and the sequence runs as usual");
        r.set(false, false, false);
        expectPhase(r, CrossingPhase::Opening, "clearing releases it");
        r.run(6.0);
        expectPhase(r, CrossingPhase::Idle, "back to idle");
    }

    std::puts("\nA movement that armed the crossing and then turned back:");
    {
        Rig r;
        r.set(true, false, false);
        expectPhase(r, CrossingPhase::Closing, "armed by the approach circuit");
        r.run(6.0);
        expectPhase(r, CrossingPhase::Secured, "secured, waiting for a train");

        r.set(false, false, false); // reversed away, everything clear
        r.run(30.0);
        expectPhase(r, CrossingPhase::Secured,
                    "still shut 30 s later - it is waiting, not stuck yet");
        r.run(35.0); // past the 60 s all-clear
        expectPhase(r, CrossingPhase::Opening, "the timeout releases it");
        r.run(6.0);
        expectPhase(r, CrossingPhase::Idle, "and it returns to idle");
    }

    std::puts("\nA second train, once the first has gone:");
    {
        Rig r;
        r.set(true, false, false);
        r.run(6.0);
        r.set(false, true, false);
        r.run(1.0);
        r.set(false, false, false);
        r.run(6.0);
        expectPhase(r, CrossingPhase::Idle, "first train done");
        // A fresh edge on the same circuit has to arm it again - the gate is on the
        // edge, not on the circuit having never been used.
        r.set(true, false, false);
        expectPhase(r, CrossingPhase::Closing, "a fresh approach edge arms it again");
    }

    std::puts("\nThe approach distance follows the line speed:");
    {
        const double fast = approachDistance(130.0);
        const double slow = approachDistance(40.0);
        std::printf("     130 km/h -> %.0f m,  40 km/h -> %.0f m,  unknown -> %.0f m\n",
                    fast, slow, approachDistance(0.0));
        check(fast > slow, "a fast line arms further out than a slow one");
        check(fast <= kOuterMaxM && slow >= kOuterMinM, "both inside the clamp");
        check(approachDistance(0.0) == slow,
              "an unknown limit is treated as the slow case, not as zero");
        check(innerHalfM() > kSignalOffsetM,
              "the inner circuit outlasts the signal it protects");
    }

    // The overlay format. Worth guarding: the name quoting in these files has been
    // changed once already, and a crossing that will not reload is a crossing that has
    // to be placed twice.
    if (argc > 1) {
        std::puts("\nThe overlay round-trips:");
        std::vector<LevelCrossing> xs;
        // A name with spaces and Norwegian letters, which is what quoting is for. Not one
        // with a quote in it: quoteName documents that names are filtered on entry, so
        // that cannot arise, and a test for it would only be testing a fiction.
        xs.push_back({1, "Fauskeveien nord", 0x6d7, 0.30, 0.0, false});
        xs.push_back({2, "Røsvikveien", 0x6d9, 0.755, 900.0, false});
        xs.push_back({3, "Leivset", 0x6da, 0.10, 0.0, true});   // barriers, no override
        xs.push_back({4, "Finneid bru", 0x6db, 0.90, 750.0, true}); // both fields
        check(writeLevelCrossings(argv[1], xs), "write");
        const std::vector<LevelCrossing> back = loadLevelCrossings(argv[1]);
        check(back.size() == 4, "every record comes back");
        bool same = back.size() == xs.size();
        for (std::size_t i = 0; same && i < back.size(); ++i)
            same = back[i].id == xs[i].id && back[i].name == xs[i].name &&
                   back[i].trackId == xs[i].trackId &&
                   std::abs(back[i].frac - xs[i].frac) < 1e-9 &&
                   std::abs(back[i].outerM - xs[i].outerM) < 1e-9 &&
                   back[i].barriers == xs[i].barriers;
        check(same, "ids, names, tracks, fracs, overrides and barriers survive");
        check(back.size() > 1 && back[0].outerM == 0.0 && back[1].outerM == 900.0,
              "an absent override stays absent, a set one is kept");
        check(back.size() > 3 && !back[0].barriers && back[2].barriers && back[3].barriers,
              "and the two variants are told apart");

        // The two trailing fields are read as keywords, so neither position nor presence
        // of the other matters - and a file written before barriers existed still loads.
        {
            const std::string path =
                std::string(argv[1]) + "/overlay/level-crossings.txt";
            std::ofstream f(path, std::ios::trunc);
            f << "crossing 1 \"old style\" 6d7:0.5\n"          // as it was written before
                 "crossing 2 \"old with override\" 6d8:0.5 800\n"
                 "crossing 3 \"barriers only\" 6d9:0.5 barriers\n"
                 "crossing 4 \"reversed\" 6da:0.5 barriers 650\n"; // keyword first
            f.close();
            const std::vector<LevelCrossing> old = loadLevelCrossings(argv[1]);
            check(old.size() == 4, "all four forms load");
            check(old.size() > 1 && !old[0].barriers && old[0].outerM == 0.0,
                  "a line written before barriers existed is unchanged");
            check(old.size() > 1 && !old[1].barriers && old[1].outerM == 800.0,
                  "an override on its own still reads");
            check(old.size() > 2 && old[2].barriers && old[2].outerM == 0.0,
                  "the keyword on its own does not eat the line");
            check(old.size() > 3 && old[3].barriers && old[3].outerM == 650.0,
                  "and the two may come in either order");
        }
    }

    // The TXP's flag. Each post is its own: setting one says nothing about any other,
    // and nothing about whether the station is manned - a manned station has no flag out
    // most of the time.
    {
        std::puts("\nThe flag posts:");
        std::vector<FlagColour> shown(3, FlagColour::None);
        // Cycling is the whole control: none -> red -> green -> none, per post.
        auto cycle = [&](int i) {
            shown[i] = shown[i] == FlagColour::None    ? FlagColour::Red
                       : shown[i] == FlagColour::Red   ? FlagColour::Green
                                                       : FlagColour::None;
        };
        check(shown[0] == FlagColour::None && shown[1] == FlagColour::None &&
                  shown[2] == FlagColour::None,
              "every fixture starts empty");

        cycle(0);
        check(shown[0] == FlagColour::Red, "one cycle puts a red flag out");
        check(shown[1] == FlagColour::None && shown[2] == FlagColour::None,
              "and no other post is touched");

        cycle(1);
        cycle(1);
        check(shown[1] == FlagColour::Green, "a second post goes to green on its own");
        check(shown[0] == FlagColour::Red,
              "while the first keeps its red - the posts are independent");

        cycle(0);
        check(shown[0] == FlagColour::Green && shown[1] == FlagColour::Green,
              "two greens at one station are allowed");

        cycle(0);
        check(shown[0] == FlagColour::None, "a third cycle takes it down again");
        check(shown[1] == FlagColour::Green, "leaving the other post as it was");
    }

    // Permission to leave. Unlike the flags this one really is a person, so a station
    // can only have it showing in one place - and unlike the entry signals it does not
    // care whether the station is manned.
    {
        std::puts("\nThe TXP's departure signal:");
        const std::string A = "Fauske", B = "Rognan";
        const std::string of[3] = {A, A, B};
        std::map<std::string, int> showingAt; // station -> position, absent = none
        auto shown = [&](int i) {
            const auto it = showingAt.find(of[i]);
            return it != showingAt.end() && it->second == i;
        };
        // Toggling, as the panel does it.
        auto toggle = [&](int i) {
            const auto it = showingAt.find(of[i]);
            if (it != showingAt.end() && it->second == i) showingAt.erase(of[i]);
            else showingAt[of[i]] = i;
        };
        check(!shown(0) && !shown(1) && !shown(2), "nobody is standing out to begin with");

        toggle(0);
        check(shown(0), "the signal is shown where it was set");
        check(!shown(1), "and not at the station's other position");
        check(!shown(2), "nor at the other station");

        toggle(1);
        check(shown(1) && !shown(0),
              "showing the second takes the TXP away from the first - one place at a time");

        toggle(2);
        check(shown(2) && shown(1), "a different station has its own TXP");

        toggle(1);
        check(!shown(1), "toggling the shown one stands the TXP down");
        check(shown(2), "leaving the other station as it was");
    }

    // The warning bell runs on its own clock, not the phases.
    //
    // It has to survive the change from Closing to Secured - phaseSince restarts there,
    // and timing the bell off that would retrigger it halfway and ring for kBellS again -
    // and it has to stop while the crossing is still shut and still flashing, which is the
    // whole point: the lights carry the warning after the bell has had its say.
    {
        std::puts("\nThe crossing's warning bell:");
        const LevelCrossing x;   // lights only: the bell is the same either way
        CrossingState st;
        CrossingOccupancy occ;
        double now = 100.0;
        check(!crossingBell(st, now), "silent while the crossing is idle");

        occ.outerA = true; // a train arms the approach
        stepCrossing(x, st, occ, now);
        check(st.phase == CrossingPhase::Closing, "the sequence starts");
        check(crossingBell(st, now), "the bell starts with it");

        // Through the phase change, which is where a bell timed off phaseSince breaks.
        now += kTrainDelayS + 1.0;
        occ.outerA = false;
        stepCrossing(x, st, occ, now);
        check(st.phase == CrossingPhase::Secured, "and reaches Secured");
        check(crossingBell(st, now), "the bell rings on through the phase change");

        now = 100.0 + kBellS - 0.5;
        stepCrossing(x, st, occ, now);
        check(crossingBell(st, now), "still ringing just before its time is up");

        now = 100.0 + kBellS + 0.5;
        stepCrossing(x, st, occ, now);
        check(!crossingBell(st, now), "silent once kBellS has passed");
        check(st.phase == CrossingPhase::Secured, "while the crossing is still shut");
        check(crossingLights(st.phase).roadRed && crossingLights(st.phase).fast,
              "and the road lights are still flashing red");

        // A later train gets a bell of its own rather than staying silent.
        now += 200.0;
        occ.inner = true;
        stepCrossing(x, st, occ, now);   // release
        occ.inner = false;
        now += kTrainDelayS + 1.0;
        stepCrossing(x, st, occ, now);   // Opening -> Idle
        while (st.phase != CrossingPhase::Idle && now < 1e5) {
            now += 1.0;
            stepCrossing(x, st, occ, now);
        }
        check(st.phase == CrossingPhase::Idle, "the crossing opens again");
        occ.outerB = true;
        stepCrossing(x, st, occ, now);
        check(crossingBell(st, now), "and the next train rings the bell afresh");
    }

    // The barriers run on a clock that does not line up with the phases at either end.
    //
    // They are still up for the first 2 s of Secured, and still coming up for 3 s after
    // Opening has handed back to Idle - so anything that read the boom position off the
    // phase would be wrong twice per train, and would snap rather than sweep.
    {
        std::puts("\nA half-barrier crossing, lowering and lifting:");
        Rig r;
        r.x.barriers = true;
        check(r.st.barrier == 0.0f, "the booms start up");

        r.set(true, false, false); // a train arms the approach
        r.run(kBarrierDelayS - 1.0);
        check(r.st.barrier == 0.0f, "still up a second before their delay is out");
        check(r.phase() == CrossingPhase::Secured,
              "even though the crossing is already secured");

        r.run(2.0); // past the 7 s
        check(r.st.barrier > 0.0f && r.st.barrier < 1.0f, "on their way down after it");
        check(r.st.barrierMoving(), "and reported as moving, so the mesh is rebuilt");

        // Fully down at 7 + 8 = 15 s, which is the kSequenceS the approach already allows.
        r.run(kBarrierTravelS);
        check(r.st.barrier == 1.0f, "fully down 8 s after they started");
        check(!r.st.barrierMoving(), "and no longer moving");

        r.set(false, true, false); // the train arrives on the crossing
        r.run(3.0);
        check(r.st.barrier == 1.0f, "held down while the train is on it");

        r.set(false, false, true); // off the inner circuit
        check(r.phase() == CrossingPhase::Opening, "clearing the inner starts the release");
        r.run(1.0);
        check(r.st.barrier < 1.0f, "the booms start lifting at once, with the red delay");

        // The part a phase-derived barrier gets wrong: Opening is only 5 s and the lift
        // takes 8, so the booms are still rising after the crossing has reopened.
        r.run(4.5);
        check(r.phase() == CrossingPhase::Idle, "the crossing is back to idle");
        check(r.st.barrier > 0.0f, "with the booms still on their way up");
        r.run(3.0);
        check(r.st.barrier == 0.0f, "fully up 8 s after the lift began");
    }

    // The booms keep their own 8 s whatever the phases do around them.
    //
    // A second train can arm the crossing while they are still coming up - Opening runs
    // only 5 s of their 8 - and they must go on rising to the top rather than being
    // yanked by the new sequence. They are moved at a rate toward where they ought to be,
    // never placed from a timestamp, so this needs no special case and cannot jump.
    {
        std::puts("\nA second train while the booms are still coming up:");
        Rig r;
        r.x.barriers = true;
        r.set(true, false, false);
        r.run(kBarrierDelayS + kBarrierTravelS + 1.0);
        check(r.st.barrier == 1.0f, "the first train has them fully down");

        r.set(false, true, false);
        r.set(false, false, true); // and is off the crossing
        r.run(kTrainDelayS + 0.5);
        const float caught = r.st.barrier;
        check(r.phase() == CrossingPhase::Idle, "the crossing reopens after its delay");
        check(caught > 0.0f && caught < 1.0f, "with the booms still part-way up");

        r.set(false, false, false); // clear, so the next arrival is a fresh edge
        r.set(true, false, false);
        check(r.phase() == CrossingPhase::Closing, "the next train arms it again");
        check(r.st.barrier > 0.0f && r.st.barrier <= caught,
              "the booms are still where they were, not snapped anywhere");

        r.run(3.0); // the rest of their own 8 s
        check(r.st.barrier == 0.0f, "they finish coming up on their own clock");
        r.run(kBarrierDelayS - 3.0 + 1.0);
        check(r.st.barrier > 0.0f, "and only then start down for the new train");
        r.run(kBarrierTravelS);
        check(r.st.barrier == 1.0f, "reaching fully down 8 s after that");
    }

    // A crossing secured by lights alone has nothing to move, and must be untouched by
    // any of this.
    {
        std::puts("\nA crossing without barriers:");
        Rig r; // r.x.barriers stays false
        r.set(true, false, false);
        r.run(kBarrierDelayS + kBarrierTravelS + 5.0);
        check(r.phase() == CrossingPhase::Secured, "runs its sequence as before");
        check(r.st.barrier == 0.0f, "and never moves a boom");
        check(!r.st.barrierMoving(), "so it never asks for a rebuild");
    }

    // Where the distants stand. A repeat is only worth having if a driver reading it still
    // has room to stop, and only safe if the train has already armed the crossing before
    // it can be read - so it is four fifths of the braking distance, but never so far out
    // that it leaves the approach circuit, and never so close that it is inside the inner
    // one.
    {
        std::puts("\nThe crossing's distant signals:");
        auto at = [](double kmh) {
            return distantDistance(kmh, approachDistance(kmh));
        };
        auto braking = [](double kmh) {
            const double v = (kmh > 1.0 ? kmh : 40.0) / 3.6;
            return v * v / (2.0 * kCrossingBrakeDecel);
        };
        // The ordinary case: straight four fifths of the braking distance.
        for (double kmh : {60.0, 100.0, 130.0}) {
            const bool ok = std::abs(at(kmh) - kDistantOfBraking * braking(kmh)) < 0.5;
            check(ok, "at " + std::to_string(static_cast<int>(kmh)) +
                          " km/h it is four fifths of the braking distance");
        }
        // Always inside the approach circuit, by enough to have been detected first.
        for (double kmh : {20.0, 60.0, 130.0, 210.0}) {
            const double app = approachDistance(kmh);
            check(at(kmh) < app - 100.0,
                  "at " + std::to_string(static_cast<int>(kmh)) +
                      " km/h it sits well inside the approach circuit");
        }
        // The fast end is where the raw braking figure crowds the circuit: the approach
        // is clamped at kOuterMaxM and the braking distance is not, so above about
        // 213 km/h it would leave the circuit altogether and below that it still eats the
        // margin. Either way the cap is what holds it in.
        check(kDistantOfBraking * braking(210.0) >
                  kDistantOfApproach * approachDistance(210.0),
              "at 210 km/h the braking figure alone would eat the margin");
        check(at(210.0) < approachDistance(210.0) - 100.0, "so it is capped short of it");

        // The slow end is where it would land inside the inner circuit instead.
        check(kDistantOfBraking * braking(20.0) < innerHalfM(),
              "at 20 km/h the braking figure alone would be inside the inner circuit");
        check(at(20.0) > innerHalfM(), "so it is held outside it");
        check(at(20.0) > kSignalOffsetM,
              "and clear of the crossing's own head besides");

        // An unknown limit falls back as the approach distance does, rather than to zero.
        check(std::abs(at(0.0) - at(40.0)) < 0.5,
              "an unknown line speed falls back to 40 km/h, not to nothing");

        // A crossing whose circuits were shortened by hand brings its repeats in too.
        check(distantDistance(130.0, 400.0) < distantDistance(130.0, approachDistance(130.0)),
              "a hand-set shorter approach pulls the distants in with it");
    }

    // A station needs several positions - a TXP at one end of Fauske cannot be seen from
    // the other - and the editor has to draw all of them while the sim draws only the one
    // being signalled. Easy to collapse into a single rule by handing the editor the same
    // showing vector the sim uses, which would leave the author unable to see what a
    // station covers. Driven on the mesh, over a straight synthetic track, no dataset.
    {
        std::puts("\nEvery position is drawn in the editor, only the shown one in the sim:");
        std::vector<TrackPoly> polys;
        TrackPoly p;
        p.id = 0x6d7;
        for (int i = 0; i <= 10; ++i) p.pts.push_back({1000.0 * i / 10, 0.0, 0.0});
        polys.push_back(p);

        std::vector<TxpPosition> ps;
        ps.push_back({1, "vest", 0x6d7, 0.20, 1, 1, "Fauske"});
        ps.push_back({2, "midt", 0x6d7, 0.50, 1, 1, "Fauske"});
        ps.push_back({3, "aust", 0x6d7, 0.80, -1, -1, "Fauske"});

        TxpMesh none, one, all;
        none.build(ps, std::vector<char>(ps.size(), 0), polys, glm::dvec3(0.0));
        std::vector<char> justMid(ps.size(), 0);
        justMid[1] = 1;
        one.build(ps, justMid, polys, glm::dvec3(0.0));
        all.build(ps, std::vector<char>(ps.size(), 1), polys, glm::dvec3(0.0));

        const std::size_t per = one.vertices().size();
        check(none.vertices().empty(), "nobody stands out when nothing is showing");
        check(per > 0, "the one being signalled is drawn");
        check(all.vertices().size() == per * ps.size(),
              "the editor draws a figure at every position, not just the shown one");

        // And they must stand apart: three figures on one spot would author as one.
        std::vector<glm::vec3> where;
        for (std::size_t i = 0; i < ps.size(); ++i) {
            std::vector<char> sh(ps.size(), 0);
            sh[i] = 1;
            TxpMesh m;
            m.build(ps, sh, polys, glm::dvec3(0.0));
            glm::vec3 lo(1e9f);
            for (const TrackVertex& v : m.vertices()) lo = glm::min(lo, v.pos);
            where.push_back(lo);
        }
        float closest = 1e9f;
        for (std::size_t i = 0; i < where.size(); ++i)
            for (std::size_t j = i + 1; j < where.size(); ++j)
                closest = std::min(closest, glm::length(where[i] - where[j]));
        check(closest > 5.0f, "the positions stand in genuinely different places");
    }

    if (argc > 1) {
        std::puts("\nThe TXP-position overlay round-trips:");
        std::vector<TxpPosition> ts;
        ts.push_back({1, "Fauske spor 1", 0x6d7, 0.42, 1, 1, ""});
        ts.push_back({2, "Fauske spor 2", 0x6d9, 0.18, -1, -1, "Fauske"});
        check(writeTxpPositions(argv[1], ts), "write");
        const std::vector<TxpPosition> back = loadTxpPositions(argv[1]);
        check(back.size() == 2, "both positions come back");
        bool same = back.size() == ts.size();
        for (std::size_t i = 0; same && i < back.size(); ++i)
            same = back[i].id == ts[i].id && back[i].name == ts[i].name &&
                   back[i].trackId == ts[i].trackId &&
                   std::abs(back[i].frac - ts[i].frac) < 1e-9 &&
                   back[i].dir == ts[i].dir && back[i].side == ts[i].side &&
                   back[i].station == ts[i].station;
        check(same, "ids, names, tracks, fracs, directions, sides and overrides survive");
        check(back.size() > 1 && back[1].dir == -1 && back[1].side == -1,
              "a flipped direction and a flipped side are both kept");
    }

    if (argc > 1) {
        std::puts("\nThe flag-post overlay round-trips:");
        std::vector<FlagPost> ps;
        ps.push_back({1, "Fauske vest", 0x6d7, 0.31, 1, ""});
        ps.push_back({2, "Rognan søndre", 0x6d9, 0.62, -1, "Rognan"});
        check(writeFlagPosts(argv[1], ps), "write");
        const std::vector<FlagPost> back = loadFlagPosts(argv[1]);
        check(back.size() == 2, "both posts come back");
        bool same = back.size() == ps.size();
        for (std::size_t i = 0; same && i < back.size(); ++i)
            same = back[i].id == ps[i].id && back[i].name == ps[i].name &&
                   back[i].trackId == ps[i].trackId &&
                   std::abs(back[i].frac - ps[i].frac) < 1e-9 &&
                   back[i].side == ps[i].side && back[i].station == ps[i].station;
        check(same, "ids, names, tracks, fracs, sides and overrides survive");
        check(back.size() > 1 && back[0].side == 1 && back[1].side == -1,
              "a flipped side is kept as flipped");
    }

    // A link connector normally takes the medium of the ends it joins, which is right
    // until a hole runs through a hill between two *surface* ends: the piece the export
    // dropped was a tunnel, and carving it instead cuts a trench the depth of the hill.
    // The keyword says so outright.
    if (argc > 1) {
        std::puts("\nA link can name its own medium:");
        std::vector<TrackEdit> es(3);
        es[0].kind = TrackEdit::Link;
        es[0].a = {100.0, 200.0, 10.0}; es[0].b = {150.0, 260.0, 11.0}; // inferred
        es[1].kind = TrackEdit::Link;
        es[1].a = {300.0, 400.0, 20.0}; es[1].b = {350.0, 460.0, 21.0};
        es[1].medium = 0x55; // through a hill
        es[2].kind = TrackEdit::Link;
        es[2].a = {500.0, 600.0, 30.0}; es[2].b = {550.0, 660.0, 31.0};
        es[2].medium = 0x20; // and the other way round, for a hole in the open
        check(writeTrackOverlay(argv[1], es), "write");
        const std::vector<TrackEdit> back = loadTrackOverlay(argv[1]);
        check(back.size() == 3, "all three links come back");
        check(back.size() > 2 && back[0].medium == 0,
              "no keyword means take it from the ends, as before");
        check(back.size() > 2 && back[1].medium == 0x55, "tunnel survives the round trip");
        check(back.size() > 2 && back[2].medium == 0x20, "and so does surface");
        bool coords = back.size() == es.size();
        for (std::size_t i = 0; coords && i < back.size(); ++i)
            coords = std::abs(back[i].a.x - es[i].a.x) < 1e-3 &&
                     std::abs(back[i].b.z - es[i].b.z) < 1e-3;
        check(coords, "the endpoints are unharmed by the extra token");
    }

    // Two vertices at one spot on one track. The export produces them - a spike is a
    // duplicated point - and until now they could not be told apart: an elev edit matches
    // on position, both are equally near it, and the first found always won. So one of
    // the pair could be corrected and the other could not be reached at all.
    {
        std::puts("\nTwo track vertices at the same spot:");
        std::vector<TrackSegment> segs(1);
        segs[0].trackId = 0x684;
        segs[0].medium = 0x20;
        segs[0].pts = {{0.0, 0.0, 10.0},
                       {100.0, 0.0, 20.0},   // the spike, duplicated
                       {100.0, 0.0, 20.0},
                       {200.0, 0.0, 9.0}};

        // Without the discriminator both edits land on the first of the pair, which is
        // exactly the trap: it looks as though the second point cannot be edited.
        std::vector<TrackSegment> a = segs;
        std::vector<TrackEdit> plain(2);
        for (TrackEdit& e : plain) {
            e.kind = TrackEdit::Elev;
            e.a = {100.0, 0.0, 9.5};
            e.track = 0x684;
        }
        applyTrackOverlay(a, plain);
        check(std::abs(a[0].pts[1].z - 9.5) < 1e-6, "the first of the pair is corrected");
        check(std::abs(a[0].pts[2].z - 20.0) < 1e-6,
              "and the second is untouched however many times it is tried");

        // With it, the first edit moves one out of the way and the second finds the other.
        std::vector<TrackSegment> b = segs;
        std::vector<TrackEdit> keyed(2);
        for (TrackEdit& e : keyed) {
            e.kind = TrackEdit::Elev;
            e.a = {100.0, 0.0, 9.5};
            e.track = 0x684;
            e.hasFromZ = true;
            e.fromZ = 20.0;
        }
        applyTrackOverlay(b, keyed);
        check(std::abs(b[0].pts[1].z - 9.5) < 1e-6 && std::abs(b[0].pts[2].z - 9.5) < 1e-6,
              "naming the height they have now reaches both");
        check(std::abs(b[0].pts[0].z - 10.0) < 1e-6 &&
                  std::abs(b[0].pts[3].z - 9.0) < 1e-6,
              "and nothing either side of them moves");

        // The height names *which* vertex, not what to search near: it must still only
        // match inside the position tolerance.
        std::vector<TrackSegment> c = segs;
        std::vector<TrackEdit> far(1);
        far[0].kind = TrackEdit::Elev;
        far[0].a = {160.0, 0.0, 1.0}; // 40 m from any vertex
        far[0].track = 0x684;
        far[0].hasFromZ = true;
        far[0].fromZ = 20.0;
        applyTrackOverlay(c, far);
        bool moved = false;
        for (std::size_t i = 0; i < c[0].pts.size(); ++i)
            if (std::abs(c[0].pts[i].z - segs[0].pts[i].z) > 1e-6) moved = true;
        check(!moved, "a height given far from any vertex still matches nothing");
    }

    if (argc > 1) {
        std::puts("\nThe elev discriminator round-trips:");
        std::vector<TrackEdit> es(2);
        es[0].kind = TrackEdit::Elev;
        es[0].a = {10.0, 20.0, 30.0};
        es[0].track = 0x684;
        es[1] = es[0];
        es[1].hasFromZ = true;
        es[1].fromZ = 571.789;
        check(writeTrackOverlay(argv[1], es), "write");
        const std::vector<TrackEdit> back = loadTrackOverlay(argv[1]);
        check(back.size() == 2, "both come back");
        check(back.size() > 1 && !back[0].hasFromZ, "one without keeps its track id only");
        check(back.size() > 1 && back[0].track == 0x684, "which survives");
        check(back.size() > 1 && back[1].hasFromZ &&
                  std::abs(back[1].fromZ - 571.789) < 1e-6,
              "and the other keeps the height that names its vertex");
    }

    // Who may pass a train order to whom. Derived, not authored: a station is a TXP
    // station because a position was placed at it, and its neighbours are the stations
    // either side of it on the running line. Driven here over a made-up railway, so what
    // is being tested is the derivation and not one dataset's geography.
    {
        std::puts("\nThe TXP graph derives itself from the positions:");
        // A 3000 m running line, and a short siding beside the middle station.
        std::vector<glm::dvec3> main3;
        for (int i = 0; i <= 30; ++i) main3.push_back({i * 100.0, 0.0, 0.0});
        std::vector<glm::dvec3> siding = {{1400.0, 20.0, 0.0}, {1600.0, 20.0, 0.0}};
        // And a second line elsewhere, standing in for one severed from the first.
        std::vector<glm::dvec3> other;
        for (int i = 0; i <= 10; ++i) other.push_back({i * 100.0, 9000.0, 0.0});

        std::vector<TrackPoly> polys = {{1, main3}, {2, siding}, {3, other}};
        auto mkPath = [](std::uint32_t id, const std::vector<glm::dvec3>& w) {
            std::vector<glm::vec3> p;
            std::vector<std::uint16_t> spd;
            for (const glm::dvec3& q : w) {
                p.push_back({(float)q.x, (float)q.y, (float)q.z});
                spd.push_back(100);
            }
            return TrackPath(id, 0, p, spd);
        };
        std::vector<TrackPath> paths;
        paths.push_back(mkPath(1, main3));
        paths.push_back(mkPath(2, siding));
        paths.push_back(mkPath(3, other));

        std::vector<TxpPosition> ps(6);
        auto at = [&](int i, std::uint32_t t, double f) { ps[i].id = i + 1;
                                                          ps[i].trackId = t; ps[i].frac = f; };
        at(0, 1, 0.10);  // Alpha, on the running line
        at(1, 1, 0.50);  // Beta, likewise
        at(2, 2, 0.50);  // Beta again, but out on its siding
        at(3, 1, 0.90);  // Gamma
        at(4, 3, 0.20);  // Delta, on the other line
        at(5, 3, 0.80);  // Epsilon, likewise
        std::vector<SignalStation> att(6);
        const char* names[6] = {"Alpha", "Beta", "Beta", "Gamma", "Delta", "Epsilon"};
        for (int i = 0; i < 6; ++i) att[i].name = names[i];

        TxpGraph g;
        g.build(ps, att, {}, polys, paths, glm::dvec3(0.0));

        check(g.nodes().size() == 5, "one node per station, not one per position");
        // Beta has a position on a siding as well; it must be ordered by the running
        // line, which is the only one the next station is reached along.
        int ib = -1;
        for (std::size_t i = 0; i < g.nodes().size(); ++i)
            if (g.nodes()[i].name == "Beta") ib = (int)i;
        check(ib >= 0 && g.nodes()[ib].path == 0,
              "a station with a siding is placed on the running line");

        check(g.linked("Alpha", "Beta") && g.linked("Beta", "Gamma"),
              "consecutive stations on one line are neighbours");
        check(g.linked("Beta", "Alpha"), "and the link reads the same both ways round");
        check(!g.linked("Alpha", "Gamma"),
              "but a station two along is not - there is one between them");

        check(g.neighbours("Beta").size() == 2, "a station in the middle deals with two");
        check(g.neighbours("Alpha").size() == 1, "one at the end of the line with one");
        check(g.neighbours("Nowhere").empty(), "and a station with no TXP with nobody");

        check(g.next("Alpha", true) == "Beta" && g.next("Beta", true) == "Gamma",
              "next walks down the line");
        check(g.next("Gamma", false) == "Beta", "and back up it");
        check(g.next("Gamma", true).empty(),
              "with nobody to offer a train to past the last station");

        // The severed-line case: two stations that cannot reach each other.
        check(g.linked("Delta", "Epsilon"), "the other line chains on its own");
        check(!g.linked("Gamma", "Delta"),
              "and nothing is offered across a break to a station on another line");
    }

    // Manning a station is not a local act.
    //
    // The chain that actually works the line is the *manned* stations and the sections
    // between them. A station opening between two others takes over a section that was
    // already theirs, so it has to ask and they have to agree - and they may only agree
    // if that section is clear, because a station cannot appear in the middle of one with
    // a train in it and hold a road it was never told about.
    {
        std::puts("\nOpening stations onto the train-order network:");
        // Bodø - Oteråga - Fauske - Rognan along one line, as the export gives them.
        std::vector<glm::dvec3> line;
        for (int i = 0; i <= 40; ++i) line.push_back({i * 100.0, 0.0, 0.0});
        std::vector<TrackPoly> polys = {{1, line}};
        std::vector<glm::vec3> lp;
        std::vector<std::uint16_t> spd;
        for (const glm::dvec3& q : line) { lp.push_back({(float)q.x, 0.0f, 0.0f}); spd.push_back(100); }
        std::vector<TrackPath> paths;
        paths.push_back(TrackPath(1, 0, lp, spd));

        const char* names[4] = {"Bodø", "Oteråga", "Fauske", "Rognan"};
        const double fracs[4] = {0.05, 0.35, 0.65, 0.95};
        std::vector<TxpPosition> ps(4);
        std::vector<SignalStation> att(4);
        for (int i = 0; i < 4; ++i) {
            ps[i].id = i + 1; ps[i].trackId = 1; ps[i].frac = fracs[i];
            att[i].name = names[i];
        }
        TxpGraph g;
        g.build(ps, att, {}, polys, paths, glm::dvec3(0.0));

        // The line is clear unless this says otherwise.
        std::string occupied;
        auto clear = [&](const std::string& a, const std::string& b) {
            return occupied.empty() || !(occupied == a + "-" + b || occupied == b + "-" + a);
        };
        auto kinds = [](const TxpExchange& r) {
            std::string s;
            for (const TxpMessage& m : r.exchange)
                s += (m.kind == TxpMsgKind::Connect ? "C" :
                      m.kind == TxpMsgKind::Accept ? "A" : "R");
            return s;
        };

        TxpNetwork net;

        // Bodø first: nobody to ask.
        TxpExchange r = net.open(g, "Bodø", clear);
        check(r.accepted, "the first station opens");
        check(r.exchange.empty(), "with nothing sent - there is nobody to ask");
        check(net.links().empty(), "and no section is worked yet");

        // Fauske next, two along: one neighbour to ask.
        r = net.open(g, "Fauske", clear);
        check(r.accepted && kinds(r) == "CA", "the second sends one connect and is accepted");
        check(r.exchange.size() > 0 && r.exchange[0].to == "Bodø", "it asks Bodø");
        check(net.linked("Bodø", "Fauske"), "and the two now work the line between them");
        check(net.links().size() == 1, "which is the only section");

        // Oteråga sits *between* them: it must ask both, and takes over their section.
        r = net.open(g, "Oteråga", clear);
        check(r.accepted && kinds(r) == "CCAA", "opening between two asks both of them");
        check(net.linked("Bodø", "Oteråga") && net.linked("Oteråga", "Fauske"),
              "and it works a section with each");
        check(!net.linked("Bodø", "Fauske"),
              "the section that spanned it is gone - it did not stay alongside");
        check(net.links().size() == 2, "one long section has become two");

        // Closing it hands the whole of what it held back to the two either side.
        net.close(g, "Oteråga");
        check(net.linked("Bodø", "Fauske") && net.links().size() == 1,
              "closing it joins the two either side again");
        check(!net.isOpen("Oteråga"), "and it is out of the network");

        // The same opening, refused: a train stands between Bodø and Fauske.
        occupied = "Bodø-Fauske";
        r = net.open(g, "Oteråga", clear);
        check(!r.accepted && kinds(r) == "CCRR", "an occupied section refuses both connects");
        check(!r.exchange.empty() && !r.exchange.back().reason.empty(),
              "and says why");
        check(!net.isOpen("Oteråga"), "the station does not open");
        check(net.linked("Bodø", "Fauske") && net.links().size() == 1,
              "and nothing about the section it asked for moves");

        // Once the train has gone it opens as before.
        occupied.clear();
        r = net.open(g, "Oteråga", clear);
        check(r.accepted && net.links().size() == 2, "with the line clear it opens");

        // Rognan at the far end: only one neighbour, and it is Fauske - not Oteråga,
        // which is manned but has Fauske between it and Rognan.
        r = net.open(g, "Rognan", clear);
        check(r.accepted && kinds(r) == "CA", "the end of the line asks its one neighbour");
        check(r.exchange.size() > 0 && r.exchange[0].to == "Fauske",
              "which is the nearest manned station, not the first one opened");
        check(net.linksOf("Fauske").size() == 2, "Fauske now works a section either side");
        check(net.linksOf("Rognan").size() == 1, "and Rognan only the one");

        // A station nobody put a TXP at is not part of this, and says nothing.
        r = net.open(g, "Røkland", clear);
        check(r.accepted && r.exchange.empty(),
              "a station with no TXP position opens without asking anyone");
        check(!net.isOpen("Røkland"), "and works no sections");
    }

    // Dispatching a train, which is a line being asked for, given, held and released.
    //
    // The section is the unit. It belongs to both ends equally, so there is one record and
    // not one per station - but an order on it runs one way, and that is what decides
    // which end may report the train onto the line and which may report it arrived.
    {
        std::puts("\nDispatching a train between two stations:");
        std::vector<glm::dvec3> line;
        for (int i = 0; i <= 40; ++i) line.push_back({i * 100.0, 0.0, 0.0});
        std::vector<TrackPoly> polys = {{1, line}};
        std::vector<glm::vec3> lp;
        std::vector<std::uint16_t> spd;
        for (const glm::dvec3& q : line) { lp.push_back({(float)q.x, 0.0f, 0.0f}); spd.push_back(100); }
        std::vector<TrackPath> paths;
        paths.push_back(TrackPath(1, 0, lp, spd));
        const char* names[4] = {"Bodø", "Oteråga", "Fauske", "Rognan"};
        const double fracs[4] = {0.05, 0.35, 0.65, 0.95};
        std::vector<TxpPosition> ps(4);
        std::vector<SignalStation> att(4);
        for (int i = 0; i < 4; ++i) {
            ps[i].id = i + 1; ps[i].trackId = 1; ps[i].frac = fracs[i];
            att[i].name = names[i];
        }
        TxpGraph g;
        g.build(ps, att, {}, polys, paths, glm::dvec3(0.0));

        auto kinds = [](const TxpExchange& r) {
            std::string s;
            for (const TxpMessage& m : r.exchange)
                s += m.kind == TxpMsgKind::Connect     ? "C"
                     : m.kind == TxpMsgKind::Accept    ? "A"
                     : m.kind == TxpMsgKind::Reject    ? "R"
                     : m.kind == TxpMsgKind::Request   ? "q"
                     : m.kind == TxpMsgKind::LineClear ? "c"
                     : m.kind == TxpMsgKind::OnTrack   ? "t"
                     : m.kind == TxpMsgKind::Arrived   ? "v"
                                                       : "x";
            return s;
        };
        TxpNetwork net;
        net.open(g, "Bodø", {});
        net.open(g, "Oteråga", {});
        net.open(g, "Fauske", {});
        check(net.links().size() == 2, "three manned stations work two sections");

        // The whole sequence, one section at a time.
        TxpExchange r = net.requestDispatch("Bodø", "Oteråga", TxpTrainType::Passenger);
        check(r.accepted && kinds(r) == "qc", "a request on a clear line is granted");
        check(net.state("Bodø", "Oteråga") == TxpLineState::Prepared,
              "and the line is prepared for the train");
        check(net.state("Oteråga", "Bodø") == TxpLineState::Prepared,
              "which both ends see - there is one section, not one record each");
        check(net.state("Oteråga", "Fauske") == TxpLineState::Clear,
              "the next section along is untouched");

        r = net.trainOnTrack("Bodø", "Oteråga");
        check(r.accepted && kinds(r) == "t", "the train is reported onto the line");
        check(net.state("Bodø", "Oteråga") == TxpLineState::Occupied, "which now holds it");

        r = net.trainArrived("Oteråga", "Bodø");
        check(r.accepted && kinds(r) == "v", "the far end reports it arrived");
        check(net.state("Bodø", "Oteråga") == TxpLineState::Clear, "and the line is clear");

        // And is usable again straight away.
        r = net.requestDispatch("Oteråga", "Bodø", TxpTrainType::Cargo);
        check(r.accepted, "the line can be given again, and the other way round");
        check(net.section("Bodø", "Oteråga")->type == TxpTrainType::Cargo,
              "carrying the type it was asked for");
        check(net.section("Bodø", "Oteråga")->from == "Oteråga",
              "and remembering which end it was given to");

        // Nothing else may have it meanwhile.
        r = net.requestDispatch("Bodø", "Oteråga", TxpTrainType::Passenger);
        check(!r.accepted && kinds(r) == "qR", "a second request on a held line is refused");
        r = net.requestDispatch("Oteråga", "Bodø", TxpTrainType::Other);
        check(!r.accepted, "refused from the end that was given it, too");

        // Only the right end may send each message.
        check(!net.trainOnTrack("Bodø", "Oteråga").accepted,
              "the end that was not given the line cannot report a train onto it");
        net.trainOnTrack("Oteråga", "Bodø");
        check(net.state("Bodø", "Oteråga") == TxpLineState::Occupied, "the right end can");
        check(!net.trainArrived("Oteråga", "Bodø").accepted,
              "and cannot then report the arrival of its own train");
        check(net.trainArrived("Bodø", "Oteråga").accepted,
              "which is for the end it is running to");
        check(net.state("Bodø", "Oteråga") == TxpLineState::Clear, "releasing the line");

        // Nothing works on a line with nothing booked on it.
        check(!net.trainOnTrack("Bodø", "Oteråga").accepted,
              "no train can be reported onto a line nobody asked for");
        check(!net.trainArrived("Bodø", "Oteråga").accepted,
              "nor can one arrive off it");
        check(!net.requestDispatch("Bodø", "Rognan", TxpTrainType::Other).accepted,
              "and no line can be asked for between stations that work none");

        // Cancelling, which is only until the train has left.
        r = net.requestDispatch("Bodø", "Oteråga", TxpTrainType::Passenger);
        check(r.accepted, "a line is given");
        check(!net.cancelDispatch("Oteråga", "Bodø").accepted,
              "the far end cannot withdraw someone else's train");
        r = net.cancelDispatch("Bodø", "Oteråga");
        check(r.accepted && kinds(r) == "x", "the end that asked can withdraw it");
        check(net.state("Bodø", "Oteråga") == TxpLineState::Clear, "and the line is clear");
        net.requestDispatch("Bodø", "Oteråga", TxpTrainType::Passenger);
        net.trainOnTrack("Bodø", "Oteråga");
        check(!net.cancelDispatch("Bodø", "Oteråga").accepted,
              "once it has left it cannot be withdrawn - only its arrival clears the line");

        // What a held section blocks, which is the point of holding it.
        check(!net.close(g, "Bodø").accepted,
              "a station holding a train order cannot close");
        check(!net.close(g, "Oteråga").accepted, "nor can the far end of that order");
        check(net.close(g, "Fauske").accepted, "one clear of it still can");
        net.open(g, "Fauske", {});

        // Rognan opening would split Fauske - and beyond, which is clear; but opening a
        // station into the held Bodø - Oteråga section is refused by the books.
        net.close(g, "Oteråga"); // refused, still open - confirm the order survived
        check(net.state("Bodø", "Oteråga") == TxpLineState::Occupied,
              "a refused close leaves the order exactly as it was");

        net.trainArrived("Oteråga", "Bodø"); // clear it down again
        check(net.close(g, "Oteråga").accepted, "and once clear the station can close");
        check(net.linked("Bodø", "Fauske"),
              "the two either side taking the whole of what it held");

        // Now the interaction the user asked for: a held line refuses a station opening
        // into it.
        net.requestDispatch("Bodø", "Fauske", TxpTrainType::Cargo);
        r = net.open(g, "Oteråga", {});
        check(!r.accepted && kinds(r) == "CCRR",
              "opening between two stations is refused while their line is booked");
        check(!net.isOpen("Oteråga"), "so it does not open");
        check(net.linked("Bodø", "Fauske") && net.links().size() == 1,
              "and the section stays whole, with its order intact");
        check(net.state("Bodø", "Fauske") == TxpLineState::Prepared, "still prepared");

        net.cancelDispatch("Bodø", "Fauske");
        check(net.open(g, "Oteråga", {}).accepted,
              "withdrawn, the station opens as it would have");
    }

    // A distant on a main signal's own mast, rather than out on the line on its own post.
    // Two mains on one straight track, the first carrying the distant: what it shows is
    // decided by the second, and by whether the first is at danger.
    std::puts("\nA distant hanging on a main signal's mast:");
    {
        std::vector<TrackPoly> polys(1);
        polys[0].id = 1;
        for (int i = 0; i <= 20; ++i) polys[0].pts.push_back({i * 100.0, 0.0, 0.0}); // 2 km
        const TrackJunctions junctions; // one track, so nothing joins it
        const SwitchNetwork net;        // and no turnout the walk has to resolve

        auto mast = [&](double frac, int dir) {
            SignalPlacement p;
            p.kind = SignalKind::Entry;
            p.at = {1, frac};
            p.forward = trackTangent(polys, 1, frac, dir);
            p.world = fracToWorld(polys, 1, frac);
            return p;
        };
        std::vector<SignalPlacement> ps{mast(0.1, +1), mast(0.5, +1)};
        ps[0].withDistant = true; // the mast with the distant; the other is 800 m ahead
        auto settle = [&](SignalAspect here, SignalAspect ahead) {
            ps[0].aspect = here;
            ps[1].aspect = ahead;
            updateDistantAspects(ps, polys, junctions, net);
            return ps[0].distantAspect;
        };

        check(settle(SignalAspect::Stop, SignalAspect::Clear) == SignalAspect::Dark,
              "switched off entirely while its own main is at danger");
        check(settle(SignalAspect::Clear, SignalAspect::Clear) == SignalAspect::Clear,
              "expect clear once the main lets you by and the road ahead is clear");
        check(settle(SignalAspect::Clear, SignalAspect::ClearReduced) ==
                  SignalAspect::ClearReduced,
              "and expect a clear over a deviation when that is what is ahead");
        check(settle(SignalAspect::Clear, SignalAspect::Stop) == SignalAspect::Stop,
              "expect stop when the signal ahead is at danger");
        check(ps[0].aspect == SignalAspect::Clear,
              "none of which touches what the main under it is showing");

        // It must not read the signal it hangs on - that would have it repeat the very
        // aspect the driver is looking at, and a clear main would show expect-clear at a
        // station with nothing beyond it.
        std::vector<SignalPlacement> alone{ps[0]};
        alone[0].aspect = SignalAspect::Clear;
        updateDistantAspects(alone, polys, junctions, net);
        check(alone[0].distantAspect == SignalAspect::Stop,
              "a mast with nothing ahead of it warns of a stop, and never reads itself");

        // A main facing the other way protects opposing moves; the walk steps over its
        // back, exactly as it does for a distant on its own post.
        std::vector<SignalPlacement> facing{mast(0.1, +1), mast(0.5, -1)};
        facing[0].withDistant = true;
        facing[0].aspect = SignalAspect::Clear;
        facing[1].aspect = SignalAspect::Clear;
        updateDistantAspects(facing, polys, junctions, net);
        check(facing[0].distantAspect == SignalAspect::Stop,
              "a signal ahead facing the other way is not what it repeats");
    }

    // Two routes over one road: a chain or a head-on.
    //
    // Overlapping is not by itself a conflict. An entry route finishes on the platform road
    // that a departure from it begins on, and both are set at once to let a train through a
    // station without stopping it. Facing each other over the same rails is the conflict.
    std::puts("\nTwo routes over the same rails:");
    {
        auto leg = [](std::uint32_t track, double from, double to) {
            SignalPath p;
            p.parts.push_back({track, from, to});
            return p;
        };
        // The through move: in over 0.2 -> 0.6, out over 0.4 -> 0.9, sharing 0.4 -> 0.6.
        check(!routesOppose(leg(1, 0.2, 0.6), leg(1, 0.4, 0.9)),
              "a departure following an arrival is not opposed");
        check(!routesOppose(leg(1, 0.4, 0.9), leg(1, 0.2, 0.6)),
              "and it does not matter which is asked about first");
        check(routesOppose(leg(1, 0.2, 0.6), leg(1, 0.9, 0.4)),
              "the same rails run the other way is");
        check(!routesOppose(leg(1, 0.2, 0.6), leg(1, 0.6, 0.9)),
              "meeting at a border is a join, not an overlap");
        check(!routesOppose(leg(1, 0.6, 0.2), leg(1, 0.9, 0.6)),
              "and that holds whichever way the pair runs");
        check(!routesOppose(leg(1, 0.2, 0.6), leg(2, 0.6, 0.2)),
              "opposite ways on different tracks never meet");

        // Only one leg of several need face the other way.
        SignalPath two = leg(1, 0.2, 0.6);
        two.parts.push_back({2, 0.0, 0.5});
        check(routesOppose(two, leg(2, 0.4, 0.1)),
              "one leg facing the other way is enough");
        check(!routesOppose(two, leg(2, 0.1, 0.4)), "and the same way is still not");
    }

    // How a mast is built is a fact about the mast, and several route records sharing a
    // start border are one mast - so each flag has to survive the file and land on the
    // signal they make up.
    if (argc > 1) {
        std::puts("\nSaying in the overlay how a mast is built:");
        const std::string root = argv[1];
        std::error_code ec;
        std::filesystem::create_directories(root + "/overlay", ec);

        std::vector<TrackPoly> polys(1);
        polys[0].id = 1;
        for (int i = 0; i <= 20; ++i) polys[0].pts.push_back({i * 100.0, 0.0, 0.0});

        auto route = [](int id, double from, double to) {
            SignalPath p;
            p.id = id;
            p.name = "R" + std::to_string(id);
            p.start = {1, from};
            p.end = {1, to};
            p.parts.push_back({1, from, to});
            return p;
        };
        std::vector<SignalPath> es{route(1, 0.1, 0.6), route(2, 0.1, 0.7),
                                   route(3, 0.9, 0.6)};
        es[1].distant = true; // one of the two sharing the border says so; that is enough
        check(writeEntrySignals(root, es), "the entry signals write");

        const std::vector<SignalPath> back = loadEntrySignals(root);
        check(back.size() == 3, "and read back");
        check(back.size() == 3 && !back[0].distant && back[1].distant && !back[2].distant,
              "with the flag on the record that carried it and no other");

        const std::vector<SignalPlacement> ps =
            signalPlacements(back, polys, SignalKind::Entry);
        check(ps.size() == 2, "two borders, so two masts");
        if (ps.size() == 2) {
            check(ps[0].paths.size() == 2 && ps[0].withDistant,
                  "the mast the two routes share carries the distant");
            check(!ps[1].withDistant, "and the one at the other end does not");
        }

        // The two-lamp head is the other thing a record can say about its mast, and the two
        // are independent: a mast may carry either, both or neither. Through the exit
        // signals this time, since both files take both keywords.
        {
            std::vector<SignalPath> xs{route(1, 0.1, 0.6), route(2, 0.1, 0.7),
                                       route(3, 0.9, 0.6)};
            xs[0].twoLamp = true;  // the shared mast: two lamps, no distant
            xs[2].twoLamp = true;  // the far mast: two lamps and a distant
            xs[2].distant = true;
            check(writeExitSignals(root, xs), "the exit signals write");
            const std::vector<SignalPath> r = loadExitSignals(root);
            check(r.size() == 3 && r[0].twoLamp && !r[1].twoLamp && r[2].twoLamp,
                  "the head flag comes back on the records that carried it");
            check(r.size() == 3 && !r[0].distant && r[2].distant,
                  "and the two flags do not stand in for one another");
            const std::vector<SignalPlacement> xp =
                signalPlacements(r, polys, SignalKind::Exit);
            check(xp.size() == 2 && xp[0].twoLamp && !xp[0].withDistant,
                  "a mast with two lamps and no distant");
            check(xp.size() == 2 && xp[1].twoLamp && xp[1].withDistant,
                  "and one with both");
            std::filesystem::remove(root + "/overlay/exit-signals.txt", ec);
        }

        // A file written before the keywords existed has to go on loading unchanged.
        {
            std::ofstream f(root + "/overlay/entry-signals.txt", std::ios::trunc);
            f << "entry 9 \"OLD\" 1:0.2 1:0.8 type C2 1:0.2:0.8\n";
        }
        const std::vector<SignalPath> old = loadEntrySignals(root);
        check(old.size() == 1 && !old[0].distant && !old[0].twoLamp &&
                  old[0].type == RouteType::C2,
              "a record written before the keywords loads, without either");
        std::filesystem::remove(root + "/overlay/entry-signals.txt", ec);
    }

    // The dataset's own script. What is checked here is the host, not any API: that a
    // script runs, that a bad one is survivable, and that what it defines is still there
    // afterwards - which is what makes hooks possible later rather than a rewrite.
    if (argc > 1) {
        std::puts("\nRunning the dataset's script:");
        const std::string root = argv[1];
        std::error_code ec;
        std::filesystem::create_directories(root + "/overlay", ec);
        const std::string path = root + "/overlay/overlay.lua";
        auto put = [&](const char* text) {
            std::ofstream f(path, std::ios::trunc);
            f << text;
        };

        if (!Script::available()) {
            std::puts("  built without Lua - nothing to check");
        } else {
            std::filesystem::remove(path, ec);
            {
                Script s;
                check(!s.run(root), "a dataset with no script runs nothing, and says so"
                                    " by returning false rather than by complaining");
            }
            {
                put("hello_from_lua = 41 + 1\n");
                Script s;
                check(s.run(root), "a script that runs reports that it did");
                check(s.hasGlobal("hello_from_lua"),
                      "and what it defined is still there afterwards");
                check(!s.hasGlobal("never_set_this"), "while what it did not is not");
            }
            {
                // Two runs share one interpreter, which is the point of keeping it open.
                put("first_pass = true\n");
                Script s;
                check(s.run(root), "a first script runs");
                put("second_saw_first = (first_pass == true)\n");
                check(s.run(root), "a second runs in the same interpreter");
                check(s.hasGlobal("second_saw_first"),
                      "and sees what the first one left behind");
            }
            {
                put("print(\"unterminated\n");
                Script s;
                check(!s.run(root), "a script that does not parse fails without throwing");
            }
            {
                put("error('boom')\n");
                Script s;
                check(!s.run(root), "and so does one that goes wrong while running");
            }
            std::filesystem::remove(path, ec); // leave the scratch dir as it was found
        }
    }

    std::printf("\n%s\n", failures ? "FAILED" : "PASSED");
    return failures ? 1 : 0;
}
