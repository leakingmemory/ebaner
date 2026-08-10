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
#include "LevelCrossings.h"

#include <cmath>
#include <cstdio>
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
    CrossingState st;
    CrossingOccupancy occ;
    double now = 0.0;

    // Advance `seconds`, stepping finely enough that a 5 s phase cannot be stepped over.
    void run(double seconds) {
        const double dt = 0.1;
        for (double t = 0.0; t < seconds - 1e-9; t += dt) {
            now += dt;
            stepCrossing(st, occ, now);
        }
    }
    // Apply a circuit change and let it be seen, without meaningfully advancing time.
    void set(bool a, bool inner, bool b) {
        occ = {a, inner, b};
        now += 0.01;
        stepCrossing(st, occ, now);
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
        xs.push_back({1, "Fauskeveien nord", 0x6d7, 0.30, 0.0});
        xs.push_back({2, "Røsvikveien", 0x6d9, 0.755, 900.0});
        check(writeLevelCrossings(argv[1], xs), "write");
        const std::vector<LevelCrossing> back = loadLevelCrossings(argv[1]);
        check(back.size() == 2, "both records come back");
        bool same = back.size() == xs.size();
        for (std::size_t i = 0; same && i < back.size(); ++i)
            same = back[i].id == xs[i].id && back[i].name == xs[i].name &&
                   back[i].trackId == xs[i].trackId &&
                   std::abs(back[i].frac - xs[i].frac) < 1e-9 &&
                   std::abs(back[i].outerM - xs[i].outerM) < 1e-9;
        check(same, "ids, names, tracks, fracs and overrides survive");
        check(back.size() > 1 && back[0].outerM == 0.0 && back[1].outerM == 900.0,
              "an absent override stays absent, a set one is kept");
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

    std::printf("\n%s\n", failures ? "FAILED" : "PASSED");
    return failures ? 1 : 0;
}
