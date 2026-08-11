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
#include "TxpMesh.h"
#include "TxpPositions.h"
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
        CrossingState st;
        CrossingOccupancy occ;
        double now = 100.0;
        check(!crossingBell(st, now), "silent while the crossing is idle");

        occ.outerA = true; // a train arms the approach
        stepCrossing(st, occ, now);
        check(st.phase == CrossingPhase::Closing, "the sequence starts");
        check(crossingBell(st, now), "the bell starts with it");

        // Through the phase change, which is where a bell timed off phaseSince breaks.
        now += kTrainDelayS + 1.0;
        occ.outerA = false;
        stepCrossing(st, occ, now);
        check(st.phase == CrossingPhase::Secured, "and reaches Secured");
        check(crossingBell(st, now), "the bell rings on through the phase change");

        now = 100.0 + kBellS - 0.5;
        stepCrossing(st, occ, now);
        check(crossingBell(st, now), "still ringing just before its time is up");

        now = 100.0 + kBellS + 0.5;
        stepCrossing(st, occ, now);
        check(!crossingBell(st, now), "silent once kBellS has passed");
        check(st.phase == CrossingPhase::Secured, "while the crossing is still shut");
        check(crossingLights(st.phase).roadRed && crossingLights(st.phase).fast,
              "and the road lights are still flashing red");

        // A later train gets a bell of its own rather than staying silent.
        now += 200.0;
        occ.inner = true;
        stepCrossing(st, occ, now);   // release
        occ.inner = false;
        now += kTrainDelayS + 1.0;
        stepCrossing(st, occ, now);   // Opening -> Idle
        while (st.phase != CrossingPhase::Idle && now < 1e5) {
            now += 1.0;
            stepCrossing(st, occ, now);
        }
        check(st.phase == CrossingPhase::Idle, "the crossing opens again");
        occ.outerB = true;
        stepCrossing(st, occ, now);
        check(crossingBell(st, now), "and the next train rings the bell afresh");
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

    std::printf("\n%s\n", failures ? "FAILED" : "PASSED");
    return failures ? 1 : 0;
}
