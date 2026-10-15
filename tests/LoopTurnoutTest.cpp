// ebaner - a Norwegian railway simulator.
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

// A reversing loop: one road that leaves a switch, runs round, and comes back onto the
// same switch from the other side. The path builder chains it into a single path whose
// end lies on its own beginning, so the turnout's main and branch are the same path at
// two places far apart along it.
//
// Every loop turnout in the dataset that stands alone at its point is driven through three ways: set for the loop and
// taken facing, which must carry the set round onto the loop's far end; set straight,
// which must leave it on the through road; and come round the loop to the switch from the
// branch, which must put it back on the through road heading out the way it came in.
//
// Usage: LoopTurnoutTest <datasetRoot>   (exit 77 = skipped, no dataset)

#include "SwitchNetwork.h"
#include "SwitchTypes.h"
#include "TerrainData.h"
#include "TrackPath.h"
#include "Vehicle.h"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

namespace {

int failures = 0;

void check(bool ok, const char* what, double got, double want) {
    std::printf("  %-58s %9.1f (want %8.1f) %s\n", what, got, want, ok ? "ok" : "FAILED");
    if (!ok) ++failures;
}

// Whether another turnout stands at the same point. Where several track ends meet on one
// node the detector makes twins, each read onto whichever branch it found, and which of
// them a train meets first decides where it goes - an ambiguity the overlay settles site
// by site with `noswitch`, and not one this test can drive through and call a result.
bool hasTwin(const std::vector<Turnout>& tos, int i) {
    for (int j = 0; j < static_cast<int>(tos.size()); ++j)
        if (j != i && std::hypot(tos[j].world.x - tos[i].world.x,
                                 tos[j].world.y - tos[i].world.y) < 3.0)
            return true;
    return false;
}

// Run a set `dist` metres in `sign` along s, a metre at a time so no turnout is jumped.
void run(Vehicle& v, float dist, float sign) {
    for (float d = 0.0f; d < dist; d += 1.0f) v.advance(sign);
}

} // namespace

int main(int argc, char** argv) {
    const std::string root = argc > 1 ? argv[1] : "../norway-rails";
    if (!std::filesystem::exists(root)) {
        std::printf("no dataset at %s - skipping\n", root.c_str());
        return 77;
    }

    TerrainData data;
    data.load(root, 4000.0);
    const std::vector<TrackPath> paths = buildTrackPaths(data);
    SwitchNetwork net;
    net.build(data, paths, loadSwitchSuppressions(root));
    applySwitchTypes(net, loadSwitchTypes(root));
    const VehicleSpec& spec = kVehicleSpecs[0];

    int loops = 0;
    const std::vector<Turnout>& tos = net.turnouts();
    for (int i = 0; i < static_cast<int>(tos.size()); ++i) {
        const Turnout& t = tos[i];
        if (t.mainPath < 0 || t.mainPath != t.sidingPath || t.inverted) continue;
        if (hasTwin(tos, i)) {
            std::printf("\nloop turnout %x at %.0f %.0f has a twin there - not driven\n",
                        t.sidingTrack, t.world.x, t.world.y);
            continue;
        }
        const float len = paths[t.mainPath].length();
        // Room for the runs below: 50 m either side of the point on the through road, and
        // 50 m of loop behind the branch end.
        if (t.sMain < 60.0f || len - t.sMain < 60.0f || len < 300.0f) continue;
        ++loops;
        std::printf("\nloop turnout %x at %.0f %.0f (path %d, %.0f m, point at %.0f, "
                    "loop end at %.0f):\n",
                    t.sidingTrack, t.world.x, t.world.y, t.mainPath, len, t.sMain, t.sSiding);
        const float f = static_cast<float>(t.facingS);
        const float start = t.sMain - f * 50.0f;
        const TrackPath* p = &paths[t.mainPath];
        // Which way along s the loop is entered from its far end, and how far round a set
        // that took the loop 50 m past the point should then be from that end.
        const float intoLoop = t.sSiding < 0.5f * len ? 1.0f : -1.0f;

        {
            net.setState(i, SwitchState::Diverging);
            Vehicle v(p, spec, start);
            v.attachNetwork(&paths, &net);
            run(v, 100.0f, f);
            check(v.pathIdx() == t.mainPath && std::abs(v.s() - (t.sSiding + intoLoop * 50.0f)) < 2.0f,
                  "set for the loop, facing: it goes round, onto the loop's end", v.s(),
                  t.sSiding + intoLoop * 50.0f);
        }
        {
            net.setState(i, SwitchState::Straight);
            Vehicle v(p, spec, start);
            v.attachNetwork(&paths, &net);
            run(v, 100.0f, f);
            check(std::abs(v.s() - (t.sMain + f * 50.0f)) < 2.0f,
                  "set straight: it stays on the through road", v.s(), t.sMain + f * 50.0f);
        }
        {
            // Back round: 50 m inside the loop, heading for its end, switch set for it.
            net.setState(i, SwitchState::Diverging);
            Vehicle v(p, spec, t.sSiding + intoLoop * 50.0f);
            v.attachNetwork(&paths, &net);
            run(v, 100.0f, -intoLoop);
            check(v.pathIdx() == t.mainPath && std::abs(v.s() - (t.sMain - f * 50.0f)) < 2.0f,
                  "round the loop to the switch: back out the way it came in", v.s(),
                  t.sMain - f * 50.0f);
            check(net.state(i) == SwitchState::Diverging, "  and the switch is not run through",
                  static_cast<double>(net.state(i)), static_cast<double>(SwitchState::Diverging));
        }
        net.setState(i, SwitchState::Straight);
    }
    check(loops > 0, "loop turnouts found in the dataset", loops, 1);

    std::printf("\n%s\n", failures == 0 ? "all ok" : "FAILURES");
    return failures == 0 ? 0 : 1;
}
