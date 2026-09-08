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

// What a real train hands the track circuits.
//
// Vehicle::occupiedSpans walks the body from its rear-most axle to its front-most and
// reports the stretches of road under it - split wherever the body crosses a turnout onto
// another path. OccupancyTest checks the rule those spans are fed to; this checks the spans
// themselves, on a train that is actually built, driven and parted.
//
// The three things worth proving here are the three that were asked for: that parting a
// train needs nothing undone, that standing still is not a special case, and that a set on
// a siding is on the siding and not on the main beside it.
//
// Usage: TrainSpanTest <datasetRoot>   (exit 77 = skipped, no dataset)

#include "Consist.h"
#include "Occupancy.h"
#include "SwitchNetwork.h"
#include "SwitchTypes.h"
#include "TerrainData.h"
#include "TrackCircuits.h"
#include "TrackPath.h"
#include "Vehicle.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

namespace {

int failures = 0;

void check(bool ok, const char* what, double got, double want) {
    std::printf("  %-54s %9.3f (want %8.3f) %s\n", what, got, want, ok ? "ok" : "FAILED");
    if (!ok) ++failures;
}

// Total length of road a set of spans covers.
double covered(const std::vector<PathSpan>& spans) {
    double m = 0.0;
    for (const PathSpan& s : spans) m += s.s1 - s.s0;
    return m;
}

std::vector<PathSpan> trainSpans(const Consist& t, bool& complete) {
    std::vector<PathSpan> all;
    complete = t.occupiedSpans(all);
    return all;
}

bool sameSpans(const std::vector<PathSpan>& a, const std::vector<PathSpan>& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i)
        if (a[i].pathIdx != b[i].pathIdx || std::abs(a[i].s0 - b[i].s0) > 1e-3f ||
            std::abs(a[i].s1 - b[i].s1) > 1e-3f)
            return false;
    return true;
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
    if (paths.empty()) {
        std::puts("no track geometry - cannot test");
        return 1;
    }
    SwitchNetwork net;
    net.build(data, paths, loadSwitchSuppressions(root));
    applySwitchTypes(net, loadSwitchTypes(root));

    // Somewhere a track circuit actually is. Most of the 417 km of line has none authored
    // on it, and a train parked out there would hold nothing and prove nothing, so the
    // longest section there is settles where to stand.
    const TrackCircuits circuits = loadTrackCircuits(root);
    const SectionSpans secs = resolveSectionSpans(circuits, paths);
    const TrackPath* run = nullptr;
    float startS = 0.0f;
    {
        float best = 0.0f;
        for (const SectionSpans::Entry& e : secs.entries) {
            const float len = e.span.s1 - e.span.s0;
            if (len <= best) continue;
            best = len;
            run = &paths[e.span.pathIdx];
            startS = 0.5f * (e.span.s0 + e.span.s1);
        }
    }
    const VehicleSpec* two = nullptr;
    for (const VehicleSpec& v : kVehicleSpecs)
        if (v.body == BodyClass93 && v.units == 2) two = &v;
    if (run == nullptr || two == nullptr) {
        std::puts("no circuits or no two-set vehicle - cannot test");
        return 1;
    }

    std::puts("\na two-set train standing in the longest circuit there is:");
    Consist t(&paths, run, *two, startS);
    t.attachNetwork(&paths, &net);
    bool complete = false;
    std::vector<PathSpan> spans = trainSpans(t, complete);
    check(complete, "the whole body found rail to stand on", complete ? 1 : 0, 1);
    check(!spans.empty(), "spans reported", static_cast<double>(spans.size()), 1);

    // How long the train is over its axles: the outermost axle of the front set to the
    // outermost of the back one. That is what the spans should add up to, since a circuit
    // senses wheels and the overhangs beyond them are not on the rail electrically.
    double axleLen = 0.0;
    {
        const std::vector<float> offs = t.axleOffsets();
        const auto mm = std::minmax_element(offs.begin(), offs.end());
        axleLen = *mm.second - *mm.first;
    }
    check(std::abs(covered(spans) - axleLen) < 0.5,
          "and they cover the train over its axles (m)", covered(spans), axleLen);

    std::puts("\nthe same train, not moving:");
    {
        // Nothing in occupiedSpans reads a speed. Stepping the train while it is held in
        // emergency must therefore change nothing at all - the case a counter keyed on
        // direction has to be careful about, and this one cannot be.
        const std::vector<PathSpan> before = trainSpans(t, complete);
        for (int i = 0; i < 120; ++i) t.update(1.0f / 60.0f, 0.0f);
        const std::vector<PathSpan> after = trainSpans(t, complete);
        check(std::abs(t.speed()) < 1e-6, "it really is at a stand (m/s)",
              std::abs(t.speed()), 0.0);
        check(sameSpans(before, after), "two seconds later, the same road",
              sameSpans(before, after) ? 1 : 0, 1);
    }

    std::puts("\nparting it:");
    {
        const std::vector<PathSpan> whole = trainSpans(t, complete);
        const char* why = nullptr;
        check(t.mayUncouple(0, why), "the coupler may be parted at a stand",
              t.mayUncouple(0, why) ? 1 : 0, 1);
        std::optional<Consist> rear = t.uncoupleAfter(0);
        check(rear.has_value(), "and it parts", rear.has_value() ? 1 : 0, 1);
        if (rear.has_value()) {
            // The two portions together, exactly as the sim will gather them.
            std::vector<PathSpan> parted = trainSpans(t, complete);
            const std::vector<PathSpan> back = trainSpans(*rear, complete);
            parted.insert(parted.end(), back.begin(), back.end());
            Vehicle::coalesceSpans(parted);
            // The two portions cover a little less road than the one train did, and that
            // is right: the rails between two *coupled* sets lie between two axles of one
            // train and are held, while two portions merely standing near each other hold
            // only what is under them. The difference is one coupler gap.
            const double lost = covered(whole) - covered(parted);
            check(lost > 0.0 && lost < 20.0, "less road by one coupler gap (m)", lost, 0.0);

            // What matters is not the metres but the sections, and those must not move.
            // Occupancy is derived from where each set is; parting a train moves no set, so
            // there is no tally to divide and nothing to recompute - which is the whole
            // reason a split needs no special handling.
            std::vector<char> before, after;
            computeOccupancy(secs, whole, before);
            computeOccupancy(secs, parted, after);
            int heldBefore = 0;
            for (const char c : before) heldBefore += c ? 1 : 0;
            check(heldBefore > 0, "the train holds some circuit at all", heldBefore, 1);
            check(before == after, "and exactly the same ones after parting",
                  before == after ? 1 : 0, 1);
        }
    }

    // A set on a siding is on the siding. The geometric test could not be sure of this
    // where a loop converges on its main, because it decided by distance; here the set
    // carries its own path index and there is nothing to decide.
    std::puts("\na set standing on a siding beside its main:");
    {
        const TrackPath* siding = nullptr;
        for (const TrackPath& p : paths)
            if (p.trackType() == 1 && p.length() > 400.0f &&
                (siding == nullptr || p.length() < siding->length()))
                siding = &p;
        if (siding == nullptr) {
            std::puts("    no siding long enough in this dataset - skipped");
        } else {
            const VehicleSpec* one = nullptr;
            for (const VehicleSpec& v : kVehicleSpecs)
                if (v.body == BodyClass93 && v.units == 1) one = &v;
            Consist s(&paths, siding, *one, 0.5f * siding->length());
            s.attachNetwork(&paths, &net);
            const std::vector<PathSpan> sp = trainSpans(s, complete);
            const int want = static_cast<int>(siding - paths.data());
            int onSiding = 0, elsewhere = 0;
            for (const PathSpan& x : sp) (x.pathIdx == want ? onSiding : elsewhere)++;
            check(onSiding > 0, "its road is the siding's own", onSiding, 1);
            check(elsewhere == 0, "and no part of it is anywhere else", elsewhere, 0);
        }
    }

    // Every span must be a real stretch of its own path, not a number off the end of one.
    std::puts("\nevery span reported is on its path:");
    {
        int bad = 0;
        Consist t2(&paths, run, *two, startS + 300.0f);
        t2.attachNetwork(&paths, &net);
        const std::vector<PathSpan> sp = trainSpans(t2, complete);
        for (const PathSpan& x : sp) {
            if (x.pathIdx < 0 || x.pathIdx >= static_cast<int>(paths.size())) { ++bad; continue; }
            if (x.s0 > x.s1) { ++bad; continue; }
            if (x.s0 < -1e-3f || x.s1 > paths[x.pathIdx].length() + 1e-3f) ++bad;
        }
        check(bad == 0, "spans within their path and correctly ordered", bad, 0);
    }

    std::printf("\n%s\n", failures == 0 ? "all ok" : "FAILURES");
    return failures == 0 ? 0 : 1;
}
