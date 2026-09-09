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

// Putting a train on the line: where one goes, and when it may not go there at all.
//
// The interesting cases are all refusals. A train dropped on a path too short for it has
// its outer sets pinned at the arc-length clamp, draws as a heap of triangles and derails
// on the first step with nothing said - so a short path has to be refused before anything
// is built, not diagnosed afterwards. A train dropped where another one is standing
// interpenetrates it and the track circuits read the pair as one.
//
// No dataset. TrackPath takes points directly, so a few straight lines are the whole world
// here, as in UncoupleTest.

#include "Consist.h"
#include "Stations.h"
#include "TrackPath.h"
#include "TrainYard.h"
#include "Vehicle.h"

#include <cmath>
#include <cstdio>
#include <deque>
#include <string>
#include <vector>

namespace {

int failures = 0;

void check(bool ok, const std::string& what) {
    std::printf("  %-64s %s\n", what.c_str(), ok ? "ok" : "FAILED");
    if (!ok) ++failures;
}

void check(bool ok, const std::string& what, double got, double want) {
    std::printf("  %-64s %s (got %g, want %g)\n", what.c_str(), ok ? "ok" : "FAILED", got,
                want);
    if (!ok) ++failures;
}

VehicleSpec class93(int units) {
    VehicleSpec sp{};
    for (const VehicleSpec& v : kVehicleSpecs)
        if (v.body == BodyClass93 && v.units == 1) sp = v;
    sp.units = units;
    return sp;
}

// A straight length of line along +x at `y`, `len` metres long, of the given track type
// (0 main, 1 siding).
TrackPath line(std::uint32_t id, float y, float len, std::uint8_t type = 0) {
    std::vector<glm::vec3> pts;
    for (float x = 0.0f; x <= len + 0.01f; x += 25.0f) pts.push_back({x, y, 0.0f});
    return TrackPath(id, type, pts, std::vector<std::uint16_t>(pts.size(), 100));
}

} // namespace

int main() {
    std::puts("\nwhere a train goes:");
    {
        // Three parallel mains 500 m apart. Asking beside the middle one must land on the
        // middle one - the point of the search is that it finds the *nearest* road, and a
        // spawn that quietly picked another would put the train a field away.
        std::vector<TrackPath> paths;
        paths.push_back(line(1u, 0.0f, 1000.0f));
        paths.push_back(line(2u, 500.0f, 1000.0f));
        paths.push_back(line(3u, 1000.0f, 1000.0f));

        const Placement p = placeTrainNear(paths, glm::vec2(400.0f, 505.0f), class93(1));
        check(p.path != nullptr, "a placement is found");
        check(p.pathIdx == 1, "and it is the nearest line, not another", p.pathIdx, 1);
        check(std::abs(p.s - 400.0f) < 6.0f, "at the point asked for, to a sample step (m)",
              p.s, 400.0f);
    }

    std::puts("\nand where it does not:");
    {
        // A siding is not somewhere to drop a train from a menu: it may be shorter than the
        // train, and it is not where a train waiting to run belongs.
        std::vector<TrackPath> paths;
        paths.push_back(line(1u, 0.0f, 1000.0f, 1));
        const Placement p = placeTrainNear(paths, glm::vec2(400.0f, 0.0f), class93(1));
        check(p.path == nullptr, "a siding is not offered");
        check(p.why != nullptr, "and it says why");
    }
    {
        std::vector<TrackPath> paths;
        // 30 m of line and a Class 93 whose outer axles are 32.5 m apart. What has to be
        // on the rail is the axles, not the body: 60 m of line does hold one, with the
        // ends hanging over. 30 m does not.
        paths.push_back(line(1u, 0.0f, 30.0f));
        const Placement p = placeTrainNear(paths, glm::vec2(15.0f, 0.0f), class93(1));
        check(p.path == nullptr, "30 m of line will not hold a Class 93");
        check(p.why != nullptr && std::string(p.why).find("long enough") != std::string::npos,
              "and the reason names the length");
    }
    {
        // A station on a line this export does not carry. Something is always "nearest",
        // and without a limit the train lands on it - hundreds of kilometres from the name
        // that was asked for, with nothing to say so.
        std::vector<TrackPath> paths;
        paths.push_back(line(1u, 0.0f, 1000.0f));
        const Placement far = placeTrainNear(paths, glm::vec2(0.0f, 400000.0f), class93(1));
        check(far.path == nullptr, "400 km from any line is refused, not answered");
        check(far.why != nullptr && std::string(far.why).find("near") != std::string::npos,
              "and the reason says it is nowhere near");

        const Placement near = placeTrainNear(paths, glm::vec2(500.0f, 3000.0f), class93(1));
        check(near.path != nullptr, "but 3 km off the line is still near enough");
    }
    {
        std::vector<TrackPath> paths;
        const Placement p = placeTrainNear(paths, glm::vec2(0.0f, 0.0f), class93(1));
        check(p.path == nullptr, "nowhere at all is refused rather than crashed");
    }

    std::puts("\nthree sets need more road than one:");
    {
        std::vector<TrackPath> paths;
        paths.push_back(line(1u, 0.0f, 100.0f));
        const Placement one = placeTrainNear(paths, glm::vec2(50.0f, 0.0f), class93(1));
        const Placement three = placeTrainNear(paths, glm::vec2(50.0f, 0.0f), class93(3));
        check(one.path != nullptr, "100 m holds one set");
        check(three.path == nullptr, "but not three");
        check(three.half > one.half, "and the three-set half-length is the greater (m)",
              three.half, one.half);

        // Long enough for three, and the margin has to keep the outer axles on the rail:
        // the spot chosen must be at least `half` from either end.
        std::vector<TrackPath> big;
        big.push_back(line(1u, 0.0f, 1000.0f));
        const Placement p = placeTrainNear(big, glm::vec2(0.0f, 0.0f), class93(3));
        check(p.path != nullptr, "1000 m holds three sets");
        check(p.s >= p.half, "and asking at the very end still keeps them on the rail (m)",
              p.s, p.half);
    }

    std::puts("\nnot on top of a train that is already there:");
    {
        std::vector<TrackPath> paths;
        paths.push_back(line(1u, 0.0f, 1000.0f));
        paths.push_back(line(2u, 500.0f, 1000.0f));

        std::deque<Consist> trains;
        trains.emplace_back(&paths, &paths[0], class93(2), 500.0f);
        // Without this the sets know their path but not the vector it lives in, and the
        // walk that reports the road under them has nothing to walk. A null network is
        // fine - it means "no turnouts", which is true of a straight line.
        trains.front().attachNetwork(&paths, nullptr);

        const Placement onTop = placeTrainNear(paths, glm::vec2(500.0f, 0.0f), class93(1));
        check(onTop.path != nullptr, "a spot beside the standing train is found");
        check(!placementClear(trains, onTop), "but it is refused: a train is standing there");

        const Placement clear = placeTrainNear(paths, glm::vec2(50.0f, 0.0f), class93(1));
        check(clear.path != nullptr, "a spot at the other end of the same line is found");
        check(placementClear(trains, clear), "and that one is clear");

        // The other road runs 500 m away in the world but the test is topological: a spot
        // on a different path is clear however close the two happen to run.
        const Placement other = placeTrainNear(paths, glm::vec2(500.0f, 500.0f), class93(1));
        check(other.pathIdx == 1, "the parallel line is a different road", other.pathIdx, 1);
        check(placementClear(trains, other), "and standing on it is clear");

        check(!placementClear(trains, Placement{}), "a refused placement is never clear");
    }

    std::puts("\nnaming where a train is:");
    {
        std::vector<Station> all;
        all.push_back({"Trofors", glm::dvec3(0.0, 0.0, 0.0), 'S', "Nordlandsbanen"});
        all.push_back({"Svenningdal", glm::dvec3(0.0, 12000.0, 0.0), 'S', "Nordlandsbanen"});
        const Station* s = nearestStation(all, glm::dvec3(0.0, 11000.0, 0.0));
        check(s != nullptr && s->name == "Svenningdal", "the nearer of two is named");
        check(nearestStation({}, glm::dvec3(0.0)) == nullptr, "and no stations is not a crash");
    }

    std::printf("\n%s\n", failures == 0 ? "all ok" : "FAILURES");
    return failures == 0 ? 0 : 1;
}
