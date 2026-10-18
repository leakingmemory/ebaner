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

// Vehicle detail by distance: what a train leaves out as the camera draws away from it.
//
// The vehicle buffer's index list is fixed when it is attached, so a part left out has to
// keep its vertices - they collapse to one point instead - and the parts have to be met
// in the same order on every refresh. Get that wrong and the train draws as a heap of
// triangles. So this checks, for every vehicle in the table:
//   - from inside it, a refresh is bit for bit what the build made;
//   - from further away, the same number of vertices, and fewer of them standing anywhere
//     but on the dead point, step by step until at 20 km there is nothing.
//
// No dataset: a straight TrackPath of points is the whole world here.

#include "Consist.h"
#include "TrackPath.h"
#include "Vehicle.h"
#include "VehicleMesh.h"

#include <glm/gtc/matrix_transform.hpp>

#include <cstdio>
#include <cstring>
#include <deque>
#include <string>
#include <vector>

namespace {

int failures = 0;

void check(bool ok, const std::string& what) {
    if (!ok) {
        std::printf("  %-70s FAILED\n", what.c_str());
        ++failures;
    }
}

// A vertex a left-out part has been collapsed to: black, facing up, on the unit's centre.
bool isDead(const TrackVertex& v) {
    return v.color == glm::vec3(0.0f) && v.normal == glm::vec3(0.0f, 0.0f, 1.0f) &&
           v.uv == glm::vec2(0.0f);
}

std::size_t live(const std::vector<TrackVertex>& vs) {
    std::size_t n = 0;
    for (const TrackVertex& v : vs) n += isDead(v) ? 0 : 1;
    return n;
}

bool sameBits(const std::vector<TrackVertex>& a, const std::vector<TrackVertex>& b) {
    return a.size() == b.size() &&
           std::memcmp(a.data(), b.data(), a.size() * sizeof(TrackVertex)) == 0;
}

} // namespace

int main() {
    // A view that sees everything, so only the distance decides.
    const glm::mat4 all = glm::ortho(-1e5f, 1e5f, -1e5f, 1e5f, -1e5f, 1e5f);

    std::vector<glm::vec3> pts;
    for (int i = 0; i <= 120; ++i) pts.push_back({static_cast<float>(i) * 25.0f, 0.0f, 0.0f});
    std::vector<TrackPath> paths;
    paths.emplace_back(1u, 0u, pts, std::vector<std::uint16_t>(pts.size(), 100));

    for (const VehicleSpec& spec : kVehicleSpecs) {
        std::deque<Consist> trains;
        trains.emplace_back(&paths, &paths[0], spec, 2000.0f);
        const Consist& t = trains.front();
        const std::string name = spec.name;

        VehicleMesh m;
        m.build(trains);
        const std::vector<TrackVertex> built = m.vertices();

        // From inside the leading unit, which is where a driver sits.
        const glm::vec3 cab = t.unit(0).pose().pos + glm::vec3(0.0f, 0.0f, 2.5f);
        m.refresh(trains, all, cab);
        check(sameBits(m.vertices(), built), name + ": from the cab, the build exactly");

        // Square off the middle of the train, at each distance.
        glm::vec3 mid(0.0f);
        for (int i = 0; i < t.unitCount(); ++i) mid += t.unit(i).pose().pos;
        mid /= static_cast<float>(t.unitCount());
        std::size_t prev = live(built);
        std::vector<std::size_t> at;
        for (const float d : {150.0f, 1000.0f, 5000.0f, 20000.0f}) {
            m.refresh(trains, all, mid + glm::vec3(0.0f, d, 2.0f));
            check(m.vertices().size() == built.size(),
                  name + ": the same vertex count at " + std::to_string(int(d)) + " m");
            const std::size_t n = live(m.vertices());
            check(n <= prev, name + ": no more at " + std::to_string(int(d)) + " m");
            at.push_back(n);
            prev = n;
        }
        check(at[2] < live(built), name + ": the wheels go by 5 km");
        // A bare wheelset is nothing but wheels, so it is right that nothing is left of it.
        const bool bodyless =
            t.unit(0).bodySectionFrames().empty() && t.unit(0).bogieFrames().empty();
        check(at[2] > 0 || bodyless, name + ": the body is still there at 5 km");
        check(at[3] == 0, name + ": nothing at 20 km");
        // A machine with a cab desk loses its instruments first.
        if (spec.body == BodyDi4 || spec.body == BodyCD312 || spec.body == BodyType76) {
            check(at[0] < live(built), name + ": no instruments at 150 m");
            check(at[1] < at[0], name + ": no interior at 1 km");
        }
        std::printf("  %-40s %7zu vertices; at 150 m %zu, 1 km %zu, 5 km %zu, 20 km %zu\n",
                    spec.name, built.size(), at[0], at[1], at[2], at[3]);
    }

    if (failures) {
        std::printf("%d check(s) FAILED\n", failures);
        return 1;
    }
    std::printf("PASS\n");
    return 0;
}
