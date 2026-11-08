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

#include "WaterAreas.h"

#include "TerrainData.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

std::vector<WaterArea> loadWaterAreas(const std::string& datasetRoot) {
    std::vector<WaterArea> out;
    std::ifstream f(datasetRoot + "/overlay/water-areas.txt");
    if (!f) return out;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream is(line);
        std::string kind, type;
        WaterArea a;
        is >> kind >> type >> a.level;
        if (kind != "water" || !is) continue;
        if (type == "sea") a.cover = 82;
        else if (type == "lake") a.cover = 81;
        else continue;
        double x = 0.0, y = 0.0;
        while (is >> x >> y) a.shore.push_back({x, y});
        if (a.shore.size() >= 2 && a.shore.front() == a.shore.back()) a.shore.pop_back();
        if (a.shore.size() < 3) continue;
        a.lo = a.hi = a.shore.front();
        for (const glm::dvec2& p : a.shore) {
            a.lo = glm::min(a.lo, p);
            a.hi = glm::max(a.hi, p);
        }
        out.push_back(std::move(a));
    }
    std::printf("[WaterAreas] %zu area(s)\n", out.size());
    return out;
}

bool insideWaterArea(const WaterArea& a, const glm::dvec2& p) {
    if (p.x < a.lo.x || p.x > a.hi.x || p.y < a.lo.y || p.y > a.hi.y) return false;
    // Even-odd crossings of a ray running east from p.
    bool in = false;
    const std::size_t n = a.shore.size();
    for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
        const glm::dvec2& u = a.shore[i];
        const glm::dvec2& v = a.shore[j];
        if ((u.y > p.y) != (v.y > p.y) &&
            p.x < u.x + (p.y - u.y) * (v.x - u.x) / (v.y - u.y))
            in = !in;
    }
    return in;
}

std::size_t applyWaterAreas(Tile& t, const std::vector<WaterArea>& areas) {
    constexpr int P = TerrainData::PIXELS;
    std::size_t set = 0;
    for (const WaterArea& a : areas) {
        if (a.hi.x < t.originX || a.lo.x > t.originX + t.extent ||
            a.hi.y < t.originY || a.lo.y > t.originY + t.extent)
            continue;
        // Only the rows and columns whose centres fall within the area's bounds.
        const auto first = [&](double v) { return std::max(0, int(std::ceil(v - 0.5))); };
        const auto last = [&](double v) { return std::min(P - 1, int(std::floor(v - 0.5))); };
        const int c0 = first((a.lo.x - t.originX) / t.resolution);
        const int c1 = last((a.hi.x - t.originX) / t.resolution);
        const int r0 = first((t.originY + t.extent - a.hi.y) / t.resolution);
        const int r1 = last((t.originY + t.extent - a.lo.y) / t.resolution);
        for (int r = r0; r <= r1; ++r)
            for (int c = c0; c <= c1; ++c) {
                const glm::dvec2 centre(t.originX + (c + 0.5) * t.resolution,
                                        t.originY + t.extent - (r + 0.5) * t.resolution);
                if (!insideWaterArea(a, centre)) continue;
                const std::size_t i = static_cast<std::size_t>(r) * P + c;
                t.heights[i] = a.level;
                if (!t.landcover.empty()) t.landcover[i] = a.cover;
                ++set;
            }
    }
    return set;
}
