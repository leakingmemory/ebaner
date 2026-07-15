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

#include "EarClip.h"

namespace {

bool pointInTri(const glm::vec2& p, const glm::vec2& a, const glm::vec2& b,
                const glm::vec2& c) {
    const float d1 = (p.x - b.x) * (a.y - b.y) - (a.x - b.x) * (p.y - b.y);
    const float d2 = (p.x - c.x) * (b.y - c.y) - (b.x - c.x) * (p.y - c.y);
    const float d3 = (p.x - a.x) * (c.y - a.y) - (c.x - a.x) * (p.y - a.y);
    const bool neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
    const bool pos = (d1 > 0) || (d2 > 0) || (d3 > 0);
    return !(neg && pos);
}

} // namespace

std::vector<int> earClipTriangulate(const std::vector<glm::vec2>& poly) {
    const int n = static_cast<int>(poly.size());
    std::vector<int> tris;
    if (n < 3) return tris;

    double area = 0.0;
    for (int i = 0, j = n - 1; i < n; j = i++)
        area += static_cast<double>(poly[j].x) * poly[i].y -
                static_cast<double>(poly[i].x) * poly[j].y;
    const bool ccw = area > 0.0;

    std::vector<int> v(n);
    for (int i = 0; i < n; ++i) v[i] = ccw ? i : (n - 1 - i); // work CCW

    int m = n, guard = 0;
    while (m > 3 && guard++ < 4 * n * n) {
        bool clipped = false;
        for (int i = 0; i < m; ++i) {
            const int i0 = v[(i + m - 1) % m], i1 = v[i], i2 = v[(i + 1) % m];
            const glm::vec2 a = poly[i0], b = poly[i1], c = poly[i2];
            const float cross =
                (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
            if (cross <= 0.0f) continue; // reflex vertex
            bool ear = true;
            for (int k = 0; k < m; ++k) {
                const int vk = v[k];
                if (vk == i0 || vk == i1 || vk == i2) continue;
                if (pointInTri(poly[vk], a, b, c)) { ear = false; break; }
            }
            if (!ear) continue;
            tris.push_back(i0);
            tris.push_back(i1);
            tris.push_back(i2);
            v.erase(v.begin() + i);
            --m;
            clipped = true;
            break;
        }
        if (!clipped) break; // degenerate polygon; give up on the remainder
    }
    if (m == 3) {
        tris.push_back(v[0]);
        tris.push_back(v[1]);
        tris.push_back(v[2]);
    }
    return tris;
}
