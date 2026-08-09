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

#include <cmath>
#include <cstdint>

// A flat (x,y) cell hash, for "what is near this point" over the rail network.
//
// The tolerances that question is asked with are metres - two track ends touch, an
// endpoint sits on a line - while the network is now the whole dataset rather than one
// station. Every such search that scans the network end to end is quadratic, and at
// ~6k segments and ~105k points that is the difference between a pause and a hang. The
// cell is deliberately far larger than any of those tolerances, so a query only ever
// touches its own cell and the ring around it.
namespace grid {

constexpr double kCell = 32.0; // m

inline std::int64_t key(double x, double y) {
    const std::int64_t cx = static_cast<std::int64_t>(std::floor(x / kCell));
    const std::int64_t cy = static_cast<std::int64_t>(std::floor(y / kCell));
    return (cx << 32) ^ (cy & 0xFFFFFFFF);
}

// Calls fn(key) for every cell within `reach` metres of (x, y).
template <typename F>
void forCellsNear(double x, double y, double reach, F fn) {
    const std::int64_t x0 = static_cast<std::int64_t>(std::floor((x - reach) / kCell));
    const std::int64_t x1 = static_cast<std::int64_t>(std::floor((x + reach) / kCell));
    const std::int64_t y0 = static_cast<std::int64_t>(std::floor((y - reach) / kCell));
    const std::int64_t y1 = static_cast<std::int64_t>(std::floor((y + reach) / kCell));
    for (std::int64_t cx = x0; cx <= x1; ++cx)
        for (std::int64_t cy = y0; cy <= y1; ++cy)
            fn((cx << 32) ^ (cy & 0xFFFFFFFF));
}

// Calls fn(key) for every cell the segment a->b passes through. Sampled along the
// segment rather than over its bounding box, so a long diagonal registers the cells it
// actually crosses and not the whole rectangle around it. Because that sampling can
// clip a corner unseen, a *query* against an index built this way should use a reach of
// at least one cell.
template <typename F>
void forCellsAlong(double ax, double ay, double bx, double by, F fn) {
    const double len = std::hypot(bx - ax, by - ay);
    const int steps = 1 + static_cast<int>(len / (kCell * 0.5));
    std::int64_t last = key(ax, ay) + 1; // anything but the first key
    for (int i = 0; i <= steps; ++i) {
        const double f = static_cast<double>(i) / steps;
        const std::int64_t k = key(ax + (bx - ax) * f, ay + (by - ay) * f);
        if (k != last) fn(k);
        last = k;
    }
}

} // namespace grid
