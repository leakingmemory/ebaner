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

// Water areas: a shore drawn in the overlay puts a surface the survey could not see back
// at its level. Every cell whose centre is inside the ring becomes water at that level,
// and nothing outside it changes. Synthetic tiles and a file in a temporary directory, so
// it needs no dataset.

#include "TerrainData.h"
#include "WaterAreas.h"

#include <cstdio>
#include <filesystem>
#include <fstream>

namespace fs = std::filesystem;

namespace {

int failures = 0;

void check(bool ok, const char* what) {
    std::printf("  %-60s %s\n", what, ok ? "ok" : "FAILED");
    if (!ok) ++failures;
}

Tile flatTile(double ox, double oy, float h, std::uint8_t cover) {
    Tile t;
    t.originX = ox;
    t.originY = oy;
    t.resolution = 10.0;
    t.extent = 2560.0;
    t.heights.assign(TerrainData::PIXELS * TerrainData::PIXELS, h);
    t.landcover.assign(TerrainData::PIXELS * TerrainData::PIXELS, cover);
    return t;
}

std::size_t at(int row, int col) {
    return static_cast<std::size_t>(row) * TerrainData::PIXELS + col;
}

} // namespace

int main() {
    const fs::path root = fs::temp_directory_path() / "ebaner-water-area-test";
    fs::create_directories(root / "overlay");
    {
        std::ofstream f(root / "overlay" / "water-areas.txt");
        f << "# comment\n"
             "water sea 0.25 1000 1000 1100 1000 1100 1050 1000 1050 1000 1000\n"
             "water lake 312.5 0 0 50 0 25 40\n"
             "water pond 1 0 0 1 0 1 1\n"   // no such kind
             "water sea 1 0 0 1 0\n";       // too few points
    }
    const std::vector<WaterArea> areas = loadWaterAreas(root.string());
    fs::remove_all(root);

    check(areas.size() == 2, "reads the two well-formed areas, skips the rest");
    if (areas.size() != 2) return 1;
    check(areas[0].cover == 82 && areas[0].level == 0.25f, "sea is AR50 82 at its level");
    check(areas[0].shore.size() == 4, "a repeated closing point is dropped");
    check(areas[1].cover == 81, "lake is AR50 81");

    check(insideWaterArea(areas[1], {25.0, 10.0}), "inside a triangle");
    check(!insideWaterArea(areas[1], {45.0, 30.0}), "outside it, within its bounds");

    // A tile whose SW corner is at (0, 0): column c has its centre at 10c + 5, row r at
    // 2560 - 10r - 5.
    Tile t = flatTile(0.0, 0.0, 6.0f, 50);
    const std::size_t n = applyWaterAreas(t, {areas[0]});
    // Centres from x 1005..1095 (10 columns), y 1005..1045 (5 rows).
    check(n == 50, "sets exactly the cells whose centres are inside");
    const int row1045 = (2560 - 1045 - 5) / 10, col1005 = 100;
    check(t.heights[at(row1045, col1005)] == 0.25f && t.landcover[at(row1045, col1005)] == 82,
          "a corner cell is water at the level");
    check(t.heights[at(row1045 - 1, col1005)] == 6.0f &&
              t.landcover[at(row1045 - 1, col1005)] == 50,
          "the cell north of it is untouched");
    check(t.heights[at(row1045, col1005 - 1)] == 6.0f, "the cell west of it is untouched");

    // Cells below the level are raised to it: a water surface is flat.
    Tile low = flatTile(0.0, 0.0, -1.0f, 82);
    applyWaterAreas(low, {areas[0]});
    check(low.heights[at(row1045, col1005)] == 0.25f, "a cell below the level is raised to it");

    // A tile the area does not reach is left alone.
    Tile far = flatTile(5120.0, 5120.0, 6.0f, 50);
    check(applyWaterAreas(far, areas) == 0, "a tile outside every area is untouched");

    std::printf("%s\n", failures ? "FAILED" : "all passed");
    return failures ? 1 : 0;
}
