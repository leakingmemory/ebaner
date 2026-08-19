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

#include <glm/glm.hpp>

#include <string>
#include <vector>

// One vertex of the 2-D text overlay: position in normalised device coords
// (-1..1, y down) and a solid colour.
struct TextVertex {
    glm::vec2 pos;
    glm::vec3 color;
};

// Append the triangles for `text` (an 8x8 bitmap font, one solid quad per lit
// glyph pixel) to `out`, starting at screen pixel (xPx, yPx) with `pxScale`
// pixels per glyph pixel, in the given colour. `fbW`/`fbH` are the framebuffer
// size (to convert to NDC). Monospace, 8*pxScale px per character cell.
void appendText(std::vector<TextVertex>& out, const std::string& text, float xPx,
                float yPx, float pxScale, const glm::vec3& color, int fbW, int fbH);

// How many characters `text` draws as. UTF-8, so this is not its byte count: anything
// laying text out (panel widths, centring) has to ask this rather than size().
std::size_t textChars(const std::string& text);

// The 8x8 glyph bitmap for `ch` (row-major, bit i of each byte = column i, LSB leftmost),
// or nullptr outside ASCII. Exposed so a sign standing in the world can carry the same
// numerals the HUD draws, rather than needing an asset of its own.
const unsigned char* fontGlyph(char ch);

// The slice of a menu's items that fits on screen, kept around the selection.
//
// The panel scales with the framebuffer - every length in it is a multiple of the same
// `fbH/240` - so how many rows fit is a constant, not something a bigger screen buys more
// of. Past that count the panel simply grew off the top and bottom of the screen, taking
// the first and last items with it and saying nothing.
struct MenuWindow {
    int first = 0;        // index of the first item drawn
    int count = 0;        // how many are drawn
    bool moreAbove = false;
    bool moreBelow = false;
};
MenuWindow menuWindow(int itemCount, int selected, int fbH);

// Append a centred modal menu: a dark panel carrying `title` and the `items` list,
// with the item at `selected` highlighted (marked and brighter). Drawn opaque over
// the scene with the same text overlay, `fbW`/`fbH` being the framebuffer size.
//
// A list too long for the screen is windowed around the selection (menuWindow) with a
// count of what is out of sight at each end, so it stays workable however long it gets.
void appendMenu(std::vector<TextVertex>& out, const std::string& title,
                const std::vector<std::string>& items, int selected, int fbW, int fbH);
