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

#pragma once

#include "TerrainMesh.h" // Vertex
#include "TrackMesh.h"   // TrackVertex

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <glm/glm.hpp>
#include <glm/gtc/packing.hpp>

// The scenery as the graphics card holds it. The meshes are built in full floats, which is
// what the builders want to compute in, but most of a float is wasted on what it carries:
// a normal needs a degree or so, a colour 8 bits a channel. On the card it is held packed,
// at half the size or less, and every frame reads less of it. Packed on its way up, by the
// renderer, so nothing that builds a mesh knows about it.

// Track, roads, buildings, platforms, signs and bores: 24 bytes against 48.
//   normal  8 bits a component, and the texture layer in the fourth (an integer, so it is
//           exact: the shader reads it back as w * 127)
//   colour  8 bits a channel
//   uv      half floats. Only the ballast top carries one, and its along-track coordinate
//           is kept small by the builder for that reason (see TrackMesh).
// Lamps and glazing are not in it: they live in the meshes that are rebuilt as they
// change, which stay in TrackVertex.
struct PackedSolidVertex {
    glm::vec3 pos;
    std::int8_t normalLayer[4];
    std::uint8_t color[4];
    std::uint16_t uv[2];
};
static_assert(sizeof(PackedSolidVertex) == 24, "PackedSolidVertex is laid out for the GPU");

// The ground: 16 bytes against 32. The normal is octahedral, two 12-bit numbers, which
// is finer than a tenth of a degree, because the hillshading is the whole look of the
// ground and banding on a long slope would show. The top 8 bits are the AR50 land cover.
// The elevation is not stored: it is the height above the scene origin plus the origin's.
struct PackedGroundVertex {
    glm::vec3 pos;
    std::uint32_t normalCover;
};
static_assert(sizeof(PackedGroundVertex) == 16, "PackedGroundVertex is laid out for the GPU");

namespace packing {

inline std::int8_t snorm8(float x) {
    return static_cast<std::int8_t>(std::lround(std::clamp(x, -1.0f, 1.0f) * 127.0f));
}
inline std::uint8_t unorm8(float x) {
    return static_cast<std::uint8_t>(std::lround(std::clamp(x, 0.0f, 1.0f) * 255.0f));
}

// A unit vector folded onto the octahedron and unfolded into the square [-1, 1]^2.
inline glm::vec2 octEncode(glm::vec3 n) {
    n /= (std::fabs(n.x) + std::fabs(n.y) + std::fabs(n.z));
    glm::vec2 p(n.x, n.y);
    if (n.z < 0.0f) {
        const float sx = n.x >= 0.0f ? 1.0f : -1.0f;
        const float sy = n.y >= 0.0f ? 1.0f : -1.0f;
        p = glm::vec2((1.0f - std::fabs(n.y)) * sx, (1.0f - std::fabs(n.x)) * sy);
    }
    return p;
}

// What the ground shader does with it (terrain.vert), for checking the packing.
inline glm::vec3 octDecode(glm::vec2 f) {
    glm::vec3 n(f.x, f.y, 1.0f - std::fabs(f.x) - std::fabs(f.y));
    const float t = std::max(-n.z, 0.0f);
    n.x += n.x >= 0.0f ? -t : t;
    n.y += n.y >= 0.0f ? -t : t;
    return glm::normalize(n);
}

constexpr std::uint32_t kOctMax = 4095; // 12 bits a component

inline std::uint32_t octQuantise(float x) {
    return static_cast<std::uint32_t>(
        std::lround((std::clamp(x, -1.0f, 1.0f) * 0.5f + 0.5f) * kOctMax));
}
inline float octDequantise(std::uint32_t q) {
    return static_cast<float>(q) / kOctMax * 2.0f - 1.0f;
}

} // namespace packing

inline PackedSolidVertex packSolid(const TrackVertex& v) {
    using namespace packing;
    PackedSolidVertex p;
    p.pos = v.pos;
    p.normalLayer[0] = snorm8(v.normal.x);
    p.normalLayer[1] = snorm8(v.normal.y);
    p.normalLayer[2] = snorm8(v.normal.z);
    p.normalLayer[3] = static_cast<std::int8_t>(std::clamp(std::lround(v.texLayer), -127L, 127L));
    p.color[0] = unorm8(v.color.r);
    p.color[1] = unorm8(v.color.g);
    p.color[2] = unorm8(v.color.b);
    p.color[3] = 255;
    p.uv[0] = static_cast<std::uint16_t>(glm::packHalf1x16(v.uv.x));
    p.uv[1] = static_cast<std::uint16_t>(glm::packHalf1x16(v.uv.y));
    return p;
}

inline PackedGroundVertex packGround(const Vertex& v) {
    using namespace packing;
    const glm::vec2 o = octEncode(v.normal);
    const auto cover = static_cast<std::uint32_t>(std::clamp(std::lround(v.landcover), 0L, 255L));
    return {v.pos, octQuantise(o.x) | (octQuantise(o.y) << 12) | (cover << 24)};
}

inline glm::vec3 unpackGroundNormal(std::uint32_t w) {
    using namespace packing;
    return octDecode(glm::vec2(octDequantise(w & kOctMax), octDequantise((w >> 12) & kOctMax)));
}
