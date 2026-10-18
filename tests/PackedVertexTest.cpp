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

// What packing the scenery for the graphics card gives up.
//
// The renderer packs every static vertex on its way up (PackedVertex.h), and the shaders
// unpack it. The only way that goes wrong without anyone seeing it straight away is
// precision - a hillside shaded in steps, a texture layer read back as its neighbour - so
// this measures it: every direction on a fine sphere through the ground's octahedral
// normal, every byte of the solid normal, the texture layers the meshes use.

#include "PackedVertex.h"

#include <cmath>
#include <cstdio>

namespace {

int failures = 0;

void expect(bool ok, const char* what) {
    if (!ok) {
        std::printf("FAIL: %s\n", what);
        ++failures;
    }
}

float angleDeg(glm::vec3 a, glm::vec3 b) {
    return glm::degrees(std::acos(std::clamp(glm::dot(glm::normalize(a), glm::normalize(b)),
                                             -1.0f, 1.0f)));
}

} // namespace

int main() {
    // The ground: an octahedral normal in 2 x 12 bits. Every direction, the poles and the
    // folds of the octahedron included, comes back within a tenth of a degree.
    float worstGround = 0.0f;
    for (int i = 0; i <= 180; ++i) {
        for (int j = 0; j < 360; ++j) {
            const float th = glm::radians(static_cast<float>(i)),
                        ph = glm::radians(static_cast<float>(j));
            const glm::vec3 n(std::sin(th) * std::cos(ph), std::sin(th) * std::sin(ph),
                              std::cos(th));
            Vertex v{};
            v.pos = glm::vec3(1.0f, 2.0f, 3.0f);
            v.normal = n;
            v.landcover = 81.0f;
            const PackedGroundVertex p = packGround(v);
            worstGround = std::max(worstGround, angleDeg(n, unpackGroundNormal(p.normalCover)));
            expect((p.normalCover >> 24) == 81u, "land cover survives the packing");
        }
    }
    std::printf("ground normal: worst %.4f deg\n", worstGround);
    expect(worstGround < 0.1f, "ground normal within 0.1 deg");

    // The solid scenery: an 8-bit normal is within a degree, which flat faces lit by one
    // sun do not show.
    float worstSolid = 0.0f;
    for (int i = 0; i <= 180; i += 3) {
        for (int j = 0; j < 360; j += 3) {
            const float th = glm::radians(static_cast<float>(i)),
                        ph = glm::radians(static_cast<float>(j));
            const glm::vec3 n(std::sin(th) * std::cos(ph), std::sin(th) * std::sin(ph),
                              std::cos(th));
            TrackVertex v{};
            v.normal = n;
            const PackedSolidVertex p = packSolid(v);
            const glm::vec3 back(p.normalLayer[0] / 127.0f, p.normalLayer[1] / 127.0f,
                                 p.normalLayer[2] / 127.0f);
            worstSolid = std::max(worstSolid, angleDeg(n, back));
        }
    }
    std::printf("solid normal: worst %.4f deg\n", worstSolid);
    expect(worstSolid < 1.0f, "solid normal within a degree");

    // The texture layer is an integer and must come back as exactly that integer: the
    // shader tells ballast (0), solid (-1) and tunnel rock (-5) apart by it. It reads the
    // byte as snorm, w / 127, and rounds w * 127.
    for (const float layer : {0.0f, -1.0f, -5.0f, 9.0f}) {
        TrackVertex v{};
        v.normal = glm::vec3(0.0f, 0.0f, 1.0f);
        v.texLayer = layer;
        const PackedSolidVertex p = packSolid(v);
        const float w = std::max(p.normalLayer[3] / 127.0f, -1.0f);
        expect(std::round(w * 127.0f) == layer, "texture layer is exact");
    }

    // Colour to 8 bits a channel; the ballast's uv as half floats, which the track keeps
    // in a few sleeper pitches.
    {
        TrackVertex v{};
        v.normal = glm::vec3(0.0f, 0.0f, 1.0f);
        v.color = glm::vec3(0.58f, 0.40f, 1.0f);
        v.uv = glm::vec2(1.0f, 5.75f);
        const PackedSolidVertex p = packSolid(v);
        expect(std::fabs(p.color[0] / 255.0f - 0.58f) <= 0.5f / 255.0f, "colour to 8 bits");
        expect(p.color[2] == 255, "full channel stays full");
        expect(glm::unpackHalf1x16(p.uv[0]) == 1.0f, "uv.x exact");
        expect(std::fabs(glm::unpackHalf1x16(p.uv[1]) - 5.75f) < 1e-3f, "uv.y to a thousandth");
    }

    if (failures) return 1;
    std::printf("PASS\n");
    return 0;
}
