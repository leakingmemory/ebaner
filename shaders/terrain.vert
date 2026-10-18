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

#version 450

layout(location = 0) in vec3 inPos;      // scene-origin-relative metres (z up = elevation)
// The normal, octahedral in two 12-bit numbers, and the AR50 land cover in the top 8 bits
// (PackedGroundVertex).
layout(location = 1) in uint inNormalCover;

layout(push_constant) uniform PushConstants {
    mat4 viewProj;
    vec4 sunDir;   // xyz = direction TO sun (normalised); w = min elevation
    vec4 camPos;   // xyz = camera position (scene-relative);  w = max elevation
    vec4 params;   // z = height of the scene origin above sea level
} pc;

layout(location = 0) out vec3 vNormal;
layout(location = 1) out float vElevation;
layout(location = 2) out vec3 vWorldPos;
layout(location = 3) flat out int vLandcover;

vec3 octDecode(vec2 f) {
    vec3 n = vec3(f, 1.0 - abs(f.x) - abs(f.y));
    const float t = max(-n.z, 0.0);
    n.x += n.x >= 0.0 ? -t : t;
    n.y += n.y >= 0.0 ? -t : t;
    return normalize(n);
}

void main() {
    const vec2 oct = vec2(float(inNormalCover & 4095u), float((inNormalCover >> 12) & 4095u)) /
                     4095.0 * 2.0 - 1.0;
    vNormal = octDecode(oct);
    vElevation = inPos.z + pc.params.z; // metres above sea level
    vWorldPos = inPos;
    vLandcover = int(inNormalCover >> 24);
    gl_Position = pc.viewProj * vec4(inPos, 1.0);
}
