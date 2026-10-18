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

// The static scenery - track, roads, buildings, platforms, signs, bores - packed
// (PackedSolidVertex). The same outputs as track.vert, for track.frag. Nothing in it is a
// lamp, so none of track.vert's lamp sizing is here.
layout(location = 0) in vec3 inPos;         // scene-origin-relative metres (z up)
layout(location = 1) in vec4 inNormalLayer; // xyz normal; w = texture layer / 127
layout(location = 2) in vec4 inColor;
layout(location = 3) in vec2 inUv;

layout(push_constant) uniform PushConstants {
    mat4 viewProj;
    vec4 sunDir;
    vec4 camPos;
    vec4 params;
} pc;

layout(location = 0) out vec3 vWorldPos;
layout(location = 1) out vec3 vNormal;
layout(location = 2) out vec3 vColor;
layout(location = 3) out vec2 vUv;
layout(location = 4) flat out float vTexLayer;

void main() {
    vWorldPos = inPos;
    vNormal = inNormalLayer.xyz;
    vColor = inColor.rgb;
    vUv = inUv;
    vTexLayer = round(inNormalLayer.w * 127.0);
    gl_Position = pc.viewProj * vec4(inPos, 1.0);
}
