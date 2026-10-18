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

// The one sleeper box, drawn once for every sleeper on the line. The box is in its own
// frame (x across the track, y along it, z up); each instance is where a sleeper lies and
// the track's frame there, banked with the cant.
layout(location = 0) in vec3 inPos;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec3 inColor;
layout(location = 3) in vec2 inUv;
layout(location = 4) in float inTexLayer;
layout(location = 5) in vec3 iPos;     // scene-origin-relative metres
layout(location = 6) in vec3 iRight;
layout(location = 7) in vec3 iTangent;
layout(location = 8) in vec3 iUp;

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
    const mat3 frame = mat3(iRight, iTangent, iUp);
    const vec3 pos = iPos + frame * inPos;
    vWorldPos = pos;
    vNormal = normalize(frame * inNormal);
    vColor = inColor;
    vUv = inUv;
    vTexLayer = inTexLayer;
    gl_Position = pc.viewProj * vec4(pos, 1.0);
}
