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

#version 450

layout(location = 0) in vec3 inPos;      // scene-origin-relative metres (z up = elevation)
layout(location = 1) in vec3 inNormal;
layout(location = 2) in float inElevation; // metres above sea level
layout(location = 3) in float inLandcover; // AR50 artype code (0 = none)

layout(push_constant) uniform PushConstants {
    mat4 viewProj;
    vec4 sunDir;   // xyz = direction TO sun (normalised); w = min elevation
    vec4 camPos;   // xyz = camera position (scene-relative);  w = max elevation
} pc;

layout(location = 0) out vec3 vNormal;
layout(location = 1) out float vElevation;
layout(location = 2) out vec3 vWorldPos;
layout(location = 3) flat out int vLandcover;

void main() {
    vNormal = inNormal;
    vElevation = inElevation;
    vWorldPos = inPos;
    vLandcover = int(inLandcover + 0.5);
    gl_Position = pc.viewProj * vec4(inPos, 1.0);
}
