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

// Editor overlay: raw rail geo-points (POINT_LIST) and links (LINE_LIST). Shared
// by both overlay pipelines; the line pipeline ignores gl_PointSize.

layout(location = 0) in vec3 inPos;   // scene-origin-relative metres (z up)
layout(location = 1) in vec3 inColor;

layout(push_constant) uniform PushConstants {
    mat4 viewProj;
    vec4 sunDir;   // xyz = direction to sun; w = min elevation
    vec4 camPos;   // xyz = camera position (scene-relative); w = max elevation
} pc;

layout(location = 0) out vec3 vColor;

void main() {
    vColor = inColor;
    gl_Position = pc.viewProj * vec4(inPos, 1.0);
    // Screen size shrinks gently with distance so far dots don't dominate;
    // clamped to a readable range (large enough that near points read clearly).
    float dist = length(inPos - pc.camPos.xyz);
    gl_PointSize = clamp(1800.0 / max(dist, 1.0), 9.0, 28.0);
}
