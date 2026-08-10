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

layout(location = 0) in vec3 inPos;      // scene-origin-relative metres (z up)
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec3 inColor;
layout(location = 3) in vec2 inUv;
layout(location = 4) in float inTexLayer;

layout(push_constant) uniform PushConstants {
    mat4 viewProj;
    vec4 sunDir;   // xyz = direction to sun; w = min elevation
    vec4 camPos;   // xyz = camera position (scene-relative); w = max elevation
    vec4 params;   // x = scene alpha; y = blink clock (seconds, wrapped)
} pc;

layout(location = 0) out vec3 vWorldPos;
layout(location = 1) out vec3 vNormal;
layout(location = 2) out vec3 vColor;
layout(location = 3) out vec2 vUv;
layout(location = 4) flat out float vTexLayer;

// A lit signal lamp is tagged with a texLayer sentinel. It is emissive, so its normal
// slot is free and instead carries the lamp's own centre - which lets the lens be blown
// up about that centre so it never falls below a minimum apparent size. A real signal
// lamp is still picked out at a range where its head has long since become too small to
// make out; a scale-accurate 0.13 m lens would be a fraction of a pixel there.
const float kLampMinAngle = 0.0024; // minimum apparent radius, radians

void main() {
    vec3 pos = inPos;
    // A flashing lamp keeps its apparent size only while it is lit. Blown up in its dark
    // phase it would paint a fat dark disc where the lens should have shrunk away with the
    // head - the very thing an unlit lens avoids by not being tagged at all.
    // Bounded, not open-ended: the band below the lamps is tunnel rock, whose normal is a
    // normal and not a lamp centre to be blown up about.
    const bool isLamp = inTexLayer < -2.5 && inTexLayer > -4.5;
    // A blinking lamp carries its period in uv.x and its phase in uv.y, so each one runs
    // on its own clock; a steady lamp (-3) is lit always.
    const bool lampLit =
        inTexLayer > -3.5 ||
        (inUv.x > 0.0 && fract((pc.params.y + inUv.y) / inUv.x) < 0.5);
    if (isLamp && lampLit) {
        const vec3 c = inNormal; // lamp centre
        vec3 off = inPos - c;
        const float r = length(off);
        const float minR = distance(c, pc.camPos.xyz) * kLampMinAngle;
        if (r > 1e-6 && r < minR) pos = c + off * (minR / r);
    }
    vWorldPos = pos;
    vNormal = inNormal;
    vColor = inColor;
    vUv = inUv;
    vTexLayer = inTexLayer;
    gl_Position = pc.viewProj * vec4(pos, 1.0);
}
