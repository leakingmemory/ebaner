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

layout(location = 0) in vec3 vWorldPos;
layout(location = 1) in vec3 vNormal;
layout(location = 2) in vec3 vColor;
layout(location = 3) in vec2 vUv;
layout(location = 4) flat in float vTexLayer;

layout(set = 0, binding = 0) uniform sampler2DArray uLand;

layout(push_constant) uniform PushConstants {
    mat4 viewProj;
    vec4 sunDir;   // xyz = direction to sun; w = min elevation
    vec4 camPos;   // xyz = camera position (scene-relative); w = max elevation
    vec4 params;   // x = scene alpha (editor ghosting; 1 = opaque)
} pc;

layout(location = 0) out vec4 outColor;

// Ballast texture-array layers (must match Textures.h landtex::Layer).
const int BALLAST = 9;       // plain crushed stone (near)
const int BALLAST_TIES = 10; // ballast + sleeper stripe (distant)

void main() {
    float dist = length(vWorldPos - pc.camPos.xyz);

    vec3 base;
    if (vTexLayer >= 0.0) {
        // Ballast top: plain near the camera (real 3-D sleepers sit on it), fading
        // to the sleeper-stripe texture at range as the 3-D sleeper boxes drop out.
        vec3 plain = texture(uLand, vec3(vUv, float(BALLAST))).rgb;
        vec3 ties = texture(uLand, vec3(vUv, float(BALLAST_TIES))).rgb;
        float f = smoothstep(160.0, 240.0, dist);
        base = mix(plain, ties, f) * vColor;
    } else {
        // Solid parts: rails, sleepers, ballast sides.
        base = vColor;
    }

    // A lit signal lamp is its own light source: no sun term and no haze, so it keeps its
    // full colour however far away and whatever the sun is doing. An entry signal's danger
    // lamp flashes: on the dark beat it reads as an unlit lens rather than vanishing, which
    // is what the lens looks like between flashes.
    if (vTexLayer < -2.5 && vTexLayer > -4.5) {
        const bool dark = vTexLayer < -3.5 && pc.params.y < 0.5;
        outColor = vec4(dark ? base * 0.16 : base, pc.params.x);
        return;
    }

    // Rock inside a tunnel bore: ambient only. With the sun term applied the sun shines
    // through the mountain and lights whichever wall happens to face it.
    if (vTexLayer < -4.5) {
        outColor = vec4(base * 0.35, pc.params.x);
        return;
    }

    // Simple sun lighting, matching the terrain's ambient/diffuse balance.
    float ndl = max(dot(normalize(vNormal), normalize(pc.sunDir.xyz)), 0.0);
    vec3 color = base * (0.35 + 0.65 * ndl);

    // Same distance haze as the terrain so track recedes consistently.
    float haze = clamp(dist / 40000.0, 0.0, 0.65);
    color = mix(color, vec3(0.70, 0.78, 0.86), haze);

    // Glazing (vehicle windows/windscreen) is tagged with a texLayer sentinel and
    // drawn translucent; everything else is opaque.
    float alpha = (vTexLayer < -1.5 && vTexLayer > -2.5) ? 0.30 : 1.0;
    outColor = vec4(color, alpha * pc.params.x);
}
