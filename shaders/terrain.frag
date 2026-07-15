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

layout(location = 0) in vec3 vNormal;
layout(location = 1) in float vElevation;
layout(location = 2) in vec3 vWorldPos;
layout(location = 3) flat in int vLandcover;

layout(set = 0, binding = 0) uniform sampler2DArray uLand;

layout(push_constant) uniform PushConstants {
    mat4 viewProj;
    vec4 sunDir;   // xyz = direction TO sun (normalised); w = min elevation
    vec4 camPos;   // xyz = camera position (scene-relative);  w = max elevation
} pc;

layout(location = 0) out vec4 outColor;

// AR50 artype -> texture-array layer (must match Textures.h landtex::Layer).
int layerForArtype(int a) {
    if (a == 10) return 1; // built-up
    if (a == 20) return 2; // agriculture
    if (a == 30) return 3; // forest
    if (a == 50) return 4; // open land
    if (a == 60) return 5; // bog
    if (a == 70) return 6; // glacier
    if (a == 81) return 7; // freshwater
    if (a == 82) return 8; // ocean
    return 0;              // unclassified / not mapped
}

// Elevation ramp (used where there is no land-cover class).
vec3 elevRamp(float t) {
    const int N = 8;
    float pos[N] = float[](0.00, 0.15, 0.30, 0.45, 0.55, 0.70, 0.85, 1.00);
    vec3  col[N] = vec3[](
        vec3(  1.0,  97.0,  69.0) / 255.0,
        vec3( 46.0, 153.0,  79.0) / 255.0,
        vec3(121.0, 200.0,  87.0) / 255.0,
        vec3(233.0, 230.0, 110.0) / 255.0,
        vec3(205.0, 163.0,  69.0) / 255.0,
        vec3(157.0, 110.0,  68.0) / 255.0,
        vec3(185.0, 176.0, 172.0) / 255.0,
        vec3(255.0, 255.0, 255.0) / 255.0);
    if (t <= pos[0]) return col[0];
    for (int i = 1; i < N; ++i)
        if (t <= pos[i]) {
            float f = (t - pos[i-1]) / (pos[i] - pos[i-1]);
            return mix(col[i-1], col[i], f);
        }
    return col[N-1];
}

void main() {
    vec3 N = normalize(vNormal);
    vec3 L = normalize(pc.sunDir.xyz);
    float diff = max(dot(N, L), 0.0);
    float ambient = 0.35;
    float shade = ambient + (1.0 - ambient) * diff;

    vec3 base;
    if (vLandcover == 0) {
        // No land-cover data: colour by elevation.
        float t = clamp((vElevation - pc.sunDir.w) /
                        max(pc.camPos.w - pc.sunDir.w, 1.0), 0.0, 1.0);
        base = elevRamp(t);
    } else {
        // Textured land class; mix two scales to break up obvious tiling.
        float layer = float(layerForArtype(vLandcover));
        vec3 t1 = texture(uLand, vec3(vWorldPos.xy / 48.0, layer)).rgb;
        vec3 t2 = texture(uLand, vec3(vWorldPos.xy / 197.0, layer)).rgb;
        base = mix(t1, t2, 0.35);
    }

    vec3 color = base * shade;

    // Distance haze toward the horizon for depth cueing.
    float dist = length(vWorldPos - pc.camPos.xyz);
    float haze = clamp(dist / 40000.0, 0.0, 0.65);
    color = mix(color, vec3(0.70, 0.78, 0.86), haze);

    outColor = vec4(color, 1.0);
}
