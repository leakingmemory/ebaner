#version 450

layout(location = 0) in vec3 vNormal;
layout(location = 1) in float vElevation;
layout(location = 2) in vec3 vWorldPos;

layout(push_constant) uniform PushConstants {
    mat4 viewProj;
    vec4 sunDir;   // xyz = direction TO sun (normalised)
    vec4 camPos;   // xyz = camera position (scene-relative)
} pc;

layout(location = 0) out vec4 outColor;

// Hypsometric colour ramp: sea -> shore -> lowland green -> brown -> rock -> snow.
vec3 hypsometric(float h) {
    // Control points (elevation metres, colour).
    const int N = 6;
    float he[N]   = float[](  0.0,   5.0,  150.0,  500.0, 1000.0, 1500.0);
    vec3  hc[N]   = vec3[](
        vec3(0.15, 0.32, 0.45),   // water / sea level
        vec3(0.76, 0.72, 0.52),   // shoreline sand
        vec3(0.36, 0.52, 0.28),   // lowland green
        vec3(0.52, 0.46, 0.30),   // upland brown
        vec3(0.55, 0.53, 0.50),   // bare rock grey
        vec3(0.95, 0.95, 0.97));  // snow

    if (h <= he[0]) return hc[0];
    for (int i = 1; i < N; ++i) {
        if (h <= he[i]) {
            float t = (h - he[i-1]) / (he[i] - he[i-1]);
            return mix(hc[i-1], hc[i], t);
        }
    }
    return hc[N-1];
}

void main() {
    vec3 N = normalize(vNormal);
    // Terrain faces may wind either way; light the visible side.
    vec3 L = normalize(pc.sunDir.xyz);
    float diff = max(dot(N, L), 0.0);

    // Hillshade: ambient term + diffuse sun.
    float ambient = 0.35;
    float shade = ambient + (1.0 - ambient) * diff;

    vec3 base = hypsometric(vElevation);
    vec3 color = base * shade;

    // Subtle distance haze toward the horizon for depth cueing.
    float dist = length(vWorldPos - pc.camPos.xyz);
    float haze = clamp(dist / 40000.0, 0.0, 0.65);
    vec3 hazeColor = vec3(0.70, 0.78, 0.86);
    color = mix(color, hazeColor, haze);

    outColor = vec4(color, 1.0);
}
