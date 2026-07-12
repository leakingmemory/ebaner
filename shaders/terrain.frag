#version 450

layout(location = 0) in vec3 vNormal;
layout(location = 1) in float vElevation;
layout(location = 2) in vec3 vWorldPos;
layout(location = 3) flat in int vLandcover;

layout(push_constant) uniform PushConstants {
    mat4 viewProj;
    vec4 sunDir;   // xyz = direction TO sun (normalised); w = min elevation
    vec4 camPos;   // xyz = camera position (scene-relative);  w = max elevation
} pc;

layout(location = 0) out vec4 outColor;

// AR50 land-cover tint (matches terrainmapper's palette).
vec3 landCoverColor(int a) {
    if (a == 10) return vec3(160.0, 160.0, 160.0) / 255.0; // built-up
    if (a == 20) return vec3(240.0, 220.0, 130.0) / 255.0; // agriculture
    if (a == 30) return vec3( 34.0, 139.0,  34.0) / 255.0; // forest
    if (a == 50) return vec3(194.0, 178.0, 128.0) / 255.0; // open land
    if (a == 60) return vec3(107.0, 142.0,  35.0) / 255.0; // bog
    if (a == 70) return vec3(220.0, 240.0, 255.0) / 255.0; // glacier
    if (a == 80) return vec3( 65.0, 105.0, 225.0) / 255.0; // freshwater
    if (a == 81) return vec3( 30.0,  60.0, 150.0) / 255.0; // sea
    return vec3(0.5);
}

// Elevation ramp normalised to [0,1] (terrainmapper's stops).
vec3 elevRamp(float t) {
    const int N = 8;
    float pos[N] = float[](0.00, 0.15, 0.30, 0.45, 0.55, 0.70, 0.85, 1.00);
    vec3  col[N] = vec3[](
        vec3(  1.0,  97.0,  69.0) / 255.0,  // dark green — lowland
        vec3( 46.0, 153.0,  79.0) / 255.0,  // green
        vec3(121.0, 200.0,  87.0) / 255.0,  // light green
        vec3(233.0, 230.0, 110.0) / 255.0,  // yellow
        vec3(205.0, 163.0,  69.0) / 255.0,  // tan
        vec3(157.0, 110.0,  68.0) / 255.0,  // brown
        vec3(185.0, 176.0, 172.0) / 255.0,  // grey — alpine
        vec3(255.0, 255.0, 255.0) / 255.0); // white — peaks
    if (t <= pos[0]) return col[0];
    for (int i = 1; i < N; ++i) {
        if (t <= pos[i]) {
            float f = (t - pos[i-1]) / (pos[i] - pos[i-1]);
            return mix(col[i-1], col[i], f);
        }
    }
    return col[N-1];
}

// Base terrain colour from elevation + land cover (mirrors terrainmapper).
vec3 baseColor(float elev, int a, float mn, float mx) {
    // Water / glacier override elevation entirely.
    if (a == 80) return vec3( 65.0, 105.0, 225.0) / 255.0;
    if (a == 81) return vec3( 30.0,  60.0, 150.0) / 255.0;
    if (a == 70) return vec3(220.0, 240.0, 255.0) / 255.0;
    // Below sea level → dark teal.
    if (elev <= 0.0) return vec3(0.0, 80.0, 80.0) / 255.0;

    float t = clamp((elev - mn) / max(mx - mn, 1.0), 0.0, 1.0);
    vec3 ec = elevRamp(t);
    if (a == 0) return ec;                       // no land cover → pure ramp
    return mix(ec, landCoverColor(a), 0.4);      // 60% elevation, 40% land cover
}

void main() {
    vec3 N = normalize(vNormal);
    vec3 L = normalize(pc.sunDir.xyz);
    float diff = max(dot(N, L), 0.0);
    float ambient = 0.35;
    float shade = ambient + (1.0 - ambient) * diff;

    vec3 base = baseColor(vElevation, vLandcover, pc.sunDir.w, pc.camPos.w);
    vec3 color = base * shade;

    // Distance haze toward the horizon for depth cueing.
    float dist = length(vWorldPos - pc.camPos.xyz);
    float haze = clamp(dist / 40000.0, 0.0, 0.65);
    color = mix(color, vec3(0.70, 0.78, 0.86), haze);

    outColor = vec4(color, 1.0);
}
