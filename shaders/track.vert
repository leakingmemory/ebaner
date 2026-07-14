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
} pc;

layout(location = 0) out vec3 vWorldPos;
layout(location = 1) out vec3 vNormal;
layout(location = 2) out vec3 vColor;
layout(location = 3) out vec2 vUv;
layout(location = 4) flat out float vTexLayer;

void main() {
    vWorldPos = inPos;
    vNormal = inNormal;
    vColor = inColor;
    vUv = inUv;
    vTexLayer = inTexLayer;
    gl_Position = pc.viewProj * vec4(inPos, 1.0);
}
