#version 450

layout(location = 0) in vec3 inPos;      // scene-origin-relative metres (z up = elevation)
layout(location = 1) in vec3 inNormal;
layout(location = 2) in float inElevation; // metres above sea level

layout(push_constant) uniform PushConstants {
    mat4 viewProj;
    vec4 sunDir;   // xyz = direction TO sun (normalised), w unused
    vec4 camPos;   // xyz = camera position (scene-relative), w unused
} pc;

layout(location = 0) out vec3 vNormal;
layout(location = 1) out float vElevation;
layout(location = 2) out vec3 vWorldPos;

void main() {
    vNormal = inNormal;
    vElevation = inElevation;
    vWorldPos = inPos;
    gl_Position = pc.viewProj * vec4(inPos, 1.0);
}
