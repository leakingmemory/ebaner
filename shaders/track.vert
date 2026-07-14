#version 450

layout(location = 0) in vec3 inPos; // scene-origin-relative metres (z up)

layout(push_constant) uniform PushConstants {
    mat4 viewProj;
    vec4 sunDir;   // xyz = direction to sun; w = min elevation
    vec4 camPos;   // xyz = camera position (scene-relative); w = max elevation
} pc;

layout(location = 0) out vec3 vWorldPos;

void main() {
    vWorldPos = inPos;
    gl_Position = pc.viewProj * vec4(inPos, 1.0);
}
