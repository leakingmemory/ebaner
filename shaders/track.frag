#version 450

layout(location = 0) in vec3 vWorldPos;

layout(push_constant) uniform PushConstants {
    mat4 viewProj;
    vec4 sunDir;   // xyz = direction to sun; w = min elevation
    vec4 camPos;   // xyz = camera position (scene-relative); w = max elevation
} pc;

layout(location = 0) out vec4 outColor;

void main() {
    vec3 color = vec3(0.45, 0.12, 0.12); // dark rail red (single track colour)

    // Match the terrain's distance haze so track recedes consistently.
    float dist = length(vWorldPos - pc.camPos.xyz);
    float haze = clamp(dist / 40000.0, 0.0, 0.65);
    color = mix(color, vec3(0.70, 0.78, 0.86), haze);

    outColor = vec4(color, 1.0);
}
