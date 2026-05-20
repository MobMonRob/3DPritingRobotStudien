#version 150

in vec3 aPos;
in float aPhi;

uniform mat4 uMVP;
uniform float uPhiMin;
uniform float uPhiMax;

out float vT;   // normalised Phi in [0, 1]

void main() {
    float range = max(uPhiMax - uPhiMin, 1e-6);
    vT = clamp((aPhi - uPhiMin) / range, 0.0, 1.0);
    gl_Position = uMVP * vec4(aPos, 1.0);
}
