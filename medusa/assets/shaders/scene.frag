#version 150

in vec3 vNormal;
uniform bool uWireframe;
uniform float uAlpha;
out vec4 FragColor;

void main() {
    vec3 base = vec3(0.20, 0.55, 0.90);
    if (uWireframe) {
        FragColor = vec4(base, uAlpha);
    } else {
        vec3 N = normalize(vNormal);
        vec3 L = normalize(vec3(0.5, 1.0, 0.3));
        float diff = max(dot(N, L), 0.0);
        vec3 color = base * (0.25 + 0.75 * diff);
        FragColor = vec4(color, uAlpha);
    }
}
