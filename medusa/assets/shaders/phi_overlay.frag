#version 150

in float vT;
out vec4 FragColor;

// Approximate Viridis colormap (cubic fit to the official LUT).
// Maps t in [0,1] to perceptually uniform color from dark blue/purple
// (Phi=0, build plate) to bright yellow (Phi=1, top of object).
vec3 viridis(float t) {
    t = clamp(t, 0.0, 1.0);
    vec3 c0 = vec3(0.2777, 0.0054, 0.3340);
    vec3 c1 = vec3(0.1058, 1.4046, 1.3845);
    vec3 c2 = vec3(-0.3308, 0.2148, 0.0950);
    vec3 c3 = vec3(-4.6342, -5.7991, -19.3324);
    vec3 c4 = vec3(6.2280, 14.1799, 56.6905);
    vec3 c5 = vec3(4.7763, -13.7451, -65.3530);
    vec3 c6 = vec3(-5.4354, 4.6458, 26.3124);
    return c0 + t*(c1 + t*(c2 + t*(c3 + t*(c4 + t*(c5 + t*c6)))));
}

void main() {
    FragColor = vec4(viridis(vT), 1.0);
}
