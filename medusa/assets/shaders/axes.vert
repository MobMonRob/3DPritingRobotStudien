#version 150

in vec3 aPos;
in vec3 aColor;
uniform mat4 uMVP;
out vec3 vColor;

void main() {
    vColor = aColor;
    gl_Position = uMVP * vec4(aPos, 1.0);
}
