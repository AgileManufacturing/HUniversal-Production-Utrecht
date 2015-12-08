#version 330
uniform mat4 trans;
uniform mat4 view;
uniform mat4 proj;
in vec3 position;
in vec3 normal;
out vec3 outNormal;
void main() {
    gl_Position = proj * view * (trans * vec4(position,1.0));
    outNormal = ( view * trans * vec4(normal,0)).xyz;
}
