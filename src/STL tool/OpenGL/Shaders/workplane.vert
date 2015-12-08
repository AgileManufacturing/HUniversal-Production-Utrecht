#version 330
uniform mat4 proj;
uniform mat4 view;
in vec3 position;
in vec3 color;
out vec3 fragmentColor;
void main(){
    gl_Position = proj * view * vec4(position,1.0);
    fragmentColor = color;
}
