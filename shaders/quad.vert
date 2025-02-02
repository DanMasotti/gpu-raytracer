#version 410 core
layout(location = 0) in vec2 in_position;
layout(location = 4) in vec2 in_texCoord;

out vec2 texCoord;

out vec4 position;

void main() {
    texCoord = in_texCoord;
    gl_Position = vec4(in_position, 0.0, 1.0);
    position = gl_Position;
}
