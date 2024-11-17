#version 330 core
layout (location = 1) in vec2 vertex; // <vec2 pos, vec2 tex>
layout (location = 0) in vec3 acolor; // <vec2 pos, vec2 tex>
out vec3 Color;
void main()
{
    gl_Position = vec4(vertex, 0.0, 1.0);
    Color = acolor;
}  
