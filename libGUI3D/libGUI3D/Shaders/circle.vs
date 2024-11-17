#version 330 core
layout (location = 0) in vec4 aPos;
layout (location = 1) in vec4 aColor;

out vec3 FragPos;
out vec3 LightPos;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform vec3 lightPos;


out VS_OUT {
    	vec4 color;
} vs_out;

void main()
{
    gl_Position = projection * view * model * aPos;
    FragPos = vec3(view * model * aPos);
    LightPos = vec3(view * vec4(lightPos, 1));   
    vs_out.color = (aColor);
}
