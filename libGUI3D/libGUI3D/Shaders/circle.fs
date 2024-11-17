#version 330 core
out vec4 FragColor;

in vec3 FragPos;
in vec3 LightPos;
in vec4 fColor;

uniform sampler2D texture1;
uniform vec4 lightColor;

void main()
{
    FragColor = (fColor);
} 
