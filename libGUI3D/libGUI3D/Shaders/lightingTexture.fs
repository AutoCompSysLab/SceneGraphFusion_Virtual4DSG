#version 330 core
out vec4 FragColor;

in vec2 TexCoords;

uniform sampler2D texture1;
uniform vec4 lightColor;

void main()
{
    FragColor = lightColor * texture(texture1, TexCoords);
}
