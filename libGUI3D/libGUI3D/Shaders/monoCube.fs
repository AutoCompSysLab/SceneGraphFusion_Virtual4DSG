#version 330 core
out vec4 FragColor;
uniform samplerCube textureCube;
in vec2 TexCoords;
void main()
{
    FragColor = texture(textureCube, vec3(TexCoords, 1));
}
