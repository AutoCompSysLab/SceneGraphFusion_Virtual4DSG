#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoords;

out vec3 cFragPos;
out vec3 cNormal;
out vec2 TexCoords;
out vec3 cLightPosition;
out vec3 cLightDirection;
out mat4 viewPos;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform vec3 lightPosition;
uniform vec3 lightDirection;

void main()
{
    cFragPos = vec3(model * vec4(aPos, 1.0));
    cNormal = mat3(transpose(inverse(view * model))) * aNormal;
    TexCoords = aTexCoords;
    cLightPosition = vec3(view * vec4(lightPosition, 1.0));
    cLightDirection = vec3(view * vec4(lightDirection, 1.0));
    viewPos = view;
    gl_Position = projection * view * vec4(cFragPos, 1.0);
}
