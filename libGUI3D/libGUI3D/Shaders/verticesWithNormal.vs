#version 330 core
layout (location = 0) in vec4 aPos;
layout (location = 1) in vec4 aNormal;
layout (location = 2) in vec4 aColor;

out vec3 FragPos;
//out vec2 TexCoords;
out vec3 Normal;
out vec3 wLightPos;
out vec4 Color;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform vec3 lightPos;

void main()
{
    gl_Position = projection * view * model * aPos;
    FragPos = vec3(view * model * aPos);
    Normal = mat3(transpose(inverse(view * model))) * vec3(aNormal);// 消除非等比變形造成的誤差

//    if(Normal[0] * view[0][3] + Normal[1] * view[1][3] + Normal[2] *view[2][3] > 0) Normal *= -1;

    wLightPos = vec3(view * vec4(lightPos, 1));
//    TexCoords = aTexCoords;
    Color = aColor;
    
}
