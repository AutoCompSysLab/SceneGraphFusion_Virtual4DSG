#version 330 core
out vec4 FragColor;
in vec3 FragPos;

uniform vec4 color;
uniform float thickness;

void main()
{
    if (abs(FragPos.y) < thickness && abs(FragPos.z) < thickness) {
      FragColor = vec4(1,0,0,1); // x-axis
    } else if (abs(FragPos.x) < thickness && abs(FragPos.z) < thickness) {
      FragColor = vec4(0,1,0,1); // y-axis
    } else if (abs(FragPos.y) < thickness && abs(FragPos.x) < thickness) {
      FragColor = vec4(0,0,1,1); // z -axis
    } else
    if ((abs(FragPos.x - round(FragPos.x)) < thickness && abs(FragPos.y - round(FragPos.y)) < thickness) ||
        (abs(FragPos.x - round(FragPos.x)) < thickness && abs(FragPos.z - round(FragPos.z)) < thickness) ||
        (abs(FragPos.z - round(FragPos.z)) < thickness && abs(FragPos.y - round(FragPos.y)) < thickness))
        {
        FragColor = color;
    } else {
        FragColor = vec4(0.f,0.f,0.f,0.f);
    }
} 
