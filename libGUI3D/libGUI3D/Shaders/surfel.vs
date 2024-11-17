#version 330
layout(location = 0) in vec3 position;
layout(location = 1) in vec3 color;
layout(location = 2) in vec3 normal;
layout(location = 3) in float radius;


uniform mat4 model;
uniform mat4 view;

out VS_OUT {
  vec4 color;
  float radius;
  vec3 normal;
} vs_out;

void main() {
    gl_Position = view * model * vec4(position,1);
    vs_out.color = vec4(color,1);
    vs_out.radius = radius;
    mat3 normalMatrix = mat3 (transpose(inverse(view*model)));
    vs_out.normal = vec3(vec4(normalMatrix * normal,0.0));
}
