#version 330
in vec4 fColor;
in vec2 v_coord;
out vec4 color;

void main() {
    float sq_norm = dot(v_coord, v_coord);
    if (sq_norm > 1.f) discard;
    color = fColor;
}
