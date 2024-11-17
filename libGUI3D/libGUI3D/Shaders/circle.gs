#version 330 core
layout (points) in;
layout (triangle_strip, max_vertices = 8) out;

in VS_OUT {
    	vec4 color;
} gs_in[];

out vec4 fColor;


uniform mat4 modelPose;
uniform float radius;

//const float radius = 0.01;
void build_house(vec4 position)
{    
    fColor = gs_in[0].color; // gs_in[0] since there's only one input vertex
    gl_Position = position + vec4(-radius, -radius, 0.0, 0.0); // 1:bottom-left   
    EmitVertex();   
    gl_Position = position + vec4( radius, -radius, 0.0, 0.0); // 2:bottom-right
    EmitVertex();
    gl_Position = position + vec4(-radius,  radius, 0.0, 0.0); // 3:top-left
    EmitVertex();
    gl_Position = position + vec4( radius,  radius, 0.0, 0.0); // 4:top-right
    EmitVertex();
    EndPrimitive();
}

void main() {    
    build_house(gl_in[0].gl_Position);
//	build_cam(gl_in[0].gl_Position);
}
