#version 330 core
layout (points) in;
layout (triangle_strip, max_vertices = 4) out;

in VS_OUT {
  vec4 color;
  float radius;
  vec3 normal;
} gs_in[];

out vec4 fColor;
out vec2 v_coord;

uniform float diffuse_strength;
uniform mat4 projection;

mat3 rotateAlign( vec3 u1, vec3 u2)
{
    vec3 axis = cross( u1, u2 );

    float cosA = dot( u1, u2 );
    float k = 1.0f / (1.0f + cosA);

    mat3 result = mat3( (axis.x * axis.x * k) + cosA,
                 (axis.y * axis.x * k) - axis.z, 
                 (axis.z * axis.x * k) + axis.y,
                 (axis.x * axis.y * k) + axis.z,  
                 (axis.y * axis.y * k) + cosA,      
                 (axis.z * axis.y * k) - axis.x,
                 (axis.x * axis.z * k) - axis.y,  
                 (axis.y * axis.z * k) + axis.x,  
                 (axis.z * axis.z * k) + cosA 
                 );

    return result;
}

void build_house(vec4 position)
{    
    fColor = gs_in[0].color; // gs_in[0] since there's only one input vertex
    float radius = gs_in[0].radius*1;
    vec4 normal = vec4(gs_in[0].normal,0.0);
    vec3 up_axis = vec3(0.0, 0.0, 1);
    mat3 rotmat = rotateAlign(vec3(normal), up_axis);

    
    float diffuse = abs(dot(normal.xyz, up_axis));
    fColor = fColor*(1-diffuse_strength) + fColor*diffuse_strength*diffuse;
    fColor.w=1;
    
    

    vec3 offset = vec3(-1.0,-1.0,0) * radius;
    gl_Position = projection * ( position + vec4(rotmat*offset,0) );
    v_coord = vec2(-1.0,-1.0);
    EmitVertex();   

    offset = vec3(1.0,-1.0,0) * radius;
    gl_Position = projection * ( position + vec4(rotmat*offset,0) );
    v_coord = vec2(1.0,-1.0);
    EmitVertex();

    offset = vec3(-1.0,1.0,0) * radius;
    gl_Position = projection * ( position + vec4(rotmat*offset,0) );
    v_coord = vec2(-1.0,1.0);
    EmitVertex();

    offset = vec3(1.0,1.0,0) * radius;
    gl_Position = projection * ( position + vec4(rotmat*offset,0) );
    v_coord = vec2(1.0,1.0);
    EmitVertex();
    EndPrimitive();
}

void main() {    
    build_house(gl_in[0].gl_Position);
//	build_cam(gl_in[0].gl_Position);
}
