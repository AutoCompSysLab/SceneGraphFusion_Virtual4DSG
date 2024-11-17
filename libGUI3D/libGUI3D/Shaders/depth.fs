// 'colorImage' is a sampler2D with the depth image
// read from the current depth buffer bound to it.
//
#version 330 core
out vec4 FragColor;

in vec2 TexCoords;

uniform sampler2D texture1;



float LinearizeDepth()
{
    float zNear = 0.4;    // TODO: Replace by the zNear of your perspective projection
    float zFar  = 5.0; // TODO: Replace by the zFar  of your perspective projection
    float depth = texture(texture1, TexCoords).x;
    return (2.0 * zNear) / (zFar + zNear - depth * (zFar - zNear));
}
//float LinearizeDepth()
//{
//    float zNear = 0.4;    // TODO: Replace by the zNear of your perspective projection
//    float zFar  = 5.0; // TODO: Replace by the zFar  of your perspective projection
//    float depthSample = texture(texture1, TexCoords).x;
//    float zLinear = 2.0 * zNear * zFar / (zFar + zNear - (depthSample*2-1) * (zFar - zNear));
//    return zLinear;
//}

void main()
{
    float c = LinearizeDepth();
    FragColor = vec4(c, c, c, 1.0);
}
