#version 330 core
out vec4 FragColor;

in vec2 TexCoords;

uniform sampler2D screenTexture;

void main()
{
//    vec3 col = texture(screenTexture, TexCoords).rgb;
//    FragColor = vec4(col, 1.0);

    vec4 texColor = texture(screenTexture, TexCoords);
//     if (texColor.a < 0.5)
//       discard;
    FragColor = texColor;
    
//    float average = 0.2126 * col.r + 0.7152 * col.g + 0.0722 * col.b; // for grayscale
//    FragColor = vec4(col.r,col.r,col.r, 1.0); // for depth
}
