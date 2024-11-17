#version 330 core
out vec4 FragColor;

struct Material {
    sampler2D diffuse;
    sampler2D specular;
    float shininess;
};

struct Light {
    vec3 position;
    vec3 direction;
    float cutOff;
    float outerCutOff;
    
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
    
    float constant;
    float linear;
    float quadratic;
};

in vec3 cFragPos;
in vec3 cNormal;
in vec2 TexCoords;
in vec3 cLightPosition;
in vec3 cLightDirection;
in mat4 viewPos;

uniform Material material;
uniform Light light;

void main()
{
    // ambient
    vec3 ambient = light.ambient * texture(material.diffuse, TexCoords).rgb;
    
    // diffuse
    vec3 norm = normalize(cNormal);
    vec3 lightDir = normalize(/*cLightPosition */- cFragPos);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = light.diffuse * diff * texture(material.diffuse, TexCoords).rgb;
    
    // specular
    vec3 viewDir = normalize(/*cLightPosition */- cFragPos);
    vec3 reflectDir = reflect(-lightDir, norm);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);
    vec3 specular = light.specular * spec * texture(material.specular, TexCoords).rgb;
    
    // spotlight (soft edges)
    lightDir = normalize(/*cLightPosition */- vec3(viewPos*vec4(cFragPos,1)));
    float theta = dot(lightDir, -normalize(vec3(0,0,1)));
    theta = dot(lightDir, -normalize(light.direction));
    float epsilon = (light.cutOff - light.outerCutOff);
    float intensity = clamp((theta - light.outerCutOff) / epsilon, 0.0, 1.0);
    diffuse  *= intensity;
    specular *= intensity;
    if(theta > light.cutOff)
    FragColor = vec4(texture(material.diffuse, TexCoords).rgb * intensity, 1.0);
    else
        FragColor = vec4(0,0,0,0);
    
    // attenuation
    float distance    = length(cLightPosition - cFragPos);
    float attenuation = 1.0 / (light.constant + light.linear * distance + light.quadratic * (distance * distance));
    ambient  *= attenuation;
    diffuse   *= attenuation;
    specular *= attenuation;
    
    vec3 result = ambient + diffuse + specular;
//    FragColor = vec4(result, 1.0);
}
