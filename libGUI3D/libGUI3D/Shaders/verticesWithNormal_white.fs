#version 330 core
out vec4 FragColor;

in vec3 Normal;
in vec3 FragPos;
in vec3 wLightPos;

uniform vec4 lightColor;
uniform float sign;

void main()
{
    // ambient
    float ambientStrength = 0.3;
    vec4 ambient = ambientStrength * lightColor;
    
    // diffuse
    vec3 norm = sign * normalize(Normal);
    vec3 lightDir = normalize(wLightPos - FragPos);
    float diff = max(dot(norm, lightDir), 0.0);
    vec4 diffuse = diff * lightColor * 0.5;
    
    // specular
    float specularStrength = 0.9;
    vec3 viewDir = normalize(-FragPos);//
    vec3 reflectDir = reflect(-lightDir, norm);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32);
    vec4 specular = specularStrength * spec * lightColor;
    
    vec4 result = (ambient + diffuse + specular);
    //FragColor = result;
    FragColor = vec4(vec3(result),1);
} 
