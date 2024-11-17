#version 330 core
struct Material {
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
    float shininess;
};

struct SpotLight {
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

struct Light {
    vec3 position;
    float cutOff;
    float outerCutOff;
    
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
    
    float constant;
    float linear;
    float quadratic;
};

out vec4 FragColor;

in vec3 FragPos;
in vec2 TexCoords;
in vec3 Normal;
in vec3 LightPos;

uniform sampler2D texture1;
uniform sampler2D texture2;
//uniform vec3 viewPos;
uniform Material material;
uniform Light light;

float setAttenuation(float distance){
    float constant  = 1.f;
    float linear    = 0.f;
    float quadratic = 0.f;
    if (distance <= 7) {
        linear    = 0.9f;
        quadratic = 2.5f;
    } else if (distance > 7 && distance <= 13) {
        linear    = 0.7f;
        quadratic = 1.8f;
    } else if (distance > 13 && distance <= 20) {
        linear    = 0.35f;
        quadratic = 0.44f;
    } else if (distance > 20 && distance <= 32) {
        linear    = 0.22f;
        quadratic = 0.20f;
    } else if (distance > 32 && distance <= 50) {
        linear    = 0.14f;
        quadratic = 0.07f;
    } else if (distance > 50 && distance <= 65) {
        linear    = 0.09f;
        quadratic = 0.032f;
    } else if (distance > 65 && distance <= 100) {
        linear    = 0.045f;
        quadratic = 0.0075;
    } else if (distance > 100 && distance <= 160) {
        linear    = 0.027f;
        quadratic = 0.0028;
    } else if (distance > 160 && distance <= 200) {
        linear    = 0.022f;
        quadratic = 0.0019;
    } else if (distance > 200 && distance <= 325) {
        linear    = 0.014f;
        quadratic = 0.0007;
    } else if (distance > 325 && distance <= 600) {
        linear    = 0.007f;
        quadratic = 0.0002;
    } else if (distance > 600 && distance <= 3250) {
        linear    = 0.0014f;
        quadratic = 0.000007;
    } else {
        linear    = 1.f;
        quadratic = 1.f;
    }
    return 1.f - 1.0f / (constant + linear * distance + quadratic * (distance * distance));
}

void main()
{
    // ambient
    vec3 ambient = light.ambient * material.ambient;
    
    // diffuse
    vec3 norm = normalize(Normal);
    vec3 lightDir = normalize(light.position - FragPos);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = light.diffuse * (diff * material.diffuse);
    
    // specular
    vec3 viewDir = normalize(/*viewPos*/ - FragPos); // use pose-based (not world-based)
    vec3 reflectDir = reflect(-lightDir, norm);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);
    vec3 specular = light.specular * (spec * material.specular);
    
    // spotlight (soft edges)
    //float theta = dot(lightDir, -normalize(light.direction));
    float theta = dot(viewDir, normalize(vec3(0,0,1)));
    float epsilon = (light.cutOff - light.outerCutOff);
    float intensity = clamp((theta - light.outerCutOff) / epsilon, 0.0, 1.0);
    diffuse  *= intensity;
    specular *= intensity;
    
    // attenuation
    float distance    = length(-FragPos);
    float attenuation = 1.0f / (light.constant + light.linear * distance + light.quadratic * (distance * distance));
    ambient  *= attenuation;
    diffuse  *= attenuation;
    specular *= attenuation;

    FragColor = vec4(ambient + diffuse, 1.0) * texture(texture1, TexCoords)
        +  vec4(specular, 1.0) * texture(texture2, TexCoords);
}
