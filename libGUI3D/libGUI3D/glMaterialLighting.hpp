//
//  materialsLighting.h
//  DFGGUI
//
//  Created by Shun-Cheng Wu on 27/Dec/2018.
//

#ifndef materialsLighting_h
#define materialsLighting_h

#include <glm/glm.hpp>
#include <map>
#include "glShader.hpp"

namespace glUtil{
    class ShaderMatrialLighting{
    public:
        enum Materials{
            emerald, jade, obsidian, pearl, ruby, turquoise, brass, bronze, chrome, copper, gold, silver,
            blackPlastic, cyanPlastic, greenPlastic, redPlastic, whitePlastic, yellowPlastic,
            blackRubber, cyanRubber, greenRubber, redRubber, whiteRubber, yellowRubber
        };
        
        ShaderMatrialLighting(){
            setDefault();
        }
        /// must manually call use() before call this
        void setTo(Shader *shader, Materials type, std::string name = "material"){
            shader->set(name + ".ambient", typeMaps[type].ambient);
            shader->set(name + ".diffuse", typeMaps[type].diffuse);
            shader->set(name + ".specular", typeMaps[type].specular);
            shader->set(name + ".shininess", typeMaps[type].shininess);
        }
        
    private:
        struct Lighting {
            glm::vec3 ambient;
            glm::vec3 diffuse;
            glm::vec3 specular;
            float shininess;
            void init(const glm::vec3 &ambient_, const glm::vec3 &diffuse_, const glm::vec3 &specular_, const float &shininess_){
                ambient = ambient_;
                diffuse = diffuse_;
                specular = specular_;
                shininess = shininess_*128;
            }
        };
        std::map<Materials, Lighting> typeMaps;
        void setDefault(){
            typeMaps[emerald].init(glm::vec3(0.0215f, 0.1745f, 0.0215f), glm::vec3(0.07568f, 0.61424f, 0.07568f), glm::vec3(0.633f, 0.727811f, 0.633f), 0.6f);
            typeMaps[jade].init(glm::vec3(0.135f, 0.2225f, 0.1575f), glm::vec3(0.54f, 0.89f, 0.63f), glm::vec3(0.316228f, 0.316228f, 0.316228f), 0.1f);
            typeMaps[obsidian].init(glm::vec3(0.05375, 0.05f, 0.06625f), glm::vec3(0.18275, 0.17f, 0.22525f), glm::vec3(0.332741f, 0.328634f, 0.346435f), 0.3f);
            typeMaps[pearl].init(glm::vec3(0.25f, 0.20725f, 0.20725f), glm::vec3(1.f, 0.829f, 0.829f), glm::vec3(0.296648f, 0.296648f, 0.296648f), 0.088f);
            typeMaps[ruby].init(glm::vec3(0.1745f, 0.01175f, 0.01175f), glm::vec3(0.61424f, 0.04136f, 0.04136f), glm::vec3(0.727811f, 0.626959f, 0.626959f), 0.6f);
            typeMaps[turquoise].init(glm::vec3(0.1f, 0.18725f, 0.1745f), glm::vec3(0.396f, 0.74151f, 0.49102f), glm::vec3(0.297254f, 0.30829f, 0.306678f), 0.1f);
            typeMaps[brass].init(glm::vec3(0.329412, 0.223529, 0.027451), glm::vec3(0.780392, 0.568627, 0.113725), glm::vec3(0.992156, 0.941176, 0.807843),  0.21794872);
            typeMaps[bronze].init(glm::vec3(0.0215f, 0.1275f, 0.054f), glm::vec3(0.714f, 0.4284f, 0.18144f), glm::vec3(0.393548f, 0.271906f, 0.166721f), 0.2f);
            typeMaps[chrome].init(glm::vec3(0.25f, 0.25f, 0.25f), glm::vec3(0.4f, 0.4f, 0.4f), glm::vec3(0.774597, 0.774597, 0.774597), 0.6f);
            typeMaps[copper].init(glm::vec3(0.19125, 0.0735f, 0.0225), glm::vec3(0.7038, 0.27048, 0.0828), glm::vec3(0.256777, 0.137622, 0.086014), 0.1f);
            typeMaps[gold].init(glm::vec3(0.24725, 0.1995f, 0.0745f), glm::vec3(0.75164f, 0.60648f, 0.22648f), glm::vec3(0.628281f, 0.555802f, 0.366065f), 0.4f);
            typeMaps[silver].init(glm::vec3(0.19225f, 0.19225f, 0.19225f), glm::vec3(0.50754f, 0.50754f, 0.50754f), glm::vec3(0.508273f, 0.508273f, 0.508273f), 0.4f);
            typeMaps[blackPlastic].init(glm::vec3(0.f), glm::vec3(0.01f), glm::vec3(0.5), 0.25f);
            typeMaps[cyanPlastic].init(glm::vec3(0.f, 0.1f, 0.06f), glm::vec3(0.f, 0.50980392f, 0.50980392f), glm::vec3(0.50196078f), 0.25f);
            typeMaps[greenPlastic].init(glm::vec3(0.f), glm::vec3(0.1f, 0.35f, 0.1f), glm::vec3(0.45f, 0.55f, 0.45f), 0.25f);
            typeMaps[redPlastic].init(glm::vec3(0.f), glm::vec3(0.5f, 0.f, 0.f), glm::vec3(0.7f, 0.6f, 0.6f), 0.25f);
            typeMaps[whitePlastic].init(glm::vec3(0.f), glm::vec3(0.55f), glm::vec3(0.7f), 0.25f);
            typeMaps[yellowPlastic].init(glm::vec3(0.f), glm::vec3(0.5f, 0.5f, 0.f), glm::vec3(0.6f, 0.6f, 0.5f), 0.25f);
            typeMaps[blackPlastic].init(glm::vec3(0.02f), glm::vec3(0.01f), glm::vec3(0.4f), 0.078125f);
            typeMaps[cyanRubber].init(glm::vec3(0.f, 0.05f, 0.05f), glm::vec3(0.4f, 0.5f, 0.5f), glm::vec3(0.04f, 0.7f, 0.7f), 0.078125f);
            typeMaps[greenRubber].init(glm::vec3(0.f, 0.05f, 0.0f), glm::vec3(0.4f, 0.5f, 0.4f), glm::vec3(0.04f, 0.7f, 0.04f), 0.078125f);
            typeMaps[redRubber].init(glm::vec3(0.05f, 0.f, 0.f), glm::vec3(0.5f, 0.4f, 0.4f), glm::vec3(0.7f, 0.04f, 0.04f), 0.078125f);
            typeMaps[whiteRubber].init(glm::vec3(0.05f), glm::vec3(0.5f), glm::vec3(0.7f), 0.078125);
            typeMaps[yellowRubber].init(glm::vec3(0.05f, 0.05f, 0.f), glm::vec3(0.5f, 0.5f, 0.4f), glm::vec3(0.7f, 0.7f, 0.04f), 0.078125f);
        }
    };
}

#endif /* materialsLighting_h */
