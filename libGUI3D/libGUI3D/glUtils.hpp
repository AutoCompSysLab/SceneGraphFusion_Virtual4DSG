//
//  glUtils.hpp
//  DFGGUI
//
//  Created by Shun-Cheng Wu on 01/Jan/2019.
//

#ifndef glUtils_hpp
#define glUtils_hpp

#include <stdio.h>
#include <vector>
#include <string>
#include <iostream>

//#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "glShader.hpp"
#include "glMesh.hpp"

namespace glUtil {

    template<class T>
    static Eigen::Matrix<T,4,4> lookAt(Eigen::Matrix<T,3,1> const& eye,
                                Eigen::Matrix<T,3,1> const& center,
                                Eigen::Matrix<T,3,1> const & up) {
        const Eigen::Matrix<T,3,1> f = (center - eye).normalized();
        Eigen::Matrix<T,3,1> u = up.normalized();
        const Eigen::Matrix<T,3,1> s = f.cross(u).normalized();
        u = s.cross(f);
        Eigen::Matrix<T,4,4> res;
        res <<  s.x(), s.y(), s.z(), -s.dot(eye),
                u.x(), u.y(), u.z(), -u.dot(eye),
                -f.x(), -f.y(), -f.z(), f.dot(eye),
                0, 0, 0, 1;
        return res;
    };

    template<class T>
    static Eigen::Matrix<T,4,4> Perspective(float fx,float fy, float cx, float cy, int width, int height, double n, double f) {
        assert(f > n);
        Eigen::Matrix<T,4,4> res = Eigen::Matrix<T,4,4>::Zero();
        res << 2 * fx / width, 0, -(2*(cx / width) -1), 0,
                0, 2 * fy / height, -(2*(cy / height) -1), 0,
                0, 0,  -(f+n)/(f-n), -2*f*n/(f-n),
                0, 0, -1, 0;
        return res;
    }

    [[maybe_unused]] static Eigen::Matrix4f GetViewMatrix(const Eigen::Matrix4f &pose){ // input pose: column-major
        Eigen::Matrix4f camera_pose = pose.transpose(); // column-major -> row-major
        Eigen::Vector3f camera_direction;
        Eigen::Vector3f camera_right;
        Eigen::Vector3f camera_up;
        Eigen::Vector3f camera_eye;
        Eigen::Vector3f camera_center;
        Eigen::Matrix4f view_pose;
        camera_direction = camera_pose.block<3, 3>(0, 0) * Eigen::Vector3f(0, 0, 1); 
        camera_right = camera_pose.block<3, 3>(0, 0) * Eigen::Vector3f(1, 0, 0); 
        camera_up = camera_right.cross(camera_direction); 
        camera_eye = camera_pose.block<3, 1>(0, 3);
        camera_center = camera_eye + 1 * camera_direction;
        view_pose = lookAt(camera_eye, camera_center, camera_up);
        return view_pose;
    }

    template<typename T, int m, int n>
    static inline glm::mat<m, n, float, glm::precision::highp> E2GLM(const Eigen::Matrix<T, m, n>& em)
    {
        glm::mat<m, n, float, glm::precision::highp> mat;
        for (int i = 0; i < m; ++i)
        {
            for (int j = 0; j < n; ++j)
            {
                mat[j][i] = em(i, j);
            }
        }
        return mat;
    }

    template<typename T, int m, int n>
    static inline Eigen::Matrix<T, m, n> GLM2E(const glm::mat<m, n, float, glm::precision::highp> &mat) {
        Eigen::Matrix<T, m, n> em;
        for (int i = 0; i < m; ++i)
        {
            for (int j = 0; j < n; ++j)
            {
                em(i, j) = mat[j][i];
            }
        }
        return em;
    }

    template<typename T, int m>
    static inline glm::vec<m, float, glm::precision::highp> E2GLM(const Eigen::Matrix<T, m, 1>& em)
    {
        glm::vec<m, float, glm::precision::highp> v;
        for (int i = 0; i < m; ++i)
        {
            v[i] = em(i);
        }
        return v;
    }

    class Utils {
    public:
        // faces should have 6 paths: right, left, front, back, top, bottom
        static unsigned int loadCubemap(const std::vector<std::string> &faces);
        uint loadTexture(char const * path);
        static void save_screen( const char *spath );
        
    };
    
    class ShapeVertices {
    public:
        static std::vector<Vertex> skybox;
        static std::vector<Vertex> cube;
        static std::vector<Vertex> plane;
        static std::vector<Vertex> quad;
    };
    
    
    

    class SkyBox : public Model_base {
    public:
        SkyBox(const std::vector<std::string> &faces):mesh(NULL), textureId(0)
        {
            textureId = Utils().loadCubemap(faces);
        }
        
        ~SkyBox(){
            glDeleteTextures(1, &textureId);
        }
        void init(){
            mesh = new Mesh(ShapeVertices().skybox);
            mesh->addTexture("skybox", textureId, GL_TEXTURE_CUBE_MAP);
        }
        
        void setViewProjection(const glm::mat4 &view, const glm::mat4 &projection){
            if(shader == NULL){
                shader = new Shader();
                shader->compileShader(vertexShader, fragmentShader);
                hasOwnership = true;
            }
            shader->use();
            shader->set("view", view);
            shader->set("projection", projection);
        }
        
        void Draw(){
            glDepthFunc(GL_LEQUAL);  // change depth function so depth test passes when values are equal to depth buffer's content
            mesh->setShader(shader);
            mesh->Draw();
            glDepthFunc(GL_LESS); // set depth function back to default
        }
        
        unsigned int getTextureId(){ return textureId; }
    private:
        Mesh *mesh;
        unsigned int textureId;

        const char *vertexShader ="#version 330 core\n"
        "layout (location = 0) in vec3 aPos;\n"
        "out vec3 TexCoords;\n"
        "uniform mat4 projection;\n"
        "uniform mat4 view;\n"
        "void main(){\n"
        "TexCoords = aPos;\n"
        "vec4 pos = projection * view * vec4(aPos, 1.0);\n"
        "gl_Position = pos.xyww;}\n\0";
        const char *fragmentShader = "#version 330 core\n"
        "out vec4 FragColor;\n"
        "in vec3 TexCoords;\n"
        "uniform samplerCube skybox1;\n"
        "void main(){\n"
            "FragColor = texture(skybox1, TexCoords);\n"
        "}\0";

    };

}

#endif /* glUtils_hpp */
