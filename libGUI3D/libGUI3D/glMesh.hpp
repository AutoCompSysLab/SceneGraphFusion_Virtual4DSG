#ifndef GLMESH_H
#define GLMESH_H

//#include <glad/glad.h> // holds all OpenGL type declarations

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "glShader.hpp"

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <map>

namespace glUtil{
    struct Vertex {
        // position
        glm::vec3 Position;
        // color
        glm::vec3 Color;
        // normal
        glm::vec3 Normal;
        // texCoords
        glm::vec2 TexCoords;
        // tangent
        glm::vec3 Tangent;
        // bitangent
        glm::vec3 Bitangent;
        Vertex(){}
        Vertex(glm::vec3 p, glm::vec3 c = glm::vec3(0.f), glm::vec3 n = glm::vec3(0.f), glm::vec2 te = glm::vec2(0.f), glm::vec3 ta = glm::vec3(0.f), glm::vec3 b = glm::vec3(0.f)){
            Position = p;
            Color = c;
            Normal = n;
            TexCoords = te;
            Tangent = ta;
            Bitangent = b;
        }
        Vertex(float p1, float p2=0, float p3=0,
                float c1=0, float c2=0, float c3=0,
                float n1=0, float n2=0, float n3=0, float te1=0, float te2=0, float ta1=0, float ta2=0, float ta3=0, float b1=0, float b2=0, float b3=0){
            Position = glm::vec3(p1,p2,p3);
            Normal = glm::vec3(n1,n2,n3);
            Color = glm::vec3(c1,c2,c3);
            TexCoords = glm::vec2(te1,te2);
            Tangent = glm::vec3(ta1,ta2,ta3);
            Bitangent = glm::vec3(b1,b2,b3);
        }
    };
    
    struct Texture {
        unsigned int id;
        int type;
        std::string name;
        std::string path;
    };
    
    class Mesh : public Model_base {
    public:
        /*  Mesh Data  */
        std::vector<Vertex> vertices;
        std::vector<unsigned int> indices;
        std::vector<Texture> textures;
        unsigned int VAO;
        
        /*  Functions  */
        // constructor
        Mesh(const std::vector<Vertex> &vertices, const std::vector<unsigned int> &indices, const std::vector<Texture> &textures)
        {
            this->vertices = vertices;
            this->indices = indices;
            this->textures = textures;
            setupMesh();
        }
        
        Mesh(const std::vector<Vertex> &vertices)
        {
            this->vertices = vertices;
            this->indices.clear();
            this->textures = textures;
            setupMesh();
        }

        void UpdateMesh(){
            Reset();
            setupMesh();
        }
        
        virtual ~Mesh()
        {
            Reset();
        }
        
        void addTexture(Texture &texture)
        {
            textures.push_back(texture);
        }

        void addTexture(std::string name, unsigned int id, int type = GL_TEXTURE_2D, unsigned int order = 0)
        {
            Texture texture;
            texture.id = id;
            texture.type = type;
            texture.name = name;
            textures.push_back(texture);
        }
        
        /// inherit the init from Model_base
        void init(){}
        
        // render the mesh
        void Draw()
        {
            for(auto& pair : texNameMap) pair.second = 1;
            // bind appropriate textures
            for(unsigned int i = 0; i < textures.size(); i++)
            {
                // retrieve texture number (the N in diffuse_textureN)
                std::string number;
                const std::string &name = textures[i].name;
                if(texNameMap.count(name) == 0) texNameMap[name] = 1;
                number = std::to_string(texNameMap[name]++);
                // now set the sampler to the correct texture unit
                const std::string texName = name + number;
                //printf("texName: %s\n", texName.c_str());

                glActiveTexture(GL_TEXTURE0 + i); // active proper texture unit before binding
                shader->set(texName.c_str(), static_cast<int>(i));
                glBindTexture(textures[i].type, textures[i].id);// and finally bind the texture
            }
            
            // draw mesh
            glBindVertexArray(VAO);
            if(indices.size())
                glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
            else
                glDrawArrays(GL_TRIANGLES, 0, vertices.size());
            glBindVertexArray(0);
            
            // always good practice to set everything back to defaults once configured.
            glActiveTexture(GL_TEXTURE0);
        }

        std::vector<std::pair<std::string, std::string>>  generateTexNames(){
            std::vector<std::pair<std::string, std::string>> nameVec;
            for(auto& pair : texNameMap) pair.second = 1;
            for(unsigned int i = 0; i < textures.size(); i++)
            {
                // retrieve texture number (the N in diffuse_textureN)
                std::string number;
                const std::string &name = textures[i].name;
                if(texNameMap.count(name) == 0) texNameMap[name] = 1;
                number = std::to_string(texNameMap[name]++);
                // now set the sampler to the correct texture unit
                const std::string texName = name + number;
                std::string GLType;
//                std::cout << textures[i].type << " " << GL_TEXTURE_CUBE_MAP << " " << GL_TEXTURE_2D << std::endl;
                switch (textures[i].type) {
                    case GL_TEXTURE_CUBE_MAP:
                        GLType = "samplerCube";
                        break;
                    case GL_TEXTURE_2D:
                        GLType = "sampler2D";
                        break;
                    default:
                        throw "[glMesh][generateTexNames]Type not support\n";
                        break;
                }
                
                nameVec.push_back(std::make_pair(texName, GLType));
            }
            return nameVec;
        }
    private:
        //
        std::map<std::string, unsigned int> texNameMap;
        
        /*  Render data  */
        unsigned int VBO, EBO;
        
        /*  Functions    */
        // initializes all the buffer objects/arrays
        void setupMesh()
        {
            // create buffers/arrays
            glGenVertexArrays(1, &VAO);
            glGenBuffers(1, &VBO);
            if(indices.size())
                glGenBuffers(1, &EBO);
            
            glBindVertexArray(VAO);
            // load data into vertex buffers
            glBindBuffer(GL_ARRAY_BUFFER, VBO);
            // A great thing about structs is that their memory layout is sequential for all its items.
            // The effect is that we can simply pass a pointer to the struct and it translates perfectly to a glm::vec3/2 array which
            // again translates to 3/2 floats which translates to a byte array.
            glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), &vertices[0], GL_STATIC_DRAW);
            
            if(indices.size()) {
                glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
                glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), &indices[0], GL_STATIC_DRAW);
            }
            
            // set the vertex attribute pointers
            // vertex Positions
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
            // vertex Colors
            glEnableVertexAttribArray(1);
            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Color));
            // vertex normals
            glEnableVertexAttribArray(2);
            glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Normal));
            // vertex texture coords
            glEnableVertexAttribArray(3);
            glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, TexCoords));
            // vertex tangent
            glEnableVertexAttribArray(4);
            glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Tangent));
            // vertex bitangent
            glEnableVertexAttribArray(5);
            glVertexAttribPointer(5, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Bitangent));

            glBindVertexArray(0);
        }

        void Reset(){
            glDeleteVertexArrays(1, &VAO);
            glDeleteBuffers(1, &VBO);
            glDeleteBuffers(1, &EBO);
        }
    };
}
#endif
