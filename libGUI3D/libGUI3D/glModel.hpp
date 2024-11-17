#ifndef GLMODEL_H
#define GLMODEL_H

//#include <glad/glad.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <stb_image.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include "glMesh.hpp"
#include "glShader.hpp"

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
//#include <map>
#include <vector>
#include <set>

#include <Eigen/Core>

namespace glUtil{
    
    
    class Model : private Model_base
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        using Model_base::setShader;
        
        /*  Model Data */
        struct Boundaries{
            float mX=NAN, pX=NAN, mY=NAN, pY=NAN, mZ=NAN, pZ=NAN;
            float xRange()const{return pX-mX;}
            float yRange()const{return pY-mY;}
            float zRange()const{return pZ-mZ;}
        };
        std::vector<Texture> textures_loaded;    // stores all the textures loaded so far, optimization to make sure textures aren't loaded more than once.
        std::vector<Mesh*> meshes;
        std::string directory, modelName;
        Eigen::Matrix4f model_pose;
        Boundaries boundaries;
        bool gammaCorrection;
        
        /*  Functions   */
        // constructor, expects a filepath to a 3D model.
        Model(std::string const &path, bool gamma = false) : gammaCorrection(gamma),
        hasLights(false), hasMeshes(false), hasCameras(false), hasTextures(false),
        hasMaterials(false), hasAnimations(false)
        {
            vCorrName.clear();
            vCorrName.push_back(corrNameMap(aiTextureType_DIFFUSE   , "texture_diffuse"));
            vCorrName.push_back(corrNameMap(aiTextureType_SPECULAR  , "texture_specular"));
            vCorrName.push_back(corrNameMap(aiTextureType_AMBIENT   , "texture_ambient"));
            vCorrName.push_back(corrNameMap(aiTextureType_EMISSIVE  , "texture_emissive"));
            vCorrName.push_back(corrNameMap(aiTextureType_HEIGHT    , "texture_height"));
            vCorrName.push_back(corrNameMap(aiTextureType_NORMALS   , "texture_normals"));
            vCorrName.push_back(corrNameMap(aiTextureType_SHININESS , "texture_shininess"));
            vCorrName.push_back(corrNameMap(aiTextureType_OPACITY   , "texture_opacity"));
            vCorrName.push_back(corrNameMap(aiTextureType_DISPLACEMENT, "texture_displacement"));
            vCorrName.push_back(corrNameMap(aiTextureType_LIGHTMAP  , "texture_lightmap"));
            vCorrName.push_back(corrNameMap(aiTextureType_REFLECTION, "texture_reflection"));
            //vCorrName.push_back(corrNameMap(aiTextureType_UNKNOWN, "texture_unknown"));
            loadModel(path);
        }
        ~Model(){
            for(auto& pMesh : meshes)
                delete pMesh;
        }
        Model& operator = (const Model & model){
            if (this != &model) {
                if(this->meshes.size())
                    for(auto& pMesh : this->meshes)
                        delete pMesh;
                if(this->shader)
                    delete this->shader;
                this->shader = model.shader;
                this->textures_loaded = model.textures_loaded;
                this->meshes = model.meshes;
                this->directory = model.directory;
                this->gammaCorrection = model.gammaCorrection;
                this->vCorrName = model.vCorrName;
            }
            return *this;
        }
        
        
        /// No need to init here. Just to inehrit the virtual class in Model_base
        void init(){};
       
        /// Draw the model using auto-generated shader
        void Draw()
        {
            // Check file exist
            if(shader == NULL){
                std::string path = modelName;
                std::ifstream f(path);
                if(!f.is_open()) outputShaderTemplate(path);
                std::string pathvs=path+".vs", pathfs=path+".fs";
                shader = new Shader(pathvs.c_str(), pathfs.c_str());
                hasOwnership = true;
            }
            for(unsigned int i = 0; i < meshes.size(); i++){
                meshes[i]->setShader(shader);
                meshes[i]->Draw();
            }
        }
        
        /**
         Manually change/add the texture to this model. The texture must be loaded first.

         @param name The name of this texture in the target shader
         @param textureId the loaded texture id
         @param type The type of this texture. e.g. GL_TEXTURE_2D
         @param order Where to put this texture. If order <1 && <= texture vector size, put to the back of the vector.
         */
        void addTexture(std::string name, unsigned int textureId, int type = GL_TEXTURE_2D, unsigned int order = 0)
        {
            for(unsigned int i = 0; i < meshes.size(); i++)
            {
                if(meshes[i]->textures.size() > order && order >0)
                {
                    auto& texture = meshes[i]->textures[order-1];
                    texture.id = textureId;
                    texture.name = name;
                    texture.type = type;
                    texture.path = "";
                } else {
                    Texture texture;
                    texture.id = textureId;
                    texture.name = name;
                    texture.type = type;
                    texture.path = "";
                    meshes[i]->textures.push_back(texture);
                }
            }
        }
        
        void outputShaderTemplate(std::string outputPath = "shaderTemplate"){
            std::fstream ss(outputPath + ".fs", std::fstream::out);
            ss << "#version 330 core\n";
            
            std::set<std::pair<std::string, std::string>> unique_names;
            for(const auto& mesh : meshes) {
                for (auto names : mesh->generateTexNames()) {
                    unique_names.insert(names);
                }
            }
            for (const auto& name:unique_names) {
                ss << "uniform " << name.second << " " << name.first << ";\n";
            }
            
            ss << "out vec4 FragColor;\n";
            ss << "in vec3 FragPose;\n";
            ss << "in vec3 Color;\n";
            ss << "in vec3 Normal;\n";
            ss << "in vec2 TexCoords;\n";
            ss << "in vec3 Tangent;\n";
            ss << "in vec3 BiTangent;\n";
            ss << "void main(){\n";
            ss << "\tvec4 result = vec4(0,0,0,0);\n";
            for (const auto& name:unique_names)
                ss << "\tresult += texture(" << name.first << ", TexCoords);\n";
            ss << "\tresult /= " << unique_names.size() << ".0;\n";
            ss << "\tresult.w = 1;\n";
            ss << "\tFragColor = result;\n";
            ss << "}\n";
            
            ss.close();
            
            
            ss.open(outputPath + ".vs", std::fstream::out);
            ss << "#version 330 core\n";
            ss << "layout (location = 0) in vec3 aPos;\n";
            ss << "layout (location = 1) in vec3 aColor;\n";
            ss << "layout (location = 2) in vec3 aNormal;\n";
            ss << "layout (location = 3) in vec2 aTexCoords;\n";
            ss << "layout (location = 4) in vec3 aTangent;\n";
            ss << "layout (location = 5) in vec3 aBiTangent;\n";
            ss << "out vec3 FragPos;\n";
            ss << "out vec3 Color;\n";
            ss << "out vec3 Normal;\n";
            ss << "out vec2 TexCoords;\n";
            ss << "out vec3 Tangent;\n";
            ss << "out vec3 BiTangent;\n";
            ss << "uniform mat4 model;\n";
            ss << "uniform mat4 view;\n";
            ss << "uniform mat4 projection;\n";
            ss << "void main(){\n";
            ss << "\tFragPos = vec3(model * vec4(aPos, 1.0));\n";
            ss << "\tColor = aColor;\n";
            ss << "\tNormal = mat3(transpose(inverse(model))) * aNormal;\n";
            ss << "\tTexCoords = aTexCoords;\n";
            ss << "\tgl_Position = projection * view * vec4(FragPos, 1.0);\n";
            ss << "}\n";
            ss.close();
            printf("Shader Template Generated!\n");
        }


        auto computeCentroid(){
            Eigen::Vector3f centroid = Eigen::Vector3f::Zero();//glm::value_ptr(model->meshes[0]->vertices[0].Position));
            size_t counter=0;
            for (glUtil::Mesh *mesh : this->meshes) {
                for(auto &vertex : mesh->vertices){
                    for(size_t i=0;i<3;++i){
                        centroid[i] += vertex.Position[i];
                    }
                    counter++;
                }
            }
            centroid /= float(counter);
            return centroid;
        }

        void translate(const Eigen::Vector3f &translation){
            for (glUtil::Mesh *mesh : this->meshes) {
                for(auto &vertex : mesh->vertices){
                    for(size_t i=0;i<3;++i)
                        vertex.Position[i] += translation[i];
                }
                mesh->UpdateMesh();
            }
        }

        void transform(const Eigen::Matrix4f &transform){
            for (glUtil::Mesh *mesh : this->meshes) {
                for(auto &vertex : mesh->vertices){
                    auto point = Eigen::Map<Eigen::Vector3f>(glm::value_ptr(vertex.Position));
                    point = transform.topLeftCorner<3,3>() * point + transform.topRightCorner<3,1>();
                }
                mesh->UpdateMesh();
            }
        }

        auto findMaximumDistance(){
            float max_distance=0.f;
            for (glUtil::Mesh *mesh : this->meshes) {
                for(auto &vertex : mesh->vertices){
                    auto pos = Eigen::Map<Eigen::Vector3f>(glm::value_ptr(vertex.Position));
                    auto norm = pos.norm();
                    if(norm > max_distance) max_distance = norm;
                }
            }
            return max_distance;
        }

        void scale(float scale){
            for (glUtil::Mesh *mesh : this->meshes) {
                for(auto &vertex : mesh->vertices){
                    auto pos = Eigen::Map<Eigen::Vector3f>(glm::value_ptr(vertex.Position));
                    pos *= scale;
                }
            }
        }

        void update(){
            for (glUtil::Mesh *mesh : this->meshes)
                mesh->UpdateMesh();
        }

        // Print INFO
        friend std::ostream& operator<<(std::ostream& os, const Model& model)
        {
            printf("Model: %s\n", model.directory.c_str());
            printf("Loaded texture: \n");
            printf("id, name, type, path\n");
            for (size_t i=0; i<model.textures_loaded.size();++i) {
                printf("%d %s %d %s\n", model.textures_loaded[i].id, model.textures_loaded[i].name.c_str(),
                       model.textures_loaded[i].type, model.textures_loaded[i].path.c_str());
            }
            printf("Boundaries: \n");
            printf("   [-,+,length]\n");
            printf("x: %f %f %f\n", model.boundaries.mX, model.boundaries.pX, model.boundaries.pX - model.boundaries.mX);
            printf("y: %f %f %f\n", model.boundaries.mY, model.boundaries.pY, model.boundaries.pY - model.boundaries.mY);
            printf("z: %f %f %f\n", model.boundaries.mZ, model.boundaries.pZ, model.boundaries.zRange());
            printf("hasLights: %d\n", model.hasLights);
            printf("hasMeshes: %d\n", model.hasMeshes);
            printf("hasCameras: %d\n", model.hasCameras);
            printf("hasTextures: %d\n", model.hasTextures);
            printf("hasMaterials: %d\n", model.hasMaterials);
            printf("hasAnimations: %d\n", model.hasAnimations);
            return os;
        }
    private:
        struct corrNameMap{
            aiTextureType type;
            std::string name;
            corrNameMap(aiTextureType type_, std::string name_){
                type = type_;
                name = name_;
            }
        };
        std::vector<corrNameMap> vCorrName;
        
        bool hasLights, hasMeshes, hasCameras, hasTextures, hasMaterials, hasAnimations; // material = textures
        
        /*  Functions   */
        // loads a model with supported ASSIMP extensions from file and stores the resulting meshes in the meshes std::vector.
        void loadModel(std::string const &path)
        {
            // read file via ASSIMP
            Assimp::Importer importer;
            auto flags = aiProcess_Triangulate
                         | aiProcess_OptimizeMeshes
                         | aiProcess_JoinIdenticalVertices
                         | aiProcess_Triangulate
                         | aiProcess_CalcTangentSpace
                         | aiProcess_FlipUVs
                         ;
//            aut flags = aiProcess_Triangulate | aiProcess_FlipUVs | aiProcess_CalcTangentSpace;
            const aiScene* scene = importer.ReadFile(path, flags);
            hasLights = scene->HasLights();
            hasMeshes = scene->HasMeshes();
            hasCameras = scene->HasCameras();
            hasTextures = scene->HasTextures();
            hasMaterials = scene->HasMaterials();
            hasAnimations = scene->HasAnimations();

            // check for errors
            if(!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) // if is Not Zero
            {
                std::cout << "ERROR::ASSIMP:: " << importer.GetErrorString() << std::endl;
                return;
            }
            // retrieve the directory path of the filepath
            directory = path.substr(0, path.find_last_of('/'));
            modelName = path.substr(directory.length()+1, path.length());
            modelName = modelName.substr(0, modelName.find_last_of('.'));
            
            // process ASSIMP's root node recursively
            processNode(scene->mRootNode, scene);
        }
        
        // processes a node in a recursive fashion. Processes each individual mesh located at the node and repeats this process on its children nodes (if any).
        void processNode(aiNode *node, const aiScene *scene)
        {
            // process each mesh located at the current node
            for(unsigned int i = 0; i < node->mNumMeshes; i++)
            {
                // the node object only contains indices to index the actual objects in the scene.
                // the scene contains all the data, node is just to keep stuff organized (like relations between nodes).
                aiMesh* aimesh = scene->mMeshes[node->mMeshes[i]];
                meshes.push_back(processMesh(aimesh, scene));
            }
            // after we've processed all of the meshes (if any) we then recursively process each of the children nodes
            for(unsigned int i = 0; i < node->mNumChildren; i++)
            {
                processNode(node->mChildren[i], scene);
            }
            
        }
        
        Mesh* processMesh(aiMesh *mesh, const aiScene *scene)
        {
            // data to fill
            std::vector<Vertex> vertices;
            std::vector<unsigned int> indices;
            std::vector<Texture> textures;
            
            // Walk through each of the mesh's vertices
            for(unsigned int i = 0; i < mesh->mNumVertices; i++)
            {
                Vertex vertex;
                glm::vec3 vector; // we declare a placeholder std::vector since assimp uses its own std::vector class that doesn't directly convert to glm's vec3 class so we transfer the data to this placeholder glm::vec3 first.
                // positions
                vector.x = mesh->mVertices[i].x;
                vector.y = mesh->mVertices[i].y;
                vector.z = mesh->mVertices[i].z;
                if(mesh->HasVertexColors(0)){
                    vertex.Color.r = mesh->mColors[0][i].r;
                    vertex.Color.g = mesh->mColors[0][i].g;
                    vertex.Color.b = mesh->mColors[0][i].b;
                }
                
                if(std::isnan(boundaries.pX)){
                    boundaries.pX = boundaries.mX = vector.x;
                    boundaries.pY = boundaries.mY = vector.y;
                    boundaries.pZ = boundaries.mZ = vector.z;
                } else {
                    if (vector.x > boundaries.pX) boundaries.pX = vector.x;
                    else if (vector.x < boundaries.mX) boundaries.mX = vector.x;
                    if (vector.y > boundaries.pY) boundaries.pY = vector.y;
                    else if (vector.y < boundaries.mY) boundaries.mY = vector.y;
                    if (vector.z > boundaries.pZ) boundaries.pZ = vector.z;
                    else if (vector.z < boundaries.mZ) boundaries.mZ = vector.z;
                }
                
                vertex.Position = vector;
                // normals
                if(mesh->mNormals != NULL) {
                    vector.x = mesh->mNormals[i].x;
                    vector.y = mesh->mNormals[i].y;
                    vector.z = mesh->mNormals[i].z;
                } else { //FIXME: all 0 is bad, but what is better?
                    vector.x = 0;
                    vector.y = 0;
                    vector.z = 0;
                }
                vertex.Normal = vector;
                // texture coordinates
                if(mesh->mTextureCoords[0]) // does the mesh contain texture coordinates?
                {
                    glm::vec2 vec;
                    // a vertex can contain up to 8 different texture coordinates. We thus make the assumption that we won't
                    // use models where a vertex can have multiple texture coordinates so we always take the first set (0).
                    vec.x = mesh->mTextureCoords[0][i].x;
                    vec.y = mesh->mTextureCoords[0][i].y;
                    vertex.TexCoords = vec;
                }
                else
                    vertex.TexCoords = glm::vec2(0.0f, 0.0f);
                // tangent
                if(mesh->mTangents != NULL) {
                    vector.x = mesh->mTangents[i].x;
                    vector.y = mesh->mTangents[i].y;
                    vector.z = mesh->mTangents[i].z;
                } else { //FIXME: all 0 is bad, but what is better?
                    vector.x = 0;
                    vector.y = 0;
                    vector.z = 0;
                }
                vertex.Tangent = vector;
                // bitangent
                if(mesh->mBitangents != NULL) {
                    vector.x = mesh->mBitangents[i].x;
                    vector.y = mesh->mBitangents[i].y;
                    vector.z = mesh->mBitangents[i].z;
                } else { //FIXME: all 0 is bad, but what is better?
                    vector.x = 0;
                    vector.y = 0;
                    vector.z = 0;
                }
                vertex.Bitangent = vector;
                vertices.push_back(vertex);
            }
            // now walk through each of the mesh's faces (a face is a mesh its triangle) and retrieve the corresponding vertex indices.
            for(unsigned int i = 0; i < mesh->mNumFaces; i++)
            {
                aiFace face = mesh->mFaces[i];
                // retrieve all indices of the face and store them in the indices vector
                for(unsigned int j = 0; j < face.mNumIndices; j++)
                    indices.push_back(face.mIndices[j]);
            }
            // process materials
            aiMaterial* material = scene->mMaterials[mesh->mMaterialIndex];
            // we assume a convention for sampler names in the shaders. Each diffuse texture should be named
            // as 'texture_diffuseN' where N is a sequential number ranging from 1 to MAX_SAMPLER_NUMBER.
            // Same applies to other texture as the following list summarizes:
            // diffuse: texture_diffuseN
            // specular: texture_specularN
            // normal: texture_normalN
            
            for(const auto& corr : vCorrName){
                std::vector<Texture> maps = loadMaterialTextures(material, corr.type, corr.name);
                textures.insert(textures.end(), maps.begin(), maps.end());
            }

            // return a mesh object created from the extracted mesh data
//            return Mesh(vertices, indices, textures);
            Mesh* pMesh = new Mesh(vertices, indices, textures);
            return pMesh;
        }

        struct Material {
            glm::vec3 Diffuse;
            glm::vec3 Specular;
            glm::vec3 Ambient;
            float Shininess;
        };
        Material loadMaterial(aiMaterial* mat) {
            Material material;
            aiColor3D color(0.f, 0.f, 0.f);
            float shininess;

            mat->Get(AI_MATKEY_COLOR_DIFFUSE, color);
            material.Diffuse = glm::vec3(color.r, color.b, color.g);

            mat->Get(AI_MATKEY_COLOR_AMBIENT, color);
            material.Ambient = glm::vec3(color.r, color.b, color.g);

            mat->Get(AI_MATKEY_COLOR_SPECULAR, color);
            material.Specular = glm::vec3(color.r, color.b, color.g);

            mat->Get(AI_MATKEY_SHININESS, shininess);
            material.Shininess = shininess;

            return material;
        }
        
        // checks all material textures of a given type and loads the textures if they're not loaded yet.
        // the required info is returned as a Texture struct.
        std::vector<Texture> loadMaterialTextures(aiMaterial *mat, aiTextureType type, std::string typeName)
        {
            std::vector<Texture> textures;
            for(unsigned int i = 0; i < mat->GetTextureCount(type); i++)
            {
                aiString str;
                mat->GetTexture(type, i, &str);
                // check if texture was loaded before and if so, continue to next iteration: skip loading a new texture
                bool skip = false;
                for(unsigned int j = 0; j < textures_loaded.size(); j++)
                {
                    if(std::strcmp(textures_loaded[j].path.data(), str.C_Str()) == 0)
                    {
                        textures.push_back(textures_loaded[j]);
                        skip = true; // a texture with the same filepath has already been loaded, continue to next one. (optimization)
                        break;
                    }
                }
                if(!skip)
                {   // if texture hasn't been loaded already, load it
                    Texture texture;
                    texture.id = TextureFromFile(str.C_Str(), this->directory);
                    texture.name = typeName;
                    texture.type = GL_TEXTURE_2D;
                    texture.path = str.C_Str();
                    textures.push_back(texture);
                    textures_loaded.push_back(texture);  // store it as texture loaded for entire model, to ensure we won't unnecesery load duplicate textures.
                }
            }
            return textures;
        }
        
        unsigned int TextureFromFile(const char *path, const std::string &directory, bool gamma = false)
        {
            std::string filename = std::string(path);
            filename = directory + '/' + filename;
            
            unsigned int textureID;
            glGenTextures(1, &textureID);
            
            int width, height, nrComponents;
            unsigned char *data = stbi_load(filename.c_str(), &width, &height, &nrComponents, 0);
            if (data)
            {
                GLenum format;
                if (nrComponents == 1)
                    format = GL_RED;
                else if (nrComponents == 3)
                    format = GL_RGB;
                else if (nrComponents == 4)
                    format = GL_RGBA;
                else
                    throw "glMODEL::TextureFromFile::format doesn't support.\n";
                
                glBindTexture(GL_TEXTURE_2D, textureID);
                glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
                glGenerateMipmap(GL_TEXTURE_2D);
                
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                
                stbi_image_free(data);
            }
            else
            {
                std::cout << "Texture failed to load at path: " << path << std::endl;
                stbi_image_free(data);
            }
            
            return textureID;
        }
    };

}
#endif
