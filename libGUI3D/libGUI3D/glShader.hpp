/*
 Usage:
 Initialise the Shader with the paths of vertex shader and fragment shader
 In the main loop of the system, call use() first and then call set (if you want
 to set uniform argument value).
*/
#ifndef SHADER_H
#define SHADER_H


//#if defined(__linux__)
//#include <GL/gl.h>
//#elif defined(__APPLE__)
//#include <OpenGL/gl.h>
//#endif

#if defined(IMGUI_IMPL_OPENGL_LOADER_GL3W)
#include <GL/gl3w.h>    // Initialize with gl3wInit()
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLEW)
#include <GL/glew.h>    // Initialize with glewInit()
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLAD)
#include <glad/glad.h>  // Initialize with gladLoadGL()
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLBINDING)
#define GLFW_INCLUDE_NONE         // GLFW including OpenGL headers causes ambiguity or multiple definition errors.
#include <glbinding/glbinding.h>  // Initialize with glbinding::initialize()
#include <glbinding/gl/gl.h>
using namespace gl;
//#else
//#include IMGUI_IMPL_OPENGL_LOADER_CUSTOM
#endif

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <typeinfo>
#ifdef COMPILE_WITH_EIGEN
#include <Eigen/Geometry>
#endif

namespace detail{
    template <typename T> struct type_traits;
    template <> struct type_traits<uint32_t> { enum { type = GL_UNSIGNED_INT, integral = 1 }; };
    template <> struct type_traits<int32_t> { enum { type = GL_INT, integral = 1 }; };
    template <> struct type_traits<uint16_t> { enum { type = GL_UNSIGNED_SHORT, integral = 1 }; };
    template <> struct type_traits<int16_t> { enum { type = GL_SHORT, integral = 1 }; };
    template <> struct type_traits<uint8_t> { enum { type = GL_UNSIGNED_BYTE, integral = 1 }; };
    template <> struct type_traits<int8_t> { enum { type = GL_BYTE, integral = 1 }; };
    template <> struct type_traits<double> { enum { type = GL_DOUBLE, integral = 0 }; };
    template <> struct type_traits<float> { enum { type = GL_FLOAT, integral = 0 }; };
//    template <> struct type_traits<half_float::half> { enum { type = GL_HALF_FLOAT, integral = 0 }; };
//    template <typename T> struct serialization_helper;
}
namespace glUtil{
    class Shader
    {
    public:
        unsigned int ID;
        // constructor generates the shader on the fly
        /// give path to load code and compile
        Shader(){}
        Shader(std::string vertexPath, std::string fragmentPath, std::string geometryPath = "")
        {
            // 1. retrieve the vertex/fragment source code from filePath
            std::string vertexCode;
            std::string fragmentCode;
            std::string geometryCode;
//            std::ifstream vShaderFile;
//            std::ifstream fShaderFile;
//            std::ifstream gShaderFile;
//            // ensure ifstream objects can throw exceptions:
//            vShaderFile.exceptions (std::ifstream::failbit | std::ifstream::badbit);
//            fShaderFile.exceptions (std::ifstream::failbit | std::ifstream::badbit);
//            gShaderFile.exceptions (std::ifstream::failbit | std::ifstream::badbit);

            // check file exist
            auto file_exist=[](const std::string &name)->bool{
                std::ifstream f(name.c_str());
                return f.good();
            };
//            bool isFile = file_exist(vertexPath) && file_exist(fragmentPath);
//            if (!geometryPath.empty()) isFile &= file_exist(geometryPath);

            auto load=[file_exist](const std::string &path)->std::string{
                if(path.empty()) return "";
                if (file_exist(path)) {
                    std::ifstream file;
                    file.exceptions (std::ifstream::failbit | std::ifstream::badbit);
                    file.open(path);
                    std::stringstream stringstream;
                    stringstream << file.rdbuf();
                    return stringstream.str();
                } else {
                    return path;
                }
            };
            vertexCode = load(vertexPath);
            fragmentCode = load(fragmentPath);
            geometryCode = load(geometryPath);

//            if(isFile){
//                try
//                {
//                    // open files
//                    vShaderFile.open(vertexPath);
//                    fShaderFile.open(fragmentPath);
//                    if(geometryPath != "")
//                        gShaderFile.open(geometryPath);
//
//                    std::stringstream vShaderStream, fShaderStream, gShaderStream;
//                    // read file's buffer contents into streams
//                    vShaderStream << vShaderFile.rdbuf();
//                    fShaderStream << fShaderFile.rdbuf();
//                    if(geometryPath != "")
//                        gShaderStream << gShaderFile.rdbuf();
//                    // close file handlers
//                    vShaderFile.close();
//                    fShaderFile.close();
//                    if(geometryPath != "")
//                        gShaderFile.close();
//                    // convert stream into string
//                    vertexCode   = vShaderStream.str();
//                    fragmentCode = fShaderStream.str();
//                    if(geometryPath != "")
//                        geometryCode = gShaderStream.str();
//                }
//                catch (const std::ifstream::failure &e)
//                {
//                    std::cout << "ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ" << std::endl;
//                }
//            } else {
//                vertexCode = vertexPath;
//                fragmentCode = fragmentPath;
//                geometryCode = geometryPath;
//            }

            const char *vShaderCode = vertexCode.c_str();
            const char *fShaderCode = fragmentCode.c_str();
            const char *gShaderCode = geometryPath == "" ? NULL: geometryCode.c_str();
            compileShader(vShaderCode, fShaderCode, gShaderCode);
        }

        
        /// directly give code to compile
        void compileShader(const std::string &vShaderCode, const std::string &fShaderCode){
            compileShader(vShaderCode.c_str(), fShaderCode.c_str(), nullptr);
        }
        
        /// directly give code to compile
        void compileShader(const char *vShaderCode, const char *fShaderCode, const char *geShaderCode){
            // 2. compile shaders
            unsigned int vertex, fragment, geometry=0;
            // vertex shader
            vertex = glCreateShader(GL_VERTEX_SHADER);
            glShaderSource(vertex, 1, &vShaderCode, NULL);
            glCompileShader(vertex);
            checkCompileErrors(vertex, "VERTEX");
            // fragment Shader
            fragment = glCreateShader(GL_FRAGMENT_SHADER);
            glShaderSource(fragment, 1, &fShaderCode, NULL);
            glCompileShader(fragment);
            checkCompileErrors(fragment, "FRAGMENT");
            // Geometry Shader
            if(geShaderCode) {
                geometry = glCreateShader(GL_GEOMETRY_SHADER);
                glShaderSource(geometry, 1, &geShaderCode, NULL);
                glCompileShader(geometry);
                checkCompileErrors(geometry, "GEOMETRY");
            }

            // shader Program
            ID = glCreateProgram();
            glAttachShader(ID, vertex);
            glAttachShader(ID, fragment);
            if(geShaderCode)
                glAttachShader(ID, geometry);
            glLinkProgram(ID);
            checkCompileErrors(ID, "PROGRAM");
            // delete the shaders as they're linked into our program now and no longer necessary
            glDeleteShader(vertex);
            glDeleteShader(fragment);
            if(geShaderCode)
                glDeleteShader(geometry);
        }
        
        // activate the shader
        // ------------------------------------------------------------------------
        void use()
        {
            glUseProgram(ID);
        }
        
        /// utility uniform functions. If it is set for texture (sampler), use setTexture or explicitly type cast to int, to prevent error.
        template <typename T>
        void set(const std::string &name, T value) const
        {
            if(typeid(T) == typeid(uint))
                glUniform1ui(glGetUniformLocation(ID, name.c_str()), value); // value: number of element if a vector type is given.
            else if(typeid(T) == typeid(int))
                glUniform1i(glGetUniformLocation(ID, name.c_str()), value);
            else if(typeid(T) == typeid(float) || typeid(T) == typeid(double))
                glUniform1f(glGetUniformLocation(ID, name.c_str()), value);
            else if(typeid(T) == typeid(bool))
                glUniform1i(glGetUniformLocation(ID, name.c_str()), value);
            else
                throw "ERROR::GLUNIFORM::Input type does not support!. \n";
        }
        
        template <typename T>
        void set(const std::string &name, T value1, T value2) const
        {
            if(typeid(T) == typeid(uint))
                glUniform2ui(glGetUniformLocation(ID, name.c_str()), value1, value2);
            else if(typeid(T) == typeid(int))
                glUniform2i(glGetUniformLocation(ID, name.c_str()), value1, value2);
            else if(typeid(T) == typeid(float) || typeid(T) == typeid(double))
                glUniform2f(glGetUniformLocation(ID, name.c_str()), value1, value2);
            else if(typeid(T) == typeid(bool))
                throw "ERROR::GLUNIFORM::BOOL type only accept one input argument!. \n";
            else
                throw "ERROR::GLUNIFORM::Input type does not support!. \n";
        }
        
        template <typename T>
        void set(const std::string &name, T value1, T value2, T value3) const
        {
            if(typeid(T) == typeid(uint))
                glUniform3ui(glGetUniformLocation(ID, name.c_str()), value1, value2, value3);
            else if(typeid(T) == typeid(int))
                glUniform3i(glGetUniformLocation(ID, name.c_str()), value1, value2, value3);
            else if(typeid(T) == typeid(float) || typeid(T) == typeid(double))
                glUniform3f(glGetUniformLocation(ID, name.c_str()), value1, value2, value3);
            else if(typeid(T) == typeid(bool))
                throw "ERROR::GLUNIFORM::BOOL type only accept one input argument!. \n";
            else
                throw "ERROR::GLUNIFORM::Input type does not support!. \n";
        }

        template <typename T>
        void set(const std::string &name, T value1, T value2, T value3, T value4) const
        {
            if(typeid(T) == typeid(uint))
                glUniform4ui(glGetUniformLocation(ID, name.c_str()), value1, value2, value3, value4);
            else if(typeid(T) == typeid(int))
                glUniform4i(glGetUniformLocation(ID, name.c_str()), value1, value2, value3, value4);
            else if(typeid(T) == typeid(float) || typeid(T) == typeid(double))
                glUniform4f(glGetUniformLocation(ID, name.c_str()), value1, value2, value3, value4);
            else if(typeid(T) == typeid(bool))
                throw "ERROR::GLUNIFORM::BOOL type only accept one input argument!. \n";
            else
                throw "ERROR::GLUNIFORM::Input type does not support!. \n";
        }

#ifdef COMPILE_WITH_EIGEN
        template <typename T>
        void set(const std::string &name, const Eigen::Matrix<T, 4, 4> &mat, bool warn = true) {
            glUniformMatrix4fv(glGetUniformLocation(ID, name.c_str()), 1, GL_FALSE, mat.template cast<float>().data());
        }

        /// Initialize a uniform parameter with a 3x3 affine transform (float)
        template <typename T>
        void set(const std::string &name, const Eigen::Transform<T, 3, 3> &affine, bool warn = true) {
            glUniformMatrix4fv(glGetUniformLocation(ID, name.c_str()), 1, GL_FALSE, affine.template cast<float>().data());
        }

        /// Initialize a uniform parameter with a 3x3 matrix (float)
        template <typename T>
        void set(const std::string &name, const Eigen::Matrix<T, 3, 3> &mat, bool warn = true) {
            glUniformMatrix3fv(glGetUniformLocation(ID, name.c_str()), 1, GL_FALSE, mat.template cast<float>().data());
        }

        /// Initialize a uniform parameter with a 2x2 affine transform (float)
        template <typename T>
        void set(const std::string &name, const Eigen::Transform<T, 2, 2> &affine, bool warn = true) {
            glUniformMatrix3fv(glGetUniformLocation(ID, name.c_str()), 1, GL_FALSE, affine.template cast<float>().data());
        }

        /// Initialize a uniform parameter with a 3D vector
        template <typename T>
        void set(const std::string &name, const Eigen::Matrix<T, 3, 1>  &v, bool warn = true) {
            if(typeid(T) == typeid(uint))
                glUniform3ui(glGetUniformLocation(ID, name.c_str()), (unsigned int) v.x(), (unsigned int) v.y(), (unsigned int) v.z());
            else if(typeid(T) == typeid(int))
                glUniform3i(glGetUniformLocation(ID, name.c_str()), (int) v.x(), (int) v.y(), (int) v.z());
            else if(typeid(T) == typeid(float) || typeid(T) == typeid(double))
                glUniform3f(glGetUniformLocation(ID, name.c_str()), (float) v.x(), (float) v.y(), (float) v.z());
            else if(typeid(T) == typeid(bool))
                throw "ERROR::GLUNIFORM::BOOL type only accept one input argument!. \n";
            else
                throw "ERROR::GLUNIFORM::Input type does not support!. \n";
        }

        /// Initialize a uniform parameter with a 4D vector
        template <typename T>
        void set(const std::string &name, const Eigen::Matrix<T, 4, 1>  &v, bool warn = true) {
            if(typeid(T) == typeid(uint))
                glUniform4ui(glGetUniformLocation(ID, name.c_str()), (unsigned int) v.x(), (unsigned int) v.y(), (unsigned int) v.z(), (unsigned int) v.w());
            else if(typeid(T) == typeid(int))
                glUniform4i(glGetUniformLocation(ID, name.c_str()), (int) v.x(), (int) v.y(), (int) v.z(), (int) v.w());
            else if(typeid(T) == typeid(float) || typeid(T) == typeid(double))
                glUniform4f(glGetUniformLocation(ID, name.c_str()), (float) v.x(), (float) v.y(), (float) v.z(), (float) v.w());
            else if(typeid(T) == typeid(bool))
                throw "ERROR::GLUNIFORM::BOOL type only accept one input argument!. \n";
            else
                throw "ERROR::GLUNIFORM::Input type does not support!. \n";
        }

//        /// Initialize a uniform parameter with a 3D vector (float)
//        template <typename T, typename std::enable_if<detail::type_traits<T>::integral == 0, int>::type = 0>
//        void set(const std::string &name, const Eigen::Matrix<T, 3, 1>  &v, bool warn = true) {
//            glUniform3f(glGetUniformLocation(ID, name.c_str()), (float) v.x(), (float) v.y(), (float) v.z());
//        }

//        /// Initialize a uniform parameter with a 4D vector (int)
//        template <typename T, typename std::enable_if<detail::type_traits<T>::integral == 1, int>::type = 0>
//        void set(const std::string &name, const Eigen::Matrix<T, 4, 1>  &v, bool warn = true) {
//            glUniform4i(glGetUniformLocation(ID, name.c_str()), (int) v.x(), (int) v.y(), (int) v.z(), (int)v.w());
//        }
//
//        /// Initialize a uniform parameter with a 4D vector (float)
//        template <typename T, typename std::enable_if<detail::type_traits<T>::integral == 0, int>::type = 0>
//        void set(const std::string &name, const Eigen::Matrix<T, 4, 1>  &v, bool warn = true) {
//            glUniform4f(glGetUniformLocation(ID, name.c_str()), (float) v.x(), (float) v.y(), (float) v.z(),(float) v.w());
//        }
#endif
        
        /// prevent error due to incorrect value type (should be int).
        void setTexture(const std::string &name, int value) const
        {
            glUniform1i(glGetUniformLocation(ID, name.c_str()), value);
        }
        
        void set(const std::string &name, glm::mat4 matrix) const
        {
            glUniformMatrix4fv(glGetUniformLocation(ID, name.c_str()), 1, GL_FALSE, glm::value_ptr(matrix));
        }
        
        void set(const std::string &name, glm::vec3 vec) const
        {
            glUniform3fv(glGetUniformLocation(ID, name.c_str()), 1, glm::value_ptr(vec));
        }
        
        void set(const std::string &name, glm::vec4 vec) const
        {
            glUniform4fv(glGetUniformLocation(ID, name.c_str()), 1, glm::value_ptr(vec));
        }
        
    private:
        // utility function for checking shader compilation/linking errors.
        // ------------------------------------------------------------------------
        void checkCompileErrors(unsigned int shader, std::string type)
        {
            int success;
            char infoLog[1024];
            if (type != "PROGRAM")
            {
                glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
                if (!success)
                {
                    glGetShaderInfoLog(shader, 1024, NULL, infoLog);
                    std::cout << "ERROR::SHADER_COMPILATION_ERROR of type: " << type << "\n" << infoLog << "\n -- --------------------------------------------------- -- " << std::endl;
                }
            }
            else
            {
                glGetProgramiv(shader, GL_LINK_STATUS, &success);
                if (!success)
                {
                    glGetProgramInfoLog(shader, 1024, NULL, infoLog);
                    std::cout << "ERROR::PROGRAM_LINKING_ERROR of type: " << type << "\n" << infoLog << "\n -- --------------------------------------------------- -- " << std::endl;
                }
            }
        }
    };
    
    class Model_base {
    public:
        Model_base():shader(NULL), hasOwnership(false){}
        virtual ~Model_base(){
            if(hasOwnership)
                if(shader != NULL)
                    delete shader;
        }
        virtual void init() = 0;
        virtual void Draw() = 0;
        virtual void setShader(Shader *shader){
            if(hasOwnership)
                if(this->shader != NULL)
                    delete shader;
            this->shader = shader;
            hasOwnership = false;
        }
        virtual void addTexture(std::string name, unsigned int textureId, int type = GL_TEXTURE_2D, unsigned int order = 0){};
    protected:
        Shader *shader;
        bool hasOwnership;
    };
    
    
} // end of namespace glUtil
#endif
