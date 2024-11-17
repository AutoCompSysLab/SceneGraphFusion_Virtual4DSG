//
// Created by sc on 8/23/20.
//

#ifndef GRAPHSLAM_IMAGEDRAWER_H
#define GRAPHSLAM_IMAGEDRAWER_H
//
//#ifdef COMPILE_WITH_EIGEN
//#include <Eigen/Core>
//#endif

#include "glShader.hpp"
#include <memory>
namespace glUtil {
    class ImageDrawer {
    public:
        ~ImageDrawer();
        void Init(unsigned int textureWidth, unsigned int textureHeight, GLenum format = GL_RGBA);
        void Update(const unsigned char *data, int width, int height);
        void Draw();

        GLenum mFormat;
        unsigned int VAO, VBO, textID;
        std::unique_ptr<glUtil::Shader> mShader;
    private:
        bool bInited=false;
    };
}

#endif //GRAPHSLAM_IMAGEDRAWER_H
