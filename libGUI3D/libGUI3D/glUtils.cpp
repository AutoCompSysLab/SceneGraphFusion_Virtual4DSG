//
//  glUtils.cpp
//  DFGGUI
//
//  Created by Shun-Cheng Wu on 01/Jan/2019.
//

#include "glUtils.hpp"

namespace glUtil {
#ifdef WITH_STB
    // loads a cubemap texture from 6 individual texture faces
    // order:
    // +X (right)
    // -X (left)
    // +Y (top)
    // -Y (bottom)
    // +Z (front)
    // -Z (back)
    // -------------------------------------------------------
    unsigned int Utils::loadCubemap(const std::vector<std::string> &faces)
    {
        if(faces.size() != 6)
            throw std::runtime_error("GLUTIL::LOADCUBEMAP::faces must have 6 elements.\n");
        
        unsigned int textureID;
        glGenTextures(1, &textureID);
        glBindTexture(GL_TEXTURE_CUBE_MAP, textureID);
        
        int width, height, nrChannels;
        for (unsigned int i = 0; i < faces.size(); i++)
        {
            unsigned char *data = stbi_load(faces[i].c_str(), &width, &height, &nrChannels, 0);
            if (data)
            {
                glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i,
                             0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data
                             );
                stbi_image_free(data);
            }
            else
            {
                printf("UTILS::LOADCUBEMAP::Cubemap texture failed to load at path: %s\n", faces[i].c_str());
                stbi_image_free(data);
            }
        }
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
        
        return textureID;
    }
    
    uint Utils::loadTexture(char const * path){
        unsigned int textureID;
        glGenTextures(1, &textureID);
        
        int width, height, nrComponents;
        unsigned char *data = stbi_load(path, &width, &height, &nrComponents, 0);
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
                throw std::runtime_error("GUI::LOADTEXTURE::Image Channel not supported.\n");
            
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
#else
    unsigned int Utils::loadCubemap(const std::vector<std::string> &faces){
        throw std::runtime_error("load cube map requires stb library!\n");
    }
    uint Utils::loadTexture(char const *path) {
        throw std::runtime_error("load cube map requires stb library!\n");
    }
#endif
    
    void Utils::save_screen( const char *spath )
    {
        
        GLint vp[4];
        glGetIntegerv( GL_VIEWPORT, vp );
        
        int x,y, w,h;
        x = vp[0];
        y = vp[1];
        w = vp[2];
        h = vp[3];
        
        int j;
        
        unsigned char *bottomup_pixel = (unsigned char *) malloc( w*h*3*sizeof(unsigned char) );
        unsigned char *topdown_pixel = (unsigned char *) malloc( w*h*3*sizeof(unsigned char) );
        
        
        //Byte alignment (that is, no alignment)
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        
        glReadPixels( x, y, w, h, GL_RGB, GL_UNSIGNED_BYTE, bottomup_pixel);
        for( j=0; j<h; j++ )
            memcpy( &topdown_pixel[j*w*3], &bottomup_pixel[(h-j-1)*w*3], w*3*sizeof(unsigned char) );
        
        
        FILE *f0 = fopen( spath, "wb" );
        if( f0==NULL )
        {
            printf( "[Error] : SaveScreen(), Cannot open %s for writting.\n", spath );
            exit(-1);
        }
        
        fprintf( f0, "P6\n%d %d\n255\n", w, h);
        fwrite( topdown_pixel, sizeof(unsigned char), w*h*3, f0);
        fclose( f0 );
        
        free(bottomup_pixel);
        free(topdown_pixel);
    }
    
    std::vector<Vertex> ShapeVertices::skybox = {
        // positions
        Vertex(-1.0f,  1.0f, -1.0f),
        Vertex(-1.0f, -1.0f, -1.0f),
        Vertex(1.0f, -1.0f, -1.0f),
        Vertex(1.0f, -1.0f, -1.0f),
        Vertex(1.0f,  1.0f, -1.0f),
        Vertex(-1.0f,  1.0f, -1.0f),
        
        Vertex(-1.0f, -1.0f,  1.0f),
        Vertex(-1.0f, -1.0f, -1.0f),
        Vertex(-1.0f,  1.0f, -1.0f),
        Vertex(-1.0f,  1.0f, -1.0f),
        Vertex(-1.0f,  1.0f,  1.0f),
        Vertex(-1.0f, -1.0f,  1.0f),
        
        Vertex(1.0f, -1.0f, -1.0f),
        Vertex(1.0f, -1.0f,  1.0f),
        Vertex(1.0f,  1.0f,  1.0f),
        Vertex(1.0f,  1.0f,  1.0f),
        Vertex(1.0f,  1.0f, -1.0f),
        Vertex(1.0f, -1.0f, -1.0f),
        
        Vertex(-1.0f, -1.0f,  1.0f),
        Vertex(-1.0f,  1.0f,  1.0f),
        Vertex(1.0f,  1.0f,  1.0f),
        Vertex(1.0f,  1.0f,  1.0f),
        Vertex(1.0f, -1.0f,  1.0f),
        Vertex(-1.0f, -1.0f,  1.0f),
        
        Vertex(-1.0f,  1.0f, -1.0f),
        Vertex(1.0f,  1.0f, -1.0f),
        Vertex(1.0f,  1.0f,  1.0f),
        Vertex(1.0f,  1.0f,  1.0f),
        Vertex(-1.0f,  1.0f,  1.0f),
        Vertex(-1.0f,  1.0f, -1.0f),
        
        Vertex(-1.0f, -1.0f, -1.0f),
        Vertex(-1.0f, -1.0f,  1.0f),
        Vertex(1.0f, -1.0f, -1.0f),
        Vertex(1.0f, -1.0f, -1.0f),
        Vertex(-1.0f, -1.0f,  1.0f),
        Vertex(1.0f, -1.0f,  1.0f)
    };
    std::vector<Vertex> ShapeVertices::cube = {
        Vertex(-0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  0.0f,  0.0f),
        Vertex(0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  1.0f,  0.0f),
        Vertex(0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  1.0f,  1.0f),
        Vertex(0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  1.0f,  1.0f),
        Vertex(-0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  0.0f,  1.0f),
        Vertex(-0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  0.0f,  0.0f),
        Vertex(-0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  0.0f,  0.0f),
        Vertex(0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  1.0f,  0.0f),
        Vertex(0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  1.0f,  1.0f),
        Vertex(0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  1.0f,  1.0f),
        Vertex(-0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  0.0f,  1.0f),
        Vertex(-0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  0.0f,  0.0f),

        Vertex(-0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f,  1.0f,  0.0f),
        Vertex(-0.5f,  0.5f, -0.5f, -1.0f,  0.0f,  0.0f,  1.0f,  1.0f),
        Vertex(-0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f,  0.0f,  1.0f),
        Vertex(-0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f,  0.0f,  1.0f),
        Vertex(-0.5f, -0.5f,  0.5f, -1.0f,  0.0f,  0.0f,  0.0f,  0.0f),
        Vertex(-0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f,  1.0f,  0.0f),
        
        Vertex(0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f,  1.0f,  0.0f),
        Vertex(0.5f,  0.5f, -0.5f,  1.0f,  0.0f,  0.0f,  1.0f,  1.0f),
        Vertex(0.5f, -0.5f, -0.5f,  1.0f,  0.0f,  0.0f,  0.0f,  1.0f),
        Vertex(0.5f, -0.5f, -0.5f,  1.0f,  0.0f,  0.0f,  0.0f,  1.0f),
        Vertex(0.5f, -0.5f,  0.5f,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f),
        Vertex(0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f,  1.0f,  0.0f),
        
        Vertex(-0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,  0.0f,  1.0f),
        Vertex(0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,  1.0f,  1.0f),
        Vertex(0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,  1.0f,  0.0f),
        Vertex(0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,  1.0f,  0.0f),
        Vertex(-0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,  0.0f,  0.0f),
        Vertex(-0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,  0.0f,  1.0f),
        
        Vertex(-0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,  0.0f,  1.0f),
        Vertex(0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,  1.0f,  1.0f),
        Vertex(0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,  1.0f,  0.0f),
        Vertex(0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,  1.0f,  0.0f),
        Vertex(-0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,  0.0f,  0.0f),
        Vertex(-0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,  0.0f,  1.0f)
    };
    
    
    std::vector<Vertex> ShapeVertices::plane = {
        Vertex(1.0f, 0.f,  1.0f, 0.f, 1.f, 0.f,  1.0f, 0.0f),
        Vertex(-1.0f, 0.,  1.0f, 0.f, 1.f, 0.f,  0.0f, 0.0f),
        Vertex(-1.0f, 0., -1.0f, 0.f, 1.f, 0.f,  0.0f, 1.0f),
        
        Vertex(1.0f, 0.f,  1.0f, 0.f, 1.f, 0.f,  1.0f, 0.0f),
        Vertex(-1.0f, 0.f, -1.0f, 0.f, 1.f, 0.f,  0.0f, 1.0f),
        Vertex(1.0f, 0.f, -1.0f, 0.f, 1.f, 0.f,  1.0f, 1.0f)
    };
    float quadVertices[] = { // vertex attributes for a quad that fills the entire screen in Normalized Device Coordinates.
        // positions   // texCoords
        -1.0f,  1.0f,  0.0f, 1.0f,
        -1.0f, -1.0f,  0.0f, 0.0f,
        1.0f, -1.0f,  1.0f, 0.0f,
        
        -1.0f,  1.0f,  0.0f, 1.0f,
        1.0f, -1.0f,  1.0f, 0.0f,
        1.0f,  1.0f,  1.0f, 1.0f
    };
    std::vector<Vertex> ShapeVertices::quad = { // vertex attributes for a quad that fills the entire screen in Normalized Device Coordinates.
        // positions   // texCoords
        Vertex(-1.0f,  1.0f,  0.f,  0.f,  0.f,  0.f, 0.0f, 1.0f),
        Vertex(-1.0f, -1.0f,  0.f,  0.f,  0.f,  0.f, 0.0f, 0.0f),
        Vertex(1.0f, -1.0f,  0.f,  0.f,  0.f,  0.f, 1.0f, 0.0f),
        
        Vertex(-1.0f,  1.0f,  0.f,  0.f,  0.f,  0.f, 0.0f, 1.0f),
        Vertex(1.0f, -1.0f,  0.f,  0.f,  0.f,  0.f, 1.0f, 0.0f),
        Vertex(1.0f,  1.0f,  0.f,  0.f,  0.f,  0.f, 1.0f, 1.0f)
    };
}
