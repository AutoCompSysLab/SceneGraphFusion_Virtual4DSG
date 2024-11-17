#pragma once

namespace glUtil {
    class GLImageRenderer {
    public:

        GLImageRenderer(unsigned char *data, int windowWidth, int windowHeight) : width_(windowWidth),
                                                                                  height_(windowHeight) {
            float quadVertices[] = { // vertex attributes for a quad that fills the entire screen in Normalized Device Coordinates.
                    // positions   // texCoords
                    -1.0f, 1.0f, 0.0f, 1.0f,
                    -1.0f, -1.0f, 0.0f, 0.0f,
                    1.0f, -1.0f, 1.0f, 0.0f,

                    -1.0f, 1.0f, 0.0f, 1.0f,
                    1.0f, -1.0f, 1.0f, 0.0f,
                    1.0f, 1.0f, 1.0f, 1.0f
            };
            // screen quad VAO
            unsigned int &quadVAO = VAO;
            unsigned int &quadVBO = VBO;
            glGenVertexArrays(1, &quadVAO);
            glGenBuffers(1, &quadVBO);
            glBindVertexArray(quadVAO);
            glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
            glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), &quadVertices, GL_STATIC_DRAW);
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *) 0);
            glEnableVertexAttribArray(1);
            glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *) (2 * sizeof(float)));

            glGenTextures(1, &textID);
            glBindTexture(GL_TEXTURE_2D, textID);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, windowWidth, windowHeight, 0, GL_RGB,
                         GL_UNSIGNED_BYTE, data);

            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            // For Alpha texture
            GLenum format = GL_RGBA;
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, format == GL_RGBA ? GL_CLAMP_TO_EDGE
                                                                                : GL_REPEAT); // for this tutorial: use GL_CLAMP_TO_EDGE to prevent semi-transparent borders. Due to interpolation it takes texels from next repeat
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, format == GL_RGBA ? GL_CLAMP_TO_EDGE : GL_REPEAT);
        }
        ~GLImageRenderer(){
            glDeleteVertexArrays(1, &VAO);
            glDeleteBuffers(1, &VBO);
            glDeleteBuffers(1, &textID);
        }

        void update(unsigned char *data, int width, int height) {
            glBindTexture(GL_TEXTURE_2D, textID);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
        }

        void draw(glUtil::Shader *shader) {
            /// Draw Image
            glActiveTexture(GL_TEXTURE0);
            shader->use();
            glBindVertexArray(VAO);
            glBindTexture(GL_TEXTURE_2D, textID);
            glDrawArrays(GL_TRIANGLES, 0, 6);
        }

        unsigned int getTextID() { return textID; }

        unsigned int getVAO() { return VAO; }

        unsigned int getVBO() { return VBO; }

        int getWidth() {return width_;}
        int getHeight(){return height_;}

    private:
        int width_, height_;
        unsigned int VAO, VBO, textID;
    };
}