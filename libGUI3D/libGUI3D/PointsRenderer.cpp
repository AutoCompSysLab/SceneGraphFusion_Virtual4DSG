//
// Created by sc on 8/19/20.
//
#include "PointsRenderer.h"

using namespace PSLAM;

PointRenderer::PointRenderer():mBufferSize(1e6),mPointSize(0),mbBufferInited(false){
    InitShader();
}

void PointRenderer::Update(float *points, float *colors, size_t size){
    UpdateBuffer(size);
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);

    glBufferSubData(GL_ARRAY_BUFFER, 0,
                    size * 4 * sizeof(GLfloat), points);
    glBufferSubData(GL_ARRAY_BUFFER, mBufferSize * 4 * sizeof(GLfloat),
                    size * 4 * sizeof(GLfloat), colors);
    glBindVertexArray(0);
}

void PointRenderer::SetPointRadius(float radius) {
    mPointRadius = radius;
    mShader->use();
    mShader->set("radius",mPointRadius);
}

glUtil::Shader *PointRenderer::GetShader(){return mShader.get();}

void PointRenderer::Draw(glm::mat4 projection, glm::mat4 viewMatrix) const{
    mShader->use();
    mShader->set("projection", projection);
    mShader->set("view", viewMatrix);
    glBindVertexArray(VAO);
    glDrawArrays(GL_POINTS, 0, mPointSize);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void PointRenderer::InitShader(){
    const std::string shaderPath = std::string(GUI_FOLDER_PATH) + "Shaders/";
    mBufferSize = 1e6;
    mPointRadius = 0.05;
    mShader = std::make_shared<glUtil::Shader>(shaderPath + "circle.vs",
                                               shaderPath + "circle.fs",
                                               shaderPath + "circle.gs");
    SetPointRadius(mPointRadius);
    glm::mat4 modelMat = glm::mat4(1.f);
    mShader->use();
    mShader->set("model", modelMat);
}

void PointRenderer::UpdateBuffer(size_t newSize, bool force){
    mPointSize = newSize;
    if (!force)
        if (mBufferSize > newSize && mbBufferInited) return;

    mBufferSize = (std::floor(newSize / 1e6) + 1) * 1e6;

    if (mbBufferInited) {
        glDeleteVertexArrays(1, &VAO);
        glDeleteBuffers(1, &VBO);
    }

    const uint numStride = 8;
    const size_t totalSize = mBufferSize * numStride;

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * totalSize, nullptr, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(GLfloat), 0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(GLfloat),
                          (void *) (mBufferSize * 4 * sizeof(GLfloat)));
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    mbBufferInited=true;
}