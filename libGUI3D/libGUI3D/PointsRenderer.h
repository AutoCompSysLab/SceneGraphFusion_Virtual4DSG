//
// Created by sc on 8/19/20.
//

#ifndef GT_GRAPH_VIEWER_POINTSRENDERER_H
#define GT_GRAPH_VIEWER_POINTSRENDERER_H

#include <memory>
#include "glShader.hpp"
namespace PSLAM {
    class PointRenderer {
    public:
        explicit PointRenderer();
        /**
         *
         * @param points must be float4 or equivalent
         * @param colors must be float4 or equivalent. value should be from 0.f~1.f
         * @param size
         */
        void Update(float *points, float *colors, size_t size);

        glUtil::Shader *GetShader();

        void Draw(glm::mat4 projection, glm::mat4 viewMatrix) const;
        void SetPointRadius(float radius);
        float mPointRadius;
    private:
        std::shared_ptr<glUtil::Shader> mShader;
        size_t mBufferSize, mPointSize;
        bool mbBufferInited;
        unsigned int VAO,VBO;
        void InitShader();
        
        void UpdateBuffer(size_t newSize, bool force = false);
    };
}

#endif //GT_GRAPH_VIEWER_POINTSRENDERER_H
