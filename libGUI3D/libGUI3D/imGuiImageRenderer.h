#pragma once
#include "glImageRenderer.h"

namespace SC {
    namespace IMGUI{
        class ImGUI_ImageRenderWrapper {
        public:
            ImGUI_ImageRenderWrapper(unsigned char *data, int width, int height){
                imageRenderer_.reset(new glUtil::GLImageRenderer(nullptr,width, height));
                textureBuffer_.reset(new ITMUChar4Image(width, height,true));
            }
            void Update(){
                imageRenderer_->update((uchar *)textureBuffer_->GetData(MEMORYDEVICE_CPU),
                                                 textureBuffer_->noDims.x,
                                                 textureBuffer_->noDims.y);
            }
            ITMUChar4Image* getBuffer(){return textureBuffer_.get();}

            void ShowImage(Vector2i imgSize){
                ImGuiIO& io = ImGui::GetIO();
                ImVec2 pos = ImGui::GetCursorScreenPos();
                auto tex_id = reinterpret_cast<ImTextureID>(imageRenderer_->getTextID());
//                auto width = static_cast<float>(imageRenderer_->getWidth());
//                auto height = static_cast<float>(imageRenderer_->getHeight());
                auto width = imgSize.x;
                auto height = imgSize.y;
                ImGui::Image(tex_id, ImVec2(imgSize.x,imgSize.y),
                             ImVec2(0, 0), ImVec2(1, 1), ImVec4(1.0f, 1.0f, 1.0f, 1.0f), ImVec4(1.0f, 1.0f, 1.0f, 0.5f));
                if (ImGui::IsItemHovered())
                {
                    ImGui::BeginTooltip();
                    float region_sz = 32.0f;
                    float region_x = io.MousePos.x - pos.x - region_sz * 0.5f; if (region_x < 0.0f) region_x = 0.0f; else if (region_x > width - region_sz) region_x = width - region_sz;
                    float region_y = io.MousePos.y - pos.y - region_sz * 0.5f; if (region_y < 0.0f) region_y = 0.0f; else if (region_y > height - region_sz) region_y = height - region_sz;
                    float zoom = 4.0f;
                    ImGui::Text("Min: (%.2f, %.2f)", region_x, region_y);
                    ImGui::Text("Max: (%.2f, %.2f)", region_x + region_sz, region_y + region_sz);
                    float factor_x = float(width) / float(textureBuffer_->noDims.x) * textureBuffer_->noDims.x;
                    float factor_y = float(height) / float(textureBuffer_->noDims.y) *textureBuffer_->noDims.y;
                    ImVec2 uv0 = ImVec2((region_x) / factor_x, (region_y) / factor_y);
                    ImVec2 uv1 = ImVec2((region_x + region_sz) / factor_x, (region_y + region_sz) / factor_y);
                    ImGui::Image(tex_id, ImVec2(region_sz * zoom, region_sz * zoom), uv0, uv1, ImVec4(1.0f, 1.0f, 1.0f, 1.0f), ImVec4(1.0f, 1.0f, 1.0f, 0.5f));
                    ImGui::EndTooltip();
                }
            }
        private:
            std::unique_ptr<glUtil::GLImageRenderer> imageRenderer_;
            std::unique_ptr<ITMUChar4Image> textureBuffer_;
        };
    }
}