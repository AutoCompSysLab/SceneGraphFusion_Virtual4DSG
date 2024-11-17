#pragma once
#include "camera_control.h"
#include "projection_control.hpp"
#include <memory>

#ifdef COMPILE_WITH_EIGEN
#include <Eigen/Core>
#endif

namespace glUtil{
    // An abstract camera class that processes input and calculates the corresponding Euler Angles, Vectors and Matrices for use in OpenGL
    class  Camera
    {
    public:
        explicit Camera(int windowWidth, int windowHeight, glm::vec3 position = glm::vec3(0.0f, 0.0f, 0.0f),
               glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f),
               float yaw = -90.0f,
               float pitch = 0){
            bShowUI=false;
            camera_control_.reset(new SC::CameraControl(position,up,yaw,pitch));
            projection_control_.reset(new SC::ProjectionControl(windowWidth,windowHeight));
        }

        void setSize(int windowWidth, int windowHeight){
            projection_control_->setSize(windowWidth,windowHeight);
        }

        bool mouse_control() {
            bool changed = false;
            ImGuiIO& io = ImGui::GetIO();
            auto mouse_pos = ImGui::GetMousePos();
//            auto drag_delta = ImGui::GetMouseDragDelta();

            glm::vec2 p(mouse_pos.x, mouse_pos.y);

            for (int i = 0; i < 3; i++) {
                if (ImGui::IsMouseClicked(i)) {
                    changed=true;
                    camera_control_->mouse(p, i, true);
                }
                if (ImGui::IsMouseReleased(i)) {
                    changed=true;
                    camera_control_->mouse(p, i, false);
                }
                if (ImGui::IsMouseDragging(i)) {
                    changed=true;
                    camera_control_->drag(p, i);
                }

            }
            camera_control_->scroll(glm::vec2(io.MouseWheel, io.MouseWheelH));
            if(io.MouseWheel != 0 || io.MouseWheelH != 0 )changed=true;
            return changed;
        }

#ifdef COMPILE_WITH_EIGEN
        /// Model View (World View)
        Eigen::Matrix4f getCameraPose() {
            return Eigen::Matrix4f(glm::value_ptr(camera_control_->GetViewMatrix()));
//            Eigen::Matrix4f newT;
//            memcpy(newT.data(), glm::value_ptr(camera_control_->GetViewMatrix()), 16 * sizeof(float));
//            memcpy(newT.topRightCorner(3, 1).data(), glm::value_ptr(camera_control_->Position), 3 * sizeof(float));
//            return newT;
        }
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif

        void drawUI(){
            if(!bShowUI) return;
            ImGui::Begin("GL Camera", &bShowUI);
            if(ImGui::BeginMenu("Camera", bShowUI)){
                ImGui::MenuItem("Projection Control", nullptr, &projection_control_->bShowUI);
                ImGui::MenuItem("Camera Control", nullptr, &camera_control_->bShowUI);
                ImGui::EndMenu();
            }
            ImGui::End();
            projection_control_->draw_ui();
            camera_control_->drawUI();
        }

        std::unique_ptr<SC::CameraControlBase> camera_control_;
        std::unique_ptr<SC::ProjectionControl> projection_control_;

        bool bShowUI;
    private:
    };
    
} // end of namespace glUtil