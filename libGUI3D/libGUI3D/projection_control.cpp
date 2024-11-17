#include "projection_control.hpp"

#include <imgui.h>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <stdexcept>

using namespace SC;

ProjectionControl::ProjectionControl(unsigned int windowWidth, unsigned int windowHeight)
:
fovy(45.0f),
bShowUI(false),
near(0.001f),
far(50.0f),
width_(windowWidth),height_(windowHeight),
projection_mode(0),
width(10.0f)
{}

ProjectionControl::~ProjectionControl() = default;

//void ProjectionControl::SetParams(float fx,float fy, float cx, float cy){
//    width_ = cx*2;
//    height_ = cy*2;
//    fovy = 2*atan(height_/(2*fy));
//}

glm::mat4 ProjectionControl::projection_matrix(float fx, float fy, float cx, float cy, int w, int h, float n, float f) {
    glm::mat4 proj;
    throw std::runtime_error("this is not working for now. Check glUtils::Perspective");
//    proj = glm::perspective<float>(glm::radians(fovy),
//                                   (float) w / (float) h,
//                                   n, f);
    proj[0][0] = 2 * fx / float(w);
    proj[2][0] = -(2*(cx / float(w)) - 1);
    proj[1][1] = 2 * fy / float(h);
    proj[2][1] = -(2*(cy / float(h)) - 1);
    proj[2][2] = -(f + n) / (f - n);
    proj[3][2] = -2 * (f * n) / (f - n);
    proj[2][3] = -1;
    return proj;
}

glm::mat4 ProjectionControl::projection_matrix() const {
    double aspect_ratio = float(width_)/float(height_);
    glm::mat4 proj;
    if(projection_mode == 0) {
        proj = glm::perspective<float>(glm::radians(fovy),
                                       (float) width_ /
                                       (float) height_, near, far);
    } else {
        proj = glm::ortho<float>(-width/2.f,width/2.f,-width/2.f/aspect_ratio, width/2.f/aspect_ratio, near,far);
    }
    return proj;
//  return Eigen::Map<Eigen::Matrix4f>(glm::value_ptr(proj));
}

void ProjectionControl::show() {
    bShowUI = true;
}

void ProjectionControl::draw_ui() {
  if(!bShowUI) {
    return;
  }

  ImGui::Begin("projection control", &bShowUI, ImGuiWindowFlags_AlwaysAutoResize);
  const char* modes[] = { "PERSPECTIVE", "ORTHOGONAL" };
  ImGui::Combo("Mode", &projection_mode, modes, IM_ARRAYSIZE(modes));
  if(projection_mode == 0){
    ImGui::DragFloat("FOV", &fovy, 0.1f, 1.0f, 180.0f);
  } else {
    ImGui::DragFloat("Width", &width, 0.1f, 1.0f, 1000.0f);
  }

  ImGui::DragFloat("Near", &near, 1.0f, 0.1f, far);
  ImGui::DragFloat("Far", &far, 1.0f, near, 10000.0f);

  ImGui::End();
}