//
// Created by jw on 11/28/24.
//

#ifndef GRAPHSLAM_RENDERERVIRTUAL4DSG_H
#define GRAPHSLAM_RENDERERVIRTUAL4DSG_H
#include "Renderer.h"
#include <memory>
#include <Eigen/Core>

namespace PSLAM {
    class MeshRendererVirtual4DSG : public MeshRendererInterface {
    public:
        MeshRendererVirtual4DSG(int width, int height, const std::string &dataPath, const std::string &sequence, bool toReference):
                MeshRendererInterface(width,height,dataPath,sequence) {
            Init();
        }
    private:
        std::unique_ptr<glUtil::Shader> mShader;
        std::unique_ptr<glUtil::Model> mModel;

        void Init(){
            mShader = std::make_unique<glUtil::Shader>(vs, fs);
            // mModel = std::make_unique<glUtil::Model>( m_folder+"/"+m_scanId+"/PMesh.ply" );
            // mModel = std::make_unique<glUtil::Model>( m_folder+"/"+m_scanId+"/labels.instances.annotated.v2.ply" );
            mModel = std::make_unique<glUtil::Model>( m_folder + "/" + m_scanId +  "/mesh.refined.v2.obj");

            // OBJ 파일의 좌표계를 왼손에서 오른손 좌표계로 변환
            Eigen::Vector3f translation(0.0f, 0.0f, -15.94f); // z축 방향 offset (only for OBJ 파일)
            mModel->translate(translation);
            Eigen::Matrix4f convertLHtoRH = Eigen::Matrix4f::Identity();
            convertLHtoRH(0, 0) = -1;  // X 축 반전 (only for OBJ 파일)
            convertLHtoRH(2, 2) = -1;  // Z 축 반전
            mModel->transform(convertLHtoRH);

            mModel->setShader(mShader.get());
        }

        void Render(const Eigen::Matrix4f &projection, const Eigen::Matrix4f &view, float near, float far) override{
            glEnable(GL_DEPTH_TEST);

            glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
            glClearColor(0.00f, 0.00f, 0.00f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            glViewport(0,0,m_width,m_height);

            mShader->use();
            mShader->set("view", view);
            mShader->set("projection", projection);
            mShader->set("model", glm::mat4(1.f));
            mModel->Draw();

            RenderRGB(m_rgb);
            cv::flip(m_rgb, m_rgb, 0);
            cv::cvtColor(m_rgb, m_rgb, cv::COLOR_RGB2BGR);

            RenderDepth(m_depth,near,far);
            cv::flip(m_depth, m_depth, 0);
            m_depth.convertTo(m_depth, CV_16U);

            glBindFramebuffer(GL_FRAMEBUFFER, 0);
        }

        void RenderRGB(cv::Mat &img) {
            img = cv::Mat(m_height, m_width, CV_8UC3);
            glReadPixels(0, 0, m_width,m_height, GL_RGB, GL_UNSIGNED_BYTE,
                         img.data);
        }

        void RenderDepth(cv::Mat &img, float near, float far) {
            img.create(m_height, m_width, CV_32F);
            glReadPixels(0, 0,m_width,m_height, GL_DEPTH_COMPONENT, GL_FLOAT, img.data);
            for (size_t i = 0; i < (size_t) img.rows * img.cols; ++i) {
                if (img.at<float>(i) == 1 || img.at<float>(i) == 0) img.at<float>(i) = -1;
                else {
                    const float zn = (img.at<float>(i) * 2 - 1);
                    const float ze = 2.0f * near * far / (far + near - zn * (far - near));
                    img.at<float>(i) = ze * 1000;;
                }
            }
        }

        std::string vs =
                "#version 330 core\n"
                "layout (location = 0) in vec3 aPos;\n"
                "layout (location = 1) in vec3 aColor;\n"
                "out vec3 color;\n"
                "uniform mat4 model;\n"
                "uniform mat4 view;\n"
                "uniform mat4 projection;\n"
                "\n"
                "void main()\n"
                "{\n"
                "    gl_Position = projection * view * model * vec4(aPos, 1.0);\n"
                "    color = aColor;\n"
                "}";

        std::string fs =
                "#version 330 core\n"
                "out vec4 FragColor;\n"
                "\n"
                "in vec3 color;\n"
                "\n"
                "void main()\n"
                "{\n"
                "    FragColor = vec4(color,1);\n"
                "}";
    };
}


#endif //GRAPHSLAM_RENDERERSCANNET_H


/////////////////////////////////////////////////////
//
// Created by jw on 11/28/24.
//

// #pragma once

// #include "Renderer.h"
// #include "../libGUI3D/libGUI3D/glModel.hpp"
// #include <memory>
// #include <string>
// #include <opencv2/imgproc.hpp>
// #include <unordered_map>
// #include "Scan3R_json_loader.h"

// namespace PSLAM {
//     class MeshRendererVirtual4DSG : public MeshRendererInterface {
//     public:
//         MeshRendererVirtual4DSG(int width, int height, const std::string &dataPath, const std::string &sequence, bool toReference):
//                 MeshRendererInterface(width,height,dataPath,sequence){
//             mb_toRef = toReference;
//             Init();
//         }

//         void DrawScene(glUtil::Model *model, glUtil::Shader *shader){
//             glClearColor(0.00f, 0.00f, 0.00f, 1.0f);
//             glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//             shader->use();
//             model->Draw();
//         }


//         void Render(const Eigen::Matrix4f &projection, const Eigen::Matrix4f &view, float near, float far) override{
//             glEnable(GL_DEPTH_TEST);
//             glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
//             glViewport(0,0,m_width,m_height);
//             mShaderLabel->use();
//             mShaderLabel->set("view", view);
//             mShaderLabel->set("projection", projection);
//             mShaderLabel->set("model", glm::mat4(1.f));

//             mShaderRGB->use();
//             mShaderRGB->set("view", view);
//             mShaderRGB->set("projection", projection);
//             mShaderRGB->set("model", glm::mat4(1.f));

//             // Render Label & Instances
//             DrawScene(mModelLabel.get(), mShaderLabel.get());
//             RenderLabel(m_label);
//             cv::flip(m_label, m_label, 0);
//             RenderInstance(m_label,m_instance);

//             // Render RGB & Depth
//             DrawScene(mModelRGB.get(), mShaderRGB.get());
//             RenderRGB(m_rgb);
//             cv::flip(m_rgb, m_rgb, 0);
//             cv::cvtColor(m_rgb, m_rgb, cv::COLOR_RGB2BGR);

//             RenderDepth(m_depth,near,far);
//             cv::flip(m_depth, m_depth, 0);
//             m_depth.convertTo(m_depth, CV_16U);

// //            double min;
// //            double max;
// //            cv::minMaxIdx(depth, &min, &max);
// //            std::cout << min << ", " << max << "\n";
// //            cv::Mat adjMap;
// //            cv::convertScaleAbs(depth, adjMap, 255 / (max-min),min);
// //            applyColorMap(adjMap, adjMap, cv::COLORMAP_TURBO);
// //            cv::imshow("label", label);
// //            cv::imshow("rgb", rgb);
// //            cv::imshow("adjMap", adjMap);
// //            cv::waitKey(0);

//             glBindFramebuffer(GL_FRAMEBUFFER, 0);
//         }
// //        float mNearPlane, mFarPlane;
//     private:
//         bool mb_toRef; // move mesh to the frame of reference scan
//         std::unique_ptr<glUtil::Shader> mShaderLabel, mShaderRGB;
//         std::unique_ptr<glUtil::Model> mModelLabel, mModelRGB;
//         std::unordered_map<unsigned long, int> m_color2instances;
//         std::unordered_map<int, std::string> m_instances2label;
//         cv::Mat m_label, m_instance;

//         void Init(){
//             // Load Virtual4DSG and mesh data
//             mModelLabel = std::make_unique<glUtil::Model>( m_folder + "/" + m_scanId +  "/labels.instances.annotated.v2.ply");
//             mModelRGB = std::make_unique<glUtil::Model>( m_folder + "/" + m_scanId +  "/mesh.refined.v2.obj");
//             // mModelRGB = std::make_unique<glUtil::Model>( m_folder + "/" + m_scanId +  "/PMesh.ply");

//             // PLY 모델의 좌표계를 왼손에서 오른손 좌표계로 변환
//             Eigen::Matrix4f convertLHtoRH = Eigen::Matrix4f::Identity();
//             convertLHtoRH(2, 2) = -1;  // Z 축 반전
//             // convertLHtoRH(0, 0) = -1;  // X 축 반전
//             // mModelRGB->transform(convertLHtoRH);
//             mModelLabel->transform(convertLHtoRH);

//             if(!LoadObjects( m_folder + "/objects.json")) throw std::runtime_error("unable to load object data!");


//             // Load and set shader
//             mShaderLabel = std::make_unique<glUtil::Shader>(vs, fs);
//             mShaderRGB = std::make_unique<glUtil::Shader>(vs_texture, fs_texture);
//             mModelLabel->setShader(mShaderLabel.get());
//             mModelRGB->setShader(mShaderRGB.get());
//         }

//         void RenderRGB(cv::Mat &img) {
//             img = cv::Mat(m_height, m_width, CV_8UC3);
//             glReadPixels(0, 0, m_width,m_height, GL_RGB, GL_UNSIGNED_BYTE,img.data);
//         }

//         void RenderLabel(cv::Mat &img){
//             img = cv::Mat(m_height, m_width, CV_8UC3);
//             glReadPixels(0, 0, m_width,m_height, GL_RGB, GL_UNSIGNED_BYTE, img.data);
//         }

//         void RenderInstance(const cv::Mat &labelImg, cv::Mat &instanceImg) {
//             instanceImg = cv::Mat(m_height, m_width, CV_8UC3);

//             auto RGB2Hex = [](int r, int g, int b) -> unsigned long{
//                 return ((r & 0xff) << 16) + ((g & 0xff) << 8) + (b & 0xff);
//             };

//             for (int i = 0; i < m_width; i++) {
//                 for (int j = 0; j < m_height; j++) {
//                     const cv::Vec3b& vec = labelImg.at<cv::Vec3b>(j, i);
//                     const unsigned long color_hex = RGB2Hex(vec(2), vec(1), vec(0));
//                     if (m_color2instances.find(color_hex) != m_color2instances.end()) {
//                         const unsigned short Id = m_color2instances[color_hex]; // instance ID
//                         instanceImg.at<unsigned short>(j, i) = Id;
//                     } else instanceImg.at<unsigned short>(j, i) = 0;
//                 }
//             }
//         }

//         void RenderDepth(cv::Mat &img, float near, float far) {
//             img = cv::Mat::zeros(m_height, m_width, CV_32F);
//             glReadPixels(0, 0,m_width,m_height, GL_DEPTH_COMPONENT, GL_FLOAT, img.data);
//             for (size_t i = 0; i < (size_t) img.rows * img.cols; ++i) {
//                 if (img.at<float>(i) == 1 || img.at<float>(i) == 0) img.at<float>(i) = -1;
//                 else {
//                     const float zn = (img.at<float>(i) * 2 - 1);
//                     const float ze = 2.0f * near * far / (far + near - zn * (far - near));
//                     img.at<float>(i) = ze * 1000;;
//                 }
//             }
// //            cv::resize(img, img, cv::Size(loader->m_depthWidth, loader->m_depthHeight), 0, 0, cv::INTER_NEAREST);
//         }

//         bool LoadObjects(const std::string& obj_file) {
//             std::ifstream is(obj_file);
//             std::string dataset((std::istreambuf_iterator<char>(is)), std::istreambuf_iterator<char>());
//             std::string err;
//             const auto json = json11::Json::parse(dataset, err);
//             if (err != "") {
//                 std::cout << "didn't find objects.json in " << m_folder << std::endl;
//                 return false;
//             }
//             // Iterates through all the scans.
//             for (auto &scan_json: json["scans"].array_items()) {
//                 const std::string& scan_id = scan_json["scan"].string_value();
//                 if (scan_id != m_scanId) continue;
//                 const auto& objects = scan_json["objects"].array_items();
//                 for (const auto& obj: objects) {
//                     const int id = std::stoi(obj["id"].string_value());
//                     const std::string hex_str = obj["ply_color"].string_value();
//                     unsigned long color_hex;
//                     std::istringstream iss(hex_str.substr(1));
//                     iss >> std::hex >> color_hex;
//                     m_color2instances[color_hex] = id;
//                     m_instances2label[id] = obj["label"].string_value();
//                 }
//             }
//             return !m_color2instances.empty();
//         }

//         std::string vs =
//                 "#version 330 core\n"
//                 "layout (location = 0) in vec3 aPos;\n"
//                 "layout (location = 1) in vec3 aColor;\n"
//                 "out vec3 color;\n"
//                 "uniform mat4 model;\n"
//                 "uniform mat4 view;\n"
//                 "uniform mat4 projection;\n"
//                 "\n"
//                 "void main()\n"
//                 "{\n"
//                 "    gl_Position = projection * view * model * vec4(aPos, 1.0);\n"
//                 "    color = aColor;\n"
//                 "}";

//         std::string fs =
//                 "#version 330 core\n"
//                 "out vec4 FragColor;\n"
//                 "\n"
//                 "in vec3 color;\n"
//                 "\n"
//                 "void main()\n"
//                 "{\n"
//                 "    FragColor = vec4(color,1);\n"
//                 "}";

//         std::string vs_texture =
//                 "#version 330 core\n"
// //                "layout (location = 0) in vec3 position;\n"
// //                "layout (location = 1) in vec3 normal;\n"
// //                "layout (location = 2) in vec2 texCoords;\n"
//                 "layout (location = 0) in vec3 aPos;\n"
//                 "layout (location = 1) in vec3 aColor;\n"
//                 "layout (location = 2) in vec3 aNormal;\n"
//                 "layout (location = 3) in vec2 aTexCoords;\n"
//                 "layout (location = 4) in vec3 aTangent;\n"
//                 "layout (location = 5) in vec3 aBiTangent;\n"
//                 "\n"
//                 "out vec2 TexCoords;\n"
//                 "\n"
//                 "uniform mat4 model;\n"
//                 "uniform mat4 view;\n"
//                 "uniform mat4 projection;\n"
//                 "\n"
//                 "void main() \n"
//                 "{\n"
//                 "    gl_Position = projection * view * model * vec4(aPos, 1.0);\n"
//                 "    TexCoords = aTexCoords;\n"
//                 "}";

//         std::string fs_texture =
//                 "#version 330 core\n"
//                 "\n"
//                 "in vec2 TexCoords;\n"
//                 "out vec4 color;\n"
//                 "\n"
//                 "uniform sampler2D texture_diffuse;\n"
//                 "\n"
//                 "void main()\n"
//                 "{\n"
//                 "    color = vec4(texture(texture_diffuse, TexCoords));\n"
//                 "}";
//     };
// }
