#pragma once
#include <functional>
#include "../libGUI/GUI.h"
#include "glShader.hpp"
#include "glCamera.hpp"
#include "projection_control.hpp"
#include "glMesh.hpp"
#include "glUtils.hpp"
#include <map>
#include "camera_control.h"
#include "glImageRenderer.h"
#ifdef COMPILE_WITH_FREETYPE
#include <ft2build.h>
#include <memory>
#include FT_FREETYPE_H
#endif

namespace SC{
    struct Character {
        GLuint TextureID;   // ID handle of the glyph texture
        glm::ivec2 Size;    // Size of glyph
        glm::ivec2 Bearing;  // Offset from baseline to left/top of glyph
        GLuint Advance;    // Horizontal offset to advance to next glyph
    };
    class FPSManager{
    public:
        explicit FPSManager(double targetFPS = 30.0){
            fpsCounter_  = fps_ = targetFPS;
            diff_ = 0;
            fps_time_pre_ = fps_time_ = lasttime_ = -1;
            targetFPS_ = targetFPS;
            maxPeriod_ = 1.0 / targetFPS_;
        }
        /// Prevent unstable image jumpping.
        void wait(){
            if(lasttime_ > 0)
                while (glfwGetTime() < lasttime_ + maxPeriod_);
            lasttime_ = glfwGetTime();
        }

        void updateFPS(){
            stop();
            if(fps_time_pre_ < 0) { // init
                start();
                return;
            }
            checkUpdate();
        }

        inline void start(){
            fps_time_pre_ = glfwGetTime();
        }
        inline void stop(){
            fps_time_ = glfwGetTime();
        }

        inline void checkUpdate(){
            diff_ += fps_time_-fps_time_pre_;
            fpsCounter_++;
            if (diff_ > 1.0) {
                fps_ = double(fpsCounter_) / diff_;
                fpsCounter_ = 0;
                fps_time_pre_ += diff_;
                diff_=0;
            }
        }

//    double getFPS(){return fps_;}
        double& getFPS(){return fps_;}
    private:
        double fps_time_pre_, fps_time_, targetFPS_, fps_;
        double maxPeriod_, lasttime_;
        unsigned short fpsCounter_;
        double diff_;

    };
    struct task_element_t {
        GLFWWindowContainer* window_;
        int key_;
        const std::function< void() > no_id;
        std::string description;
        explicit task_element_t(GLFWWindowContainer* window, int key, std::function< void() >  f, std::string description) :
                window_(window), key_(key), no_id(std::move(f)), description(std::move(description)) { }
    };

    class GUI3D : public GUI_base{
    public:
        explicit GUI3D(const std::string &name, int width, int height, bool visible=true);
        virtual ~GUI3D();

        virtual void drawUI();
        virtual void drawGL();


        /// Adding new key callback
        template <typename Task> void registerKeyFunciton(GLFWWindowContainer* window, int key, Task func, const std::string &description = ""){
            registeredFunctions_.emplace_back(window,key,static_cast<std::function< void() >>(func), description);
        }

        void showRegisteredKeyFunction();

//        Eigen::Matrix4f getCameraPose() {
//            Eigen::Matrix4f newT;
//            memcpy(newT.data(), glm::value_ptr(camera_control->GetViewMatrix()), 16 * sizeof(float));
//            memcpy(newT.topRightCorner(3, 1).data(), glm::value_ptr(camera_control->Position), 3 * sizeof(float));
//            return newT;
//        }

        void setPlotTracjectory(bool option) {bPlotTrajectory = option;}

        glUtil::Camera* GetCam(){return glCam.get();}

        void RenderText (const std::string &text, float pos_x, float pos_y, float scale, const glm::vec3 &color);
    protected:
        std::map<std::string, unsigned int> glBuffers, glVertexArrays, glFrameBuffers, glTextures;
        std::map<std::string, glUtil::Shader*> glShaders;
        std::map<std::string, glUtil::Model_base*> glObjests;
//        std::map<std::string, glUtil::Model*> glModels;
        std::map<GLchar, Character> Characters;
        std::map<int, bool> bKeyProcessFinished;
        FPSManager *fps_;
        bool bShowGrid, bShowFPS;
        bool bPlotTrajectory;

        struct task_element_t {
            GLFWWindowContainer* window_;
            int key_;
            const std::function< void() > no_id;
            std::string description;
            explicit task_element_t(GLFWWindowContainer* window, int key, std::function< void() >  f, std::string description) :
                    window_(window), key_(key), no_id(std::move(f)), description(std::move(description)) { }
        };
        std::vector<task_element_t> registeredFunctions_;
        std::vector<glm::vec3> trajectories_;

        void RenderText(GLuint VAO, GLuint VBO, glUtil::Shader *shader, std::string text, GLfloat x, GLfloat y, GLfloat scale, glm::vec3 color);
        virtual void processInput(GLFWwindow* window);
        virtual void basicInputRegistration();
        virtual void basicProcess();
        virtual void plot_trajectory(const glm::mat4 *projection);
        virtual void add_trajectory(float x, float y, float z, float interval=0.002);
        /// return true is something changed
        bool mouseControl();

//        virtual void scroll_callback_impl(GLFWwindow* window, double xoffset, double yoffset);
//        virtual void mouse_callback_impl(GLFWwindow *window, double xpos, double ypos);

        void buildScreen();
        void buildCamera();
        void buildGrid();
        void buildText();
        void buildFreeType();
        void buildTrajectory();

        bool bShowCameraUI;
        void cameraUI();

        std::unique_ptr<glUtil::Camera> glCam;
        glm::vec3 camPose, camUp;
        float yaw, pitch, fov, fovMax, camSpeed;
    private:
        int init();
    };
}
