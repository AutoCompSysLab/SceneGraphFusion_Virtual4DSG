#pragma once
#include "imgui.h"
#if defined(IMGUI_IMPL_OPENGL_LOADER_GL3W)
#include <GL/gl3w.h>    // Initialize with gl3wInit()
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLEW)
#include <GL/glew.h>    // Initialize with glewInit()
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLAD)
#include <glad/glad.h>  // Initialize with gladLoadGL()
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLBINDING)
#define GLFW_INCLUDE_NONE         // GLFW including OpenGL headers causes ambiguity or multiple definition errors.
#include <glbinding/glbinding.h>  // Initialize with glbinding::initialize()
#include <glbinding/gl/gl.h>
using namespace gl;
#else
#include IMGUI_IMPL_OPENGL_LOADER_CUSTOM
#endif
// Include glfw3.h after our OpenGL definitions
#include <GLFW/glfw3.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
//#include <ImGuiFileDialog.h>
//#include <ImGuiFileDialogConfig.h>
#include <vector>
#include <string>
#include <utility>
namespace SC{
//    #define GUIInstance GUI::getInstance()
    enum DisplayMode {RGB, DEPTH};

    struct GLFWWindowContainer {
        GLFWwindow* window;
        std::string name_;
        int width, height;
        int runtimeWidth{}, runtimeHeight{};
//        float nearPlane=0.4, farPlane=500.f;
        GLFWWindowContainer(std::string name, int width, int height, float nearPlane=0.4, float farPlane=500.f, bool visible=true):
        name_(std::move(name)), width(width),height(height){
            if(!visible)
                glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
            window = glfwCreateWindow(width,height,name_.c_str(),nullptr,nullptr);
        }
        ~GLFWWindowContainer(){
            glfwDestroyWindow(window);
        };
    };
    class GUI_base {
    public:
        static GUI_base& getInstance()
        {
            return *ptrInstance;
        }
        GUI_base();
        virtual ~GUI_base();

        void initWindow(const std::string &name, int width, int height, bool visible=true);

        virtual void run();

        /// Draw ImGUI related
        virtual void drawUI();
        /// Draw OpenGL related
        virtual void drawGL();
        /// Draw OpenGL from RGBD related
        // virtual void drawRGBD();

        GLFWWindowContainer * GetWindow(){return window_;}
    protected:
        bool bDrawDemoUI;
        static GUI_base *ptrInstance;
        GLFWWindowContainer *window_;

        // Call Back Functions
        static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods){
            getInstance().key_callback_impl(window, key, scancode, action, mods);
        }
        virtual void key_callback_impl(GLFWwindow* window, int key, int scancode, int action, int mods);
        static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods){
            getInstance().mouse_button_callback_impl(window, button, action, mods);
        }
        virtual void mouse_button_callback_impl(GLFWwindow* window, int button, int action, int mods);
        static void mouse_callback(GLFWwindow* window, double xpos, double ypos){
            getInstance().mouse_callback_impl(window, xpos, ypos);
        }
        virtual void mouse_callback_impl(GLFWwindow* window, double xpos, double ypos);
        static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset){
            getInstance().scroll_callback_impl(window, xoffset, yoffset);
        }
        virtual void scroll_callback_impl(GLFWwindow* window, double xoffset, double yoffset);
        static void framebuffer_size_callback(GLFWwindow* window, int width, int height){
            getInstance().framebuffer_size_callback_impl(window, width, height);
        }
        virtual void framebuffer_size_callback_impl(GLFWwindow* window, int width, int height);
        static void error_callback(int error, const char* description){
            getInstance().error_callback_impl(error, description);
        }
        virtual void error_callback_impl(int error, const char* description);

        void ImGuiFileDialog_demo();

        std::vector<std::string> ImGuiFileSelect(const std::string &name = "Browse", int num = 1);

        void HelpMarker(const char* desc);
    private:
        std::string glsl_version;

        void init();
    };
}