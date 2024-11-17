#include "../GUI.h"

class GUI_fixed : public SC::GUI_base{
public:
    GUI_fixed(const std::string &name, int width, int height){
        GUI_base::initWindow(name,width,height);
        windowStyle();
    }

    void drawUI() override{
        SC::GUI_base::drawUI();
        drawLeftPane();
        bottomRightOverlay();
    }

    void windowStyle(){
        auto& style = ImGui::GetStyle();
        style.WindowRounding = 0;
    }

    void drawLeftPane(){
        auto& io = ImGui::GetIO();
        ImGui::SetNextWindowPos(ImVec2(0,0), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(200,io.DisplaySize.y), ImGuiCond_Always);
        ImGui::Begin("Fixed left panel",nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_MenuBar);
        // menu bar
        if(ImGui::BeginMenuBar()){
            if(ImGui::BeginMenu("System")){
                ImGui::EndMenu();
            };
            if(ImGui::BeginMenu("1")){
                ImGui::EndMenu();
            };
            if(ImGui::BeginMenu("2")){
                ImGui::EndMenu();
            };
            ImGui::EndMenuBar();
        }
        ImGui::End();
    }

    void bottomRightOverlay(){
        const float DISTANCE = 10.0f;
        bool p_open = true;
        static int corner = 3;
        ImGuiIO& io = ImGui::GetIO();
        if (corner != -1)
        {
            ImVec2 window_pos = ImVec2((corner & 1) ? io.DisplaySize.x - DISTANCE : DISTANCE, (corner & 2) ? io.DisplaySize.y - DISTANCE : DISTANCE);
            ImVec2 window_pos_pivot = ImVec2((corner & 1) ? 1.0f : 0.0f, (corner & 2) ? 1.0f : 0.0f);
            ImGui::SetNextWindowPos(window_pos, ImGuiCond_Always, window_pos_pivot);
        }
        ImGui::SetNextWindowBgAlpha(0.35f); // Transparent background
        if (ImGui::Begin("System Info", &p_open, (corner != -1 ? ImGuiWindowFlags_NoMove : 0) | ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav))
        {
            ImGui::Text("System Info");
            ImGui::Separator();
            if (ImGui::IsMousePosValid()) {
                ImGui::Text("Mouse Position: (%.1f,%.1f)", io.MousePos.x, io.MousePos.y);

                unsigned char pixelColor[4];
                glReadPixels(io.MousePos.x, window_->height - io.MousePos.y, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, &pixelColor);
                ImGui::Text("Color: %d %d %d %d", (int)pixelColor[0],(int)pixelColor[1],(int)pixelColor[2],(int)pixelColor[3]);
                ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
            } else
                ImGui::Text("Mouse Position: <invalid>");

            if (ImGui::BeginPopupContextWindow())
            {
                if (ImGui::MenuItem("Custom",       NULL, corner == -1)) corner = -1;
                if (ImGui::MenuItem("Top-left",     NULL, corner == 0)) corner = 0;
                if (ImGui::MenuItem("Top-right",    NULL, corner == 1)) corner = 1;
                if (ImGui::MenuItem("Bottom-left",  NULL, corner == 2)) corner = 2;
                if (ImGui::MenuItem("Bottom-right", NULL, corner == 3)) corner = 3;
                if (p_open && ImGui::MenuItem("Close")) p_open = false;
                ImGui::EndPopup();
            }
        }
        ImGui::End();
    }
};

int main(){
    GUI_fixed gui("Test_window", 1200,800);
    gui.run();
    return 0;
}