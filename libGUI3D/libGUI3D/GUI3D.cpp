#include "GUI3D.h"
using namespace SC;

GUI3D::GUI3D(const std::string &name, int width, int height,bool visible){
    GUI_base::initWindow(name,width,height,visible);
    fps_ = new FPSManager();
    bShowFPS = bShowGrid = false;
    bPlotTrajectory = true;
    bShowCameraUI=true;//todo: not here

    camPose = glm::vec3(0.0f, 0.f,  -4.5f);
    camUp = glm::vec3(0.f, 1.f, 0.f);
    yaw = 0.f; // y-axis 0-180-360: +z, 90-270-450: -z
    pitch = 0.f; // z-axis
    glCam.reset(new glUtil::Camera(window_->width,window_->height,camPose, camUp, yaw, pitch));
    init();
}
GUI3D::~GUI3D(){
    for (auto &model : glObjests)
        if (model.second != NULL) delete model.second;
//    for (auto &model : glModels)
//        if (model.second != NULL) delete model.second;
    for (auto &shader : glShaders)
        if (shader.second != NULL) delete shader.second;
    for (auto &texture : glTextures)
        glDeleteTextures(1, &texture.second);
    for (auto vao : glVertexArrays)
        glDeleteVertexArrays(1, &vao.second);
    for (auto vbo : glBuffers)
        glDeleteBuffers(1, &vbo.second);
    for (auto fbo : glFrameBuffers)
        glDeleteFramebuffers(1, &fbo.second);
    delete fps_;
}

int GUI3D::init(){
    basicInputRegistration();
    buildScreen();
    buildCamera();
    buildGrid();
    buildText();
    buildFreeType();
    buildTrajectory();
    return 1;
}

void GUI3D::drawUI() {
    GUI_base::drawUI();
    cameraUI();
    glCam->drawUI();
    mouseControl();
}

void GUI3D::drawGL(){
    processInput(window_->window);
    basicProcess();
}

void GUI3D::processInput(GLFWwindow* window) {
    auto io = ImGui::GetIO();
    if(!io.WantCaptureKeyboard) {
        for (const auto &func : registeredFunctions_) {
            if (glfwGetKey(func.window_->window, func.key_) == GLFW_PRESS && bKeyProcessFinished[func.key_]) {
                bKeyProcessFinished[func.key_] = !bKeyProcessFinished[func.key_];
                func.no_id();
            }
            if (glfwGetKey(func.window_->window, func.key_) == GLFW_RELEASE) bKeyProcessFinished[func.key_] = true;
        }
    }
}

void GUI3D::basicInputRegistration(){
    registerKeyFunciton(window_, GLFW_KEY_C,
                        [&]() {
                            if (!bShowGrid) {
                                printf("Show Grid On\n");
                                bShowGrid = true;
                            } else {
                                printf("Show Grid Off\n");
                                bShowGrid = false;
                            }
                        });
    /// X Show FPS
    registerKeyFunciton(window_, GLFW_KEY_X, [&]() { bShowFPS = !bShowFPS; });
}

void GUI3D::buildScreen(){
    const std::string shaderPath = std::string(GUI_FOLDER_PATH) + "Shaders/";
    /// Screen
    {
        glShaders["Screen"] = new glUtil::Shader(shaderPath + "2D.vs",shaderPath + "screen.fs");
        glm::mat4 projection = glm::ortho(0.0f, static_cast<GLfloat>(window_->runtimeWidth), 0.0f,
                                          static_cast<GLfloat>(window_->runtimeHeight));
        glShaders["Screen"]->use();
        glShaders["Screen"]->set("projection", projection);
        glShaders["Screen"]->set("screenTexture", 0);

        //TODO: debug screen3D
        glShaders["Screen3D"] = new glUtil::Shader(shaderPath + "2D_in_3D.vs",shaderPath + "2D_in_3D.fs");
        glShaders["Screen3D"]->use();
        glShaders["Screen3D"]->set("screenTexture", 0);
    }
}
void GUI3D::buildCamera(){
    const std::string shaderPath = std::string(GUI_FOLDER_PATH) + "Shaders/";
    /// Camera
    {
        float cam_width = 0.3, cam_height = 0.2, cam_front = 1.0, cam_back = 0.f, ratio = 3;
        float camera_points[] = { // vertex attributes for a quad that fills the entire screen in Normalized Device Coordinates.
                -cam_width, -cam_height, cam_back, // Left, Bottom, Back
                cam_width, -cam_height, cam_back,  // Right, Bottom, Back
                -cam_width, cam_height, cam_back,  // Left, Top, Back
                cam_width, cam_height, cam_back,   // Right, Top, Back
                -cam_width * ratio, -cam_height * ratio, cam_front, // Left, Bottom, Front
                cam_width * ratio, -cam_height * ratio, cam_front,  // Right, Bottom, Front
                -cam_width * ratio, cam_height * ratio, cam_front,  // Left, Top, Front
                cam_width * ratio, cam_height * ratio, cam_front,  // Right, Top, Front
                0, 0, cam_back, // Center, Back
                cam_width, 0, cam_back,
                0, cam_height, cam_back
        };

        unsigned int indices_line[] = {
                0, 1,
                2, 0,
                3, 2,
                3, 1,
                4, 5,
                4, 6,
                7, 5,
                7, 6,
                0, 4,
                1, 5,
                2, 6,
                3, 7,

                8, 9,
                8, 10
        };

        // Camera
        std::string name = "Camera";
        glShaders["Camera"] = new glUtil::Shader(shaderPath + "camera_shader.vs",
                                                 shaderPath + "camera_shader.fs");
        glShaders["Camera"]->use();
        glShaders["Camera"]->set("color", glm::vec4(0, 1, 0, 1));
        unsigned int &VBO = glBuffers[name], &VAO = glVertexArrays[name], &EBO = glBuffers[name + "_ebo"];
        glGenBuffers(1, &VBO);
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &EBO);
        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(camera_points), &camera_points, GL_STATIC_DRAW);

        // Triangles
//            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
//            glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);
        // Lines
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices_line), indices_line, GL_STATIC_DRAW);

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), 0);
        glBindVertexArray(0);
    }
}
void GUI3D::buildGrid(){
    const std::string shaderPath = std::string(GUI_FOLDER_PATH) + "Shaders/";
    /// Grid
    {
        glShaders["grid"] = new glUtil::Shader(shaderPath + "grid.vs",shaderPath + "grid.fs");
        glObjests["Plane"] = (glUtil::Model_base *) new glUtil::Mesh(glUtil::ShapeVertices::plane);
    }
}
void GUI3D::buildText(){
    const std::string shaderPath = std::string(GUI_FOLDER_PATH) + "Shaders/";
    /// Text
    {
        glShaders["Text"] = new glUtil::Shader(shaderPath+ "text.vs",
                                               shaderPath + "text.fs");
        glm::mat4 projection = glm::ortho(0.0f, static_cast<GLfloat>(window_->runtimeWidth), 0.0f,
                                          static_cast<GLfloat>(window_->runtimeHeight));
        glShaders["Text"]->use();
        glShaders["Text"]->set("projection", projection);
        // Configure VAO/VBO for texture quads
        glGenVertexArrays(1, &glVertexArrays["textVAO"]);
        glGenBuffers(1, &glBuffers["textVBO"]);
        glBindVertexArray(glVertexArrays["textVAO"]);
        glBindBuffer(GL_ARRAY_BUFFER, glBuffers["textVBO"]);
        glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 6 * 4, NULL, GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(GLfloat), 0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
    }
}


void GUI3D::buildTrajectory() {
    /// Trajectory
    const std::string shaderPath = std::string(GUI_FOLDER_PATH) + "Shaders/";
    {
        // Camera
        std::string name = "Trajectory";
        glShaders[name] = new glUtil::Shader(shaderPath + "camera_shader.vs",
                                             shaderPath + "camera_shader.fs");
        glShaders[name]->use();
        glShaders[name]->set("color", glm::vec4(0, 1, 0, 1));
        size_t totalSize = 2<<8;
        glGenVertexArrays(1, &glVertexArrays[name]);
        glGenBuffers(1, &glBuffers[name]);
        glBindVertexArray(glVertexArrays[name]);
        glBindBuffer(GL_ARRAY_BUFFER, glBuffers[name]);
        glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * totalSize * 3, NULL, GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), 0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
    }
}


void GUI3D::buildFreeType(){
#ifdef COMPILE_WITH_FREETYPE
    const std::string shaderPath = std::string(GUI_FOLDER_PATH) + "Shaders/";
    const std::string fontPath = std::string(GUI_FOLDER_PATH) + "fonts/";
    /// FreeType
    {
        FT_Library ft;
        // All functions return a value different than 0 whenever an error occurred
        if (FT_Init_FreeType(&ft))
            throw std::runtime_error("ERROR::FREETYPE: Could not init FreeType Library");
        // Load font as face
        FT_Face face;
        if (FT_New_Face(ft, (fontPath + "Montserrat-Regular.ttf").c_str(), 0, &face))
            throw std::runtime_error("ERROR::FREETYPE: Failed to load font");
        // Set size to load glyphs as
        FT_Set_Pixel_Sizes(face, 0, 48);

        // Disable byte-alignment restriction
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

        // Load first 128 characters of ASCII set
        for (GLubyte c = 0; c < 128; c++) {
            // Load character glyph
            if (FT_Load_Char(face, c, FT_LOAD_RENDER)) {
                std::cout << "WARNING::FREETYTPE: Failed to load Glyph" << std::endl;
                continue;
            }
            // Generate texture
            GLuint texture;
            glGenTextures(1, &texture);
            glBindTexture(GL_TEXTURE_2D, texture);
            glTexImage2D(
                    GL_TEXTURE_2D,
                    0,
                    GL_RED,
                    face->glyph->bitmap.width,
                    face->glyph->bitmap.rows,
                    0,
                    GL_RED,
                    GL_UNSIGNED_BYTE,
                    face->glyph->bitmap.buffer
            );
            // Set texture options
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            // Now store character for later use
            Character character = {
                    texture,
                    glm::ivec2(face->glyph->bitmap.width, face->glyph->bitmap.rows),
                    glm::ivec2(face->glyph->bitmap_left, face->glyph->bitmap_top),
                    static_cast<GLuint>(face->glyph->advance.x)
            };
            Characters.insert(std::pair<GLchar, Character>(c, character));
        }
        // Destroy FreeType once we're finished
        FT_Done_Face(face);
        FT_Done_FreeType(ft);
    }
#endif
}

void GUI3D::basicProcess() {
    auto projection = glCam->projection_control_->projection_matrix();

    /// GRID
    if (bShowGrid) {
        glDisable(GL_DEPTH_TEST);
        glUtil::Mesh *Plane = (glUtil::Mesh *) glObjests["Plane"];
        //                glUtil::Shader *shader = glShaders["Plain"];
//            glUtil::Shader *shader = glShaders["Lamp"];
        glUtil::Shader *shader = glShaders["grid"];
        Plane->setShader(shader);
        shader->use();
        glm::mat4 model = glm::mat4(1.f);
        //                model = glm::translate(model, glm::vec3(0, 0, 0));
        model = glm::scale(model, glm::vec3(20.f));// radius (meter)
        shader->set("view", glCam->camera_control_->GetViewMatrix());
        shader->set("projection", projection);
        shader->set("model", model);
        shader->set("color", glm::vec4(0, 0, 0, 0.8));
        shader->set("thickness", 0.01);
        Plane->Draw();


        model = glm::rotate(model, glm::radians(90.f), glm::vec3(1.f, 0.f, 0));
        shader->set("model", model);
        Plane->Draw();

        model = glm::rotate(model, glm::radians(90.f), glm::vec3(0.f, 0.f, 1.0));
        shader->set("model", model);
        Plane->Draw();
        glEnable(GL_DEPTH_TEST);
    }
    /// Draw Text
    if (bShowFPS) {
        glDisable(GL_DEPTH_TEST);
        RenderText("FPS: " + std::to_string((int) std::floor(fps_->getFPS())),
                   10.f, 10.f, 0.5f, glm::vec3(0.5f, 0.8f, 0.2f));
        glEnable(GL_DEPTH_TEST);
    }

    if(bPlotTrajectory) {
        glDisable(GL_DEPTH_TEST);
        plot_trajectory(&projection);
        glEnable(GL_DEPTH_TEST);
    }
}


bool GUI3D::mouseControl(){
    ImGuiIO& io = ImGui::GetIO();
    if(!io.WantCaptureMouse) {
        // let the main canvas handle the mouse input
        return glCam->mouse_control();
    }
    return false;
}

void GUI3D::plot_trajectory(const glm::mat4 *projection){
    const std::string name = "Trajectory";
    if(glShaders.find(name) == glShaders.end())return;
    glUtil::Shader *shader = glShaders[name];
    glm::mat4 modelMat = glm::mat4(1.f);
    shader->use();
    shader->set("lightPos", glCam->camera_control_->Position /*glm::vec3(0,2,0)*/);
    shader->set("model", modelMat);
    shader->set("projection", *projection);
    shader->set("view", glCam->camera_control_->GetViewMatrix());

    glBindVertexArray(glVertexArrays[name]);
    glBindBuffer(GL_ARRAY_BUFFER, glBuffers[name]);
    glBufferSubData(GL_ARRAY_BUFFER, 0,
                    trajectories_.size() * 3 * sizeof(GLfloat), trajectories_.data());
    glDrawArrays(GL_LINE_STRIP, 0, trajectories_.size());
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void GUI3D::add_trajectory(float x, float y, float z, float interval){
    if(std::isnan(x) || std::isinf(x)) return;
    const std::string name = "Trajectory";
    if (trajectories_.empty()) {
        trajectories_.reserve(2<<8);
        trajectories_.emplace_back(x, y, z);
    } else {
        const auto &prev = trajectories_.back();
        glm::vec3 curr(x,y,z);
        if (glm::distance(curr, prev) > interval) {
            trajectories_.push_back(curr);
            if(trajectories_.capacity() - trajectories_.size() < 8) {
                printf("Exceed and reallocate");
                trajectories_.reserve(trajectories_.capacity()*2);

                if (glVertexArrays.find(name) != glVertexArrays.end()) {
                    glDeleteVertexArrays(1, &glVertexArrays.at(name));
                    glDeleteBuffers(1, &glBuffers.at(name));
                }

                glGenVertexArrays(1, &glVertexArrays[name]);
                glGenBuffers(1, &glBuffers[name]);
                glBindVertexArray(glVertexArrays[name]);
                glBindBuffer(GL_ARRAY_BUFFER, glBuffers[name]);
                glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * trajectories_.capacity()*3, NULL, GL_DYNAMIC_DRAW);
                glEnableVertexAttribArray(0);
                glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), 0);
                glBindBuffer(GL_ARRAY_BUFFER, 0);
                glBindVertexArray(0);
            }
        }
    }
}

void GUI3D::RenderText(GLuint VAO, GLuint VBO, glUtil::Shader *shader, std::string text, GLfloat x, GLfloat y, GLfloat scale, glm::vec3 color) {

// Activate corresponding render state
    shader->use();
    shader->set("textColor", glm::vec3(color.x, color.y, color.z));
//        glUniform3f(glGetUniformLocation(shader->Program, "textColor"), color.x, color.y, color.z);
    glActiveTexture(GL_TEXTURE0);
    glBindVertexArray(VAO);

// Iterate through all characters
    std::string::const_iterator c;
    for (c = text.begin(); c != text.end(); c++) {
        Character ch = Characters[*c];

        GLfloat xpos = x + ch.Bearing.x * scale;
        GLfloat ypos = y - float(ch.Size.y - ch.Bearing.y) * scale;

        GLfloat w = ch.Size.x * scale;
        GLfloat h = ch.Size.y * scale;
        // Update VBO for each character
        GLfloat vertices[6][4] = {
                {xpos,     ypos + h, 0.0, 0.0},
                {xpos,     ypos,     0.0, 1.0},
                {xpos + w, ypos,     1.0, 1.0},

                {xpos,     ypos + h, 0.0, 0.0},
                {xpos + w, ypos,     1.0, 1.0},
                {xpos + w, ypos + h, 1.0, 0.0}
        };
        // Render glyph texture over quad
        glBindTexture(GL_TEXTURE_2D, ch.TextureID);
        // Update content of VBO memory
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices),
                        vertices); // Be sure to use glBufferSubData and not glBufferData

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        // Render quad
        glDrawArrays(GL_TRIANGLES, 0, 6);
        // Now advance cursors for next glyph (note that advance is number of 1/64 pixels)
        x += (ch.Advance >> 6) *
             scale; // Bitshift by 6 to get value in pixels (2^6 = 64 (divide amount of 1/64th pixels by 64 to get amount of pixels))
    }
    glBindVertexArray(0);
    glBindTexture(GL_TEXTURE_2D, 0);
}

void GUI3D::showRegisteredKeyFunction(){
    for(auto func:registeredFunctions_)
        printf("window[%s], key[%d], function[%p], description[%s]\n", func.window_->name_.c_str(), func.key_, &func.no_id, func.description.c_str());
}


void GUI3D::cameraUI() {
    if(!bShowCameraUI) return;
//    ImGui::Begin("Projection control", &bShowCameraUI, ImGuiWindowFlags_AlwaysAutoResize);
//    ImGui::DragFloat("Near",camera_control->Position)
//    ImGui::DragFloat3("Position", &camera_control->Position[0]);
//    ImGui::End();
}

void GUI3D::RenderText(const std::string &text, float pos_x, float pos_y, float scale, const glm::vec3 &color) {
    RenderText(glVertexArrays["textVAO"], glBuffers["textVBO"], glShaders["Text"],
               text, pos_x,pos_y,scale,color);
}
