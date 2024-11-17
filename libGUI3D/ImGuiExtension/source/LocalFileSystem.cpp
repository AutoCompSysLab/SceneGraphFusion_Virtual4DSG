//
// Created by sc on 1/28/21.
//
#include <ImGuiExtension/LocalFileSystem.h>
#include <vector>
//using namespace ImGui;

//bool ui_file_open(const char *label, const char *btn, char *buf, size_t buf_size, const char *title,
//                  int filter_num, const char *const *filter_patterns)
//{
//    bool ret = ImGui::InputText(label, buf, buf_size);
//    ImGui::SameLine();
//
//    if(ImGui::Button(btn))
//    {
//        const char *filename = tinyfd_openFileDialog(title, "", filter_num, filter_patterns,
//                                                     nullptr, false);
//        if(filename) strcpy(buf, filename);
//        ret = true;
//    }
//    return ret;
//}


bool ImGui::FileSelectButton(const char *label, char *output_buffer, size_t buffer_size, const char *window_title, const std::vector<const char*>& filter_patterns){
    if(ImGui::Button(label))
    {
        const char *filename = tinyfd_openFileDialog(window_title, "", filter_patterns.size(), filter_patterns.data(),
                                                     nullptr, false);
        if(filename) strcpy(output_buffer, filename);
        return true;
    }
    return false;

//    ImGui::Begin("File system");
//    constexpr size_t kFilenameBufSize = 512;
//    static char name_buf[kFilenameBufSize];
//    constexpr const char *kFilter[] = {"*"};
//    ui_file_open("Filename", "...##5", name_buf, kFilenameBufSize, "Filename",1, kFilter);
//    ImGui::End();
}


bool ImGui::FolderSelectButton(const char *label, char *output_buffer, size_t buffer_size, const char *window_title) {
    if (ImGui::Button(label)) {
        const char *filename = tinyfd_selectFolderDialog(window_title, "");
        if (filename) strcpy(output_buffer, filename);
        return true;
    }
    return false;

//    ImGui::Begin("File system");
//    constexpr size_t kFilenameBufSize = 512;
//    static char name_buf[kFilenameBufSize];
//    constexpr const char *kFilter[] = {"*"};
//    ui_file_open("Filename", "...##5", name_buf, kFilenameBufSize, "Filename",1, kFilter);
//    ImGui::End();
}