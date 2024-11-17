//
// Created by sc on 1/28/21.
//

#ifndef LIBGUI3D_LOCALFILESYSTEM_H
#define LIBGUI3D_LOCALFILESYSTEM_H
#include "tinyfiledialogs.h"
#include "imgui.h"
#include <vector>
namespace ImGui {
    /**
     * Add a button to open the local file dialog.
     * @param label The label of the button
     * @param output_buffer butter to store the output file path
     * @param buffer_size buffer size
     * @param window_title The title of the opened window
     * @param filter_patterns The default pattern of the files you are looking for (e.g. for obj -> *.obj)
     */
    IMGUI_API bool FileSelectButton(const char *label, char *output_buffer, size_t buffer_size, const char *window_title = "Select File", const std::vector<const char*>& filter_patterns = {"*"});

    IMGUI_API bool FolderSelectButton(const char *label, char *output_buffer, size_t buffer_size, const char *window_title = "Select File");
}

#endif //LIBGUI3D_LOCALFILESYSTEM_H
