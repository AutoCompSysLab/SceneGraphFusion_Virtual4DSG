INCLUDE(../cmake/UseImGUI.cmake)

SET(sources
        source/LocalFileSystem.cpp
        source/tinyfiledialogs.c
        )
SET(headers
        include/ImGuiExtension/LocalFileSystem.h
        include/ImGuiExtension/tinyfiledialogs.h
        )

ADD_LIBRARY(ImGuiExtension ${sources} ${headers})
target_link_libraries(ImGuiExtension PUBLIC imgui)
target_include_directories(ImGuiExtension PUBLIC include)