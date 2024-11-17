# GUI #
#######
INCLUDE(../cmake/UseImGUI.cmake)

SET(sources
        GUI.cpp
        ../ImGuiExtension/source/tinyfiledialogs.c
        )
SET(headers
        GUI.h
        ../ImGuiExtension/include/ImGuiExtension/tinyfiledialogs.h
        )

ADD_LIBRARY(GUI ${sources} ${headers})
target_link_libraries(GUI
        PUBLIC ImGuiExtension
)



OPTION(BUILD_TEST_GUI "Build a test programe on GUI" False)
IF(BUILD_TEST_GUI)
    ADD_SUBDIRECTORY(TEST)
ENDIF()
