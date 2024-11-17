# Find ImGUI
# Use ImGUI as an external project
#
###
SET(NAME ImGUI)
SET(URL "https://github.com/ocornut/imgui.git")
SET(${NAME}_INSTALL_DIR  ${CMAKE_BINARY_DIR}/external/${NAME})

IF(NOT EXISTS ${CMAKE_BINARY_DIR}/external/${NAME}/.github)
    IF(EXISTS ${${NAME}_INSTALL_DIR})
        file(REMOVE_RECURSE ${${NAME}_INSTALL_DIR})
    ENDIF()
    MESSAGE("cloning ${NAME}..." )
    find_package(Git)
    #include(FindPythonInterp)
    file(MAKE_DIRECTORY ${${NAME}_INSTALL_DIR})
    execute_process(
            COMMAND ${GIT_EXECUTABLE} clone ${URL} ${NAME}
            WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/external
    )
ENDIF()

if (NOT TARGET ${NAME}-update)
    add_custom_target(${NAME}-update
            COMMENT "Updated ${NAME}"
            WORKING_DIRECTORY ${${NAME}_INSTALL_DIR}
            COMMAND ${GIT_EXECUTABLE} pull
            #DEPENDS ${NAME}
            )
ENDIF()

#SET(NAME ImGUIFileDialog)
#SET(URL "https://github.com/ShunChengWu/ImGuiFileDialog.git")
#SET(${NAME}_INSTALL_DIR  ${CMAKE_BINARY_DIR}/external/${NAME})
#SET(${NAME}_DOWNLOADED 0)
#IF(NOT ${NAME}_DOWNLOADED)
#    find_package(Git)
#    include(FindPythonInterp)
#    file(MAKE_DIRECTORY ${${NAME}_INSTALL_DIR})
#    execute_process(
#            COMMAND ${GIT_EXECUTABLE} clone ${URL} ${NAME}
#            WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/external
#    )
#    SET(${NAME}_DOWNLOADED 1 CACHE STRING "Set to 1 if imgui is found, 0 otherwise")
#ENDIF()
##message("DIR: " ${${NAME}_INSTALL_DIR})
#add_custom_command(OUTPUT ${NAME}_update
#        COMMAND ${GIT_EXECUTABLE} reset --hard
#        COMMAND ${GIT_EXECUTABLE} pull
#        WORKING_DIRECTORY ${${NAME}_INSTALL_DIR}
#        )
#
#add_custom_target(${NAME}-update
#        COMMENT "Updated ${NAME}"
#        #WORKING_DIRECTORY ${${NAME}_INSTALL_DIR}
#        #COMMAND ${GIT_EXECUTABLE} reset --hard; ${GIT_EXECUTABLE} pull
#        DEPENDS ${NAME}_update
#        )

if (NOT TARGET imgui)
    SET(OpenGL_GL_PREFERENCE GLVND)
    find_package(OpenGL REQUIRED)
    FIND_PACKAGE(gl3w REQUIRED)
    find_package(glfw3 REQUIRED)
    find_package( Threads REQUIRED)
    message("gl3w_SOURCE: " ${gl3w_SOURCE})
    add_library(imgui STATIC
            ${gl3w_SOURCE}
    #        ${ImGUIFileDialog_INSTALL_DIR}/ImGuiFileDialog/ImGuiFileDialog.cpp
            ${ImGUI_INSTALL_DIR}/imgui.cpp
            ${ImGUI_INSTALL_DIR}/imgui_demo.cpp
            ${ImGUI_INSTALL_DIR}/imgui_draw.cpp
            ${ImGUI_INSTALL_DIR}/imgui_tables.cpp
            ${ImGUI_INSTALL_DIR}/imgui_widgets.cpp
            ${ImGUI_INSTALL_DIR}/backends/imgui_impl_glfw.cpp
            ${ImGUI_INSTALL_DIR}/backends/imgui_impl_opengl3.cpp
            )
    #message("gl3w_INCLUDE_DIRS: " ${gl3w_INCLUDE_DIRS})
    target_include_directories(imgui
            PUBLIC ${gl3w_INCLUDE_DIRS}
            PUBLIC ${ImGUI_INSTALL_DIR}
            PUBLIC ${ImGUI_INSTALL_DIR}/backends/
    #        PUBLIC ${ImGUIFileDialog_INSTALL_DIR}/ImGuiFileDialog/
            )

    target_link_libraries(imgui
            PUBLIC ${OPENGL_LIBRARIES}
            PUBLIC glfw
            PUBLIC dl
            PUBLIC ${CMAKE_THREAD_LIBS_INIT}
            )

    #set_target_properties(imgui PROPERTIES LINKER_LANGUAGE C LINKER_LANGUAGE CXX)
    #set_property(TARGET imgui PROPERTY C_STANDARD 11 CXX_STANDARD 11)
    target_compile_definitions(imgui PUBLIC -DIMGUI_IMPL_OPENGL_LOADER_GL3W)

    if (WIN32)
        TARGET_COMPILE_DEFINITIONS(imgui PUBLIC WIN32)
    endif (WIN32)
    if (UNIX)
        TARGET_COMPILE_DEFINITIONS(imgui PUBLIC LINUX)
        TARGET_COMPILE_DEFINITIONS(imgui PUBLIC UNIX)
    endif (UNIX)

    IF (APPLE)
        TARGET_COMPILE_DEFINITIONS(imgui PUBLIC APPLE)
    endif(APPLE)
ENDIF()