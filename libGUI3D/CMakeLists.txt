cmake_minimum_required(VERSION 3.13)
project(libGUI3D)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

OPTION(BUILD_DEMO "Build demo" OFF)

#################################
# Specify the CMake module path #
#################################
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})

#############
## External #
#############
#INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseCxxTools.cmake)

######################
# Add subdirectories #
######################
ADD_SUBDIRECTORY(ImGuiExtension)
ADD_SUBDIRECTORY(libGUI)
ADD_SUBDIRECTORY(libGUI3D)

IF(BUILD_DEMO)
    ADD_EXECUTABLE(demo demo.cpp)
    TARGET_LINK_LIBRARIES(demo GUI3D)
ENDIF()