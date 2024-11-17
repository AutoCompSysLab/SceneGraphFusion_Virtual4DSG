# GUI3D #
#########
find_package(GLM REQUIRED)
FIND_PACKAGE(FreeType2 REQUIRED)
FIND_PACKAGE(Eigen3)

SET(sources
        GUI3D.cpp
        glUtils.cpp
        projection_control.cpp
        PointsRenderer.cpp
        ImageDrawer.cpp
        )
SET(headers
        GUI3D.h
        glShader.hpp
        glCamera.hpp
        glUtils.hpp
        PointsRenderer.h
        )

ADD_LIBRARY(GUI3D ${sources} ${headers})
target_link_libraries(GUI3D
        PUBLIC GUI
        )
TARGET_INCLUDE_DIRECTORIES(GUI3D
        PUBLIC ${glm_INCLUDE_DIRS}
        PUBLIC ./stb
        )
IF(FreeType2_FOUND)
    message("FreeType2_LIBRARIES: " ${FreeType2_LIBRARIES})
    TARGET_INCLUDE_DIRECTORIES(GUI3D PUBLIC ${FreeType2_INCLUDE_DIRS})
    TARGET_LINK_LIBRARIES(GUI3D PUBLIC ${FreeType2_LIBRARIES} )
    TARGET_LINK_LIBRARIES(GUI3D PUBLIC freetype)
    target_compile_definitions(GUI3D PUBLIC -DCOMPILE_WITH_FREETYPE)
ENDIF()

IF(TARGET Eigen3::Eigen)
    GET_TARGET_PROPERTY(EIGEN3_INCLUDE_DIR Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
ENDIF()
IF(EIGEN3_INCLUDE_DIR)
    TARGET_COMPILE_DEFINITIONS(GUI3D PUBLIC -DCOMPILE_WITH_EIGEN)
    TARGET_INCLUDE_DIRECTORIES(GUI3D PUBLIC ${EIGEN3_INCLUDE_DIR})
ENDIF()

TARGET_COMPILE_DEFINITIONS(GUI3D PUBLIC GUI_FOLDER_PATH="${CMAKE_CURRENT_SOURCE_DIR}/")