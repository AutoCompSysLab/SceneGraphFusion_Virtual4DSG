find_package(assimp QUIET)
OPTION(WITH_ASSIMP "Build with Assimp support?" ${assimp_FOUND})

IF(WITH_ASSIMP)
    MESSAGE(STATUS "WITH Assimp")
    find_package(assimp REQUIRED)
    INCLUDE_DIRECTORIES(${assimp_INCLUDE_DIRS})
ENDIF()