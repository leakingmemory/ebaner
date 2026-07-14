cmake_minimum_required(VERSION 3.20)
project(ebaner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# ---- Dependencies ---------------------------------------------------------
find_package(Vulkan REQUIRED)

# GLFW: prefer the CMake package, fall back to pkg-config.
find_package(glfw3 QUIET)
if(NOT glfw3_FOUND)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(GLFW REQUIRED IMPORTED_TARGET glfw3)
    set(GLFW_TARGET PkgConfig::GLFW)
else()
    set(GLFW_TARGET glfw)
endif()

# GLM: header-only. Package if present, otherwise rely on system include dir.
find_package(glm QUIET)

# ---- Shader compilation ---------------------------------------------------
find_program(GLSLC glslc HINTS ${Vulkan_GLSLC_EXECUTABLE})
if(NOT GLSLC)
    message(FATAL_ERROR "glslc not found; required to compile shaders")
endif()

set(SHADER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/shaders)
set(SHADER_OUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/shaders)
file(MAKE_DIRECTORY ${SHADER_OUT_DIR})

set(SHADER_SPV)
foreach(shader terrain.vert terrain.frag track.vert track.frag)
    set(src ${SHADER_DIR}/${shader})
    set(spv ${SHADER_OUT_DIR}/${shader}.spv)
    add_custom_command(
        OUTPUT ${spv}
        COMMAND ${GLSLC} ${src} -o ${spv}
        DEPENDS ${src}
        COMMENT "Compiling shader ${shader}"
        VERBATIM)
    list(APPEND SHADER_SPV ${spv})
endforeach()
add_custom_target(shaders DEPENDS ${SHADER_SPV})

# ---- Executable -----------------------------------------------------------
add_executable(ebaner
    src/main.cpp
    src/TerrainData.cpp
    src/TerrainMesh.cpp
    src/Camera.cpp
    src/Textures.cpp
    src/TrackPath.cpp
    src/TrackMesh.cpp
    src/VulkanRenderer.cpp)

add_dependencies(ebaner shaders)

target_link_libraries(ebaner PRIVATE Vulkan::Vulkan ${GLFW_TARGET})
if(glm_FOUND)
    target_link_libraries(ebaner PRIVATE glm::glm)
endif()

# Directory holding the compiled .spv files, exposed to the app at runtime.
target_compile_definitions(ebaner PRIVATE
    SHADER_DIR="${SHADER_OUT_DIR}")
