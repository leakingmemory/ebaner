# ebaner - a Vulkan viewer for terrainmapper rail/terrain exports.
# Copyright (C) 2026 Jan-Espen Oversand <sigsegv@radiotube.org>
#
# This file is part of ebaner. ebaner is free software: you can redistribute it
# and/or modify it under the terms of version 3 of the GNU General Public License
# as published by the Free Software Foundation. ebaner is distributed WITHOUT ANY
# WARRANTY; see the GNU General Public License for details. You should have
# received a copy of the license along with ebaner; if not, see
# <https://www.gnu.org/licenses/>.

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

# Audio (optional): procedural brake sounds. Prefer PulseAudio (talks to the sound
# server directly, like other desktop apps); fall back to PortAudio. The build stays
# silent if neither is present.
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(PULSE QUIET IMPORTED_TARGET libpulse-simple)
    pkg_check_modules(PORTAUDIO QUIET IMPORTED_TARGET portaudio-2.0)
endif()

# ---- Shader compilation ---------------------------------------------------
find_program(GLSLC glslc HINTS ${Vulkan_GLSLC_EXECUTABLE})
if(NOT GLSLC)
    message(FATAL_ERROR "glslc not found; required to compile shaders")
endif()

set(SHADER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/shaders)
set(SHADER_OUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/shaders)
file(MAKE_DIRECTORY ${SHADER_OUT_DIR})

set(SHADER_SPV)
foreach(shader terrain.vert terrain.frag track.vert track.frag text.vert text.frag
               overlay.vert overlay_line.frag overlay_point.frag)
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

# ---- Shared engine library ------------------------------------------------
# The rendering/loading engine, shared by the `ebaner` viewer and the
# `ebaner-trackedit` editor. App-specific code (vehicle sim, audio, driver cam)
# lives in the individual executables.
add_library(ebaner_engine STATIC
    src/TerrainData.cpp
    src/TerrainCarve.cpp
    src/TerrainMesh.cpp
    src/Camera.cpp
    src/Textures.cpp
    src/TrackPath.cpp
    src/TrackMesh.cpp
    src/TrackGraph.cpp
    src/TrackOverlay.cpp
    src/TrackCircuits.cpp
    src/RoadMesh.cpp
    src/BuildingMesh.cpp
    src/PlatformMesh.cpp
    src/SwitchMesh.cpp
    src/SwitchNetwork.cpp
    src/EarClip.cpp
    src/Font.cpp
    src/VulkanRenderer.cpp)

add_dependencies(ebaner_engine shaders)

target_link_libraries(ebaner_engine PUBLIC Vulkan::Vulkan ${GLFW_TARGET})
if(glm_FOUND)
    target_link_libraries(ebaner_engine PUBLIC glm::glm)
endif()

# Directory holding the compiled .spv files, exposed to apps at runtime.
target_compile_definitions(ebaner_engine PUBLIC
    SHADER_DIR="${SHADER_OUT_DIR}")

# ---- ebaner (viewer + vehicle sim) ----------------------------------------
add_executable(ebaner
    src/main.cpp
    src/VehicleMesh.cpp
    src/Vehicle.cpp
    src/Audio.cpp)
add_dependencies(ebaner shaders)
target_link_libraries(ebaner PRIVATE ebaner_engine)
if(PULSE_FOUND)
    target_link_libraries(ebaner PRIVATE PkgConfig::PULSE)
    target_compile_definitions(ebaner PRIVATE HAVE_PULSE=1)
    message(STATUS "PulseAudio found: brake sounds via PulseAudio (preferred)")
endif()
if(PORTAUDIO_FOUND)
    target_link_libraries(ebaner PRIVATE PkgConfig::PORTAUDIO)
    target_compile_definitions(ebaner PRIVATE HAVE_PORTAUDIO=1)
    message(STATUS "PortAudio found: brake sounds via PortAudio (fallback)")
endif()
if(NOT PULSE_FOUND AND NOT PORTAUDIO_FOUND)
    message(STATUS "No audio backend found: building without sound")
endif()

# ---- ebaner-trackedit (track-network viewer / editor) ---------------------
add_executable(ebaner-trackedit
    src/trackedit_main.cpp)
add_dependencies(ebaner-trackedit shaders)
target_link_libraries(ebaner-trackedit PRIVATE ebaner_engine)
