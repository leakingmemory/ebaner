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

# So clangd can resolve includes outside src/ (the tests) without guessing.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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
    # Lua (optional): behaviour authored alongside the data, in overlay/overlay.lua.
    #
    # pkg_search_module, not pkg_check_modules: it takes the first module that exists,
    # where pkg_check_modules would demand all of them. The names have to be versioned -
    # a distribution may ship lua5.4.pc with no lua.pc, no /usr/include/lua.h and no
    # liblua.so at all, which is enough to defeat both find_package(Lua) and a search for
    # a bare "lua". luajit is deliberately not in the list: it would answer a search for
    # "some Lua" and quietly give 5.1 semantics instead of 5.4.
    pkg_search_module(LUA QUIET IMPORTED_TARGET lua5.4 lua-5.4 lua54)
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
    src/Streaming.cpp
    src/Stations.cpp
    src/StationPicker.cpp
    src/SimpleEntrySignals.cpp
    src/LevelCrossings.cpp
    src/CrossingMesh.cpp
    src/FlagPosts.cpp
    src/FlagMesh.cpp
    src/AvalancheSignals.cpp
    src/AvalancheMesh.cpp
    src/TxpPositions.cpp
    src/TxpMesh.cpp
    src/TxpGraph.cpp
    src/TxpNetwork.cpp
    src/SignalPaths.cpp
    src/LineBlock.cpp
    src/Occupancy.cpp
    src/SignalMesh.cpp
    src/SpeedLimits.cpp
    src/SpeedSignMesh.cpp
    src/TunnelMesh.cpp
    src/RoadMesh.cpp
    src/BuildingMesh.cpp
    src/PlatformMesh.cpp
    src/SwitchMesh.cpp
    src/SwitchNetwork.cpp
    src/SwitchTypes.cpp
    src/EarClip.cpp
    src/Font.cpp
    src/Script.cpp
    src/VulkanRenderer.cpp)

add_dependencies(ebaner_engine shaders)

target_link_libraries(ebaner_engine PUBLIC Vulkan::Vulkan ${GLFW_TARGET})
if(glm_FOUND)
    target_link_libraries(ebaner_engine PUBLIC glm::glm)
endif()
# PRIVATE: no Lua header escapes Script.cpp, so nothing that uses the engine needs the
# include path. CMake still carries the link requirement through for a static library.
if(LUA_FOUND)
    target_link_libraries(ebaner_engine PRIVATE PkgConfig::LUA)
    target_compile_definitions(ebaner_engine PRIVATE HAVE_LUA=1)
    message(STATUS "Lua found (${LUA_VERSION}): overlay.lua scripting enabled")
else()
    message(STATUS "No Lua found: building without overlay.lua scripting")
endif()

# Directory holding the compiled .spv files, exposed to apps at runtime.
target_compile_definitions(ebaner_engine PUBLIC
    SHADER_DIR="${SHADER_OUT_DIR}")

# ---- ebaner (viewer + vehicle sim) ----------------------------------------
add_executable(ebaner
    src/main.cpp
    src/VehicleMesh.cpp
    src/Vehicle.cpp
    src/Consist.cpp
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

add_executable(ebaner-dumptrack
    tools/dumptrack_main.cpp)
add_dependencies(ebaner-dumptrack shaders)
target_link_libraries(ebaner-dumptrack PRIVATE ebaner_engine)
target_include_directories(ebaner-dumptrack PRIVATE src)

# ---- tests ----------------------------------------------------------------
# They run against a real terrainmapper export, since everything here is shaped by
# what the data is actually like; there is no synthetic dataset to check against.
# A test skips (exit 77) rather than fails when the export is not there, so a build
# without one is still clean.
set(EBANER_TEST_DATASET "${CMAKE_SOURCE_DIR}/../norway-rails"
    CACHE PATH "terrainmapper export the tests run against")

enable_testing()
# Vehicle/Consist: the turnout-knock check runs a real train over a real turnout, and
# the consist checks couple two sets and drive them.
add_executable(TrackEditTest tests/TrackEditTest.cpp src/Vehicle.cpp src/Consist.cpp)
add_dependencies(TrackEditTest shaders)
target_link_libraries(TrackEditTest PRIVATE ebaner_engine)
target_include_directories(TrackEditTest PRIVATE src)
add_test(NAME track-edit COMMAND TrackEditTest "${EBANER_TEST_DATASET}")
set_tests_properties(track-edit PROPERTIES SKIP_RETURN_CODE 77)

# A block-length signal route: needs the real network, so it skips without one.
add_executable(SignalRouteTest tests/SignalRouteTest.cpp)
add_dependencies(SignalRouteTest shaders)
target_link_libraries(SignalRouteTest PRIVATE ebaner_engine)
target_include_directories(SignalRouteTest PRIVATE src)
add_test(NAME signal-route COMMAND SignalRouteTest "${EBANER_TEST_DATASET}")
set_tests_properties(signal-route PROPERTIES SKIP_RETURN_CODE 77)

# The line block between two stations: the block signals and the direction lock. Needs the
# real geometry for the roads, but synthesises the block signals themselves, so it does not
# wait on anything being committed to the overlay repository.
add_executable(LineBlockTest tests/LineBlockTest.cpp)
add_dependencies(LineBlockTest shaders)
target_link_libraries(LineBlockTest PRIVATE ebaner_engine)
target_include_directories(LineBlockTest PRIVATE src)
add_test(NAME line-block COMMAND LineBlockTest "${EBANER_TEST_DATASET}")
set_tests_properties(line-block PROPERTIES SKIP_RETURN_CODE 77)

# The track/fraction bridge: authored (track, frac) <-> path arc length, both ways, at every
# border in the dataset. Needs the real geometry, so it skips without it.
add_executable(TrackBridgeTest tests/TrackBridgeTest.cpp)
add_dependencies(TrackBridgeTest shaders)
target_link_libraries(TrackBridgeTest PRIVATE ebaner_engine)
target_include_directories(TrackBridgeTest PRIVATE src)
add_test(NAME track-bridge COMMAND TrackBridgeTest "${EBANER_TEST_DATASET}")
set_tests_properties(track-bridge PROPERTIES SKIP_RETURN_CODE 77)

# Track circuits decided by overlap rather than by proximity: sections and trains as
# stretches of road. Needs the real geometry, so it skips without it.
add_executable(OccupancyTest tests/OccupancyTest.cpp)
add_dependencies(OccupancyTest shaders)
target_link_libraries(OccupancyTest PRIVATE ebaner_engine)
target_include_directories(OccupancyTest PRIVATE src)
add_test(NAME occupancy COMMAND OccupancyTest "${EBANER_TEST_DATASET}")
set_tests_properties(occupancy PROPERTIES SKIP_RETURN_CODE 77)

# What a real train hands the circuits: the stretches of road under its body, across
# turnouts, at a stand, and after being parted in two.
add_executable(TrainSpanTest tests/TrainSpanTest.cpp src/Vehicle.cpp src/Consist.cpp)
add_dependencies(TrainSpanTest shaders)
target_link_libraries(TrainSpanTest PRIVATE ebaner_engine)
target_include_directories(TrainSpanTest PRIVATE src)
add_test(NAME train-span COMMAND TrainSpanTest "${EBANER_TEST_DATASET}")
set_tests_properties(train-span PROPERTIES SKIP_RETURN_CODE 77)

# What deciding every signal's aspect costs. It runs inside a frame on every circuit
# transition, so this is the net that catches it going back to walking the whole network.
add_executable(SignallingCostTest tests/SignallingCostTest.cpp)
add_dependencies(SignallingCostTest shaders)
target_link_libraries(SignallingCostTest PRIVATE ebaner_engine)
target_include_directories(SignallingCostTest PRIVATE src)
add_test(NAME signalling-cost COMMAND SignallingCostTest "${EBANER_TEST_DATASET}")
set_tests_properties(signalling-cost PROPERTIES SKIP_RETURN_CODE 77)

# The crossing sequence is pure logic and needs no export, so it always runs.
add_executable(CrossingSequenceTest tests/CrossingSequenceTest.cpp)
add_dependencies(CrossingSequenceTest shaders)
target_link_libraries(CrossingSequenceTest PRIVATE ebaner_engine)
target_include_directories(CrossingSequenceTest PRIVATE src)
# The writable dir is where the overlay round-trip writes its scratch file; without one
# the test still runs, just without that part.
add_test(NAME crossing-sequence
         COMMAND CrossingSequenceTest "${CMAKE_CURRENT_BINARY_DIR}/crossing-test")

# The rolling-noise synth. Audio.cpp builds into the viewer rather than the engine
# library, so it is compiled in here directly - and deliberately without HAVE_PULSE /
# HAVE_PORTAUDIO, which leaves it pure DSP that opens no device. What the test measures
# is the shipped synth, not a copy of it. Needs no export.
# Vehicle.cpp comes along because Audio::update reads the vehicle; the test drives the
# synth through setRolling and never builds one, but the symbols have to resolve.
add_executable(RollingNoiseTest tests/RollingNoiseTest.cpp src/Audio.cpp src/Vehicle.cpp
               src/Consist.cpp)
add_dependencies(RollingNoiseTest shaders)
target_link_libraries(RollingNoiseTest PRIVATE ebaner_engine)
target_include_directories(RollingNoiseTest PRIVATE src)
add_test(NAME rolling-noise COMMAND RollingNoiseTest)

# Uncoupling: parting a train at a coupler and the hold both portions come away with.
# Needs no dataset and no export - TrackPath takes points directly, so a kilometre of
# straight and a kilometre of 2 % falling grade are the whole world it works in.
add_executable(UncoupleTest tests/UncoupleTest.cpp src/Vehicle.cpp src/Consist.cpp)
add_dependencies(UncoupleTest shaders)
target_link_libraries(UncoupleTest PRIVATE ebaner_engine)
target_include_directories(UncoupleTest PRIVATE src)
add_test(NAME uncouple COMMAND UncoupleTest)
