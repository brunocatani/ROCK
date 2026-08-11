cmake_minimum_required(VERSION 4.2)

set(ROCK_SDK_EXAMPLES_STANDALONE OFF)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(ROCK_SDK_EXAMPLES_STANDALONE ON)
  project(ROCKSDKExamples VERSION 0.1.0 LANGUAGES CXX)

  macro(rock_sdk_set_from_environment variable_name)
    if(NOT DEFINED ${variable_name} AND DEFINED ENV{${variable_name}})
      set(${variable_name} "$ENV{${variable_name}}")
    endif()
  endmacro()

  rock_sdk_set_from_environment(VCPKG_ROOT)
  if(NOT DEFINED VCPKG_ROOT OR VCPKG_ROOT STREQUAL "")
    message(FATAL_ERROR "VCPKG_ROOT is required for a standalone SDK example build")
  endif()
  set(CMAKE_TOOLCHAIN_FILE
      "${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
      CACHE STRING "")
  set(VCPKG_TARGET_TRIPLET "x64-windows-static" CACHE STRING "")
  set(CMAKE_MSVC_RUNTIME_LIBRARY
      "MultiThreaded$<$<CONFIG:Debug>:Debug>"
      CACHE STRING "")

  rock_sdk_set_from_environment(COMMON_LIB_F4VR_PATH)
  if(NOT DEFINED COMMON_LIB_F4VR_PATH OR COMMON_LIB_F4VR_PATH STREQUAL "")
    message(FATAL_ERROR
      "COMMON_LIB_F4VR_PATH is required for a standalone SDK example build")
  endif()

  set(F4SE_SUPPORT_XBYAK ON CACHE BOOL "" FORCE)
  set(ENABLE_FALLOUT_NG OFF CACHE BOOL "" FORCE)
  set(ENABLE_FALLOUT_F4 OFF CACHE BOOL "" FORCE)
  set(ENABLE_FALLOUT_VR ON CACHE BOOL "" FORCE)
  add_subdirectory("${COMMON_LIB_F4VR_PATH}" "CommonLibF4" EXCLUDE_FROM_ALL)
endif()

set(ROCK_SDK_ROOT "${CMAKE_CURRENT_LIST_DIR}/..")
set(ROCK_SDK_EXAMPLE_COMMON
  "${CMAKE_CURRENT_LIST_DIR}/common/ExampleRuntime.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/common/ExampleRuntime.h"
)

include("${CMAKE_CURRENT_LIST_DIR}/cmake/VerifyFo4VrLoader.cmake")
rock_sdk_verify_fo4vr_loader(
  "${CMAKE_CURRENT_LIST_DIR}/common/ExampleRuntime.cpp")

find_package(spdlog CONFIG REQUIRED)

function(add_rock_sdk_example target_name source_path)
  add_library(${target_name} SHARED
    ${ROCK_SDK_EXAMPLE_COMMON}
    "${source_path}"
  )
  target_compile_features(${target_name} PRIVATE cxx_std_23)
  target_compile_definitions(${target_name} PRIVATE
    UNICODE
    _UNICODE
    WIN32_LEAN_AND_MEAN
    NOMINMAX
    NOMMNOSOUND
  )
  target_include_directories(${target_name} PRIVATE
    "${ROCK_SDK_ROOT}/include"
    "${CMAKE_CURRENT_LIST_DIR}/common"
  )
  target_link_libraries(${target_name} PRIVATE
    CommonLibF4::CommonLibF4
    spdlog::spdlog
  )
  target_compile_options(${target_name} PRIVATE
    /sdl
    /utf-8
    /permissive-
    /Zc:__cplusplus
    /Zc:preprocessor
    /W4
    "$<$<CONFIG:RELEASE>:/Zc:inline;/JMC-;/Ob3>"
  )
  target_link_options(${target_name} PRIVATE
    "$<$<CONFIG:DEBUG>:/INCREMENTAL;/OPT:NOREF;/OPT:NOICF>"
    "$<$<CONFIG:RELEASE>:/INCREMENTAL:NO;/OPT:REF;/OPT:ICF;/DEBUG:FULL>"
  )
  set_target_properties(${target_name} PROPERTIES
    PREFIX ""
    FOLDER "ROCK SDK Examples"
  )
endfunction()

add_rock_sdk_example(
  ROCKSDKHandStateMonitor
  "${CMAKE_CURRENT_LIST_DIR}/mods/HandStateMonitor.cpp")
add_rock_sdk_example(
  ROCKSDKWeaponInspector
  "${CMAKE_CURRENT_LIST_DIR}/mods/WeaponInspector.cpp")
add_rock_sdk_example(
  ROCKSDKSurfaceClimber
  "${CMAKE_CURRENT_LIST_DIR}/mods/SurfaceClimber.cpp")
add_rock_sdk_example(
  ROCKSDKContactVisualizer
  "${CMAKE_CURRENT_LIST_DIR}/mods/ContactVisualizer.cpp")

add_custom_target(ROCKSDKExamplePlugins)
add_dependencies(ROCKSDKExamplePlugins
  ROCKSDKHandStateMonitor
  ROCKSDKWeaponInspector
  ROCKSDKSurfaceClimber
  ROCKSDKContactVisualizer
)
