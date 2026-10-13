cmake_minimum_required(VERSION 4.2)


set(NAME "ROCK")
set(FRIENDLY_NAME "ROCK")
set(VERSION 0.92.2)

message(">>> Start build for '${NAME}' v${VERSION}")

set(ROOT_DIR  "${CMAKE_CURRENT_SOURCE_DIR}")
set(SOURCE_DIR "${ROOT_DIR}/src")
set(BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}")
get_filename_component(WORKSPACE_ROOT "${ROOT_DIR}/../.." ABSOLUTE)

if(PROJECT_SOURCE_DIR STREQUAL PROJECT_BINARY_DIR)
  message(FATAL_ERROR ">> In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there.")
endif()

macro(set_from_environment VARIABLE)
  if(NOT DEFINED ${VARIABLE} AND DEFINED ENV{${VARIABLE}})
    set(${VARIABLE} $ENV{${VARIABLE}})
  endif()
endmacro()


message(">>> Setup vcpkg")
set_from_environment(VCPKG_ROOT)
if(DEFINED VCPKG_ROOT)
  # Build-triggered regeneration does not inherit configure-preset environment.
  set(VCPKG_ROOT "${VCPKG_ROOT}" CACHE PATH "Selected vcpkg installation")
  set(CMAKE_TOOLCHAIN_FILE "${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" CACHE STRING "")
  set(VCPKG_TARGET_TRIPLET "x64-windows-static" CACHE STRING "")
  set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>" CACHE STRING "")
else()
  message(FATAL_ERROR ">> Variable VCPKG_ROOT is not set!")
endif()


if(DEFINED COMMON_LIB_F4VR_PATH AND NOT COMMON_LIB_F4VR_PATH STREQUAL "")
  set(COMMON_LIB_F4VR_PATH_SELECTED ${COMMON_LIB_F4VR_PATH} CACHE STRING "" FORCE)
  message(">>> Using CommonLibF4VR from custom variable: '${COMMON_LIB_F4VR_PATH_SELECTED}'")
else()
  set(COMMON_LIB_F4VR_PATH_SELECTED "${WORKSPACE_ROOT}/libraries_and_tools/CommonLibF4VR" CACHE STRING "" FORCE)
  message(">>> Using canonical pristine workspace CommonLibF4VR: '${COMMON_LIB_F4VR_PATH_SELECTED}'")
endif()
if(NOT EXISTS "${COMMON_LIB_F4VR_PATH_SELECTED}/CommonLibF4/include/REL/Relocation.h")
  message(FATAL_ERROR ">> CommonLibF4VR path not found in '${COMMON_LIB_F4VR_PATH_SELECTED}'!\n >> Restore libraries_and_tools/CommonLibF4VR or set COMMON_LIB_F4VR_PATH.")
endif()
set(COMMON_LIB_F4VR_PATH "${COMMON_LIB_F4VR_PATH_SELECTED}" CACHE PATH "" FORCE)

project(${NAME} VERSION ${VERSION} LANGUAGES CXX)
message(">>> Building project '${PROJECT_NAME}' v:${PROJECT_VERSION}")
message(">>> Using CommonLibF4VR in: '${COMMON_LIB_F4VR_PATH_SELECTED}'")

add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
set(Boost_USE_STATIC_RUNTIME OFF CACHE BOOL "")
set(Boost_USE_STATIC_LIBS ON)
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION OFF)
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_DEBUG OFF)

set(F4SE_SUPPORT_XBYAK ON CACHE BOOL "" FORCE)
set(ENABLE_FALLOUT_NG OFF CACHE BOOL "" FORCE)
set(ENABLE_FALLOUT_F4 OFF CACHE BOOL "" FORCE)
set(ENABLE_FALLOUT_VR ON CACHE BOOL "" FORCE)

if(NOT F4SE_SUPPORT_XBYAK OR ENABLE_FALLOUT_F4 OR ENABLE_FALLOUT_NG OR NOT ENABLE_FALLOUT_VR)
  message(FATAL_ERROR ">> Invalid CommonLibF4VR configuration; remove the build tree and configure again.")
endif()

add_subdirectory(${COMMON_LIB_F4VR_PATH_SELECTED} "CommonLibF4" EXCLUDE_FROM_ALL)
find_package(nlohmann_json CONFIG REQUIRED)

set(ROCK_OPENVR_ROOT "${ROOT_DIR}/third_party/openvr")
if(NOT EXISTS "${ROCK_OPENVR_ROOT}/openvr.h" OR NOT EXISTS "${ROCK_OPENVR_ROOT}/openvr_api.lib")
  message(FATAL_ERROR ">> ROCK's vendored OpenVR SDK is incomplete at '${ROCK_OPENVR_ROOT}'.")
endif()
add_library(ROCKOpenVR UNKNOWN IMPORTED)
set_target_properties(ROCKOpenVR PROPERTIES
  IMPORTED_LOCATION "${ROCK_OPENVR_ROOT}/openvr_api.lib"
  INTERFACE_INCLUDE_DIRECTORIES "${ROCK_OPENVR_ROOT}"
)
add_library(ROCK::OpenVR ALIAS ROCKOpenVR)


configure_file(${ROOT_DIR}/cmake/version.rc.in  ${BUILD_DIR}/version.rc  @ONLY)
configure_file(${ROOT_DIR}/cmake/Version.h.in   ${BUILD_DIR}/include/Version.h @ONLY)
source_group(TREE ${BUILD_DIR} FILES ${BUILD_DIR}/include/Version.h)

file(GLOB_RECURSE headers CONFIGURE_DEPENDS "${SOURCE_DIR}/*.h")
file(GLOB_RECURSE sources CONFIGURE_DEPENDS "${SOURCE_DIR}/*.cpp")
file(GLOB_RECURSE inls    CONFIGURE_DEPENDS "${SOURCE_DIR}/*.inl")
source_group(TREE ${ROOT_DIR} FILES ${headers} ${sources} ${inls})

add_library(${PROJECT_NAME} SHARED
  ${headers}
  ${sources}
  ${inls}
  ${BUILD_DIR}/include/Version.h
  ${BUILD_DIR}/version.rc
  "${ROOT_DIR}/.clang-format"
  "${ROOT_DIR}/.editorconfig"
)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_23)
target_compile_definitions(${PROJECT_NAME} PRIVATE _UNICODE F4VR_WITH_OPENVR=1)

# Optimize across ROCK translation units in Release without changing the SDK,
# floating-point model or minimum CPU ISA. The toolchain must support this path.
include(CheckIPOSupported)
check_ipo_supported(RESULT ROCK_IPO_SUPPORTED OUTPUT ROCK_IPO_ERROR LANGUAGES CXX)
if(NOT ROCK_IPO_SUPPORTED)
  message(FATAL_ERROR "ROCK Release IPO is unavailable: ${ROCK_IPO_ERROR}")
endif()
set_property(TARGET ${PROJECT_NAME} PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)

target_include_directories(${PROJECT_NAME}
  PRIVATE
    ${BUILD_DIR}/include
    ${SOURCE_DIR}
)

target_link_libraries(${PROJECT_NAME}
  PRIVATE
    CommonLibF4::CommonLibF4
    nlohmann_json::nlohmann_json
    ROCK::OpenVR
)

find_package(spdlog CONFIG REQUIRED)
find_package(mmio CONFIG REQUIRED)

target_precompile_headers(${PROJECT_NAME} PRIVATE src/PCH.h)

target_compile_options(${PROJECT_NAME}
  PRIVATE
    /sdl
	/utf-8
	/Zi

	/permissive-

	/Zc:alignedNew
	/Zc:auto
	/Zc:char8_t
	/Zc:__cplusplus
	/Zc:externC
	/Zc:externConstexpr
	/Zc:forScope
	/Zc:hiddenFriend
	/Zc:implicitNoexcept
	/Zc:lambda
	/Zc:noexceptTypes
	/Zc:preprocessor
	/Zc:referenceBinding
	/Zc:rvalueCast
	/Zc:sizedDealloc
	/Zc:strictStrings
	/Zc:ternary
	/Zc:threadSafeInit
	/Zc:tlsGuards
	/Zc:trigraphs
	/Zc:wchar_t

	/external:anglebrackets
	/external:W0

    /W4
    /wd4715
    "$<$<CONFIG:RELEASE>:/Zc:inline;/JMC-;/Ob3>"
)

target_link_options(${PROJECT_NAME}
  PRIVATE
    "$<$<CONFIG:DEBUG>:/INCREMENTAL;/OPT:NOREF;/OPT:NOICF>"
    "$<$<CONFIG:RELEASE>:/INCREMENTAL:NO;/OPT:REF;/OPT:ICF;/DEBUG:FULL;/CGTHREADS:2>"
)

string(REPLACE "/Ob2" "/Ob3" CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")

# ROCK separates plugin iteration, verification, and release packaging because the
# normal development loop needs a deployable DLL/PDB without implicitly building
# test executables or archives. Tests and release packages remain explicit paths,
# while fast local builds can still copy the plugin into the configured mod root.
option(POST_BUILD_COPY_PLUGIN "Copy ROCK.dll and ROCK.pdb to configured mod root after building." OFF)
set(COPY_PLUGIN_BASE_PATH "" CACHE STRING "Semicolon-separated mod roots that contain F4SE/Plugins.")
option(ROCK_PACKAGE_RELEASE "Package Release builds into a .7z archive after ROCK is built." ON)
option(BUILD_ROCK_TESTS "Build ROCK focused regression tests" OFF)
# ROCK publishes its provider API from include/ROCK, the only copy of those
# headers. Consumers add this include directory; there is no SDK copy.
set(ROCK_PUBLIC_INCLUDE_DIR "${ROOT_DIR}/include")
include_directories("${ROCK_PUBLIC_INCLUDE_DIR}")
# RPS UI Framework publishes its own headers; ROCK binds it at runtime and works
# without it (world drawing and UI input cooperation are then unavailable).
set(RPSUI_INCLUDE_DIR "${WORKSPACE_ROOT}/../ui_mods/RPS_UI_Framework/SDK/include" CACHE PATH
  "RPS UI Framework public header directory.")
if(NOT EXISTS "${RPSUI_INCLUDE_DIR}/RPSUIWorldDrawApi.h")
  message(FATAL_ERROR ">> RPS UI Framework headers not found at '${RPSUI_INCLUDE_DIR}'; set RPSUI_INCLUDE_DIR.")
endif()
target_include_directories(${PROJECT_NAME} PRIVATE "${RPSUI_INCLUDE_DIR}")

include("${ROOT_DIR}/cmake/VerifyDistributionInputs.cmake")
get_target_property(ROCK_BUILD_INPUTS ${PROJECT_NAME} SOURCES)
rock_verify_distribution_inputs(
  TREES "${ROOT_DIR}/data/mod" "${ROCK_PUBLIC_INCLUDE_DIR}"
  FILES ${ROCK_BUILD_INPUTS})

if(POST_BUILD_COPY_PLUGIN)
  if(DEFINED COPY_PLUGIN_BASE_PATH AND NOT COPY_PLUGIN_BASE_PATH STREQUAL "")
    set(COPY_PLUGIN_PATH_LIST ${COPY_PLUGIN_BASE_PATH})

    message(">>> Create post build command: copy targets to:")
    foreach(copy_path IN LISTS COPY_PLUGIN_PATH_LIST)
      message("    ${copy_path}/F4SE/Plugins/")
      add_custom_command(
        TARGET ${PROJECT_NAME}
        POST_BUILD
        COMMENT "Copying ${PROJECT_NAME}.dll and .pdb to plugins folder: '${copy_path}/F4SE/Plugins/'"
        COMMAND ${CMAKE_COMMAND}
          "-DINPUT_TREES=${ROOT_DIR}/data/mod"
          -P "${ROOT_DIR}/cmake/VerifyDistributionInputs.cmake"
        COMMAND ${CMAKE_COMMAND} -E make_directory ${copy_path}/F4SE/Plugins/
        COMMAND ${CMAKE_COMMAND} -E rm -f "${copy_path}/ROCK.esp"
        COMMAND ${CMAKE_COMMAND} -E rm -rf "${copy_path}/Scripts/ROCK"
        COMMAND ${CMAKE_COMMAND} -E rm -rf "${copy_path}/Meshes/ROCK"
        COMMAND ${CMAKE_COMMAND} -E copy_directory "${ROOT_DIR}/data/mod" "${copy_path}"
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${PROJECT_NAME}> ${copy_path}/F4SE/Plugins/
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_PDB_FILE:${PROJECT_NAME}> ${copy_path}/F4SE/Plugins/
      )
    endforeach()
  else()
    message(FATAL_ERROR "Variable 'COPY_PLUGIN_BASE_PATH' is not defined but 'POST_BUILD_COPY_PLUGIN' is true!")
  endif()
else()
  message(">>> No post build copy")
endif()

if(ROCK_PACKAGE_RELEASE)
  message(">>> Create post build command: package mod on release build")
  add_custom_command(
    TARGET ${PROJECT_NAME}
    POST_BUILD
    COMMAND ${CMAKE_COMMAND}
            -DPROJECT_NAME=${PROJECT_NAME}
            -DPROJECT_FRIENDLY_NAME=${FRIENDLY_NAME}
            -DPROJECT_VERSION=${PROJECT_VERSION}
            -DROOT_DIR=${ROOT_DIR}
            -DBUILD_DIR=${BUILD_DIR}
            -DTARGET_FILE="$<TARGET_FILE:${PROJECT_NAME}>"
            -DTARGET_PDB_FILE="$<TARGET_PDB_FILE:${PROJECT_NAME}>"
            -DCONFIG=$(Configuration)
            -P "${ROOT_DIR}/cmake/package.cmake"
  )
else()
  message(">>> Release package post build command disabled")
endif()

if(BUILD_ROCK_TESTS)
  add_executable(ROCKHoldingGripFormatTests tests/HoldingGripFormatTests.cpp)
  target_include_directories(ROCKHoldingGripFormatTests PRIVATE "${SOURCE_DIR}")
  target_link_libraries(ROCKHoldingGripFormatTests PRIVATE nlohmann_json::nlohmann_json)
  target_compile_features(ROCKHoldingGripFormatTests PRIVATE cxx_std_20)
  target_compile_options(ROCKHoldingGripFormatTests PRIVATE /UNDEBUG)
  add_test(NAME ROCKHoldingGripFormatTests COMMAND ROCKHoldingGripFormatTests)
  add_executable(ROCKHoldingGripMathTests tests/HoldingGripMathTests.cpp)
  target_include_directories(ROCKHoldingGripMathTests PRIVATE "${SOURCE_DIR}")
  target_compile_features(ROCKHoldingGripMathTests PRIVATE cxx_std_20)
  target_compile_options(ROCKHoldingGripMathTests PRIVATE /UNDEBUG)
  add_test(NAME ROCKHoldingGripMathTests COMMAND ROCKHoldingGripMathTests)

  enable_testing()


  # Aggregate build target for every registered policy test. add_rock_test
  # attaches each test executable as a dependency, so the full-suite runner
  # (tools/Invoke-RockTests.ps1) cannot run a test whose binary was never
  # built. Do not list test targets here manually.
  add_custom_target(ROCKPolicyTestBinaries)

  include("${ROOT_DIR}/cmake/RockTestRegistration.cmake")

  # Public headers: each compiles alone inside a packed caller with only the
  # public include directory, published records and tables keep their layout,
  # and the one consumer example builds.
  file(GLOB ROCK_PUBLIC_HEADERS CONFIGURE_DEPENDS "${ROCK_PUBLIC_INCLUDE_DIR}/ROCK/*.h")
  set(ROCK_PUBLIC_HEADER_SOURCES "${BUILD_DIR}/public-header-tests/Main.cpp")
  file(CONFIGURE OUTPUT "${BUILD_DIR}/public-header-tests/Main.cpp" CONTENT "int main() { return 0; }\n")
  foreach(ROCK_PUBLIC_HEADER IN LISTS ROCK_PUBLIC_HEADERS)
    get_filename_component(ROCK_PUBLIC_HEADER_NAME "${ROCK_PUBLIC_HEADER}" NAME_WE)
    set(ROCK_PUBLIC_HEADER_SOURCE "${BUILD_DIR}/public-header-tests/${ROCK_PUBLIC_HEADER_NAME}.cpp")
    file(CONFIGURE OUTPUT "${ROCK_PUBLIC_HEADER_SOURCE}"
      CONTENT "#pragma pack(push, 1)\n#include <ROCK/${ROCK_PUBLIC_HEADER_NAME}.h>\n#pragma pack(pop)\n")
    list(APPEND ROCK_PUBLIC_HEADER_SOURCES "${ROCK_PUBLIC_HEADER_SOURCE}")
  endforeach()
  add_executable(ROCKPublicHeaderTests ${ROCK_PUBLIC_HEADER_SOURCES} "${ROOT_DIR}/tests/ConsumerExample.cpp")
  set_target_properties(ROCKPublicHeaderTests PROPERTIES INCLUDE_DIRECTORIES "${ROCK_PUBLIC_INCLUDE_DIR}")
  target_compile_features(ROCKPublicHeaderTests PRIVATE cxx_std_20)
  add_rock_test(ROCKPublicHeaderTests)
  add_executable(ROCKPublicAbiLayoutTests "${ROOT_DIR}/tests/ModularAbiLayoutTests.cpp")
  set_target_properties(ROCKPublicAbiLayoutTests PROPERTIES INCLUDE_DIRECTORIES "${ROCK_PUBLIC_INCLUDE_DIR};${ROOT_DIR}/tests")
  target_compile_features(ROCKPublicAbiLayoutTests PRIVATE cxx_std_20)
  add_rock_test(ROCKPublicAbiLayoutTests)
  add_executable(ROCKNativeInteractionAbiTests "${ROOT_DIR}/tests/NativeInteractionAbiTests.cpp")
  set_target_properties(ROCKNativeInteractionAbiTests PROPERTIES INCLUDE_DIRECTORIES "${ROCK_PUBLIC_INCLUDE_DIR}")
  target_compile_features(ROCKNativeInteractionAbiTests PRIVATE cxx_std_20)
  add_rock_test(ROCKNativeInteractionAbiTests)

  add_executable(ROCKSupportTriggerInputTests
    "${ROOT_DIR}/tests/SupportTriggerInputTests.cpp"
    "${SOURCE_DIR}/physics-interaction/input/SupportTriggerInput.cpp")
  target_compile_features(ROCKSupportTriggerInputTests PRIVATE cxx_std_23)
  target_include_directories(ROCKSupportTriggerInputTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKSupportTriggerInputTests)

  add_executable(ROCKPhysicalReloadInputTests
    "${ROOT_DIR}/tests/PhysicalReloadInputTests.cpp"
    "${SOURCE_DIR}/physics-interaction/input/PhysicalReloadInput.cpp")
  target_compile_features(ROCKPhysicalReloadInputTests PRIVATE cxx_std_23)
  target_include_directories(ROCKPhysicalReloadInputTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKPhysicalReloadInputTests)

  add_executable(ROCKSavedGrabOffsetFormatTests
    "${ROOT_DIR}/tests/SavedGrabOffsetFormatTests.cpp"
    "${SOURCE_DIR}/physics-interaction/grab/SavedGrabOffsetFormat.cpp"
  )
  target_compile_features(ROCKSavedGrabOffsetFormatTests PRIVATE cxx_std_23)
  target_include_directories(ROCKSavedGrabOffsetFormatTests PRIVATE ${SOURCE_DIR})
  target_link_libraries(ROCKSavedGrabOffsetFormatTests PRIVATE nlohmann_json::nlohmann_json)
  add_rock_test(ROCKSavedGrabOffsetFormatTests)

  add_executable(ROCKGrabInputIntentPolicyTests
    "${ROOT_DIR}/tests/GrabInputIntentPolicyTests.cpp"
  )
  target_compile_features(ROCKGrabInputIntentPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKGrabInputIntentPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKGrabInputIntentPolicyTests)

  add_executable(ROCKFarPullGestureTests
    "${ROOT_DIR}/tests/FarPullGestureTests.cpp"
  )
  target_compile_features(ROCKFarPullGestureTests PRIVATE cxx_std_23)
  target_include_directories(ROCKFarPullGestureTests PRIVATE ${SOURCE_DIR} "${ROCK_OPENVR_ROOT}")
  add_rock_test(ROCKFarPullGestureTests)

  add_executable(ROCKInputRemapPolicyTests
    "${ROOT_DIR}/tests/InputRemapPolicyTests.cpp"
  )
  target_compile_features(ROCKInputRemapPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKInputRemapPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKInputRemapPolicyTests)

  add_executable(ROCKEquippedWeaponHandlingRequestTests
    "${ROOT_DIR}/tests/EquippedWeaponHandlingRequestTests.cpp"
  )
  target_compile_features(ROCKEquippedWeaponHandlingRequestTests PRIVATE cxx_std_23)
  target_include_directories(ROCKEquippedWeaponHandlingRequestTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKEquippedWeaponHandlingRequestTests)

  add_executable(ROCKVirtualHolstersCompatibilityTests
    "${ROOT_DIR}/tests/VirtualHolstersCompatibilityTests.cpp"
  )
  target_compile_features(ROCKVirtualHolstersCompatibilityTests PRIVATE cxx_std_23)
  target_include_directories(ROCKVirtualHolstersCompatibilityTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKVirtualHolstersCompatibilityTests)

  add_executable(ROCKWeaponGapDecompositionTests
    "${ROOT_DIR}/tests/WeaponGapDecompositionTests.cpp"
  )
  target_compile_features(ROCKWeaponGapDecompositionTests PRIVATE cxx_std_23)
  target_include_directories(ROCKWeaponGapDecompositionTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKWeaponGapDecompositionTests)

  add_executable(ROCKInteractionSafetyPolicyTests
    "${ROOT_DIR}/tests/InteractionSafetyPolicyTests.cpp"
  )
  target_compile_features(ROCKInteractionSafetyPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKInteractionSafetyPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKInteractionSafetyPolicyTests)

  add_executable(ROCKCharacterProxyFaultPolicyTests
    "${ROOT_DIR}/tests/CharacterProxyFaultPolicyTests.cpp"
  )
  target_compile_features(ROCKCharacterProxyFaultPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKCharacterProxyFaultPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKCharacterProxyFaultPolicyTests)

  add_executable(ROCKNativeMemoryTests
    "${ROOT_DIR}/tests/NativeMemoryTests.cpp"
    "${SOURCE_DIR}/physics-interaction/native/NativeMemory.cpp"
  )
  target_compile_features(ROCKNativeMemoryTests PRIVATE cxx_std_23)
  target_include_directories(ROCKNativeMemoryTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKNativeMemoryTests)

  add_executable(ROCKHavokRefCountTests "${ROOT_DIR}/tests/HavokRefCountTests.cpp")
  target_compile_features(ROCKHavokRefCountTests PRIVATE cxx_std_23)
  target_include_directories(ROCKHavokRefCountTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKHavokRefCountTests)
  add_executable(ROCKNiReferenceOwnershipTests "${ROOT_DIR}/tests/NiReferenceOwnershipTests.cpp")
  target_compile_features(ROCKNiReferenceOwnershipTests PRIVATE cxx_std_23)
  target_include_directories(ROCKNiReferenceOwnershipTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKNiReferenceOwnershipTests)

  add_executable(ROCKFo4VrActorStatePolicyTests
    "${ROOT_DIR}/tests/Fo4VrActorStatePolicyTests.cpp"
  )
  target_compile_features(ROCKFo4VrActorStatePolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKFo4VrActorStatePolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKFo4VrActorStatePolicyTests)

  add_executable(ROCKGrabFingerLocalTransformPolicyTests
    "${ROOT_DIR}/tests/GrabFingerLocalTransformPolicyTests.cpp"
    "${SOURCE_DIR}/physics-interaction/grab/GeneratedGrabFingerCalibration.cpp"
  )
  target_compile_features(ROCKGrabFingerLocalTransformPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKGrabFingerLocalTransformPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKGrabFingerLocalTransformPolicyTests)

  add_executable(ROCKSurfaceMeshGrabPolicyTests
    "${ROOT_DIR}/tests/SurfaceMeshGrabPolicyTests.cpp"
  )
  target_compile_features(ROCKSurfaceMeshGrabPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKSurfaceMeshGrabPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKSurfaceMeshGrabPolicyTests)

  add_executable(ROCKContactSignalSubscriptionPolicyTests
    "${ROOT_DIR}/tests/ContactSignalSubscriptionPolicyTests.cpp"
  )
  target_compile_features(ROCKContactSignalSubscriptionPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKContactSignalSubscriptionPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKContactSignalSubscriptionPolicyTests)

  add_executable(ROCKGrabAuthorityFramePolicyTests
    "${ROOT_DIR}/tests/GrabAuthorityFramePolicyTests.cpp"
  )
  target_compile_features(ROCKGrabAuthorityFramePolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKGrabAuthorityFramePolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKGrabAuthorityFramePolicyTests)

  add_executable(ROCKTransformMathDoublePrecisionPolicyTests
    "${ROOT_DIR}/tests/TransformMathDoublePrecisionPolicyTests.cpp"
  )
  target_compile_features(ROCKTransformMathDoublePrecisionPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKTransformMathDoublePrecisionPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKTransformMathDoublePrecisionPolicyTests)

  add_executable(ROCKGrabLocalGraspSolvePolicyTests
    "${ROOT_DIR}/tests/GrabLocalGraspSolvePolicyTests.cpp"
  )
  target_compile_features(ROCKGrabLocalGraspSolvePolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKGrabLocalGraspSolvePolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKGrabLocalGraspSolvePolicyTests)

  add_executable(ROCKHeldPlacementPolicyTests "${ROOT_DIR}/tests/HeldPlacementPolicyTests.cpp")
  target_compile_features(ROCKHeldPlacementPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKHeldPlacementPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKHeldPlacementPolicyTests)

  add_executable(ROCKLooseWeaponLayerRepairTests "${ROOT_DIR}/tests/LooseWeaponLayerRepairTests.cpp")
  target_compile_features(ROCKLooseWeaponLayerRepairTests PRIVATE cxx_std_23)
  target_include_directories(ROCKLooseWeaponLayerRepairTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKLooseWeaponLayerRepairTests)

  add_executable(ROCKEjectionContactRequestsTests "${ROOT_DIR}/tests/EjectionContactRequestsTests.cpp")
  target_compile_features(ROCKEjectionContactRequestsTests PRIVATE cxx_std_23)
  target_include_directories(ROCKEjectionContactRequestsTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKEjectionContactRequestsTests)

  add_executable(ROCKObjectRestRequestsTests "${ROOT_DIR}/tests/ObjectRestRequestsTests.cpp")
  target_compile_features(ROCKObjectRestRequestsTests PRIVATE cxx_std_23)
  target_include_directories(ROCKObjectRestRequestsTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKObjectRestRequestsTests)

  add_executable(ROCKGeneratedBodyContactRegistryPolicyTests
    "${ROOT_DIR}/tests/GeneratedBodyContactRegistryPolicyTests.cpp"
  )
  target_compile_features(ROCKGeneratedBodyContactRegistryPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKGeneratedBodyContactRegistryPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKGeneratedBodyContactRegistryPolicyTests)

  add_executable(ROCKHavokTimingFixPolicyTests
    "${ROOT_DIR}/tests/HavokTimingFixPolicyTests.cpp"
  )
  target_compile_features(ROCKHavokTimingFixPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKHavokTimingFixPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKHavokTimingFixPolicyTests)

  add_executable(ROCKEquippedWeaponDropMomentumPolicyTests
    "${ROOT_DIR}/tests/EquippedWeaponDropMomentumPolicyTests.cpp"
  )
  target_compile_features(ROCKEquippedWeaponDropMomentumPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKEquippedWeaponDropMomentumPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKEquippedWeaponDropMomentumPolicyTests)

  add_executable(ROCKRagdollInteractionTests
    "${ROOT_DIR}/tests/RagdollInteractionTests.cpp"
  )
  target_compile_features(ROCKRagdollInteractionTests PRIVATE cxx_std_23)
  target_include_directories(ROCKRagdollInteractionTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKRagdollInteractionTests)

  add_executable(ROCKGrabAuthorityProxyPolicyTests
    "${ROOT_DIR}/tests/GrabAuthorityProxyPolicyTests.cpp"
  )
  target_compile_features(ROCKGrabAuthorityProxyPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKGrabAuthorityProxyPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKGrabAuthorityProxyPolicyTests)

  add_executable(ROCKGrabAuthoritySourceClockPolicyTests
    "${ROOT_DIR}/tests/GrabAuthoritySourceClockPolicyTests.cpp"
  )
  target_compile_features(ROCKGrabAuthoritySourceClockPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKGrabAuthoritySourceClockPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKGrabAuthoritySourceClockPolicyTests)

  add_executable(ROCKHandColliderFramePolicyTests
    "${ROOT_DIR}/tests/HandColliderFramePolicyTests.cpp"
  )
  target_compile_features(ROCKHandColliderFramePolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKHandColliderFramePolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKHandColliderFramePolicyTests)

  add_executable(ROCKColliderLookupAndTuningTests
    "${ROOT_DIR}/tests/ColliderLookupAndTuningTests.cpp"
    "${SOURCE_DIR}/physics-interaction/collision/ColliderTuning.cpp"
  )
  target_compile_features(ROCKColliderLookupAndTuningTests PRIVATE cxx_std_23)
  target_include_directories(ROCKColliderLookupAndTuningTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKColliderLookupAndTuningTests)

  add_executable(ROCKCpuWorkReductionTests "${ROOT_DIR}/tests/CpuWorkReductionTests.cpp")
  target_compile_features(ROCKCpuWorkReductionTests PRIVATE cxx_std_23)
  target_include_directories(ROCKCpuWorkReductionTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKCpuWorkReductionTests)

  add_executable(ROCKHknpBodyFramePolicyTests
    "${ROOT_DIR}/tests/HknpBodyFramePolicyTests.cpp"
  )
  target_compile_features(ROCKHknpBodyFramePolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKHknpBodyFramePolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKHknpBodyFramePolicyTests)

  add_executable(ROCKImmersiveAidPolicyTests
    "${ROOT_DIR}/tests/ImmersiveAidPolicyTests.cpp"
  )
  target_compile_features(ROCKImmersiveAidPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKImmersiveAidPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKImmersiveAidPolicyTests)

  add_executable(ROCKNativeScopeAdmissionTests
    "${ROOT_DIR}/tests/NativeScopeAdmissionTests.cpp"
  )
  target_compile_features(ROCKNativeScopeAdmissionTests PRIVATE cxx_std_23)
  target_include_directories(ROCKNativeScopeAdmissionTests PRIVATE ${SOURCE_DIR})
  target_link_libraries(ROCKNativeScopeAdmissionTests PRIVATE CommonLibF4::CommonLibF4)
  add_rock_test(ROCKNativeScopeAdmissionTests)

  add_executable(ROCKNativeExecutionSlotStubTests "${ROOT_DIR}/tests/NativeExecutionSlotStubTests.cpp")
  target_compile_features(ROCKNativeExecutionSlotStubTests PRIVATE cxx_std_23)
  target_include_directories(ROCKNativeExecutionSlotStubTests PRIVATE ${SOURCE_DIR})
  target_link_libraries(ROCKNativeExecutionSlotStubTests PRIVATE CommonLibF4::CommonLibF4)
  add_rock_test(ROCKNativeExecutionSlotStubTests)

  add_executable(ROCKNativeSecondaryModelBranchTests "${ROOT_DIR}/tests/NativeSecondaryModelBranchTests.cpp")
  target_compile_features(ROCKNativeSecondaryModelBranchTests PRIVATE cxx_std_23)
  target_include_directories(ROCKNativeSecondaryModelBranchTests PRIVATE ${SOURCE_DIR})
  target_link_libraries(ROCKNativeSecondaryModelBranchTests PRIVATE CommonLibF4::CommonLibF4)
  add_rock_test(ROCKNativeSecondaryModelBranchTests)

  add_executable(ROCKNativeModelAncestryTests
    "${ROOT_DIR}/tests/NativeModelAncestryTests.cpp"
    "${SOURCE_DIR}/physics-interaction/native/NativeMemory.cpp")
  target_compile_features(ROCKNativeModelAncestryTests PRIVATE cxx_std_23)
  target_include_directories(ROCKNativeModelAncestryTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKNativeModelAncestryTests)

  add_executable(ROCKNativePlayerCollisionSuppressionPolicyTests
    "${ROOT_DIR}/tests/NativePlayerCollisionSuppressionPolicyTests.cpp"
  )
  target_compile_features(ROCKNativePlayerCollisionSuppressionPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKNativePlayerCollisionSuppressionPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKNativePlayerCollisionSuppressionPolicyTests)

  add_executable(ROCKNativeMeleeSuppressionPolicyTests
    "${ROOT_DIR}/tests/NativeMeleeSuppressionPolicyTests.cpp"
  )
  target_compile_features(ROCKNativeMeleeSuppressionPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKNativeMeleeSuppressionPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKNativeMeleeSuppressionPolicyTests)

  add_executable(ROCKNativeGrabHapticSuppressionPolicyTests
    "${ROOT_DIR}/tests/NativeGrabHapticSuppressionPolicyTests.cpp"
  )
  target_compile_features(ROCKNativeGrabHapticSuppressionPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKNativeGrabHapticSuppressionPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKNativeGrabHapticSuppressionPolicyTests)

  add_executable(ROCKHeldWeaponInstantTransitionPolicyTests
    "${ROOT_DIR}/tests/HeldWeaponInstantTransitionPolicyTests.cpp"
  )
  target_compile_features(ROCKHeldWeaponInstantTransitionPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKHeldWeaponInstantTransitionPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKHeldWeaponInstantTransitionPolicyTests)

  add_executable(ROCKHeldScenePresentationPolicyTests
    "${ROOT_DIR}/tests/HeldScenePresentationPolicyTests.cpp"
  )
  target_compile_features(ROCKHeldScenePresentationPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKHeldScenePresentationPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKHeldScenePresentationPolicyTests)

  add_executable(ROCKWeaponInventoryStackSelectionPolicyTests
    "${ROOT_DIR}/tests/WeaponInventoryStackSelectionPolicyTests.cpp"
  )
  target_compile_features(ROCKWeaponInventoryStackSelectionPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKWeaponInventoryStackSelectionPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKWeaponInventoryStackSelectionPolicyTests)

  add_executable(ROCKHandFingerMirrorMathTests
    "${ROOT_DIR}/tests/HandFingerMirrorMathTests.cpp"
  )
  target_compile_features(ROCKHandFingerMirrorMathTests PRIVATE cxx_std_23)
  target_include_directories(ROCKHandFingerMirrorMathTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKHandFingerMirrorMathTests)

  add_executable(ROCKPhysicsCallbackQuiescenceGateTests
    "${ROOT_DIR}/tests/PhysicsCallbackQuiescenceGateTests.cpp"
  )
  target_compile_features(ROCKPhysicsCallbackQuiescenceGateTests PRIVATE cxx_std_23)
  target_include_directories(ROCKPhysicsCallbackQuiescenceGateTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKPhysicsCallbackQuiescenceGateTests)

  add_executable(ROCKHavokCompoundShapeKeyPolicyTests
    "${ROOT_DIR}/tests/HavokCompoundShapeKeyPolicyTests.cpp"
  )
  target_compile_features(ROCKHavokCompoundShapeKeyPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKHavokCompoundShapeKeyPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKHavokCompoundShapeKeyPolicyTests)

  add_executable(ROCKDynamicWeaponCollisionPolicyTests
    "${ROOT_DIR}/tests/DynamicWeaponCollisionPolicyTests.cpp"
  )
  target_compile_features(ROCKDynamicWeaponCollisionPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKDynamicWeaponCollisionPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKDynamicWeaponCollisionPolicyTests)


  add_executable(ROCKHandWorldClaimRegistryPolicyTests
    "${ROOT_DIR}/tests/HandWorldClaimRegistryPolicyTests.cpp"
  )
  target_compile_features(ROCKHandWorldClaimRegistryPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKHandWorldClaimRegistryPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKHandWorldClaimRegistryPolicyTests)

  add_executable(ROCKTrackedHandIsolationPolicyTests
    "${ROOT_DIR}/tests/TrackedHandIsolationPolicyTests.cpp"
  )
  target_compile_features(ROCKTrackedHandIsolationPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKTrackedHandIsolationPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKTrackedHandIsolationPolicyTests)

  add_executable(ROCKWeaponInputFilterTests "${ROOT_DIR}/tests/WeaponInputFilterTests.cpp")
  target_compile_features(ROCKWeaponInputFilterTests PRIVATE cxx_std_23)
  target_include_directories(ROCKWeaponInputFilterTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKWeaponInputFilterTests)

  add_executable(ROCKRenderedBoneTransportPolicyTests
    "${ROOT_DIR}/tests/RenderedBoneTransportPolicyTests.cpp"
  )
  target_compile_features(ROCKRenderedBoneTransportPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKRenderedBoneTransportPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKRenderedBoneTransportPolicyTests)

  add_executable(ROCKWeaponAimBasisTests
    "${ROOT_DIR}/tests/WeaponAimBasisTests.cpp"
  )
  target_compile_features(ROCKWeaponAimBasisTests PRIVATE cxx_std_23)
  target_include_directories(ROCKWeaponAimBasisTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKWeaponAimBasisTests)

  add_executable(ROCKGunstockModeTests "${ROOT_DIR}/tests/GunstockModeTests.cpp")
  target_compile_features(ROCKGunstockModeTests PRIVATE cxx_std_23)
  target_include_directories(ROCKGunstockModeTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKGunstockModeTests)

  add_executable(ROCKGunstockDebugTests
    "${ROOT_DIR}/tests/GunstockDebugTests.cpp"
    "${SOURCE_DIR}/physics-interaction/weapon/GunstockDebug.cpp")
  target_compile_features(ROCKGunstockDebugTests PRIVATE cxx_std_23)
  target_include_directories(ROCKGunstockDebugTests PRIVATE ${SOURCE_DIR})
  add_test(NAME ROCKGunstockDebugTests COMMAND ROCKGunstockDebugTests)

  add_executable(ROCKFrikPresentedHandTests "${ROOT_DIR}/tests/FrikPresentedHandTests.cpp")
  target_compile_features(ROCKFrikPresentedHandTests PRIVATE cxx_std_23)
  target_include_directories(ROCKFrikPresentedHandTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKFrikPresentedHandTests)

  add_executable(ROCKPresentedPoseServiceTests "${ROOT_DIR}/tests/PresentedPoseServiceTests.cpp")
  target_compile_features(ROCKPresentedPoseServiceTests PRIVATE cxx_std_23)
  target_include_directories(ROCKPresentedPoseServiceTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKPresentedPoseServiceTests)

  add_executable(ROCKTimedReleaseVelocityHistoryTests "${ROOT_DIR}/tests/TimedReleaseVelocityHistoryTests.cpp")
  target_compile_features(ROCKTimedReleaseVelocityHistoryTests PRIVATE cxx_std_23)
  target_include_directories(ROCKTimedReleaseVelocityHistoryTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKTimedReleaseVelocityHistoryTests)

  set(ROCK_COMMONLIB_TEST_TARGETS
    ROCKColliderLookupAndTuningTests
    ROCKCpuWorkReductionTests
    ROCKGrabInputIntentPolicyTests
    ROCKInputRemapPolicyTests
    ROCKEquippedWeaponHandlingRequestTests
    ROCKInteractionSafetyPolicyTests
    ROCKGrabFingerLocalTransformPolicyTests
    ROCKSurfaceMeshGrabPolicyTests
    ROCKContactSignalSubscriptionPolicyTests
    ROCKGrabAuthorityFramePolicyTests
    ROCKTransformMathDoublePrecisionPolicyTests
    ROCKGrabLocalGraspSolvePolicyTests
    ROCKGeneratedBodyContactRegistryPolicyTests
    ROCKHavokTimingFixPolicyTests
    ROCKHeldScenePresentationPolicyTests
    ROCKEquippedWeaponDropMomentumPolicyTests
    ROCKGrabAuthorityProxyPolicyTests
    ROCKGrabAuthoritySourceClockPolicyTests
    ROCKHandColliderFramePolicyTests
    ROCKHknpBodyFramePolicyTests
    ROCKImmersiveAidPolicyTests
    ROCKNativePlayerCollisionSuppressionPolicyTests
    ROCKNativeMeleeSuppressionPolicyTests
    ROCKNativeGrabHapticSuppressionPolicyTests
    ROCKHeldWeaponInstantTransitionPolicyTests
    ROCKWeaponInventoryStackSelectionPolicyTests
    ROCKPhysicsCallbackQuiescenceGateTests
    ROCKHavokCompoundShapeKeyPolicyTests
    ROCKDynamicWeaponCollisionPolicyTests
    ROCKHandWorldClaimRegistryPolicyTests
    ROCKTrackedHandIsolationPolicyTests
    ROCKWeaponInputFilterTests
    ROCKRenderedBoneTransportPolicyTests
    ROCKWeaponAimBasisTests
    ROCKGunstockModeTests
    ROCKGunstockDebugTests
    ROCKFrikPresentedHandTests
  )
  # Every policy executable uses the same CommonLib-heavy precompiled header and
  # config stub. Build those once, then reuse the PCH artifact and support library
  # instead of paying that cost independently for every focused test target.
  add_library(ROCKPolicyTestSupport STATIC
    "${ROOT_DIR}/tests/RockConfigPolicyStub.cpp"
    "${ROOT_DIR}/tests/PerformanceProfilerPolicyStub.cpp"
    # Real CommonLib Ni math, so policy tests run the arithmetic the DLL ships.
    "${COMMON_LIB_F4VR_PATH_SELECTED}/CommonLibF4/src/RE/NetImmerse/NiPoint.cpp"
    "${COMMON_LIB_F4VR_PATH_SELECTED}/CommonLibF4/src/RE/NetImmerse/NiMatrix3.cpp"
  )
  target_compile_features(ROCKPolicyTestSupport PUBLIC cxx_std_23)
  target_include_directories(ROCKPolicyTestSupport
    PUBLIC
      ${SOURCE_DIR}
      ${BUILD_DIR}/include
      "${ROCK_OPENVR_ROOT}"
      "${COMMON_LIB_F4VR_PATH_SELECTED}/CommonLibF4/include"
  )
  target_compile_definitions(ROCKPolicyTestSupport
    PUBLIC
      WINVER=0x0601
      _WIN32_WINNT=0x0601
      F4SE_SUPPORT_XBYAK=1
      ENABLE_FALLOUT_VR=1
      ROCK_POLICY_TEST_NO_RUNTIME_LOGGER=1
  )
  # Match the assertion policy of every consumer reusing this PCH.
  target_compile_options(ROCKPolicyTestSupport PRIVATE /UNDEBUG)
  target_precompile_headers(ROCKPolicyTestSupport PRIVATE "${ROOT_DIR}/tests/CommonLibPolicyPCH.h")
  target_link_libraries(ROCKPolicyTestSupport PUBLIC spdlog::spdlog mmio::mmio)
  if(TARGET xbyak::xbyak)
    target_link_libraries(ROCKPolicyTestSupport PUBLIC xbyak::xbyak)
  endif()

  add_executable(ROCKGrabMeshQueryTests "${ROOT_DIR}/tests/GrabMeshQueryTests.cpp")
  target_compile_features(ROCKGrabMeshQueryTests PRIVATE cxx_std_23)
  list(APPEND ROCK_COMMONLIB_TEST_TARGETS ROCKGrabMeshQueryTests)
  add_rock_test(ROCKGrabMeshQueryTests)

  add_executable(ROCKWeaponInteractionQueryTests "${ROOT_DIR}/tests/WeaponInteractionQueryTests.cpp")
  target_compile_features(ROCKWeaponInteractionQueryTests PRIVATE cxx_std_23)
  list(APPEND ROCK_COMMONLIB_TEST_TARGETS ROCKWeaponInteractionQueryTests)
  add_rock_test(ROCKWeaponInteractionQueryTests)
  add_executable(ROCKWeaponPartDrivePoseTests "${ROOT_DIR}/tests/WeaponPartDrivePoseTests.cpp")
  target_compile_features(ROCKWeaponPartDrivePoseTests PRIVATE cxx_std_23)
  list(APPEND ROCK_COMMONLIB_TEST_TARGETS ROCKWeaponPartDrivePoseTests)
  add_rock_test(ROCKWeaponPartDrivePoseTests)

  add_executable(ROCKConfigurationStoreTests
    "${ROOT_DIR}/tests/ConfigurationStoreTests.cpp"
    "${SOURCE_DIR}/config/ConfigurationStore.cpp"
    "${SOURCE_DIR}/RockConfigLoad.cpp"
  )
  target_compile_features(ROCKConfigurationStoreTests PRIVATE cxx_std_23)
  target_compile_definitions(ROCKConfigurationStoreTests PRIVATE ROCK_CONFIG_REFERENCE_DIR="${ROOT_DIR}/data/config")
  list(APPEND ROCK_COMMONLIB_TEST_TARGETS ROCKConfigurationStoreTests)
  add_rock_test(ROCKConfigurationStoreTests)

  add_executable(ROCKConfigurationProviderTests
    "${ROOT_DIR}/tests/ConfigurationProviderTests.cpp"
    "${SOURCE_DIR}/api/providers/ConfigurationProvider.cpp"
    "${SOURCE_DIR}/RockConfig.cpp"
    "${SOURCE_DIR}/RockConfigLoad.cpp"
    "${SOURCE_DIR}/config/ConfigurationStore.cpp"
  )
  target_compile_features(ROCKConfigurationProviderTests PRIVATE cxx_std_23)
  target_include_directories(ROCKConfigurationProviderTests PRIVATE ${SOURCE_DIR})
  target_precompile_headers(ROCKConfigurationProviderTests PRIVATE "${ROOT_DIR}/tests/CommonLibPolicyPCH.h" <F4SE/Logger.h>)
  target_link_libraries(ROCKConfigurationProviderTests PRIVATE CommonLibF4::CommonLibF4 spdlog::spdlog)
  add_rock_test(ROCKConfigurationProviderTests)

  foreach(ROCK_COMMONLIB_TEST_TARGET IN LISTS ROCK_COMMONLIB_TEST_TARGETS)
    target_precompile_headers(${ROCK_COMMONLIB_TEST_TARGET} REUSE_FROM ROCKPolicyTestSupport)
    target_link_libraries(${ROCK_COMMONLIB_TEST_TARGET} PRIVATE ROCKPolicyTestSupport)
  endforeach()

  add_executable(ROCKDebugOverlayFrameAdmissionTests
    "${ROOT_DIR}/tests/DebugOverlayFrameAdmissionTests.cpp"
  )
  target_compile_features(ROCKDebugOverlayFrameAdmissionTests PRIVATE cxx_std_23)
  target_include_directories(ROCKDebugOverlayFrameAdmissionTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKDebugOverlayFrameAdmissionTests)

  add_executable(ROCKDebugOverlaySnapshotPoolTests
    "${ROOT_DIR}/tests/DebugOverlaySnapshotPoolTests.cpp"
  )
  target_compile_features(ROCKDebugOverlaySnapshotPoolTests PRIVATE cxx_std_23)
  target_include_directories(ROCKDebugOverlaySnapshotPoolTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKDebugOverlaySnapshotPoolTests)

  add_executable(ROCKGripZoneIndicatorsTests
    "${ROOT_DIR}/tests/GripZoneIndicatorsTests.cpp"
    "${SOURCE_DIR}/physics-interaction/visual/GripZoneIndicators.cpp"
  )
  target_compile_features(ROCKGripZoneIndicatorsTests PRIVATE cxx_std_23)
  target_include_directories(ROCKGripZoneIndicatorsTests PRIVATE ${SOURCE_DIR})
  target_compile_definitions(ROCKGripZoneIndicatorsTests PRIVATE ROCK_POLICY_TEST_NO_RUNTIME_LOGGER)
  target_precompile_headers(ROCKGripZoneIndicatorsTests PRIVATE <F4SE/Impl/PCH.h>)
  target_link_libraries(ROCKGripZoneIndicatorsTests PRIVATE CommonLibF4::CommonLibF4)
  add_rock_test(ROCKGripZoneIndicatorsTests)

  add_executable(ROCKDebugOverlayLineBatchTests
    "${ROOT_DIR}/tests/DebugOverlayLineBatchTests.cpp"
  )
  target_compile_features(ROCKDebugOverlayLineBatchTests PRIVATE cxx_std_23)
  target_include_directories(ROCKDebugOverlayLineBatchTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKDebugOverlayLineBatchTests)

  add_executable(ROCKModularEventStreamTests "${ROOT_DIR}/tests/ModularEventStreamTests.cpp")
  target_compile_features(ROCKModularEventStreamTests PRIVATE cxx_std_23)
  target_include_directories(ROCKModularEventStreamTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKModularEventStreamTests)

  add_executable(ROCKModularExportsTests "${ROOT_DIR}/tests/ModularExportsTests.cpp")
  target_compile_features(ROCKModularExportsTests PRIVATE cxx_std_23)
  target_compile_options(ROCKModularExportsTests PRIVATE /UNDEBUG)

  add_executable(ROCKProviderApiAbiTests
    "${ROOT_DIR}/tests/ProviderApiAbiTests.cpp"
  )
  target_compile_features(ROCKProviderApiAbiTests PRIVATE cxx_std_23)
  target_include_directories(ROCKProviderApiAbiTests PRIVATE ${SOURCE_DIR} "${ROOT_DIR}/tests")
  add_rock_test(ROCKProviderApiAbiTests)
  add_executable(ROCKLooseWeaponStatePathTests "${ROOT_DIR}/tests/LooseWeaponStatePathTests.cpp")
  target_compile_features(ROCKLooseWeaponStatePathTests PRIVATE cxx_std_23)
  target_include_directories(ROCKLooseWeaponStatePathTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKLooseWeaponStatePathTests)

  add_executable(ROCKProviderInstanceAccessTests "${ROOT_DIR}/tests/ProviderInstanceAccessTests.cpp")
  target_compile_features(ROCKProviderInstanceAccessTests PRIVATE cxx_std_23)
  target_include_directories(ROCKProviderInstanceAccessTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKProviderInstanceAccessTests)

  add_executable(ROCKQueuedHandInputTests "${ROOT_DIR}/tests/QueuedHandInputTests.cpp")
  target_compile_features(ROCKQueuedHandInputTests PRIVATE cxx_std_23)
  target_include_directories(ROCKQueuedHandInputTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKQueuedHandInputTests)

  add_executable(ROCKDefaultGrabPoseRegistryTests "${ROOT_DIR}/tests/DefaultGrabPoseRegistryTests.cpp")
  add_executable(ROCKHeldObjectTargetTests "${ROOT_DIR}/tests/HeldObjectTargetTests.cpp")
  add_executable(ROCKWeaponPartGrabPointTests "${ROOT_DIR}/tests/WeaponPartGrabPointTests.cpp")
  target_compile_features(ROCKWeaponPartGrabPointTests PRIVATE cxx_std_23)
  target_include_directories(ROCKWeaponPartGrabPointTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKWeaponPartGrabPointTests)
  add_executable(ROCKWeaponPartCollisionLeaseTests "${ROOT_DIR}/tests/WeaponPartCollisionLeaseTests.cpp")
  add_executable(ROCKAuthoredInteractionTests "${ROOT_DIR}/tests/AuthoredInteractionTests.cpp")
  target_compile_features(ROCKAuthoredInteractionTests PRIVATE cxx_std_23)
  target_include_directories(ROCKAuthoredInteractionTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKAuthoredInteractionTests)
  target_compile_features(ROCKWeaponPartCollisionLeaseTests PRIVATE cxx_std_23)
  target_include_directories(ROCKWeaponPartCollisionLeaseTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKWeaponPartCollisionLeaseTests)
  target_compile_features(ROCKHeldObjectTargetTests PRIVATE cxx_std_23)
  target_include_directories(ROCKHeldObjectTargetTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKHeldObjectTargetTests)
  target_compile_features(ROCKDefaultGrabPoseRegistryTests PRIVATE cxx_std_23)
  target_include_directories(ROCKDefaultGrabPoseRegistryTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKDefaultGrabPoseRegistryTests)

  add_executable(ROCKProviderApiNegotiationTests
    "${ROOT_DIR}/tests/ProviderApiNegotiationTests.cpp"
    "${ROOT_DIR}/tests/ModularTestTables.cpp"
    "${SOURCE_DIR}/api/Discovery.cpp"
  )
  target_compile_features(ROCKProviderApiNegotiationTests PRIVATE cxx_std_23)
  target_include_directories(ROCKProviderApiNegotiationTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKProviderApiNegotiationTests)

  add_executable(ROCKModularMarshallingTests "${ROOT_DIR}/tests/ModularMarshallingTests.cpp")
  target_compile_features(ROCKModularMarshallingTests PRIVATE cxx_std_23)
  target_include_directories(ROCKModularMarshallingTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKModularMarshallingTests)

  add_executable(ROCKModularSnapshotReadsTests
    "${ROOT_DIR}/tests/ModularSnapshotReadsTests.cpp"
    "${SOURCE_DIR}/api/providers/SnapshotReads.cpp")
  target_compile_features(ROCKModularSnapshotReadsTests PRIVATE cxx_std_23)
  target_include_directories(ROCKModularSnapshotReadsTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKModularSnapshotReadsTests)

  add_executable(ROCKModularEventDeliveryTests
    "${ROOT_DIR}/tests/ModularEventDeliveryTests.cpp"
    "${SOURCE_DIR}/api/EventStreams.cpp")
  target_compile_features(ROCKModularEventDeliveryTests PRIVATE cxx_std_23)
  target_include_directories(ROCKModularEventDeliveryTests PRIVATE ${SOURCE_DIR})
  target_compile_definitions(ROCKModularEventDeliveryTests PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
  add_rock_test(ROCKModularEventDeliveryTests)

  add_executable(ROCKProviderRegistrationTests "${ROOT_DIR}/tests/ProviderRegistrationTests.cpp")
  target_compile_features(ROCKProviderRegistrationTests PRIVATE cxx_std_23)
  target_include_directories(ROCKProviderRegistrationTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKProviderRegistrationTests)

  add_executable(ROCKProviderWeaponSourceTests
    "${ROOT_DIR}/tests/ProviderWeaponSourceTests.cpp"
    "${SOURCE_DIR}/physics-interaction/core/interaction/ProviderWeaponSources.cpp"
    "${SOURCE_DIR}/api/EventStreams.cpp")
  target_compile_features(ROCKProviderWeaponSourceTests PRIVATE cxx_std_23)
  target_include_directories(ROCKProviderWeaponSourceTests PRIVATE "${ROOT_DIR}/tests/fixtures/weapon_sources" ${SOURCE_DIR})
  target_compile_definitions(ROCKProviderWeaponSourceTests PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
  add_rock_test(ROCKProviderWeaponSourceTests)

  add_executable(ROCKProviderHandItemClientTests
    "${ROOT_DIR}/tests/ProviderHandItemClientTests.cpp"
  )
  target_compile_features(ROCKProviderHandItemClientTests PRIVATE cxx_std_23)
  target_include_directories(ROCKProviderHandItemClientTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKProviderHandItemClientTests)

  add_executable(ROCKProviderExternalBodyRegistryTests
    "${ROOT_DIR}/tests/ProviderExternalBodyRegistryTests.cpp"
  )
  target_compile_features(ROCKProviderExternalBodyRegistryTests PRIVATE cxx_std_23)
  target_include_directories(ROCKProviderExternalBodyRegistryTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKProviderExternalBodyRegistryTests)

  add_executable(ROCKProviderTouchGrabRegistryTests
    "${ROOT_DIR}/tests/ProviderTouchGrabRegistryTests.cpp"
  )
  target_compile_features(ROCKProviderTouchGrabRegistryTests PRIVATE cxx_std_23)
  target_include_directories(ROCKProviderTouchGrabRegistryTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKProviderTouchGrabRegistryTests)

  add_executable(ROCKTouchGrabMathTests
    "${ROOT_DIR}/tests/TouchGrabMathTests.cpp"
  )
  target_compile_features(ROCKTouchGrabMathTests PRIVATE cxx_std_23)
  target_include_directories(ROCKTouchGrabMathTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKTouchGrabMathTests)

  add_executable(ROCKGameFrameTimingPolicyTests
    "${ROOT_DIR}/tests/GameFrameTimingPolicyTests.cpp"
  )
  target_compile_features(ROCKGameFrameTimingPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKGameFrameTimingPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKGameFrameTimingPolicyTests)

  add_executable(ROCKPhysicsTimingSamplePolicyTests
    "${ROOT_DIR}/tests/PhysicsTimingSamplePolicyTests.cpp"
  )
  target_compile_features(ROCKPhysicsTimingSamplePolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKPhysicsTimingSamplePolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKPhysicsTimingSamplePolicyTests)

  add_executable(ROCKStorageSceneLookupTests "${ROOT_DIR}/tests/StorageSceneLookupTests.cpp")
  target_compile_features(ROCKStorageSceneLookupTests PRIVATE cxx_std_23)
  target_include_directories(ROCKStorageSceneLookupTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKStorageSceneLookupTests)
  add_executable(ROCKAttachedSceneLookupTests "${ROOT_DIR}/tests/AttachedSceneLookupTests.cpp")
  target_compile_features(ROCKAttachedSceneLookupTests PRIVATE cxx_std_23)
  target_include_directories(ROCKAttachedSceneLookupTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKAttachedSceneLookupTests)

  add_executable(ROCKProviderLeasePolicyTests
    "${ROOT_DIR}/tests/ProviderLeasePolicyTests.cpp"
  )
  target_compile_features(ROCKProviderLeasePolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKProviderLeasePolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKProviderLeasePolicyTests)

  add_executable(ROCKDebugOverlayShapePipelineTests
    "${ROOT_DIR}/tests/DebugOverlayShapePipelineTests.cpp"
    "${SOURCE_DIR}/physics-interaction/debug/DebugOverlayShapeGeometry.cpp"
    "${SOURCE_DIR}/physics-interaction/debug/DebugOverlayShapePipeline.cpp"
  )
  target_compile_features(ROCKDebugOverlayShapePipelineTests PRIVATE cxx_std_23)
  target_include_directories(ROCKDebugOverlayShapePipelineTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKDebugOverlayShapePipelineTests)

  add_executable(ROCKWeaponSceneTraversalTests "${ROOT_DIR}/tests/WeaponSceneTraversalTests.cpp")
  target_compile_features(ROCKWeaponSceneTraversalTests PRIVATE cxx_std_23)
  target_include_directories(ROCKWeaponSceneTraversalTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKWeaponSceneTraversalTests)

  rock_write_test_catalog()


  find_program(ROCK_POWERSHELL_EXECUTABLE NAMES pwsh powershell)
  if(ROCK_POWERSHELL_EXECUTABLE)
    file(GLOB ROCK_SOURCE_BOUNDARY_TESTS CONFIGURE_DEPENDS "${ROOT_DIR}/tests/*SourceTests.ps1")
    foreach(ROCK_SOURCE_BOUNDARY_TEST IN LISTS ROCK_SOURCE_BOUNDARY_TESTS)
      get_filename_component(ROCK_SOURCE_BOUNDARY_TEST_NAME "${ROCK_SOURCE_BOUNDARY_TEST}" NAME_WE)
      add_test(
        NAME ${ROCK_SOURCE_BOUNDARY_TEST_NAME}
        COMMAND "${ROCK_POWERSHELL_EXECUTABLE}"
                -NoProfile
                -ExecutionPolicy Bypass
                -File "${ROCK_SOURCE_BOUNDARY_TEST}"
                -Root "${ROOT_DIR}"
      )
      set_tests_properties(
        ${ROCK_SOURCE_BOUNDARY_TEST_NAME}
        PROPERTIES
          LABELS "source-boundary"
          TIMEOUT 60
      )
    endforeach()
  else()
    message(WARNING "PowerShell was not found; ROCK source-boundary .ps1 tests will not be registered in this configure.")
  endif()
endif()
