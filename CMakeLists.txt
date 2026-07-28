cmake_minimum_required(VERSION 4.2)


set(NAME "ROCK")
set(FRIENDLY_NAME "ROCK")
set(VERSION 0.5.0)

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
configure_file(${ROOT_DIR}/cmake/resources.rc.in ${BUILD_DIR}/resources.rc @ONLY)
source_group(TREE ${BUILD_DIR} FILES ${BUILD_DIR}/include/Version.h ${BUILD_DIR}/resources.rc)

file(GLOB_RECURSE headers CONFIGURE_DEPENDS "${SOURCE_DIR}/*.h")
file(GLOB_RECURSE sources CONFIGURE_DEPENDS "${SOURCE_DIR}/*.cpp")
file(GLOB_RECURSE inls    CONFIGURE_DEPENDS "${SOURCE_DIR}/*.inl")
source_group(TREE ${ROOT_DIR} FILES ${headers} ${sources} ${inls})

list(APPEND resource_files
  "${ROOT_DIR}/data/config/${NAME}.ini"
)
source_group("Resource Files" FILES ${resource_files})

add_library(${PROJECT_NAME} SHARED
  ${headers}
  ${sources}
  ${inls}
  ${resource_files}
  ${BUILD_DIR}/include/Version.h
  ${BUILD_DIR}/version.rc
  ${BUILD_DIR}/resources.rc
  "${ROOT_DIR}/.clang-format"
  "${ROOT_DIR}/.editorconfig"
)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_23)
target_compile_definitions(${PROJECT_NAME} PRIVATE _UNICODE F4VR_WITH_OPENVR=1)

target_include_directories(${PROJECT_NAME}
  PRIVATE
    ${BUILD_DIR}/include
    ${SOURCE_DIR}
)

target_link_libraries(${PROJECT_NAME}
  PRIVATE
    CommonLibF4::CommonLibF4
    ROCK::OpenVR
    d3d11.lib
    dxgi.lib
    d3dcompiler.lib
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
    "$<$<CONFIG:RELEASE>:/INCREMENTAL:NO;/OPT:REF;/OPT:ICF;/DEBUG:FULL>"
)

string(REPLACE "/Ob2" "/Ob3" CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")

# ROCK separates plugin iteration, verification, and release packaging because the
# normal development loop needs a deployable DLL/PDB without implicitly building
# test executables or archives. Tests and release packages remain explicit paths,
# while fast local builds can still copy the plugin into the configured mod root.
option(POST_BUILD_COPY_PLUGIN "Copy ROCK.dll and ROCK.pdb to configured mod root after building." OFF)
set(COPY_PLUGIN_BASE_PATH "" CACHE STRING "Semicolon-separated mod roots that contain F4SE/Plugins.")
option(ROCK_PACKAGE_RELEASE "Package Release builds into a .7z archive after ROCK is built." ON)

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

option(BUILD_ROCK_TESTS "Build ROCK focused regression tests" OFF)
if(BUILD_ROCK_TESTS)
  enable_testing()

  function(add_rock_test TARGET_NAME)
    add_test(NAME ${TARGET_NAME} COMMAND $<TARGET_FILE:${TARGET_NAME}>)
    set_tests_properties(${TARGET_NAME} PROPERTIES LABELS "policy")
  endfunction()

  add_executable(ROCKGrabInputIntentPolicyTests
    "${ROOT_DIR}/tests/GrabInputIntentPolicyTests.cpp"
  )
  target_compile_features(ROCKGrabInputIntentPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKGrabInputIntentPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKGrabInputIntentPolicyTests)

  add_executable(ROCKInputRemapPolicyTests
    "${ROOT_DIR}/tests/InputRemapPolicyTests.cpp"
  )
  target_compile_features(ROCKInputRemapPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKInputRemapPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKInputRemapPolicyTests)

  add_executable(ROCKWeaponInteractionPolicyTests
    "${ROOT_DIR}/tests/WeaponInteractionPolicyTests.cpp"
  )
  target_compile_features(ROCKWeaponInteractionPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKWeaponInteractionPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKWeaponInteractionPolicyTests)

  add_executable(ROCKInteractionSafetyPolicyTests
    "${ROOT_DIR}/tests/InteractionSafetyPolicyTests.cpp"
  )
  target_compile_features(ROCKInteractionSafetyPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKInteractionSafetyPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKInteractionSafetyPolicyTests)

  add_executable(ROCKFo4VrActorStatePolicyTests
    "${ROOT_DIR}/tests/Fo4VrActorStatePolicyTests.cpp"
  )
  target_compile_features(ROCKFo4VrActorStatePolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKFo4VrActorStatePolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKFo4VrActorStatePolicyTests)

  add_executable(ROCKGrabFingerLocalTransformPolicyTests
    "${ROOT_DIR}/tests/GrabFingerLocalTransformPolicyTests.cpp"
  )
  target_compile_features(ROCKGrabFingerLocalTransformPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKGrabFingerLocalTransformPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKGrabFingerLocalTransformPolicyTests)

  add_executable(ROCKGrabHapticPolicyTests
    "${ROOT_DIR}/tests/GrabHapticPolicyTests.cpp"
  )
  target_compile_features(ROCKGrabHapticPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKGrabHapticPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKGrabHapticPolicyTests)

  add_executable(ROCKDynamicHandCollisionFeedbackPolicyTests
    "${ROOT_DIR}/tests/DynamicHandCollisionFeedbackPolicyTests.cpp"
  )
  target_compile_features(ROCKDynamicHandCollisionFeedbackPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKDynamicHandCollisionFeedbackPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKDynamicHandCollisionFeedbackPolicyTests)

  add_executable(ROCKGrabContactEvidencePolicyTests
    "${ROOT_DIR}/tests/GrabContactEvidencePolicyTests.cpp"
  )
  target_compile_features(ROCKGrabContactEvidencePolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKGrabContactEvidencePolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKGrabContactEvidencePolicyTests)

  add_executable(ROCKHandSemanticContactPolicyTests
    "${ROOT_DIR}/tests/HandSemanticContactPolicyTests.cpp"
  )
  target_compile_features(ROCKHandSemanticContactPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKHandSemanticContactPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKHandSemanticContactPolicyTests)

  add_executable(ROCKGrabContactPatchClusterPolicyTests
    "${ROOT_DIR}/tests/GrabContactPatchClusterPolicyTests.cpp"
  )
  target_compile_features(ROCKGrabContactPatchClusterPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKGrabContactPatchClusterPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKGrabContactPatchClusterPolicyTests)

  add_executable(ROCKContactSignalSubscriptionPolicyTests
    "${ROOT_DIR}/tests/ContactSignalSubscriptionPolicyTests.cpp"
  )
  target_compile_features(ROCKContactSignalSubscriptionPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKContactSignalSubscriptionPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKContactSignalSubscriptionPolicyTests)

  add_executable(ROCKPhysicsCreationGatePolicyTests
    "${ROOT_DIR}/tests/PhysicsCreationGatePolicyTests.cpp"
  )
  target_compile_features(ROCKPhysicsCreationGatePolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKPhysicsCreationGatePolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKPhysicsCreationGatePolicyTests)

  add_executable(ROCKRuntimeStatePolicyTests
    "${ROOT_DIR}/tests/RockRuntimeStatePolicyTests.cpp"
  )
  target_compile_features(ROCKRuntimeStatePolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKRuntimeStatePolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKRuntimeStatePolicyTests)

  add_executable(ROCKGrabPivotAuthorityPolicyTests
    "${ROOT_DIR}/tests/GrabPivotAuthorityPolicyTests.cpp"
  )
  target_compile_features(ROCKGrabPivotAuthorityPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKGrabPivotAuthorityPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKGrabPivotAuthorityPolicyTests)

  add_executable(ROCKGrabMotionControllerPolicyTests
    "${ROOT_DIR}/tests/GrabMotionControllerPolicyTests.cpp"
  )
  target_compile_features(ROCKGrabMotionControllerPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKGrabMotionControllerPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKGrabMotionControllerPolicyTests)

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

  add_executable(ROCKGrabThreePhasePolicyTests
    "${ROOT_DIR}/tests/GrabThreePhasePolicyTests.cpp"
  )
  target_compile_features(ROCKGrabThreePhasePolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKGrabThreePhasePolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKGrabThreePhasePolicyTests)

  add_executable(ROCKGrabPinchPocketPolicyTests
    "${ROOT_DIR}/tests/GrabPinchPocketPolicyTests.cpp"
  )
  target_compile_features(ROCKGrabPinchPocketPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKGrabPinchPocketPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKGrabPinchPocketPolicyTests)

  add_executable(ROCKGeneratedKeyframedBodyDrivePolicyTests
    "${ROOT_DIR}/tests/GeneratedKeyframedBodyDrivePolicyTests.cpp"
  )
  target_compile_features(ROCKGeneratedKeyframedBodyDrivePolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKGeneratedKeyframedBodyDrivePolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKGeneratedKeyframedBodyDrivePolicyTests)

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

  add_executable(ROCKHeldObjectStutterGuardPolicyTests
    "${ROOT_DIR}/tests/HeldObjectStutterGuardPolicyTests.cpp"
  )
  target_compile_features(ROCKHeldObjectStutterGuardPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKHeldObjectStutterGuardPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKHeldObjectStutterGuardPolicyTests)

  add_executable(ROCKEquippedWeaponDropMomentumPolicyTests
    "${ROOT_DIR}/tests/EquippedWeaponDropMomentumPolicyTests.cpp"
  )
  target_compile_features(ROCKEquippedWeaponDropMomentumPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKEquippedWeaponDropMomentumPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKEquippedWeaponDropMomentumPolicyTests)

  add_executable(ROCKHeldBodySetDrivePolicyTests
    "${ROOT_DIR}/tests/HeldBodySetDrivePolicyTests.cpp"
  )
  target_compile_features(ROCKHeldBodySetDrivePolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKHeldBodySetDrivePolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKHeldBodySetDrivePolicyTests)

  add_executable(ROCKSkinnedBodyResolverPolicyTests
    "${ROOT_DIR}/tests/SkinnedBodyResolverPolicyTests.cpp"
  )
  target_compile_features(ROCKSkinnedBodyResolverPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKSkinnedBodyResolverPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKSkinnedBodyResolverPolicyTests)

  add_executable(ROCKMechanicalConnectedBodySetPolicyTests
    "${ROOT_DIR}/tests/MechanicalConnectedBodySetPolicyTests.cpp"
  )
  target_compile_features(ROCKMechanicalConnectedBodySetPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKMechanicalConnectedBodySetPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKMechanicalConnectedBodySetPolicyTests)

  add_executable(ROCKArticulatedAcquisitionPolicyTests
    "${ROOT_DIR}/tests/ArticulatedAcquisitionPolicyTests.cpp"
  )
  target_compile_features(ROCKArticulatedAcquisitionPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKArticulatedAcquisitionPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKArticulatedAcquisitionPolicyTests)

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

  add_executable(ROCKGrabLocomotionJagPolicyTests
    "${ROOT_DIR}/tests/GrabLocomotionJagPolicyTests.cpp"
  )
  target_compile_features(ROCKGrabLocomotionJagPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKGrabLocomotionJagPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKGrabLocomotionJagPolicyTests)

  add_executable(ROCKHandColliderFramePolicyTests
    "${ROOT_DIR}/tests/HandColliderFramePolicyTests.cpp"
  )
  target_compile_features(ROCKHandColliderFramePolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKHandColliderFramePolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKHandColliderFramePolicyTests)

  add_executable(ROCKHknpBodyFramePolicyTests
    "${ROOT_DIR}/tests/HknpBodyFramePolicyTests.cpp"
  )
  target_compile_features(ROCKHknpBodyFramePolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKHknpBodyFramePolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKHknpBodyFramePolicyTests)

  add_executable(ROCKGrabNodeNamePolicyTests
    "${ROOT_DIR}/tests/GrabNodeNamePolicyTests.cpp"
  )
  target_compile_features(ROCKGrabNodeNamePolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKGrabNodeNamePolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKGrabNodeNamePolicyTests)

  add_executable(ROCKShoulderStashPolicyTests
    "${ROOT_DIR}/tests/ShoulderStashPolicyTests.cpp"
  )
  target_compile_features(ROCKShoulderStashPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKShoulderStashPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKShoulderStashPolicyTests)

  add_executable(ROCKMouthConsumePolicyTests
    "${ROOT_DIR}/tests/MouthConsumePolicyTests.cpp"
    "${SOURCE_DIR}/physics-interaction/consume/MouthConsumeDetector.cpp"
  )
  target_compile_features(ROCKMouthConsumePolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKMouthConsumePolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKMouthConsumePolicyTests)

  add_executable(ROCKFeedbackHapticsPolicyTests
    "${ROOT_DIR}/tests/FeedbackHapticsPolicyTests.cpp"
    "${SOURCE_DIR}/physics-interaction/feedback/FeedbackHaptics.cpp"
  )
  target_compile_features(ROCKFeedbackHapticsPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKFeedbackHapticsPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKFeedbackHapticsPolicyTests)

  add_executable(ROCKWeaponCollisionLifecyclePolicyTests
    "${ROOT_DIR}/tests/WeaponCollisionLifecyclePolicyTests.cpp"
  )
  target_compile_features(ROCKWeaponCollisionLifecyclePolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKWeaponCollisionLifecyclePolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKWeaponCollisionLifecyclePolicyTests)

  add_executable(ROCKActiveGrabBodyLifecyclePolicyTests
    "${ROOT_DIR}/tests/ActiveGrabBodyLifecyclePolicyTests.cpp"
  )
  target_compile_features(ROCKActiveGrabBodyLifecyclePolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKActiveGrabBodyLifecyclePolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKActiveGrabBodyLifecyclePolicyTests)

  add_executable(ROCKDynamicMovableStaticGrabPolicyTests
    "${ROOT_DIR}/tests/DynamicMovableStaticGrabPolicyTests.cpp"
  )
  target_compile_features(ROCKDynamicMovableStaticGrabPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKDynamicMovableStaticGrabPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKDynamicMovableStaticGrabPolicyTests)

  add_executable(ROCKSelectionPrecisionPolicyTests
    "${ROOT_DIR}/tests/SelectionPrecisionPolicyTests.cpp"
  )
  target_compile_features(ROCKSelectionPrecisionPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKSelectionPrecisionPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKSelectionPrecisionPolicyTests)

  add_executable(ROCKSelectionBeamPolicyTests
    "${ROOT_DIR}/tests/SelectionBeamPolicyTests.cpp"
  )
  target_compile_features(ROCKSelectionBeamPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKSelectionBeamPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKSelectionBeamPolicyTests)

  add_executable(ROCKNativePlayerCollisionSuppressionPolicyTests
    "${ROOT_DIR}/tests/NativePlayerCollisionSuppressionPolicyTests.cpp"
  )
  target_compile_features(ROCKNativePlayerCollisionSuppressionPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKNativePlayerCollisionSuppressionPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKNativePlayerCollisionSuppressionPolicyTests)

  add_executable(ROCKNativeGrabHapticSuppressionPolicyTests
    "${ROOT_DIR}/tests/NativeGrabHapticSuppressionPolicyTests.cpp"
  )
  target_compile_features(ROCKNativeGrabHapticSuppressionPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKNativeGrabHapticSuppressionPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKNativeGrabHapticSuppressionPolicyTests)

  add_executable(ROCKPipboyEquipPolicyTests
    "${ROOT_DIR}/tests/PipboyEquipPolicyTests.cpp"
  )
  target_compile_features(ROCKPipboyEquipPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKPipboyEquipPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKPipboyEquipPolicyTests)

  add_executable(ROCKEquippedWeaponTransitionPolicyTests
    "${ROOT_DIR}/tests/EquippedWeaponTransitionPolicyTests.cpp"
  )
  target_compile_features(ROCKEquippedWeaponTransitionPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKEquippedWeaponTransitionPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKEquippedWeaponTransitionPolicyTests)

  add_executable(ROCKHeldWeaponInstantTransitionPolicyTests
    "${ROOT_DIR}/tests/HeldWeaponInstantTransitionPolicyTests.cpp"
  )
  target_compile_features(ROCKHeldWeaponInstantTransitionPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKHeldWeaponInstantTransitionPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKHeldWeaponInstantTransitionPolicyTests)

  add_executable(ROCKWeaponInventoryStackSelectionPolicyTests
    "${ROOT_DIR}/tests/WeaponInventoryStackSelectionPolicyTests.cpp"
  )
  target_compile_features(ROCKWeaponInventoryStackSelectionPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKWeaponInventoryStackSelectionPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKWeaponInventoryStackSelectionPolicyTests)

  add_executable(ROCKAuthoredWeaponGripCapturePolicyTests
    "${ROOT_DIR}/tests/AuthoredWeaponGripCapturePolicyTests.cpp"
  )
  target_compile_features(ROCKAuthoredWeaponGripCapturePolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKAuthoredWeaponGripCapturePolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKAuthoredWeaponGripCapturePolicyTests)

  add_executable(ROCKNativeIdleGripPreharvestPolicyTests
    "${ROOT_DIR}/tests/NativeIdleGripPreharvestPolicyTests.cpp"
  )
  target_compile_features(ROCKNativeIdleGripPreharvestPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKNativeIdleGripPreharvestPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKNativeIdleGripPreharvestPolicyTests)

  add_executable(ROCKAuthoredWeaponGripAuthorityPolicyTests
    "${ROOT_DIR}/tests/AuthoredWeaponGripAuthorityPolicyTests.cpp"
  )
  target_compile_features(ROCKAuthoredWeaponGripAuthorityPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKAuthoredWeaponGripAuthorityPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKAuthoredWeaponGripAuthorityPolicyTests)

  add_executable(ROCKPhysicsCallbackQuiescenceGateTests
    "${ROOT_DIR}/tests/PhysicsCallbackQuiescenceGateTests.cpp"
  )
  target_compile_features(ROCKPhysicsCallbackQuiescenceGateTests PRIVATE cxx_std_23)
  target_include_directories(ROCKPhysicsCallbackQuiescenceGateTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKPhysicsCallbackQuiescenceGateTests)

  add_executable(ROCKDynamicHandTwinGeometryPolicyTests
    "${ROOT_DIR}/tests/DynamicHandTwinGeometryPolicyTests.cpp"
  )
  target_compile_features(ROCKDynamicHandTwinGeometryPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKDynamicHandTwinGeometryPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKDynamicHandTwinGeometryPolicyTests)

  add_executable(ROCKDynamicHandCollisionTransitionPolicyTests
    "${ROOT_DIR}/tests/DynamicHandCollisionTransitionPolicyTests.cpp"
  )
  target_compile_features(ROCKDynamicHandCollisionTransitionPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKDynamicHandCollisionTransitionPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKDynamicHandCollisionTransitionPolicyTests)

  set(ROCK_COMMONLIB_TEST_TARGETS
    ROCKGrabInputIntentPolicyTests
    ROCKInputRemapPolicyTests
    ROCKWeaponInteractionPolicyTests
    ROCKInteractionSafetyPolicyTests
    ROCKGrabFingerLocalTransformPolicyTests
    ROCKGrabHapticPolicyTests
    ROCKDynamicHandCollisionFeedbackPolicyTests
    ROCKGrabContactEvidencePolicyTests
    ROCKHandSemanticContactPolicyTests
    ROCKGrabContactPatchClusterPolicyTests
    ROCKContactSignalSubscriptionPolicyTests
    ROCKPhysicsCreationGatePolicyTests
    ROCKRuntimeStatePolicyTests
    ROCKGrabPivotAuthorityPolicyTests
    ROCKGrabMotionControllerPolicyTests
    ROCKGrabAuthorityFramePolicyTests
    ROCKTransformMathDoublePrecisionPolicyTests
    ROCKGrabThreePhasePolicyTests
    ROCKGrabPinchPocketPolicyTests
    ROCKGeneratedKeyframedBodyDrivePolicyTests
    ROCKGeneratedBodyContactRegistryPolicyTests
    ROCKHavokTimingFixPolicyTests
    ROCKHeldObjectStutterGuardPolicyTests
    ROCKEquippedWeaponDropMomentumPolicyTests
    ROCKHeldBodySetDrivePolicyTests
    ROCKSkinnedBodyResolverPolicyTests
    ROCKMechanicalConnectedBodySetPolicyTests
    ROCKArticulatedAcquisitionPolicyTests
    ROCKGrabAuthorityProxyPolicyTests
    ROCKGrabAuthoritySourceClockPolicyTests
    ROCKGrabLocomotionJagPolicyTests
    ROCKHandColliderFramePolicyTests
    ROCKHknpBodyFramePolicyTests
    ROCKGrabNodeNamePolicyTests
    ROCKShoulderStashPolicyTests
    ROCKMouthConsumePolicyTests
    ROCKFeedbackHapticsPolicyTests
    ROCKWeaponCollisionLifecyclePolicyTests
    ROCKActiveGrabBodyLifecyclePolicyTests
    ROCKDynamicMovableStaticGrabPolicyTests
    ROCKSelectionPrecisionPolicyTests
    ROCKSelectionBeamPolicyTests
    ROCKNativePlayerCollisionSuppressionPolicyTests
    ROCKNativeGrabHapticSuppressionPolicyTests
    ROCKPipboyEquipPolicyTests
    ROCKEquippedWeaponTransitionPolicyTests
    ROCKHeldWeaponInstantTransitionPolicyTests
    ROCKWeaponInventoryStackSelectionPolicyTests
    ROCKAuthoredWeaponGripCapturePolicyTests
    ROCKNativeIdleGripPreharvestPolicyTests
    ROCKAuthoredWeaponGripAuthorityPolicyTests
    ROCKPhysicsCallbackQuiescenceGateTests
    ROCKDynamicHandTwinGeometryPolicyTests
    ROCKDynamicHandCollisionTransitionPolicyTests
  )
  # Every policy executable uses the same CommonLib-heavy precompiled header and
  # config stub. Build those once, then reuse the PCH artifact and support library
  # instead of paying that cost independently for every focused test target.
  add_library(ROCKPolicyTestSupport STATIC
    "${ROOT_DIR}/tests/RockConfigPolicyStub.cpp"
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
  target_precompile_headers(ROCKPolicyTestSupport PRIVATE "${ROOT_DIR}/tests/CommonLibPolicyPCH.h")
  target_link_libraries(ROCKPolicyTestSupport PUBLIC spdlog::spdlog mmio::mmio)
  if(TARGET xbyak::xbyak)
    target_link_libraries(ROCKPolicyTestSupport PUBLIC xbyak::xbyak)
  endif()

  foreach(ROCK_COMMONLIB_TEST_TARGET IN LISTS ROCK_COMMONLIB_TEST_TARGETS)
    target_precompile_headers(${ROCK_COMMONLIB_TEST_TARGET} REUSE_FROM ROCKPolicyTestSupport)
    target_link_libraries(${ROCK_COMMONLIB_TEST_TARGET} PRIVATE ROCKPolicyTestSupport)
  endforeach()
  add_custom_target(ROCKPolicyTestBinaries DEPENDS ${ROCK_COMMONLIB_TEST_TARGETS})

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

  add_executable(ROCKDebugOverlayRuntimeSettingsTests
    "${ROOT_DIR}/tests/DebugOverlayRuntimeSettingsTests.cpp"
  )
  target_compile_features(ROCKDebugOverlayRuntimeSettingsTests PRIVATE cxx_std_23)
  target_include_directories(ROCKDebugOverlayRuntimeSettingsTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKDebugOverlayRuntimeSettingsTests)

  add_executable(ROCKDebugOverlayLineBatchTests
    "${ROOT_DIR}/tests/DebugOverlayLineBatchTests.cpp"
  )
  target_compile_features(ROCKDebugOverlayLineBatchTests PRIVATE cxx_std_23)
  target_include_directories(ROCKDebugOverlayLineBatchTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKDebugOverlayLineBatchTests)

  add_executable(ROCKProviderDebugOverlayRuntimeTests
    "${ROOT_DIR}/tests/ProviderDebugOverlayRuntimeTests.cpp"
    "${SOURCE_DIR}/api/ProviderDebugOverlayRuntime.cpp"
  )
  target_compile_features(ROCKProviderDebugOverlayRuntimeTests PRIVATE cxx_std_23)
  target_include_directories(ROCKProviderDebugOverlayRuntimeTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKProviderDebugOverlayRuntimeTests)

  add_executable(ROCKProviderApiAbiTests
    "${ROOT_DIR}/tests/ProviderApiAbiTests.cpp"
  )
  target_compile_features(ROCKProviderApiAbiTests PRIVATE cxx_std_23)
  target_include_directories(ROCKProviderApiAbiTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKProviderApiAbiTests)

  add_executable(ROCKProviderApiNegotiationTests
    "${ROOT_DIR}/tests/ProviderApiNegotiationTests.cpp"
  )
  target_compile_features(ROCKProviderApiNegotiationTests PRIVATE cxx_std_23)
  target_include_directories(ROCKProviderApiNegotiationTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKProviderApiNegotiationTests)

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

  add_executable(ROCKProviderLeasePolicyTests
    "${ROOT_DIR}/tests/ProviderLeasePolicyTests.cpp"
  )
  target_compile_features(ROCKProviderLeasePolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKProviderLeasePolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKProviderLeasePolicyTests)

  add_executable(ROCKDebugOverlayShapeGeometryTests
    "${ROOT_DIR}/tests/DebugOverlayShapeGeometryTests.cpp"
    "${SOURCE_DIR}/physics-interaction/debug/DebugOverlayShapeGeometry.cpp"
  )
  target_compile_features(ROCKDebugOverlayShapeGeometryTests PRIVATE cxx_std_23)
  target_include_directories(ROCKDebugOverlayShapeGeometryTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKDebugOverlayShapeGeometryTests)

  add_executable(ROCKDebugOverlayShapePipelineTests
    "${ROOT_DIR}/tests/DebugOverlayShapePipelineTests.cpp"
    "${SOURCE_DIR}/physics-interaction/debug/DebugOverlayShapeGeometry.cpp"
    "${SOURCE_DIR}/physics-interaction/debug/DebugOverlayShapePipeline.cpp"
  )
  target_compile_features(ROCKDebugOverlayShapePipelineTests PRIVATE cxx_std_23)
  target_include_directories(ROCKDebugOverlayShapePipelineTests PRIVATE ${SOURCE_DIR})
  target_link_libraries(ROCKDebugOverlayShapePipelineTests PRIVATE d3d11.lib)
  add_rock_test(ROCKDebugOverlayShapePipelineTests)

  add_executable(ROCKDebugOverlayShaderCompileTests
    "${ROOT_DIR}/tests/DebugOverlayShaderCompileTests.cpp"
  )
  target_compile_features(ROCKDebugOverlayShaderCompileTests PRIVATE cxx_std_23)
  target_include_directories(ROCKDebugOverlayShaderCompileTests PRIVATE ${SOURCE_DIR})
  target_link_libraries(ROCKDebugOverlayShaderCompileTests PRIVATE d3dcompiler.lib)
  add_rock_test(ROCKDebugOverlayShaderCompileTests)

  add_dependencies(
    ROCKPolicyTestBinaries
    ROCKDebugOverlayFrameAdmissionTests
    ROCKDebugOverlayLineBatchTests
    ROCKProviderDebugOverlayRuntimeTests
    ROCKProviderApiAbiTests
    ROCKProviderApiNegotiationTests
    ROCKProviderExternalBodyRegistryTests
    ROCKProviderTouchGrabRegistryTests
    ROCKTouchGrabMathTests
    ROCKProviderLeasePolicyTests
    ROCKDebugOverlayShapeGeometryTests
    ROCKDebugOverlayShapePipelineTests
    ROCKDebugOverlayRuntimeSettingsTests
    ROCKDebugOverlaySnapshotPoolTests
    ROCKDebugOverlayShaderCompileTests
  )

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
      set_tests_properties(${ROCK_SOURCE_BOUNDARY_TEST_NAME} PROPERTIES LABELS "source-boundary")
    endforeach()
  else()
    message(WARNING "PowerShell was not found; ROCK source-boundary .ps1 tests will not be registered in this configure.")
  endif()
endif()
