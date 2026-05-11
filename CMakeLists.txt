cmake_minimum_required(VERSION 4.2)


set(NAME "ROCK")
set(FRIENDLY_NAME "ROCK")
set(VERSION 0.1.0)

message(">>> Start build for '${NAME}' v${VERSION}")

set(ROOT_DIR  "${CMAKE_CURRENT_SOURCE_DIR}")
set(SOURCE_DIR "${ROOT_DIR}/src")
set(BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}")

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


if(DEFINED F4VR_COMMON_FRAMEWORK_PATH AND NOT F4VR_COMMON_FRAMEWORK_PATH STREQUAL "")
  set(F4VR_COMMON_FRAMEWORK_PATH_SELECTED ${F4VR_COMMON_FRAMEWORK_PATH} CACHE STRING "" FORCE)
  message(">>> Using F4VR-CommonFramework from custom variable: '${F4VR_COMMON_FRAMEWORK_PATH_SELECTED}'")
else()
  set(F4VR_COMMON_FRAMEWORK_PATH_SELECTED "${ROOT_DIR}/external/F4VR-CommonFramework" CACHE STRING "" FORCE)
  message(">>> Using F4VR-CommonFramework in external: '${F4VR_COMMON_FRAMEWORK_PATH_SELECTED}'")
endif()
if(NOT EXISTS "${F4VR_COMMON_FRAMEWORK_PATH_SELECTED}/src/ModBase.h")
  message(FATAL_ERROR ">> F4VR-CommonFramework path not found in '${F4VR_COMMON_FRAMEWORK_PATH_SELECTED}'!\n >> Did you init submodules? (git submodule update --init --recursive)\n >> or set F4VR_COMMON_FRAMEWORK_PATH?")
endif()

project(${NAME} VERSION ${VERSION} LANGUAGES CXX)
message(">>> Building project '${PROJECT_NAME}' v:${PROJECT_VERSION}")
message(">>> Using found F4VR-CommonFramework in: '${F4VR_COMMON_FRAMEWORK_PATH_SELECTED}'")

add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
set(Boost_USE_STATIC_RUNTIME OFF CACHE BOOL "")
set(Boost_USE_STATIC_LIBS ON)
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION OFF)
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_DEBUG OFF)

add_subdirectory(${F4VR_COMMON_FRAMEWORK_PATH_SELECTED} "F4VR-CommonFramework" EXCLUDE_FROM_ALL)


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
target_compile_definitions(${PROJECT_NAME} PRIVATE _UNICODE)

target_include_directories(${PROJECT_NAME}
  PRIVATE
    ${BUILD_DIR}/include
    ${SOURCE_DIR}
)

target_link_libraries(${PROJECT_NAME}
  PRIVATE
    F4VRCommon::framework
    d3d11.lib
    dxgi.lib
    d3dcompiler.lib
)

find_package(cpptrace CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(mmio CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE cpptrace::cpptrace)

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

if(POST_BUILD_COPY_PLUGIN)
  if(DEFINED COPY_PLUGIN_BASE_PATH)
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

option(BUILD_ROCK_TESTS "Build ROCK focused regression tests" OFF)
if(BUILD_ROCK_TESTS)
  enable_testing()

  function(add_rock_test TARGET_NAME)
    add_test(NAME ${TARGET_NAME} COMMAND $<TARGET_FILE:${TARGET_NAME}>)
  endfunction()

  add_executable(ROCKTransformConventionTests
    "${ROOT_DIR}/tests/TransformConventionTests.cpp"
  )
  target_compile_features(ROCKTransformConventionTests PRIVATE cxx_std_23)
  target_include_directories(ROCKTransformConventionTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKTransformConventionTests)

  add_executable(ROCKSelectionHighlightPolicyTests
    "${ROOT_DIR}/tests/SelectionHighlightPolicyTests.cpp"
  )
  target_compile_features(ROCKSelectionHighlightPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKSelectionHighlightPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKSelectionHighlightPolicyTests)

  add_executable(ROCKWorldOriginDiagnosticsTests
    "${ROOT_DIR}/tests/WorldOriginDiagnosticsTests.cpp"
  )
  target_compile_features(ROCKWorldOriginDiagnosticsTests PRIVATE cxx_std_23)
  target_include_directories(ROCKWorldOriginDiagnosticsTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKWorldOriginDiagnosticsTests)

  add_executable(ROCKBodyFramePolicyTests
    "${ROOT_DIR}/tests/BodyFramePolicyTests.cpp"
  )
  target_compile_features(ROCKBodyFramePolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKBodyFramePolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKBodyFramePolicyTests)

  add_executable(ROCKInputRemapPolicyTests
    "${ROOT_DIR}/tests/InputRemapPolicyTests.cpp"
  )
  target_compile_features(ROCKInputRemapPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKInputRemapPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKInputRemapPolicyTests)

  add_executable(ROCKGrabInputIntentPolicyTests
    "${ROOT_DIR}/tests/GrabInputIntentPolicyTests.cpp"
  )
  target_compile_features(ROCKGrabInputIntentPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKGrabInputIntentPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKGrabInputIntentPolicyTests)

  add_executable(ROCKDebugControllerPolicyTests
    "${ROOT_DIR}/tests/DebugControllerPolicyTests.cpp"
  )
  target_compile_features(ROCKDebugControllerPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKDebugControllerPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKDebugControllerPolicyTests)

  add_executable(ROCKDebugOverlayPolicyTests
    "${ROOT_DIR}/tests/DebugOverlayPolicyTests.cpp"
  )
  target_compile_features(ROCKDebugOverlayPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKDebugOverlayPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKDebugOverlayPolicyTests)

  add_executable(ROCKHandStateMachineTests
    "${ROOT_DIR}/tests/HandStateMachineTests.cpp"
    "${SOURCE_DIR}/physics-interaction/hand/HandInteractionStateMachine.cpp"
  )
  target_compile_features(ROCKHandStateMachineTests PRIVATE cxx_std_23)
  target_include_directories(ROCKHandStateMachineTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKHandStateMachineTests)

  add_executable(ROCKHavokRuntimePolicyTests
    "${ROOT_DIR}/tests/HavokRuntimePolicyTests.cpp"
    "${ROOT_DIR}/tests/HavokRuntimePolicyScaleStub.cpp"
  )
  target_compile_features(ROCKHavokRuntimePolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKHavokRuntimePolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKHavokRuntimePolicyTests)

  add_executable(ROCKUnsafeNativeToggleBoundaryTests
    "${ROOT_DIR}/tests/UnsafeNativeToggleBoundaryTests.cpp"
  )
  target_compile_features(ROCKUnsafeNativeToggleBoundaryTests PRIVATE cxx_std_23)
  target_compile_definitions(ROCKUnsafeNativeToggleBoundaryTests PRIVATE
    "ROCK_PHYSICS_INTERACTION_SOURCE_DIR=\"${SOURCE_DIR}/physics-interaction\""
  )
  add_rock_test(ROCKUnsafeNativeToggleBoundaryTests)

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

  add_executable(ROCKGrabNodeNamePolicyTests
    "${ROOT_DIR}/tests/GrabNodeNamePolicyTests.cpp"
  )
  target_compile_features(ROCKGrabNodeNamePolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKGrabNodeNamePolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKGrabNodeNamePolicyTests)

  add_executable(ROCKProviderBoundaryTests
    "${ROOT_DIR}/tests/ProviderBoundaryTests.cpp"
  )
  target_compile_features(ROCKProviderBoundaryTests PRIVATE cxx_std_23)
  target_include_directories(ROCKProviderBoundaryTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKProviderBoundaryTests)

  add_executable(ROCKPhysicsLifecycleStateTests
    "${ROOT_DIR}/tests/PhysicsLifecycleStateTests.cpp"
  )
  target_compile_features(ROCKPhysicsLifecycleStateTests PRIVATE cxx_std_23)
  target_include_directories(ROCKPhysicsLifecycleStateTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKPhysicsLifecycleStateTests)

  add_executable(ROCKContactPipelinePolicyTests
    "${ROOT_DIR}/tests/ContactPipelinePolicyTests.cpp"
  )
  target_compile_features(ROCKContactPipelinePolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKContactPipelinePolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKContactPipelinePolicyTests)

  add_executable(ROCKContactRuntimePolicyTests
    "${ROOT_DIR}/tests/ContactRuntimePolicyTests.cpp"
  )
  target_compile_features(ROCKContactRuntimePolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKContactRuntimePolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKContactRuntimePolicyTests)

  add_executable(ROCKSoftContactMathTests
    "${ROOT_DIR}/tests/SoftContactMathTests.cpp"
  )
  target_compile_features(ROCKSoftContactMathTests PRIVATE cxx_std_23)
  target_include_directories(ROCKSoftContactMathTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKSoftContactMathTests)

  add_executable(ROCKNativeContactEvidenceTests
    "${ROOT_DIR}/tests/NativeContactEvidenceTests.cpp"
  )
  target_compile_features(ROCKNativeContactEvidenceTests PRIVATE cxx_std_23)
  target_include_directories(ROCKNativeContactEvidenceTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKNativeContactEvidenceTests)

  add_executable(ROCKBodyContactRuntimeTests
    "${ROOT_DIR}/tests/BodyContactRuntimeTests.cpp"
  )
  target_compile_features(ROCKBodyContactRuntimeTests PRIVATE cxx_std_23)
  target_include_directories(ROCKBodyContactRuntimeTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKBodyContactRuntimeTests)

  add_executable(ROCKShoulderStashPolicyTests
    "${ROOT_DIR}/tests/ShoulderStashPolicyTests.cpp"
  )
  target_compile_features(ROCKShoulderStashPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKShoulderStashPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKShoulderStashPolicyTests)

  add_executable(ROCKContactTargetIdentityPolicyTests
    "${ROOT_DIR}/tests/ContactTargetIdentityPolicyTests.cpp"
  )
  target_compile_features(ROCKContactTargetIdentityPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKContactTargetIdentityPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKContactTargetIdentityPolicyTests)

  add_executable(ROCKWeaponCollisionLifecyclePolicyTests
    "${ROOT_DIR}/tests/WeaponCollisionLifecyclePolicyTests.cpp"
  )
  target_compile_features(ROCKWeaponCollisionLifecyclePolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKWeaponCollisionLifecyclePolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKWeaponCollisionLifecyclePolicyTests)

  add_executable(ROCKSeeThroughScopesPolicyTests
    "${ROOT_DIR}/tests/SeeThroughScopesPolicyTests.cpp"
  )
  target_compile_features(ROCKSeeThroughScopesPolicyTests PRIVATE cxx_std_23)
  target_include_directories(ROCKSeeThroughScopesPolicyTests PRIVATE ${SOURCE_DIR})
  add_rock_test(ROCKSeeThroughScopesPolicyTests)

  set(ROCK_COMMONLIB_TEST_TARGETS
    ROCKTransformConventionTests
    ROCKSelectionHighlightPolicyTests
    ROCKWorldOriginDiagnosticsTests
    ROCKBodyFramePolicyTests
    ROCKInputRemapPolicyTests
    ROCKGrabInputIntentPolicyTests
    ROCKHandStateMachineTests
    ROCKHavokRuntimePolicyTests
    ROCKGrabFingerLocalTransformPolicyTests
    ROCKGrabHapticPolicyTests
    ROCKGrabNodeNamePolicyTests
    ROCKProviderBoundaryTests
    ROCKPhysicsLifecycleStateTests
    ROCKContactPipelinePolicyTests
    ROCKContactRuntimePolicyTests
    ROCKSoftContactMathTests
    ROCKNativeContactEvidenceTests
    ROCKBodyContactRuntimeTests
    ROCKShoulderStashPolicyTests
    ROCKContactTargetIdentityPolicyTests
    ROCKWeaponCollisionLifecyclePolicyTests
    ROCKSeeThroughScopesPolicyTests
  )
  foreach(ROCK_COMMONLIB_TEST_TARGET IN LISTS ROCK_COMMONLIB_TEST_TARGETS)
    target_include_directories(${ROCK_COMMONLIB_TEST_TARGET}
      PRIVATE
        ${BUILD_DIR}/include
        "${F4VR_COMMON_FRAMEWORK_PATH_SELECTED}/src"
        "${F4VR_COMMON_FRAMEWORK_PATH_SELECTED}/external/openvr"
        "${COMMON_LIB_F4VR_PATH_SELECTED}/CommonLibF4/include"
    )
    target_compile_definitions(${ROCK_COMMONLIB_TEST_TARGET}
      PRIVATE
        WINVER=0x0601
        _WIN32_WINNT=0x0601
        F4SE_SUPPORT_XBYAK=1
        ENABLE_FALLOUT_VR=1
        ROCK_POLICY_TEST_NO_RUNTIME_LOGGER=1
    )
    target_precompile_headers(${ROCK_COMMONLIB_TEST_TARGET} PRIVATE "${ROOT_DIR}/tests/CommonLibPolicyPCH.h")
    target_link_libraries(${ROCK_COMMONLIB_TEST_TARGET} PRIVATE spdlog::spdlog mmio::mmio)
    if(TARGET xbyak::xbyak)
      target_link_libraries(${ROCK_COMMONLIB_TEST_TARGET} PRIVATE xbyak::xbyak)
    endif()
    target_sources(${ROCK_COMMONLIB_TEST_TARGET} PRIVATE "${ROOT_DIR}/tests/RockConfigPolicyStub.cpp")
  endforeach()

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
    endforeach()
  else()
    message(WARNING "PowerShell was not found; ROCK source-boundary .ps1 tests will not be registered in this configure.")
  endif()
endif()
