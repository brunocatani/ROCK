# Post-Build event script to package the mod into a .7z file ONLY for release builds

string(TOLOWER "${CONFIG}" CONFIG_LOWER)
if(NOT CONFIG_LOWER STREQUAL "release")
  message("Skipping post-build packaging: not a release build: '${CONFIG}'")
  return()
endif()

if(NOT DEFINED RPS_SDK_ROOT OR RPS_SDK_ROOT STREQUAL "")
  message(FATAL_ERROR "RPS_SDK_ROOT is required to package ROCK's public SDK.")
endif()
set(ROCK_PUBLIC_SDK_DIR "${RPS_SDK_ROOT}/SDK/ROCK")
foreach(REQUIRED_SDK_PATH IN ITEMS
    "${ROCK_PUBLIC_SDK_DIR}/README.md"
    "${ROCK_PUBLIC_SDK_DIR}/include/ROCKProviderApi.h"
    "${ROCK_PUBLIC_SDK_DIR}/include/ROCKApi.h"
    "${ROCK_PUBLIC_SDK_DIR}/docs/PublicApi.md"
    "${ROCK_PUBLIC_SDK_DIR}/examples/CMakeLists.txt")
  if(NOT EXISTS "${REQUIRED_SDK_PATH}")
    message(FATAL_ERROR "Independent RPS_SDK is incomplete; missing '${REQUIRED_SDK_PATH}'.")
  endif()
endforeach()

foreach(API_HEADER IN ITEMS ROCKProviderApi.h ROCKApi.h)
  file(SHA256 "${ROOT_DIR}/src/api/${API_HEADER}" SOURCE_HEADER_HASH)
  file(SHA256 "${ROCK_PUBLIC_SDK_DIR}/include/${API_HEADER}" SDK_HEADER_HASH)
  if(NOT SOURCE_HEADER_HASH STREQUAL SDK_HEADER_HASH)
    message(FATAL_ERROR
      "Independent SDK header '${API_HEADER}' does not match ROCK's runtime ABI header.")
  endif()
endforeach()

include("${ROOT_DIR}/cmake/VerifyDistributionInputs.cmake")
rock_verify_distribution_inputs(TREES "${ROOT_DIR}/data/mod" "${ROCK_PUBLIC_SDK_DIR}")

set(PACKAGE_DIR "${BUILD_DIR}/package")
set(PACKAGE_STAGE_DIR "${PACKAGE_DIR}/stageing")
set(PACKAGE_STAGE_PLUGINS_DIR "${PACKAGE_STAGE_DIR}/F4SE/Plugins")
set(PACKAGE_STAGE_SDK_DIR "${PACKAGE_STAGE_DIR}/SDK")
string(TIMESTAMP PACKAGE_DATE "%Y%m%d")
set(TARGET_ZIP "${PACKAGE_DIR}/${PROJECT_FRIENDLY_NAME} - v${PROJECT_VERSION} - ${PACKAGE_DATE}.7z")

message("Packaging release build into '${TARGET_ZIP}'")
file(REMOVE_RECURSE "${PACKAGE_STAGE_DIR}")
file(MAKE_DIRECTORY "${PACKAGE_STAGE_PLUGINS_DIR}")
file(MAKE_DIRECTORY "${PACKAGE_STAGE_SDK_DIR}")
if(EXISTS "${ROOT_DIR}/data/mod")
  file(COPY "${ROOT_DIR}/data/mod/" DESTINATION "${PACKAGE_STAGE_DIR}")
endif()
file(COPY "${TARGET_FILE}" DESTINATION "${PACKAGE_STAGE_PLUGINS_DIR}")
file(COPY "${TARGET_PDB_FILE}" DESTINATION "${PACKAGE_STAGE_PLUGINS_DIR}")

file(COPY "${ROCK_PUBLIC_SDK_DIR}" DESTINATION "${PACKAGE_STAGE_SDK_DIR}")

execute_process(COMMAND ${CMAKE_COMMAND} -E tar cf "${TARGET_ZIP}" --format=7zip -- . WORKING_DIRECTORY "${PACKAGE_STAGE_DIR}")
