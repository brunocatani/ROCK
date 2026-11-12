# Post-build script: writes the two files of a release into build/package, a .zip
# of the mod that installs as it is and the readme beside it. Release builds only.

string(TOLOWER "${CONFIG}" CONFIG_LOWER)
if(NOT CONFIG_LOWER STREQUAL "release")
  message("Skipping post-build packaging: not a release build: '${CONFIG}'")
  return()
endif()


include("${ROOT_DIR}/cmake/VerifyDistributionInputs.cmake")
rock_verify_distribution_inputs(TREES "${ROOT_DIR}/data/mod")

set(PACKAGE_DIR "${BUILD_DIR}/package")
set(PACKAGE_STAGE_DIR "${PACKAGE_DIR}/stageing")
set(PACKAGE_STAGE_PLUGINS_DIR "${PACKAGE_STAGE_DIR}/F4SE/Plugins")
string(TIMESTAMP PACKAGE_DATE "%Y%m%d")
set(TARGET_ZIP "${PACKAGE_DIR}/${PROJECT_FRIENDLY_NAME} - v${PROJECT_VERSION} - ${PACKAGE_DATE}.zip")

message("Packaging release build into '${TARGET_ZIP}'")
file(REMOVE_RECURSE "${PACKAGE_STAGE_DIR}")
file(MAKE_DIRECTORY "${PACKAGE_STAGE_PLUGINS_DIR}")
if(EXISTS "${ROOT_DIR}/data/mod")
  file(COPY "${ROOT_DIR}/data/mod/" DESTINATION "${PACKAGE_STAGE_DIR}")
endif()
file(COPY "${TARGET_FILE}" DESTINATION "${PACKAGE_STAGE_PLUGINS_DIR}")

# SteamVR legacy binding templates for non-Quest controllers. Players install them
# through SteamVR; they never deploy into a game or mod folder.
file(COPY "${ROOT_DIR}/steamvr_bindings" DESTINATION "${PACKAGE_STAGE_DIR}")

# Only files the game or the player's install use go in the zip. The PDB, the
# license, readmes and development files (provider headers, example INIs, docs)
# stay out; mod authors take include/ROCK from the release tag.
file(GLOB_RECURSE STAGED_FILES LIST_DIRECTORIES false "${PACKAGE_STAGE_DIR}/*")
foreach(STAGED_FILE IN LISTS STAGED_FILES)
  cmake_path(GET STAGED_FILE FILENAME STAGED_NAME)
  string(TOLOWER "${STAGED_NAME}" STAGED_NAME)
  if(STAGED_NAME MATCHES "\\.(pdb|md|txt|ini|h)$" OR STAGED_NAME MATCHES "^(license|readme|copying)")
    message(FATAL_ERROR "package.cmake: the zip must not hold '${STAGED_FILE}'")
  endif()
endforeach()

file(REMOVE "${TARGET_ZIP}")
execute_process(COMMAND ${CMAKE_COMMAND} -E tar cf "${TARGET_ZIP}" --format=zip -- .
                WORKING_DIRECTORY "${PACKAGE_STAGE_DIR}"
                COMMAND_ERROR_IS_FATAL ANY)
file(COPY_FILE "${ROOT_DIR}/README.md" "${PACKAGE_DIR}/README.md")
message("Packaged '${TARGET_ZIP}' with README.md beside it")
