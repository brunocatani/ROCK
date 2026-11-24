# The base folders may include each other and the headers directly under
# physics-interaction/, never a feature folder. Run by the ROCKBaseLayer test:
#   cmake -DSOURCE_DIR=<src> -P CheckBaseLayer.cmake
# RockConfig.h is the one exception: settings are read everywhere.
set(BASE_FOLDERS native collision performance timing)
set(PROBLEMS "")
set(CHECKED 0)

file(GLOB FILES "${SOURCE_DIR}/physics-interaction/*.h" "${SOURCE_DIR}/physics-interaction/*.cpp")
foreach(FOLDER IN ITEMS rock_support physics-interaction/native physics-interaction/collision
    physics-interaction/performance physics-interaction/timing)
  file(GLOB_RECURSE FOUND "${SOURCE_DIR}/${FOLDER}/*.h" "${SOURCE_DIR}/${FOLDER}/*.cpp" "${SOURCE_DIR}/${FOLDER}/*.inl")
  list(APPEND FILES ${FOUND})
endforeach()

foreach(FILE IN LISTS FILES)
  math(EXPR CHECKED "${CHECKED} + 1")
  file(STRINGS "${FILE}" LINES REGEX "^[ \t]*#[ \t]*include[ \t]*\"")
  foreach(LINE IN LISTS LINES)
    string(REGEX REPLACE "^[^\"]*\"([^\"]+)\".*$" "\\1" HEADER "${LINE}")
    if(HEADER MATCHES "^physics-interaction/([^/]+)/")
      if(NOT CMAKE_MATCH_1 IN_LIST BASE_FOLDERS)
        string(APPEND PROBLEMS "${FILE}: ${HEADER}\n")
      endif()
    elseif(HEADER MATCHES "^(api|config|devbench|mods-config|setup-wizard)/")
      string(APPEND PROBLEMS "${FILE}: ${HEADER}\n")
    endif()
  endforeach()
endforeach()

if(CHECKED EQUAL 0)
  message(FATAL_ERROR "No base files found under '${SOURCE_DIR}'.")
endif()
if(NOT PROBLEMS STREQUAL "")
  message(FATAL_ERROR "Base folders include feature folders:\n${PROBLEMS}")
endif()
message(STATUS "${CHECKED} base files include no feature folder.")
