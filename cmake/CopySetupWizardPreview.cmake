# Copies the setup wizard preview to the folder it is opened from (SOURCE, DESTINATION).
# A preview open there cannot be overwritten, but Windows lets a running exe be renamed:
# it moves aside as ROCKSetupWizardPreview.<time>.old.exe and keeps running, and the new
# build takes its place for the next start. Moved-aside copies are removed once closed.
set(target "${DESTINATION}/ROCKSetupWizardPreview.exe")
file(MAKE_DIRECTORY "${DESTINATION}")
file(GLOB previous "${DESTINATION}/ROCKSetupWizardPreview.*.old.exe")
foreach(file IN LISTS previous)
  # Fails, and stays, while that copy still runs.
  execute_process(COMMAND "${CMAKE_COMMAND}" -E rm -f "${file}" RESULT_VARIABLE ignored OUTPUT_QUIET ERROR_QUIET)
endforeach()
file(COPY_FILE "${SOURCE}" "${target}" RESULT result ONLY_IF_DIFFERENT)
if(result)
  string(TIMESTAMP stamp "%Y%m%d%H%M%S")
  file(RENAME "${target}" "${DESTINATION}/ROCKSetupWizardPreview.${stamp}.old.exe" RESULT moved)
  if(NOT moved)
    file(COPY_FILE "${SOURCE}" "${target}" RESULT result)
  endif()
endif()
if(result)
  message(WARNING "The setup wizard preview was not copied to '${DESTINATION}' (${result}).")
endif()
