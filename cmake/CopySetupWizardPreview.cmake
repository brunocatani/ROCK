# Copies the setup wizard preview to the folder it is opened from (SOURCE, DESTINATION).
# A preview open there cannot be replaced: that warns and leaves the build to succeed,
# and the next build copies it.
file(MAKE_DIRECTORY "${DESTINATION}")
file(COPY_FILE "${SOURCE}" "${DESTINATION}/ROCKSetupWizardPreview.exe" RESULT result ONLY_IF_DIFFERENT)
if(result)
  message(WARNING "The setup wizard preview was not copied to '${DESTINATION}' (${result}). "
    "Close the preview that is open there; the next build copies it.")
endif()
