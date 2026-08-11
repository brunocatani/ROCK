function(rock_sdk_verify_fo4vr_loader source_path)
  if(NOT EXISTS "${source_path}")
    message(FATAL_ERROR "SDK example loader source is missing: ${source_path}")
  endif()

  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${source_path}")
  file(READ "${source_path}" loader_source)

  foreach(required_text IN ITEMS
      "F4SEPlugin_Query"
      "F4SEPlugin_Load"
      "a_f4se->IsEditor()"
      "REL::Module::IsVR()"
      "REL::Module::get().version()"
      "executableVersion != F4SE::RUNTIME_VR_1_2_72")
    string(FIND "${loader_source}" "${required_text}" found_at)
    if(found_at EQUAL -1)
      message(FATAL_ERROR
        "SDK FO4VR loader regression: '${required_text}' is absent from ${source_path}")
    endif()
  endforeach()

  if(loader_source MATCHES
      "RuntimeVersion\\(\\)[ \\t\\r\\n]*[=!<>]+[ \\t\\r\\n]*F4SE::RUNTIME_(LATEST_)?VR")
    message(FATAL_ERROR
      "SDK FO4VR loader regression: loader RuntimeVersion() was compared with a VR executable constant")
  endif()
  if(loader_source MATCHES
      "F4SE::RUNTIME_(LATEST_)?VR[ \\t\\r\\n]*[=!<>]+[ \\t\\r\\n]*[^;\\n]*RuntimeVersion\\(\\)")
    message(FATAL_ERROR
      "SDK FO4VR loader regression: a VR executable constant was compared with loader RuntimeVersion()")
  endif()
endfunction()
