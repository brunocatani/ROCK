#pragma once

// PCH.h -- Precompiled header for the ROCK F4SE plugin.
//
// WHY: Mirrors FRIK's PCH.h but stripped of FRIK-specific includes. ROCK is a separate
// DLL that links the same F4VR-CommonFramework and CommonLibF4VR, so it needs the same
// base includes (F4SE, RE, REL) plus the framework's Logger for consistent logging
// infrastructure across both DLLs.

#define NOMMNOSOUND

#include "F4SE/F4SE.h"
#include "RE/Fallout.h"

#include <REL/Relocation.h>

#include <algorithm>

using namespace std::literals;

// F4VR-CommonFramework's f4sevr/Common.h uses bare min/max calls expecting Windows macros,
// but CommonLibF4VR defines NOMINMAX. Provide std::min/max as unqualified names.
using std::min;
using std::max;

#include "Logger.h"

using namespace f4cf;

#define DLLEXPORT __declspec(dllexport)

#include "Version.h"
