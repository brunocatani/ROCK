// Precompiled header for every ROCK translation unit (target_precompile_headers
// in CMakeLists.txt): CommonLibF4VR, F4SE, ROCK's logger and the generated
// Version.h. Anything added here recompiles the whole plugin when it changes.
#pragma once

#define NOMMNOSOUND
#ifndef NOMINMAX
#define NOMINMAX
#endif

#include "F4SE/F4SE.h"
#include "RE/Fallout.h"

#include <REL/Relocation.h>

#include <algorithm>

using namespace std::literals;
using std::max;
using std::min;

#include "rock_support/Logger.h"

#define DLLEXPORT __declspec(dllexport)

#include "Version.h"
