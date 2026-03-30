#pragma once

/*
 * Standalone policy tests include CommonLibF4VR headers but do not run inside
 * the F4SE plugin loader. They still need CommonLib's compile-time prelude for
 * REL, REX, RTTI, VTABLE, and stl aliases, while avoiding leaked Windows macros
 * that corrupt later CommonLib declarations. This header is force-included only
 * for policy test targets; the production DLL continues to use src/PCH.h.
 */

#define NOMMNOSOUND
#ifndef NOMINMAX
#define NOMINMAX
#endif

#include "F4SE/Impl/PCH.h"

#ifdef near
#undef near
#endif
#ifdef far
#undef far
#endif
#ifdef MEM_RELEASE
#undef MEM_RELEASE
#endif
#ifdef MEM_COMMIT
#undef MEM_COMMIT
#endif
#ifdef MEM_RESERVE
#undef MEM_RESERVE
#endif
#ifdef PAGE_EXECUTE_READ
#undef PAGE_EXECUTE_READ
#endif
#ifdef PAGE_EXECUTE_READWRITE
#undef PAGE_EXECUTE_READWRITE
#endif
#ifdef MAX_PATH
#undef MAX_PATH
#endif
