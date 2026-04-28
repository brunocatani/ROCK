# ROCK/FRIK Logging Level Pass - 2026-04-27

This pass keeps ROCK on the same spdlog numeric level model already used by FRIK instead of introducing a separate logger. HIGGS exposes a single config-driven log level through its INI, while FRIK already has `[Debug].iLogLevel`; matching that shape lets both DLLs be diagnosed with one convention and keeps production logs readable. The implementation demotes mesh scans, per-frame witnesses, and internal hook diagnostics to debug/trace, leaves lifecycle and user-visible state changes at info, and keeps rejects/failures at warn/error.

Level mapping:

- `0`: trace
- `1`: debug
- `2`: info
- `3`: warn
- `4`: error
- `5`: critical
- `6`: off

ROCK now reads `[Debug] iLogLevel`, `sLogPattern`, and `iLogSampleMilliseconds` from `ROCK.ini`. FRIK's default log pattern was updated to the same timestamp format. ROCK grab popups are now behind `bDebugShowGrabNotifications` so diagnostic display output does not appear during production play.
