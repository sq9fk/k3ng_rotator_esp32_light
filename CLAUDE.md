# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

K3NG Rotator Controller: an Arduino sketch that turns an Arduino/AVR (or Teensy, Due, Maple Mini, etc.) into an antenna
rotator controller, emulating Yaesu GS-232A/B, Easycom, and DCU-1 control protocols so it can be driven by hamlib
rotctl/rotctld, HRD, N1MM, PstRotator, and similar programs. It supports azimuth-only or azimuth/elevation rotators,
many position sensor types (potentiometer, rotary/incremental encoders, digital compasses, accelerometers, absolute
encoders), LCD/Nextion displays, GPS/RTC clock sync, moon/sun/satellite tracking, and a master/remote-slave mode for
split sensor placement.

Full user documentation lives in the project wiki: https://github.com/k3ng/k3ng_rotator_controller/wiki — the code
in this repo intentionally does not restate it.

There is no build system, package manifest, or test suite in this repo — it is compiled exclusively via the Arduino
IDE (or `arduino-cli`) as a sketch.

## Build / compile / "test"

- Open `k3ng_rotator_controller/k3ng_rotator_controller.ino` in the Arduino IDE.
- Third-party libraries the sketch depends on (HMC5883L, LSM303, RTClib, TimeLib, TimerOne/TimerFive, P13, etc.) live
  in `libraries/` in this repo and must be copied/symlinked into the Arduino sketchbook `libraries/` folder (or
  installed via Library Manager) before compiling — they are not fetched automatically.
- Select the correct board (Uno, Mega, Leonardo/Micro, Teensy, Due, Maple Mini, etc.) in the IDE before verifying —
  some `#include`s in the .ino (e.g. `CONTROL_PORT_SERIAL_PORT_CLASS` in `rotator_hardware.h`) are board-conditional.
- "Testing" a change means: verify (compile) the sketch with a representative combination of `FEATURE_`/`OPTION_`
  defines enabled (see below), and ideally upload to real hardware — there is no unit test harness or CI.
- Compiling with the wrong combination of features is a common source of errors; the dependency-checking file
  (`rotator_dependencies.h`) is designed to fail the build early with a clear `#error` when features conflict, rather
  than fail silently.

## Architecture

### Everything is one translation unit, configured at compile time

`k3ng_rotator_controller.ino` (~22k lines) is effectively the entire program. It `#include`s a series of `.h` files
that are really just extensions of the same file (not independent modules with their own compilation), all guarded
by preprocessor `#ifdef`/`#if defined(...)` blocks keyed off `FEATURE_*` and `OPTION_*` macros. There is no runtime
plugin system — enabling/disabling capability is entirely a recompile-time decision. When making changes:

- Assume any function or global you touch may be wrapped in `#ifdef FEATURE_X` / `#endif` and only exists in some
  configurations. Grep for the guarding feature before assuming a symbol is always available.
- New functionality should follow the same pattern: gate it behind a new or existing `FEATURE_`/`OPTION_` macro
  rather than making it unconditional, unless it's a genuine bug fix.

### Configuration file layering

Two independent axes of compile-time configuration, both resolved at the top of the .ino via includes:

1. **Hardware profile** (`rotator_hardware.h`): uncommenting `HARDWARE_M0UPU`, `HARDWARE_WB6KCN`, etc. switches in a
   matched trio of `rotator_features_<profile>.h` / `rotator_pins_<profile>.h` / `rotator_settings_<profile>.h`
   instead of the generic `rotator_features.h` / `rotator_pins.h` / `rotator_settings.h`. If any `HARDWARE_*` profile
   is active, `HARDWARE_CUSTOM` is defined and the generic files are excluded — don't edit the generic files for a
   profile-based build, edit the profile-specific ones.
2. **Feature/option flags** (`rotator_features.h`): every capability (protocol emulation, sensors, displays, tracking,
   master/slave) is a commented-out `#define FEATURE_...` or `#define OPTION_...`. Users uncomment what they need.
   `rotator_dependencies.h` then cross-checks the resulting combination and emits `#error`s for invalid/incompatible
   combinations (e.g. can't enable both Yaesu and Easycom emulation, moon/sun tracking requires elevation + clock,
   etc.) and derives implied macros (e.g. any I2C LCD feature implies `FEATURE_I2C_LCD` implies `FEATURE_WIRE_SUPPORT`).
   Read `rotator_dependencies.h` before adding a new feature flag that interacts with existing ones.

Pin assignments (`rotator_pins.h`) use `0` as the sentinel for "not connected/disabled" — a pin macro must be checked
against `0`, not just for definedness, in most cases.

### Runtime shape

- `setup()` (near the end of the .ino): `initialize_serial()` → `initialize_peripherals()` → `read_settings_from_eeprom()`
  → `initialize_pins()` → `read_azimuth(0)` → `initialize_display()` → `initialize_rotary_encoders()` →
  `initialize_interrupts()` → `run_this_once()`.
- `loop()` is a long, mostly-flat sequence of `check_*()` / `service_*()` calls, almost every one wrapped in an
  `#ifdef FEATURE_X`. `service_rotation()` and `read_headings()` are called repeatedly throughout the loop (not just
  once) because rotation state and position must stay current across the other, potentially slow, checks.
- State machines for azimuth/elevation motion (`az_state`, `el_state`) use named constants defined in `rotator.h`
  (`IDLE`, `SLOW_START_CW`, `NORMAL_CW`, `INITIALIZE_DIR_CHANGE_TO_CW`, etc.) — these encode a multi-step
  slow-start/normal-rotation/slow-down/direction-change sequence, not simple on/off states.
- Persistent settings (calibration, last position, park position, autopark, tracking thresholds, stepper motor state)
  live in the `config_t` struct (in the .ino, near the top after includes) and are saved/loaded from EEPROM; many
  struct members are themselves behind `#if defined(FEATURE_X)` guards, so the EEPROM layout is configuration-dependent
  — a struct built with one feature set will not read back correctly under a different one.
- `rotator.h` centralizes shared macros/constants (state enums, debug process IDs, Nextion API capability bitmasks,
  request-queue states) used across the whole .ino — check here first when a bare constant is unfamiliar.
- `rotator_command_processing.h` is currently empty; command parsing for the Yaesu/Easycom/DCU-1 protocols lives
  directly in the .ino, not in that header.

### Supporting files

- `rotator_debug.cpp`/`.h`: `DebugClass`, a thin print/println/write wrapper used for debug output throughout the
  sketch; `rotator_debug_log_activation.h` controls which debug categories are compiled in.
- `rotator_k3ngdisplay.cpp`/`.h`: LCD/display abstraction layer supporting several physical display types (classic 4-bit,
  various I2C LCDs, Nextion) behind a common interface.
- `rotator_clock_and_gps.h`, `rotator_moon_and_sun.h`, `rotator_stepper.h`, `rotator_ethernet.h`, `rotator_language.h`:
  each holds the logic (or in the case of `rotator_language.h`, translated string tables) for one optional subsystem;
  all are only meaningfully populated when their corresponding `FEATURE_*` is enabled.
- `k3ng_rotator_controller/Nextion/`: pre-built Nextion HMI touchscreen display projects (`.HMI` files) and fonts,
  used with `FEATURE_NEXTION_DISPLAY` — these are binary display-designer projects, not source to edit directly.
- `libraries/`: vendored copies of third-party Arduino libraries this sketch depends on (sensor drivers, LCD drivers,
  timers, RTC, time). Treat these as third-party code — avoid modifying unless fixing a bug that affects this
  project's usage, since they're otherwise meant to track upstream.

## Working in this repo

- Prefer additive, `#ifdef`-gated changes over restructuring, since any refactor risks silently breaking one of the
  many hardware/feature combinations that can't all be tested locally.
- When changing shared/ungated code (e.g. `loop()`'s call sequence, `config_t`, `rotator.h` constants), consider the
  effect across *all* feature combinations, not just the one you're testing with.
- There's no formatter/linter config in this repo; match the surrounding file's existing indentation and brace style
  exactly rather than reformatting.
