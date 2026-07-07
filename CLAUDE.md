# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

`k3ng_rotator_esp32_light`: a fork of the [K3NG Rotator Controller](https://github.com/k3ng/k3ng_rotator_controller)
trimmed to a single board and a "light" feature set, built with PlatformIO instead of the Arduino IDE.

- **Board**: Arduino Nano ESP32 only. All other hardware profiles (M0UPU, WB6KCN, WB6KCN_K3NG, TEST, EA4TX_ARS_USB)
  and AVR-only code (the stepper motor driver, which needs TimerOne/TimerFive/digitalWriteFast — none of which work
  on ESP32) have been removed.
- **Active feature set**: Yaesu GS-232B control-port emulation over both USB serial and WiFi (a new `FEATURE_WIFI`
  TCP control port, since the Nano ESP32 has WiFi but no Ethernet jack), azimuth-only rotation via a pulse/impulse
  position sensor (with a standard rotary encoder kept available as a compile-time alternative), and a 4-bit
  parallel LCD (with a generic I2C LCD backpack kept available as a compile-time alternative).
- **Removed outright**, including their source code (not just left disabled): moon/sun/satellite tracking (and the
  Nextion touchscreen display system, which was satellite/tracking-adjacent and equally unused), GPS, RTC/clock,
  master/remote-slave operation (serial or Ethernet), Easycom/DCU-1 protocol emulation, park/autopark, timed-buffer
  rotation, joystick control, limit-sense, azimuth/elevation rotation-stall detection, the audible alert, analog
  output pins, ancillary pin control (`\N`/`\F`/`\W` backslash commands), azimuth/elevation calibration correction,
  all position sensors other than pulse input and the standard rotary encoder, and all display variants other than
  4-bit LCD and the generic I2C LCD. The vendored `libraries/`, `tle/`, and `k3ng_rotator_controller/Nextion/`
  directories were deleted wholesale since nothing in the active feature set needs them.
- **Kept, deliberately**: `FEATURE_AZ_PRESET_ENCODER`/`FEATURE_EL_PRESET_ENCODER` (rotary-encoder azimuth/elevation
  preset support) and their `check_az_preset_potentiometer()`/`check_preset_encoders()` code are inactive in the
  current `rotator_features.h` but were intentionally *not* removed during cleanup — leave this code in place.
- **Elevation control** (`FEATURE_ELEVATION_CONTROL`) is left in place but permanently undefined/unused rather than
  physically deleted — see "Inert dead code" below for why.

Full upstream user documentation lives in the project wiki: https://github.com/k3ng/k3ng_rotator_controller/wiki —
much of it (tracking, GPS/RTC, master/slave, other sensors/displays) no longer applies to this fork.

## Build / compile / flash

This is a PlatformIO project (`platformio.ini` at repo root, sketch under `src/`).

- `pio run` — compile.
- `pio run -t upload` — compile and flash to a connected Nano ESP32.
- `pio device monitor` — serial monitor (9600 baud, matches `CONTROL_PORT_BAUD_RATE`).
- Before flashing: edit `src/rotator_settings.h` to set `WIFI_SSID`/`WIFI_PASSWORD`, and edit `src/rotator_pins.h` to
  set `az_position_pulse_pin` (or the rotary encoder pins, if you switch `FEATURE_AZ_POSITION_PULSE_INPUT` for
  `FEATURE_AZ_POSITION_ROTARY_ENCODER` in `src/rotator_features.h`) to match your actual wiring — these ship as `0`
  (disabled) or carried-over placeholder values from the original AVR pinout and are marked `TODO` where they need
  per-build confirmation.
- "Testing" a change means compiling for the `nano_esp32` PlatformIO env and, ideally, flashing real hardware — there
  is no unit test harness or CI. There is no way to verify rotation behavior, WiFi control, or LCD output without
  physical hardware; don't claim a change "works" from static analysis or compilation alone.

## Architecture

### Everything is one translation unit, configured at compile time

`src/k3ng_rotator_controller.ino` (~10k lines) is effectively the entire program. It `#include`s a series of `.h`
files that are really just extensions of the same file (not independent modules with their own compilation), all
guarded by preprocessor `#ifdef`/`#if defined(...)` blocks keyed off `FEATURE_*` and `OPTION_*` macros. There is no
runtime plugin system — enabling/disabling capability is entirely a recompile-time decision. When making changes:

- Assume any function or global you touch may be wrapped in `#ifdef FEATURE_X` / `#endif` and only exists in some
  configurations. Grep for the guarding feature before assuming a symbol is always available.
- New functionality should follow the same pattern: gate it behind a new or existing `FEATURE_`/`OPTION_` macro
  rather than making it unconditional, unless it's a genuine bug fix.

### Feature configuration

`src/rotator_features.h` is the single place that turns capabilities on/off (there's no more hardware-profile picker
— `src/rotator_hardware.h` hardcodes `CONTROL_PORT_SERIAL_PORT_CLASS` to `HardwareSerial` for this one board).
`src/rotator_dependencies.h` cross-checks the resulting combination and emits `#error`s for invalid combinations
(e.g. must pick exactly one AZ position sensor) — read it before adding a feature flag that interacts with existing
ones. Pin assignments (`src/rotator_pins.h`) use `0` as the sentinel for "not connected/disabled" — a pin macro must
be checked against `0`, not just for definedness, in most cases.

### Inert dead code

Almost every removed feature (moon/sun/satellite tracking, the Nextion display, GPS/RTC/clock, master/remote-slave,
Ethernet, Easycom/DCU-1 emulation, park/autopark, timed-buffer, joystick, limit-sense, rotation-stall detection,
audible alert, analog output pins, ancillary pin control, azimuth/elevation calibration correction, most position
sensors and display variants, the A2 absolute encoder, most of the stepper motor driver) was physically deleted from
`src/k3ng_rotator_controller.ino`, not just left disabled — including standalone service functions, global
declarations, settings blocks, and `loop()`/`setup()` call sites. This was done incrementally with a `pio run`
compile check after nearly every deletion, which is what made it safe to remove code woven into shared functions (a
dangling reference shows up as a compile error immediately — missing libraries, undefined macros, etc.): sensor
blocks inside `read_azimuth()`, tracking blocks inside `process_backslash_command()`/`update_lcd_display()`, the
Easycom/DCU-1 parsing branches inside `check_serial()`, and several elevation-only functions
(`read_elevation()`, `current_el_state()`, `el_check_operation_timeout()`, `update_el_variable_outputs()`,
`el_position_pulse_interrupt_handler()`, the elevation-button section of `check_buttons()`) were all removed this way
once the compiler confirmed no other code depended on them.

**Elevation control** (`FEATURE_ELEVATION_CONTROL`) is the one substantial exception remaining, left in place but
permanently undefined rather than fully excised: what's left of it is genuinely interleaved *inside* shared functions
that this fork's azimuth-only configuration does use (`rotator()`, `service_rotation()`, `read_headings()`, the `\W`
timed-buffer command, etc.), as single-line conditionals inside much larger active logic rather than clean standalone
blocks — removing it would mean restructuring those functions, not deleting dead text. A few equally-interleaved
remnants of other removed features share this fate (e.g. a `row_override`/`parsed_elevation` pair that's now
"set but not used" rather than fully dead — see git history for details). `FEATURE_AZ_PRESET_ENCODER`/
`FEATURE_EL_PRESET_ENCODER` are inactive but were deliberately *not* touched (kept for potential future use, not a
removal candidate). Since none of the genuinely-dead `FEATURE_*` macros above are ever defined in
`rotator_features.h`, the preprocessor excludes all of it at zero compile/runtime cost regardless of whether it was
physically deleted or left inert — deleting it further is a readability nice-to-have, not a correctness fix.

### Runtime shape

- `setup()`: `initialize_serial()` → `initialize_peripherals()` (includes `initialize_wifi()`) →
  `read_settings_from_eeprom()` → `initialize_pins()` → `read_azimuth(0)` → `initialize_display()` →
  `initialize_rotary_encoders()` → `initialize_interrupts()` → `run_this_once()`.
- `loop()` is a long, mostly-flat sequence of `check_*()` / `service_*()` calls, almost every one wrapped in an
  `#ifdef FEATURE_X`. `service_rotation()` and `read_headings()` are called repeatedly throughout the loop (not just
  once) because rotation state and position must stay current across the other, potentially slow, checks.
  `service_wifi()` is called alongside `check_serial()` — both feed the same `process_yaesu_command()`/
  `process_backslash_command()` dispatch, distinguished by a `source_port` bitmask (`CONTROL_PORT0` vs `WIFI_PORT0`
  in `rotator.h`).
- State machine constants for azimuth motion (`az_state`) live in `rotator.h` (`IDLE`, `SLOW_START_CW`, `NORMAL_CW`,
  `INITIALIZE_DIR_CHANGE_TO_CW`, etc.) — these encode a multi-step slow-start/normal-rotation/slow-down/
  direction-change sequence, not simple on/off states. The elevation equivalents (`el_state` and friends) are also
  defined but unused in this fork's active configuration.
- Persistent settings (calibration, last position, azimuth display mode) live in the `config_t` struct (in the .ino,
  near the top after includes) and are saved/loaded from EEPROM. **ESP32's `EEPROM` library is flash-emulated**:
  `read_settings_from_eeprom()` calls `EEPROM.begin(sizeof(configuration) + 16)` once before any access, and
  `write_settings_to_eeprom()` calls `EEPROM.commit()` after writing — both were added for this port and don't exist
  upstream (AVR's real EEPROM needs neither). Many `config_t` members are still behind now-permanently-false
  `#if defined(FEATURE_X)` guards, so the struct layout matches what upstream would produce with only this fork's
  active features enabled.
- `rotator.h` centralizes shared macros/constants (state enums, debug process IDs, request-queue states) used across
  the whole .ino — check here first when a bare constant is unfamiliar.

### Supporting files

- `src/rotator_debug.cpp`/`.h`: `DebugClass`, a thin print/println/write wrapper used for debug output throughout the
  sketch; `rotator_debug_log_activation.h` controls which debug categories are compiled in.
- `src/rotator_k3ngdisplay.cpp`/`.h`: LCD/display abstraction layer; only the 4-bit and generic-I2C (YWROBOT) paths
  are wired up in this fork.

## Working in this repo

- Prefer additive, `#ifdef`-gated changes over restructuring — this codebase (and the dead code described above) is
  fragile to blind refactors that can't be compile-tested against real hardware.
- When changing shared/ungated code (`loop()`'s call sequence, `config_t`, `rotator.h` constants, the WiFi/serial
  control-port dispatch), consider the effect on both active AZ-sensor options (pulse input and rotary encoder) and
  both display options (4-bit LCD and I2C), since either can be toggled on via `rotator_features.h` without other
  code changes.
- There's no formatter/linter config in this repo; match the surrounding file's existing indentation and brace style
  exactly rather than reformatting.
