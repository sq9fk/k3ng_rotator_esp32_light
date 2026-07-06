/* ---------------------- Features and Options - you must configure this !! ------------------------------------------------

  This fork targets a single board (Arduino Nano ESP32) and a trimmed
  "light" feature set: Yaesu GS-232B control-port emulation over both USB
  serial and WiFi, azimuth-only rotation via a pulse/impulse position sensor
  (with a standard rotary encoder kept available as an alternative), and a
  4-bit parallel LCD (with a generic I2C LCD kept available as an
  alternative). Moon/sun/satellite tracking, GPS, RTC/clock, master/remote-
  slave operation, the stepper motor driver, all other position sensors, and
  all other display variants have been removed outright (not just left
  disabled) — see CLAUDE.md for the rationale.

*/

/* main features */

// #define FEATURE_ELEVATION_CONTROL      // uncomment this for AZ/EL rotators (elevation logic remains in the codebase but is untested/unmaintained in this fork)
#define FEATURE_YAESU_EMULATION           // Yaesu GS-232 emulation on control port (USB serial and WiFi)

#define LANGUAGE_ENGLISH         // all languages customized in rotator_language.h
// #define LANGUAGE_SPANISH
// #define LANGUAGE_CZECH
// #define LANGUAGE_ITALIAN
// #define LANGUAGE_PORTUGUESE_BRASIL
// #define LANGUAGE_GERMAN
// #define LANGUAGE_FRENCH
// #define LANGUAGE_DUTCH
// #define LANGUAGE_NORWEGIAN_BOKMAAL

/* position sensor - pick one */
#define FEATURE_AZ_POSITION_PULSE_INPUT     // pulse/impulse azimuth position sensor (active default)
// #define FEATURE_AZ_POSITION_ROTARY_ENCODER  // standard 2-phase rotary encoder (alternative to pulse input)

// #define FEATURE_EL_POSITION_PULSE_INPUT     // only meaningful together with FEATURE_ELEVATION_CONTROL above

/* display - pick one. NOTE: you must also flip the matching define at the top of rotator_k3ngdisplay.h to match */
#define FEATURE_4_BIT_LCD_DISPLAY        // classic 4-bit parallel LCD wired directly to GPIO (active default)
// #define FEATURE_YWROBOT_I2C_DISPLAY   // generic PCF8574 I2C LCD backpack (alternative to 4-bit LCD)

/* network control port (Nano ESP32 has WiFi, not an Ethernet jack) */
#define FEATURE_WIFI

/* less often used features and options */
#define OPTION_GS_232B_EMULATION          // comment this out to default to Yaesu GS-232A emulation
// #define OPTION_SERIAL_HELP_TEXT        // Yaesu help command prints help
// #define OPTION_C_COMMAND_SENDS_AZ_AND_EL  // uncomment this when using Yaesu emulation with Ham Radio Deluxe
// #define OPTION_DELAY_C_CMD_OUTPUT         // uncomment this when using Yaesu emulation with Ham Radio Deluxe
// #define OPTION_BUTTON_RELEASE_NO_SLOWDOWN  // disables slowdown when CW or CCW button is released, or stop button is depressed
// #define OPTION_MORE_SERIAL_CHECKS
// #define OPTION_ALLOW_ROTATIONAL_AND_CONFIGURATION_CMDS_AT_BOOT_UP // if disabled, rotational and configuration commands will be ignored on the serial port for the first 10 seconds after boot up
// #define OPTION_SAVE_MEMORY_EXCLUDE_EXTENDED_COMMANDS
// #define OPTION_SAVE_MEMORY_EXCLUDE_BACKSLASH_CMDS
// #define OPTION_CLI_VT100   // use VT100 terminal emulation
// #define OPTION_BLINK_OVERLAP_LED
// #define OPTION_ROTATION_INDICATOR_PIN

/* rotary encoder options (only relevant if FEATURE_AZ_POSITION_ROTARY_ENCODER is enabled above) */
#define OPTION_ENCODER_HALF_STEP_MODE
#define OPTION_ENCODER_ENABLE_PULLUPS          // define to enable weak pullups on rotary encoder pins
#define OPTION_AZ_POSITION_ROTARY_ENCODER_HARD_LIMIT // stop azimuth at lower and upper limit rather than rolling over

/* pulse input options (only relevant if FEATURE_AZ_POSITION_PULSE_INPUT is enabled above) */
#define OPTION_AZ_POSITION_PULSE_HARD_LIMIT  // stop azimuth at lower and upper limit rather than rolling over
#define OPTION_POSITION_PULSE_INPUT_PULLUPS  // define to enable weak pullups on position pulse inputs
// #define OPTION_PULSE_IGNORE_AMBIGUOUS_PULSES // ignore pulses that arrive when no rotation is active

/* display options */
#define OPTION_DISPLAY_STATUS
#define OPTION_DISPLAY_HEADING
#define OPTION_DISPLAY_HEADING_AZ_ONLY
// #define OPTION_DISPLAY_DIRECTION_STATUS   // LCD N, W, E, S, NW, etc. direction indicator
#define OPTION_DISPLAY_VERSION_ON_STARTUP  // code provided by Paolo, IT9IPQ
// #define OPTION_LCD_HEADING_FIELD_FIXED_DECIMAL_PLACE
