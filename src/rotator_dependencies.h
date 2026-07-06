
/* ---------------------- dependency checking - don't touch this unless you know what you are doing ---------------------*/

#if defined(FEATURE_DCU_1_EMULATION) && defined(FEATURE_ELEVATION_CONTROL)
  #error "FEATURE_ELEVATION_CONTROL isn't supported with FEATURE_DCU_1_EMULATION"
#endif

#if defined(FEATURE_EL_POSITION_PULSE_INPUT) && !defined(FEATURE_ELEVATION_CONTROL)
  #undef FEATURE_EL_POSITION_PULSE_INPUT
#endif

#if !defined(FEATURE_AZ_POSITION_PULSE_INPUT) && !defined(FEATURE_AZ_POSITION_ROTARY_ENCODER)
  #error "You must specify one AZ position sensor feature (FEATURE_AZ_POSITION_PULSE_INPUT or FEATURE_AZ_POSITION_ROTARY_ENCODER)"
#endif

#if defined(FEATURE_ELEVATION_CONTROL) && !defined(FEATURE_EL_POSITION_PULSE_INPUT)
  #error "You must specify one EL position sensor feature (FEATURE_EL_POSITION_PULSE_INPUT)"
#endif

#if defined(FEATURE_AZ_POSITION_ROTARY_ENCODER) && !defined(FEATURE_ROTARY_ENCODER_SUPPORT)
  #define FEATURE_ROTARY_ENCODER_SUPPORT
#endif

#if defined(FEATURE_4_BIT_LCD_DISPLAY) || defined(FEATURE_YWROBOT_I2C_DISPLAY)
  #define FEATURE_LCD_DISPLAY
#endif

#if defined(FEATURE_YWROBOT_I2C_DISPLAY)
  #define FEATURE_I2C_LCD
  #define FEATURE_WIRE_SUPPORT
#endif

#if defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION) || defined(FEATURE_DCU_1_EMULATION)
  #define CONTROL_PROTOCOL_EMULATION
#endif
