/* Arduino Rotator Controller

   Anthony Good
   K3NG
   anthony.good@gmail.com
  
  Documentation: https://github.com/k3ng/k3ng_rotator_controller/wiki

  Support: https://groups.io/g/radioartisan

  YouTube Channel: https://www.youtube.com/channel/UC5o8UM1-heT5kJbwnJRkUYg
  
   Code contributions, testing, ideas, bug fixes, hardware, support, encouragement, and/or bourbon provided by:
     John W3SA
     Gord VO1GPK
     Anthony M0UPU
     Pete VE5VA
     Marcin SP5IOU
     Hjalmar OZ1JHM
     Sverre LA3ZA
     Bent OZ1CT
     Erick WB6KCN
     Norm N3YKF
     Jan OK2ZAW
     Jim M0CKE
     Mike AD0CZ
     Paolo IT9IPQ
     Antonio IZ7DDA
     Johan PA3FPQ
     Jurgen PE1LWT
     Gianfranco IZ8EWD 
     Jasper PA2J
     Máximo EA1DDO
     Matt VK5ZM
     ...and others
  
   Translations provided by
     Máximo EA1DDO
     Jan OK2ZAW
     Paolo IT9IPQ
     Ismael PY4PI
     Robert DL5ROB
     David ON4BDS


   (If you contributed something and I forgot to put your name and call in here, *please* email me!)
  
 ***************************************************************************************************************

 
    Copyright (C) 2021  Anthony Good, K3NG

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

 ***************************************************************************************************************
  
  
  
  All copyrights are the property of their respective owners
  
 
 
  Full documentation is currently located here: https://github.com/k3ng/k3ng_rotator_controller/wiki

      Rules for using this code:

          Rule #1: Read the documentation at https://github.com/k3ng/k3ng_rotator_controller/wiki
  
          Rule #2: Refer to rule #1.

          Rule #3: Help others.

          Rule #4: Have fun.

  NOTICE: THIS SOFTWARE IS PROVIDED FREE OF CHARGE.  NO WARRANTY IS EXPESSED OR IMPLIED.  IT IS UP TO THE USER OF THIS SOFTWARE
  IS RESPONSIBLE FOR THE SAFETY OF ANY SYSTEM THAT IS DEPLOYED USING THIS SOFTWARE, ANY VARIANT OF THIS SOFTWARE, OR ANY DIRIVATIVE WORK.
  USERS OF THIS SOFTWARE MUST DEPLOY SAFETY INTERLOCKS INDEPENDENT OF THIS SOFTWARE TO ENSURE THE SAFETY OF PEOPLE, ANIMALS, AND
  PROPERTY.  THIS MAY INCLUDE BUT IS NOT LIMITED TO MECHANICAL INTERLOCKS WHICH INTERRUPT POWER TO ROTATIONAL HARDWARE IN THE EVENT
  PHYSICAL LIMITS OF ROTATIONAL HARDWARE IS REACHED OR IF THE SOFTWARE BECOMES UNRESPONSIVE OR IF SYSTEM CONTROL IS LOST FOR ANY
  REASON.

  THIS SOFTWARE MAY NOT BE IN ANY SYSTEM THAT IS RESPONSIBLE FOR HUMAN SAFETY, NUCLEAR POWER, PROJECTILES, OR AVIATION.


    Recent Update History

    Prior to 2.0.2015040401 (May still need to be documented in the wiki):

        DEBUG_POLOLU_LSM303_CALIBRATION (rotator_features.h)
        OPTION_HAMLIB_EASYCOM_AZ_EL_COMMAND_HACK
        LANGUAGE_PORTUGUESE_BRASIL (thanks Ismael, PY4PI)
        AZ_INCREMENTAL_ENCODER_ZERO_PULSE_POSITION
        EL_INCREMENTAL_ENCODER_ZERO_PULSE_POSITION
        OPTION_BLINK_OVERLAP_LED and OPTION_OVERLAP_LED_BLINK_MS setting
        FEATURE_SUN_PUSHBUTTON_AZ_EL_CALIBRATION and FEATURE_MOON_PUSHBUTTON_AZ_EL_CALIBRATION; pin_sun_pushbutton_calibration, pin_moon_pushbutton_calibration
        Working on FEATURE_AUTOCORRECT
        OPTION_EL_PULSE_DEBOUNCE code - (thanks Gianfranco, IZ8EWD)
        #define EL_POSITION_PULSE_DEBOUNCE 500  // in ms
        OPTION_HH12_10_BIT_READINGS in hh12.h (thanks Johan PA3FPQ)
        #define BRAKE_ACTIVE_STATE HIGH
        #define BRAKE_INACTIVE_STATE LOW
        OPTION_SCANCON_2RMHF3600_INC_ENCODER - thanks Jasper, PA2J
        Added remote slave commands:
          RC - read coordinates (returns RCxx.xxxx -xxx.xxxx)
          GS - query GPS status (returns GS0 (no sync) or GS1 (sync))  
        OPTION_SYNC_MASTER_COORDINATES_TO_SLAVE 
        reset_pin
        OPTION_RESET_METHOD_JMP_ASM_0
        Change /E command to do setup() for system reset

    2.0.2015040401 Changed configuration.azimuth_offset to use raw_azimuth rather than azimuth for calculation

    2.0.2015040402 Fixed bug with compiling FEATURE_MASTER_WITH_ETHERNET_SLAVE without FEATURE_CLOCK

    2.0.2015050401 Fixed bug with WNW being display on LCD direction indicator rather than WSW (Thanks Radek, OK2NMA)

    2.0.2015051301
      Fixed bug with remote slave AZ and EL commands not returning decimal places (i.e. xxx.000000)
      Working on remote unit double backslash commands

    2.0.2015051901 LANGUAGE_GERMAN (Thanks Ronny, DM2RM)  (documented in wiki)

    2.0.2015052501
      Working on SEI Bus and A2 Encoders
      Working on remote unit double backslash commands

    2.0.2015061601  
      Working on converting over LCD display code to k3ngdisplay library
      #define DISPLAY_DEGREES_STRING "\xDF"
      last_az_incremental_encoder_position & az_incremental_encoder_position changed to long
      k3ng_remote_rotator_controller class

    2.0.2015070301
      Fixed compile error involving clock_temp_string in display code when compiling multiple clock display widgets is attempted  
      Still working on new display code and local/remote unit code

    2.0.2015070302
      FEATURE_AZ_POSITION_INCREMENTAL_ENCODER conversion to long data types (Thanks Daniel Cussen)

    2.0.2015070401
      gps_sync pin bug fixed  

    2.0.2015071201
      FEATURE_YWROBOT_I2C_DISPLAY (code provided by Máximo EA1DDO)

    2.0.2015071701  
      FEATURE_AZ_POSITION_INCREMENTAL_ENCODER code fixed (Thanks Daniel Cussen)

    2.0.2015090401
      Breaking out portions of ino file into .h files...
        #include "rotator_clock_and_gps.h"
        #include "rotator_command_processing.h" 
        #include "rotator_moon_and_sun.h"
        #include "rotator_ethernet.h"
        #include "rotator_stepper.h"

    2.0.2015090402
        #include "rotator_language.h"
        OPTION_SAVE_MEMORY_EXCLUDE_REMOTE_CMDS
        /?FS command - Full Status Report

    2.0.2015090601
      Updates to rotator_language.h
      Fixed k3ngdisplay.h / LiquidCrystal.h compilation problems with Arduino IDE
      Integrated DebugClass (debug.h and debug.cpp) contributed from Matt VK5ZM


    2.0.2015092001
      LANGUAGE_FRENCH (contributed by Marc-Andre, VE2EVN) 
      fixed issue with rotator_analog_az inferring with other pins if defined but not used 

    2.0.2015092002
      Fixed issue with compiling DEBUG_GPS 

    2.0.2015111501
      Fixed issues with compilation under Arduino 1.6.6 (gave up on various include files... I'll do things the right way in the rewrite)   

    2.0.2015111502
      LANGUAGE_DUTCH courtesy of David, ON4BDS

    2.0.2015121801
      Fixed bug in update_lcd_display() with display always showing DOWN with elevation rotation (Thanks, UA9OLB)

    2.0.2015122001
      Created OPTION_REVERSE_AZ_HH12_AS5045 and OPTION_REVERSE_EL_HH12_AS5045

    2.0.2015122801
      Bug fixes involving OPTION_CLOCK_ALWAYS_HAVE_HOUR_LEADING_ZERO (Thanks, UA9OLB)  

    2.0.2015122802
      Bug fixes involving buttons and OPTION_EL_MANUAL_ROTATE_LIMITS (Thanks, UA9OLB)

    2.0.2015122901  
      Corrections to bug fixes involving OPTION_CLOCK_ALWAYS_HAVE_HOUR_LEADING_ZERO (Thanks, UA9OLB)
  
    2.0.2016011801
      Fixed compilation bug involving last_moon_tracking_check_time and last_sun_tracking_check_time with some combinations of features

    2.0.2016012001
      Fixed bug with DEBUG_GPS_SERIAL and also improved GPS port reading  

    2.0.2016012101
      Fixed bug with OPTION_REVERSE_AZ_HH12_AS5045 and OPTION_REVERSE_EL_HH12_AS5045

    2.0.2016012102  
      Fixed issues with k3ngdisplay.h / k3ngdisplay.cpp

    2.0.2016012301
      Further work to get k3ngdisplay files to play with Arduino IDE 1.6.7

    2.0.2016021601
      DEBUG_HH12 more information output

    2.0.2016030101
       FEATURE_AZ_POSITION_HH12_AS5045_SSI: AZIMUTH_STARTING_POINT_DEFAULT used in heading calculation now    

    2.0.2016030201
      Fixed FEATURE_ADAFRUIT_BUTTONS to work with k3ngdisplay library and updated k3ngdisplay library to support Adafruit RGB display buttons   

    2.0.2016030401   
      Changed I2C_LCD_COLOR default to WHITE 

    2.0.2016030402
      OPTION_SEND_STRING_OUT_CONTROL_PORT_WHEN_INITIALIZING

    2.0.2016030501
      FEATURE_SAINSMART_I2C_LCD

    2.0.2016031001
      OPTION_DISPLAY_HEADING_AZ_ONLY with settings LCD_AZ_ONLY_HEADING_ROW, LCD_AZ_ONLY_HEADING_FIELD_SIZE
      OPTION_DISPLAY_HEADING_EL_ONLY with settings LCD_EL_ONLY_HEADING_ROW, LCD_EL_ONLY_HEADING_FIELD_SIZE

    2.0.2016032901
      Fixed issues with FEATURE_RFROBOT_I2C_DISPLAY compiling
      Corrected notes in features files about customizing features in rotator_k3ngdisplay.h 

    2.0.2016042801
      Fixed compilation error with FEATURE_AZIMUTH_CORRECTION and FEATURE_ELEVATION_CORRECTION 

    2.0.2016051501
      Fixed bug in submit_request() with slow down (Thanks Olli, DH2WQ)   

    2.0.2016071801
      Fixed bug with Maidenhead not being calculated when FEATURE_MOON_TRACKING or FEATURE_SUN_TRACKING wasn't compiled  

    2.0.2016083001
      Re-merged changes manually from dfannin submitted issue 30 - incorrect index for row_override; pull request 31
      (Couldn't get pull from git to compile correctly, not sure why)

    2.0.20160090701
      I screwed up.  I blew away F6FVY's pull request 29.  Restoring that.  There was a bug in the merged code that caused compile issue I was working on in 2.0.2016083001
        New Commands (which need to be documented):

            \Ix[x][x] - set az starting point
            \I - display the current az starting point
            \Jx[x][x] - set az rotation capability
            \J - display the current az rotation capability
            \Kx - force disable the az brake even if a pin is defined (x: 0 = enable, 1 = disable)
            \K - display the current az brake state
            \Q - Save settings in the EEPROM and restart

    2.0.2016090702
      Implemented simpler fix for issue 30 - incorrect index for row_override: byte row_override[LCD_ROWS+1]

    2.0.2016090801
      Corrected error in FEATURE_ROTARY_ENCODER_SUPPORT ttable (thanks, frye.dale)

    2.0.2016092501
      Working on FEATURE_AZ_POSITION_ROTARY_ENCODER_USE_PJRC_LIBRARY and FEATURE_EL_POSITION_ROTARY_ENCODER_USE_PJRC_LIBRARY
      Fixed bug with last row of LCD display getting blanked out
      FEATURE_TEST_DISPLAY_AT_STARTUP
      Noted in various settings files that AZIMUTH_STARTING_POINT_DEFAULT and AZIMUTH_ROTATION_CAPABILITY_DEFAULT are used only for initializing EEPROM
      Fixed an issue with FEATURE_AZ_POSITION_HH12_AS5045_SSI and FEATURE_AZ_POSITION_INCREMENTAL_ENCODER using AZIMUTH_STARTING_POINT_DEFAULT rather than azimuth_starting_point variable

    2.0.2016100301
      FEATURE_AZ_POSITION_ROTARY_ENCODER_USE_PJRC_LIBRARY and FEATURE_EL_POSITION_ROTARY_ENCODER_USE_PJRC_LIBRARY ready for testing

    2.0.2016102201
      Fixed bug with FEATURE_AZ_POSITION_HH12_AS5045_SSI, negative offset, and crossing between 359 and 0 degrees

    2.0.2017010101
      Minor update in comments in settings files  

    2.0.2017010102
      Fixed bug in FEATURE_ELEVATION_CONTROL with brake control (Thanks, zoobie40)

    2.0.2017041901
      Fixed bug - when azimithal rotation was in progress and an azimuth heading that was within the tolerance was submitted, rotation was not stopped (Thanks, Laurent, F6FVY)

    2.0.2017042401
      configuration.brake_az_disabled is now set to 0 (not disabled) when initializing eeprom (Thanks, Patrick, TK5EP)

    2017.05.13.01
      Added the \V command to FEATURE_CLOCK to set timezone offset  

    2017.05.13.02
      Fixed bug with timezone offset functionality  

    2017.07.24.01
      Fixed bug with "strcat(workstring." (Thanks, Russ, K0WFS)

    2017.07.24.02
      Fixed typos in a few places with "or" in if statements.  Not sure how that happened :-/  (Thanks, Russ, K0WFS)

    2017.07.31.01
      Fixed various LCD display clock options to display local time

    2017.08.01.01
      Fixed local time display bugs and local time calculation for negative offset timezones (UTC-x)

    2017.08.02.01
      FEATURE_AUTOPARK created and documented here https://github.com/k3ng/k3ng_rotator_controller/wiki/705-Park-and-AutoPark

    2017.08.14.01
      Added \+ command which switched LCD azimuth display mode between normal, raw, and +overlap modes  

    2017.09.03.01
      Added auxiliary pins for rotate LEDs: pin_led_cw, pin_led_ccw, pin_led_up, and pin_led_down, and related settings PIN_LED_ACTIVE_STATE, PIN_LED_INACTIVE_STATE  

    2017.09.03.02
      Added pins pin_autopark_disable and pin_autopark_timer_reset for FEATURE_AUTOPARK

    2017.09.05.01
      Added FEATURE_AUDIBLE_ALERT documented here: https://github.com/k3ng/k3ng_rotator_controller/wiki/455-Human-Interface:-Audible-Alert

    2017.11.14.01
      Merged pulled request #42 - allowing functions to return their calculated values https://github.com/k3ng/k3ng_rotator_controller/pull/42 (Thanks, SQ6EMM)  

    2018.01.25.01
      FEATURE_AZ_POSITION_HMC5883L_USING_JARZEBSKI_LIBRARY
      {need to document in wiki after someone tests}

    2018.01.25.02
      FEATURE_AZ_POSITION_DFROBOT_QMC5883
      {need to document in wiki after someone tests}

    2018.01.28.01
      Enhanced master/slave link TX sniff output  

    2018.02.01.01
      Added serial port support for ARDUINO_MAPLE_MINI,ARDUINO_AVR_PROMICRO,ARDUINO_AVR_LEONARDO,ARDUINO_AVR_MICRO,ARDUINO_AVR_YUN,ARDUINO_AVR_ESPLORA,ARDUINO_AVR_LILYPAD_USB,ARDUINO_AVR_ROBOT_CONTROL,ARDUINO_AVR_ROBOT_MOTOR,ARDUINO_AVR_LEONARDO_ETH,TEENSYDUINO  

    2018.02.02.01
      Minor updates to DEBUG_ACCEL

    2018.02.05.01
      Disabled free memory check in DEBUG_DUMP for TEENSYDUINO to fix compilation erroring out (Thanks, Martin, HS0ZED)

    2018.02.11.01
      Merge of https://github.com/k3ng/k3ng_rotator_controller/pull/45 (Thanks, IT9IPQ) 

    2018.02.24.01
      Added OPTION_GPS_DO_PORT_FLUSHES   

    2018.02.25.01
      Small change to FEATURE_GPS and gps_port_read

    2018.03.02.01
      Added code to handle GPS serial data that is missing terminator characters.  Created OPTION_GPS_EXCLUDE_MISSING_LF_CR_HANDLING which disables this function. 

    2018.03.03.01
      Changed some formatting of the debug log output
      Added the /?CV command to query software version

    2018.03.04.01
      GPS serial port reading is now paused if the GPS library has a valid sentence processed 

    2018.03.06.01
      Additional DEBUG_GPS code and OPTION_MORE_SERIAL_CHECKS for some GPS problem troubleshooting

    2018.03.08.01
      Added OPTION_MORE_SERIAL_CHECKS
      Added OPTION_RFROBOT_I2C_DISPLAY_BACKLIGHT_OFF to rotator_k3ngdisplay.h
     
    2018.03.11.01
      GPS performance tweak - now ignoring gps_data_available and reading all data available on GPS port

    2018.03.14.01
      SET_I2C_BUS_SPEED in settings file; set I2C bus speed to help address I2C I/O time impact serial port performance

    2018.04.21.01
      Added OPTION_STEPPER_MOTOR_USE_TIMER_ONE_INSTEAD_OF_FIVE for FEATURE_STEPPER_MOTOR.  Also added TimerOne library to Github.

    2018.05.16.01
      Added FEATURE_AZ_POSITION_MECHASOLUTION_QMC5883 - QMC5883 digital compass support using Mechasolution library at https://github.com/keepworking/Mecha_QMC5883L
      Modified MechaQMC5883.cpp to get rid of compiler warning about ::read

    2018.10.17.01
      Added FEATURE_MIDAS_I2C_DISPLAY 

    2018.10.17.02
      Added OVERLAP_LED_ACTIVE_STATE and OVERLAP_LED_INACTIVE_STATE settings

    2018.10.19.01
      Added OPTION_SAVE_MEMORY_EXCLUDE_BACKSLASH_CMDS

    2018.10.19.02
      Added Hygain DCU-1 protocol emulation - FEATURE_DCU_1_EMULATION

    2018.10.19.03
      Added FEATURE_FABO_LCD_PCF8574_DISPLAY
      Added PRESET_ENCODER_CHANGE_TIME_MS in settings files  

    2018.12.25.01     
      Fixed bug in RTC sync timing affecting SYNC_WITH_RTC_SECONDS (Thanks, Fred, VK2EFL for fix, and Steve, N4TTY for discovery)

    2019.01.03.01
      Updated GS-232 M and W commands to accept azimuths over 360 degrees and improved parameter verification 

    2020.02.05.01
      Moved debug defines to rotator_debug_log_activation.h
      FEATURE_AZ_ROTATION_STALL_DETECTION & FEATURE_EL_ROTATION_STALL_DETECTION
        OPTION_ROTATION_STALL_DETECTION_SERIAL_MESSAGE
        Settings
          STALL_CHECK_FREQUENCY_MS_AZ
          STALL_CHECK_DEGREES_THRESHOLD_AZ
          STALL_CHECK_FREQUENCY_MS_EL
          STALL_CHECK_DEGREES_THRESHOLD_EL
        Pins: az_rotation_stall_detected, el_rotation_stall_detected
        
    2020.02.05.02
      Minor add to DEBUG_RTC  

    2020.03.07.01
      Added LCD_PERIODIC_REDRAW_TIME_SECS, LCD_CLEAR_BEFORE_REDRAW, LCD_REDRAW_UPON_COMMANDS to settings files
      Fixed bug with sun and moon tracking deactivation (\U and \M CLI commands) not stopping rotation (Thanks Steve VE3RX)
      Minor addition in the k3ngdisplay code for display updates and redraws

    2020.03.08.01
      Change made to ensure slow start completes before slow stop activates  
      Add \H command line interface command - clear and redraw the LCD display

    2020.03.11.01
      Upon deactivation of moon or sun tracking using the button pins (moon_tracking_button, sun_tracking_button) or the activation lines (moon_tracking_activate_line, sun_tracking_activate_line), any in progress rotation will now stop (Thanks Steve VE3RX)

    2020.03.16.01
      Implemented a round robin screen redraw in rotator_k3ngdisplay.cpp 

    2020.03.30.01
      FEATURE_NEXTION_DISPLAY_OLD - Nextion display support UNDER DEVELOPMENT
      Added file rotator_nextion.h
      Documentation in progress: https://github.com/k3ng/k3ng_rotator_controller/wiki/425-Human-Interface:-Nextion-Display

    2020.03.30.02
      Fixed potential calculation issue with AZIMUTH_SMOOTHING_FACTOR and ELEVATION_SMOOTHING_FACTOR

    2020.04.02.01
      Fixed bug with AZIMUTH_SMOOTHING_FACTOR and ELEVATION_SMOOTHING_FACTOR (Thanks Steve VE3RX)
      More work on FEATURE_NEXTION_DISPLAY (UNDER DEVELOPMENT)

    2020.04.02.02
      Improved FEATURE_PARK not parked mode detection

    2020.04.03.01
      Fixed issue with 20 column LCD displays and spacing of az and el readings (Thanks Steve VE3RX) 

    2020.04.18.01  
      More work on FEATURE_NEXTION_DISPLAY, Nextion Display API (UNDER DEVELOPMENT)
      Documentation in progress: https://github.com/k3ng/k3ng_rotator_controller/wiki/425-Human-Interface:-Nextion-Display

    2020.04.19.01  
      More work on FEATURE_NEXTION_DISPLAY, Nextion Display API (UNDER DEVELOPMENT)
      Documentation in progress: https://github.com/k3ng/k3ng_rotator_controller/wiki/425-Human-Interface:-Nextion-Display

    2020.04.27.01
      New pin: pin_status_led - Status LED - blinks when there is rotation in progress

    2020.04.28.01
      More work on FEATURE_NEXTION_DISPLAY
      Nextion HMI file contributed by Jan ZS1VDV 

    2020.05.06.01  
      More work on FEATURE_NEXTION_DISPLAY
      Updated Nextion documentation: https://github.com/k3ng/k3ng_rotator_controller/wiki/425-Human-Interface:-Nextion-Display

    2020.05.07.01
      More work on FEATURE_NEXTION_DISPLAY
      Updated Nextion documentation: https://github.com/k3ng/k3ng_rotator_controller/wiki/425-Human-Interface:-Nextion-Display
      New Extended Backslash Commands
        \?AO - Azimuth Full CCW Calibration
        \?AF - Azimuth Full CW Calibration
        \?EO - Elevation Full DOWN Calibration
        \?EF - Elevation Full UP Calibration
      Fixed several issues with debug dump log  
      Updated command reference: https://github.com/k3ng/k3ng_rotator_controller/wiki/820-Command-Reference

    2020.05.08.01
      Made improvement to rotator_k3ngdisplay.cpp to reduce LCD bus traffic from cursor positioning

    2020.05.11.01
      Handle ARDUINO_AVR_MICRO having Serial port as Serial_ class and Serial1 as HardwareSerial class

    2020.05.11.02
      Added OPTION_ALLOW_ROTATIONAL_AND_CONFIGURATION_CMDS_AT_BOOT_UP which is disabled by default.  By default the controller will ignore at boot up GS-232 commands that start rotation or change the configuration.
        Related setting (new): ROTATIONAL_AND_CONFIGURATION_CMD_IGNORE_TIME_MS 5000   (ignore commands the first 5 seconds after boot up)

    2020.05.26.01
      Hopefully fixed issues with Arduino Leonardo and Micro and serial port classes
      Added note on Arduino Leonardo and Micro serial configuration in rotator_hardware.h

    2020.06.12.01
      \A Azimuth calibration command now also modifies azimuth_starting_point so that proper rotation decisions are made for large calibration values  

    2020.06.12.02
      Typo fix on \X0 command

    2020.06.12.03
      azimuth, raw_azimuth, and elevation variables changed from int to float datatypes
      Eliminated HEADING_MULTIPLIER, LCD_HEADING_MULTIPLIER settings related to azimuth, raw_azimuth, and elevation variables being int datatypes
      Eliminated FEATURE_ONE_DECIMAL_PLACE_HEADINGS and FEATURE_TWO_DECIMAL_PLACE_HEADINGS
      LCD_DECIMAL_PLACES now is only setting need to change number of decimal places displayed

    2020.06.13.01
      More work on \A azimuth calibration command and proper modification of azimuth_starting_point and azimuth_offset.  I think I got it right now.

    2020.06.13.02
      FEATURE_EASYCOM_EMULATION: Deprecated OPTION_EASYCOM_AZ_QUERY_COMMAND and OPTION_EASYCOM_EL_QUERY_COMMAND.  AZ and EL commands without parameters are standard in Easycom 2 protocol, so no need to have as an OPTION
      Changed some elevation variables I missed in 2020.06.12.03 from int to float
      Updated submit_request() for float heading datatypes
      Updated azimuth and elevation readings for float datatypes
      LCD_DECIMAL_PLACES setting has been renamed DISPLAY_DECIMAL_PLACES as it also applies to the Nextion display unit and API
      FEATURE_DCU_1_EMULATION: Rewrote to comply with published command specification.  AI1 and ; commands now implemented

    2020.06.14.01
      azimuth_rotation_capability variable changed to long datatype to support values up to 2,147,483,647

    2020.06.20.01
      Refactoring of read_azimuth()
      In settings files deprecated ROTATE_PIN_INACTIVE_VALUE and ROTATE_PIN_ACTIVE_VALUE; replaced with ROTATE_PIN_AZ_INACTIVE_VALUE, ROTATE_PIN_AZ_ACTIVE_VALUE, ROTATE_PIN_EL_INACTIVE_VALUE, ROTATE_PIN_EL_ACTIVE_VALUE

    2020.06.26.01
      Removed errant HEADING_MULTIPLIER from PARK settings in settings files (Thanks, Adam VK4GHZ )

    2020.06.26.02
      In rotator_hardware.h, got rid of serial port class definitions for the GPS, GPS mirror, and remote serial ports, and have one define CONTROL_PORT_SERIAL_PORT_CLASS for the control port (usually the Serial port)
      Arduino Leonardo, Micro, and Yún users will need to change CONTROL_PORT_SERIAL_PORT_CLASS in rotator_hardware.h if not using the Serial port as the control port
      Changed and reorganize serial port mappins in rotator_settings*.h files

    2020.07.01.01
      Fixed \I and \J commands when used with no argument so they return current values of azimuth starting point and azimuth rotation capability
      Added CONFIG_DIRTY and CONFIG_NOT_DIRTY flags to periodic debug logging
      Made resetting of Nextion display upon Arduino boot up more reliable
      Bug OPTION_CLOCK_ALWAYS_HAVE_HOUR_LEADING_ZERO and Nextion display (Thanks, Adam VK4GHZ ) 
      Fixed bug with FEATURE_SUN_TRACKING and FEATURE_MOON_TRACKING and Nextion display API variable gMSS (Thanks, Adam VK4GHZ ) 

    2020.07.01.02
      Added FEATURE_ADC_RESOLUTION12 contributed by Adam VK4GHZ 

    2020.07.17.01
      Merged Pull Request 71 - Update rotator_dependencies.h, fixing issue: device was unresponsive when using LSM303D sensor with FEATURE_AZ_POSITION_POLOLU_LSM303 ( https://github.com/k3ng/k3ng_rotator_controller/pull/71/ ) (Thanks, 7x2uv)  

    2020.07.17.02
      DISPLAY_DEGREES_STRING is now broken out into LCD_DISPLAY_DEGREES_STRING and NEXTION_DISPLAY_DEGREES_STRING in settings files
      Yaesu Help (H command) updated to include missing commands (Thanks, Adam VK4GHZ) 

    2020.07.18.01
      Added extended backslash commands
        \?CGxxxx[xx]        - convert grid to coordinates
        \?RG                - read grid square  
        \?CCxxxxx yyyyy     - convert coordinates to grid xxxxx = latitude, yyyyy = longitude
        \?BCxxxxx yyyyy     - calculate bearing to coordinate target xxxxx = latitude, yyyyy = longitude
        \?BGxxxx[xx]        - calculate bearing to grid target
        \?GCxxxxx yyyyy     - go to coordinate target xxxxx = latitude, yyyyy = longitude (rotate azimuth)
        \?GTxxxx[xx]        - go to grid target (rotate azimuth) 
      Added function maidenhead_to_coordinates(grid,latitude_degrees, longitude_degrees) from Adam VK4GHZ code contribution
      Added function calculate_target_bearing(source_latitude, source_longitude, target_latitude, target_longitude) from Adam VK4GHZ code contribution
   
    2020.07.18.02
      FEATURE_NEXTION_DISPLAY
        Fixed updating of vMAS, vMES, vSAS, and vSES API variables
        Fixed degree symbol in vSS1 API variable
        Add variable gDP (Integer) - Number of decimal places used in various heading variables (set by DISPLAY_DECIMAL_PLACES)

    2020.07.19.01
      Added DEBUG_PROCESSES
      Removed several instances where update_time() was being called unnecessarily.  update_time() was consuming about 21% CPU time, now down to about 12%    
      
    2020.07.19.02  
      FEATURE_NEXTION_DISPLAY
        Changed updating of gMSS API variable to reflect moon and sun visibility without tracking activated
   
    2020.07.21.01
      FEATURE_NEXTION_DISPLAY
        Added NOT_PROVISIONED state to gCS Clock Status API variable
        Added gX and gY API variables for heading Cartesian coordinates, for future use to drive combined azimuth and elevation gauges 

    2020.07.22.01
      Developing FEATURE_SATELLITE_TRACKING.  Yea.  
      FEATURE_NEXTION_DISPLAY: call service_nextion_display() right after rebooting display at start up

    2020.07.23.01
      Lots more work on FEATURE_SATELLITE_TRACKING.  It's only outputting AO-7 in debug logs right now.
      Pushing to Github in case I get hit by a bus before I finish this.

    2020.07.24.01
      After pulling my hair out for two days, I rewrote the satellite tracking to use the P13 library from Mark VandeWettering
      \^ command to activate and deactive satellite tracking
      \~ command to view satellite tracking status  

    2020.07.25.01
      More work on FEATURE_SATELLITE_TRACKING
        Added LCD support
        Command \!           - erase satellite TLE file area in eeprom
        Command \@           - print satellite TLE file area from eeprom
        Command \octothorpe  - load satellite Keplarian elements file into eeprom (copy and paste bare TLE)
        Command \$xxxxxxx    - change current satellite (xxxxxx = satellite name in stored TLE file)
        Added OPTION_DISPLAY_SATELLITE_TRACKING_CONTINUOUSLY to LCD functionality
        OPTION_DISPLAY_MOON_OR_SUN_TRACKING_CONDITIONAL changed to OPTION_DISPLAY_MOON_OR_SUN_OR_SAT_TRACKING_CONDITIONAL
        Setting LCD_MOON_OR_SUN_TRACKING_CONDITIONAL_ROW changed to LCD_MOON_OR_SUN_OR_SAT_TRACKING_CONDITIONAL_ROW
        Under construction documentation https://github.com/k3ng/k3ng_rotator_controller/wiki/707-Satellite-Tracking 

      2020.07.25.02
        FEATURE_NEXTION_DISPLAY: Added vConResult string API variable which returns the results of backslash commands from the Nextion display

      2020.07.25.03
        FEATURE_NEXTION_DISPLAY
          New API variables:
            gGF          Integer               GPS fix age in mS
            vSAT         String[16]            Current Satellite Name
            vTAS         String[6]             Satellite Azimuth String
            vTES         String[6]             Satellite Elevation String 
            vTLA         String[7]             Satellite Latitude String
            vTLO         String[7]             Satellite Longitude String
          gMSS has been expanded to include satellite functionality:
            gMSS         Integer, Bit Mapped   Moon, Sun, and Satellite Status
                                                    Bit Values                                  
                                                       satellite_tracking_active 16
                                                       satellite_visible 32
          gSC now has a bit mapping for FEATURE_SATELLITE_TRACKING:
            gSC         Integer, Bit Mapped   System Capabilities
                                                    Bit Values
                                                       SATELLITE 1024
      2020.07.26.01
        FEATURE_SATELLITE_TRACKING
          \% command - print upcoming passes for current satellite
          \~ command now shows upcoming pass AOS and LOS  
          \^1 activate satellite tracking command now pre-rotates to satellite next AOS az and el
        Added OPTION_DISPLAY_SATELLITE_TRACKING_ALTERNATING - LCD display one line that alternates current satellite az/el, AOS date/time, and AOS az/le  
        Settings TRACKING_ACTIVE_CHAR and TRACKING_INACTIVE_CHAR deprecated
        New settings
          OPTION_DISPLAY_SATELLITE_TRACKING_ALTERNATING_TIME_MS 5000
          LCD_DISPLAY_SATELLITE_VISIBLE_NOT_TRACKED_CHAR "~"
          LCD_DISPLAY_SATELLITE_VISIBLE_TRACKED_CHAR "*"
          LCD_DISPLAY_SATELLITE_NOT_VISIBLE_TRACKED_CHAR "."
          LCD_DISPLAY_MOON_TRACKING_ACTIVE_CHAR "*"
          LCD_DISPLAY_MOON_TRACKING_INACTIVE_CHAR "-"
          LCD_DISPLAY_SUN_TRACKING_ACTIVE_CHAR "*"
          LCD_DISPLAY_SUN_TRACKING_INACTIVE_CHAR "-"
        Added nasabare.txt bare TLE file to Github in tle/ directory

      2020.07.26.02
        FEATURE_SATELLITE_TRACKING
          \% command now calculates max elevation for each pass
        FEATURE_NEXTION_DISPLAY
          vConResult API variable no longer has response code in results from extended backslash commands

      2020.07.27.01
        Fixed issue with race condition when changing moon, sun, and satellite tracking modes

      2020.07.28.01    
        FEATURE_SATELLITE_TRACKING
          OPTION_DISPLAY_SATELLITE_TRACKING_ALTERNATING now displays "AOS in <time>" and "LOS in <time>".
          New API variables:
            vADF         String[11]            Satellite Next AOS Full Date String (YYYY-MM-DD)
            vADS         String[5]             Satellite Next AOS Short Date String (MM-DD)
            vATS         String[5]             Satellite Next AOS Time String (HH:MM)
            vLDF         String[11]            Satellite Next LOS Full Date String (YYYY-MM-DD)
            vLDS         String[5]             Satellite Next LOS Short Date String (MM-DD)
            vLTS         String[5]             Satellite Next LOS Time String (HH:MM)
            vALI         String[13]            Satellite AOS/LOS In String (Examples: "AOS in 1d13h","LOS in 30m12s","LOS in 5s")
          There is a bug with the "AOS/LOS in time" function (satellite_aos_los_string()) that only seems to affect HUBBLE, showing negative seconds.  Investigating.

      2020.07.30.01
        Changed \?GC and \?GT and other coordinate and grid related commands to not require GPS, clock, or moon/sun tracking features.

      2020.07.31.01
        Fixed bug introduced in 2020.07.27.01 with moon, sun, and satellite tracking deactivation not stopping rotation

      2020.08.03.01
        FEATURE_SATELLITE_TRACKING
          Fixed bug with controller locking up upon boot up if previously selected satellite wasn't available in TLEs stored in eeprom
          Enhanced TLE storage so unnecessary data in TLE line 1 isn't stored in eeprom, resulting in space savings and allowing more TLEs to be stored  
          Working on \| command to list satellites available for tracking

      2020.08.08.01   
        FEATURE_SATELLITE_TRACKING
          Finished \| command to list all available satellites.  This will be updated later to show next AOS of each satellite.
          Added vS1..vS34 API variables that contain all satellites available for tracking
          \$ command will now first search for a literal match of the entered satellite name, and if no match is found, it will match on the first four characters 

      2020.08.09.01
        FEATURE_SATELLITE_TRACKING
          Fixed bug with vS1..vS34 API variables (Thanks, Adam, VK4GHZ)

      2020.08.10.01
        FEATURE_SATELLITE_TRACKING
          Changed TLE loading routine to store carriage returns as 0xFE in eeprom and eliminated differences with systems that do carriage return
          versus carriage return + line feed, however, regardless TLEs cannot be loaded with the Arduino IDE Serial Monitor
          as it strips multi-line pasted text in the send window.  Argh.  You need to use PuTTY on Windows.  Screen on *nix works fine.

      2020.08.10.02              
        FEATURE_SATELLITE_TRACKING
          Fixed bug with \# load TLE command not populating satellite list array correctly (Thanks, Adam, VK4GHZ)
        FEATURE_NEXTION_DISPLAY
          Changed initialization so we don't wait for 200 mS doing nothing while the display initializes after we reset it
        Got rid of 1 second delay at start up.  There's really no need for it.   

      2020.08.13.01
        FEATURE_SATELLITE_TRACKING
          gSC API Variable
            This is now the first variable written to the display after initialization
            Added additional capability bit values
              ENGLISH 2048
              SPANISH 4096
              CZECH 8192
              PORTUGUESE_BRASIL 16384
              GERMAN 32768
              FRENCH 65536
          Language Support (rotator_language.h)
            NEXTION_GPS_STRING
            NEXTION_PARKING_STRING
            NEXTION_PARKED_STRING
            NEXTION_OVERLAP_STRING
          vSS3 API variable deprecated
          Parked and Overlap messages move to vSS2 API variable
          Change wait time after reboot of Nextion display from 200 mS to 300 mS

      2020.08.15.01
        FEATURE_SATELLITE_TRACKING
          The \% command (print current satellite AOS and LOS times) now runs asynchronously in the background

      2020.08.15.02
        FEATURE_CLOCK
          Refactoring of the variables for various clocks into structs to make things look neater 

      2020.08.16.01
        FEATURE_NEXTION_DISPLAY
          Improved initialization; we now look for 0xFF 0xFF 0xFF from the Nextion at boot up and then get to work.
        FEATURE_SATELLITE_TRACKING
          The \~ command (print current satellite status) can now have a periodic print out (example: \~5 = print status every 5 seconds) 
        Refactored process_backslash_command() code to recover a boatload of SRAM (local variable space).  Yeahhh.   

      2020.08.17.01  
        FEATURE_NEXTION_DISPLAY
          Removed languages from gSC API variable and added two new bit values:
            PARK 2048
            AUTOPARK 4096
          Create new API variable gL for language
            ENGLISH 1
            SPANISH 2
            CZECH 4
            PORTUGUESE_BRASIL 8
            GERMAN 16
            FRENCH 32
        FEATURE_SATELLITE_TRACKING
          Created FEATURE_SATELLITE_TRACKING_MULTI_SAT_AOS_LOS
            Created \& command which prints out the next AOS and LOS for each satellite    

      2020.08.18.01
        Elimated the need for avr/wdt.h include file
        FEATURE_SATELLITE_TRACKING
          Refactoring service_calc_satellite_data() to do all satellite az, el, lat, long, next AOS, and next LOS calculations.
          Enhanced the satellite[] struct to contain more information.  Going to have service_calc_satellite_data() populate this.

      2020.08.19.01 
        FEATURE_NEXTION_DISPLAY
          Fixed bugs with gSC and gL API variables.  I picked the wrong week to quit sniffing glue.   

      2020.08.20.01
        FEATURE_SATELLITE_TRACKING
          Removed FEATURE_SATELLITE_TRACKING_MULTI_SAT_AOS_LOS.  It's all within FEATURE_SATELLITE_TRACKING now.  We're not going to get satellite tracking to fit on a Nano, so no need to break out sub-features for memory saving.
          Significant updates to the \| (list satellites command).  It now displays current az, el, lat, long, next aos, next los, next aos max el, time to next aos, and aos status.  Updating is done in the background.  This is some cool $41t.
        FEATURE_NEXTION_DISPLAY
          Fixed bug with vSS2 API variable and Parked and Parking messages
        Added OPTION_CLI_VT100 for sending VT100 escape codes.  More on this later.

      2020.08.23.01
        The \X0 command (clear azimuth and elevation calibration / offsets) is now available without FEATURE_MOON_TRACKING and FEATURE_SUN_TRACKING enabled 
        FEATURE_SATELLITE_TRACKING
          The \| list satellites command is now sorted by next AOS time.
      
      2020.08.24.01
        FEATURE_NEXTION_DISPLAY & FEATURE_SATELLITE_TRACKING
          Added vSatNx, vSatOx, and vSatAx API variables which send a list of satellites and their next AOS time  
          Added NEXTION_NUMBER_OF_NEXT_SATELLITES setting.  This determines how many satellites are sent via the vSatNx, vSatOx, and vSatAx API variables
          https://github.com/k3ng/k3ng_rotator_controller/wiki/425-Human-Interface:-Nextion-Display for more information

      2020.08.24.02
        DEBUG_LOOP - additional code for various new subroutines  

      2020.08.25.01
        DEBUG_SATELLITE_ARRAY_ORDER - additional debugging code

      2020.08.25.02  
        FEATURE_SATELLITE_TRACKING
          Significant changes to the satellite data array updating algorithm
          More debugging code everywhere
          Improved AOS detection consistency
          Made it Harder Better Faster Stronger  

      2020.08.26.02    
        FEATURE_SATELLITE_TRACKING
          Significant updates to next AOS and LOS time calculations.  This is some crazy code that throttles the calculation resolution back depending on the amount of time the calculation has been running. 
          Better handling of TLE loading at boot up and handling a corrupt TLE file

      2020.08.26.03
        FEATURE_SATELLITE_TRACKING
          Even more optimizations!  It's working great!
        FEATURE_NEXTION_DISPLAY
          Fixed bug with gMSS API variable introduced with satellite related API variable code  

      2020.08.28.01
        FEATURE_NEXTION_DISPLAY & FEATURE_SATELLITE_TRACKING
          Fixed bug with vSS1 degrees symbol

      2020.08.29.01
        FEATURE_SATELLITE_TRACKING
          Yet another significant update to service_calc_satellite_data() - next AOS and LOS calculations are now accurate within 5 seconds!
      
      2020.08.29.02
        FEATURE_CLOCK 
          Fixed bug with CLOCK_DEFAULT_*_AT_BOOTUP settings not being used at boot up
          \O (Oscar) command (set clock manually) will now optionally take seconds in the argument (i.e. \O202008292032 or \O20200829203255)
        FEATURE_SATELLITE_TRACKING & FEATURE_CLOCK
          The \O (Oscar) command (set clock manually) now resets the satellite data array so it's recalculated using the new time setting
        FEATURE_NEXTION_DISPLAY & FEATURE_AUTOPARK
          Added an addition bit value to API variable gVS (various statuses), autopark_active 32768
        FEATURE_NEXTION_DISPLAY
          Enhanced initialization routine to retry if "i'm alive" bytes are not received
          Additional logging in DEBUG_NEXTION_DISPLAY_INIT
           
      2020.08.30.01
        FEATURE_PARK
          The park azimuth and elevation is now stored in the EEPROM configuration
          Settings PARK_AZIMUTH and PARK_ELEVATION have been deprecated
          The \P command has been enhanced:
            \PA[x][x][x] - set the park azimuth
            \PE[x][x][x] - set the park elevation
            \PA or \PE (no parameter) - report current park azimuth and elevation 
            \P still initiates park
        FEATURE_NEXTION_DISPLAY & FEATURE_SATELLITE_TRACKING
          Changing the current satellite on the Nextion display no longer echoes the command output on the control port  

      2020.09.01.01    
        FEATURE_NEXTION_DISPLAY & FEATURE_PARK 
          Added vPA and vPE API variables for park azimuth and elevation settings
          Added vAT API variable for autopark time setting in minutes
        FEATURE_NEXTION_DISPLAY
          Added transient user messaging capability for vSS1 and vSS2 via request_transient_message() call  
          \PA or \PE command from Nextion pushes output to vSS1 in transient message
        Now updating software version date based on UTC date, not Eastern US date.  G'day, mates! :-)

      2020.09.01.02
        Overlap functionality can now be disabled by commenting out define ANALOG_AZ_OVERLAP_DEGREES in settings file  
        Setting AZIMUTH_STARTING_POINT_DEFAULT renamed to AZIMUTH_STARTING_POINT_EEPROM_INITIALIZE
        Setting AZIMUTH_ROTATION_CAPABILITY_DEFAULT renamed to AZIMUTH_ROTATION_CAPABILITY_EEPROM_INITIALIZE

      2020.09.01.03  
        FEATURE_NEXTION_DISPLAY & FEATURE_SATELLITE_TRACKING
          Fixed bug with satellite visible bit in gMSS API variable getting set erroneously after boot up
        FEATURE_NEXTION_DISPLAY
          Deprecated the API variable vRCAPIv.  It wasn't providing much value.

      2020.09.02.01
        Settings Files
          setting ANALOG_AZ_FULL_CCW renamed to ANALOG_AZ_FULL_CCW_EEPROM_INITIALIZE
          setting ANALOG_AZ_FULL_CW renamed to ANALOG_AZ_FULL_CW_EEPROM_INITIALIZE
          setting ANALOG_EL_0_DEGREES renamed to ANALOG_EL_FULL_DOWN_EEPROM_INITIALIZE
          setting ANALOG_EL_MAX_ELEVATION renamed to ANALOG_EL_FULL_UP_EEPROM_INITIALIZE
          Added additional comments for clarification
        FEATURE_NEXTION_DISPLAY
          Fixed conflicts between moon, sun, and satellite API variables and issues with gMSS variable 

      2020.09.04.01
        FEATURE_NEXTION_DISPLAY
          Added new bit value to gVS (various statuses): configuration_dirty 65536 
        FEATURE_AUTOPARK
          Cleaned up some code for the \Y command that may have had a bug  

      2020.09.09.01
        FEATURE_NEXTION_DISPLAY
          Some code cleanup  

      2020.09.17.01
        FEATURE_AUTOPARK & FEATURE_PARK - raise an error at compile time if FEATURE_AUTOPARK is enabled without FEATURE_PARK rather than silently including FEATURE_PARK
        DEBUG_SATELLITE_TRACKING_CALC - fixed compiler error

      2020.10.20.01
        Merge of pull request 79 - Added Norwegian Bokmål language https://github.com/k3ng/k3ng_rotator_controller/pull/79  (Thanks, Supermagnum)
        LANGUAGE_NORWEGIAN_BOKMAAL added with pull request 79 merge

      2020.12.23.01
        Removed EA4TX hardware support due to copying of other open source project hardware and selling it 

      2021.01.26.01
        (Hopefully) Fixed heading decimal place issues (float vs. int) in FEATURE_AZ_POSITION_HH12_AS5045_SSI, FEATURE_EL_POSITION_HH12_AS5045_SSI, and FEATURE_EASYCOM_EMULATION

      2021.02.16.01
        Fixed potential for latest coordinates from GPS not being used for satellite tracking 

      2021.03.07.01
        Fixed potential issue with AZ_POSITION_ROTARY_ENCODER_DEG_PER_PULSE and EL_POSITION_ROTARY_ENCODER_DEG_PER_PULSE decimal values being truncated
        Updated default AO7TEST TLE

      2021.03.07.02
        Fixed issue in FEATURE_EL_POSITION_PULSE_INPUT with EL_POSITION_ROTARY_ENCODER_DEG_PER_PULSE decimal values being truncated

      2021.03.07.03
        Additional work on FEATURE_AZ_POSITION_PULSE_INPUT to properly handle float values and preserve decimal places

      2021.03.08.01
        Even more work on FEATURE_AZ_POSITION_PULSE_INPUT to properly handle float values and preserve decimal places

      2021.03.20.01
        The satellite, sun, and moon automatic tracking algorithms have been enhanced and have new runtime settings

          tracking check interval = the interval in mS the system performs a tracking calculation and decision
          rotation interval = the minimum amount of time in mS between rotation initiations
          degrees difference threshold = the decimal degrees difference between the current az/el and the desired az/el which must be met or exceed in order to initiate rotation

          Both the rotation interval and degrees difference threshold must be met in order to initiate rotation.
          The rotation interval and/or degrees difference threshold can be set to 0 to disable.

        New commands:

          \(                  - show satellite, sun, and moon automatic tracking parameters

          \?TSxxxx            - set satellite tracking check interval (mS)
          \?TUxxxx            - set sun tracking check interval (mS)
          \?TMxxxx            - set moon tracking check interval (mS)

          \?TXxxxx            - set satellite rotation interval (mS)
          \?TYxxxx            - set sun rotation interval (mS)
          \?TZxxxx            - set moon rotation interval (mS)

          \?TAx[.]x            - set satellite degrees difference threshold
          \?TBx[.]x            - set sun degrees difference threshold
          \?TCx[.]x            - set moon degrees difference threshold

      2021.04.01.01
        FEATURE_NEXTION_DISPLAY
          New Variables:
            gV2         Integer, Bit Mapped   Various States 2
              Bit Values
                audible_alert_enabled 1

            gTS          Integer               Satellite tracking check interval (mS)
            gTU          Integer               Sun tracking check interval (mS)
            gTM          Integer               Moon tracking check interval (mS)
            gTX          Integer               Satellite tracking rotation interval (mS)
            gTY          Integer               Sun tracking rotation interval (mS)
            gTZ          Integer               Moon tracking rotation interval (mS)
            vTA          String[5]             Satellite tracking degrees difference threshold
            vTB          String[5]             Sun tracking degrees difference threshold
            vTC          String[5]             Moon tracking degrees difference threshold

          New Bit Mapped in Existing Variable:         
            gSC         Integer, Bit Mapped   System Capabilities
              Bit Values
                AUDIBLE_ALERT 8192

      2021.04.02.01
        FEATURE_AUDIBLE_ALERT new commands:
          \-   Query audible alert state
          \-0  Disable audible alerts
          \-1  Enable audible alerts
          \-X  Silence current audible alert
          \-~  Manually trigger audible alert    
          \-A0 Disable AZ target alert
          \-A1 Enable AZ target alert
          \-E0 Disable EL target alert
          \-E1 Enable EL target alert  

      2021.04.07.01
        Fixed issue with el_incremental_encoder_position rolling over to 0 after 32,767 or so
        Happy Birthday, Mom

      2021.06.24.01
        Fixed issues with integer conversions involving azimuth and elevation (Thanks, Alexander, RV6FX)

      2021.07.17.01
        FEATURE_SATELLITE_TRACKING - Fixed issue with \! command not clearing out satellit array and leaving invalid choices displayed on Nextion display
        FEATURE_NEXTION_DISPLAY - Added \?NG command which prompts the rotator controller to send the gSC variable immediately to the Nextion
        
      2021.07.18.01
        FEATURE_SATELLITE_TRACKING - Fixed another issue with the \! not loading up the AO7-TEST default satellite (Thanks, Karl Jan Skontorp)
        FEATURE_NEXTION_DISPLAY - Major improvement in Nextion display startup timing (Thanks, Adam, VK4GHZ)

      2021.07.18.02
        FEATURE_NEXTION_DISPLAY - Added code to watch for unexpected reset / 'i am alive' from Nextion

      2021.10.05.01
        DEBUG_ROTATOR - added code to traceback who called rotator() 

      2021.10.05.02
        Cleaned up code and pins files to remove az_stepper_motor_direction and el_stepper_motor_direction and throw a compiler error if either are defined.  (Use the rotate_* pins instead.)
        Reference: https://github.com/k3ng/k3ng_rotator_controller/commit/b36c5a1d4c2cf21fe342d5c07ded6093adf71503 

      2021.10.06.01  
        FEATURE_STEPPER_MOTOR and Variable frequency outputs (rotate_cw_freq, rotate_ccw_freq, rotate_up_freq, rotate_down_freq):
          Clarified in settings files minimum and maximum values for AZ_VARIABLE_FREQ_OUTPUT_LOW, AZ_VARIABLE_FREQ_OUTPUT_HIGH, EL_VARIABLE_FREQ_OUTPUT_LOW, EL_VARIABLE_FREQ_OUTPUT_HIGH
        FEATURE_STEPPER_MOTOR - added code to limit high frequency value to 2000 hertz
        Big thanks to Fred, VK2WS!
        DEVELOPMENT_TIMELIB - working on recoding clock and time functions to the TimeLib library, in preparation for supporting Teensy RTC

      2021.10.08.01
        DEVELOPMENT_TIMELIB - Have TimeLib integrated with all time-related functionality, including FEATURE_CLOCK, FEATURE_GPS, FEATURE_MOON_TRACKING, FEATURE_SUN_TRACKING, FEATURE_RTC_DS1307, FEATURE_RTC_PCF8583
        Place a define for DEVELOPMENT_TIMELIB in your rotator_features file to test

      2021.10.12.01
        FEATURE_STEPPER_MOTOR: Added several options to change maximum frequency supported.  (Not tested on hardware yet.)
          OPTION_STEPPER_MOTOR_MAX_2_KHZ (enabled by default)
          OPTION_STEPPER_MOTOR_MAX_5_KHZ
          OPTION_STEPPER_MOTOR_MAX_10_KHZ
          OPTION_STEPPER_MOTOR_MAX_20_KHZ
  
      2021.10.13.01
        FEATURE_STEPPER_MOTOR: Added OPTION_STEPPER_MOTOR_MAX_50_KHZ

      2021.10.13.02
        DEBUG_STEPPER: More code  

      2021.10.14.01
        FEATURE_STEPPER_MOTOR
          Removed OPTION_STEPPER_MOTOR_MAX_50_KHZ.  Too much overhead from interrupts.
          Implemented faster digital writes using digitalWriteFast library (library now included in Github)
          Added OPTION_STEPPER_DO_NOT_USE_DIGITALWRITEFAST_LIBRARY to disable digitalWriteFast library use

      2021.10.14.02
        Added OPTION_CLI_VT100 - VT100 terminal emulation; currently used only by satellite tracking status command \|
        Updated hardcoded default AO7TEST TLE

      2021.10.15.01
        FEATURE_SATELLITE_TRACKING:
          Added \& command to manually invoke recalculation of all satellite tracking data
          All satellite data is now recalculated if the clock comes out of FREE_RUNNING state or if the latitude / longitude (current location) changes
        All time functionality permanent changed to use TimeLib library
        Working on adding Sparkfun u-blox GNSS library (OPTION_GPS_USE_SPARKFUN_U_BLOX_GNSS_LIBRARY)  

      2021.10.17.01
        FEATURE_SATELLITE_TRACKING: I *believe* I fixed the issue with satellite tracking using DEFAULT_LATITUDE and DEFAULT_LONGITUDE rather than GPS-derived coordinates

      2021.10.19.01
        Increment Encoder Sensors: Attempt to get large EL_POSITION_INCREMENTAL_ENCODER_PULSES_PER_REV working correctly

      2021.10.19.02
        FEATURE_NEXTION_DISPLAY: testing new initialization code (OPTION_NEW_NEXTION_INIT_CODE)

      2021.10.19.03  
        FEATURE_NEXTION_DISPLAY: optimization of OPTION_NEW_NEXTION_INIT_CODE

      2021.10.19.04 
        FEATURE_NEXTION_DISPLAY: more work on initialization code 

      2021.11.22.01
        FEATURE_NEXTION_DISPLAY: Added OPTION_SEND_NEXTION_RESET_AT_BOOTUP which may fix Nextion boot up issues for some
        Also working on FEATURE_CALIBRATION 

      2021.12.16.01
        Fixed compilation issues with update_time() and set_clock. errors with FEATURE_REMOTE_UNIT_SLAVE, FEATURE_MASTER_WITH_SERIAL_SLAVE (thanks, Razvan Popescu)  

      2021.12.27.01
        Coded FEATURE_MASTER_SEND_AZ_ROTATION_COMMANDS_TO_REMOTE and FEATURE_MASTER_SEND_EL_ROTATION_COMMANDS_TO_REMOTE but haven't tested yet
        \M command: query moon azimuth and elevation
        \U command: query sun azimuth and elevation
        Still working on FEATURE_CALIBRATION
        
      2021.12.27.02
        Tested FEATURE_MASTER_SEND_AZ_ROTATION_COMMANDS_TO_REMOTE and FEATURE_MASTER_SEND_EL_ROTATION_COMMANDS_TO_REMOTE and fixed issue with link pings (PG) being sent way too often

      2022.02.20.01
        FEATURE_SATELLITE_TRACKING: Updated P13 library to have observer.update_location function.  Controller code now uses update_location rather than instantiating new Observer objects
        Updated hardcoded AO17TEST TLE 

      2023.09.29.0800
        Added FEATURE_AZ_POSITION_HH12_AS5045_SSI_RELATIVE; not tested  

      2023.09.29.2039
        More code in DEBUG_HH12

      2023.10.02.2302
        More code in DEBUG_HH12  

      2023.10.03.2326
        Fixed bugs in FEATURE_AZ_POSITION_HH12_AS5045_SSI_RELATIVE
      
      2023.10.06.2200
        FEATURE_AZ_POSITION_HH12_AS5045_SSI_RELATIVE: fixed bugs

    All library files should be placed in directories likes \sketchbook\libraries\library1\ , \sketchbook\libraries\library2\ , etc.
    Anything rotator_*.* should be in the ino directory!

    qwerty was here
    
  Documentation: https://github.com/k3ng/k3ng_rotator_controller/wiki

  Support: https://groups.io/g/radioartisan

  YouTube Channel: https://www.youtube.com/channel/UC5o8UM1-heT5kJbwnJRkUYg

  */

#define CODE_VERSION "2023.10.06.2200"


#include <EEPROM.h>
#include <math.h>

#include "rotator_hardware.h"

#include "rotator_features.h"

#include "rotator_dependencies.h"

#ifdef FEATURE_4_BIT_LCD_DISPLAY
  #include <LiquidCrystal.h>  // required for classic 4 bit interface LCD display (FEATURE_4_BIT_LCD_DISPLAY)
#endif

#ifdef FEATURE_YWROBOT_I2C_DISPLAY
  #include <LiquidCrystal_I2C.h> // required for the generic PCF8574 I2C LCD backpack (FEATURE_YWROBOT_I2C_DISPLAY)
#endif

#ifdef FEATURE_LCD_DISPLAY
  #include "rotator_k3ngdisplay.h"
#endif

#ifdef FEATURE_WIRE_SUPPORT
  #include <Wire.h>  // required for FEATURE_I2C_LCD variants
#endif

#ifdef FEATURE_WIFI
  #include <WiFi.h>
#endif

#include "rotator.h"

#include "rotator_pins.h"

#include "rotator_settings.h"

#if !defined(OPTION_USE_OLD_TIME_CODE)
#if defined(FEATURE_CLOCK)
  #include <TimeLib.h>
#endif
#endif


#include "rotator_language.h"
#include "rotator_debug.h"
#include "rotator_debug_log_activation.h"




/*----------------------- variables -------------------------------------*/

byte incoming_serial_byte = 0;
byte reset_the_unit = 0;
float azimuth = 0;
float raw_azimuth = 0;
float target_azimuth = 0;
float target_raw_azimuth = 0;
byte control_port_buffer[COMMAND_BUFFER_SIZE];
int control_port_buffer_index = 0;
byte az_state = IDLE;
byte debug_mode = DEFAULT_DEBUG_STATE;
int analog_az = 0;
unsigned long last_debug_output_time = 0;
unsigned long az_last_rotate_initiation = 0;
byte azimuth_button_was_pushed = 0;
byte brake_az_engaged = 0;
byte brake_el_engaged = 0;
byte configuration_dirty = 0;
unsigned long last_serial_receive_time = 0;
byte az_slowstart_active = AZ_SLOWSTART_DEFAULT;
byte az_slowdown_active = AZ_SLOWDOWN_DEFAULT;
byte az_request = 0;
float az_request_parm = 0;
byte az_request_queue_state = NONE;
unsigned long az_slowstart_start_time = 0;
byte az_slow_start_step = 0;
unsigned long az_last_step_time = 0;
byte az_slow_down_step = 0;
unsigned long az_timed_slow_down_start_time = 0;
byte backslash_command = 0;
byte normal_az_speed_voltage = 0;
byte current_az_speed_voltage = 0;
double latitude = DEFAULT_LATITUDE;
double longitude = DEFAULT_LONGITUDE;
double altitude_m = DEFAULT_ALTITUDE_M;
byte periodic_debug_dump_time_seconds = 3;

DebugClass debug;

struct config_t {
  byte configuration_struct_version;
  byte configuration_struct_subversion;
  byte calibration_points;
  int analog_az_full_ccw;
  int analog_az_full_cw;
  int analog_el_0_degrees;
  int analog_el_max_elevation;
  float last_azimuth;
  float last_elevation;  
  long last_az_incremental_encoder_position;
  long last_el_incremental_encoder_position;
  float azimuth_offset;
  float elevation_offset;
  int azimuth_starting_point;
  long azimuth_rotation_capability;
  byte brake_az_disabled;
  float clock_timezone_offset;
  byte autopark_active;
  unsigned int autopark_time_minutes;
  byte azimuth_display_mode;
  int park_azimuth;
  int park_elevation;  
  #if defined(FEATURE_MOON_TRACKING) || defined(FEATURE_SUN_TRACKING)
    unsigned int tracking_sun_check_frequency_ms;
    unsigned int tracking_moon_check_frequency_ms;
    unsigned int tracking_sun_minimum_rotation_interval_ms;
    unsigned int tracking_moon_minimum_rotation_interval_ms;
    float tracking_sun_degrees_difference_threshold;
    float tracking_moon_degrees_difference_threshold;
  #endif
  #if defined(FEATURE_SATELLITE_TRACKING)
    float tracking_sat_degrees_difference_threshold; 
    unsigned int tracking_sat_check_frequency_ms;
    unsigned int tracking_sat_minimum_rotation_interval_ms;  
    char current_satellite[17];
  #endif
  #if defined(FEATURE_AUDIBLE_ALERT)
    byte audible_alert_enabled; 
    byte audible_alert_enabled_az_target;
    byte audible_alert_enabled_el_target;
  #endif
  #if defined(FEATURE_CALIBRATION)
    float calibration_az_from[CALIBRATION_POINTS];
    float calibration_az_to[CALIBRATION_POINTS];
    float calibration_el_from[CALIBRATION_POINTS];
    float calibration_el_to[CALIBRATION_POINTS]; 
    byte calibration_az_flag[CALIBRATION_POINTS];  // 0 = empty, 1 = default endpoint, 2 = sun calibration point, 3 = moon calibration point
    byte calibration_el_flag[CALIBRATION_POINTS];
  #endif
} configuration;


#ifdef FEATURE_TIMED_BUFFER
  int timed_buffer_azimuths[TIMED_INTERVAL_ARRAY_SIZE];
  int timed_buffer_number_entries_loaded = 0;
  int timed_buffer_entry_pointer = 0;
  int timed_buffer_interval_value_seconds = 0;
  unsigned long last_timed_buffer_action_time = 0;
  byte timed_buffer_status = EMPTY;
#endif // FEATURE_TIMED_BUFFER

#ifdef FEATURE_ELEVATION_CONTROL
  float elevation = 0;
  float target_elevation = 0;
  byte el_request = 0;
  float el_request_parm = 0;
  byte el_request_queue_state = NONE;
  byte el_slowstart_active = EL_SLOWSTART_DEFAULT;
  byte el_slowdown_active = EL_SLOWDOWN_DEFAULT;
  unsigned long el_slowstart_start_time = 0;
  byte el_slow_start_step = 0;
  unsigned long el_last_step_time = 0;
  byte el_slow_down_step = 0;
  unsigned long el_timed_slow_down_start_time = 0;
  byte normal_el_speed_voltage = 0;
  byte current_el_speed_voltage = 0;
  byte el_state = IDLE;
  int analog_el = 0;
  unsigned long el_last_rotate_initiation = 0;
  byte elevation_button_was_pushed = 0;
  #ifdef FEATURE_TIMED_BUFFER
    int timed_buffer_elevations[TIMED_INTERVAL_ARRAY_SIZE];
  #endif // FEATURE_TIMED_BUFFER  
#endif // FEATURE_ELEVATION_CONTROL

#ifdef FEATURE_ROTARY_ENCODER_SUPPORT
  #ifdef OPTION_ENCODER_HALF_STEP_MODE      // Use the half-step state table (emits a code at 00 and 11)
    const unsigned char ttable[6][4] = {
      { 0x3,  0x2,  0x1, 0x0  }, { 0x23, 0x0,  0x1, 0x0  },
      { 0x13, 0x2,  0x0, 0x0  }, { 0x3,  0x5,  0x4, 0x0  },
      { 0x3,  0x3,  0x4, 0x10 }, { 0x3,  0x5,  0x3, 0x20 }
    };
  #else                                      // Use the full-step state table (emits a code at 00 only)
    const unsigned char ttable[7][4] = {
      {0x0, 0x2, 0x4,  0x0}, {0x3, 0x0, 0x1, 0x10},
      {0x3, 0x2, 0x0,  0x0}, {0x3, 0x2, 0x1,  0x0},
      {0x6, 0x0, 0x4,  0x0}, {0x6, 0x5, 0x0, 0x20},
      {0x6, 0x5, 0x4,  0x0},
    };
  #endif // OPTION_ENCODER_HALF_STEP_MODE

  #ifdef FEATURE_AZ_PRESET_ENCODER            // Rotary Encoder State Tables
    float az_encoder_raw_degrees = 0;
    volatile unsigned char az_encoder_state = 0;
    #ifdef FEATURE_EL_PRESET_ENCODER
      volatile unsigned char el_encoder_state = 0;
      float el_encoder_degrees = 0;
    #endif // FEATURE_EL_PRESET_ENCODER
    byte preset_encoders_state = ENCODER_IDLE;
  #endif // FEATURE_AZ_PRESET_ENCODER
#endif // FEATURE_ROTARY_ENCODER_SUPPORT

#ifdef DEBUG_PROFILE_LOOP_TIME
  float average_loop_time = 0;
#endif // DEBUG_PROFILE_LOOP_TIME

#ifdef FEATURE_AZ_POSITION_PULSE_INPUT
  volatile float az_position_pulse_input_azimuth = 0;
  volatile byte last_known_az_state = 0;
#endif // FEATURE_AZ_POSITION_PULSE_INPUT

#ifdef FEATURE_EL_POSITION_PULSE_INPUT
  volatile float el_position_pulse_input_elevation = 0;
  volatile byte last_known_el_state = 0;
  #ifdef OPTION_EL_PULSE_DEBOUNCE
    unsigned long last_el_pulse_debounce = 0;
  #endif //OPTION_EL_PULSE_DEBOUNCE
#endif // FEATURE_EL_POSITION_PULSE_INPUT

#if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_MASTER_WITH_SERIAL_SLAVE) 
  byte serial_read_event_flag[] = { 0, 0, 0, 0, 0 };
  byte control_port_buffer_carriage_return_flag = 0;
#endif

#if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_MASTER_WITH_SERIAL_SLAVE) || defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE)
  unsigned long last_ping_receive_time = 0;
  byte master_remote_link_state = MASTER_REMOTE_LINK_DOWN;
#endif

#if defined(FEATURE_MASTER_WITH_SERIAL_SLAVE) || defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE)
  byte remote_unit_port_buffer[COMMAND_BUFFER_SIZE];
  int remote_unit_port_buffer_index = 0;
  byte remote_unit_port_buffer_carriage_return_flag = 0;
  unsigned long serial1_last_receive_time = 0;
  // byte remote_unit_command_submitted = 0;
  // unsigned long last_remote_unit_command_time = 0;
  unsigned int remote_unit_command_timeouts = 0;
  unsigned int remote_unit_bad_results = 0;
  unsigned long remote_unit_good_results = 0;
  unsigned int remote_unit_incoming_buffer_timeouts = 0;
  // byte remote_unit_command_results_available = 0;
  // float remote_unit_command_result_float = 0;
  float remote_unit_azimuth_float = 0;
  float remote_unit_elevation_float = 0;
  byte remote_port_rx_sniff = 0;
  byte remote_port_tx_sniff = 0;
  byte suspend_remote_commands = 0;
  #if defined(OPTION_SYNC_MASTER_CLOCK_TO_SLAVE) && defined(FEATURE_CLOCK)
    byte clock_synced_to_remote = 0;
  #endif  
#endif //FEATURE_MASTER_WITH_SERIAL_SLAVE

#ifdef DEBUG_POSITION_PULSE_INPUT
  volatile unsigned long az_pulse_counter = 0;
  volatile unsigned long el_pulse_counter = 0;
  volatile unsigned int az_pulse_counter_ambiguous = 0;
  volatile unsigned int el_pulse_counter_ambiguous = 0;
#endif // DEBUG_POSITION_PULSE_INPUT

#ifdef FEATURE_PARK
  byte park_status = NOT_PARKED;
  byte park_serial_initiated = 0;
#endif // FEATURE_PARK

#ifdef FEATURE_AZ_POSITION_INCREMENTAL_ENCODER
  volatile byte read_azimuth_lock = 0;
  volatile long az_incremental_encoder_position = 0;
  volatile byte az_3_phase_encoder_last_phase_a_state = 0;
  volatile byte az_3_phase_encoder_last_phase_b_state = 0;
  #ifdef DEBUG_AZ_POSITION_INCREMENTAL_ENCODER
    volatile long az_position_incremental_encoder_interrupt = 0;
  #endif // DEBUG_AZ_POSITION_INCREMENTAL_ENCODER
#endif // FEATURE_AZ_POSITION_INCREMENTAL_ENCODER

#ifdef FEATURE_EL_POSITION_INCREMENTAL_ENCODER
  volatile long el_incremental_encoder_position = 0;
  volatile byte el_3_phase_encoder_last_phase_a_state = 0;
  volatile byte el_3_phase_encoder_last_phase_b_state = 0;
  volatile byte read_elevation_lock = 0;
  #ifdef DEBUG_EL_POSITION_INCREMENTAL_ENCODER
    volatile long el_position_incremental_encoder_interrupt = 0;
  #endif // DEBUG_EL_POSITION_INCREMENTAL_ENCODER
#endif // FEATURE_EL_POSITION_INCREMENTAL_ENCODER

#if defined(FEATURE_AZ_POSITION_INCREMENTAL_ENCODER) || defined(FEATURE_EL_POSITION_INCREMENTAL_ENCODER)
  volatile byte service_rotation_lock = 0;
#endif

#if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(CONTROL_PROTOCOL_EMULATION) || defined(FEATURE_CLOCK) || defined(UNDER_DEVELOPMENT_REMOTE_UNIT_COMMANDS)
  CONTROL_PORT_SERIAL_PORT_CLASS * control_port;
#endif

#ifdef FEATURE_MOON_TRACKING
  byte moon_tracking_active = 0;
  byte moon_visible = 0;
  double moon_azimuth = 0;
  double moon_elevation = 0;
#endif // FEATURE_MOON_TRACKING

#ifdef FEATURE_SUN_TRACKING
  float sun_azimuth = 0;
  float sun_elevation = 0;
  cTime c_time;
  cLocation c_loc;
  cSunCoordinates c_sposn;
  byte sun_visible = 0;
  byte sun_tracking_active = 0;
#endif // FEATURE_SUN_TRACKING


#ifdef FEATURE_CLOCK
#if defined(OPTION_USE_OLD_TIME_CODE)
  unsigned long millis_at_last_calibration = 0;

  struct tm {
    byte seconds;         
    byte minutes;
    byte hours;
    byte day;     
    byte month;
    unsigned int year;
  };

  tm current_clock, local_clock, temp_datetime, set_clock;


#endif // defined(OPTION_USE_OLD_TIME_CODE)
#endif // FEATURE_CLOCK


#if defined(FEATURE_GPS) || defined(FEATURE_RTC) || defined(FEATURE_CLOCK)
  byte clock_status = FREE_RUNNING;
#endif // defined(FEATURE_GPS) || defined(FEATURE_RTC)

#ifdef FEATURE_GPS
  byte gps_data_available = 0;
#endif // FEATURE_GPS

#ifdef FEATURE_WIFI
  WiFiServer wifi_server0(WIFI_TCP_PORT);
  WiFiClient wifi_client0;
  byte wifi_port_buffer[COMMAND_BUFFER_SIZE];
  int wifi_port_buffer_index = 0;
#endif //FEATURE_WIFI

#ifdef FEATURE_POWER_SWITCH
  unsigned long last_activity_time = 0;
#endif //FEATURE_POWER_SWITCH

#ifdef FEATURE_AZIMUTH_CORRECTION
  const float azimuth_calibration_from[]  = AZIMUTH_CALIBRATION_FROM_ARRAY;    
  const float azimuth_calibration_to[]    = AZIMUTH_CALIBRATION_TO_ARRAY;
#endif // FEATURE_AZIMUTH_CORRECTION

#ifdef FEATURE_ELEVATION_CORRECTION
  const float elevation_calibration_from[]  = ELEVATION_CALIBRATION_FROM_ARRAY;
  const float elevation_calibration_to[]    = ELEVATION_CALIBRATION_TO_ARRAY;
#endif // FEATURE_ELEVATION_CORRECTION

#ifdef FEATURE_AUTOCORRECT
  byte autocorrect_state_az = AUTOCORRECT_INACTIVE;
  float autocorrect_az = 0;
  unsigned long autocorrect_az_submit_time = 0;
  #ifdef FEATURE_ELEVATION_CONTROL
    byte autocorrect_state_el = AUTOCORRECT_INACTIVE;
    float autocorrect_el = 0;
    unsigned long autocorrect_el_submit_time = 0;
  #endif //FEATURE_ELEVATION_CONTROL
#endif //FEATURE_AUTOCORRECT

#ifdef FEATURE_AZ_POSITION_A2_ABSOLUTE_ENCODER
  float az_a2_encoder = 0;
#endif //FEATURE_AZ_POSITION_A2_ABSOLUTE_ENCODER

#ifdef FEATURE_EL_POSITION_A2_ABSOLUTE_ENCODER
  float el_a2_encoder = 0;
#endif //FEATURE_EL_POSITION_A2_ABSOLUTE_ENCODER 

#if defined(FEATURE_LCD_DISPLAY)
  byte perform_screen_redraw = 0;
  K3NGdisplay k3ngdisplay(LCD_COLUMNS,LCD_ROWS,LCD_UPDATE_TIME);
#endif   

#if defined(FEATURE_AZ_POSITION_HMC5883L) || defined(FEATURE_AZ_POSITION_HMC5883L_USING_JARZEBSKI_LIBRARY)
  HMC5883L compass;
#endif //FEATURE_AZ_POSITION_HMC5883L

#if defined(FEATURE_AZ_POSITION_MECHASOLUTION_QMC5883)  
  MechaQMC5883 compass;
#endif //FEATURE_AZ_POSITION_MECHASOLUTION_QMC5883

#if defined(FEATURE_AZ_POSITION_DFROBOT_QMC5883)  
  DFRobot_QMC5883 compass;
#endif //FEATURE_AZ_POSITION_DFROBOT_QMC5883  

#ifdef FEATURE_EL_POSITION_ADXL345_USING_LOVE_ELECTRON_LIB
  ADXL345 accel;
#endif //FEATURE_EL_POSITION_ADXL345_USING_LOVE_ELECTRON_LIB

#ifdef FEATURE_EL_POSITION_ADXL345_USING_ADAFRUIT_LIB
  Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
#endif //FEATURE_EL_POSITION_ADXL345_USING_ADAFRUIT_LIB

#if defined(FEATURE_EL_POSITION_ADAFRUIT_LSM303) || defined(FEATURE_AZ_POSITION_ADAFRUIT_LSM303)
  Adafruit_LSM303 lsm;
#endif

#if defined(FEATURE_AZ_POSITION_POLOLU_LSM303) || defined(FEATURE_EL_POSITION_POLOLU_LSM303)
  LSM303 compass;
  LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32768, -32768, -32768};
  char report[80];
#endif //FEATURE_AZ_POSITION_POLOLU_LSM303

#if defined(FEATURE_AZ_POSITION_HH12_AS5045_SSI) || defined(FEATURE_AZ_POSITION_HH12_AS5045_SSI_RELATIVE)
  #include "hh12.h"
  hh12 azimuth_hh12;
#endif //FEATURE_AZ_POSITION_HH12_AS5045_SSI

#if defined(FEATURE_EL_POSITION_HH12_AS5045_SSI)  || defined(FEATURE_EL_POSITION_HH12_AS5045_SSI_RELATIVE)
  #include "hh12.h"
  hh12 elevation_hh12;
#endif //FEATURE_EL_POSITION_HH12_AS5045_SSI

#ifdef FEATURE_GPS
  #if defined(OPTION_GPS_USE_TINY_GPS_LIBRARY)
    TinyGPS gps;
  #endif
  #if defined(OPTION_GPS_USE_SPARKFUN_U_BLOX_GNSS_LIBRARY)
    SFE_UBLOX_GNSS myGNSS;
  #endif

#endif //FEATURE_GPS

#ifdef FEATURE_RTC_DS1307
  RTC_DS1307 rtc;
#endif //FEATURE_RTC_DS1307

#ifdef FEATURE_RTC_PCF8583
  PCF8583 rtc(0xA0);
#endif //FEATURE_RTC_PCF8583

#ifdef HARDWARE_EA4TX_ARS_USB
  #undef LCD_COLUMNS
  #undef LCD_ROWS
  #define LCD_COLUMNS 16
  #define LCD_ROWS 2
#endif //HARDWARE_EA4TX_ARS_USB

#ifdef HARDWARE_M0UPU
  #undef LCD_ROWS
  #define LCD_ROWS 2
#endif //HARDWARE_M0UPU

#ifdef FEATURE_AZ_POSITION_A2_ABSOLUTE_ENCODER
  #define AZ_A2_ENCODER_RESOLUTION 32767 /*36000*/
  #define AZ_A2_ENCODER_ADDRESS 0x00
  #define AZ_QUERY_FREQUENCY_MS 250
  #define AZ_A2_ENCODER_MODE MODE_TWO_BYTE_POSITION /*|MODE_MULTITURN*/
#endif  //FEATURE_AZ_POSITION_A2_ABSOLUTE_ENCODER

#ifdef FEATURE_EL_POSITION_A2_ABSOLUTE_ENCODER
  #define EL_A2_ENCODER_RESOLUTION 32767 /*36000*/
  #define EL_A2_ENCODER_ADDRESS 0x00
  #define EL_QUERY_FREQUENCY_MS 250
  #define EL_A2_ENCODER_MODE MODE_MULTITURN /*|MODE_TWO_BYTE_POSITION*/
#endif  //FEATURE_EL_POSITION_A2_ABSOLUTE_ENCODER

#if defined(FEATURE_AZ_POSITION_A2_ABSOLUTE_ENCODER) || defined(FEATURE_EL_POSITION_A2_ABSOLUTE_ENCODER)
  #include "sei_bus.h"
  SEIbus SEIbus1(&Serial1,9600,pin_sei_bus_busy,pin_sei_bus_send_receive);
  //             (Serial Port,Baud Rate,Busy Pin,Send/Receive Pin)
  #define SEI_BUS_COMMAND_TIMEOUT_MS 6000
#endif

#ifdef FEATURE_AZ_POSITION_ROTARY_ENCODER_USE_PJRC_LIBRARY
  Encoder encoder_pjrc_az(az_rotary_position_pin1, az_rotary_position_pin2);
  float encoder_pjrc_previous_az_position  = 0;
  float encoder_pjrc_current_az_position;
#endif

#ifdef FEATURE_EL_POSITION_ROTARY_ENCODER_USE_PJRC_LIBRARY
  Encoder encoder_pjrc_el(el_rotary_position_pin1, el_rotary_position_pin2);
  float encoder_pjrc_previous_el_position  = 0;
  float encoder_pjrc_current_el_position;
#endif  

#ifdef FEATURE_AUTOPARK
  unsigned long last_activity_time_autopark = 0;
#endif  




/* ------------------ let's start doing some stuff now that we got the formalities out of the way --------------------*/

void setup() {

  initialize_serial();

  initialize_peripherals();

  read_settings_from_eeprom();

  initialize_pins();

  read_azimuth(0);

  initialize_display();

  initialize_rotary_encoders();

  initialize_interrupts();

  run_this_once();


}

/*-------------------------- here's where the magic happens --------------------------------*/

void loop() {

  #ifdef DEBUG_LOOP
    control_port->println("loop()");
    control_port->flush();
  #endif // DEBUG_LOOP

  #if defined(FEATURE_CLOCK) && defined(OPTION_USE_OLD_TIME_CODE)
    update_time();
  #endif

  service_process_debug(DEBUG_PROCESSES_SERVICE,0);

  check_serial();
  read_headings();

  service_request_queue();
  service_rotation();
  az_check_operation_timeout();
  #ifdef FEATURE_TIMED_BUFFER
    check_timed_interval();
  #endif // FEATURE_TIMED_BUFFER
  read_headings();
  check_buttons();
  check_overlap();
  check_brake_release();
  #ifdef FEATURE_ELEVATION_CONTROL
    el_check_operation_timeout();
  #endif

  #if defined(FEATURE_AZ_ROTATION_STALL_DETECTION)
    az_check_rotation_stall();
  #endif

  #if defined(FEATURE_EL_ROTATION_STALL_DETECTION) && defined(FEATURE_ELEVATION_CONTROL)
    el_check_rotation_stall();
  #endif

  #ifdef OPTION_MORE_SERIAL_CHECKS
    check_serial();
  #endif

  #ifdef FEATURE_LCD_DISPLAY
    update_lcd_display();
  #endif

  #ifdef OPTION_MORE_SERIAL_CHECKS
    check_serial();
  #endif

  #ifdef OPTION_AZ_MANUAL_ROTATE_LIMITS
    check_az_manual_rotate_limit();
  #endif
  #if defined(OPTION_EL_MANUAL_ROTATE_LIMITS) && defined(FEATURE_ELEVATION_CONTROL)
    check_el_manual_rotate_limit();
  #endif

  #ifndef FEATURE_REMOTE_UNIT_SLAVE
    check_az_speed_pot();
    #ifdef FEATURE_AZ_PRESET_ENCODER            // Rotary Encoder or Preset Selector
      check_preset_encoders();
    #else
      check_az_preset_potentiometer();
    #endif // FEATURE_AZ_PRESET_ENCODER
  #endif // ndef FEATURE_REMOTE_UNIT_SLAVE

  #ifdef DEBUG_DUMP
    output_debug();
  #endif //DEBUG_DUMP

  #ifdef OPTION_MORE_SERIAL_CHECKS
    check_serial();
  #endif

  read_headings();

  service_rotation();

  check_for_dirty_configuration();

  #ifdef DEBUG_PROFILE_LOOP_TIME
    profile_loop_time();
  #endif //DEBUG_PROFILE_LOOP_TIME

  #if defined(FEATURE_MASTER_WITH_SERIAL_SLAVE) || defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE)
    service_remote_communications_incoming_buffer();
  #endif // defined(FEATURE_MASTER_WITH_SERIAL_SLAVE) || defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE)

  #if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_MASTER_WITH_SERIAL_SLAVE) || defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE)
    service_master_remote_link_state();
  #endif

  #ifdef FEATURE_JOYSTICK_CONTROL
    check_joystick();
  #endif // FEATURE_JOYSTICK_CONTROL

  #ifdef FEATURE_ROTATION_INDICATOR_PIN
    service_rotation_indicator_pin();
  #endif // FEATURE_ROTATION_INDICATOR_PIN

  #if defined(FEATURE_PARK)
    service_park();
    #if defined(FEATURE_AUTOPARK)
      service_autopark();
    #endif
  #endif // FEATURE_PARK

  #ifdef FEATURE_LIMIT_SENSE
    check_limit_sense();
  #endif // FEATURE_LIMIT_SENSE

  #ifdef OPTION_MORE_SERIAL_CHECKS
    check_serial();
  #endif

  read_headings();

  service_rotation();

  #ifdef OPTION_MORE_SERIAL_CHECKS
    check_serial();
  #endif

  #ifdef FEATURE_WIFI
    service_wifi();
  #endif // FEATURE_WIFI

  #ifdef FEATURE_POWER_SWITCH
    service_power_switch();
  #endif //FEATURE_POWER_SWITCH

  service_blink_led();

  #ifdef FEATURE_ANALOG_OUTPUT_PINS
    service_analog_output_pins();
  #endif //FEATURE_ANALOG_OUTPUT_PINS


  #if defined(FEATURE_AUDIBLE_ALERT)
    audible_alert(AUDIBLE_ALERT_SERVICE);
  #endif //FEATURE_AUDIBLE_ALERT
  
  #if defined(pin_status_led)
    service_status_led();
  #endif

  #if defined(FEATURE_SATELLITE_TRACKING)
    service_satellite_tracking(0,0);
    service_calc_satellite_data(0,0,0,SERVICE_CALC_DO_NOT_PRINT_HEADER,SERVICE_CALC_SERVICE,SERVICE_CALC_DO_NOT_PRINT_DONE,0);
    //service_calculate_multi_satellite_upcoming_aos_and_los(SERVICE_CALC_SERVICE);
  #endif

  check_for_reset_flag();

  #ifdef OPTION_MORE_SERIAL_CHECKS
    check_serial();
  #endif

} // loop 

/* -------------------------------------- subroutines -----------------------------------------------
 


                                  Where the real work happens...
 


----------------------------------------------------------------------------------------------------- */

#if defined(pin_status_led)
  void service_status_led(){


  #ifdef DEBUG_LOOP
    control_port->println("service_status_led()");
    control_port->flush();
  #endif // DEBUG_LOOP    

    static byte led_is_on = 0;
    static unsigned long last_led_transition = 0;

    if (pin_status_led){
      #if defined(FEATURE_ELEVATION_CONTROL)
        if ((current_az_state() == NOT_DOING_ANYTHING) && (current_el_state() == NOT_DOING_ANYTHING)){
          if (!led_is_on){
            digitalWriteEnhanced(pin_status_led, HIGH);
            led_is_on = 1;
            last_led_transition = millis();
          }
        } else {
          if ((millis() - last_led_transition) > 500){
            if (led_is_on){
              digitalWriteEnhanced(pin_status_led, LOW);
              led_is_on = 0;
            } else {
            digitalWriteEnhanced(pin_status_led, HIGH);
            led_is_on = 1;           
            }
            last_led_transition = millis();
          }
        }
      #else // ! defined(FEATURE_ELEVATION_CONTROL)
        if (current_az_state() == NOT_DOING_ANYTHING){
          if (!led_is_on){
            digitalWriteEnhanced(pin_status_led, HIGH);
            led_is_on = 1;
            last_led_transition = millis();
          }
        } else {
          if ((millis() - last_led_transition) > 500){
            if (led_is_on){
              digitalWriteEnhanced(pin_status_led, LOW);
              led_is_on = 0;
            } else {
            digitalWriteEnhanced(pin_status_led, HIGH);
            led_is_on = 1;           
            }
            last_led_transition = millis();
          }
        }
      #endif //defined(FEATURE_ELEVATION_CONTROL)
    }

  }
#endif //defined(pin_status_led)

// --------------------------------------------------------------

#if defined(FEATURE_AUDIBLE_ALERT)
  byte audible_alert(byte how_called){

    static unsigned long alert_start_time = 0;

    switch(how_called){
      case AUDIBLE_ALERT_SERVICE:
        if ((alert_start_time) && ((millis() - alert_start_time) > AUDIBLE_ALERT_DURATION_MS)) {
          if (AUDIBLE_ALERT_TYPE == 1){
            digitalWriteEnhanced(pin_audible_alert, AUDIBLE_PIN_INACTIVE_STATE);
          }
          if (AUDIBLE_ALERT_TYPE == 2){
            noTone(pin_audible_alert);
          }
          alert_start_time = 0;
        }
        break;
      case AUDIBLE_ALERT_ACTIVATE:
        if (configuration.audible_alert_enabled){
          if (AUDIBLE_ALERT_TYPE == 1){
            digitalWriteEnhanced(pin_audible_alert, AUDIBLE_PIN_ACTIVE_STATE);
          }
          if (AUDIBLE_ALERT_TYPE == 2){
            tone(pin_audible_alert, AUDIBLE_PIN_TONE_FREQ);
          }
          if (millis() == 0){
            alert_start_time = 1;
          } else {
            alert_start_time = millis();
          }
        }
        break; 
      case AUDIBLE_ALERT_MANUAL_ACTIVATE:
        if (!configuration.audible_alert_enabled){
          configuration.audible_alert_enabled = 1;
          configuration_dirty = 1;
        }  
        if ((AUDIBLE_ALERT_TYPE == 1)){
          digitalWriteEnhanced(pin_audible_alert, AUDIBLE_PIN_ACTIVE_STATE);
        }
        if (AUDIBLE_ALERT_TYPE == 2){
          tone(pin_audible_alert, AUDIBLE_PIN_TONE_FREQ);
        }
        alert_start_time = millis();
        break;           
      case AUDIBLE_ALERT_SILENCE:
        if (AUDIBLE_ALERT_TYPE == 1){
          digitalWriteEnhanced(pin_audible_alert, AUDIBLE_PIN_INACTIVE_STATE);
        }
        if (AUDIBLE_ALERT_TYPE == 2){
          noTone(pin_audible_alert);
        }
        break;
      case AUDIBLE_ALERT_DISABLE:
        if (AUDIBLE_ALERT_TYPE == 1){
          digitalWriteEnhanced(pin_audible_alert, AUDIBLE_PIN_INACTIVE_STATE);
        }
        if (AUDIBLE_ALERT_TYPE == 2){
          noTone(pin_audible_alert);
        }
        alert_start_time = 0;
        configuration.audible_alert_enabled = 0;
        configuration_dirty = 1;
        break;  
      case AUDIBLE_ALERT_ENABLE:
        configuration.audible_alert_enabled = 1;
        configuration_dirty = 1;
        break; 

    }

    if (alert_start_time){
      return(1);
    } else {
      return(0);
    }  

  }
#endif //FEATURE_AUDIBLE_ALERT

/* "make something rare and of value" */

// --------------------------------------------------------------



// --------------------------------------------------------------

void read_headings(){

  #ifdef DEBUG_LOOP
    control_port->println("read_headings()");
  #endif // DEBUG_LOOP

  #ifdef DEBUG_PROCESSES
    service_process_debug(DEBUG_PROCESSES_PROCESS_ENTER,PROCESS_READ_HEADINGS);
  #endif


  read_azimuth(0);

  #ifdef FEATURE_ELEVATION_CONTROL
    read_elevation(0);
  #endif

  #ifdef DEBUG_PROCESSES
    service_process_debug(DEBUG_PROCESSES_PROCESS_EXIT,PROCESS_READ_HEADINGS);
  #endif  

}

// --------------------------------------------------------------

void service_blink_led(){


  #ifdef blink_led
  static unsigned long last_blink_led_transition = 0;
  static byte blink_led_status = 0;


  if (((millis() - last_blink_led_transition) >= 1000) && (blink_led != 0)) {
    if (blink_led_status) {
      digitalWriteEnhanced(blink_led, LOW);
      blink_led_status = 0;
    } else {
      digitalWriteEnhanced(blink_led, HIGH);
      blink_led_status = 1;
    }
    last_blink_led_transition = millis();
  }
  #endif // blink_led

  


} /* service_blink_led */


// --------------------------------------------------------------
void check_for_reset_flag(){

  #ifdef DEBUG_PROCESSES
    service_process_debug(DEBUG_PROCESSES_PROCESS_ENTER,PROCESS_MISC_ADMIN);
  #endif    

  static unsigned long detected_reset_flag_time = 0;

  if (reset_the_unit){
    if (detected_reset_flag_time == 0){
      detected_reset_flag_time = millis();
    } else {
      if ((millis()-detected_reset_flag_time) > 5000){  // let things run for 5 seconds


        #ifdef reset_pin
        digitalWrite(reset_pin,HIGH);
        #else // reset_pin

        #ifdef OPTION_RESET_METHOD_JMP_ASM_0
        asm volatile ("  jmp 0"); // reboot!     // doesn't work on Arduino Mega but works on SainSmart Mega.
        //wdt_enable(WDTO_30MS); while(1) {};  //doesn't work on Mega
        #else //OPTION_RESET_METHOD_JMP_ASM_0
        setup();
        reset_the_unit = 0;
        #endif //OPTION_RESET_METHOD_JMP_ASM_0
        
        #endif //reset_pin
      }
    }
  }

  #ifdef DEBUG_PROCESSES
    service_process_debug(DEBUG_PROCESSES_PROCESS_EXIT,PROCESS_MISC_ADMIN);
  #endif    

}

// --------------------------------------------------------------
#ifdef DEBUG_PROFILE_LOOP_TIME
void profile_loop_time(){

  static unsigned long last_time = 0;
  static unsigned long last_print_time = 0;


  average_loop_time = (average_loop_time + (millis() - last_time)) / 2.0;
  last_time = millis();

  if (debug_mode) {
    if ((millis() - last_print_time) > 1000) {
      debug.print("avg loop time: ");
      debug.print(average_loop_time, 2);
      debug.println("");
      last_print_time = millis();
    }
  }


} /* profile_loop_time */

#endif //DEBUG_PROFILE_LOOP_TIME
// --------------------------------------------------------------
void check_az_speed_pot() {

  static unsigned long last_pot_check_time = 0;
  int pot_read = 0;
  byte new_azimuth_speed_voltage = 0;


  if (az_speed_pot /*&& azimuth_speed_voltage*/ && ((millis() - last_pot_check_time) > 500)) {
    pot_read = analogReadEnhanced(az_speed_pot);
    new_azimuth_speed_voltage = map(pot_read, SPEED_POT_LOW, SPEED_POT_HIGH, SPEED_POT_LOW_MAP, SPEED_POT_HIGH_MAP);
    if (new_azimuth_speed_voltage != normal_az_speed_voltage) {
      #ifdef DEBUG_AZ_SPEED_POT
        if (debug_mode) {
          debug.print("check_az_speed_pot: normal_az_speed_voltage: ");
          debug.print(normal_az_speed_voltage);
          debug.print(" new_azimuth_speed_voltage:");
          debug.print(new_azimuth_speed_voltage);
          debug.println("");
        }
      #endif // DEBUG_AZ_SPEED_POT
      normal_az_speed_voltage = new_azimuth_speed_voltage;
      update_az_variable_outputs(normal_az_speed_voltage);
      #if defined(OPTION_EL_SPEED_FOLLOWS_AZ_SPEED) && defined(FEATURE_ELEVATION_CONTROL)
        normal_el_speed_voltage = new_azimuth_speed_voltage;
        update_el_variable_outputs(normal_el_speed_voltage);
      #endif // OPTION_EL_SPEED_FOLLOWS_AZ_SPEED
    }
    last_pot_check_time = millis();
  }

} /* check_az_speed_pot */
// --------------------------------------------------------------
void check_az_preset_potentiometer() {


  byte check_pot = 0;
  static unsigned long last_pot_check_time = 0;
  static int last_pot_read = 9999;
  int pot_read = 0;
  int new_pot_azimuth = 0;
  byte button_read = 0;
  static byte pot_changed_waiting = 0;

  if (az_preset_pot) {
    if (last_pot_read == 9999) {  // initialize last_pot_read the first time we hit this subroutine
      last_pot_read = analogReadEnhanced(az_preset_pot);
    }

    if (!pot_changed_waiting) {
      if (preset_start_button) { // if we have a preset start button, check it
        button_read = digitalReadEnhanced(preset_start_button);
        if (button_read == BUTTON_ACTIVE_STATE) {
          check_pot = 1;
        }
      } else {  // if not, check the pot every 500 mS
        if ((millis() - last_pot_check_time) < 250) {
          check_pot = 1;
        }
      }

      if (check_pot) {
        pot_read = analogReadEnhanced(az_preset_pot);
        new_pot_azimuth = map(pot_read, AZ_PRESET_POT_FULL_CW, AZ_PRESET_POT_FULL_CCW, AZ_PRESET_POT_FULL_CW_MAP, AZ_PRESET_POT_FULL_CCW_MAP);
        if ((abs(last_pot_read - pot_read) > 4) && (abs(new_pot_azimuth - raw_azimuth) > AZIMUTH_TOLERANCE)) {
          pot_changed_waiting = 1;
          #ifdef DEBUG_AZ_PRESET_POT
          if (debug_mode) {
            debug.println("check_az_preset_potentiometer: in pot_changed_waiting");
          }
          #endif // DEBUG_AZ_PRESET_POT
          last_pot_read = pot_read;
        }
      }
      last_pot_check_time = millis();
    } else {  // we're in pot change mode
      pot_read = analogReadEnhanced(az_preset_pot);
      if (abs(pot_read - last_pot_read) > 3) {  // if the pot has changed, reset the timer
        last_pot_check_time = millis();
        last_pot_read = pot_read;
      } else {
        if ((millis() - last_pot_check_time) >= 250) {  // has it been awhile since the last pot change?
          new_pot_azimuth = map(pot_read, AZ_PRESET_POT_FULL_CW, AZ_PRESET_POT_FULL_CCW, AZ_PRESET_POT_FULL_CW_MAP, AZ_PRESET_POT_FULL_CCW_MAP);
          #ifdef DEBUG_AZ_PRESET_POT
          if (debug_mode) {
            debug.print("check_az_preset_potentiometer: pot change - current raw_azimuth: ");
            debug.print(raw_azimuth,0);
            debug.print(" new_azimuth: ");
            debug.print(new_pot_azimuth);
            debug.println("");
          }
          #endif // DEBUG_AZ_PRESET_POT
          submit_request(AZ, REQUEST_AZIMUTH_RAW, new_pot_azimuth, DBG_CHECK_AZ_PRESET_POT);
          pot_changed_waiting = 0;
          last_pot_read = pot_read;
          last_pot_check_time = millis();
        }
      }
    }
  } // if (az_preset_pot)
} /* check_az_preset_potentiometer */
// --------------------------------------------------------------

void initialize_rotary_encoders(){


  #ifdef DEBUG_LOOP
    control_port->println("initialize_rotary_encoders()");
    control_port->flush();
  #endif // DEBUG_LOOP

  #ifdef FEATURE_AZ_PRESET_ENCODER
  pinModeEnhanced(az_rotary_preset_pin1, INPUT);
  pinModeEnhanced(az_rotary_preset_pin2, INPUT);
  az_encoder_raw_degrees = raw_azimuth;
  #ifdef OPTION_ENCODER_ENABLE_PULLUPS
  digitalWriteEnhanced(az_rotary_preset_pin1, HIGH);
  digitalWriteEnhanced(az_rotary_preset_pin2, HIGH);
  #endif // OPTION_ENCODER_ENABLE_PULLUPS
  #endif // FEATURE_AZ_PRESET_ENCODER


  #if defined(FEATURE_EL_PRESET_ENCODER) && defined(FEATURE_ELEVATION_CONTROL)
    pinModeEnhanced(el_rotary_preset_pin1, INPUT);
    pinModeEnhanced(el_rotary_preset_pin2, INPUT);
    el_encoder_degrees = elevation;
    #ifdef OPTION_ENCODER_ENABLE_PULLUPS
      digitalWriteEnhanced(el_rotary_preset_pin1, HIGH);
      digitalWriteEnhanced(el_rotary_preset_pin2, HIGH);
    #endif // OPTION_ENCODER_ENABLE_PULLUPS
  #endif // defined(FEATURE_EL_PRESET_ENCODER) && defined(FEATURE_ELEVATION_CONTROL)

  #ifdef FEATURE_AZ_POSITION_ROTARY_ENCODER
    #ifdef OPTION_ENCODER_ENABLE_PULLUPS
      pinModeEnhanced(az_rotary_position_pin1, INPUT_PULLUP);
      pinModeEnhanced(az_rotary_position_pin2, INPUT_PULLUP);
    #else
      pinModeEnhanced(az_rotary_position_pin1, INPUT);
      pinModeEnhanced(az_rotary_position_pin2, INPUT);
    #endif // OPTION_ENCODER_ENABLE_PULLUPS
  #endif // FEATURE_AZ_POSITION_ROTARY_ENCODER


  #ifdef FEATURE_EL_POSITION_ROTARY_ENCODER
  pinModeEnhanced(el_rotary_position_pin1, INPUT);
  pinModeEnhanced(el_rotary_position_pin2, INPUT);
  #ifdef OPTION_ENCODER_ENABLE_PULLUPS
  digitalWriteEnhanced(el_rotary_position_pin1, HIGH);
  digitalWriteEnhanced(el_rotary_position_pin2, HIGH);
  #endif // OPTION_ENCODER_ENABLE_PULLUPS
  #endif // FEATURE_EL_POSITION_ROTARY_ENCODER


  #ifdef FEATURE_AZ_POSITION_INCREMENTAL_ENCODER
    pinMode(az_incremental_encoder_pin_phase_a, INPUT);
    pinMode(az_incremental_encoder_pin_phase_b, INPUT);
    pinMode(az_incremental_encoder_pin_phase_z, INPUT);
    #ifdef OPTION_INCREMENTAL_ENCODER_PULLUPS
      digitalWrite(az_incremental_encoder_pin_phase_a, HIGH);
      digitalWrite(az_incremental_encoder_pin_phase_b, HIGH);
      digitalWrite(az_incremental_encoder_pin_phase_z, HIGH);
    #endif // OPTION_INCREMENTAL_ENCODER_PULLUPS
    attachInterrupt(AZ_POSITION_INCREMENTAL_ENCODER_A_PIN_INTERRUPT, az_position_incremental_encoder_interrupt_handler, CHANGE);
    attachInterrupt(AZ_POSITION_INCREMENTAL_ENCODER_B_PIN_INTERRUPT, az_position_incremental_encoder_interrupt_handler, CHANGE);
    delay(250);
    az_3_phase_encoder_last_phase_a_state = digitalRead(az_incremental_encoder_pin_phase_a);
    az_3_phase_encoder_last_phase_b_state = digitalRead(az_incremental_encoder_pin_phase_b);
  #endif // FEATURE_AZ_POSITION_INCREMENTAL_ENCODER

  #if defined(FEATURE_EL_POSITION_INCREMENTAL_ENCODER) && defined(FEATURE_ELEVATION_CONTROL)
  pinMode(el_incremental_encoder_pin_phase_a, INPUT);
  pinMode(el_incremental_encoder_pin_phase_b, INPUT);
  pinMode(el_incremental_encoder_pin_phase_z, INPUT);
  #ifdef OPTION_INCREMENTAL_ENCODER_PULLUPS
  digitalWrite(el_incremental_encoder_pin_phase_a, HIGH);
  digitalWrite(el_incremental_encoder_pin_phase_b, HIGH);
  digitalWrite(el_incremental_encoder_pin_phase_z, HIGH);
  #endif // OPTION_INCREMENTAL_ENCODER_PULLUPS
  attachInterrupt(EL_POSITION_INCREMENTAL_ENCODER_A_PIN_INTERRUPT, el_position_incremental_encoder_interrupt_handler, CHANGE);
  attachInterrupt(EL_POSITION_INCREMENTAL_ENCODER_B_PIN_INTERRUPT, el_position_incremental_encoder_interrupt_handler, CHANGE);
  delay(250);
  el_3_phase_encoder_last_phase_a_state = digitalRead(el_incremental_encoder_pin_phase_a);
  el_3_phase_encoder_last_phase_b_state = digitalRead(el_incremental_encoder_pin_phase_b);
  #endif // defined(FEATURE_EL_POSITION_INCREMENTAL_ENCODER) && defined(FEATURE_ELEVATION_CONTROL)

} /* initialize_rotary_encoders */


// --------------------------------------------------------------
#ifdef FEATURE_AZ_PRESET_ENCODER
void check_preset_encoders(){

  static unsigned long last_encoder_change_time = 0;
  byte button_read = HIGH;
  byte number_columns = 0;
  static byte submit_encoder_change = 0;
  static unsigned long last_preset_start_button_start = 0;
  static unsigned long last_preset_start_button_kill = 0;
  static unsigned long last_encoder_move = 0;

  #ifdef FEATURE_AZ_PRESET_ENCODER
  static unsigned long az_timestamp[5];
  #endif // FEATURE_AZ_PRESET_ENCODER

  #ifdef FEATURE_EL_PRESET_ENCODER
  static unsigned long el_timestamp[5];
  #endif // FEATURE_EL_PRESET_ENCODER

  #ifdef FEATURE_AZ_PRESET_ENCODER
  az_encoder_state = ttable[az_encoder_state & 0xf][((digitalReadEnhanced(az_rotary_preset_pin2) << 1) | digitalReadEnhanced(az_rotary_preset_pin1))];
  unsigned char az_encoder_result = az_encoder_state & 0x30;
  #ifdef DEBUG_PRESET_ENCODERS
  static byte last_az_rotary_preset_pin1 = 0;
  static byte last_az_rotary_preset_pin2 = 0;

  if ((debug_mode) && (( last_az_rotary_preset_pin1 != digitalReadEnhanced(az_rotary_preset_pin1)) || ( last_az_rotary_preset_pin2 != digitalReadEnhanced(az_rotary_preset_pin2)))) {
    debug.print("check_preset_encoders: az_rotary_preset_pin1: ");
    debug.print(digitalReadEnhanced(az_rotary_preset_pin1));
    debug.print(" az_rotary_preset_pin2: ");
    debug.print(digitalReadEnhanced(az_rotary_preset_pin2));
    debug.print(" encoder_result: ");
    debug.print(az_encoder_result);
    debug.println("");
  }
  last_az_rotary_preset_pin1 = digitalReadEnhanced(az_rotary_preset_pin1);
  last_az_rotary_preset_pin2 = digitalReadEnhanced(az_rotary_preset_pin2);
  #endif // DEBUG_PRESET_ENCODERS

  #endif // FEATURE_AZ_PRESET_ENCODER

  #ifdef FEATURE_EL_PRESET_ENCODER
  el_encoder_state = ttable[el_encoder_state & 0xf][((digitalReadEnhanced(el_rotary_preset_pin2) << 1) | digitalReadEnhanced(el_rotary_preset_pin1))];
  unsigned char el_encoder_result = el_encoder_state & 0x30;
  #endif // FEATURE_EL_PRESET_ENCODER

  #ifdef FEATURE_AZ_PRESET_ENCODER
  if (az_encoder_result) {                                 // If rotary encoder modified
    az_timestamp[0] = az_timestamp[1];                     // Encoder step timer
    az_timestamp[1] = az_timestamp[2];
    az_timestamp[2] = az_timestamp[3];
    az_timestamp[3] = az_timestamp[4];
    az_timestamp[4] = millis();

    last_encoder_move = millis();


    #ifdef DEBUG_PRESET_ENCODERS
    char tempchar[12] = "";
    if (debug_mode) {
      debug.print("check_preset_encoders: az_timestamps:");
      for (int y = 0; y < 5; y++) {
        debug.print(" ");
        dtostrf(az_timestamp[y],0,0,tempchar);
        debug.print(tempchar);
      }
      debug.println("");
    }
    #endif // DEBUG_PRESET_ENCODERS

    unsigned long az_elapsed_time = (az_timestamp[4] - az_timestamp[0]); // Encoder step time difference for 10's step

    #ifdef OPTION_PRESET_ENCODER_RELATIVE_CHANGE
    if ((preset_encoders_state == ENCODER_IDLE) || (preset_encoders_state == ENCODER_EL_PENDING)) {
      if (az_request_queue_state == IN_PROGRESS_TO_TARGET) {
        az_encoder_raw_degrees = target_raw_azimuth;
      } else {
        az_encoder_raw_degrees = raw_azimuth;
      }
    }
    #endif // OPTION_PRESET_ENCODER_RELATIVE_CHANGE

    // bbbbbb

    #ifndef OPTION_PRESET_ENCODER_0_360_DEGREES
    if (az_encoder_result == DIR_CW) {
      #ifdef DEBUG_PRESET_ENCODERS
      debug.print("check_preset_encoders: az CW");
      #endif // DEBUG_PRESET_ENCODERS
      if (az_elapsed_time < 250 /* mSec */) {
        az_encoder_raw_degrees += (5);
      } else { az_encoder_raw_degrees += (1); };                                                                                      // Single deg increase unless encoder turned quickly then 10's step
      // if (az_encoder_raw_degrees >=360) {az_encoder_raw_degrees -= 360;};
      if (az_encoder_raw_degrees > ((configuration.azimuth_starting_point + configuration.azimuth_rotation_capability))) {
        az_encoder_raw_degrees =
          ((configuration.azimuth_starting_point)
           /* + ((azimuth_starting_point+azimuth_rotation_capability)) - az_encoder_raw_degrees*/);
      }
    }
    if (az_encoder_result == DIR_CCW) {
      #ifdef DEBUG_PRESET_ENCODERS
      debug.print("check_preset_encoders: az CCW");
      #endif // DEBUG_PRESET_ENCODERS
      if (az_elapsed_time < 250 /* mSec */) {
        az_encoder_raw_degrees -= (5);
      } else { az_encoder_raw_degrees -= (1); };                                                                                       // Single deg decrease unless encoder turned quickly then 10's step
      // if (az_encoder_raw_degrees < 0) {az_encoder_raw_degrees = 360;};
      if (az_encoder_raw_degrees < (configuration.azimuth_starting_point)) {
        az_encoder_raw_degrees = (((configuration.azimuth_starting_point + configuration.azimuth_rotation_capability))
                                  /*- (az_encoder_raw_degrees-(azimuth_starting_point))*/);
      }
    }
    #else //ndef OPTION_PRESET_ENCODER_0_360_DEGREES
    if (az_encoder_result == DIR_CW) {
      #ifdef DEBUG_PRESET_ENCODERS
      debug.print("check_preset_encoders: az CW");
      #endif // DEBUG_PRESET_ENCODERS
      if (az_elapsed_time < 250) {  // Single deg increase unless encoder turned quickly then 10's step
        az_encoder_raw_degrees += (5);
      } else {
        az_encoder_raw_degrees += (1);
      }                                                                                   
      if (az_encoder_raw_degrees > 360) {
        //az_encoder_raw_degrees = 360;
        az_encoder_raw_degrees = 0;
      //} else {
       // if (az_encoder_raw_degrees < 0) {
       //   az_encoder_raw_degrees = 0;
       // }
      }
    }
    if (az_encoder_result == DIR_CCW) {
      #ifdef DEBUG_PRESET_ENCODERS
      debug.print("check_preset_encoders: az CCW");
      #endif // DEBUG_PRESET_ENCODERS
      if (az_elapsed_time < 250) {  // Single deg decrease unless encoder turned quickly then 10's step
        az_encoder_raw_degrees -= (5);
      } else { 
        az_encoder_raw_degrees -= (1); 
      }                                                                                       
      //if (az_encoder_raw_degrees > 360) {
      //  az_encoder_raw_degrees = 360;
      //} else {
        if (az_encoder_raw_degrees < 0) {
          //az_encoder_raw_degrees = 0;
          az_encoder_raw_degrees = 359;
        }
      //}
    }
    #endif //ndef OPTION_PRESET_ENCODER_0_360_DEGREES

    last_encoder_change_time = millis();     // Encoder Check Timer

    if (preset_encoders_state == ENCODER_IDLE) {
      preset_encoders_state = ENCODER_AZ_PENDING;
    } else {
      if (preset_encoders_state == ENCODER_EL_PENDING) {
        preset_encoders_state = ENCODER_AZ_EL_PENDING;
      }
    }

    #ifdef DEBUG_PRESET_ENCODERS
    debug.print("check_preset_encoders: az target: ");
    dtostrf(az_encoder_raw_degrees,0,1,tempchar);
    debug.println(tempchar);
    #endif // DEBUG_PRESET_ENCODERS

  } // if (az_encoder_result)
  #endif // FEATURE_AZ_PRESET_ENCODER

  #ifdef FEATURE_EL_PRESET_ENCODER

  #ifdef OPTION_PRESET_ENCODER_RELATIVE_CHANGE
  if ((preset_encoders_state == ENCODER_IDLE) || (preset_encoders_state == ENCODER_AZ_PENDING)) {
    if (el_request_queue_state == IN_PROGRESS_TO_TARGET) {
      el_encoder_degrees = target_elevation;
    } else {
      el_encoder_degrees = elevation;
    }
  }
    #endif // OPTION_PRESET_ENCODER_RELATIVE_CHANGE

  if (el_encoder_result) {                                 // If rotary encoder modified
    el_timestamp[0] = el_timestamp[1];                     // Encoder step timer
    el_timestamp[1] = el_timestamp[2];
    el_timestamp[2] = el_timestamp[3];
    el_timestamp[3] = el_timestamp[4];
    el_timestamp[4] = millis();

    last_encoder_move = millis();

    unsigned long el_elapsed_time = (el_timestamp[4] - el_timestamp[0]); // Encoder step time difference for 10's step

    if (el_encoder_result == DIR_CW) {                      // Rotary Encoder CW 0 - 359 Deg
      #ifdef DEBUG_PRESET_ENCODERS
      debug.println("check_preset_encoders: el CW");
      #endif // DEBUG_PRESET_ENCODERS
      if (el_elapsed_time < 250) {
        el_encoder_degrees += (5);
      } else { el_encoder_degrees += (1); };                                                                       // Single deg increase unless encoder turned quickly then 10's step
      if (el_encoder_degrees > (180)) {
        el_encoder_degrees = (180);
      }
      ;
    }
    if (el_encoder_result == DIR_CCW) {
      #ifdef DEBUG_PRESET_ENCODERS
      debug.println("check_preset_encoders: el CCW");
      #endif // DEBUG_PRESET_ENCODERS
      // Rotary Encoder CCW 359 - 0 Deg
      if (el_elapsed_time < 250) {
        el_encoder_degrees -= (5);
      } else { el_encoder_degrees -= (1); };                                                                        // Single deg decrease unless encoder turned quickly then 10's step
      if (el_encoder_degrees < 0) {
        el_encoder_degrees = 0;
      }
      ;
    }
    last_encoder_change_time = millis();     // Encoder Check Timer

    if (preset_encoders_state == ENCODER_IDLE) {
      preset_encoders_state = ENCODER_EL_PENDING;
    } else {
      if (preset_encoders_state == ENCODER_AZ_PENDING) {
        preset_encoders_state = ENCODER_AZ_EL_PENDING;
      }
    }

    #ifdef DEBUG_PRESET_ENCODERS
      debug.print("check_preset_encoders: el target: ");
      char tempchar[16];
      dtostrf(el_encoder_degrees,0,1,tempchar);
      debug.println(tempchar);
    #endif // DEBUG_PRESET_ENCODERS


  } // if (el_encoder_result)


  #endif // FEATURE_EL_PRESET_ENCODER

  if ((preset_encoders_state != ENCODER_IDLE) && (!submit_encoder_change)) { // Check button or timer
    if (preset_start_button) {                                               // if we have a preset start button, check it
      button_read = digitalReadEnhanced(preset_start_button);
      if (button_read == BUTTON_ACTIVE_STATE) {
        submit_encoder_change = 1;
        last_preset_start_button_start = millis();

        #ifdef DEBUG_PRESET_ENCODERS
        debug.println("check_preset_encoders: preset_start_button submit_encoder_change");
        #endif // DEBUG_PRESET_ENCODERS
      }
    } else {
      if ((millis() - last_encoder_change_time) > PRESET_ENCODER_CHANGE_TIME_MS) {       // if enc not changed for more than 2 sec, rotate to target
        #ifdef DEBUG_PRESET_ENCODERS
        debug.println("check_preset_encoders: timer submit_encoder_change");
        #endif // DEBUG_PRESET_ENCODERS
        submit_encoder_change = 1;
      }
    }
  } // if (!enc_changed_waiting)


  if (preset_start_button) {                                         // if we have a preset start button, check it
    button_read = digitalReadEnhanced(preset_start_button);
    if ((button_read == BUTTON_ACTIVE_STATE) && (!submit_encoder_change) && ((millis() - last_preset_start_button_start) > 250)
        && ((millis() - last_preset_start_button_kill) > 250) && (preset_encoders_state == ENCODER_IDLE)) {
      #ifdef DEBUG_PRESET_ENCODERS
      debug.println("check_preset_encoders: preset button kill");
      #endif // DEBUG_PRESET_ENCODERS
      #ifdef FEATURE_AZ_PRESET_ENCODER
      if (az_state != IDLE) {
        submit_request(AZ, REQUEST_KILL, 0, DBG_CHECK_PRESET_ENCODERS_NOT_IDLE);
      }
      #endif // FEATURE_AZ_PRESET_ENCODER
      #if defined(FEATURE_EL_PRESET_ENCODER) && defined(FEATURE_ELEVATION_CONTROL)
      if (el_state != IDLE) {
        submit_request(EL, REQUEST_KILL, 0, DBG_CHECK_PRESET_ENCODERS_NOT_IDLE);
      }
      #endif // defined(FEATURE_EL_PRESET_ENCODER) && defined(FEATURE_ELEVATION_CONTROL)
      last_preset_start_button_kill = millis();
    }
  }

  if ((submit_encoder_change) && (button_read == BUTTON_INACTIVE_STATE)) {
    #ifdef DEBUG_PRESET_ENCODERS
    debug.println("check_preset_encoders: submit_encoder_change ");
    #endif // DEBUG_PRESET_ENCODERS


    if ((preset_encoders_state == ENCODER_AZ_PENDING) || (preset_encoders_state == ENCODER_AZ_EL_PENDING)) {
      #ifndef OPTION_PRESET_ENCODER_0_360_DEGREES
      submit_request(AZ, REQUEST_AZIMUTH_RAW, az_encoder_raw_degrees, DBG_CHECK_PRESET_ENCODERS_PENDING);
      #else
      submit_request(AZ, REQUEST_AZIMUTH, az_encoder_raw_degrees, DBG_CHECK_PRESET_ENCODERS_PENDING);
      #endif //ndef OPTION_PRESET_ENCODER_0_360_DEGREES
    }

    #ifdef FEATURE_EL_PRESET_ENCODER
    if ((preset_encoders_state == ENCODER_EL_PENDING) || (preset_encoders_state == ENCODER_AZ_EL_PENDING)) {
      submit_request(EL, REQUEST_ELEVATION, el_encoder_degrees, DBG_CHECK_PRESET_ENCODERS_PENDING);
    }
    #endif // FEATURE_EL_PRESET_ENCODER

    preset_encoders_state = ENCODER_IDLE;
    submit_encoder_change = 0;
  } // if (submit_encoder_change)

  if ((preset_start_button) && (preset_encoders_state != ENCODER_IDLE) && ((millis() - last_encoder_move) > ENCODER_PRESET_TIMEOUT)) { // timeout if we have a preset start button
    preset_encoders_state = ENCODER_IDLE;
    #ifdef FEATURE_LCD_DISPLAY
    //push_lcd_update = 1;                     // push an LCD update
    #endif // FEATURE_LCD_DISPLAY
  }

} /* check_preset_encoders */

#endif // FEATURE_AZ_PRESET_ENCODER

// --------------------------------------------------------------

#ifdef OPTION_AZ_MANUAL_ROTATE_LIMITS
void check_az_manual_rotate_limit() {

  if ((current_az_state() == ROTATING_CCW) && (raw_azimuth <= (AZ_MANUAL_ROTATE_CCW_LIMIT))) {
    #ifdef DEBUG_AZ_MANUAL_ROTATE_LIMITS
      debug.print("check_az_manual_rotate_limit: stopping - hit AZ_MANUAL_ROTATE_CCW_LIMIT of ");
      debug.print(AZ_MANUAL_ROTATE_CCW_LIMIT);
      debug.println("");
    #endif // DEBUG_AZ_MANUAL_ROTATE_LIMITS
    submit_request(AZ, REQUEST_KILL, 0, DBG_CHECK_AZ_MANUAL_ROTATE_LIMIT_CCW);
  }
  if ((current_az_state() == ROTATING_CW) && (raw_azimuth >= (AZ_MANUAL_ROTATE_CW_LIMIT))) {
    #ifdef DEBUG_AZ_MANUAL_ROTATE_LIMITS
      debug.print("check_az_manual_rotate_limit: stopping - hit AZ_MANUAL_ROTATE_CW_LIMIT of ");
      debug.print(AZ_MANUAL_ROTATE_CW_LIMIT);
      debug.println("");
    #endif // DEBUG_AZ_MANUAL_ROTATE_LIMITS
    submit_request(AZ, REQUEST_KILL, 0, DBG_CHECK_AZ_MANUAL_ROTATE_LIMIT_CW);
  }
} /* check_az_manual_rotate_limit */
#endif // #ifdef OPTION_AZ_MANUAL_ROTATE_LIMITS

// --------------------------------------------------------------

#if defined(OPTION_EL_MANUAL_ROTATE_LIMITS) && defined(FEATURE_ELEVATION_CONTROL)
void check_el_manual_rotate_limit() {

  if ((current_el_state() == ROTATING_DOWN) && (elevation <= (EL_MANUAL_ROTATE_DOWN_LIMIT))) {
    #ifdef DEBUG_EL_MANUAL_ROTATE_LIMITS
      debug.print("check_el_manual_rotate_limit: stopping - hit EL_MANUAL_ROTATE_DOWN_LIMIT of ");
      debug.print(EL_MANUAL_ROTATE_DOWN_LIMIT);
      debug.println("");
    #endif // DEBUG_EL_MANUAL_ROTATE_LIMITS
    submit_request(EL, REQUEST_KILL, 0, DBG_CHECK_EL_MANUAL_ROTATE_LIMIT_DOWN);
  }
  if ((current_el_state() == ROTATING_UP) && (elevation >= (EL_MANUAL_ROTATE_UP_LIMIT))) {
    #ifdef DEBUG_EL_MANUAL_ROTATE_LIMITS
      debug.print("check_el_manual_rotate_limit: stopping - hit EL_MANUAL_ROTATE_UP_LIMIT of ");
      debug.print(EL_MANUAL_ROTATE_UP_LIMIT);
      debug.println("");
    #endif // DEBUG_EL_MANUAL_ROTATE_LIMITS
    submit_request(EL, REQUEST_KILL, 0, DBG_CHECK_EL_MANUAL_ROTATE_LIMIT_UP);
  }
} /* check_el_manual_rotate_limit */
#endif // #ifdef OPTION_EL_MANUAL_ROTATE_LIMITS


// --------------------------------------------------------------
void check_brake_release() {

  #ifdef DEBUG_PROCESSES
    service_process_debug(DEBUG_PROCESSES_PROCESS_ENTER,PROCESS_MISC_ADMIN);
  #endif    

  static byte in_az_brake_release_delay = 0;
  static unsigned long az_brake_delay_start_time = 0;

  #ifdef FEATURE_ELEVATION_CONTROL
    static byte in_el_brake_release_delay = 0;
    static unsigned long el_brake_delay_start_time = 0;
  #endif // FEATURE_ELEVATION_CONTROL

  if ((az_state == IDLE) && (brake_az_engaged)) {
    if (in_az_brake_release_delay) {
      if ((millis() - az_brake_delay_start_time) > AZ_BRAKE_DELAY) {
        brake_release(AZ, BRAKE_RELEASE_OFF);
        in_az_brake_release_delay = 0;
      }
    } else {
      az_brake_delay_start_time = millis();
      in_az_brake_release_delay = 1;
    }
  }

  if ((az_state != IDLE) && (brake_az_engaged)) {in_az_brake_release_delay = 0;}

  #ifdef FEATURE_ELEVATION_CONTROL
  if ((el_state == IDLE) && (brake_el_engaged)) {
    if (in_el_brake_release_delay) {
      if ((millis() - el_brake_delay_start_time) > EL_BRAKE_DELAY) {
        brake_release(EL, BRAKE_RELEASE_OFF);
        in_el_brake_release_delay = 0;
      }
    } else {
      el_brake_delay_start_time = millis();
      in_el_brake_release_delay = 1;
    }
  }

  if ((el_state != IDLE) && (brake_el_engaged)) {in_el_brake_release_delay = 0;}  
  #endif // FEATURE_ELEVATION_CONTROL

  #ifdef DEBUG_PROCESSES
    service_process_debug(DEBUG_PROCESSES_PROCESS_EXIT,PROCESS_MISC_ADMIN);
  #endif      

} /* check_brake_release */

// --------------------------------------------------------------
void brake_release(byte az_or_el, byte operation){

  if (az_or_el == AZ) {
    if (brake_az && (configuration.brake_az_disabled == 0)) {
      if (operation == BRAKE_RELEASE_ON) {
        digitalWriteEnhanced(brake_az, BRAKE_ACTIVE_STATE);
        brake_az_engaged = 1;
        #ifdef DEBUG_BRAKE
        debug.println("brake_release: brake_az BRAKE_RELEASE_ON");
        #endif // DEBUG_BRAKE
      } else {
        digitalWriteEnhanced(brake_az, BRAKE_INACTIVE_STATE);
        brake_az_engaged = 0;
        #ifdef DEBUG_BRAKE
        debug.println("brake_release: brake_az BRAKE_RELEASE_OFF");
        #endif // DEBUG_BRAKE
      }
    }
  } else {
    #ifdef FEATURE_ELEVATION_CONTROL
    if (brake_el) {
      if (operation == BRAKE_RELEASE_ON) {  
        digitalWriteEnhanced(brake_el, BRAKE_ACTIVE_STATE);
        brake_el_engaged = 1;
        #ifdef DEBUG_BRAKE
        debug.println("brake_release: brake_el BRAKE_RELEASE_ON");
        #endif // DEBUG_BRAKE
      } else {
        digitalWriteEnhanced(brake_el, BRAKE_INACTIVE_STATE);
        brake_el_engaged = 0;
        #ifdef DEBUG_BRAKE
        debug.println("brake_release: brake_el BRAKE_RELEASE_OFF");
        #endif // DEBUG_BRAKE
      }
    }
    #endif // FEATURE_ELEVATION_CONTROL
  }
} /* brake_release */

// --------------------------------------------------------------
void check_overlap(){

  #if defined(ANALOG_AZ_OVERLAP_DEGREES)

    #ifdef DEBUG_PROCESSES
      service_process_debug(DEBUG_PROCESSES_PROCESS_ENTER,PROCESS_MISC_ADMIN);
    #endif    

    static byte overlap_led_status = 0;
    static unsigned long last_check_time;
    #ifdef OPTION_BLINK_OVERLAP_LED
    static unsigned long last_overlap_led_transition = 0;
    static byte blink_status = 0;
    #endif //OPTION_BLINK_OVERLAP_LED

    if ((overlap_led) && ((millis() - last_check_time) > 500)) {
      if ((raw_azimuth > ANALOG_AZ_OVERLAP_DEGREES) && (!overlap_led_status)) {
        digitalWriteEnhanced(overlap_led, OVERLAP_LED_ACTIVE_STATE);
        overlap_led_status = 1;
        #ifdef OPTION_BLINK_OVERLAP_LED
        last_overlap_led_transition = millis();
        blink_status = 1;
        #endif //OPTION_BLINK_OVERLAP_LED
        #ifdef DEBUG_OVERLAP
        debug.println("check_overlap: in overlap");
        #endif // DEBUG_OVERLAP
      } else {
        if ((raw_azimuth < ANALOG_AZ_OVERLAP_DEGREES) && (overlap_led_status)) {
          digitalWriteEnhanced(overlap_led, OVERLAP_LED_INACTIVE_STATE);
          overlap_led_status = 0;
          #ifdef DEBUG_OVERLAP
          debug.println("check_overlap: overlap off");
          #endif // DEBUG_OVERLAP
        }
      }
      last_check_time = millis();

    }

    #ifdef OPTION_BLINK_OVERLAP_LED
    if ((overlap_led_status) && ((millis() - last_overlap_led_transition) >= OPTION_OVERLAP_LED_BLINK_MS)){
      if (blink_status){
        digitalWriteEnhanced(overlap_led, OVERLAP_LED_INACTIVE_STATE);
        blink_status = 0;
      } else {
        digitalWriteEnhanced(overlap_led, OVERLAP_LED_ACTIVE_STATE);
        blink_status = 1;
      }
      last_overlap_led_transition = millis();
    }
    #endif //OPTION_BLINK_OVERLAP_LED

    #ifdef DEBUG_PROCESSES
      service_process_debug(DEBUG_PROCESSES_PROCESS_EXIT,PROCESS_MISC_ADMIN);
    #endif    

  #endif //#if defined(ANALOG_AZ_OVERLAP_DEGREES)

} /* check_overlap */



// --------------------------------------------------------------
void clear_command_buffer(){

  control_port_buffer_index = 0;
  control_port_buffer[0] = 0;


}


// --------------------------------------------------------------
void check_serial(){

  #ifdef DEBUG_LOOP
    control_port->println(F("check_serial"));
    control_port->flush();
  #endif // DEBUG_LOOP    

  #ifdef DEBUG_PROCESSES
    service_process_debug(DEBUG_PROCESSES_PROCESS_ENTER,PROCESS_CHECK_SERIAL);
  #endif  

  static unsigned long serial_led_time = 0;
  char return_string[100] = "";
  static byte received_backslash = 0;

  #if defined(FEATURE_GPS)
    static byte gps_port_read = 0;
    static byte gps_port_read_data_sent = 0;
    static byte gps_missing_terminator_flag = 0;
  #endif

  #if !defined(FEATURE_AZ_POSITION_ROTARY_ENCODER) && !defined(FEATURE_AZ_POSITION_PULSE_INPUT) && !defined(FEATURE_AZ_POSITION_ROTARY_ENCODER_USE_PJRC_LIBRARY)
    long place_multiplier = 0;
    byte decimalplace = 0;
  #endif

  #ifdef FEATURE_CLOCK
    int temp_year = 0;
    byte temp_month = 0;
    byte temp_day = 0;
    byte temp_minute = 0;
    byte temp_hour = 0;
  #endif // FEATURE_CLOCK

  #if defined(FEATURE_MOON_TRACKING) || defined(FEATURE_SUN_TRACKING)
    char grid[10] = "";
    byte hit_error = 0;
  #endif // defined(FEATURE_MOON_TRACKING) || defined(FEATURE_SUN_TRACKING)

  #if defined(FEATURE_ELEVATION_CONTROL) && (defined(FEATURE_EL_POSITION_ROTARY_ENCODER) || defined(FEATURE_EL_POSITION_PULSE_INPUT) || defined(FEATURE_EL_POSITION_ROTARY_ENCODER_USE_PJRC_LIBRARY))
    float new_elevation = 9999;
  #endif

  #if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(CONTROL_PROTOCOL_EMULATION) || defined(UNDER_DEVELOPMENT_REMOTE_UNIT_COMMANDS)

  if ((serial_led) && (serial_led_time != 0) && ((millis() - serial_led_time) > SERIAL_LED_TIME_MS)) {
    digitalWriteEnhanced(serial_led, LOW);
    serial_led_time = 0;
  }

  if (control_port->available()) {
    if (serial_led) {
      digitalWriteEnhanced(serial_led, HIGH);                      // blink the LED just to say we got something
      serial_led_time = millis();
    }

    #ifdef FEATURE_POWER_SWITCH
      last_activity_time = millis();
    #endif //FEATURE_POWER_SWITCH

    #ifdef DEBUG_SERIAL
      int control_port_available = control_port->available();
    #endif // DEBUG_SERIAL

    incoming_serial_byte = control_port->read();
    last_serial_receive_time = millis();

    #ifdef DEBUG_SERIAL
      debug.print("check_serial: control_port: ");
      debug.print(control_port_available);
      debug.print(":");
      debug.print(incoming_serial_byte);
      debug.println("");
    #endif // DEBUG_SERIAL


    if ((incoming_serial_byte > 96) && (incoming_serial_byte < 123)) {  // uppercase it
      incoming_serial_byte = incoming_serial_byte - 32;
    }                                                                                                                    


    #ifdef FEATURE_EASYCOM_EMULATION   //Easycom uses spaces, linefeeds, and carriage returns as command delimiters----------

        // Easycom only

        if ((control_port_buffer[0] == '\\') || (control_port_buffer[0] == '/') || ((control_port_buffer_index == 0) && ((incoming_serial_byte == '\\') || (incoming_serial_byte == '/')))) {
          // if it's a backslash command add it to the buffer if it's not a line feed or carriage return
          if ((incoming_serial_byte != 10) && (incoming_serial_byte != 13)) { 
            control_port_buffer[control_port_buffer_index] = incoming_serial_byte;
            control_port_buffer_index++;
          }
        } else {
          // if it's an easycom command add it to the buffer if it's not a line feed, carriage return, or space
          if ((incoming_serial_byte != 10) && (incoming_serial_byte != 13) && (incoming_serial_byte != 32)) { 
            control_port_buffer[control_port_buffer_index] = incoming_serial_byte;
            control_port_buffer_index++;
          }
        }

        if (incoming_serial_byte == '\\'){
          received_backslash = 1;
        }     

        if (received_backslash){
          control_port->write(incoming_serial_byte);
        }        

        // if it is an Easycom command and we have a space, line feed, or carriage return, process it
        if (((incoming_serial_byte == 10) || (incoming_serial_byte == 13) || (incoming_serial_byte == 32)) && (control_port_buffer[0] != '\\') && (control_port_buffer[0] != '/')){
          #if defined(OPTION_HAMLIB_EASYCOM_AZ_EL_COMMAND_HACK) && defined(FEATURE_ELEVATION_CONTROL)
              if ((control_port_buffer[0]=='A') && (control_port_buffer[1]=='Z') && (control_port_buffer_index == 2)){
                unsigned long start_time_hack = millis();
                if (!control_port->available()){
                  while (((millis() - start_time_hack) < 200) && (!control_port->available())){}  // wait 200 mS for something else to pop up on the serial port
                }
                if (control_port->available()){ // is there also 'EL ' waiting for us in the buffer?
                  start_time_hack = millis();
                  while ( (control_port->available()) && ((millis() - start_time_hack) < 200) ) {
                    control_port->read();
                  }
                  control_port_buffer[0] = 'Z';
                  process_easycom_command(control_port_buffer,1,CONTROL_PORT0,return_string);
                  #if defined(FEATURE_LCD_DISPLAY)
                    perform_screen_redraw = 1;
                  #endif
                  //control_port->println(return_string); 
                  control_port->print(return_string);
                  #ifndef OPTION_HAMLIB_EASYCOM_NO_TERMINATOR_CHARACTER_HACK
                    control_port->write(incoming_serial_byte);
                  #endif //OPTION_HAMLIB_EASYCOM_NO_TERMINATOR_CHARACTER_HACK       
                } else {  // we got just a bare AZ command
                  process_easycom_command(control_port_buffer,control_port_buffer_index,CONTROL_PORT0,return_string);
                  #if defined(FEATURE_LCD_DISPLAY)
                    perform_screen_redraw = 1;
                  #endif                  
                  //control_port->println(return_string); 
                  control_port->print(return_string);
                  #ifndef OPTION_HAMLIB_EASYCOM_NO_TERMINATOR_CHARACTER_HACK
                    control_port->write(incoming_serial_byte);
                  #endif //OPTION_HAMLIB_EASYCOM_NO_TERMINATOR_CHARACTER_HACK 
                }
              } else {

                if (control_port_buffer_index > 1){
                  process_easycom_command(control_port_buffer,control_port_buffer_index,CONTROL_PORT0,return_string);
                  #if defined(FEATURE_LCD_DISPLAY)
                    perform_screen_redraw = 1;
                  #endif                  
                  //control_port->println(return_string);
                  control_port->print(return_string);
                  #ifndef OPTION_HAMLIB_EASYCOM_NO_TERMINATOR_CHARACTER_HACK
                    control_port->write(incoming_serial_byte);
                  #endif //OPTION_HAMLIB_EASYCOM_NO_TERMINATOR_CHARACTER_HACK 
                }

              }
          #else //defined(OPTION_HAMLIB_EASYCOM_AZ_EL_COMMAND_HACK) && defined(FEATURE_ELEVATION_CONTROL)
            if (control_port_buffer_index > 1){
              process_easycom_command(control_port_buffer,control_port_buffer_index,CONTROL_PORT0,return_string);
              #if defined(FEATURE_LCD_DISPLAY)
                perform_screen_redraw = 1;
              #endif              
              //control_port->println(return_string); 
              control_port->print(return_string);
              #ifndef OPTION_HAMLIB_EASYCOM_NO_TERMINATOR_CHARACTER_HACK
                control_port->write(incoming_serial_byte);
              #endif //OPTION_HAMLIB_EASYCOM_NO_TERMINATOR_CHARACTER_HACK 
            }
          #endif //defined(OPTION_HAMLIB_EASYCOM_AZ_EL_COMMAND_HACK) && defined(FEATURE_ELEVATION_CONTROL)
          clear_command_buffer();
        } else {
          // if it is a backslash command, process it if we have a carriage return
          if ((incoming_serial_byte == 13) && ((control_port_buffer[0] == '\\') || (control_port_buffer[0] == '/'))){
            received_backslash = 0;
            control_port->println();
            process_backslash_command(control_port_buffer, control_port_buffer_index, CONTROL_PORT0, INCLUDE_RESPONSE_CODE, return_string, SOURCE_CONTROL_PORT);
            #if defined(FEATURE_LCD_DISPLAY)
              perform_screen_redraw = 1;
            #endif          
            control_port->println(return_string);
            clear_command_buffer();
          }
        }


    #endif //FEATURE_EASYCOM_EMULATION ------------------------------------------------------

    #if defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_REMOTE_UNIT_SLAVE)  

        if ((incoming_serial_byte != 10) && (incoming_serial_byte != 13)) { // add it to the buffer if it's not a line feed or carriage return
          control_port_buffer[control_port_buffer_index] = incoming_serial_byte;
          control_port_buffer_index++;
      
        }

        if (incoming_serial_byte == '\\'){
          received_backslash = 1;
        }     

        if (received_backslash){
          control_port->write(incoming_serial_byte);
        }

        if (incoming_serial_byte == 13) {  // do we have a carriage return?
          if ((control_port_buffer[0] == '\\') || (control_port_buffer[0] == '/')) {
            received_backslash = 0;
            control_port->println();
            process_backslash_command(control_port_buffer, control_port_buffer_index, CONTROL_PORT0, INCLUDE_RESPONSE_CODE, return_string, SOURCE_CONTROL_PORT);
            #if defined(FEATURE_LCD_DISPLAY)
              perform_screen_redraw = 1;
            #endif            
          } else {
            #ifdef FEATURE_YAESU_EMULATION
              process_yaesu_command(control_port_buffer,control_port_buffer_index,CONTROL_PORT0,return_string);
              #if defined(FEATURE_LCD_DISPLAY)
                perform_screen_redraw = 1;
              #endif              
            #endif //FEATURE_YAESU_EMULATION

            #ifdef FEATURE_REMOTE_UNIT_SLAVE
              process_remote_slave_command(control_port_buffer,control_port_buffer_index,CONTROL_PORT0,return_string);
            #endif //FEATURE_REMOTE_UNIT_SLAVE
          }  
          control_port->println(return_string);
          clear_command_buffer();
        }

    #endif //defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_REMOTE_UNIT_SLAVE)

    #if defined(FEATURE_DCU_1_EMULATION)  
      if (incoming_serial_byte == ';'){  // either a ';' stop rotation command, or as a command terminator
        if (control_port_buffer_index == 0){  // we received just a ; (stop rotation)
          control_port_buffer[control_port_buffer_index] = incoming_serial_byte;
          control_port_buffer_index++;
        }
        control_port->write(incoming_serial_byte);  // process it, whether it's a stop rotation command or a command terminator
        process_dcu_1_command(control_port_buffer,control_port_buffer_index,CONTROL_PORT0,DCU_1_SEMICOLON,return_string);
        control_port->println(return_string);
        clear_command_buffer();                  
      }

      if ((incoming_serial_byte != 10) && (incoming_serial_byte != 13) && (incoming_serial_byte != ';')) { // add it to the buffer if it's not a line feed or carriage return
        control_port_buffer[control_port_buffer_index] = toupper(incoming_serial_byte);
        control_port_buffer_index++;
        control_port->write(incoming_serial_byte);
      }

      if (incoming_serial_byte == '\\'){
        received_backslash = 1;
      }     

      if (received_backslash){
        control_port->write(incoming_serial_byte);
      }

      if (incoming_serial_byte == 13) {  // do we have a carriage return command termination?
        if ((control_port_buffer[0] == '\\') || (control_port_buffer[0] == '/')) {   // we have a backslash command
          received_backslash = 0;
          control_port->println();
          process_backslash_command(control_port_buffer, control_port_buffer_index, CONTROL_PORT0, INCLUDE_RESPONSE_CODE, return_string, SOURCE_CONTROL_PORT);
          control_port->println(return_string);
          clear_command_buffer();
        } else {       // we have a DCU-1 command
          process_dcu_1_command(control_port_buffer,control_port_buffer_index,CONTROL_PORT0,DCU_1_CARRIAGE_RETURN,return_string);
          control_port->println(return_string);
          clear_command_buffer();
        }
      }

    #endif //defined(FEATURE_DCU_1_EMULATION) 

  } // if (control_port->available())
  #endif // defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)


  // remote unit port servicing
  #if defined(FEATURE_MASTER_WITH_SERIAL_SLAVE)
      if (REMOTE_PORT.available()) {
        incoming_serial_byte = REMOTE_PORT.read();

        #if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
            // if (serial_read_event_flag[1]) {
            //   control_port->print("EVS1");
            //   control_port->write(incoming_serial_byte);
            //   control_port->println();
            // }
            if (remote_port_rx_sniff) {
              control_port->write(incoming_serial_byte);
              control_port->flush();
            }
        #endif //defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)

        if ((incoming_serial_byte != 10) && (remote_unit_port_buffer_index < COMMAND_BUFFER_SIZE)) {
          // incoming_serial_byte = toupper(incoming_serial_byte);
          remote_unit_port_buffer[remote_unit_port_buffer_index] = incoming_serial_byte;
          remote_unit_port_buffer_index++;
          if ((incoming_serial_byte == 13) || (remote_unit_port_buffer_index == COMMAND_BUFFER_SIZE)) {
            remote_unit_port_buffer_carriage_return_flag = 1;
          }
        }
        serial1_last_receive_time = millis();

      }
  #endif // defined(FEATURE_MASTER_WITH_SERIAL_SLAVE)




  #ifdef DEBUG_PROCESSES
    service_process_debug(DEBUG_PROCESSES_PROCESS_EXIT,PROCESS_CHECK_SERIAL);
  #endif  

} /* check_serial */


// --------------------------------------------------------------
void check_buttons(){


  #ifdef DEBUG_PROCESSES
    service_process_debug(DEBUG_PROCESSES_PROCESS_ENTER,PROCESS_CHECK_BUTTONS);
  #endif  

  #ifdef FEATURE_ADAFRUIT_BUTTONS
    int buttons = 0;
    // buttons = lcd.readButtons();
    buttons = k3ngdisplay.readButtons();

    if (buttons & BUTTON_RIGHT) {
  #else
    if (button_cw && (digitalReadEnhanced(button_cw) == BUTTON_ACTIVE_STATE)) {
  #endif // FEATURE_ADAFRUIT_BUTTONS

    if (azimuth_button_was_pushed == 0) {
    #ifdef DEBUG_BUTTONS
      debug.println("check_buttons: button_cw pushed");
    #endif // DEBUG_BUTTONS
    #ifdef OPTION_AZ_MANUAL_ROTATE_LIMITS
    if (raw_azimuth < (AZ_MANUAL_ROTATE_CW_LIMIT)) {
      #endif
      submit_request(AZ, REQUEST_CW, 0, DBG_CHECK_BUTTONS_BTN_CW);
      #if defined(FEATURE_LCD_DISPLAY)
        perform_screen_redraw = 1;
      #endif
      azimuth_button_was_pushed = 1;
      #ifdef OPTION_AZ_MANUAL_ROTATE_LIMITS
    } else {
      #ifdef DEBUG_BUTTONS
        debug.println("check_buttons: exceeded AZ_MANUAL_ROTATE_CW_LIMIT");
      #endif // DEBUG_BUTTONS
    }
      #endif
    }

  } else {
    #ifdef FEATURE_ADAFRUIT_BUTTONS
    if (buttons & BUTTON_LEFT) {
    #else
    if (button_ccw && (digitalReadEnhanced(button_ccw) == BUTTON_ACTIVE_STATE)) {
    #endif // FEATURE_ADAFRUIT_BUTTONS
    if (azimuth_button_was_pushed == 0) {
    #ifdef DEBUG_BUTTONS
    debug.println("check_buttons: button_ccw pushed");
    #endif // DEBUG_BUTTONS  
    #ifdef OPTION_AZ_MANUAL_ROTATE_LIMITS
        if (raw_azimuth > (AZ_MANUAL_ROTATE_CCW_LIMIT)) {
      #endif
        submit_request(AZ, REQUEST_CCW, 0, DBG_CHECK_BUTTONS_BTN_CCW);
        #if defined(FEATURE_LCD_DISPLAY)
          perform_screen_redraw = 1;
        #endif        
        azimuth_button_was_pushed = 1;
      #ifdef OPTION_AZ_MANUAL_ROTATE_LIMITS
      } else {
        #ifdef DEBUG_BUTTONS
        debug.println("check_buttons: exceeded AZ_MANUAL_ROTATE_CCW_LIMIT");
        #endif // DEBUG_BUTTONS
      }
      #endif // OPTION_AZ_MANUAL_ROTATE_LIMITS
      }
    }
  }

#ifdef FEATURE_ADAFRUIT_BUTTONS
  if ((azimuth_button_was_pushed) && (!(buttons & 0x12))) {
    #ifdef DEBUG_BUTTONS
      debug.println("check_buttons: no button depressed");
    #endif // DEBUG_BUTTONS
    submit_request(AZ, REQUEST_STOP, 0, DBG_CHECK_BUTTONS_ADAFRUIT_STOP);
    azimuth_button_was_pushed = 0;
  }

#else
  if ((azimuth_button_was_pushed) && (digitalReadEnhanced(button_ccw) == BUTTON_INACTIVE_STATE) && (digitalReadEnhanced(button_cw) == BUTTON_INACTIVE_STATE)) {
    delay(200);
    if ((digitalReadEnhanced(button_ccw) == BUTTON_INACTIVE_STATE) && (digitalReadEnhanced(button_cw) == BUTTON_INACTIVE_STATE)) {
    #ifdef DEBUG_BUTTONS
      debug.println("check_buttons: no AZ button depressed");
    #endif // DEBUG_BUTTONS
    #ifndef OPTION_BUTTON_RELEASE_NO_SLOWDOWN
    submit_request(AZ, REQUEST_STOP, 0, DBG_CHECK_BUTTONS_RELEASE_NO_SLOWDOWN);
    #if defined(FEATURE_LCD_DISPLAY)
      perform_screen_redraw = 1;
    #endif    
    #else
    submit_request(AZ, REQUEST_KILL, 0, DBG_CHECK_BUTTONS_RELEASE_KILL);
    #endif // OPTION_BUTTON_RELEASE_NO_SLOWDOWN
    azimuth_button_was_pushed = 0;
    }
  }
#endif // FEATURE_ADAFRUIT_BUTTONS

#ifdef FEATURE_ELEVATION_CONTROL
  #ifdef FEATURE_ADAFRUIT_BUTTONS
  if (buttons & 0x08) {
  #else
  if (button_up && (digitalReadEnhanced(button_up) == BUTTON_ACTIVE_STATE)) {
  #endif // FEATURE_ADAFRUIT_BUTTONS
    if (elevation_button_was_pushed == 0) {
      #ifdef OPTION_EL_MANUAL_ROTATE_LIMITS
        if (elevation < (EL_MANUAL_ROTATE_UP_LIMIT)) {
          submit_request(EL, REQUEST_UP, 0, 66);
          #if defined(FEATURE_LCD_DISPLAY)
            perform_screen_redraw = 1;
          #endif          
          elevation_button_was_pushed = 1;
          #ifdef DEBUG_BUTTONS
            debug.println("check_buttons: button_up pushed");
          #endif // DEBUG_BUTTONS  
        } else {
          #ifdef DEBUG_BUTTONS
            debug.println("check_buttons: button_up pushed but at EL_MANUAL_ROTATE_UP_LIMIT");
          #endif // DEBUG_BUTTONS              
        }
      #else 
        submit_request(EL, REQUEST_UP, 0, 66);
        #if defined(FEATURE_LCD_DISPLAY)
          perform_screen_redraw = 1;
        #endif        
        elevation_button_was_pushed = 1;
        #ifdef DEBUG_BUTTONS
          debug.println("check_buttons: button_up pushed");
        #endif // DEBUG_BUTTONS  
      #endif //OPTION_EL_MANUAL_ROTATE_LIMITS  
    }
  } else {
    #ifdef FEATURE_ADAFRUIT_BUTTONS
    if (buttons & 0x04) {
    #else
    if (button_down && (digitalReadEnhanced(button_down) == BUTTON_ACTIVE_STATE)) {
    #endif // FEATURE_ADAFRUIT_BUTTONS
      if (elevation_button_was_pushed == 0) {

        #ifdef OPTION_EL_MANUAL_ROTATE_LIMITS
          if (elevation > (EL_MANUAL_ROTATE_DOWN_LIMIT)) {
            submit_request(EL, REQUEST_DOWN, 0, 67);
            #if defined(FEATURE_LCD_DISPLAY)
              perform_screen_redraw = 1;
            #endif            
            elevation_button_was_pushed = 1;
            #ifdef DEBUG_BUTTONS
              debug.println("check_buttons: button_down pushed");
            #endif // DEBUG_BUTTONS 
          } else {
            #ifdef DEBUG_BUTTONS
              debug.println("check_buttons: button_down pushed but at EL_MANUAL_ROTATE_DOWN_LIMIT");
            #endif // DEBUG_BUTTONS 
          }
        #else
          submit_request(EL, REQUEST_DOWN, 0, 67);
          #if defined(FEATURE_LCD_DISPLAY)
            perform_screen_redraw = 1;
          #endif          
          elevation_button_was_pushed = 1;
          #ifdef DEBUG_BUTTONS
            debug.println("check_buttons: button_down pushed");
          #endif // DEBUG_BUTTONS 
        #endif        
      }
    }
  }

#ifdef FEATURE_ADAFRUIT_BUTTONS
  if ((elevation_button_was_pushed) && (!(buttons & 0x0C))) {
  #ifdef DEBUG_BUTTONS
    debug.println("check_buttons: no EL button depressed");
  #endif // DEBUG_BUTTONS
  #ifndef OPTION_BUTTON_RELEASE_NO_SLOWDOWN
    submit_request(EL, REQUEST_STOP, 0, 68);
  #else
    submit_request(EL, REQUEST_KILL, 0, 69);
  #endif // OPTION_BUTTON_RELEASE_NO_SLOWDOWN
    #if defined(FEATURE_LCD_DISPLAY)
      perform_screen_redraw = 1;
    #endif  
    elevation_button_was_pushed = 0;
  }

#else
  if ((elevation_button_was_pushed) && (digitalReadEnhanced(button_up) == BUTTON_INACTIVE_STATE) && (digitalReadEnhanced(button_down) == BUTTON_INACTIVE_STATE)) {
    delay(200);
    if ((digitalReadEnhanced(button_up) == BUTTON_INACTIVE_STATE) && (digitalReadEnhanced(button_down) == BUTTON_INACTIVE_STATE)) {
    #ifdef DEBUG_BUTTONS
      debug.println("check_buttons: no EL button depressed");
    #endif // DEBUG_BUTTONS
    #ifndef OPTION_BUTTON_RELEASE_NO_SLOWDOWN
      submit_request(EL, REQUEST_STOP, 0, 70);
    #else
      submit_request(EL, REQUEST_KILL, 0, 71);
    #endif // OPTION_BUTTON_RELEASE_NO_SLOWDOWN
      #if defined(FEATURE_LCD_DISPLAY)
        perform_screen_redraw = 1;
      #endif    
      elevation_button_was_pushed = 0;
    }
  }
#endif // FEATURE_ADAFRUIT_BUTTONS

#endif // FEATURE_ELEVATION_CONTROL


#ifdef FEATURE_PARK
  static byte park_button_pushed = 0;
  static unsigned long last_time_park_button_pushed = 0;

  if (button_park) {
    if ((digitalReadEnhanced(button_park) == BUTTON_ACTIVE_STATE)) {
      park_button_pushed = 1;
      last_time_park_button_pushed = millis();
    #ifdef DEBUG_BUTTONS
    debug.println("check_buttons: button_park pushed");
    #endif // DEBUG_BUTTONS   
    } else {
      if ((park_button_pushed) && ((millis() - last_time_park_button_pushed) >= 250)) {
        if (park_status != PARK_INITIATED) {
          #ifdef DEBUG_BUTTONS
          debug.println("check_buttons: executing park");
          #endif // DEBUG_BUTTONS
          #if defined(FEATURE_LCD_DISPLAY)
            perform_screen_redraw = 1;
          #endif          
          initiate_park();
        } else {
          #ifdef DEBUG_BUTTONS
          debug.println("check_buttons: park aborted");
          #endif // DEBUG_BUTTONS
          submit_request(AZ, REQUEST_KILL, 0, 72);
            #ifdef FEATURE_ELEVATION_CONTROL
          submit_request(EL, REQUEST_KILL, 0, 73);
            #endif // FEATURE_ELEVATION_CONTROL
        }
        park_button_pushed = 0;
      }
    }

  }

    #endif /* ifdef FEATURE_PARK */


  if (button_stop) {
    if ((digitalReadEnhanced(button_stop) == BUTTON_ACTIVE_STATE)) {
      #ifdef DEBUG_BUTTONS
      debug.println("check_buttons: button_stop pushed");
      #endif // DEBUG_BUTTONS      
      #ifndef OPTION_BUTTON_RELEASE_NO_SLOWDOWN
      submit_request(AZ, REQUEST_STOP, 0, 74);
      #else
      submit_request(AZ, REQUEST_KILL, 0, 75);
      #endif // OPTION_BUTTON_RELEASE_NO_SLOWDOWN
      #ifdef FEATURE_ELEVATION_CONTROL
      #ifndef OPTION_BUTTON_RELEASE_NO_SLOWDOWN
      submit_request(EL, REQUEST_STOP, 0, 76);
      #else
      submit_request(EL, REQUEST_KILL, 0, 77);
      #endif // OPTION_BUTTON_RELEASE_NO_SLOWDOWN
      #endif // FEATURE_ELEVATION_CONTROL
      #if defined(FEATURE_LCD_DISPLAY)
        perform_screen_redraw = 1;
      #endif    
    }
  }

  #ifdef FEATURE_MOON_TRACKING
    static byte moon_tracking_button_pushed = 0;
    static unsigned long last_time_moon_tracking_button_pushed = 0;
    if (moon_tracking_button) {
      if ((digitalReadEnhanced(moon_tracking_button) == BUTTON_ACTIVE_STATE)) {
        moon_tracking_button_pushed = 1;
        last_time_moon_tracking_button_pushed = millis();
        #ifdef DEBUG_BUTTONS
        debug.println("check_buttons: moon_tracking_button pushed");
        #endif // DEBUG_BUTTONS
      } else {
        if ((moon_tracking_button_pushed) && ((millis() - last_time_moon_tracking_button_pushed) >= 250)) {
          if (!moon_tracking_active) {
            #ifdef DEBUG_BUTTONS
            debug.println("check_buttons: moon tracking on");
            #endif // DEBUG_BUTTONS
            change_tracking(ACTIVATE_MOON_TRACKING);
            #if defined(FEATURE_LCD_DISPLAY)
              perform_screen_redraw = 1;
            #endif                           
          } else {
            #ifdef DEBUG_BUTTONS
            debug.println("check_buttons: moon tracking off");
            #endif // DEBUG_BUTTONS
            change_tracking(DEACTIVATE_MOON_TRACKING);
            stop_rotation();
          }
          moon_tracking_button_pushed = 0;
        }
      }
    }
  #endif // FEATURE_MOON_TRACKING

  #ifdef FEATURE_SUN_TRACKING
  static byte sun_tracking_button_pushed = 0;
  static unsigned long last_time_sun_tracking_button_pushed = 0;
  if (sun_tracking_button) {
    if (digitalReadEnhanced(sun_tracking_button) == BUTTON_ACTIVE_STATE) {
      sun_tracking_button_pushed = 1;
      last_time_sun_tracking_button_pushed = millis();
      #ifdef DEBUG_BUTTONS
      debug.println("check_buttons: sun_tracking_button pushed");
      #endif // DEBUG_BUTTONS
    } else {
      if ((sun_tracking_button_pushed) && ((millis() - last_time_sun_tracking_button_pushed) >= 250)) {
        if (!sun_tracking_active) {
          #ifdef DEBUG_BUTTONS
            debug.println("check_buttons: sun tracking on");
          #endif // DEBUG_BUTTONS
          change_tracking(ACTIVATE_SUN_TRACKING);
          #if defined(FEATURE_LCD_DISPLAY)
            perform_screen_redraw = 1;
          #endif                           
        } else {
          #ifdef DEBUG_BUTTONS
            debug.print("check_buttons: sun tracking off");
          #endif // DEBUG_BUTTONS
          change_tracking(DEACTIVATE_SUN_TRACKING);
          stop_rotation();
        }
        sun_tracking_button_pushed = 0;
      }
    }
  }
  #endif // FEATURE_SUN_TRACKING


  #ifdef FEATURE_SATELLITE_TRACKING
    static byte satellite_tracking_button_pushed = 0;
    static unsigned long last_time_satellite_tracking_button_pushed = 0;
    if (satellite_tracking_button) {
      if (digitalReadEnhanced(satellite_tracking_button) == BUTTON_ACTIVE_STATE) {
        satellite_tracking_button_pushed = 1;
        last_time_satellite_tracking_button_pushed = millis();
        #ifdef DEBUG_BUTTONS
          debug.println("check_buttons: satellite_tracking_button pushed");
        #endif // DEBUG_BUTTONS
      } else {
        if ((satellite_tracking_button_pushed) && ((millis() - last_time_satellite_tracking_button_pushed) >= 250)) {
          if (!satellite_tracking_active) {
            #ifdef DEBUG_BUTTONS
              debug.println("check_buttons: sun tracking on");
            #endif // DEBUG_BUTTONS
            change_tracking(ACTIVATE_SATELLITE_TRACKING);
            #if defined(FEATURE_LCD_DISPLAY)
              perform_screen_redraw = 1;
            #endif                            
          } else {
            #ifdef DEBUG_BUTTONS
              debug.print("check_buttons: sun tracking off");
            #endif // DEBUG_BUTTONS
            change_tracking(DEACTIVATE_SATELLITE_TRACKING);
            stop_rotation();
          }
          satellite_tracking_button_pushed = 0;
        }
      }
    }
  #endif // FEATURE_SATELLITE_TRACKING


  #ifdef DEBUG_PROCESSES
    service_process_debug(DEBUG_PROCESSES_PROCESS_EXIT,PROCESS_CHECK_BUTTONS);
  #endif    

} /* check_buttons */
// --------------------------------------------------------------
#ifdef FEATURE_LCD_DISPLAY
char * idle_status(){

  #ifdef OPTION_DISPLAY_DIRECTION_STATUS
    return azimuth_direction(azimuth);
  #endif //OPTION_DISPLAY_DIRECTION_STATUS

  return((char *)"");

}
#endif //FEATURE_LCD_DISPLAY
// --------------------------------------------------------------

#if defined(FEATURE_LCD_DISPLAY) && defined(OPTION_DISPLAY_DIRECTION_STATUS)  
char * azimuth_direction(int azimuth_in){

  azimuth_in = azimuth_in;

  if (azimuth_in > 348) {
    return N_STRING;
  }
  if (azimuth_in > 326) {
    return NNW_STRING;
  }
  if (azimuth_in > 303) {
    return NW_STRING;
  }
  if (azimuth_in > 281) {
    return WNW_STRING;
  }
  if (azimuth_in > 258) {
    return W_STRING;
  }
  if (azimuth_in > 236) {
    return WSW_STRING;
  }
  if (azimuth_in > 213) {
    return SW_STRING;
  }
  if (azimuth_in > 191) {
    return SSW_STRING;
  }
  if (azimuth_in > 168) {
    return S_STRING;
  }
  if (azimuth_in > 146) {
    return SSE_STRING;
  }
  if (azimuth_in > 123) {
    return SE_STRING;
  }
  if (azimuth_in > 101) {
    return ESE_STRING;
  }
  if (azimuth_in > 78) {
    return E_STRING;
  }
  if (azimuth_in > 56) {
    return ENE_STRING;
  }
  if (azimuth_in > 33) {
    return NE_STRING;
  }
  if (azimuth_in > 11) {
    return NNE_STRING;
  }
  return N_STRING;

} /* azimuth_direction */
#endif /* ifdef FEATURE_LCD_DISPLAY */




// --------------------------------------------------------------


// --------------------------------------------------------------

#if defined(FEATURE_LCD_DISPLAY)
void update_lcd_display(){

  #ifdef DEBUG_LOOP
    control_port->println(F("update_lcd_display()"));
    control_port->flush();
  #endif // DEBUG_LOOP   


  #ifdef DEBUG_PROCESSES
    service_process_debug(DEBUG_PROCESSES_PROCESS_ENTER,PROCESS_UPDATE_LCD_DISPLAY);
  #endif    

  byte force_display_update_now = 0;
  char workstring[32] = "";
  char workstring2[32] = "";
  byte row_override[LCD_ROWS+1];

  for (int x = 0;x < (LCD_ROWS+1);x++){row_override[x] = 0;}

  k3ngdisplay.clear_pending_buffer();

  #ifdef FEATURE_MOON_TRACKING
    static unsigned long last_moon_tracking_check_time = 0;
  #endif 

  #ifdef FEATURE_SUN_TRACKING
    static unsigned long last_sun_tracking_check_time = 0;
  #endif

  // OPTION_DISPLAY_DIRECTION_STATUS - azimuth direction display ***********************************************************************************
  #if defined(OPTION_DISPLAY_DIRECTION_STATUS)   
  strcpy(workstring,azimuth_direction(azimuth));  // TODO - add left/right/center
  k3ngdisplay.print_center_fixed_field_size(workstring,LCD_DIRECTION_ROW-1,LCD_STATUS_FIELD_SIZE);
  #endif //defined(OPTION_DISPLAY_DIRECTION_STATUS)


  // OPTION_DISPLAY_HEADING - show heading ***********************************************************************************
  #if defined(OPTION_DISPLAY_HEADING)
    #if !defined(FEATURE_ELEVATION_CONTROL)                    // ---------------- az only -----------------------------------
      strcpy(workstring,AZIMUTH_STRING);
      switch(configuration.azimuth_display_mode){
        case AZ_DISPLAY_MODE_NORMAL:
        case AZ_DISPLAY_MODE_OVERLAP_PLUS:
          dtostrf(azimuth , 1, DISPLAY_DECIMAL_PLACES, workstring2);
          break;
        case AZ_DISPLAY_MODE_RAW:
          dtostrf(raw_azimuth , 1, DISPLAY_DECIMAL_PLACES, workstring2);
          break;  
      }               
      #ifdef OPTION_LCD_HEADING_FIELD_FIXED_DECIMAL_PLACE
        switch(configuration.azimuth_display_mode){
          case AZ_DISPLAY_MODE_NORMAL:
          case AZ_DISPLAY_MODE_OVERLAP_PLUS:
            if (azimuth < 100){strcat(workstring," ");}
            if (azimuth < 10){strcat(workstring," ");}  
            break;
          case AZ_DISPLAY_MODE_RAW:
            if (raw_azimuth < 100){strcat(workstring," ");}
            if (raw_azimuth < 10){strcat(workstring," ");}  
            break;            
        }
      #endif //OPTION_LCD_HEADING_FIELD_FIXED_DECIMAL_PLACE  
      #if defined(ANALOG_AZ_OVERLAP_DEGREES)
        if ((configuration.azimuth_display_mode == AZ_DISPLAY_MODE_OVERLAP_PLUS) && (raw_azimuth > ANALOG_AZ_OVERLAP_DEGREES)){
          strcat(workstring,"+");
        } 
      #endif
      strcat(workstring,workstring2);
      strcat(workstring,LCD_DISPLAY_DEGREES_STRING);
      k3ngdisplay.print_center_fixed_field_size(workstring,LCD_HEADING_ROW-1,LCD_HEADING_FIELD_SIZE);
    #else                                                       // --------------------az & el---------------------------------
      if ((azimuth >= 1000) && (elevation >= 1000) && (LCD_COLUMNS < 18)) {
        strcpy(workstring,AZ_STRING);
      } else {
        strcpy(workstring,AZ_SPACE_STRING);
      }
      switch(configuration.azimuth_display_mode){
        case AZ_DISPLAY_MODE_NORMAL:
        case AZ_DISPLAY_MODE_OVERLAP_PLUS:
          dtostrf(azimuth , 3, DISPLAY_DECIMAL_PLACES, workstring2);
          break;
        case AZ_DISPLAY_MODE_RAW:
          dtostrf(raw_azimuth , 3, DISPLAY_DECIMAL_PLACES, workstring2);
          break;  
      }  
      #ifdef OPTION_LCD_HEADING_FIELD_FIXED_DECIMAL_PLACE
        switch(configuration.azimuth_display_mode){
          case AZ_DISPLAY_MODE_NORMAL:
          case AZ_DISPLAY_MODE_OVERLAP_PLUS:
            if (azimuth < 100){strcat(workstring," ");}
            if (azimuth < 10){strcat(workstring," ");}  
            break;
          case AZ_DISPLAY_MODE_RAW:
            if (raw_azimuth < 100){strcat(workstring," ");}
            if (raw_azimuth < 10){strcat(workstring," ");}  
            break;            
        }
      #endif //OPTION_LCD_HEADING_FIELD_FIXED_DECIMAL_PLACE
        #if defined(ANALOG_AZ_OVERLAP_DEGREES)
          if ((configuration.azimuth_display_mode == AZ_DISPLAY_MODE_OVERLAP_PLUS) && (raw_azimuth > ANALOG_AZ_OVERLAP_DEGREES)){
            strcat(workstring,"+");
          }      
        #endif   
        strcat(workstring,workstring2);
        if (DISPLAY_DECIMAL_PLACES > 1){
          if (LCD_COLUMNS > 14) {
            strcat(workstring,LCD_DISPLAY_DEGREES_STRING);
          }
        } else {
          if ((LCD_COLUMNS > 18) || ((azimuth < 100) && (elevation < 100))) {
            strcat(workstring,LCD_DISPLAY_DEGREES_STRING);
          }
        }
        if (DISPLAY_DECIMAL_PLACES > 1){
          if ((elevation >= 1000) && (azimuth >= 1000) && (LCD_COLUMNS < 18)) {
            strcat(workstring,SPACE_EL_STRING);
          } else {
            strcat(workstring,SPACE_EL_SPACE_STRING);
          }
        } else {
          strcat(workstring,SPACE_EL_SPACE_STRING);
        }
      dtostrf(elevation , 1, DISPLAY_DECIMAL_PLACES, workstring2);
      #ifdef OPTION_LCD_HEADING_FIELD_FIXED_DECIMAL_PLACE
        if (elevation < 100){strcat(workstring," ");}
        if (elevation < 10){strcat(workstring," ");}    
      #endif //OPTION_LCD_HEADING_FIELD_FIXED_DECIMAL_PLACE  
      strcat(workstring,workstring2);
      if (DISPLAY_DECIMAL_PLACES > 1){
        if (LCD_COLUMNS > 14) {
          strcat(workstring,LCD_DISPLAY_DEGREES_STRING);
        }
      } else {
        if ((LCD_COLUMNS > 18) || ((azimuth < 100) && (elevation < 100))) {
          strcat(workstring,LCD_DISPLAY_DEGREES_STRING);
        }
      }
      k3ngdisplay.print_center_fixed_field_size(workstring,LCD_HEADING_ROW-1,LCD_HEADING_FIELD_SIZE);    
    #endif // FEATURE_ELEVATION_CONTROL
  #endif //defined(OPTION_DISPLAY_HEADING)  

  // OPTION_DISPLAY_HEADING_AZ_ONLY - show heading ***********************************************************************************
  #if defined(OPTION_DISPLAY_HEADING_AZ_ONLY)       
    strcpy(workstring,AZIMUTH_STRING);
      switch(configuration.azimuth_display_mode){
        case AZ_DISPLAY_MODE_NORMAL:
        case AZ_DISPLAY_MODE_OVERLAP_PLUS:
          dtostrf(azimuth , 1, DISPLAY_DECIMAL_PLACES, workstring2);
          break;
        case AZ_DISPLAY_MODE_RAW:
          dtostrf(raw_azimuth , 1, DISPLAY_DECIMAL_PLACES, workstring2);
          break;  
      }  
    #ifdef OPTION_LCD_HEADING_FIELD_FIXED_DECIMAL_PLACE
      switch(configuration.azimuth_display_mode){
        case AZ_DISPLAY_MODE_NORMAL:
        case AZ_DISPLAY_MODE_OVERLAP_PLUS:
          if (azimuth < 100){strcat(workstring," ");}
          if (azimuth < 10){strcat(workstring," ");}  
          break;
        case AZ_DISPLAY_MODE_RAW:
          if (raw_azimuth < 100){strcat(workstring," ");}
          if (raw_azimuth < 10){strcat(workstring," ");}  
          break;            
      } 
    #endif //OPTION_LCD_HEADING_FIELD_FIXED_DECIMAL_PLACE   
    #if defined(ANALOG_AZ_OVERLAP_DEGREES)
      if ((configuration.azimuth_display_mode == AZ_DISPLAY_MODE_OVERLAP_PLUS) && (raw_azimuth > ANALOG_AZ_OVERLAP_DEGREES)){
        strcat(workstring,"+");
      }     
    #endif   
    strcat(workstring,workstring2);
    strcat(workstring,LCD_DISPLAY_DEGREES_STRING);
    k3ngdisplay.print_center_fixed_field_size(workstring,LCD_AZ_ONLY_HEADING_ROW-1,LCD_AZ_ONLY_HEADING_FIELD_SIZE);
  #endif //defined(OPTION_DISPLAY_HEADING_AZ_ONLY)        


  // OPTION_DISPLAY_HEADING_EL_ONLY - show heading ***********************************************************************************
  #if defined(OPTION_DISPLAY_HEADING_EL_ONLY) && defined(FEATURE_ELEVATION_CONTROL)
      strcpy(workstring,ELEVATION_STRING);
      dtostrf(elevation , 1, DISPLAY_DECIMAL_PLACES, workstring2);
    #ifdef OPTION_LCD_HEADING_FIELD_FIXED_DECIMAL_PLACE
      if (elevation < 100){strcat(workstring," ");}
      if (elevation < 10){strcat(workstring," ");}    
    #endif //OPTION_LCD_HEADING_FIELD_FIXED_DECIMAL_PLACE  
    strcat(workstring,workstring2);
    if (DISPLAY_DECIMAL_PLACES > 1){
      if (LCD_COLUMNS > 14) {
        strcat(workstring,LCD_DISPLAY_DEGREES_STRING);
      }
    } else {
      if ((LCD_COLUMNS > 18) || (elevation < 100)) {
        strcat(workstring,LCD_DISPLAY_DEGREES_STRING);
      }
    }
    k3ngdisplay.print_center_fixed_field_size(workstring,LCD_EL_ONLY_HEADING_ROW-1,LCD_EL_ONLY_HEADING_FIELD_SIZE);    
  #endif //defined(OPTION_DISPLAY_HEADING_EL_ONLY)   

  // OPTION_DISPLAY_STATUS***********************************************************************************
  #if defined(OPTION_DISPLAY_STATUS)
    #if !defined(FEATURE_ELEVATION_CONTROL) // ---------------- az only ----------------------------------------------
      if (az_state != IDLE) {
        if (az_request_queue_state == IN_PROGRESS_TO_TARGET) { 
          if (current_az_state() == ROTATING_CW) {
            strcpy(workstring,CW_STRING);
          } else {
            strcpy(workstring,CCW_STRING);
          }
          strcat(workstring," ");
          switch(configuration.azimuth_display_mode){
            case AZ_DISPLAY_MODE_NORMAL:
            case AZ_DISPLAY_MODE_OVERLAP_PLUS:
              dtostrf(target_azimuth, 1, DISPLAY_DECIMAL_PLACES, workstring2);
              break;
            case AZ_DISPLAY_MODE_RAW:
              dtostrf(target_raw_azimuth, 1, DISPLAY_DECIMAL_PLACES, workstring2);
              break;              
          }
          #if defined(ANALOG_AZ_OVERLAP_DEGREES)
            if ((configuration.azimuth_display_mode == AZ_DISPLAY_MODE_OVERLAP_PLUS) && (raw_azimuth > ANALOG_AZ_OVERLAP_DEGREES)){
              strcat(workstring,"+");
            } 
          #endif          
          strcat(workstring,workstring2);
          strcat(workstring,LCD_DISPLAY_DEGREES_STRING);
        } else {
          if (current_az_state() == ROTATING_CW) {
            strcpy(workstring,CW_STRING);
          } else {
            strcpy(workstring,CCW_STRING);
          }
        }
        k3ngdisplay.print_center_fixed_field_size(workstring,LCD_STATUS_ROW-1,LCD_STATUS_FIELD_SIZE);
        row_override[LCD_STATUS_ROW] = 1;
      }

      #if defined(FEATURE_PARK)
        static byte last_park_status = NOT_PARKED;
        static unsigned long last_park_message_update_time = 0;
        static byte park_message_in_effect = 0;
        if (park_status != last_park_status){
          switch(park_status){
            case PARKED: 
              k3ngdisplay.print_center_fixed_field_size((char *)PARKED_STRING,LCD_STATUS_ROW-1,LCD_STATUS_FIELD_SIZE);
              row_override[LCD_STATUS_ROW] = 1;
              park_message_in_effect = 1;
              break;              
            case PARK_INITIATED:
              k3ngdisplay.print_center_fixed_field_size((char *)PARKING_STRING,LCD_STATUS_ROW-1,LCD_STATUS_FIELD_SIZE);
              row_override[LCD_STATUS_ROW] = 1;
              park_message_in_effect = 1;
              break;
            case NOT_PARKED: 
              park_message_in_effect = 0;
              break;
          }
          last_park_status = park_status;
          last_park_message_update_time = millis();
        }
       
        if (park_message_in_effect){
          if ((millis() - last_park_message_update_time) > PARKING_STATUS_DISPLAY_TIME_MS){
            park_message_in_effect = 0;
          } else {
            row_override[LCD_STATUS_ROW] = 1;
            switch(park_status){
              case PARKED: 
                k3ngdisplay.print_center_fixed_field_size((char *)PARKED_STRING,LCD_STATUS_ROW-1,LCD_STATUS_FIELD_SIZE);                
                break;              
              case PARK_INITIATED:
                k3ngdisplay.print_center_fixed_field_size((char *)PARKING_STRING,LCD_STATUS_ROW-1,LCD_STATUS_FIELD_SIZE);
                break;
            }
          }
        }
      #endif // FEATURE_PARK

      #ifdef FEATURE_AZ_PRESET_ENCODER 
        float target = 0; 
        if (preset_encoders_state == ENCODER_AZ_PENDING) {
          target = az_encoder_raw_degrees;
          if (target > 359) {
            target = target - 360;
          }
          if (target > 359) {
            target = target - 360;
          }
          strcpy(workstring,TARGET_STRING);
          dtostrf(target, 1, DISPLAY_DECIMAL_PLACES, workstring2);
          strcat(workstring,workstring2);
          strcat(workstring,LCD_DISPLAY_DEGREES_STRING);
          k3ngdisplay.print_center_fixed_field_size(workstring,LCD_STATUS_ROW-1,LCD_STATUS_FIELD_SIZE);
          row_override[LCD_STATUS_ROW] = 1;
        }
      #endif //FEATURE_AZ_PRESET_ENCODER

    #else                          // az & el ----------------------------------------------------------------------------
      strcpy(workstring,"");
      if (az_state != IDLE) {
        if (az_request_queue_state == IN_PROGRESS_TO_TARGET) { 
          if (current_az_state() == ROTATING_CW) {
            strcat(workstring,CW_STRING);
          } else {
            strcat(workstring,CCW_STRING);
          }
          strcat(workstring," ");
          switch(configuration.azimuth_display_mode){
            case AZ_DISPLAY_MODE_NORMAL:
            case AZ_DISPLAY_MODE_OVERLAP_PLUS:
              dtostrf(target_azimuth, 1, DISPLAY_DECIMAL_PLACES, workstring2);
              break;
            case AZ_DISPLAY_MODE_RAW:
              dtostrf(target_raw_azimuth, 1, DISPLAY_DECIMAL_PLACES, workstring2);
              break;              
          }
          #if defined(ANALOG_AZ_OVERLAP_DEGREES)
            if ((configuration.azimuth_display_mode == AZ_DISPLAY_MODE_OVERLAP_PLUS) && (raw_azimuth > ANALOG_AZ_OVERLAP_DEGREES)){
              strcat(workstring,"+");
            }  
          #endif  
          strcat(workstring,workstring2);
          strcat(workstring,LCD_DISPLAY_DEGREES_STRING);
          row_override[LCD_STATUS_ROW] = 1;
        } else {
          if (current_az_state() == ROTATING_CW) {
            strcpy(workstring,CW_STRING);
          } else {
            strcpy(workstring,CCW_STRING);
          }
        }        
      }
      if (el_state != IDLE) {
        if (az_state != IDLE){
          strcat(workstring," ");
        }
        if (el_request_queue_state == IN_PROGRESS_TO_TARGET) { 
          if (current_el_state() == ROTATING_UP) {
            strcat(workstring,UP_STRING);
          } else {
            strcat(workstring,DOWN_STRING);
          }
          strcat(workstring," ");
          dtostrf(target_elevation, 1, DISPLAY_DECIMAL_PLACES, workstring2);
          strcat(workstring,workstring2);
          strcat(workstring,LCD_DISPLAY_DEGREES_STRING);
          row_override[LCD_STATUS_ROW] = 1;
        } else {
          if (current_el_state() == ROTATING_UP) {
            strcat(workstring,UP_STRING);
          } else {
            strcat(workstring,DOWN_STRING);
          }
        }        
      }

      if ((az_state != IDLE) || (el_state != IDLE)){ 
        k3ngdisplay.print_center_fixed_field_size(workstring,LCD_STATUS_ROW-1,LCD_STATUS_FIELD_SIZE);
      } //<added

      #if defined(FEATURE_PARK)
        static byte last_park_status = NOT_PARKED;
        static unsigned long last_park_message_update_time = 0;
        static byte park_message_in_effect = 0;
        if (park_status != last_park_status){
          switch(park_status){
            case PARKED: 
              k3ngdisplay.print_center_fixed_field_size((char *)PARKED_STRING,LCD_STATUS_ROW-1,LCD_STATUS_FIELD_SIZE);
              row_override[LCD_STATUS_ROW] = 1;
              park_message_in_effect = 1;
              break;              
            case PARK_INITIATED:
              k3ngdisplay.print_center_fixed_field_size((char *)PARKING_STRING,LCD_STATUS_ROW-1,LCD_STATUS_FIELD_SIZE);
              row_override[LCD_STATUS_ROW] = 1;
              park_message_in_effect = 1;
              break;
            case NOT_PARKED: 
              park_message_in_effect = 0;
              break;
          }
          last_park_status = park_status;
          last_park_message_update_time = millis();
        }
       
        if (park_message_in_effect){
          if ((millis() - last_park_message_update_time) > PARKING_STATUS_DISPLAY_TIME_MS){
            park_message_in_effect = 0;
          } else {
            row_override[LCD_STATUS_ROW] = 1;
            switch(park_status){
              case PARKED: 
                k3ngdisplay.print_center_fixed_field_size((char *)PARKED_STRING,LCD_STATUS_ROW-1,LCD_STATUS_FIELD_SIZE);                
                break;              
              case PARK_INITIATED:
                k3ngdisplay.print_center_fixed_field_size((char *)PARKING_STRING,LCD_STATUS_ROW-1,LCD_STATUS_FIELD_SIZE);
                break;
            }
          }
        }
      #endif // FEATURE_PARK

      #if defined(FEATURE_AZ_PRESET_ENCODER) && !defined(FEATURE_EL_PRESET_ENCODER)
        float target = 0; 
        if (preset_encoders_state == ENCODER_AZ_PENDING) {
            target = az_encoder_raw_degrees;
          if (target > 359) {
            target = target - 360;
          }
          if (target > 359) {
            target = target - 360;
          }
          strcpy(workstring,TARGET_STRING);
          dtostrf(target , 1, DISPLAY_DECIMAL_PLACES, workstring2);
          strcat(workstring,workstring2);
          strcat(workstring,LCD_DISPLAY_DEGREES_STRING);
          k3ngdisplay.print_center_fixed_field_size(workstring,LCD_STATUS_ROW-1,LCD_STATUS_FIELD_SIZE);
          row_override[LCD_STATUS_ROW] = 1;
        }
      #endif //defined(FEATURE_AZ_PRESET_ENCODER) && !defined(FEATURE_EL_PRESET_ENCODER) 

      #if defined(FEATURE_AZ_PRESET_ENCODER) && defined(FEATURE_EL_PRESET_ENCODER)  
        float target = az_encoder_raw_degrees;
        if (target > 359) {
          target = target - 360;
        }
        if (target > 359) {
          target = target - 360;
        }

        if (preset_encoders_state != ENCODER_IDLE) {
          switch (preset_encoders_state) {
            case ENCODER_AZ_PENDING:
              strcpy(workstring,AZ_TARGET_STRING);
              dtostrf(target , 1, DISPLAY_DECIMAL_PLACES, workstring2);
              strcat(workstring,workstring2);
              strcat(workstring,LCD_DISPLAY_DEGREES_STRING);
              k3ngdisplay.print_center_fixed_field_size(workstring,LCD_STATUS_ROW-1,LCD_STATUS_FIELD_SIZE);
              row_override[LCD_STATUS_ROW] = 1;
              break;
            case ENCODER_EL_PENDING:
              strcpy(workstring,EL_TARGET_STRING);
              dtostrf(el_encoder_degrees , 1, DISPLAY_DECIMAL_PLACES, workstring2);
              strcat(workstring,workstring2);
              strcat(workstring,LCD_DISPLAY_DEGREES_STRING);
              k3ngdisplay.print_center_fixed_field_size(workstring,LCD_STATUS_ROW-1,LCD_STATUS_FIELD_SIZE);
              row_override[LCD_STATUS_ROW] = 1;
              break;
            case ENCODER_AZ_EL_PENDING:
              strcpy(workstring,TARGET_STRING);
              dtostrf(target , 1, DISPLAY_DECIMAL_PLACES, workstring2);
              strcat(workstring,workstring2);
              strcat(workstring,LCD_DISPLAY_DEGREES_STRING);
              strcat(workstring," ");
              dtostrf(el_encoder_degrees , 1, DISPLAY_DECIMAL_PLACES, workstring2);
              strcat(workstring,workstring2);
              strcat(workstring,LCD_DISPLAY_DEGREES_STRING);              
              k3ngdisplay.print_center_fixed_field_size(workstring,LCD_STATUS_ROW-1,LCD_STATUS_FIELD_SIZE);
              row_override[LCD_STATUS_ROW] = 1;
              break;
          } // switch 
        } //if (preset_encoders_state != ENCODER_IDLE)
      #endif  //defined(FEATURE_AZ_PRESET_ENCODER) && !defined(FEATURE_EL_PRESET_ENCODER)

    #endif //!defined(FEATURE_ELEVATION_CONTROL)

  #endif //defined(OPTION_DISPLAY_STATUS)




// TODO: develop status row with HH:MM time, rotation status, direction, and GPS status?

// TODO: FEATURE_PARK {done, need to test}, FEATURE_AZ_PRESET_ENCODER and FEATURE_EL_PRESET_ENCODER in status widget {done, need to test}
  

  static unsigned long last_full_screen_redraw = 0;

  if ((((millis() - last_full_screen_redraw) > (long(LCD_PERIODIC_REDRAW_TIME_SECS)*1000L)) & (LCD_PERIODIC_REDRAW_TIME_SECS > 0)) || (perform_screen_redraw && LCD_REDRAW_UPON_COMMANDS)){
    if (LCD_CLEAR_BEFORE_REDRAW){
      k3ngdisplay.clear();
    }
    k3ngdisplay.redraw();
    last_full_screen_redraw = millis();
    perform_screen_redraw = 0;
  } else {

    // do it ! ************************************
    k3ngdisplay.service(force_display_update_now);


  }

  #ifdef DEBUG_PROCESSES
    service_process_debug(DEBUG_PROCESSES_PROCESS_EXIT,PROCESS_UPDATE_LCD_DISPLAY);
  #endif      

}  
#endif // defined(FEATURE_LCD_DISPLAY) 




// --------------------------------------------------------------
#if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
void get_keystroke(){
  while (control_port->available() == 0) {
  }
  while (control_port->available() > 0)
    incoming_serial_byte = control_port->read();
}
#endif // defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)

// --------------------------------------------------------------
#ifdef FEATURE_YAESU_EMULATION
void print_wrote_to_memory(){

  control_port->println(F("Wrote to memory"));

}

#endif // FEATURE_YAESU_EMULATION
// --------------------------------------------------------------
#ifdef FEATURE_YAESU_EMULATION
void clear_serial_buffer(){

  delay(200);
  while (control_port->available()) incoming_serial_byte = control_port->read();
}

#endif // FEATURE_YAESU_EMULATION
// --------------------------------------------------------------
byte calculated_configuration_struct_subversion(){

  byte subversion = 0;

  #if defined(FEATURE_CALIBRATION)
    subversion = subversion | 1;
  #endif

  #if defined(FEATURE_AUDIBLE_ALERT)
    subversion = subversion | 8;
  #endif

  return subversion;

}

// --------------------------------------------------------------
void read_settings_from_eeprom(){

  byte * p = (byte *)(void *)&configuration;
  unsigned int i;
  int ee = 0;

  EEPROM.begin(sizeof(configuration) + 16); // ESP32's flash-emulated EEPROM requires begin() before first use

  for (i = 0; i < sizeof(configuration); i++) {
    *p++ = EEPROM.read(ee++);
  }
  
  #if defined(FEATURE_CALIBRATION)
  if ((configuration.configuration_struct_version == CONFIGURATION_STRUCT_VERSION) && (configuration.configuration_struct_subversion == calculated_configuration_struct_subversion()) && (configuration.calibration_points == CALIBRATION_POINTS)){
  #else
  if ((configuration.configuration_struct_version == CONFIGURATION_STRUCT_VERSION)){
  #endif
    
    #ifdef DEBUG_EEPROM
      if (debug_mode) {
        debug.println("read_settings_from_eeprom: reading settings from eeprom: ");
        debug.print("\nconfiguration_struct_version"):
        debug.print(configuration.configuration_struct_version);
        debug.print("\nconfiguration_struct_subversion"):
        debug.print(configuration.configuration_struct_subversion);        
        debug.print("\nanalog_az_full_ccw");
        debug.print(configuration.analog_az_full_ccw);
        debug.print("\nanalog_az_full_cw");
        debug.print(configuration.analog_az_full_cw);
        debug.print("\nanalog_el_0_degrees");
        debug.print(configuration.analog_el_0_degrees);
        debug.print("\nanalog_el_max_elevation");
        debug.print(configuration.analog_el_max_elevation);
        debug.print("\nlast_azimuth:");
        debug.print(configuration.last_azimuth, 1);
        debug.print("\nlast_elevation:");
        debug.print(configuration.last_elevation, 1);
        debug.print("\nlast_az_incremental_encoder_position:");
        debug.print(configuration.last_az_incremental_encoder_position);
        debug.print("\nlast_el_incremental_encoder_position:");
        debug.print(configuration.last_el_incremental_encoder_position);
        #if !defined(FEATURE_CALIBRATION)
          debug.print("\naz_offset:");
          debug.print(configuration.azimuth_offset,2);
          debug.print("\nel_offset:");
          debug.print(configuration.elevation_offset,2);
        #endif
        debug.print("az starting point:");
        debug.print(configuration.azimuth_starting_point);
        debug.print("az rotation capability:");
        debug.print(configuration.azimuth_rotation_capability);
        debug.print("autopark_active:");
        debug.print(configuration.autopark_active);
        debug.print("autopark_time_minutes:");
        debug.print(configuration.autopark_time_minutes);
        debug.print("azimuth_display_mode:");
        debug.print(configuration.azimuth_display_mode);
        debug.println("");
      }
    #endif // DEBUG_EEPROM

    //azimuth_starting_point = configuration.azimuth_starting_point;
    //azimuth_rotation_capability = configuration.azimuth_rotation_capability;

    #if defined(FEATURE_AZ_POSITION_INCREMENTAL_ENCODER)
      az_incremental_encoder_position = configuration.last_az_incremental_encoder_position;
    #endif

    #if defined(FEATURE_ELEVATION_CONTROL) && defined(FEATURE_EL_POSITION_INCREMENTAL_ENCODER)
      el_incremental_encoder_position = configuration.last_el_incremental_encoder_position;
    #endif


    #if defined(FEATURE_AZ_POSITION_ROTARY_ENCODER) || defined(FEATURE_AZ_POSITION_ROTARY_ENCODER_USE_PJRC_LIBRARY)
      raw_azimuth = configuration.last_azimuth;
      if (raw_azimuth >= 360) {
        azimuth = raw_azimuth - 360.0;
      } else {
        azimuth = raw_azimuth;
      }
    #endif

    #if defined(FEATURE_ELEVATION_CONTROL) && (defined(FEATURE_EL_POSITION_ROTARY_ENCODER) || defined(FEATURE_EL_POSITION_ROTARY_ENCODER_USE_PJRC_LIBRARY))
      elevation = configuration.last_elevation;
    #endif



    #ifdef FEATURE_AZ_POSITION_PULSE_INPUT
      raw_azimuth = configuration.last_azimuth;
      if (raw_azimuth >= 360) {
        azimuth = raw_azimuth - 360.0;
      } else {
        azimuth = raw_azimuth;
      }
      az_position_pulse_input_azimuth = configuration.last_azimuth;
    #endif // FEATURE_AZ_POSITION_PULSE_INPUT

    #if defined(FEATURE_ELEVATION_CONTROL) && defined(FEATURE_EL_POSITION_PULSE_INPUT)
      elevation = configuration.last_elevation;
      el_position_pulse_input_elevation = configuration.last_elevation;
    #endif // FEATURE_EL_POSITION_PULSE_INPUT

    #if defined(FEATURE_AZ_POSITION_PULSE_INPUT) || defined(FEATURE_AZ_POSITION_ROTARY_ENCODER) || defined(FEATURE_AZ_POSITION_ROTARY_ENCODER_USE_PJRC_LIBRARY)
      configuration.azimuth_offset = 0;
    #endif

    #if defined(FEATURE_EL_POSITION_PULSE_INPUT) || defined(FEATURE_EL_POSITION_ROTARY_ENCODER) || defined(FEATURE_EL_POSITION_ROTARY_ENCODER_USE_PJRC_LIBRARY)
      configuration.elevation_offset = 0;
    #endif



  } else {  // initialize eeprom with default values
    #ifdef DEBUG_EEPROM
      debug.println("read_settings_from_eeprom: uninitialized eeprom, calling initialize_eeprom_with_defaults()");
    #endif // DEBUG_EEPROM
    initialize_eeprom_with_defaults();
  }





} /* read_settings_from_eeprom */

// --------------------------------------------------------------
#if defined(FEATURE_CALIBRATION)
  void initialize_calibration_tables(){


    for (byte x = 0;x < CALIBRATION_POINTS;x++){  // initialize the calibration array
      configuration.calibration_az_from[x] = 0;
      configuration.calibration_az_to[x] = 0;   
      configuration.calibration_az_flag[x] = 0;
      configuration.calibration_el_from[x] = 0;
      configuration.calibration_el_to[x] = 0;   
      configuration.calibration_el_flag[x] = 0;  
    }

    configuration.calibration_az_from[0] = AZIMUTH_STARTING_POINT_EEPROM_INITIALIZE;
    configuration.calibration_az_to[0] = AZIMUTH_STARTING_POINT_EEPROM_INITIALIZE;
    configuration.calibration_az_flag[0] = 1;  // 1 = default endpoint  

    configuration.calibration_az_from[1] = AZIMUTH_STARTING_POINT_EEPROM_INITIALIZE + AZIMUTH_ROTATION_CAPABILITY_EEPROM_INITIALIZE;
    configuration.calibration_az_to[1] = AZIMUTH_STARTING_POINT_EEPROM_INITIALIZE + AZIMUTH_ROTATION_CAPABILITY_EEPROM_INITIALIZE; 
    configuration.calibration_az_flag[1] = 1;  // 1 = default endpoint 

    // configuration.calibration_el_from[0] = 0;  // don't need to actually set this because it was initialized above
    // configuration.calibration_el_to[0] = 0;    // don't need to actually set this because it was initialized above
    configuration.calibration_el_flag[0] = 1;  // 1 = default endpoint 

    configuration.calibration_el_from[1] = ELEVATION_MAXIMUM_DEGREES;
    configuration.calibration_el_to[1] = ELEVATION_MAXIMUM_DEGREES; 
    configuration.calibration_el_flag[1] = 1;  // 1 = default endpoint 

  }  //initialize_calibration_tables
#endif //FEATURE_CALIBRATION

// --------------------------------------------------------------
void initialize_eeprom_with_defaults(){

  #ifdef DEBUG_LOOP
    control_port->println("initialize_eeprom_with_defaults()");
    control_port->flush();
  #endif // DEBUG_LOOP

  #ifdef DEBUG_EEPROM
    debug.println("initialize_eeprom_with_defaults: writing eeprom");
  #endif // DEBUG_EEPROM

  configuration.analog_az_full_ccw = ANALOG_AZ_FULL_CCW_EEPROM_INITIALIZE;
  configuration.analog_az_full_cw = ANALOG_AZ_FULL_CW_EEPROM_INITIALIZE;
  configuration.analog_el_0_degrees = ANALOG_EL_FULL_DOWN_EEPROM_INITIALIZE;
  configuration.analog_el_max_elevation = ANALOG_EL_FULL_UP_EEPROM_INITIALIZE;
  configuration.last_azimuth = raw_azimuth;
  configuration.last_az_incremental_encoder_position = 0;
  configuration.last_el_incremental_encoder_position = 0;
  configuration.azimuth_offset = 0;
  configuration.elevation_offset = 0;
  configuration.azimuth_starting_point = AZIMUTH_STARTING_POINT_EEPROM_INITIALIZE;
  configuration.azimuth_rotation_capability = AZIMUTH_ROTATION_CAPABILITY_EEPROM_INITIALIZE;
  configuration.brake_az_disabled = 0; //(brake_az ? 1 : 0);
  configuration.clock_timezone_offset = 0;
  configuration.autopark_active = 0;
  configuration.autopark_time_minutes = 0;
  configuration.azimuth_display_mode = AZ_DISPLAY_MODE_NORMAL;
  

  #if defined(FEATURE_AUDIBLE_ALERT)
    configuration.audible_alert_enabled = 1;
    configuration.audible_alert_enabled_az_target = AUDIBLE_ALERT_AT_AZ_TARGET;
    configuration.audible_alert_enabled_el_target = AUDIBLE_ALERT_AT_EL_TARGET; 
  #endif  

  #if defined(FEATURE_CALIBRATION)
    initialize_calibration_tables();
  #endif

  #ifdef FEATURE_ELEVATION_CONTROL
    configuration.last_elevation = elevation;
  #else
    configuration.last_elevation = 0;
  #endif

  write_settings_to_eeprom();

} /* initialize_eeprom_with_defaults */


// --------------------------------------------------------------
void write_settings_to_eeprom(){

  #ifdef DEBUG_EEPROM
    debug.println("write_settings_to_eeprom: writing settings to eeprom");
  #endif // DEBUG_EEPROM

  configuration.configuration_struct_version = CONFIGURATION_STRUCT_VERSION;

  configuration.configuration_struct_subversion = calculated_configuration_struct_subversion(); 

  #if defined(FEATURE_CALIBRATION)
    configuration.calibration_points = CALIBRATION_POINTS;
  #endif

  const byte * p = (const byte *)(const void *)&configuration;
  unsigned int i;
  int ee = 0;
  for (i = 0; i < sizeof(configuration); i++) {
    EEPROM.write(ee++, *p++);
  }
  EEPROM.commit(); // required to persist writes on ESP32's flash-emulated EEPROM

  configuration_dirty = 0;

}

// --------------------------------------------------------------
// --------------------------------------------------------------

void az_check_operation_timeout(){

  // check if the last executed rotation operation has been going on too long


  if (((millis() - az_last_rotate_initiation) > OPERATION_TIMEOUT) && (az_state != IDLE)) {
    submit_request(AZ, REQUEST_KILL, 0, 78);
    #ifdef DEBUG_AZ_CHECK_OPERATION_TIMEOUT
      debug.println("az_check_operation_timeout: timeout reached, aborting rotation");
    #endif // DEBUG_AZ_CHECK_OPERATION_TIMEOUT
  }


}

// --------------------------------------------------------------
#if defined(FEATURE_AZ_ROTATION_STALL_DETECTION)
void az_check_rotation_stall(){

  // check if rotation has stalled

  static unsigned long last_check_time = 0;
  static int last_raw_azimuth = 0;
  static byte rotation_stall_pin_active = 0;

  if (az_state != IDLE){
    if (last_check_time == 0){
      last_raw_azimuth = raw_azimuth;
      last_check_time = millis();
      if (rotation_stall_pin_active){
        digitalWriteEnhanced(az_rotation_stall_detected,LOW);
        rotation_stall_pin_active = 0;
      }
    } else {
      if ((millis() - last_check_time) > STALL_CHECK_FREQUENCY_MS_AZ){
        if ((abs((raw_azimuth - last_raw_azimuth))) < STALL_CHECK_DEGREES_THRESHOLD_AZ){
          #ifdef DEBUG_ROTATION_STALL_DETECTION
            debug.println("az_check_rotation_stall: REQUEST_KILL");
          #endif
          #ifdef OPTION_ROTATION_STALL_DETECTION_SERIAL_MESSAGE
            control_port->println(F("AZ Rotation Stall Detected"));
          #endif  
          submit_request(AZ, REQUEST_KILL, 0, 78);
          digitalWriteEnhanced(az_rotation_stall_detected,HIGH);
          rotation_stall_pin_active = 1;
          last_check_time = 0;
        } else {
          last_raw_azimuth = raw_azimuth;
          last_check_time = millis();
        }
      }
    }
  } else {
    last_check_time = 0;
  }
}
#endif //FEATURE_AZ_ROTATION_STALL_DETECTION
// --------------------------------------------------------------
#if defined(FEATURE_EL_ROTATION_STALL_DETECTION) && defined(FEATURE_ELEVATION_CONTROL)
void el_check_rotation_stall(){

  // check if rotation has stalled

  static unsigned long last_check_time = 0;
  static int last_elevation = 0;
  static byte rotation_stall_pin_active = 0;

  if (el_state != IDLE){
    if (last_check_time == 0){
      last_elevation = elevation;
      last_check_time = millis();
      if (rotation_stall_pin_active){
        digitalWriteEnhanced(el_rotation_stall_detected,LOW);
        rotation_stall_pin_active = 0;
      }
    } else {
      if ((millis() - last_check_time) > STALL_CHECK_FREQUENCY_MS_EL){
        if ((abs((elevation - last_elevation))) < STALL_CHECK_DEGREES_THRESHOLD_AZ){
          #ifdef DEBUG_ROTATION_STALL_DETECTION
            debug.println("el_check_rotation_stall: REQUEST_KILL");
          #endif
          #ifdef OPTION_ROTATION_STALL_DETECTION_SERIAL_MESSAGE
            control_port->println(F("EL Rotation Stall Detected"));
          #endif            
          submit_request(EL, REQUEST_KILL, 0, 78);
          digitalWriteEnhanced(el_rotation_stall_detected,HIGH);
          rotation_stall_pin_active = 1;
          last_check_time = 0;
        } else {
          last_elevation = elevation;
          last_check_time = millis();
        }
      }
    }
  } else {
    last_check_time = 0;
  }
}
#endif //FEATURE_EL_ROTATION_STALL_DETECTION
// --------------------------------------------------------------

#ifdef FEATURE_TIMED_BUFFER
void clear_timed_buffer(){
  timed_buffer_status = EMPTY;
  timed_buffer_number_entries_loaded = 0;
  timed_buffer_entry_pointer = 0;
}
#endif // FEATURE_TIMED_BUFFER

// --------------------------------------------------------------

#ifdef FEATURE_TIMED_BUFFER
void initiate_timed_buffer(byte source_port){
  if (timed_buffer_status == LOADED_AZIMUTHS) {
    timed_buffer_status = RUNNING_AZIMUTHS;
    submit_request(AZ, REQUEST_AZIMUTH, timed_buffer_azimuths[1], 79);
    last_timed_buffer_action_time = millis();
    timed_buffer_entry_pointer = 2;
    #ifdef DEBUG_TIMED_BUFFER
    debug.println("initiate_timed_buffer: changing state to RUNNING_AZIMUTHS");
    #endif // DEBUG_TIMED_BUFFER
  } else {
    #ifdef FEATURE_ELEVATION_CONTROL
    if (timed_buffer_status == LOADED_AZIMUTHS_ELEVATIONS) {
      timed_buffer_status = RUNNING_AZIMUTHS_ELEVATIONS;
      submit_request(AZ, REQUEST_AZIMUTH, timed_buffer_azimuths[1], 80);
      submit_request(EL, REQUEST_ELEVATION, timed_buffer_elevations[1], 81);
      last_timed_buffer_action_time = millis();
      timed_buffer_entry_pointer = 2;
      #ifdef DEBUG_TIMED_BUFFER
        debug.println("initiate_timed_buffer: changing state to RUNNING_AZIMUTHS_ELEVATIONS");
      #endif // DEBUG_TIMED_BUFFER
    } else {
      print_to_port(">",source_port);  // error
    }
    #endif
  }

} /* initiate_timed_buffer */
#endif // FEATURE_TIMED_BUFFER
// --------------------------------------------------------------
#ifdef FEATURE_TIMED_BUFFER
void print_timed_buffer_empty_message(){

  #ifdef DEBUG_TIMED_BUFFER
  debug.println("check_timed_interval: completed timed buffer; changing state to EMPTY");
  #endif // DEBUG_TIMED_BUFFER

}

#endif // FEATURE_TIMED_BUFFER
// --------------------------------------------------------------
#ifdef FEATURE_TIMED_BUFFER
void check_timed_interval(){

  if ((timed_buffer_status == RUNNING_AZIMUTHS) && (((millis() - last_timed_buffer_action_time) / 1000) > timed_buffer_interval_value_seconds)) {
    timed_buffer_entry_pointer++;
    #ifdef DEBUG_TIMED_BUFFER
    debug.println("check_timed_interval: executing next timed interval step - azimuths");
    #endif // DEBUG_TIMED_BUFFER
    submit_request(AZ, REQUEST_AZIMUTH, timed_buffer_azimuths[timed_buffer_entry_pointer - 1], 82);
    last_timed_buffer_action_time = millis();
    if (timed_buffer_entry_pointer == timed_buffer_number_entries_loaded) {
      clear_timed_buffer();
      print_timed_buffer_empty_message();
    }
  }
  #ifdef FEATURE_ELEVATION_CONTROL
  if ((timed_buffer_status == RUNNING_AZIMUTHS_ELEVATIONS) && (((millis() - last_timed_buffer_action_time) / 1000) > timed_buffer_interval_value_seconds)) {
    timed_buffer_entry_pointer++;
    #ifdef DEBUG_TIMED_BUFFER
    debug.println("check_timed_interval: executing next timed interval step - az and el");
    #endif // DEBUG_TIMED_BUFFER
    submit_request(AZ, REQUEST_AZIMUTH, timed_buffer_azimuths[timed_buffer_entry_pointer - 1], 83);
    submit_request(EL, REQUEST_ELEVATION, timed_buffer_elevations[timed_buffer_entry_pointer - 1], 84);
    last_timed_buffer_action_time = millis();
    if (timed_buffer_entry_pointer == timed_buffer_number_entries_loaded) {
      clear_timed_buffer();
      print_timed_buffer_empty_message();

    }
  }
  #endif
} /* check_timed_interval */
#endif // FEATURE_TIMED_BUFFER
// --------------------------------------------------------------
#if !defined(FEATURE_CALIBRATION)
void apply_azimuth_offset(){

  if (configuration.azimuth_offset < 0){
    raw_azimuth = raw_azimuth + (2.0 * configuration.azimuth_offset);
  }

}
#endif
// --------------------------------------------------------------


float float_map(float x, float in_min, float in_max, float out_min, float out_max) {

  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

}

// --------------------------------------------------------------


void convert_raw_azimuth_to_real_azimuth(){

  if (raw_azimuth >= 360){
    azimuth = raw_azimuth - float(int(raw_azimuth / 360) * 360.0);
  } else {
    if (raw_azimuth < 0){
      azimuth = raw_azimuth + 360.0;
    } else {
      azimuth = raw_azimuth;
    }
  }

}

// --------------------------------------------------------------

void read_azimuth(byte force_read){

  static unsigned long last_measurement_time = 0;

  if (heading_reading_inhibit_pin) {
    if (digitalReadEnhanced(heading_reading_inhibit_pin)) {
      return;
    }
  }

  #ifdef DEBUG_HEADING_READING_TIME
    static unsigned long last_time = 0;
    static unsigned long last_print_time = 0;
    static float average_read_time = 0;
  #endif // DEBUG_HEADING_READING_TIME

  #ifdef DEBUG_HH12
    static unsigned long last_hh12_debug = 0;
  #endif

  #ifndef FEATURE_AZ_POSITION_GET_FROM_REMOTE_UNIT
    if (((millis() - last_measurement_time) > AZIMUTH_MEASUREMENT_FREQUENCY_MS) || (force_read)) {
  #else
    if (1) {
  #endif




    #ifdef FEATURE_AZ_POSITION_ROTARY_ENCODER
      static byte az_position_encoder_state = 0;
      az_position_encoder_state = ttable[az_position_encoder_state & 0xf][((digitalReadEnhanced(az_rotary_position_pin2) << 1) | digitalReadEnhanced(az_rotary_position_pin1))];
      byte az_position_encoder_result = az_position_encoder_state & 0x30;
      if (az_position_encoder_result) {
        if (az_position_encoder_result == DIR_CW) {
          configuration.last_azimuth = configuration.last_azimuth + (float)AZ_POSITION_ROTARY_ENCODER_DEG_PER_PULSE;
          #ifdef DEBUG_POSITION_ROTARY_ENCODER
            debug.println("read_azimuth: AZ_POSITION_ROTARY_ENCODER: CW");
          #endif // DEBUG_POSITION_ROTARY_ENCODER
        }
        if (az_position_encoder_result == DIR_CCW) {
          configuration.last_azimuth = configuration.last_azimuth - (float)AZ_POSITION_ROTARY_ENCODER_DEG_PER_PULSE;
          #ifdef DEBUG_POSITION_ROTARY_ENCODER
            debug.println("read_azimuth: AZ_POSITION_ROTARY_ENCODER: CCW");
          #endif // DEBUG_POSITION_ROTARY_ENCODER
        }

        #ifdef OPTION_AZ_POSITION_ROTARY_ENCODER_HARD_LIMIT
          if (configuration.last_azimuth < configuration.azimuth_starting_point) {
            configuration.last_azimuth = configuration.azimuth_starting_point;
          }
          if (configuration.last_azimuth > (configuration.azimuth_starting_point + configuration.azimuth_rotation_capability)) {
            configuration.last_azimuth = (configuration.azimuth_starting_point + configuration.azimuth_rotation_capability);
          }
        #else
          if (configuration.last_azimuth < 0) {
            configuration.last_azimuth += 360.0;
          }
          if (configuration.last_azimuth >= 360) {
            configuration.last_azimuth -= 360.0;
          }
        #endif // OPTION_AZ_POSITION_ROTARY_ENCODER_HARD_LIMIT

        raw_azimuth = configuration.last_azimuth;

        #ifdef FEATURE_AZIMUTH_CORRECTION
          raw_azimuth = correct_azimuth(raw_azimuth);
        #endif // FEATURE_AZIMUTH_CORRECTION

        convert_raw_azimuth_to_real_azimuth();

        configuration_dirty = 1;  // TODO: a better way to handle configuration writes; these are very frequent
      }
    #endif // FEATURE_AZ_POSITION_ROTARY_ENCODER





    #ifdef FEATURE_AZ_POSITION_PULSE_INPUT
      #ifdef DEBUG_POSITION_PULSE_INPUT
  //    if (az_position_pule_interrupt_handler_flag) {
  //      control_port->print(F("read_azimuth: az_position_pusle_interrupt_handler_flag: "));
  //      control_port->println(az_position_pule_interrupt_handler_flag);
  //      az_position_pule_interrupt_handler_flag = 0;
  //    }
      #endif // DEBUG_POSITION_PULSE_INPUT
      static float last_az_position_pulse_input_azimuth = az_position_pulse_input_azimuth;
      if (az_position_pulse_input_azimuth != last_az_position_pulse_input_azimuth) {
        #ifdef DEBUG_POSITION_PULSE_INPUT
  //        if (debug_mode){
  //          control_port->print(F("read_azimuth: last_az_position_pulse_input_azimuth:"));
  //          control_port->print(last_az_position_pulse_input_azimuth);
  //          control_port->print(F(" az_position_pulse_input_azimuth:"));
  //          control_port->print(az_position_pulse_input_azimuth);
  //          control_port->print(F(" az_pulse_counter:"));
  //          control_port->println(az_pulse_counter);
  //        }
        #endif // DEBUG_POSITION_PULSE_INPUT
        configuration.last_azimuth = az_position_pulse_input_azimuth;
        configuration_dirty = 1;
        last_az_position_pulse_input_azimuth = az_position_pulse_input_azimuth;
        raw_azimuth = configuration.last_azimuth;
        #ifdef FEATURE_AZIMUTH_CORRECTION
          raw_azimuth = correct_azimuth(raw_azimuth);
        #endif // FEATURE_AZIMUTH_CORRECTION
        #if !defined(FEATURE_CALIBRATION)  
        apply_azimuth_offset();
        #endif
        convert_raw_azimuth_to_real_azimuth();
      }
    #endif // FEATURE_AZ_POSITION_PULSE_INPUT


    last_measurement_time = millis();
  }





} /* read_azimuth */

// --------------------------------------------------------------

void output_debug(){

  #ifdef DEBUG_DUMP

    #ifdef DEBUG_PROCESSES
      service_process_debug(DEBUG_PROCESSES_PROCESS_ENTER,PROCESS_DEBUG);
    #endif


    char tempstring[32];

    #if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(CONTROL_PROTOCOL_EMULATION) || defined(UNDER_DEVELOPMENT_REMOTE_UNIT_COMMANDS)

      if (((millis() - last_debug_output_time) >= ((int)periodic_debug_dump_time_seconds*1000)) && (debug_mode)) {

        #if defined(DEBUG_GPS_SERIAL)
          debug.println("");
        #endif //DEBUG_GPS_SERIAL

        //port_flush();

        debug.print("debug: \t");
        debug.print(CODE_VERSION);
        #ifdef HARDWARE_WB6KCN
          debug.print(" HARDWARE_WB6KCN");
        #endif
        #ifdef HARDWARE_WB6KCN_K3NG
          debug.print(" HARDWARE_WB6KCN_K3NG");
        #endif        
        #ifdef HARDWARE_M0UPU
          debug.print(" HARDWARE_M0UPU");
        #endif       
        debug.print("\t\t");

        #ifdef FEATURE_CLOCK
          //update_time();
          if (configuration.clock_timezone_offset != 0){
            sprintf(tempstring, "%s", timezone_modified_clock_string());
            debug.print(tempstring);
            debug.print("UTC");
            if (configuration.clock_timezone_offset > 0){
              debug.print("+");
            }
            if (configuration.clock_timezone_offset == int(configuration.clock_timezone_offset)){
              debug.print(int(configuration.clock_timezone_offset));
            } else {

              debug.print(configuration.clock_timezone_offset);
            }
            debug.print("\t");
            sprintf(tempstring, "%s", zulu_clock_string());
            debug.print(tempstring);
          } else {
            sprintf(tempstring, "%s", zulu_clock_string());
            debug.print(tempstring);
          }       
        #else // FEATURE_CLOCK
          dtostrf((millis() / 1000),0,0,tempstring);
          debug.print("Uptime:");
          debug.print(tempstring);
        #endif // FEATURE_CLOCK

        #if defined(FEATURE_GPS) || defined(FEATURE_RTC) || (defined(FEATURE_CLOCK) && defined(OPTION_SYNC_MASTER_CLOCK_TO_SLAVE))
          debug.print("\t");
          debug.print(clock_status_string());
        #endif // defined(FEATURE_GPS) || defined(FEATURE_RTC) || (defined(FEATURE_CLOCK) && defined(OPTION_SYNC_MASTER_CLOCK_TO_SLAVE))

        #if defined(FEATURE_MOON_TRACKING) || defined(FEATURE_SUN_TRACKING) || defined(FEATURE_GPS)
          debug.print("\t");
          dtostrf(latitude,0,4,tempstring);
          debug.print(tempstring);
          debug.print(" ");
          dtostrf(longitude,0,4,tempstring);
          debug.print(tempstring);
          debug.print(" ");
          debug.print(coordinates_to_maidenhead(latitude,longitude));
        #endif

        debug.print("\t\t");


        #ifdef FEATURE_YAESU_EMULATION
          debug.print("GS-232");
          #ifdef OPTION_GS_232B_EMULATION
            debug.print("B");
          #else
            debug.print("A");
          #endif
        #endif // FEATURE_YAESU_EMULATION

        #ifdef FEATURE_EASYCOM_EMULATION
          debug.print("EASYCOM");
        #endif // FEATURE_EASYCOM_EMULATION            

        #ifdef FEATURE_DCU_1_EMULATION
          debug.print("DCU-1");
        #endif // FEATURE_DCU_1_EMULATION  

        #ifdef FEATURE_PARK
          switch (park_status) {
            case NOT_PARKED: debug.print("\tNOT_PARKED"); break;
            case PARK_INITIATED: debug.print("\tPARK_INITIATED"); break;
            case PARKED: debug.print("\tPARKED"); break;
          }
        #endif // FEATURE_PARK

        debug.println("");

        debug.print("\tAZ:");
        switch (az_state) {
          case IDLE: debug.print("IDLE"); break;
          #ifndef HARDWARE_EA4TX_ARS_USB
            case SLOW_START_CW: debug.print("SLOW_START_CW"); break;
            case SLOW_START_CCW: debug.print("SLOW_START_CCW"); break;
          #endif //ifndef HARDWARE_EA4TX_ARS_USB
          case NORMAL_CW: debug.print("NORMAL_CW"); break;
          case NORMAL_CCW: debug.print("NORMAL_CCW"); break;
          #ifndef HARDWARE_EA4TX_ARS_USB
            case SLOW_DOWN_CW: debug.print("SLOW_DOWN_CW"); break;
            case SLOW_DOWN_CCW: debug.print("SLOW_DOWN_CCW"); break;
            case INITIALIZE_SLOW_START_CW: debug.print("INITIALIZE_SLOW_START_CW"); break;
            case INITIALIZE_SLOW_START_CCW: debug.print("INITIALIZE_SLOW_START_CCW"); break;
            case INITIALIZE_TIMED_SLOW_DOWN_CW: debug.print("INITIALIZE_TIMED_SLOW_DOWN_CW"); break;
            case INITIALIZE_TIMED_SLOW_DOWN_CCW: debug.print("INITIALIZE_TIMED_SLOW_DOWN_CCW"); break;
            case TIMED_SLOW_DOWN_CW: debug.print("TIMED_SLOW_DOWN_CW"); break;
            case TIMED_SLOW_DOWN_CCW: debug.print("TIMED_SLOW_DOWN_CCW"); break;
            case INITIALIZE_DIR_CHANGE_TO_CW: debug.print("INITIALIZE_DIR_CHANGE_TO_CW"); break;
            case INITIALIZE_DIR_CHANGE_TO_CCW: debug.print("INITIALIZE_DIR_CHANGE_TO_CCW"); break;
            case INITIALIZE_NORMAL_CW: debug.print("INITIALIZE_NORMAL_CW"); break;
            case INITIALIZE_NORMAL_CCW: debug.print("INITIALIZE_NORMAL_CCW"); break; 
          #endif //ifndef HARDWARE_EA4TX_ARS_USB     
        }

        debug.print("  Q:");
        switch (az_request_queue_state) {
          case NONE: debug.print("-"); break;
          case IN_QUEUE: debug.print("IN_QUEUE"); break;
          case IN_PROGRESS_TIMED: debug.print("IN_PROGRESS_TIMED"); break;
          case IN_PROGRESS_TO_TARGET: debug.print("IN_PROGRESS_TO_TARGET"); break;
        }

        debug.print("  AZ:");
        debug.print(azimuth, DISPLAY_DECIMAL_PLACES);
        debug.print("  AZ_raw:");
        debug.print((raw_azimuth ), DISPLAY_DECIMAL_PLACES);
        //debug.print(")");


        if (az_state != IDLE) {
          debug.print("  Target:");
          debug.print(target_azimuth, DISPLAY_DECIMAL_PLACES);
       

          debug.print("  Target_raw: ");

          debug.print(target_raw_azimuth, DISPLAY_DECIMAL_PLACES);
          //debug.print(")");

          debug.print("  Secs_left:");
          debug.print((OPERATION_TIMEOUT - (millis() - az_last_rotate_initiation)) / 1000);
        }

        #ifdef FEATURE_AZ_POSITION_POTENTIOMETER
          debug.print("  Analog:");
          dtostrf(analog_az,0,0,tempstring);
          debug.print(tempstring);
          debug.print("  Range:");
          dtostrf(configuration.analog_az_full_ccw,0,0,tempstring);
          debug.print(tempstring);
          debug.print("-");
          dtostrf(configuration.analog_az_full_cw,0,0,tempstring);
          debug.print(tempstring);
          //debug.print(") ");
        #endif // FEATURE_AZ_POSITION_POTENTIOMETER

        debug.print(F("  Start:"));
        debug.print(configuration.azimuth_starting_point);
        debug.print(F("  Rotation_Capability:"));
        debug.print(configuration.azimuth_rotation_capability,0);
        debug.print(F("  Raw_Az_Range:"));
        debug.print(configuration.azimuth_starting_point);
        debug.print("-");
        debug.print((configuration.azimuth_starting_point+configuration.azimuth_rotation_capability),0);
        //debug.println("");
        //debug.print("\t");

        #ifndef HARDWARE_EA4TX_ARS_USB
          debug.print("  AZ_Speed_Norm:");
          debug.print(normal_az_speed_voltage);
          debug.print("  Current:");
          debug.print(current_az_speed_voltage);
          if (az_speed_pot) {
            debug.print("  AZ_Speed_Pot:");
            debug.print(analogReadEnhanced(az_speed_pot));
          }
          if (az_preset_pot) {
            debug.print(F("  AZ_Preset_Pot_Analog:"));
            debug.print(analogReadEnhanced(az_preset_pot));
            debug.print(F("  AZ_Preset_Pot_Setting: "));
            dtostrf((map(analogReadEnhanced(az_preset_pot), AZ_PRESET_POT_FULL_CW, AZ_PRESET_POT_FULL_CCW, AZ_PRESET_POT_FULL_CW_MAP, AZ_PRESET_POT_FULL_CCW_MAP)),0,0,tempstring);
            debug.print(tempstring);
          }
          #if defined(ANALOG_AZ_OVERLAP_DEGREES)
            if (raw_azimuth > ANALOG_AZ_OVERLAP_DEGREES){
              debug.print(F(" OVERLAP"));
            }
          #endif
          #if !defined(FEATURE_CALIBRATION)  
          debug.print(F("  Offset:"));
          dtostrf(configuration.azimuth_offset,0,2,tempstring);
          debug.print(tempstring);
          #endif
        #endif // ndef HARDWARE_EA4TX_ARS_USB
        debug.println("");

   

        #ifdef FEATURE_ELEVATION_CONTROL
          debug.print("\tEL:");
          switch (el_state) {
            case IDLE: debug.print("IDLE"); break;
            #ifndef HARDWARE_EA4TX_ARS_USB
              case SLOW_START_UP: debug.print("SLOW_START_UP"); break;
              case SLOW_START_DOWN: debug.print("SLOW_START_DOWN"); break;
            #endif //ifndef HARDWARE_EA4TX_ARS_USB
            case NORMAL_UP: debug.print("NORMAL_UP"); break;
            case NORMAL_DOWN: debug.print("NORMAL_DOWN"); break;
            #ifndef HARDWARE_EA4TX_ARS_USB
              case SLOW_DOWN_DOWN: debug.print("SLOW_DOWN_DOWN"); break;
              case SLOW_DOWN_UP: debug.print("SLOW_DOWN_UP"); break;
              case TIMED_SLOW_DOWN_UP: debug.print("TIMED_SLOW_DOWN_UP"); break;
              case TIMED_SLOW_DOWN_DOWN: debug.print("TIMED_SLOW_DOWN_DOWN"); break;
            #endif //ifndef HARDWARE_EA4TX_ARS_USB
          }

          debug.print("  Q:");
          switch (el_request_queue_state) {
            case NONE: debug.print("-"); break;
            case IN_QUEUE: debug.print("IN_QUEUE"); break;
            case IN_PROGRESS_TIMED: debug.print("IN_PROGRESS_TIMED"); break;
            case IN_PROGRESS_TO_TARGET: debug.print("IN_PROGRESS_TO_TARGET"); break;
          }
          debug.print("  EL:");
          dtostrf(elevation , 0, DISPLAY_DECIMAL_PLACES,tempstring);
          debug.print(tempstring);
          if (el_state != IDLE) {
            debug.print("  Target:");
            dtostrf(target_elevation, 0, DISPLAY_DECIMAL_PLACES,tempstring);
            debug.print(tempstring);
          }

          #ifdef FEATURE_EL_POSITION_POTENTIOMETER
            debug.print("  EL_Analog:");
            dtostrf(analog_el,0,0,tempstring);
            debug.print(tempstring);
            debug.print("  Range:");
            dtostrf(configuration.analog_el_0_degrees,0,0,tempstring);
            debug.print(tempstring);
            debug.print("-");
            dtostrf(configuration.analog_el_max_elevation,0,0,tempstring);
            debug.print(tempstring);
            //debug.print(") ");
          #endif // FEATURE_EL_POSITION_POTENTIOMETER
         
          #ifndef HARDWARE_EA4TX_ARS_USB
            debug.print("  EL_Speed_Norm:");
            debug.print(normal_el_speed_voltage);


            debug.print("  Current:");
            debug.print(current_el_speed_voltage);
            #if !defined(FEATURE_CALIBRATION)  
            debug.print("  Offset:");
            debug.print(configuration.elevation_offset, 2);
            #endif
          #endif //ifndef HARDWARE_EA4TX_ARS_USB
          debug.println("");
        #endif // FEATURE_ELEVATION_CONTROL

        //port_flush();

        #ifdef FEATURE_TIMED_BUFFER
          if (timed_buffer_status != EMPTY) {
            debug.print("  Timed_interval_buff:");
            switch (timed_buffer_status) {
              // case EMPTY: debug.print("EMPTY"); break;
              case LOADED_AZIMUTHS: debug.print("LOADED_AZIMUTHS"); break;
              case RUNNING_AZIMUTHS: debug.print("RUNNING_AZIMUTHS"); break;
              #ifdef FEATURE_ELEVATION_CONTROL
                case LOADED_AZIMUTHS_ELEVATIONS: debug.print("LOADED_AZIMUTHS_ELEVATIONS"); break;
                case RUNNING_AZIMUTHS_ELEVATIONS: debug.print("RUNNING_AZIMUTHS_ELEVATIONS"); break;
              #endif
            }

            debug.print("  Interval_secs:");
            debug.print(timed_buffer_interval_value_seconds);
            debug.print("  Entries:");
            debug.print(timed_buffer_number_entries_loaded);
            debug.print("  Entry_ptr:");
            debug.print(timed_buffer_entry_pointer);
            debug.print("  Secs_since_last_action:");
            debug.print((millis() - last_timed_buffer_action_time) / 1000);

            if (timed_buffer_number_entries_loaded > 0) {
              for (int x = 0; x < timed_buffer_number_entries_loaded; x++) {
                debug.print(x + 1);
                debug.print("\t:");
                debug.print(timed_buffer_azimuths[x]);
              #ifdef FEATURE_ELEVATION_CONTROL
                debug.print("\t- ");
                debug.print(timed_buffer_elevations[x]);
              #endif
                debug.print("\n");
              }
              debug.println("");
            }

          } // if (timed_buffer_status != EMPTY)
        #endif // FEATURE_TIMED_BUFFER


        #if defined(FEATURE_MASTER_WITH_SERIAL_SLAVE) || defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE)
          debug.print("\tRemote_Unit: Link:");
          if (master_remote_link_state == MASTER_REMOTE_LINK_UP){
            debug.print("UP");
          } else {
            debug.print("DOWN");
          }  
          debug.print(" Good:");
          debug.print(remote_unit_good_results,0);
          debug.print(" Bad:");
          debug.print(remote_unit_bad_results);
          /*debug.print(" Index: ");
          debug.print(remote_unit_port_buffer_index);*/
          debug.print(" CmdTouts:");
          debug.print(remote_unit_command_timeouts);
          debug.print(" BuffTouts:");
          debug.print(remote_unit_incoming_buffer_timeouts);
          /*debug.print(" Result: ");
          debug.print(remote_unit_command_result_float,2);*/
          debug.println("");
        #endif // defined(FEATURE_MASTER_WITH_SERIAL_SLAVE) || defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE)

        #if defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE)
          debug.print("\tEthernet Slave TCP Link State:");
          switch(ethernetslavelinkclient0_state){
            case ETHERNET_SLAVE_DISCONNECTED: debug.print("DIS");
            case ETHERNET_SLAVE_CONNECTED: debug.print("CONNECTED");
          }
          debug.print(" Reconnects:");
          debug.print(ethernet_slave_reconnects);  
          debug.println("");
        #endif // defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE)  

        #ifdef DEBUG_POSITION_PULSE_INPUT
          static unsigned long last_pulse_count_time = 0;
          static unsigned long last_az_pulse_counter = 0;
          static unsigned long last_el_pulse_counter = 0;
          debug.print("\tPulse_counters: AZ:");
          debug.print(az_pulse_counter);
          debug.print("  AZ_Ambiguous:");
          debug.print(az_pulse_counter_ambiguous);
          debug.print("  EL:");
          debug.print(el_pulse_counter);
          debug.print("  EL_Ambiguous:");
          debug.print(el_pulse_counter_ambiguous);
          debug.print("  Rate_per_sec: AZ:");
          debug.print(((az_pulse_counter - last_az_pulse_counter) / ((millis() - last_pulse_count_time) / 1000.0)),2);
          debug.print("   EL:");
          debug.print(((el_pulse_counter - last_el_pulse_counter) / ((millis() - last_pulse_count_time) / 1000.0)),2);
          debug.println("");
          last_az_pulse_counter = az_pulse_counter;
          last_el_pulse_counter = el_pulse_counter;
          last_pulse_count_time = millis();
        #endif // DEBUG_POSITION_PULSE_INPUT


        #if defined(FEATURE_AZ_POSITION_INCREMENTAL_ENCODER) && defined(DEBUG_AZ_POSITION_INCREMENTAL_ENCODER)
          debug.print("\taz_position_incremental_encoder_interrupt:");
          debug.print(az_position_incremental_encoder_interrupt);
          debug.print("  az_incremental_encoder_position:");
          debug.print(az_incremental_encoder_position,0);
        #endif // DEBUG_AZ_POSITION_INCREMENTAL_ENCODER
        #if defined(FEATURE_EL_POSITION_INCREMENTAL_ENCODER) && defined(DEBUG_EL_POSITION_INCREMENTAL_ENCODER)
          debug.print("\n\tel_position_incremental_encoder_interrupt:");
          debug.print(el_position_incremental_encoder_interrupt,0);
          debug.print("  el_incremental_encoder_position: ");
          debug.print(el_incremental_encoder_position);
        #endif // DEBUG_EL_POSITION_INCREMENTAL_ENCODER
        #if (defined(FEATURE_AZ_POSITION_INCREMENTAL_ENCODER) && defined(DEBUG_AZ_POSITION_INCREMENTAL_ENCODER)) || (defined(FEATURE_EL_POSITION_INCREMENTAL_ENCODER) && defined(DEBUG_EL_POSITION_INCREMENTAL_ENCODER))
          debug.println("");
        #endif

        #ifdef FEATURE_MOON_TRACKING
          update_moon_position();
          debug.print("\tmoon: AZ:");
          dtostrf(moon_azimuth,0,2,tempstring);
          debug.print(tempstring);
          debug.print(" EL:");
          dtostrf(moon_elevation,0,2,tempstring);
          debug.print(tempstring);
          debug.print("  TRACKING_");
          if (!moon_tracking_active) {
            debug.print("IN");
          }
          debug.print("ACTIVE ");
          if (moon_tracking_active) {
            if (!moon_visible) {
              debug.print("NOT_");
            }
            debug.print("VISIBLE");
          }
        #endif // FEATURE_MOON_TRACKING

        #ifdef FEATURE_SUN_TRACKING
          update_sun_position();
          debug.print("\tsun: AZ:");
          dtostrf(sun_azimuth,0,2,tempstring);
          debug.print(tempstring);
          debug.print(" EL:");
          dtostrf(sun_elevation,0,2,tempstring);
          debug.print(tempstring);
          debug.print("  TRACKING_");
          if (!sun_tracking_active) {
            debug.print("IN");
          }
          debug.print("ACTIVE ");
          if (sun_tracking_active) {
            if (!sun_visible) {
              debug.print("NOT_");
            }
            debug.print("VISIBLE");
          }
        #endif // FEATURE_SUN_TRACKING

        #if defined(FEATURE_MOON_TRACKING) || defined(FEATURE_SUN_TRACKING)
          debug.println("");
        #endif //defined(FEATURE_MOON_TRACKING) || defined(FEATURE_SUN_TRACKING)

        #ifdef FEATURE_GPS
          unsigned long gps_chars = 0;
          unsigned short gps_good_sentences = 0;
          unsigned short gps_failed_checksum = 0;
          char gps_temp_string[12] = "";
          float gps_lat_temp = 0;
          float gps_long_temp = 0;

          debug.print("\tGPS: satellites:");
          gps_chars = gps.satellites();
          //if (gps_chars == 255){gps_chars = 0;}
          dtostrf(gps_chars,0,0,gps_temp_string);
          debug.print(gps_temp_string);  
          unsigned long gps_fix_age_temp = 0;
          gps.f_get_position(&gps_lat_temp,&gps_long_temp,&gps_fix_age_temp); 
          debug.print("  lat:");
          debug.print(gps_lat_temp,4);
          debug.print("  long:");
          debug.print(gps_long_temp,4);
          debug.print("  altitude(m):");
          debug.print(gps.altitude()/100,0);            
          debug.print("  fix_age_mS:");
          dtostrf(gps_fix_age_temp,0,0,gps_temp_string);
          debug.print(gps_temp_string);   
          gps.stats(&gps_chars,&gps_good_sentences,&gps_failed_checksum);     
          debug.print("  data_chars:");
          dtostrf(gps_chars,0,0,gps_temp_string);
          debug.print(gps_temp_string);
          debug.print("  good_sentences:");
          dtostrf(gps_good_sentences,0,0,gps_temp_string);
          debug.print(gps_temp_string);    
          debug.print("  failed_checksum:");
          if (gps_failed_checksum == 1){gps_failed_checksum = 0;}  // hack to ignore that one failed checksum you always seem to get at boot up
          dtostrf(gps_failed_checksum,0,0,gps_temp_string);
          debug.print(gps_temp_string);    
          debug.println("");
        #endif //FEATURE_GPS



        #ifdef FEATURE_AUTOCORRECT
          debug.print("\tAutocorrect: AZ:");
          switch(autocorrect_state_az){
            case AUTOCORRECT_INACTIVE: debug.print("INACTIVE"); break;
            case AUTOCORRECT_WAITING_AZ: debug.print("AUTOCORRECT_WAITING_AZ: "); debug.print(autocorrect_az,2); break;
            case AUTOCORRECT_WATCHING_AZ: debug.print("AUTOCORRECT_WATCHING_AZ: "); debug.print(autocorrect_az,2); break;
          }

          #ifdef FEATURE_ELEVATION_CONTROL
            debug.print(" EL:");
            switch(autocorrect_state_el){
              case AUTOCORRECT_INACTIVE: debug.print("INACTIVE"); break;
              case AUTOCORRECT_WAITING_EL: debug.print("AUTOCORRECT_WAITING_EL: "); debug.print(autocorrect_el,2); break;
              case AUTOCORRECT_WATCHING_EL: debug.print("AUTOCORRECT_WATCHING_EL: "); debug.print(autocorrect_el,2); break;
            }
          #endif //FEATURE_ELEVATION_CONTROL
        #endif //DEBUG_AUTOCORRECT

        #if defined(FEATURE_SATELLITE_TRACKING)
          debug.print("\tSat:");
          debug.print(configuration.current_satellite);
          debug.print(" AZ:");
          debug.print(current_satellite_azimuth);
          debug.print(" EL:");
          debug.print(current_satellite_elevation);
          debug.print(" Lat:");
          debug.print(current_satellite_latitude);
          debug.print(" Lon:");
          debug.print(current_satellite_longitude);
          if ((satellite[current_satellite_position_in_array].status & 1) == 1) {
            debug.print("  AOS");
          } else {
            debug.print("  LOS");
          }
          debug.print("  TRACKING_");
          if (!satellite_tracking_active) {
            debug.print("IN");
          }
          debug.print("ACTIVE  Next AOS:");
         
          #if defined(OPTION_USE_OLD_TIME_CODE)
          debug.print(satellite[current_satellite_position_in_array].next_aos.year);
          debug.print("-");
          if (satellite[current_satellite_position_in_array].next_aos.month < 10){debug.print("0");}
          debug.print(satellite[current_satellite_position_in_array].next_aos.month);
          debug.print("-");    
          if (satellite[current_satellite_position_in_array].next_aos.day < 10){debug.print("0");}
          debug.print(satellite[current_satellite_position_in_array].next_aos.day);
          debug.print(" ");
          if (satellite[current_satellite_position_in_array].next_aos.hours < 10){debug.print("0");}
          debug.print(satellite[current_satellite_position_in_array].next_aos.hours);
          debug.print(":");
          if (satellite[current_satellite_position_in_array].next_aos.minutes < 10){debug.print("0");}
          debug.print(satellite[current_satellite_position_in_array].next_aos.minutes);


          debug.print(" ");
          if (current_satellite_next_aos_az < 10){debug.print(" ");}
          if (current_satellite_next_aos_az < 100){debug.print(" ");}
          debug.print(current_satellite_next_aos_az);
          debug.print(" ");
          if (current_satellite_next_aos_el > 0){debug.print(" ");}
          debug.print(current_satellite_next_aos_el);
          debug.print("  LOS:");
          debug.print(satellite[current_satellite_position_in_array].next_los.year);
          debug.print("-");
          if (satellite[current_satellite_position_in_array].next_los.month < 10){debug.print("0");}
          debug.print(satellite[current_satellite_position_in_array].next_los.month);
          debug.print("-");    
          if (satellite[current_satellite_position_in_array].next_los.day < 10){debug.print("0");}
          debug.print(satellite[current_satellite_position_in_array].next_los.day);
          debug.print(" ");
          if (satellite[current_satellite_position_in_array].next_los.hours < 10){debug.print("0");}
          debug.print(satellite[current_satellite_position_in_array].next_los.hours);
          debug.print(":");
          if (satellite[current_satellite_position_in_array].next_los.minutes < 10){debug.print("0");}
          debug.print(satellite[current_satellite_position_in_array].next_los.minutes);
          debug.print(" ");
          if (current_satellite_next_los_az < 10){debug.print(" ");}
          if (current_satellite_next_los_az < 100){debug.print(" ");}
          debug.print(current_satellite_next_los_az);
          debug.print(" ");
          if (current_satellite_next_los_el > 0){debug.print(" ");}
          debug.println(current_satellite_next_los_el);

          debug.print(F("\tSat Calc Svc: "));
          switch(service_calc_satellite_data_service_state){
            case SERVICE_IDLE:
              debug.print(F("IDLE"));
              break;
            case SERVICE_CALC_IN_PROGRESS:
              debug.print(F("SERVICE_CALC_IN_PROGRESS "));
              if (service_calc_satellite_data_task == UPDATE_SAT_ARRAY_SLOT_JUST_AZ_EL){debug.print(F("AZ_EL"));}
              if (service_calc_satellite_data_task == UPDATE_SAT_ARRAY_SLOT_AZ_EL_NEXT_AOS_LOS){debug.print(F("AZ_EL_NEXT_AOS_LOS"));}
              debug.print(" ");
              debug.print(satellite[service_calc_current_sat].name);
              break;              
          }


          #else //OPTION_USE_OLD_TIME_CODE

          debug.print(satellite[current_satellite_position_in_array].next_aos.Year+1970);
          debug.print("-");
          if (satellite[current_satellite_position_in_array].next_aos.Month < 10){debug.print("0");}
          debug.print(satellite[current_satellite_position_in_array].next_aos.Month);
          debug.print("-");    
          if (satellite[current_satellite_position_in_array].next_aos.Day < 10){debug.print("0");}
          debug.print(satellite[current_satellite_position_in_array].next_aos.Day);
          debug.print(" ");
          if (satellite[current_satellite_position_in_array].next_aos.Hour < 10){debug.print("0");}
          debug.print(satellite[current_satellite_position_in_array].next_aos.Hour);
          debug.print(":");
          if (satellite[current_satellite_position_in_array].next_aos.Minute < 10){debug.print("0");}
          debug.print(satellite[current_satellite_position_in_array].next_aos.Minute);


          debug.print(" ");
          if (current_satellite_next_aos_az < 10){debug.print(" ");}
          if (current_satellite_next_aos_az < 100){debug.print(" ");}
          debug.print(current_satellite_next_aos_az);
          debug.print(" ");
          if (current_satellite_next_aos_el > 0){debug.print(" ");}
          debug.print(current_satellite_next_aos_el);
          debug.print("  LOS:");
          debug.print(satellite[current_satellite_position_in_array].next_los.Year+1970);
          debug.print("-");
          if (satellite[current_satellite_position_in_array].next_los.Month < 10){debug.print("0");}
          debug.print(satellite[current_satellite_position_in_array].next_los.Month);
          debug.print("-");    
          if (satellite[current_satellite_position_in_array].next_los.Day < 10){debug.print("0");}
          debug.print(satellite[current_satellite_position_in_array].next_los.Day);
          debug.print(" ");
          if (satellite[current_satellite_position_in_array].next_los.Hour < 10){debug.print("0");}
          debug.print(satellite[current_satellite_position_in_array].next_los.Hour);
          debug.print(":");
          if (satellite[current_satellite_position_in_array].next_los.Minute < 10){debug.print("0");}
          debug.print(satellite[current_satellite_position_in_array].next_los.Minute);
          debug.print(" ");
          if (current_satellite_next_los_az < 10){debug.print(" ");}
          if (current_satellite_next_los_az < 100){debug.print(" ");}
          debug.print(current_satellite_next_los_az);
          debug.print(" ");
          if (current_satellite_next_los_el > 0){debug.print(" ");}
          debug.println(current_satellite_next_los_el);

          debug.print(F("\tSat Calc Svc: "));
          switch(service_calc_satellite_data_service_state){
            case SERVICE_IDLE:
              debug.print(F("IDLE"));
              break;
            case SERVICE_CALC_IN_PROGRESS:
              debug.print(F("SERVICE_CALC_IN_PROGRESS "));
              if (service_calc_satellite_data_task == UPDATE_SAT_ARRAY_SLOT_JUST_AZ_EL){debug.print(F("AZ_EL"));}
              if (service_calc_satellite_data_task == UPDATE_SAT_ARRAY_SLOT_AZ_EL_NEXT_AOS_LOS){debug.print(F("AZ_EL_NEXT_AOS_LOS"));}
              debug.print(" ");
              debug.print(satellite[service_calc_current_sat].name);
              break;              
          }

          #endif //OPTION_USE_OLD_TIME_CODE

          debug.print(F(" In Sat Calc Timeout: "));

          for (int x = 0;x < SATELLITE_LIST_LENGTH;x++){
            if((satellite[x].status & 2) == 2){
              debug.print(satellite[x].name);
              debug.print(" ");
            }             
          } 

          debug.println("$");
        #endif

        debug.print(F("\tCONFIG_"));
        if (!configuration_dirty){
          debug.print(F("NOT_"));
        }
        debug.println(F("DIRTY"));

        #if defined(ESP32)
          debug.print(F("Free heap: "));
          debug.print(ESP.getFreeHeap());
          debug.println(F("b"));
        #elif !defined(TEENSYDUINO)
          void * HP = malloc(4);
          if (HP) {free(HP);}
          unsigned long free = (unsigned long)SP - (unsigned long)HP;
          sprintf(tempstring,"%lu",(unsigned long)free);
          if ((free < 500) || (free > 10000)){
            debug.print(F("WARNING: Low memory: "));
            debug.print(tempstring);
            debug.println(F("b free"));
          }
        #endif

        debug.println("\n\n\n");

        last_debug_output_time = millis(); 

      }
    #endif // defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(CONTROL_PROTOCOL_EMULATION) || defined(UNDER_DEVELOPMENT_REMOTE_UNIT_COMMANDS)

    #ifdef DEBUG_PROCESSES
      service_process_debug(DEBUG_PROCESSES_PROCESS_EXIT,PROCESS_DEBUG);
    #endif    
    
  #endif //DEBUG_DUMP

} /* output_debug */


// --------------------------------------------------------------
void print_to_port(char * print_this,byte port){

  #if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
  
  switch(port){
    case CONTROL_PORT0: control_port->println(print_this);break;
    #ifdef FEATURE_WIFI
    case WIFI_PORT0: wifi_client0.print(print_this); break;
    #endif //FEATURE_WIFI
  }
  
  #endif //defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)

}


// --------------------------------------------------------------
void print_help(byte port){

  // The H command

  #if defined(OPTION_SERIAL_HELP_TEXT) && defined(FEATURE_YAESU_EMULATION)
    print_to_port("R Rotate Azimuth Clockwise\n",port);
    print_to_port("L Rotate Azimuth Counter Clockwise\n",port);
    print_to_port("A Stop\n",port);
    print_to_port("C Report Azimuth in Degrees\n",port);
    #ifdef FEATURE_ELEVATION_CONTROL
      print_to_port("C2 Report Azimuth and Elevation in Degrees\n",port);
    #endif // FEATURE_ELEVATION_CONTROL
    print_to_port("M### Rotate to ### degrees\n",port);
    print_to_port("MTTT XXX XXX XXX ... Timed Interval Direction Setting  (TTT = Step value in seconds, XXX = Azimuth in degrees)\n",port);
    print_to_port("T Start Timed Interval Tracking\n",port);
    print_to_port("N Report Total Number of M Timed Interval Azimuths\n",port);
    print_to_port("X1 Horizontal Rotation Low Speed\n",port);
    print_to_port("X2 Horizontal Rotation Middle 1 Speed\n",port);
    print_to_port("X3 Horizontal Rotation Middle 2 Speed\n",port);
    print_to_port("X4 Horizontal Rotation High Speed\n",port);
    print_to_port("S Stop\n",port);
    print_to_port("O Offset Calibration\n",port);
    print_to_port("F Full Scale Calibration\n",port);
    #ifdef FEATURE_ELEVATION_CONTROL
      print_to_port("U Rotate Elevation Up\n",port);
      print_to_port("D Rotate Elevation Down\n",port);
      print_to_port("E Stop Elevation Rotation\n",port);
      print_to_port("B Report Elevation in Degrees\n",port);
      print_to_port("Wxxx yyy Rotate Azimuth to xxx Degrees and Elevation to yyy Degrees\n",port);
      print_to_port("O2 Elevation Offset Calibration (0 degrees)\n",port);
      print_to_port("F2 Elevation Full Scale Calibration (180 degrees (or maximum))\n",port);
    #endif // FEATURE_ELEVATION_CONTROL
    #ifdef OPTION_GS_232B_EMULATION
      print_to_port("P36 Switch to 360 degree mode\n",port);
      print_to_port("P45 Switch to 450 degree mode\n",port);
    #endif // OPTION_GS_232B_EMULATION
    print_to_port("Z Toggle north / south centered mode\n",port);
  #endif // defined(OPTION_SERIAL_HELP_TEXT) && defined(FEATURE_YAESU_EMULATION)


} /* print_help */

// --------------- Elevation -----------------------

#ifdef FEATURE_ELEVATION_CONTROL
void el_check_operation_timeout(){

  // check if the last executed rotation operation has been going on too long

  if (((millis() - el_last_rotate_initiation) > OPERATION_TIMEOUT) && (el_state != IDLE)) {
    submit_request(EL, REQUEST_KILL, 0, 85);
    #ifdef DEBUG_EL_CHECK_OPERATION_TIMEOUT
      if (debug_mode) {
        debug.print(F("el_check_operation_timeout: timeout reached, aborting rotation\n"));
      }
    #endif // DEBUG_EL_CHECK_OPERATION_TIMEOUT
  }

}
#endif


// --------------------------------------------------------------



// --------------------------------------------------------------
#ifdef FEATURE_ELEVATION_CONTROL
void update_el_variable_outputs(byte speed_voltage){


  #ifdef DEBUG_LOOP
    control_port->println(F("update_el_variable_outputs()"));
    control_port->flush();
  #endif // DEBUG_LOOP   

  #ifdef DEBUG_VARIABLE_OUTPUTS
  debug.print("update_el_variable_outputs: speed_voltage: ");
  debug.print(speed_voltage);
  #endif // DEBUG_VARIABLE_OUTPUTS

  if (((el_state == SLOW_START_UP) || (el_state == NORMAL_UP) || (el_state == SLOW_DOWN_UP) || (el_state == TIMED_SLOW_DOWN_UP)) && (rotate_up_pwm)) {
    #ifdef DEBUG_VARIABLE_OUTPUTS
    debug.print("\trotate_up_pwm");
    #endif // DEBUG_VARIABLE_OUTPUTS
    analogWriteEnhanced(rotate_up_pwm, speed_voltage);
  }

  if (((el_state == SLOW_START_DOWN) || (el_state == NORMAL_DOWN) || (el_state == SLOW_DOWN_DOWN) || (el_state == TIMED_SLOW_DOWN_DOWN)) && (rotate_down_pwm)) {
    #ifdef DEBUG_VARIABLE_OUTPUTS
    debug.print("\trotate_down_pwm");
    #endif // DEBUG_VARIABLE_OUTPUTS
    analogWriteEnhanced(rotate_down_pwm, speed_voltage);
  }

  if (((el_state == SLOW_START_DOWN) || (el_state == NORMAL_DOWN) || (el_state == SLOW_DOWN_DOWN) || (el_state == TIMED_SLOW_DOWN_DOWN) ||
       (el_state == SLOW_START_UP) || (el_state == NORMAL_UP) || (el_state == SLOW_DOWN_UP) || (el_state == TIMED_SLOW_DOWN_UP)) && (rotate_up_down_pwm)) {
    #ifdef DEBUG_VARIABLE_OUTPUTS
    debug.print("\trotate_up_down_pwm");
    #endif // DEBUG_VARIABLE_OUTPUTS
    analogWriteEnhanced(rotate_up_down_pwm, speed_voltage);
  }


  if (((el_state == SLOW_START_UP) || (el_state == NORMAL_UP) || (el_state == SLOW_DOWN_UP) || (el_state == TIMED_SLOW_DOWN_UP)) && (rotate_up_freq)) {
    #ifdef DEBUG_VARIABLE_OUTPUTS
    debug.print("\trotate_up_freq");
    #endif // DEBUG_VARIABLE_OUTPUTS
    tone(rotate_up_freq, map(speed_voltage, 0, 255, EL_VARIABLE_FREQ_OUTPUT_LOW, EL_VARIABLE_FREQ_OUTPUT_HIGH));
  }

  if (((el_state == SLOW_START_DOWN) || (el_state == NORMAL_DOWN) || (el_state == SLOW_DOWN_DOWN) || (el_state == TIMED_SLOW_DOWN_DOWN)) && (rotate_down_freq)) {
    #ifdef DEBUG_VARIABLE_OUTPUTS
    debug.print("\trotate_down_freq");
    #endif // DEBUG_VARIABLE_OUTPUTS
    tone(rotate_down_freq, map(speed_voltage, 0, 255, EL_VARIABLE_FREQ_OUTPUT_LOW, EL_VARIABLE_FREQ_OUTPUT_HIGH));
  }

  #ifdef FEATURE_STEPPER_MOTOR

  unsigned int el_tone = 0;

  if (((el_state == SLOW_START_UP) || (el_state == NORMAL_UP) || (el_state == SLOW_DOWN_UP) || (el_state == TIMED_SLOW_DOWN_UP) || (el_state == SLOW_START_DOWN) || (el_state == NORMAL_DOWN) || (el_state == SLOW_DOWN_DOWN) || (el_state == TIMED_SLOW_DOWN_DOWN)) && (el_stepper_motor_pulse)) {
    #ifdef DEBUG_VARIABLE_OUTPUTS
    debug.print("\tel_stepper_motor_pulse: ");
    #endif // DEBUG_VARIABLE_OUTPUTS
    el_tone = map(speed_voltage, 0, 255, EL_VARIABLE_FREQ_OUTPUT_LOW, EL_VARIABLE_FREQ_OUTPUT_HIGH);

    #ifdef FEATURE_STEPPER_MOTOR
    set_el_stepper_freq(el_tone,7);
    #endif




    #ifdef DEBUG_VARIABLE_OUTPUTS
    debug.print(el_tone);
    #endif // DEBUG_VARIABLE_OUTPUTS

  }
  #endif //FEATURE_STEPPER_MOTOR

  if (elevation_speed_voltage) {
    analogWriteEnhanced(elevation_speed_voltage, speed_voltage);
  }

  #ifdef DEBUG_VARIABLE_OUTPUTS
  debug.println("");
  #endif // DEBUG_VARIABLE_OUTPUTS

  current_el_speed_voltage = speed_voltage;


} /* update_el_variable_outputs */
#endif // FEATURE_ELEVATION_CONTROL

// --------------------------------------------------------------
void update_az_variable_outputs(byte speed_voltage){

  #ifdef DEBUG_LOOP
    control_port->println("update_az_variable_outputs()");
    control_port->flush();
  #endif // DEBUG_LOOP   


  #ifdef DEBUG_VARIABLE_OUTPUTS  
  int temp_int = 0;

  debug.print("update_az_variable_outputs: az_state: ");
  switch (az_state) {
    case IDLE: debug.print("IDLE"); break;
    case SLOW_START_CW: debug.print("SLOW_START_CW"); break;
    case SLOW_START_CCW: debug.print("SLOW_START_CCW"); break;
    case NORMAL_CW: debug.print("NORMAL_CW"); break;
    case NORMAL_CCW: debug.print("NORMAL_CCW"); break;
    case SLOW_DOWN_CW: debug.print("SLOW_DOWN_CW"); break;
    case SLOW_DOWN_CCW: debug.print("SLOW_DOWN_CCW"); break;
    case INITIALIZE_SLOW_START_CW: debug.print("INITIALIZE_SLOW_START_CW"); break;
    case INITIALIZE_SLOW_START_CCW: debug.print("INITIALIZE_SLOW_START_CCW"); break;
    case INITIALIZE_TIMED_SLOW_DOWN_CW: debug.print("INITIALIZE_TIMED_SLOW_DOWN_CW"); break;
    case INITIALIZE_TIMED_SLOW_DOWN_CCW: debug.print("INITIALIZE_TIMED_SLOW_DOWN_CCW"); break;
    case TIMED_SLOW_DOWN_CW: debug.print("TIMED_SLOW_DOWN_CW"); break;
    case TIMED_SLOW_DOWN_CCW: debug.print("TIMED_SLOW_DOWN_CCW"); break;
    case INITIALIZE_DIR_CHANGE_TO_CW: debug.print("INITIALIZE_DIR_CHANGE_TO_CW"); break;
    case INITIALIZE_DIR_CHANGE_TO_CCW: debug.print("INITIALIZE_DIR_CHANGE_TO_CCW"); break;
    case INITIALIZE_NORMAL_CW: debug.print("INITIALIZE_NORMAL_CW"); break;
    case INITIALIZE_NORMAL_CCW: debug.print("INITIALIZE_NORMAL_CCW"); break;
    default: debug.print("UNDEF"); break;
  }
  debug.print(" speed_voltage: ");
  debug.print(speed_voltage);
  #endif // DEBUG_VARIABLE_OUTPUTS

  if (((az_state == SLOW_START_CW) || (az_state == NORMAL_CW) || (az_state == SLOW_DOWN_CW) || (az_state == TIMED_SLOW_DOWN_CW)) && (rotate_cw_pwm)) {
    #ifdef DEBUG_VARIABLE_OUTPUTS
    debug.print("\trotate_cw_pwm");
    #endif // DEBUG_VARIABLE_OUTPUTS
    analogWriteEnhanced(rotate_cw_pwm, speed_voltage);
  }

  if (((az_state == SLOW_START_CCW) || (az_state == NORMAL_CCW) || (az_state == SLOW_DOWN_CCW) || (az_state == TIMED_SLOW_DOWN_CCW)) && (rotate_ccw_pwm)) {
    #ifdef DEBUG_VARIABLE_OUTPUTS
    debug.print("\trotate_ccw_pwm");
    #endif // DEBUG_VARIABLE_OUTPUTS
    analogWriteEnhanced(rotate_ccw_pwm, speed_voltage);
  }

  if (((az_state == SLOW_START_CW) || (az_state == NORMAL_CW) || (az_state == SLOW_DOWN_CW) || (az_state == TIMED_SLOW_DOWN_CW) || (az_state == SLOW_START_CCW) || (az_state == NORMAL_CCW) || (az_state == SLOW_DOWN_CCW) || (az_state == TIMED_SLOW_DOWN_CCW)) && (rotate_cw_ccw_pwm)) {
    #ifdef DEBUG_VARIABLE_OUTPUTS
    debug.print("\trotate_cw_ccw_pwm");
    #endif // DEBUG_VARIABLE_OUTPUTS
    analogWriteEnhanced(rotate_cw_ccw_pwm, speed_voltage);
  }


  if (((az_state == SLOW_START_CW) || (az_state == NORMAL_CW) || (az_state == SLOW_DOWN_CW) || (az_state == TIMED_SLOW_DOWN_CW)) && (rotate_cw_freq)) {
    #ifdef DEBUG_VARIABLE_OUTPUTS
    debug.print("\trotate_cw_freq: ");
    temp_int = map(speed_voltage, 0, 255, AZ_VARIABLE_FREQ_OUTPUT_LOW, AZ_VARIABLE_FREQ_OUTPUT_HIGH);
    tone(rotate_cw_freq, temp_int);
    debug.print(temp_int);
    #else // DEBUG_VARIABLE_OUTPUTS
    tone(rotate_cw_freq, map(speed_voltage, 0, 255, AZ_VARIABLE_FREQ_OUTPUT_LOW, AZ_VARIABLE_FREQ_OUTPUT_HIGH));
    #endif // DEBUG_VARIABLE_OUTPUTS
  }

  if (((az_state == SLOW_START_CCW) || (az_state == NORMAL_CCW) || (az_state == SLOW_DOWN_CCW) || (az_state == TIMED_SLOW_DOWN_CCW)) && (rotate_ccw_freq)) {
    #ifdef DEBUG_VARIABLE_OUTPUTS
      debug.print("\trotate_ccw_freq: ");
      temp_int = map(speed_voltage, 0, 255, AZ_VARIABLE_FREQ_OUTPUT_LOW, AZ_VARIABLE_FREQ_OUTPUT_HIGH);
      tone(rotate_ccw_freq, temp_int);
      debug.print(temp_int);    
    #else // DEBUG_VARIABLE_OUTPUTS
      tone(rotate_ccw_freq, map(speed_voltage, 0, 255, AZ_VARIABLE_FREQ_OUTPUT_LOW, AZ_VARIABLE_FREQ_OUTPUT_HIGH));
    #endif // DEBUG_VARIABLE_OUTPUTS
  }

  #ifdef FEATURE_STEPPER_MOTOR

    unsigned int az_tone = 0;

    if (((az_state == SLOW_START_CW) || (az_state == NORMAL_CW) || (az_state == SLOW_DOWN_CW) || (az_state == TIMED_SLOW_DOWN_CW) || (az_state == SLOW_START_CCW) || (az_state == NORMAL_CCW) || (az_state == SLOW_DOWN_CCW) || (az_state == TIMED_SLOW_DOWN_CCW)) && (az_stepper_motor_pulse)) {
      #ifdef DEBUG_VARIABLE_OUTPUTS
        debug.print("\taz_stepper_motor_pulse: ");
      #endif // DEBUG_VARIABLE_OUTPUTS
      az_tone = map(speed_voltage, 0, 255, AZ_VARIABLE_FREQ_OUTPUT_LOW, AZ_VARIABLE_FREQ_OUTPUT_HIGH);
      set_az_stepper_freq(az_tone,1);
      #ifdef DEBUG_VARIABLE_OUTPUTS
        debug.print(az_tone);
      #endif // DEBUG_VARIABLE_OUTPUTS 
    }

  #endif //FEATURE_STEPPER_MOTOR

  if (azimuth_speed_voltage) {
    analogWriteEnhanced(azimuth_speed_voltage, speed_voltage);
  }

  #ifdef DEBUG_VARIABLE_OUTPUTS
  debug.println("");
  #endif // DEBUG_VARIABLE_OUTPUTS

  current_az_speed_voltage = speed_voltage;

} /* update_az_variable_outputs */

// --------------------------------------------------------------

void rotator(byte rotation_action, byte rotation_type, byte traceback) {

  #ifdef DEBUG_ROTATOR
    if (debug_mode) {
      control_port->flush();
      debug.print(F("rotator: traceback:"));
      debug.print(traceback);
      debug.print(" action:");
      debug.print(rotation_action);
      debug.print(F(" type:"));
      control_port->flush();
      debug.print(rotation_type);
      debug.print(F("->"));
      control_port->flush();
    }
  #endif // DEBUG_ROTATOR

  switch (rotation_type) {
    case CW:
      #ifdef DEBUG_ROTATOR
        if (debug_mode) {
          debug.print(F("CW ")); control_port->flush();
        }
      #endif // DEBUG_ROTATOR
      if (rotation_action == ACTIVATE) {
        #ifdef DEBUG_ROTATOR
          if (debug_mode) {
            debug.print(F("ACTIVATE"));
          }
        #endif // DEBUG_ROTATOR
        #if defined(FEATURE_MASTER_SEND_AZ_ROTATION_COMMANDS_TO_REMOTE)
          submit_remote_command(REMOTE_UNIT_RR_COMMAND,0,0);
//zzzzzzzz
        #else
          brake_release(AZ, BRAKE_RELEASE_ON);
          if (az_slowstart_active) {
            if (rotate_cw_pwm) {
              analogWriteEnhanced(rotate_cw_pwm, 0);
            }
            if (rotate_ccw_pwm) {
              analogWriteEnhanced(rotate_ccw_pwm, 0); digitalWriteEnhanced(rotate_ccw_pwm, LOW);
            }
            if (rotate_cw_ccw_pwm) {
              analogWriteEnhanced(rotate_cw_ccw_pwm, 0);
            }
            if (rotate_cw_freq) {
              noTone(rotate_cw_freq);
            }
            if (rotate_ccw_freq) {
              noTone(rotate_ccw_freq);
            }       
            #ifdef FEATURE_STEPPER_MOTOR
            if (az_stepper_motor_pulse) {
              set_az_stepper_freq(0,2);
              digitalWriteEnhanced(az_stepper_motor_pulse,LOW);
            }      
            #endif //FEATURE_STEPPER_MOTOR
          } else {
            if (rotate_cw_pwm) {
              analogWriteEnhanced(rotate_cw_pwm, normal_az_speed_voltage);
            }
            if (rotate_ccw_pwm) {
              analogWriteEnhanced(rotate_ccw_pwm, 0); digitalWriteEnhanced(rotate_ccw_pwm, LOW);
            }
            if (rotate_cw_ccw_pwm) {
              analogWriteEnhanced(rotate_cw_ccw_pwm, normal_az_speed_voltage);
            }
            if (rotate_cw_freq) {
              tone(rotate_cw_freq, map(normal_az_speed_voltage, 0, 255, AZ_VARIABLE_FREQ_OUTPUT_LOW, AZ_VARIABLE_FREQ_OUTPUT_HIGH));
            }
            if (rotate_ccw_freq) {
              noTone(rotate_ccw_freq);
            }  
            #ifdef FEATURE_STEPPER_MOTOR
              if (az_stepper_motor_pulse) {
                set_az_stepper_freq(map(normal_az_speed_voltage, 0, 255, AZ_VARIABLE_FREQ_OUTPUT_LOW, AZ_VARIABLE_FREQ_OUTPUT_HIGH),3);
              }
            #endif //FEATURE_STEPPER_MOTOR                 
          }
          if (rotate_cw) {
            digitalWriteEnhanced(rotate_cw, ROTATE_PIN_AZ_ACTIVE_VALUE);
            #if defined(pin_led_cw)
              digitalWriteEnhanced(pin_led_cw, PIN_LED_ACTIVE_STATE);
            #endif
          }
          if (rotate_ccw) {
            digitalWriteEnhanced(rotate_ccw, ROTATE_PIN_AZ_INACTIVE_VALUE);
            #if defined(pin_led_ccw)
              digitalWriteEnhanced(pin_led_ccw, PIN_LED_INACTIVE_STATE);
            #endif          
          }
          if (rotate_cw_ccw){
            digitalWriteEnhanced(rotate_cw_ccw, ROTATE_PIN_AZ_ACTIVE_VALUE);
          }
          #ifdef DEBUG_ROTATOR
            if (debug_mode) {
              debug.print(F(" normal_az_speed_voltage:"));
              control_port->println(normal_az_speed_voltage);
              //control_port->flush();
            }
          #endif // DEBUG_ROTATOR

      #endif //defined(FEATURE_MASTER_SEND_AZ_ROTATION_COMMANDS_TO_REMOTE)

      } else {  // DEACTIVATE
        #ifdef DEBUG_ROTATOR
          if (debug_mode) {
            debug.print(F("DEACTIVATE"));
          }
        #endif // DEBUG_ROTATOR
        #if defined(FEATURE_MASTER_SEND_AZ_ROTATION_COMMANDS_TO_REMOTE)
          submit_remote_command(REMOTE_UNIT_RA_COMMAND,0,0);
        #else
          if (rotate_cw_pwm) {
            analogWriteEnhanced(rotate_cw_pwm, 0); digitalWriteEnhanced(rotate_cw_pwm, LOW);
          }
          if (rotate_cw_ccw_pwm) {
            analogWriteEnhanced(rotate_cw_ccw_pwm, 0);
          }
          if (rotate_cw) {
            digitalWriteEnhanced(rotate_cw, ROTATE_PIN_AZ_INACTIVE_VALUE);
            #if defined(pin_led_cw)
              digitalWriteEnhanced(pin_led_cw, PIN_LED_INACTIVE_STATE);
            #endif          
          }
          if (rotate_cw_ccw){
            digitalWriteEnhanced(rotate_cw_ccw, ROTATE_PIN_AZ_INACTIVE_VALUE);
          }        
          if (rotate_cw_freq) {
            noTone(rotate_cw_freq);
          }

          #ifdef FEATURE_STEPPER_MOTOR
          if (az_stepper_motor_pulse) {
            set_az_stepper_freq(0,4);
            digitalWriteEnhanced(az_stepper_motor_pulse,HIGH);
          }      
          #endif //FEATURE_STEPPER_MOTOR 
        #endif //#if defined(FEATURE_MASTER_SEND_AZ_ROTATION_COMMANDS_TO_REMOTE)              
      }
      break;
    case CCW:
      #ifdef DEBUG_ROTATOR
        if (debug_mode) {
          debug.print(F("CCW ")); control_port->flush();
        }
      #endif // DEBUG_ROTATOR
      if (rotation_action == ACTIVATE) {
        #ifdef DEBUG_ROTATOR
          if (debug_mode) {
            debug.print(F("ACTIVATE"));
          }
        #endif // DEBUG_ROTATOR
        #if defined(FEATURE_MASTER_SEND_AZ_ROTATION_COMMANDS_TO_REMOTE)
          submit_remote_command(REMOTE_UNIT_RL_COMMAND,0,0);
        #else
          brake_release(AZ, BRAKE_RELEASE_ON);
          if (az_slowstart_active) {
            if (rotate_cw_pwm) {
              analogWriteEnhanced(rotate_cw_pwm, 0); digitalWriteEnhanced(rotate_cw_pwm, LOW);
            }
            if (rotate_ccw_pwm) {
              analogWriteEnhanced(rotate_ccw_pwm, 0);
            }
            if (rotate_cw_ccw_pwm) {
              analogWriteEnhanced(rotate_cw_ccw_pwm, 0);
            }
            if (rotate_cw_freq) {
              noTone(rotate_cw_freq);
            }
            if (rotate_ccw_freq) {
              noTone(rotate_ccw_freq);
            } 
            #ifdef FEATURE_STEPPER_MOTOR
            if (az_stepper_motor_pulse) {
              set_az_stepper_freq(0,5);
              digitalWriteEnhanced(az_stepper_motor_pulse,LOW);
            }      
            #endif //FEATURE_STEPPER_MOTOR                
          } else {
            if (rotate_cw_pwm) {
              analogWriteEnhanced(rotate_cw_pwm, 0); digitalWriteEnhanced(rotate_cw_pwm, LOW);
            }
            if (rotate_ccw_pwm) {
              analogWriteEnhanced(rotate_ccw_pwm, normal_az_speed_voltage);
            }
            if (rotate_cw_ccw_pwm) {
              analogWriteEnhanced(rotate_cw_ccw_pwm, normal_az_speed_voltage);
            }
            if (rotate_cw_freq) {
              noTone(rotate_cw_freq);
            }
            if (rotate_ccw_freq) {
              tone(rotate_ccw_freq, map(normal_az_speed_voltage, 0, 255, AZ_VARIABLE_FREQ_OUTPUT_LOW, AZ_VARIABLE_FREQ_OUTPUT_HIGH));
            }  
            #ifdef FEATURE_STEPPER_MOTOR
            if (az_stepper_motor_pulse) {
              set_az_stepper_freq(map(normal_az_speed_voltage, 0, 255, AZ_VARIABLE_FREQ_OUTPUT_LOW, AZ_VARIABLE_FREQ_OUTPUT_HIGH),6);
            }
            #endif //FEATURE_STEPPER_MOTOR 
          }
          if (rotate_cw) {
            digitalWriteEnhanced(rotate_cw, ROTATE_PIN_AZ_INACTIVE_VALUE);
            #if defined(pin_led_cw)
              digitalWriteEnhanced(pin_led_cw, PIN_LED_INACTIVE_STATE);
            #endif          
          }
          if (rotate_ccw) {
            digitalWriteEnhanced(rotate_ccw, ROTATE_PIN_AZ_ACTIVE_VALUE);
            #if defined(pin_led_ccw)
              digitalWriteEnhanced(pin_led_ccw, PIN_LED_ACTIVE_STATE);
            #endif          
          }
          if (rotate_cw_ccw){
            digitalWriteEnhanced(rotate_cw_ccw, ROTATE_PIN_AZ_ACTIVE_VALUE);
          }      
          #ifdef DEBUG_ROTATOR
          if (debug_mode) {
            debug.print(F(" normal_az_speed_voltage:"));
            control_port->println(normal_az_speed_voltage);
            control_port->flush();
          }
          #endif // DEBUG_ROTATOR
        #endif //#if defined(FEATURE_MASTER_SEND_AZ_ROTATION_COMMANDS_TO_REMOTE)  
      } else { // DEACTIVATE
        #ifdef DEBUG_ROTATOR
          if (debug_mode) {
            debug.print(F("DEACTIVATE"));
          }
        #endif // DEBUG_ROTATOR
        #if defined(FEATURE_MASTER_SEND_AZ_ROTATION_COMMANDS_TO_REMOTE)
          submit_remote_command(REMOTE_UNIT_RA_COMMAND,0,0);
        #else
          if (rotate_ccw_pwm) {
            analogWriteEnhanced(rotate_ccw_pwm, 0); digitalWriteEnhanced(rotate_ccw_pwm, LOW);
          }
          if (rotate_ccw) {
            digitalWriteEnhanced(rotate_ccw, ROTATE_PIN_AZ_INACTIVE_VALUE);
            #if defined(pin_led_ccw)
              digitalWriteEnhanced(pin_led_ccw, PIN_LED_INACTIVE_STATE);
            #endif          
          }
          if (rotate_ccw_freq) {
            noTone(rotate_ccw_freq);
          }
        #endif
      }
      break;

    #ifdef FEATURE_ELEVATION_CONTROL
      case UP:
        #ifdef DEBUG_ROTATOR
          if (debug_mode) {
            debug.print(F("ROTATION_UP ")); 
          }
        #endif // DEBUG_ROTATOR
        if (rotation_action == ACTIVATE) {
          #ifdef DEBUG_ROTATOR
            if (debug_mode) {
              debug.print(F("ACTIVATE")); 
            }
          #endif // DEBUG_ROTATOR
          #if defined(FEATURE_MASTER_SEND_EL_ROTATION_COMMANDS_TO_REMOTE)
            submit_remote_command(REMOTE_UNIT_RU_COMMAND,0,0);
          #else
            brake_release(EL, BRAKE_RELEASE_ON);
            if (el_slowstart_active) {
              if (rotate_up_pwm) {
                analogWriteEnhanced(rotate_up_pwm, 0);
              }
              if (rotate_down_pwm) {
                analogWriteEnhanced(rotate_down_pwm, 0); digitalWriteEnhanced(rotate_down_pwm, LOW);
              }
              if (rotate_up_down_pwm) {
                analogWriteEnhanced(rotate_up_down_pwm, 0);
              }
              if (rotate_up_freq) {
                noTone(rotate_up_freq);
              }
              if (rotate_down_freq) {
                noTone(rotate_down_freq);
              }
              #ifdef FEATURE_STEPPER_MOTOR
              if (el_stepper_motor_pulse) {
                set_el_stepper_freq(0,1);
                digitalWriteEnhanced(el_stepper_motor_pulse,LOW);
              }      
              #endif //FEATURE_STEPPER_MOTOR    
            } else {
              if (rotate_up_pwm) {
                analogWriteEnhanced(rotate_up_pwm, normal_el_speed_voltage);
              }
              if (rotate_down_pwm) {
                analogWriteEnhanced(rotate_down_pwm, 0); digitalWriteEnhanced(rotate_down_pwm, LOW);
              }
              if (rotate_up_down_pwm) {
                analogWriteEnhanced(rotate_up_down_pwm, normal_el_speed_voltage);
              }
              if (rotate_up_freq) {
                tone(rotate_up_freq, map(normal_el_speed_voltage, 0, 255, EL_VARIABLE_FREQ_OUTPUT_LOW, EL_VARIABLE_FREQ_OUTPUT_HIGH));
              }
              #ifdef FEATURE_STEPPER_MOTOR
              if (el_stepper_motor_pulse) {
                set_el_stepper_freq(map(normal_el_speed_voltage, 0, 255, EL_VARIABLE_FREQ_OUTPUT_LOW, EL_VARIABLE_FREQ_OUTPUT_HIGH),2);
              }
              #endif //FEATURE_STEPPER_MOTOR  
              if (rotate_down_freq) {
                noTone(rotate_down_freq);
              }
            }
            if (rotate_up) {
              digitalWriteEnhanced(rotate_up, ROTATE_PIN_EL_ACTIVE_VALUE);
              #if defined(pin_led_up)
                digitalWriteEnhanced(pin_led_up, PIN_LED_ACTIVE_STATE);
              #endif          
            }
            if (rotate_down) {
              digitalWriteEnhanced(rotate_down, ROTATE_PIN_EL_INACTIVE_VALUE);
              #if defined(pin_led_down)
                digitalWriteEnhanced(pin_led_down, PIN_LED_INACTIVE_STATE);
              #endif           
            }
            if (rotate_up_or_down) {
              digitalWriteEnhanced(rotate_up_or_down, ROTATE_PIN_EL_ACTIVE_VALUE);
            }      
            #endif //#if defined(FEATURE_MASTER_SEND_EL_ROTATION_COMMANDS_TO_REMOTE)  
        } else { // DEACTIVATE
          #ifdef DEBUG_ROTATOR
            if (debug_mode) {
              debug.print(F("DEACTIVATE"));
            }
          #endif // DEBUG_ROTATOR
          #if defined(FEATURE_MASTER_SEND_EL_ROTATION_COMMANDS_TO_REMOTE)
            submit_remote_command(REMOTE_UNIT_RE_COMMAND,0,0);
          #else
            if (rotate_up) {
              digitalWriteEnhanced(rotate_up, ROTATE_PIN_EL_INACTIVE_VALUE);
              #if defined(pin_led_up)
                digitalWriteEnhanced(pin_led_up, PIN_LED_INACTIVE_STATE);
              #endif            
            }
            if (rotate_up_pwm) {
              analogWriteEnhanced(rotate_up_pwm, 0); digitalWriteEnhanced(rotate_up_pwm, LOW);
            }
            if (rotate_up_down_pwm) {
              analogWriteEnhanced(rotate_up_down_pwm, 0);
            }
            if (rotate_up_freq) {
              noTone(rotate_up_freq);
            }
            if (rotate_up_or_down) {
              digitalWriteEnhanced(rotate_up_or_down, ROTATE_PIN_EL_INACTIVE_VALUE);
            }   
            #ifdef FEATURE_STEPPER_MOTOR
            if (el_stepper_motor_pulse) {
              set_el_stepper_freq(0,3);
              digitalWriteEnhanced(el_stepper_motor_pulse,HIGH);
            }      
            #endif //FEATURE_STEPPER_MOTOR   
          #endif //#if defined(FEATURE_MASTER_SEND_EL_ROTATION_COMMANDS_TO_REMOTE)
        }
        break;

      case DOWN:
        #ifdef DEBUG_ROTATOR
          if (debug_mode) {
            debug.print(F("ROTATION_DOWN "));
          }
        #endif // DEBUG_ROTATOR
        if (rotation_action == ACTIVATE) {
          #ifdef DEBUG_ROTATOR
            if (debug_mode) {
              debug.print(F("ACTIVATE"));
            }
          #endif // DEBUG_ROTATOR
          #if defined(FEATURE_MASTER_SEND_EL_ROTATION_COMMANDS_TO_REMOTE)
            submit_remote_command(REMOTE_UNIT_RD_COMMAND,0,0);
          #else
            brake_release(EL, BRAKE_RELEASE_ON);
            if (el_slowstart_active) {
              if (rotate_down_pwm) {
                analogWriteEnhanced(rotate_down_pwm, 0);
              }
              if (rotate_up_pwm) {
                analogWriteEnhanced(rotate_up_pwm, 0); digitalWriteEnhanced(rotate_up_pwm, LOW);
              }
              if (rotate_up_down_pwm) {
                analogWriteEnhanced(rotate_up_down_pwm, 0);
              }
              if (rotate_up_freq) {
                noTone(rotate_up_freq);
              }
              if (rotate_down_freq) {
                noTone(rotate_down_freq);
              }
              #ifdef FEATURE_STEPPER_MOTOR
              if (el_stepper_motor_pulse) {
                set_el_stepper_freq(0,4);
                digitalWriteEnhanced(el_stepper_motor_pulse,LOW);        
              }      
              #endif //FEATURE_STEPPER_MOTOR             
            } else {
              if (rotate_down_pwm) {
                analogWriteEnhanced(rotate_down_pwm, normal_el_speed_voltage);
              }
              if (rotate_up_pwm) {
                analogWriteEnhanced(rotate_up_pwm, 0); digitalWriteEnhanced(rotate_up_pwm, LOW);
              }
              if (rotate_up_down_pwm) {
                analogWriteEnhanced(rotate_up_down_pwm, normal_el_speed_voltage);
              }
              if (rotate_down_freq) {
                tone(rotate_down_freq, map(normal_el_speed_voltage, 0, 255, EL_VARIABLE_FREQ_OUTPUT_LOW, EL_VARIABLE_FREQ_OUTPUT_HIGH));
              }
              if (rotate_up_freq) {
                noTone(rotate_up_freq);
              }
              #ifdef FEATURE_STEPPER_MOTOR
              if (el_stepper_motor_pulse) {
                set_el_stepper_freq(map(normal_el_speed_voltage, 0, 255, EL_VARIABLE_FREQ_OUTPUT_LOW, EL_VARIABLE_FREQ_OUTPUT_HIGH),5);
                digitalWriteEnhanced(el_stepper_motor_pulse,LOW);           
              }      
              #endif //FEATURE_STEPPER_MOTOR             
            }
            if (rotate_up) {
              digitalWriteEnhanced(rotate_up, ROTATE_PIN_EL_INACTIVE_VALUE);
              #if defined(pin_led_up)
                digitalWriteEnhanced(pin_led_up, PIN_LED_INACTIVE_STATE);
              #endif            
            }
            if (rotate_down) {
              digitalWriteEnhanced(rotate_down, ROTATE_PIN_EL_ACTIVE_VALUE);
              #if defined(pin_led_down)
                digitalWriteEnhanced(pin_led_down, PIN_LED_ACTIVE_STATE);
              #endif            
            }
            if (rotate_up_or_down) {
              digitalWriteEnhanced(rotate_up_or_down, ROTATE_PIN_EL_ACTIVE_VALUE);
            }       
          #endif //#if defined(FEATURE_MASTER_SEND_EL_ROTATION_COMMANDS_TO_REMOTE)  
        } else {
          #ifdef DEBUG_ROTATOR
            if (debug_mode) {
              debug.print(F("DEACTIVATE"));
            }
          #endif // DEBUG_ROTATOR
          #if defined(FEATURE_MASTER_SEND_EL_ROTATION_COMMANDS_TO_REMOTE)
            submit_remote_command(REMOTE_UNIT_RE_COMMAND,0,0);
          #else
            if (rotate_down) {
              digitalWriteEnhanced(rotate_down, ROTATE_PIN_EL_INACTIVE_VALUE);
              #if defined(pin_led_down)
                digitalWriteEnhanced(pin_led_down, PIN_LED_INACTIVE_STATE);
              #endif           
            }
            if (rotate_down_pwm) {
              analogWriteEnhanced(rotate_down_pwm, 0); digitalWriteEnhanced(rotate_down_pwm, LOW);
            }
            if (rotate_up_down_pwm) {
              analogWriteEnhanced(rotate_up_down_pwm, 0);
            }
            if (rotate_down_freq) {
              noTone(rotate_down_freq);
            }
            if (rotate_up_or_down) {
              digitalWriteEnhanced(rotate_up_or_down, ROTATE_PIN_EL_INACTIVE_VALUE);
            }        
            #ifdef FEATURE_STEPPER_MOTOR
              if (el_stepper_motor_pulse) {
                set_el_stepper_freq(0,6);
                digitalWriteEnhanced(el_stepper_motor_pulse,HIGH);          
              }      
            #endif //FEATURE_STEPPER_MOTOR 
          #endif //#if defined(FEATURE_MASTER_SEND_EL_ROTATION_COMMANDS_TO_REMOTE)  
        }
        break;
      #endif // FEATURE_ELEVATION_CONTROL
  } /* switch */

  #ifdef DEBUG_ROTATOR
  if (debug_mode) {
    debug.print(F("\r\n"));
    control_port->flush();
  }
  #endif // DEBUG_ROTATOR
} /* rotator */

// --------------------------------------------------------------
void initialize_interrupts(){

  #ifdef DEBUG_LOOP
    control_port->println("initialize_interrupts()");
    control_port->flush();
  #endif // DEBUG_LOOP

  #ifdef FEATURE_AZ_POSITION_PULSE_INPUT
    attachInterrupt(digitalPinToInterrupt(az_position_pulse_pin), az_position_pulse_interrupt_handler, FALLING);
  #endif // FEATURE_AZ_POSITION_PULSE_INPUT

  #ifdef FEATURE_EL_POSITION_PULSE_INPUT
    attachInterrupt(digitalPinToInterrupt(el_position_pulse_pin), el_position_pulse_interrupt_handler, FALLING);
  #endif // FEATURE_EL_POSITION_PULSE_INPUT


}

// --------------------------------------------------------------

void initialize_pins(){

  #ifdef DEBUG_LOOP
    control_port->println("initialize_pins()");
    control_port->flush();
  #endif // DEBUG_LOOP

  #ifdef reset_pin
  pinMode(reset_pin, OUTPUT);
  digitalWrite(reset_pin, LOW);
  #endif //reset_pin

  if (serial_led) {
    pinModeEnhanced(serial_led, OUTPUT);
  }

  if (overlap_led) {
    pinModeEnhanced(overlap_led, OUTPUT);
  }

  if (brake_az) {
    pinModeEnhanced(brake_az, OUTPUT);
    digitalWriteEnhanced(brake_az, BRAKE_INACTIVE_STATE);
  }

  if (az_speed_pot) {
    pinModeEnhanced(az_speed_pot, INPUT);
    digitalWriteEnhanced(az_speed_pot, LOW);
  }

  if (az_preset_pot) {
    pinModeEnhanced(az_preset_pot, INPUT);
    digitalWriteEnhanced(az_preset_pot, LOW);
  }

  if (preset_start_button) {
    pinModeEnhanced(preset_start_button, INPUT);
    digitalWriteEnhanced(preset_start_button, HIGH);
  }

  if (button_stop) {
    pinModeEnhanced(button_stop, INPUT);
    digitalWriteEnhanced(button_stop, HIGH);
  }

  #ifdef FEATURE_ELEVATION_CONTROL
  if (brake_el) {
    pinModeEnhanced(brake_el, OUTPUT);
    digitalWriteEnhanced(brake_el, BRAKE_INACTIVE_STATE);
  }
  #endif // FEATURE_ELEVATION_CONTROL

  if (rotate_cw) {
    pinModeEnhanced(rotate_cw, OUTPUT);
  }
  if (rotate_ccw) {
    pinModeEnhanced(rotate_ccw, OUTPUT);
  }
  if (rotate_cw_pwm) {
    pinModeEnhanced(rotate_cw_pwm, OUTPUT);
  }
  if (rotate_ccw_pwm) {
    pinModeEnhanced(rotate_ccw_pwm, OUTPUT);
  }
  if (rotate_cw_ccw_pwm) {
    pinModeEnhanced(rotate_cw_ccw_pwm, OUTPUT);
  }
  if (rotate_cw_freq) {
    pinModeEnhanced(rotate_cw_freq, OUTPUT);
  }
  if (rotate_ccw_freq) {
    pinModeEnhanced(rotate_ccw_freq, OUTPUT);
  }

  if (rotate_cw_ccw) {
    pinModeEnhanced(rotate_cw_ccw, OUTPUT);
  }

  #if defined(pin_led_cw)
    pinModeEnhanced(pin_led_cw, OUTPUT);
    digitalWriteEnhanced(pin_led_cw, PIN_LED_INACTIVE_STATE);
  #endif

  #if defined(pin_led_ccw)
    pinModeEnhanced(pin_led_ccw, OUTPUT);
    digitalWriteEnhanced(pin_led_ccw, PIN_LED_INACTIVE_STATE);
  #endif

  #if defined(pin_led_up)
    pinModeEnhanced(pin_led_up, OUTPUT);
    digitalWriteEnhanced(pin_led_up, PIN_LED_INACTIVE_STATE);
  #endif

  #if defined(pin_led_down)
    pinModeEnhanced(pin_led_down, OUTPUT);
    digitalWriteEnhanced(pin_led_down, PIN_LED_INACTIVE_STATE);
  #endif

  rotator(DEACTIVATE, CW, 1);
  rotator(DEACTIVATE, CCW, 1);

  #if defined(FEATURE_AZ_POSITION_POTENTIOMETER)
    pinModeEnhanced(rotator_analog_az, INPUT);
  #endif

  if (button_cw) {
    pinModeEnhanced(button_cw, INPUT);
    digitalWriteEnhanced(button_cw, HIGH);
  }
  if (button_ccw) {
    pinModeEnhanced(button_ccw, INPUT);
    digitalWriteEnhanced(button_ccw, HIGH);
  }

  normal_az_speed_voltage = PWM_SPEED_VOLTAGE_X4;
  current_az_speed_voltage = PWM_SPEED_VOLTAGE_X4;

  #ifdef FEATURE_ELEVATION_CONTROL
  normal_el_speed_voltage = PWM_SPEED_VOLTAGE_X4;
  current_el_speed_voltage = PWM_SPEED_VOLTAGE_X4;
  #endif // FEATURE_ELEVATION_CONTROL

  if (azimuth_speed_voltage) {                 // if azimuth_speed_voltage pin is configured, set it up for PWM output
    analogWriteEnhanced(azimuth_speed_voltage, PWM_SPEED_VOLTAGE_X4);
  }


  #ifdef FEATURE_ELEVATION_CONTROL
  pinModeEnhanced(rotate_up, OUTPUT);
  pinModeEnhanced(rotate_down, OUTPUT);
  if (rotate_up_or_down) {
    pinModeEnhanced(rotate_up_or_down, OUTPUT);
  }
  if (rotate_up_pwm) {
    pinModeEnhanced(rotate_up_pwm, OUTPUT);
  }
  if (rotate_down_pwm) {
    pinModeEnhanced(rotate_down_pwm, OUTPUT);
  }
  if (rotate_up_down_pwm) {
    pinModeEnhanced(rotate_up_down_pwm, OUTPUT);
  }
  if (rotate_up_freq) {
    pinModeEnhanced(rotate_up_freq, OUTPUT);
  }
  if (rotate_down_freq) {
    pinModeEnhanced(rotate_down_freq, OUTPUT);
  }
  rotator(DEACTIVATE, UP, 2);
  rotator(DEACTIVATE, DOWN, 2);
  #ifdef FEATURE_EL_POSITION_POTENTIOMETER
  pinModeEnhanced(rotator_analog_el, INPUT);
  #endif // FEATURE_EL_POSITION_POTENTIOMETER
  if (button_up) {
    pinModeEnhanced(button_up, INPUT);
    digitalWriteEnhanced(button_up, HIGH);
  }
  if (button_down) {
    pinModeEnhanced(button_down, INPUT);
    digitalWriteEnhanced(button_down, HIGH);
  }

  if (elevation_speed_voltage) {                 // if elevation_speed_voltage pin is configured, set it up for PWM output
    analogWriteEnhanced(elevation_speed_voltage, PWM_SPEED_VOLTAGE_X4);
    normal_el_speed_voltage = PWM_SPEED_VOLTAGE_X4;
    current_el_speed_voltage = PWM_SPEED_VOLTAGE_X4;
  }

  read_elevation(0);
  #endif // FEATURE_ELEVATION_CONTROL

  #ifdef FEATURE_AZ_POSITION_PULSE_INPUT
    if (az_position_pulse_pin) {
      #ifdef OPTION_POSITION_PULSE_INPUT_PULLUPS
        pinModeEnhanced(az_position_pulse_pin, INPUT_PULLUP);
      #else
        pinModeEnhanced(az_position_pulse_pin, INPUT);
      #endif // OPTION_POSITION_PULSE_INPUT_PULLUPS
    }
  #endif // FEATURE_AZ_POSITION_PULSE_INPUT


  #ifdef FEATURE_EL_POSITION_PULSE_INPUT
    if (el_position_pulse_pin) {
      pinModeEnhanced(el_position_pulse_pin, INPUT);
      #ifdef OPTION_POSITION_PULSE_INPUT_PULLUPS
      digitalWriteEnhanced(el_position_pulse_pin, HIGH);
      #endif // OPTION_POSITION_PULSE_INPUT_PULLUPS
    }
  #endif // FEATURE_EL_POSITION_PULSE_INPUT

  #ifdef FEATURE_PARK
    if (button_park) {
      pinModeEnhanced(button_park, INPUT);
      digitalWriteEnhanced(button_park, HIGH);
    }
  #endif // FEATURE_PARK

  #ifdef FEATURE_ROTATION_INDICATOR_PIN
  if (rotation_indication_pin) {
    pinModeEnhanced(rotation_indication_pin, OUTPUT);
    digitalWriteEnhanced(rotation_indication_pin, ROTATION_INDICATOR_PIN_INACTIVE_STATE);
  }
  #endif // FEATURE_ROTATION_INDICATOR_PIN

  #ifdef FEATURE_PARK
  if (park_in_progress_pin) {
    pinModeEnhanced(park_in_progress_pin, OUTPUT);
    digitalWriteEnhanced(park_in_progress_pin, LOW);
  }
  if (parked_pin) {
    pinModeEnhanced(parked_pin, OUTPUT);
    digitalWriteEnhanced(parked_pin, LOW);
  }
    #ifdef FEATURE_AUTOPARK
      if (pin_autopark_disable) {
        pinModeEnhanced(pin_autopark_disable, INPUT);
        digitalWriteEnhanced(pin_autopark_disable, HIGH);
      }  
      if (pin_autopark_timer_reset) {
        pinModeEnhanced(pin_autopark_timer_reset, INPUT);
        digitalWriteEnhanced(pin_autopark_timer_reset, HIGH);
      }  
    #endif //FEATURE_AUTOPARK
  #endif // FEATURE_PARK

  if (blink_led) {
    pinModeEnhanced(blink_led, OUTPUT);
  }

  if (heading_reading_inhibit_pin) {
    pinModeEnhanced(heading_reading_inhibit_pin, INPUT);
  }

  #ifdef FEATURE_LIMIT_SENSE
  if (az_limit_sense_pin) {
    pinModeEnhanced(az_limit_sense_pin, INPUT);
    digitalWriteEnhanced(az_limit_sense_pin, HIGH);
  }
  #ifdef FEATURE_ELEVATION_CONTROL
  if (el_limit_sense_pin) {
    pinModeEnhanced(el_limit_sense_pin, INPUT);
    digitalWriteEnhanced(el_limit_sense_pin, HIGH);
  }
  #endif // FEATURE_ELEVATION_CONTROL
  #endif // FEATURE_LIMIT_SENSE

  #ifdef FEATURE_MOON_TRACKING
  if (moon_tracking_active_pin) {
    pinModeEnhanced(moon_tracking_active_pin, OUTPUT);
    digitalWriteEnhanced(moon_tracking_active_pin, LOW);
  }
  if (moon_tracking_activate_line) {
    pinModeEnhanced(moon_tracking_activate_line, INPUT);
    digitalWriteEnhanced(moon_tracking_activate_line, HIGH);
  }
  if (moon_tracking_button) {
    pinModeEnhanced(moon_tracking_button, INPUT);
    digitalWriteEnhanced(moon_tracking_button, HIGH);
  }
  #endif // FEATURE_MOON_TRACKING


  #ifdef FEATURE_SUN_TRACKING
  if (sun_tracking_active_pin) {
    pinModeEnhanced(sun_tracking_active_pin, OUTPUT);
    digitalWriteEnhanced(sun_tracking_active_pin, LOW);
  }
  if (sun_tracking_activate_line) {
    pinModeEnhanced(sun_tracking_activate_line, INPUT);
    digitalWriteEnhanced(sun_tracking_activate_line, HIGH);
  }
  if (sun_tracking_button) {
    pinModeEnhanced(sun_tracking_button, INPUT);
    digitalWriteEnhanced(sun_tracking_button, HIGH);
  }
  #endif // FEATURE_SUN_TRACKING
  
  
  #ifdef FEATURE_GPS
  if (gps_sync) {
    pinModeEnhanced(gps_sync, OUTPUT);
    digitalWriteEnhanced(gps_sync, LOW);
  }
  #endif //FEATURE_GPS

  #ifdef FEATURE_POWER_SWITCH
  pinModeEnhanced(power_switch, OUTPUT);
  digitalWriteEnhanced(power_switch, HIGH);
  #endif //FEATURE_POWER_SWITCH

  #ifdef FEATURE_EL_POSITION_MEMSIC_2125
    pinModeEnhanced(pin_memsic_2125_x, INPUT);
    pinModeEnhanced(pin_memsic_2125_y, INPUT);
  #endif //FEATURE_EL_POSITION_MEMSIC_2125

  #ifdef FEATURE_ANALOG_OUTPUT_PINS
    pinModeEnhanced(pin_analog_az_out, OUTPUT);
    digitalWriteEnhanced(pin_analog_az_out, LOW);
    #ifdef FEATURE_ELEVATION_CONTROL
      pinModeEnhanced(pin_analog_el_out, OUTPUT);
      digitalWriteEnhanced(pin_analog_el_out, LOW);
    #endif //FEATURE_ELEVATION_CONTROL
  #endif //FEATURE_ANALOG_OUTPUT_PINS

  #ifdef FEATURE_SUN_PUSHBUTTON_AZ_EL_CALIBRATION
    pinModeEnhanced(pin_sun_pushbutton_calibration, INPUT);
    digitalWriteEnhanced(pin_sun_pushbutton_calibration, HIGH);
  #endif //FEATURE_SUN_PUSHBUTTON_AZ_EL_CALIBRATION

  #ifdef FEATURE_MOON_PUSHBUTTON_AZ_EL_CALIBRATION
    pinModeEnhanced(pin_moon_pushbutton_calibration, INPUT);
    digitalWriteEnhanced(pin_moon_pushbutton_calibration, HIGH);
  #endif //FEATURE_MOON_PUSHBUTTON_AZ_EL_CALIBRATION

  #if defined(FEATURE_AUDIBLE_ALERT)
    pinModeEnhanced(pin_audible_alert, OUTPUT);
    digitalWriteEnhanced(pin_audible_alert, AUDIBLE_PIN_INACTIVE_STATE);
    if (AUDIBLE_ALERT_AT_STARTUP){
      audible_alert(AUDIBLE_ALERT_ACTIVATE);
    }
  #endif //FEATURE_AUDIBLE_ALERT

  #ifdef FEATURE_AZ_ROTATION_STALL_DETECTION
    pinModeEnhanced(az_rotation_stall_detected, OUTPUT);
    digitalWriteEnhanced(az_rotation_stall_detected, LOW);
  #endif //FEATURE_AZ_ROTATION_STALL_DETECTION

  #ifdef FEATURE_EL_ROTATION_STALL_DETECTION
    pinModeEnhanced(el_rotation_stall_detected, OUTPUT);
    digitalWriteEnhanced(el_rotation_stall_detected, LOW);
  #endif //FEATURE_EL_ROTATION_STALL_DETECTION  

  #ifdef pin_status_led
    if (pin_status_led){
      pinModeEnhanced(pin_status_led, OUTPUT);
      digitalWriteEnhanced(pin_status_led, HIGH);
    }
  #endif 

} /* initialize_pins */

// --------------------------------------------------------------

void initialize_serial(){

  #if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION) || defined(FEATURE_CLOCK) || defined(UNDER_DEVELOPMENT_REMOTE_UNIT_COMMANDS)
    control_port = CONTROL_PORT_MAPPED_TO;
    control_port->begin(CONTROL_PORT_BAUD_RATE);
    #if defined(OPTION_SEND_STRING_OUT_CONTROL_PORT_WHEN_INITIALIZING)
      control_port->print OPTION_SEND_STRING_OUT_CONTROL_PORT_WHEN_INITIALIZING_STRING;
    #endif

    #ifdef DEBUG_LOOP
      control_port->println("control_port up");
      control_port->flush();
    #endif // DEBUG_LOOP


  #endif

  #ifdef FEATURE_REMOTE_UNIT_SLAVE
    control_port->print(F("CS"));
    control_port->println(CODE_VERSION);
  #endif // FEATURE_REMOTE_UNIT_SLAVE

  #if defined(FEATURE_MASTER_WITH_SERIAL_SLAVE)
    //remote_unit_port = REMOTE_PORT_MAPPED_TO;
    REMOTE_PORT.begin(REMOTE_UNIT_PORT_BAUD_RATE);
  #endif

  #ifdef FEATURE_GPS
    //gps_port = GPS_PORT_MAPPED_TO;
    GPS_PORT.begin(GPS_PORT_BAUD_RATE);
    #ifdef GPS_MIRROR_PORT
      //gps_mirror_port = GPS_MIRROR_PORT;
      GPS_MIRROR_PORT.begin(GPS_MIRROR_PORT_BAUD_RATE);
    #endif //GPS_MIRROR_PORT
  #endif //FEATURE_GPS

  #ifdef DEBUG_LOOP
    control_port->println("initialize_serial() complete");
    control_port->flush();
  #endif // DEBUG_LOOP

} /* initialize_serial */


// --------------------------------------------------------------


void initialize_display(){


  #if defined(FEATURE_LCD_DISPLAY)
    #ifdef DEBUG_LOOP
      control_port->println("initialize_display()");
      control_port->flush();
    #endif // DEBUG_LOOP

    k3ngdisplay.initialize();

    #if defined(FEATURE_TEST_DISPLAY_AT_STARTUP)
      test_display();
    #endif

    #ifdef OPTION_DISPLAY_VERSION_ON_STARTUP 
      k3ngdisplay.print_center_timed_message((char *)"\x4B\x33\x4E\x47",(char *)"\x52\x6F\x74\x6F\x72\x20\x43\x6F\x6E\x74\x72\x6F\x6C\x6C\x65\x72",(char *)CODE_VERSION,SPLASH_SCREEN_TIME);
    #else
      k3ngdisplay.print_center_timed_message((char *)"\x4B\x33\x4E\x47",(char *)"\x52\x6F\x74\x6F\x72\x20\x43\x6F\x6E\x74\x72\x6F\x6C\x6C\x65\x72",SPLASH_SCREEN_TIME);
    #endif

    k3ngdisplay.service(0);


    #ifdef DEBUG_LOOP
      control_port->println("exiting initialize_display()");
      control_port->flush();
    #endif // DEBUG_LOOP


  #endif //defined(FEATURE_LCD_DISPLAY)



}


// --------------------------------------------------------------

void initialize_peripherals(){

  #ifdef DEBUG_LOOP
    control_port->println("initialize_peripherals()");
    control_port->flush();
  #endif // DEBUG_LOOP

  #ifdef FEATURE_WIRE_SUPPORT
    Wire.begin();
  #endif

  #ifdef FEATURE_AZ_POSITION_HMC5883L
    compass = HMC5883L();
    int error;
    error = compass.SetScale(1.3); // Set the scale of the compass.
     #ifndef OPTION_DISABLE_HMC5883L_ERROR_CHECKING
      if (error != 0) {
        #if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
          control_port->print(F("initialize_peripherals: compass error:"));
          control_port->println(compass.GetErrorText(error)); // check if there is an error, and print if so
        #endif
      }
    #endif //OPTION_DISABLE_HMC5883L_ERROR_CHECKING
    error = compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
    #ifndef OPTION_DISABLE_HMC5883L_ERROR_CHECKING
      if (error != 0) {
        #if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
          control_port->print(F("initialize_peripherals: compass error:"));
          control_port->println(compass.GetErrorText(error)); // check if there is an error, and print if so
        #endif
      }
    #endif //OPTION_DISABLE_HMC5883L_ERROR_CHECKING
  #endif // FEATURE_AZ_POSITION_HMC5883L

  #if defined(FEATURE_AZ_POSITION_HMC5883L_USING_JARZEBSKI_LIBRARY)
    while (!compass.begin())
    {
      #if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
        control_port->println("initialize_peripherals: could not find a valid HMC5883L sensor");
      #endif
      delay(500);
    }

    // Set measurement range
    compass.setRange(HMC5883L_RANGE_1_3GA);

    // Set measurement mode
    compass.setMeasurementMode(HMC5883L_CONTINOUS);

    // Set data rate
    compass.setDataRate(HMC5883L_DATARATE_30HZ);

    // Set number of samples averaged
    compass.setSamples(HMC5883L_SAMPLES_8);

    // Set calibration offset. See HMC5883L_calibration.ino
    compass.setOffset(0, 0);


  #endif //FEATURE_AZ_POSITION_HMC5883L_USING_JARZEBSKI_LIBRARY


  #if defined(FEATURE_AZ_POSITION_DFROBOT_QMC5883)

    while (!compass.begin()){
      #if defined(DEBUG_QMC5883)
        control_port->println("initialize_peripherals: could not find a valid QMC5883 sensor");
      #endif
      delay(500);
    }

    if(compass.isHMC()){
      #if defined(DEBUG_QMC5883)
        control_port->println("initialize_peripherals: initialize HMC5883");
      #endif
      compass.setRange(HMC5883L_RANGE_1_3GA);
      compass.setMeasurementMode(HMC5883L_CONTINOUS);
      compass.setDataRate(HMC5883L_DATARATE_15HZ);
      compass.setSamples(HMC5883L_SAMPLES_8);
    } else if(compass.isQMC()){
      #if defined(DEBUG_QMC5883)
        control_port->println("initialize_peripherals: initialize QMC5883");
      #endif  
      compass.setRange(QMC5883_RANGE_2GA);
      compass.setMeasurementMode(QMC5883_CONTINOUS); 
      compass.setDataRate(QMC5883_DATARATE_50HZ);
      compass.setSamples(QMC5883_SAMPLES_8);
    }

  #endif //FEATURE_AZ_POSITION_DFROBOT_QMC5883  


  #if defined(FEATURE_AZ_POSITION_MECHASOLUTION_QMC5883)
    compass.init();
    //compass.setMode(Mode_Continuous,ODR_200Hz,RNG_2G,OSR_256);
  #endif //FEATURE_AZ_POSITION_MECHASOLUTION_QMC5883

  #ifdef FEATURE_EL_POSITION_ADXL345_USING_LOVE_ELECTRON_LIB
    accel = ADXL345();
    accel.SetRange(2, true);
    accel.EnableMeasurements();
  #endif // FEATURE_EL_POSITION_ADXL345_USING_LOVE_ELECTRON_LIB

  #ifdef FEATURE_EL_POSITION_ADXL345_USING_ADAFRUIT_LIB
    accel.begin();
  #endif // FEATURE_EL_POSITION_ADXL345_USING_ADAFRUIT_LIB

  #ifdef FEATURE_JOYSTICK_CONTROL
    pinModeEnhanced(pin_joystick_x, INPUT);
    pinModeEnhanced(pin_joystick_y, INPUT);
  #endif // FEATURE_JOYSTICK_CONTROL

  #if defined(FEATURE_EL_POSITION_ADAFRUIT_LSM303) || defined(FEATURE_AZ_POSITION_ADAFRUIT_LSM303)
    if (!lsm.begin()) {
      #if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
        control_port->println(F("setup: LSM303 error"));
      #endif
    }
  #endif // FEATURE_EL_POSITION_ADAFRUIT_LSM303 || FEATURE_AZ_POSITION_ADAFRUIT_LSM303


  #if defined(FEATURE_AZ_POSITION_POLOLU_LSM303) || defined(FEATURE_EL_POSITION_POLOLU_LSM303)
    if (!compass.init()) {
      #if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
        control_port->println(F("setup: LSM303 error"));
      #endif
    }
    compass.enableDefault();
    compass.m_min = (LSM303::vector<int16_t>) POLOLU_LSM_303_MIN_ARRAY;
    compass.m_max = (LSM303::vector<int16_t>) POLOLU_LSM_303_MAX_ARRAY;
  #endif //defined(FEATURE_AZ_POSITION_POLOLU_LSM303) || defined(FEATURE_EL_POSITION_POLOLU_LSM303)


  #if defined(FEATURE_AZ_POSITION_HH12_AS5045_SSI) || defined(FEATURE_AZ_POSITION_HH12_AS5045_SSI_RELATIVE)
    azimuth_hh12.initialize(az_hh12_clock_pin, az_hh12_cs_pin, az_hh12_data_pin);
  #endif // FEATURE_AZ_POSITION_HH12_AS5045_SSI

  #ifdef FEATURE_EL_POSITION_HH12_AS5045_SSI
    elevation_hh12.initialize(el_hh12_clock_pin, el_hh12_cs_pin, el_hh12_data_pin);
  #endif // FEATURE_EL_POSITION_HH12_AS5045_SSI

  #if defined(FEATURE_AZ_POSITION_A2_ABSOLUTE_ENCODER) || defined(FEATURE_EL_POSITION_A2_ABSOLUTE_ENCODER)
    SEIbus1.initialize();
  #endif // defined(FEATURE_AZ_POSITION_A2_ABSOLUTE_ENCODER) || defined(FEATURE_EL_POSITION_A2_ABSOLUTE_ENCODER)



  #ifdef FEATURE_RTC_DS1307
    #ifdef DEBUG_RTC
      debug.println("initialize_peripherals: rtc.begin");
    #endif // DEBUG_RTC
    rtc.begin();
    #ifdef DEBUG_RTC
      debug.println("initialize_peripherals: begin complete");
    #endif // DEBUG_RTC    
  #endif // FEATURE_RTC_DS1307

  #ifdef FEATURE_WIFI
    initialize_wifi();
  #endif //FEATURE_WIFI

  #ifdef SET_I2C_BUS_SPEED
     TWBR = ((F_CPU / SET_I2C_BUS_SPEED) - 16) / 2;
  #endif

  #ifdef FEATURE_CLOCK
    #if defined(OPTION_USE_OLD_TIME_CODE)
      set_clock.year = CLOCK_DEFAULT_YEAR_AT_BOOTUP;
      set_clock.month = CLOCK_DEFAULT_MONTH_AT_BOOTUP;
      set_clock.day = CLOCK_DEFAULT_DAY_AT_BOOTUP;
      set_clock.hours = CLOCK_DEFAULT_HOURS_AT_BOOTUP;
      set_clock.minutes = CLOCK_DEFAULT_MINUTES_AT_BOOTUP;
      set_clock.seconds = CLOCK_DEFAULT_SECONDS_AT_BOOTUP;
    #else //OPTION_USE_OLD_TIME_CODE
      setTime(CLOCK_DEFAULT_HOURS_AT_BOOTUP,CLOCK_DEFAULT_MINUTES_AT_BOOTUP,CLOCK_DEFAULT_SECONDS_AT_BOOTUP,CLOCK_DEFAULT_DAY_AT_BOOTUP,CLOCK_DEFAULT_MONTH_AT_BOOTUP,CLOCK_DEFAULT_YEAR_AT_BOOTUP);
    #endif //OPTION_USE_OLD_TIME_CODE
  #endif


} /* initialize_peripherals */


// --------------------------------------------------------------
void submit_request(byte axis, byte request, float parm, byte called_by){

  #ifdef DEBUG_SUBMIT_REQUEST
    debug.print("submit_request: ");
    debug.print(called_by);
    debug.print(" ");
  #endif // DEBUG_SUBMIT_REQUEST

  #ifdef FEATURE_PARK
    park_status = NOT_PARKED;
  #endif // FEATURE_PARK

  // #if defined(FEATURE_LCD_DISPLAY)
  //   perform_screen_redraw = 1;
  // #endif

  if (axis == AZ) {
    #ifdef DEBUG_SUBMIT_REQUEST
      debug.print("AZ "); 
    #endif // DEBUG_SUBMIT_REQUEST
    az_request = request;
    az_request_parm = parm;
    az_request_queue_state = IN_QUEUE;
  }

  #ifdef FEATURE_ELEVATION_CONTROL
  if (axis == EL) {
    #ifdef DEBUG_SUBMIT_REQUEST
      debug.print("EL ");
    #endif // DEBUG_SUBMIT_REQUEST
    el_request = request;
    el_request_parm = parm;
    el_request_queue_state = IN_QUEUE;
  }
  #endif // FEATURE_ELEVATION_CONTROL

  #ifdef DEBUG_SUBMIT_REQUEST
    switch(request){
      case 0: debug.print("REQUEST_STOP");break;
      case 1: debug.print("REQUEST_AZIMUTH");break;
      case 2: debug.print("REQUEST_AZIMUTH_RAW");break;
      case 3: debug.print("REQUEST_CW");break;
      case 4: debug.print("REQUEST_CCW");break;
      case 5: debug.print("REQUEST_UP");break;
      case 6: debug.print("REQUEST_DOWN");break;
      case 7: debug.print("REQUEST_ELEVATION");break;
      case 8: debug.print("REQUEST_KILL");break;
    }
    debug.print(" ");
    debug.print(parm);
    debug.println("");
  #endif // DEBUG_SUBMIT_REQUEST  

} /* submit_request */
// --------------------------------------------------------------
void service_rotation(){

  #ifdef DEBUG_LOOP
    control_port->println("service_rotation()");
    control_port->flush();
  #endif // DEBUG_LOOP

  #ifdef DEBUG_PROCESSES
    service_process_debug(DEBUG_PROCESSES_PROCESS_ENTER,PROCESS_SERVICE_ROTATION);
  #endif      


  #if defined(FEATURE_AZ_POSITION_INCREMENTAL_ENCODER) || defined(FEATURE_EL_POSITION_INCREMENTAL_ENCODER)
    service_rotation_lock = 1;
  #endif

  static byte az_direction_change_flag = 0;
  static byte az_initial_slow_down_voltage = 0;

  #ifdef FEATURE_ELEVATION_CONTROL
    static byte el_direction_change_flag = 0;
    static byte el_initial_slow_down_voltage = 0;
  #endif // FEATURE_ELEVATION_CONTROL

  if (az_state == INITIALIZE_NORMAL_CW) {
    update_az_variable_outputs(normal_az_speed_voltage);
    rotator(ACTIVATE, CW, 3);
    az_state = NORMAL_CW;
  }

  if (az_state == INITIALIZE_NORMAL_CCW) {
    update_az_variable_outputs(normal_az_speed_voltage);
    rotator(ACTIVATE, CCW, 4);
    az_state = NORMAL_CCW;
  }

  if (az_state == INITIALIZE_SLOW_START_CW) {
    update_az_variable_outputs(AZ_SLOW_START_STARTING_PWM);
    rotator(ACTIVATE, CW, 5);
    az_slowstart_start_time = millis();
    az_last_step_time = 0;
    az_slow_start_step = 0;
    az_state = SLOW_START_CW;
    #ifdef DEBUG_SERVICE_ROTATION
    debug.print("service_rotation: INITIALIZE_SLOW_START_CW -> SLOW_START_CW");
    #endif // DEBUG_SERVICE_ROTATION
  }

  if (az_state == INITIALIZE_SLOW_START_CCW) {
    update_az_variable_outputs(AZ_SLOW_START_STARTING_PWM);
    rotator(ACTIVATE, CCW, 6);
    az_slowstart_start_time = millis();
    az_last_step_time = 0;
    az_slow_start_step = 0;
    az_state = SLOW_START_CCW;
    #ifdef DEBUG_SERVICE_ROTATION
    debug.print("service_rotation: INITIALIZE_SLOW_START_CCW -> SLOW_START_CCW");
    #endif // DEBUG_SERVICE_ROTATION
  }

  if (az_state == INITIALIZE_TIMED_SLOW_DOWN_CW) {
    az_direction_change_flag = 0;
    az_timed_slow_down_start_time = millis();
    az_last_step_time = millis();
    az_slow_down_step = AZ_SLOW_DOWN_STEPS - 1;
    az_state = TIMED_SLOW_DOWN_CW;
  }

  if (az_state == INITIALIZE_TIMED_SLOW_DOWN_CCW) {
    az_direction_change_flag = 0;
    az_timed_slow_down_start_time = millis();
    az_last_step_time = millis();
    az_slow_down_step = AZ_SLOW_DOWN_STEPS - 1;
    az_state = TIMED_SLOW_DOWN_CCW;
  }

  if (az_state == INITIALIZE_DIR_CHANGE_TO_CW) {
    az_direction_change_flag = 1;
    az_timed_slow_down_start_time = millis();
    az_last_step_time = millis();
    az_slow_down_step = AZ_SLOW_DOWN_STEPS - 1;
    az_state = TIMED_SLOW_DOWN_CCW;
  }

  if (az_state == INITIALIZE_DIR_CHANGE_TO_CCW) {
    az_direction_change_flag = 1;
    az_timed_slow_down_start_time = millis();
    az_last_step_time = millis();
    az_slow_down_step = AZ_SLOW_DOWN_STEPS - 1;
    az_state = TIMED_SLOW_DOWN_CW;
  }

  // slow start-------------------------------------------------------------------------------------------------
  if ((az_state == SLOW_START_CW) || (az_state == SLOW_START_CCW)) {
    if ((millis() - az_slowstart_start_time) >= AZ_SLOW_START_UP_TIME) {  // is it time to end slow start?
      #ifdef DEBUG_SERVICE_ROTATION
      debug.print("service_rotation: NORMAL_C");
      #endif // DEBUG_SERVICE_ROTATION
      if (az_state == SLOW_START_CW) {
        az_state = NORMAL_CW;
        #ifdef DEBUG_SERVICE_ROTATION
        debug.print("W");
        #endif // DEBUG_SERVICE_ROTATION
      } else {
        az_state = NORMAL_CCW;
        #ifdef DEBUG_SERVICE_ROTATION
        debug.print("CW");
        #endif // DEBUG_SERVICE_ROTATION
      }
      update_az_variable_outputs(normal_az_speed_voltage);
    } else {  // it's not time to end slow start yet, but let's check if it's time to step up the speed voltage
      if (((millis() - az_last_step_time) > (AZ_SLOW_START_UP_TIME / AZ_SLOW_START_STEPS)) && (normal_az_speed_voltage > AZ_SLOW_START_STARTING_PWM)) {
        #ifdef DEBUG_SERVICE_ROTATION
        debug.print("service_rotation: step up: ");
        debug.print(az_slow_start_step);
        debug.print(" pwm: ");
        debug.print((int)(AZ_SLOW_START_STARTING_PWM + ((normal_az_speed_voltage - AZ_SLOW_START_STARTING_PWM) * ((float)az_slow_start_step / (float)(AZ_SLOW_START_STEPS - 1)))));
        debug.println("");
        #endif // DEBUG_SERVICE_ROTATION
        update_az_variable_outputs((AZ_SLOW_START_STARTING_PWM + ((normal_az_speed_voltage - AZ_SLOW_START_STARTING_PWM) * ((float)az_slow_start_step / (float)(AZ_SLOW_START_STEPS - 1)))));
        az_last_step_time = millis();
        az_slow_start_step++;
      }
    }
  } // ((az_state == SLOW_START_CW) || (az_state == SLOW_START_CCW))


  // timed slow down ------------------------------------------------------------------------------------------------------
  if (((az_state == TIMED_SLOW_DOWN_CW) || (az_state == TIMED_SLOW_DOWN_CCW)) && ((millis() - az_last_step_time) >= (TIMED_SLOW_DOWN_TIME / AZ_SLOW_DOWN_STEPS))) {
    #ifdef DEBUG_SERVICE_ROTATION
      debug.print("service_rotation: TIMED_SLOW_DOWN step down: ");
      debug.print(az_slow_down_step);
      debug.print(" pwm: ");
      debug.print((int)(normal_az_speed_voltage * ((float)az_slow_down_step / (float)AZ_SLOW_DOWN_STEPS)));
      debug.println("");
    #endif // DEBUG_SERVICE_ROTATION
    //updated 2016-05-15
    //update_az_variable_outputs((int)(normal_az_speed_voltage * ((float)az_slow_down_step / (float)AZ_SLOW_DOWN_STEPS)));
    update_az_variable_outputs((int)(current_az_speed_voltage * ((float)az_slow_down_step / (float)AZ_SLOW_DOWN_STEPS)));
    az_last_step_time = millis();
    if (az_slow_down_step > 0) {az_slow_down_step--;}

    if (az_slow_down_step == 0) { // is it time to exit timed slow down?
      #ifdef DEBUG_SERVICE_ROTATION
        debug.print("service_rotation: TIMED_SLOW_DOWN->IDLE");
      #endif // DEBUG_SERVICE_ROTATION
      rotator(DEACTIVATE, CW, 7);
      rotator(DEACTIVATE, CCW, 7);        
      if (az_direction_change_flag) {
        if (az_state == TIMED_SLOW_DOWN_CW) {
          //rotator(ACTIVATE, CCW, 8);
          if (az_slowstart_active) {
            az_state = INITIALIZE_SLOW_START_CCW;
          } else { az_state = NORMAL_CCW; };
          az_direction_change_flag = 0;
        }
        if (az_state == TIMED_SLOW_DOWN_CCW) {
          //rotator(ACTIVATE, CW, 9);
          if (az_slowstart_active) {
            az_state = INITIALIZE_SLOW_START_CW;
          } else { az_state = NORMAL_CW; };
          az_direction_change_flag = 0;
        }
      } else {
        az_state = IDLE;
        az_request_queue_state = NONE;

        #if defined(FEATURE_AUDIBLE_ALERT)
          if (configuration.audible_alert_enabled_az_target){
            audible_alert(AUDIBLE_ALERT_ACTIVATE);
          }
        #endif

      }
    } 

  }  // ((az_state == TIMED_SLOW_DOWN_CW) || (az_state == TIMED_SLOW_DOWN_CCW))



  // slow down ---------------------------------------------------------------------------------------------------------------
  if ((az_state == SLOW_DOWN_CW) || (az_state == SLOW_DOWN_CCW)) {

    // is it time to do another step down?
    if (abs((target_raw_azimuth - raw_azimuth)) <= (((float)SLOW_DOWN_BEFORE_TARGET_AZ * ((float)az_slow_down_step / (float)AZ_SLOW_DOWN_STEPS)))) {
      #ifdef DEBUG_SERVICE_ROTATION
      debug.print("service_rotation: step down: ");
      debug.print(az_slow_down_step);
      debug.print(" pwm: ");
      debug.print((int)(AZ_SLOW_DOWN_PWM_STOP + ((az_initial_slow_down_voltage - AZ_SLOW_DOWN_PWM_STOP) * ((float)az_slow_down_step / (float)AZ_SLOW_DOWN_STEPS))));
      debug.println("");
      #endif // DEBUG_SERVICE_ROTATION
      update_az_variable_outputs((AZ_SLOW_DOWN_PWM_STOP + ((az_initial_slow_down_voltage - AZ_SLOW_DOWN_PWM_STOP) * ((float)az_slow_down_step / (float)AZ_SLOW_DOWN_STEPS))));
      if (az_slow_down_step > 0) {az_slow_down_step--;}
    }
  }  // ((az_state == SLOW_DOWN_CW) || (az_state == SLOW_DOWN_CCW))

  // normal -------------------------------------------------------------------------------------------------------------------
  // if slow down is enabled, see if we're ready to go into slowdown
  //if (((az_state == NORMAL_CW) || (az_state == SLOW_START_CW) || (az_state == NORMAL_CCW) || (az_state == SLOW_START_CCW)) &&
  if (((az_state == NORMAL_CW) || (az_state == NORMAL_CCW)) && 
      (az_request_queue_state == IN_PROGRESS_TO_TARGET) && az_slowdown_active && (abs((target_raw_azimuth - raw_azimuth)) <= SLOW_DOWN_BEFORE_TARGET_AZ)) {

    byte az_state_was = az_state;

    #ifdef DEBUG_SERVICE_ROTATION
    debug.print("service_rotation: SLOW_DOWN_C");
    #endif // DEBUG_SERVICE_ROTATION
    az_slow_down_step = AZ_SLOW_DOWN_STEPS - 1;
    if ((az_state == NORMAL_CW) || (az_state == SLOW_START_CW)) {
      az_state = SLOW_DOWN_CW;
      #ifdef DEBUG_SERVICE_ROTATION
      debug.print("W");
      #endif // DEBUG_SERVICE_ROTATION
    } else {
      az_state = SLOW_DOWN_CCW;
      #ifdef DEBUG_SERVICE_ROTATION
      debug.print("CW");
      #endif // DEBUG_SERVICE_ROTATION
    }
    
    if ((az_state_was == SLOW_START_CW) || (az_state_was == SLOW_START_CCW)){
      az_initial_slow_down_voltage = (AZ_INITIALLY_IN_SLOW_DOWN_PWM);
      update_az_variable_outputs(az_initial_slow_down_voltage);
      #ifdef DEBUG_SERVICE_ROTATION
      debug.print(" SLOW_START -> SLOW_DOWN az_initial_slow_down_voltage:");
      debug.print(az_initial_slow_down_voltage);
      debug.print(" ");
      #endif // DEBUG_SERVICE_ROTATION
    } else {
      if (AZ_SLOW_DOWN_PWM_START < current_az_speed_voltage) {
        update_az_variable_outputs(AZ_SLOW_DOWN_PWM_START);
        az_initial_slow_down_voltage = AZ_SLOW_DOWN_PWM_START;
      } else {
        az_initial_slow_down_voltage = current_az_speed_voltage;
      }
    }

  }

  // check rotation target --------------------------------------------------------------------------------------------------------
  if ((az_state != IDLE) && (az_request_queue_state == IN_PROGRESS_TO_TARGET) ) {
    if ((az_state == NORMAL_CW) || (az_state == SLOW_START_CW) || (az_state == SLOW_DOWN_CW)) {
      if ((abs(raw_azimuth - target_raw_azimuth) < (AZIMUTH_TOLERANCE)) || ((raw_azimuth > target_raw_azimuth) && ((raw_azimuth - target_raw_azimuth) < ((AZIMUTH_TOLERANCE + 5))))) {
        delay(50);
        read_azimuth(0);
        if ((abs(raw_azimuth - target_raw_azimuth) < (AZIMUTH_TOLERANCE)) || ((raw_azimuth > target_raw_azimuth) && ((raw_azimuth - target_raw_azimuth) < ((AZIMUTH_TOLERANCE + 5))))) {
          rotator(DEACTIVATE, CW, 10);
          rotator(DEACTIVATE, CCW, 10);
          az_state = IDLE;
          az_request_queue_state = NONE;
          #ifdef DEBUG_SERVICE_ROTATION
            debug.print("service_rotation: IDLE");
          #endif // DEBUG_SERVICE_ROTATION

          #if defined(FEATURE_PARK) && !defined(FEATURE_ELEVATION_CONTROL)
            if (park_status == PARK_INITIATED) {
              park_status = PARKED;
            }
          #endif // defined(FEATURE_PARK) && !defined(FEATURE_ELEVATION_CONTROL)

          #if defined(FEATURE_PARK) && defined(FEATURE_ELEVATION_CONTROL)
            if ((park_status == PARK_INITIATED) && (el_state == IDLE)) {
              park_status = PARKED;
            }
          #endif // defined(FEATURE_PARK) && !defined(FEATURE_ELEVATION_CONTROL)

          #if defined(FEATURE_AUDIBLE_ALERT)
            if (configuration.audible_alert_enabled_az_target){
              audible_alert(AUDIBLE_ALERT_ACTIVATE);
            }
          #endif

        }
      }
    } else {
      if ((abs(raw_azimuth - target_raw_azimuth) < (AZIMUTH_TOLERANCE)) || ((raw_azimuth < target_raw_azimuth) && ((target_raw_azimuth - raw_azimuth) < ((AZIMUTH_TOLERANCE + 5))))) {
        delay(50);
        read_azimuth(0);
        if ((abs(raw_azimuth - target_raw_azimuth) < (AZIMUTH_TOLERANCE)) || ((raw_azimuth < target_raw_azimuth) && ((target_raw_azimuth - raw_azimuth) < ((AZIMUTH_TOLERANCE + 5))))) {
          rotator(DEACTIVATE, CW, 11);
          rotator(DEACTIVATE, CCW, 11);
          az_state = IDLE;
          az_request_queue_state = NONE;
          #ifdef DEBUG_SERVICE_ROTATION
            debug.print("service_rotation: IDLE");
          #endif // DEBUG_SERVICE_ROTATION

          #if defined(FEATURE_PARK) && !defined(FEATURE_ELEVATION_CONTROL)
            if (park_status == PARK_INITIATED) {
              park_status = PARKED;
            }
          #endif // defined(FEATURE_PARK) && !defined(FEATURE_ELEVATION_CONTROL)

          #if defined(FEATURE_PARK) && defined(FEATURE_ELEVATION_CONTROL)
            if ((park_status == PARK_INITIATED) && (el_state == IDLE)) {
              park_status = PARKED;
            }
          #endif // defined(FEATURE_PARK) && !defined(FEATURE_ELEVATION_CONTROL)

          #if defined(FEATURE_AUDIBLE_ALERT)
            if (configuration.audible_alert_enabled_az_target){
              audible_alert(AUDIBLE_ALERT_ACTIVATE);
            }
          #endif

        }
      }
    }
  }



  #ifdef FEATURE_ELEVATION_CONTROL
  if (el_state == INITIALIZE_NORMAL_UP) {
    update_el_variable_outputs(normal_el_speed_voltage);
    rotator(ACTIVATE, UP, 12);
    el_state = NORMAL_UP;
  }

  if (el_state == INITIALIZE_NORMAL_DOWN) {
    update_el_variable_outputs(normal_el_speed_voltage);
    rotator(ACTIVATE, DOWN, 13);
    el_state = NORMAL_DOWN;
  }

  if (el_state == INITIALIZE_SLOW_START_UP) {
    update_el_variable_outputs(EL_SLOW_START_STARTING_PWM);
    rotator(ACTIVATE, UP, 14);
    el_slowstart_start_time = millis();
    el_last_step_time = 0;
    el_slow_start_step = 0;
    el_state = SLOW_START_UP;
    #ifdef DEBUG_SERVICE_ROTATION
    debug.print("service_rotation: INITIALIZE_SLOW_START_UP -> SLOW_START_UP");
    #endif // DEBUG_SERVICE_ROTATION
  }

  if (el_state == INITIALIZE_SLOW_START_DOWN) {
    update_el_variable_outputs(EL_SLOW_START_STARTING_PWM);
    rotator(ACTIVATE, DOWN, 15);
    el_slowstart_start_time = millis();
    el_last_step_time = 0;
    el_slow_start_step = 0;
    el_state = SLOW_START_DOWN;
    #ifdef DEBUG_SERVICE_ROTATION
    debug.print("service_rotation: INITIALIZE_SLOW_START_DOWN -> SLOW_START_DOWN");
    #endif // DEBUG_SERVICE_ROTATION
  }

  if (el_state == INITIALIZE_TIMED_SLOW_DOWN_UP) {
    el_direction_change_flag = 0;
    el_timed_slow_down_start_time = millis();
    el_last_step_time = millis();
    el_slow_down_step = EL_SLOW_DOWN_STEPS - 1;
    el_state = TIMED_SLOW_DOWN_UP;
  }

  if (el_state == INITIALIZE_TIMED_SLOW_DOWN_DOWN) {
    el_direction_change_flag = 0;
    el_timed_slow_down_start_time = millis();
    el_last_step_time = millis();
    el_slow_down_step = EL_SLOW_DOWN_STEPS - 1;
    el_state = TIMED_SLOW_DOWN_DOWN;
  }

  if (el_state == INITIALIZE_DIR_CHANGE_TO_UP) {
    el_direction_change_flag = 1;
    el_timed_slow_down_start_time = millis();
    el_last_step_time = millis();
    el_slow_down_step = EL_SLOW_DOWN_STEPS - 1;
    el_state = TIMED_SLOW_DOWN_DOWN;
  }

  if (el_state == INITIALIZE_DIR_CHANGE_TO_DOWN) {
    el_direction_change_flag = 1;
    el_timed_slow_down_start_time = millis();
    el_last_step_time = millis();
    el_slow_down_step = EL_SLOW_DOWN_STEPS - 1;
    el_state = TIMED_SLOW_DOWN_UP;
  }

  // slow start-------------------------------------------------------------------------------------------------
  if ((el_state == SLOW_START_UP) || (el_state == SLOW_START_DOWN)) {
    if ((millis() - el_slowstart_start_time) >= EL_SLOW_START_UP_TIME) {  // is it time to end slow start?
      #ifdef DEBUG_SERVICE_ROTATION
      debug.print("service_rotation: NORMAL_");
      #endif // DEBUG_SERVICE_ROTATION
      if (el_state == SLOW_START_UP) {
        el_state = NORMAL_UP;
        #ifdef DEBUG_SERVICE_ROTATION
          debug.print("UP");
        #endif // DEBUG_SERVICE_ROTATION
      } else {
        el_state = NORMAL_DOWN;
        #ifdef DEBUG_SERVICE_ROTATION
          debug.print("DOWN");
        #endif // DEBUG_SERVICE_ROTATION
      }
      update_el_variable_outputs(normal_el_speed_voltage);
    } else {  // it's not time to end slow start yet, but let's check if it's time to step up the speed voltage
      if (((millis() - el_last_step_time) > (EL_SLOW_START_UP_TIME / EL_SLOW_START_STEPS)) && (normal_el_speed_voltage > EL_SLOW_START_STARTING_PWM)) {
        #ifdef DEBUG_SERVICE_ROTATION
        debug.print("service_rotation: step up: ");
        debug.print(el_slow_start_step);
        debug.print(" pwm: ");
        debug.print((int)(EL_SLOW_START_STARTING_PWM + ((normal_el_speed_voltage - EL_SLOW_START_STARTING_PWM) * ((float)el_slow_start_step / (float)(EL_SLOW_START_STEPS - 1)))));
        debug.println("");
        #endif // DEBUG_SERVICE_ROTATION
        update_el_variable_outputs((EL_SLOW_START_STARTING_PWM + ((normal_el_speed_voltage - EL_SLOW_START_STARTING_PWM) * ((float)el_slow_start_step / (float)(EL_SLOW_START_STEPS - 1)))));
        el_last_step_time = millis();
        el_slow_start_step++;
      }
    }
  } // ((el_state == SLOW_START_UP) || (el_state == SLOW_START_DOWN))


  // timed slow down ------------------------------------------------------------------------------------------------------
  if (((el_state == TIMED_SLOW_DOWN_UP) || (el_state == TIMED_SLOW_DOWN_DOWN)) && ((millis() - el_last_step_time) >= (TIMED_SLOW_DOWN_TIME / EL_SLOW_DOWN_STEPS))) {
    #ifdef DEBUG_SERVICE_ROTATION
    debug.print("service_rotation: TIMED_SLOW_DOWN step down: ");
    debug.print(el_slow_down_step);
    debug.print(" pwm: ");
    debug.print((int)(normal_el_speed_voltage * ((float)el_slow_down_step / (float)EL_SLOW_DOWN_STEPS)));
    debug.println("");
    #endif // DEBUG_SERVICE_ROTATION
    update_el_variable_outputs((int)(normal_el_speed_voltage * ((float)el_slow_down_step / (float)EL_SLOW_DOWN_STEPS)));
    el_last_step_time = millis();
    if (el_slow_down_step > 0) {el_slow_down_step--;}

    if (el_slow_down_step == 0) { // is it time to exit timed slow down?
      #ifdef DEBUG_SERVICE_ROTATION
      debug.print("service_rotation: TIMED_SLOW_DOWN->IDLE");
      #endif // DEBUG_SERVICE_ROTATION
      rotator(DEACTIVATE, UP, 16);
      rotator(DEACTIVATE, DOWN, 16);
      if (el_direction_change_flag) {
        if (el_state == TIMED_SLOW_DOWN_UP) {
          if (el_slowstart_active) {
            el_state = INITIALIZE_SLOW_START_DOWN;
          } else { el_state = NORMAL_DOWN; };
          el_direction_change_flag = 0;
        }
        if (el_state == TIMED_SLOW_DOWN_DOWN) {
          if (el_slowstart_active) {
            el_state = INITIALIZE_SLOW_START_UP;
          } else { el_state = NORMAL_UP; };
          el_direction_change_flag = 0;
        }
      } else {
        el_state = IDLE;
        el_request_queue_state = NONE;

        #if defined(FEATURE_AUDIBLE_ALERT)
          if (configuration.audible_alert_enabled_el_target){
            audible_alert(AUDIBLE_ALERT_ACTIVATE);
          }
        #endif

      }
    }

  }  // ((el_state == TIMED_SLOW_DOWN_UP) || (el_state == TIMED_SLOW_DOWN_DOWN))



  // slow down ---------------------------------------------------------------------------------------------------------------
  if ((el_state == SLOW_DOWN_UP) || (el_state == SLOW_DOWN_DOWN)) {
    // is it time to do another step down?
    if (abs((target_elevation - elevation)) <= (((float)SLOW_DOWN_BEFORE_TARGET_EL * ((float)el_slow_down_step / (float)EL_SLOW_DOWN_STEPS)))) {
      #ifdef DEBUG_SERVICE_ROTATION
      debug.print("service_rotation: step down: ");
      debug.print(el_slow_down_step);
      debug.print(" pwm: ");
      debug.print((int)(EL_SLOW_DOWN_PWM_STOP + ((el_initial_slow_down_voltage - EL_SLOW_DOWN_PWM_STOP) * ((float)el_slow_down_step / (float)EL_SLOW_DOWN_STEPS))));
      debug.println("");
      #endif // DEBUG_SERVICE_ROTATION
      update_el_variable_outputs((EL_SLOW_DOWN_PWM_STOP + ((el_initial_slow_down_voltage - EL_SLOW_DOWN_PWM_STOP) * ((float)el_slow_down_step / (float)EL_SLOW_DOWN_STEPS))));
      if (el_slow_down_step > 0) {el_slow_down_step--;}
    }
  }  // ((el_state == SLOW_DOWN_UP) || (el_state == SLOW_DOWN_DOWN))

  // normal -------------------------------------------------------------------------------------------------------------------
  // if slow down is enabled, see if we're ready to go into slowdown
  if (((el_state == NORMAL_UP) || (el_state == SLOW_START_UP) || (el_state == NORMAL_DOWN) || (el_state == SLOW_START_DOWN)) &&
      (el_request_queue_state == IN_PROGRESS_TO_TARGET) && el_slowdown_active && (abs((target_elevation - elevation)) <= SLOW_DOWN_BEFORE_TARGET_EL)) {
    
    byte el_state_was = el_state;


    #ifdef DEBUG_SERVICE_ROTATION
    debug.print("service_rotation: SLOW_DOWN_");
    #endif // DEBUG_SERVICE_ROTATION
    el_slow_down_step = EL_SLOW_DOWN_STEPS - 1;
    if ((el_state == NORMAL_UP) || (el_state == SLOW_START_UP)) {
      el_state = SLOW_DOWN_UP;
      #ifdef DEBUG_SERVICE_ROTATION
      debug.print("UP");
      #endif // DEBUG_SERVICE_ROTATION
    } else {
      el_state = SLOW_DOWN_DOWN;
      #ifdef DEBUG_SERVICE_ROTATION
      debug.print("DOWN");
      #endif // DEBUG_SERVICE_ROTATION
    }

    if ((el_state_was == SLOW_START_UP) || (el_state_was == SLOW_START_DOWN)){
      el_initial_slow_down_voltage = EL_INITIALLY_IN_SLOW_DOWN_PWM;
      update_el_variable_outputs(el_initial_slow_down_voltage);
      #ifdef DEBUG_SERVICE_ROTATION
      debug.print(" SLOW_START -> SLOW_DOWN el_initial_slow_down_voltage:");
      debug.print(el_initial_slow_down_voltage);
      debug.print(" ");
      #endif // DEBUG_SERVICE_ROTATION    

    } else {
      if (EL_SLOW_DOWN_PWM_START < current_el_speed_voltage) {
        update_el_variable_outputs(EL_SLOW_DOWN_PWM_START);
        el_initial_slow_down_voltage = EL_SLOW_DOWN_PWM_START;
      } else {
        el_initial_slow_down_voltage = current_el_speed_voltage;
      }
    }
  }

  // check rotation target --------------------------------------------------------------------------------------------------------
  if ((el_state != IDLE) && (el_request_queue_state == IN_PROGRESS_TO_TARGET) ) {
    read_elevation(0);
    if ((el_state == NORMAL_UP) || (el_state == SLOW_START_UP) || (el_state == SLOW_DOWN_UP)) {
      if ((abs(elevation - target_elevation) < (ELEVATION_TOLERANCE)) || ((elevation > target_elevation) && ((elevation - target_elevation) < ((ELEVATION_TOLERANCE + 5))))) {
        #ifndef OPTION_NO_ELEVATION_CHECK_TARGET_DELAY
          delay(50);
        #endif //OPTION_NO_ELEVATION_CHECK_TARGET_DELAY
        read_elevation(0);
        if ((abs(elevation - target_elevation) < (ELEVATION_TOLERANCE)) || ((elevation > target_elevation) && ((elevation - target_elevation) < ((ELEVATION_TOLERANCE + 5))))) {
          rotator(DEACTIVATE, UP, 17);
          rotator(DEACTIVATE, DOWN, 17);
          el_state = IDLE;
          el_request_queue_state = NONE;
          #ifdef DEBUG_SERVICE_ROTATION
            debug.print("service_rotation: IDLE");
          #endif // DEBUG_SERVICE_ROTATION

          #if defined(FEATURE_AUDIBLE_ALERT)
            if (configuration.audible_alert_enabled_el_target){
              audible_alert(AUDIBLE_ALERT_ACTIVATE);
            }
          #endif

          #if defined(FEATURE_PARK)
            if ((park_status == PARK_INITIATED) && (az_state == IDLE)) {
              park_status = PARKED;
            }
          #endif // defined(FEATURE_PARK)

        }
      }
    } else {
      read_elevation(0);
      if ((abs(elevation - target_elevation) <= (ELEVATION_TOLERANCE)) || ((elevation < target_elevation) && ((target_elevation - elevation) < ((ELEVATION_TOLERANCE + 5))))) {
        #ifndef OPTION_NO_ELEVATION_CHECK_TARGET_DELAY
          delay(50);
        #endif //OPTION_NO_ELEVATION_CHECK_TARGET_DELAY
        read_elevation(0);
        if ((abs(elevation - target_elevation) <= (ELEVATION_TOLERANCE)) || ((elevation < target_elevation) && ((target_elevation - elevation) < ((ELEVATION_TOLERANCE + 5))))) {
          rotator(DEACTIVATE, UP, 18);
          rotator(DEACTIVATE, DOWN, 18);
          el_state = IDLE;
          el_request_queue_state = NONE;
          #ifdef DEBUG_SERVICE_ROTATION
          debug.print("service_rotation: IDLE");
          #endif // DEBUG_SERVICE_ROTATION

          #if defined(FEATURE_AUDIBLE_ALERT)
            if (configuration.audible_alert_enabled_el_target){
              audible_alert(AUDIBLE_ALERT_ACTIVATE);
            }
          #endif

          #if defined(FEATURE_PARK)
            if ((park_status == PARK_INITIATED) && (az_state == IDLE)) {
              park_status = PARKED;
            }
          #endif // defined(FEATURE_PARK)
        }
      }
    }
  }




  #endif // FEATURE_ELEVATION_CONTROL

  #if defined(FEATURE_AZ_POSITION_INCREMENTAL_ENCODER) || defined(FEATURE_EL_POSITION_INCREMENTAL_ENCODER)
    service_rotation_lock = 0;
  #endif


  #ifdef DEBUG_PROCESSES
    service_process_debug(DEBUG_PROCESSES_PROCESS_EXIT,PROCESS_SERVICE_ROTATION);
  #endif        

} /* service_rotation */


// --------------------------------------------------------------
void service_request_queue(){

  #ifdef DEBUG_LOOP
    control_port->println("service_request_queue()");
    control_port->flush();
  #endif // DEBUG_LOOP

  int work_target_raw_azimuth = 0;
  byte direction_to_go = 0;
  byte within_tolerance_flag = 0;

  if (az_request_queue_state == IN_QUEUE) {


    #ifdef FEATURE_POWER_SWITCH
    last_activity_time = millis();
    #endif //FEATURE_POWER_SWITCH

    #ifdef DEBUG_SERVICE_REQUEST_QUEUE
    debug.print("service_request_queue: AZ ");
    #endif // DEBUG_SERVICE_REQUEST_QUEUE

    switch (az_request) {
      case (REQUEST_STOP):
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        debug.print("REQUEST_STOP");
        #endif // DEBUG_SERVICE_REQUEST_QUEUE
        change_tracking(DEACTIVATE_ALL);
        #ifdef FEATURE_PARK
          deactivate_park();
        #endif // FEATURE_PARK
        if (az_state != IDLE) {
          if (az_slowdown_active) {
            if ((az_state == TIMED_SLOW_DOWN_CW) || (az_state == TIMED_SLOW_DOWN_CCW) || (az_state == SLOW_DOWN_CW) || (az_state == SLOW_DOWN_CCW)) {  // if we're already in timed slow down and we get another stop, do a hard stop
              rotator(DEACTIVATE, CW, 19);
              rotator(DEACTIVATE, CCW, 19);
              az_state = IDLE;
              az_request_queue_state = NONE;
            }
            if ((az_state == SLOW_START_CW) || (az_state == NORMAL_CW)) {
              az_state = INITIALIZE_TIMED_SLOW_DOWN_CW;
              az_request_queue_state = IN_PROGRESS_TIMED;
              az_last_rotate_initiation = millis();
            }
            if ((az_state == SLOW_START_CCW) || (az_state == NORMAL_CCW)) {
              az_state = INITIALIZE_TIMED_SLOW_DOWN_CCW;
              az_request_queue_state = IN_PROGRESS_TIMED;
              az_last_rotate_initiation = millis();
            }

          } else {
            rotator(DEACTIVATE, CW, 20);
            rotator(DEACTIVATE, CCW, 20);
            az_state = IDLE;
            az_request_queue_state = NONE;
          }
        } else {
          az_request_queue_state = NONE; // nothing to do - we clear the queue
        }
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        if (debug_mode) {
          control_port->println();
        }
        #endif // DEBUG_SERVICE_REQUEST_QUEUE
        break; // REQUEST_STOP

      case (REQUEST_AZIMUTH):
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        debug.print("REQUEST_AZIMUTH");
        #endif // DEBUG_SERVICE_REQUEST_QUEUE
        if ((az_request_parm >= 0) && (az_request_parm <= 360)) {
          target_azimuth = az_request_parm;
          target_raw_azimuth = az_request_parm;
          if (target_azimuth == 360) {
            target_azimuth = 0;
          }
          if ((target_azimuth > (azimuth - (AZIMUTH_TOLERANCE))) && (target_azimuth < (azimuth + (AZIMUTH_TOLERANCE)))) {
            #ifdef DEBUG_SERVICE_REQUEST_QUEUE
            debug.print(" request within tolerance");
            #endif // DEBUG_SERVICE_REQUEST_QUEUE
            within_tolerance_flag = 1;
            // az_request_queue_state = NONE;
            if (az_state != IDLE){
              submit_request(AZ, REQUEST_STOP, 0, 137);
            } else {
              az_request_queue_state = NONE;
            }
          } else {  // target azimuth is not within tolerance, we need to rotate
            #ifdef DEBUG_SERVICE_REQUEST_QUEUE
            debug.print(" ->A");
            #endif // DEBUG_SERVICE_REQUEST_QUEUE
            work_target_raw_azimuth = target_azimuth;
            #ifdef DEBUG_SERVICE_REQUEST_QUEUE
            debug.print(" work_target_raw_azimuth:");
            debug.print(work_target_raw_azimuth);
            debug.print(" azimuth_starting_point:");
            debug.print(configuration.azimuth_starting_point);
            debug.print(" ");
            #endif // DEBUG_SERVICE_REQUEST_QUEUE

            if (work_target_raw_azimuth < (configuration.azimuth_starting_point)) {
              work_target_raw_azimuth = work_target_raw_azimuth + 360;
              target_raw_azimuth = work_target_raw_azimuth;
              #ifdef DEBUG_SERVICE_REQUEST_QUEUE
              debug.print("->B");
              #endif // DEBUG_SERVICE_REQUEST_QUEUE
            }
            if ((work_target_raw_azimuth + 360) < ((configuration.azimuth_starting_point + configuration.azimuth_rotation_capability))) { // is there a second possible heading in overlap?
              if (abs(raw_azimuth - work_target_raw_azimuth) < abs((work_target_raw_azimuth + 360) - raw_azimuth)) { // is second possible heading closer?
                #ifdef DEBUG_SERVICE_REQUEST_QUEUE
                debug.print("->C");
                #endif // DEBUG_SERVICE_REQUEST_QUEUE
                if (work_target_raw_azimuth  > raw_azimuth) { // not closer, use position in non-overlap
                  direction_to_go = CW;
                  #ifdef DEBUG_SERVICE_REQUEST_QUEUE
                  debug.print("->CW!");
                  #endif // DEBUG_SERVICE_REQUEST_QUEUE
                } else {
                  direction_to_go = CCW;
                  #ifdef DEBUG_SERVICE_REQUEST_QUEUE
                  debug.print("->CCW!");
                  #endif // DEBUG_SERVICE_REQUEST_QUEUE
                }
              } else { // go to position in overlap
                #ifdef DEBUG_SERVICE_REQUEST_QUEUE
                debug.print("->D");
                #endif // DEBUG_SERVICE_REQUEST_QUEUE
                target_raw_azimuth = work_target_raw_azimuth + 360;
                if ((work_target_raw_azimuth + 360) > raw_azimuth) {
                  direction_to_go = CW;
                  #ifdef DEBUG_SERVICE_REQUEST_QUEUE
                  debug.print("->CW!");
                  #endif // DEBUG_SERVICE_REQUEST_QUEUE
                } else {
                  direction_to_go = CCW;
                  #ifdef DEBUG_SERVICE_REQUEST_QUEUE
                  debug.print("->CCW!");
                  #endif // DEBUG_SERVICE_REQUEST_QUEUE
                }
              }
            } else {  // no possible second heading in overlap
              #ifdef DEBUG_SERVICE_REQUEST_QUEUE
              debug.print("->E");
              #endif // DEBUG_SERVICE_REQUEST_QUEUE
              if (work_target_raw_azimuth  > raw_azimuth) {
                direction_to_go = CW;
              } else {
                direction_to_go = CCW;
              }
            }
          }
        } else {
          #ifdef DEBUG_SERVICE_REQUEST_QUEUE
              debug.print("->F");
          #endif // DEBUG_SERVICE_REQUEST_QUEUE
          if ((az_request_parm > 360) && (az_request_parm <= ((configuration.azimuth_starting_point + configuration.azimuth_rotation_capability)))) {
            target_azimuth = az_request_parm - 360;
            target_raw_azimuth = az_request_parm;
            if (az_request_parm > raw_azimuth) {
              direction_to_go = CW;
            } else {
              direction_to_go = CCW;
            }
          } else {
            #ifdef DEBUG_SERVICE_REQUEST_QUEUE
                debug.print(" error: bogus azimuth request:");
                debug.print(az_request_parm);
                debug.println("");
            #endif // DEBUG_SERVICE_REQUEST_QUEUE
            rotator(DEACTIVATE, CW, 21);
            rotator(DEACTIVATE, CCW, 21);
            az_state = IDLE;
            az_request_queue_state = NONE;         
            return;
          }
        }
        if (direction_to_go == CW) {
          if (((az_state == SLOW_START_CCW) || (az_state == NORMAL_CCW) || (az_state == SLOW_DOWN_CCW) || (az_state == TIMED_SLOW_DOWN_CCW)) && (az_slowstart_active)) {
            az_state = INITIALIZE_DIR_CHANGE_TO_CW;
            #ifdef DEBUG_SERVICE_REQUEST_QUEUE
            debug.print(" INITIALIZE_DIR_CHANGE_TO_CW");
            #endif // DEBUG_SERVICE_REQUEST_QUEUE
          } else {
            if ((az_state != INITIALIZE_SLOW_START_CW) && (az_state != SLOW_START_CW) && (az_state != NORMAL_CW)) { // if we're already rotating CW, don't do anything
              // rotator(ACTIVATE,CW, 22);
              if (az_slowstart_active) {
                az_state = INITIALIZE_SLOW_START_CW;
              } else { az_state = INITIALIZE_NORMAL_CW; };
            }
          }
        }
        if (direction_to_go == CCW) {
          if (((az_state == SLOW_START_CW) || (az_state == NORMAL_CW) || (az_state == SLOW_DOWN_CW) || (az_state == TIMED_SLOW_DOWN_CW)) && (az_slowstart_active)) {
            az_state = INITIALIZE_DIR_CHANGE_TO_CCW;
            #ifdef DEBUG_SERVICE_REQUEST_QUEUE
            debug.print(" INITIALIZE_DIR_CHANGE_TO_CCW");
            #endif // DEBUG_SERVICE_REQUEST_QUEUE
          } else {
            if ((az_state != INITIALIZE_SLOW_START_CCW) && (az_state != SLOW_START_CCW) && (az_state != NORMAL_CCW)) { // if we're already rotating CCW, don't do anything
              // rotator(ACTIVATE,CCW, 23);
              if (az_slowstart_active) {
                az_state = INITIALIZE_SLOW_START_CCW;
              } else { az_state = INITIALIZE_NORMAL_CCW; };
            }
          }
        }
        if (!within_tolerance_flag) {
          az_request_queue_state = IN_PROGRESS_TO_TARGET;
          az_last_rotate_initiation = millis();
        }
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        if (debug_mode) {
          control_port->println();
        }
        #endif // DEBUG_SERVICE_REQUEST_QUEUE
        break; // REQUEST_AZIMUTH

      case (REQUEST_AZIMUTH_RAW):
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        debug.print("REQUEST_AZIMUTH_RAW");
        #endif // DEBUG_SERVICE_REQUEST_QUEUE
        target_raw_azimuth = az_request_parm;
        target_azimuth = target_raw_azimuth;
        if (target_azimuth >= 360) {
          target_azimuth = target_azimuth - 360;
        }

        if (((abs(raw_azimuth - target_raw_azimuth) < (AZIMUTH_TOLERANCE))) && (az_state == IDLE)) {
          #ifdef DEBUG_SERVICE_REQUEST_QUEUE
          debug.print(" request within tolerance");
          #endif // DEBUG_SERVICE_REQUEST_QUEUE
          if (az_state != IDLE){
            submit_request(AZ, REQUEST_STOP, 0, 138);
          } else {
            az_request_queue_state = NONE;
          }
          within_tolerance_flag = 1;
        } else {
          if (target_raw_azimuth > raw_azimuth) {
            if (((az_state == SLOW_START_CCW) || (az_state == NORMAL_CCW) || (az_state == SLOW_DOWN_CCW) || (az_state == TIMED_SLOW_DOWN_CCW)) && (az_slowstart_active)) {
              az_state = INITIALIZE_DIR_CHANGE_TO_CW;
              #ifdef DEBUG_SERVICE_REQUEST_QUEUE
              debug.print(" INITIALIZE_DIR_CHANGE_TO_CW");
              #endif // DEBUG_SERVICE_REQUEST_QUEUE
            } else {
              if ((az_state != INITIALIZE_SLOW_START_CW) && (az_state != SLOW_START_CW) && (az_state != NORMAL_CW)) { // if we're already rotating CW, don't do anything
                if (az_slowstart_active) {
                  az_state = INITIALIZE_SLOW_START_CW;
                } else { az_state = INITIALIZE_NORMAL_CW; };
              }
            }
          }
          if (target_raw_azimuth < raw_azimuth) {
            if (((az_state == SLOW_START_CW) || (az_state == NORMAL_CW) || (az_state == SLOW_DOWN_CW) || (az_state == TIMED_SLOW_DOWN_CW)) && (az_slowstart_active)) {
              az_state = INITIALIZE_DIR_CHANGE_TO_CCW;
              #ifdef DEBUG_SERVICE_REQUEST_QUEUE
              debug.print(" INITIALIZE_DIR_CHANGE_TO_CCW");
              #endif // DEBUG_SERVICE_REQUEST_QUEUE
            } else {
              if ((az_state != INITIALIZE_SLOW_START_CCW) && (az_state != SLOW_START_CCW) && (az_state != NORMAL_CCW)) { // if we're already rotating CCW, don't do anything
                if (az_slowstart_active) {
                  az_state = INITIALIZE_SLOW_START_CCW;
                } else { az_state = INITIALIZE_NORMAL_CCW; };
              }
            }
          }
          if (!within_tolerance_flag) {
            az_request_queue_state = IN_PROGRESS_TO_TARGET;
            az_last_rotate_initiation = millis();
          }
        }
      #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        if (debug_mode) {
          control_port->println();
        }
      #endif // DEBUG_SERVICE_REQUEST_QUEUE
        break; // REQUEST_AZIMUTH_RAW

      case (REQUEST_CW):
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        debug.print("REQUEST_CW");
        #endif // DEBUG_SERVICE_REQUEST_QUEUE
        change_tracking(DEACTIVATE_ALL);
        #ifdef FEATURE_PARK
          deactivate_park();
        #endif // FEATURE_PARK
        if (((az_state == SLOW_START_CCW) || (az_state == NORMAL_CCW) || (az_state == SLOW_DOWN_CCW) || (az_state == TIMED_SLOW_DOWN_CCW)) && (az_slowstart_active)) {
          az_state = INITIALIZE_DIR_CHANGE_TO_CW;
          #ifdef DEBUG_SERVICE_REQUEST_QUEUE
            debug.print(" INITIALIZE_DIR_CHANGE_TO_CW");
          #endif // DEBUG_SERVICE_REQUEST_QUEUE
        } else {
          if ((az_state != SLOW_START_CW) && (az_state != NORMAL_CW)) {
            // rotator(ACTIVATE,CW, 24);
            if (az_slowstart_active) {
              az_state = INITIALIZE_SLOW_START_CW;
            } else { 
              az_state = INITIALIZE_NORMAL_CW;
            };
          }
        }
        az_request_queue_state = NONE;
        az_last_rotate_initiation = millis();
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        if (debug_mode) {
          control_port->println();
        }
        #endif // DEBUG_SERVICE_REQUEST_QUEUE
        break; // REQUEST_CW

      case (REQUEST_CCW):
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        debug.print("REQUEST_CCW");
        #endif // DEBUG_SERVICE_REQUEST_QUEUE
        change_tracking(DEACTIVATE_ALL);
        #ifdef FEATURE_PARK
          deactivate_park();
        #endif // FEATURE_PARK
        if (((az_state == SLOW_START_CW) || (az_state == NORMAL_CW) || (az_state == SLOW_DOWN_CW) || (az_state == TIMED_SLOW_DOWN_CW)) && (az_slowstart_active)) {
          az_state = INITIALIZE_DIR_CHANGE_TO_CCW;
          #ifdef DEBUG_SERVICE_REQUEST_QUEUE
            debug.print(" INITIALIZE_DIR_CHANGE_TO_CCW");
          #endif // DEBUG_SERVICE_REQUEST_QUEUE
        } else {
          if ((az_state != SLOW_START_CCW) && (az_state != NORMAL_CCW)) {
            // rotator(ACTIVATE,CCW, 25);
            if (az_slowstart_active) {
              az_state = INITIALIZE_SLOW_START_CCW;
            } else { az_state = INITIALIZE_NORMAL_CCW; };
          }
        }
        az_request_queue_state = NONE;
        az_last_rotate_initiation = millis();
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        if (debug_mode) {
          control_port->println();
        }
        #endif // DEBUG_SERVICE_REQUEST_QUEUE
        break; // REQUEST_CCW

      case (REQUEST_KILL):
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        debug.print("REQUEST_KILL");
        #endif // DEBUG_SERVICE_REQUEST_QUEUE
        change_tracking(DEACTIVATE_ALL);
        #ifdef FEATURE_PARK
          deactivate_park();
        #endif // FEATURE_PARK
        rotator(DEACTIVATE, CW, 26);
        rotator(DEACTIVATE, CCW, 26);
        az_state = IDLE;
        az_request_queue_state = NONE;
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        debug.println("");
        #endif // DEBUG_SERVICE_REQUEST_QUEUE
        break; // REQUEST_KILL
    } /* switch */

    #ifdef FEATURE_LCD_DISPLAY
    //if (az_request_queue_state != IN_QUEUE) {push_lcd_update = 1;}
    #endif //FEATURE_LCD_DISPLAY
  }

  #ifdef FEATURE_ELEVATION_CONTROL
  if (el_request_queue_state == IN_QUEUE) {

    #ifdef FEATURE_POWER_SWITCH
    last_activity_time = millis();
    #endif //FEATURE_POWER_SWITCH

    within_tolerance_flag = 0;
    #ifdef DEBUG_SERVICE_REQUEST_QUEUE
    debug.print("service_request_queue: EL ");
    #endif // DEBUG_SERVICE_REQUEST_QUEUE
    switch (el_request) {
      case (REQUEST_ELEVATION):
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        debug.print("REQUEST_ELEVATION ");
        #endif // DEBUG_SERVICE_REQUEST_QUEUE
        target_elevation = el_request_parm;

        if (target_elevation > (ELEVATION_MAXIMUM_DEGREES)) {
          target_elevation = ELEVATION_MAXIMUM_DEGREES;
          #ifdef DEBUG_SERVICE_REQUEST_QUEUE
          if (debug_mode) {
            debug.print(F("REQUEST_ELEVATION: target_elevation > ELEVATION_MAXIMUM_DEGREES"));
          }
          #endif // DEBUG_SERVICE_REQUEST_QUEUE
        }

        #ifdef OPTION_EL_MANUAL_ROTATE_LIMITS
        if (target_elevation < (EL_MANUAL_ROTATE_DOWN_LIMIT)) {
          target_elevation = EL_MANUAL_ROTATE_DOWN_LIMIT;
          #ifdef DEBUG_SERVICE_REQUEST_QUEUE
          if (debug_mode) {
            debug.print(F("REQUEST_ELEVATION: target_elevation < EL_MANUAL_ROTATE_DOWN_LIMIT"));
          }
          #endif // DEBUG_SERVICE_REQUEST_QUEUE
        }
        if (target_elevation > (EL_MANUAL_ROTATE_UP_LIMIT)) {
          target_elevation = EL_MANUAL_ROTATE_UP_LIMIT;
          #ifdef DEBUG_SERVICE_REQUEST_QUEUE
          if (debug_mode) {
            debug.print(F("REQUEST_ELEVATION: target_elevation > EL_MANUAL_ROTATE_UP_LIMIT"));
          }
          #endif // DEBUG_SERVICE_REQUEST_QUEUE
        }
        #endif // OPTION_EL_MANUAL_ROTATE_LIMITS

        if (abs(target_elevation - elevation) < (ELEVATION_TOLERANCE)) {
          #ifdef DEBUG_SERVICE_REQUEST_QUEUE
          if (debug_mode) {
            debug.print(F("requested elevation within tolerance\n"));
          }
          #endif // DEBUG_SERVICE_REQUEST_QUEUE
          within_tolerance_flag = 1;
          el_request_queue_state = NONE;
        } else {
          if (target_elevation > elevation) {
            if (((el_state == SLOW_START_DOWN) || (el_state == NORMAL_DOWN) || (el_state == SLOW_DOWN_DOWN) || (el_state == TIMED_SLOW_DOWN_DOWN)) && (el_slowstart_active)) {
              el_state = INITIALIZE_DIR_CHANGE_TO_UP;
                #ifdef DEBUG_SERVICE_REQUEST_QUEUE
              if (debug_mode) {
                debug.print(F(" INITIALIZE_DIR_CHANGE_TO_UP\n"));
              }
                #endif // DEBUG_SERVICE_REQUEST_QUEUE
            } else {
              if ((el_state != INITIALIZE_SLOW_START_UP) && (el_state != SLOW_START_UP) && (el_state != NORMAL_UP)) { // if we're already rotating UP, don't do anything
                if (el_slowstart_active) {
                  el_state = INITIALIZE_SLOW_START_UP;
                } else { el_state = INITIALIZE_NORMAL_UP; };
              }
            }
          } // (target_elevation > elevation)
          if (target_elevation < elevation) {
            if (((el_state == SLOW_START_UP) || (el_state == NORMAL_UP) || (el_state == SLOW_DOWN_UP) || (el_state == TIMED_SLOW_DOWN_UP)) && (el_slowstart_active)) {
              el_state = INITIALIZE_DIR_CHANGE_TO_DOWN;
              #ifdef DEBUG_SERVICE_REQUEST_QUEUE
              if (debug_mode) {
                debug.print(F(" INITIALIZE_DIR_CHANGE_TO_DOWN\n"));
              }
              #endif // DEBUG_SERVICE_REQUEST_QUEUE
            } else {
              if ((el_state != INITIALIZE_SLOW_START_DOWN) && (el_state != SLOW_START_DOWN) && (el_state != NORMAL_DOWN)) { // if we're already rotating DOWN, don't do anything
                if (el_slowstart_active) {
                  el_state = INITIALIZE_SLOW_START_DOWN;
                } else { el_state = INITIALIZE_NORMAL_DOWN; };
              }
            }
          }  // (target_elevation < elevation)
        }  // (abs(target_elevation - elevation) < ELEVATION_TOLERANCE)
        if (!within_tolerance_flag) {
          el_request_queue_state = IN_PROGRESS_TO_TARGET;
          el_last_rotate_initiation = millis();
        }
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        if (debug_mode) {
          control_port->println();
        }
        #endif // DEBUG_SERVICE_REQUEST_QUEUE
        break; // REQUEST_ELEVATION

      case (REQUEST_UP):
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        if (debug_mode) {
          debug.print(F("REQUEST_UP\n"));
        }
        #endif // DEBUG_SERVICE_REQUEST_QUEUE
        change_tracking(DEACTIVATE_ALL);
        #ifdef FEATURE_PARK
          deactivate_park();
        #endif // FEATURE_PARK
        if (((el_state == SLOW_START_DOWN) || (el_state == NORMAL_DOWN) || (el_state == SLOW_DOWN_DOWN) || (el_state == TIMED_SLOW_DOWN_DOWN)) && (el_slowstart_active)) {
          el_state = INITIALIZE_DIR_CHANGE_TO_UP;
          #ifdef DEBUG_SERVICE_REQUEST_QUEUE
          if (debug_mode) {
            debug.print(F("service_request_queue: INITIALIZE_DIR_CHANGE_TO_UP\n"));
          }
          #endif // DEBUG_SERVICE_REQUEST_QUEUE
        } else {
          if ((el_state != SLOW_START_UP) && (el_state != NORMAL_UP)) {
            if (el_slowstart_active) {
              el_state = INITIALIZE_SLOW_START_UP;
            } else { el_state = INITIALIZE_NORMAL_UP; };
          }
        }
        el_request_queue_state = NONE;
        el_last_rotate_initiation = millis();
          #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        if (debug_mode) {
          control_port->println();
        }
          #endif // DEBUG_SERVICE_REQUEST_QUEUE
        break; // REQUEST_UP

      case (REQUEST_DOWN):
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        if (debug_mode) {
          debug.print(F("REQUEST_DOWN\n"));
        }
        #endif // DEBUG_SERVICE_REQUEST_QUEUE
        change_tracking(DEACTIVATE_ALL);
        #ifdef FEATURE_PARK
          deactivate_park();
        #endif // FEATURE_PARK
        if (((el_state == SLOW_START_UP) || (el_state == NORMAL_UP) || (el_state == SLOW_DOWN_UP) || (el_state == TIMED_SLOW_DOWN_UP)) && (el_slowstart_active)) {
          el_state = INITIALIZE_DIR_CHANGE_TO_DOWN;
          #ifdef DEBUG_SERVICE_REQUEST_QUEUE
          if (debug_mode) {
            debug.print(F("service_request_queue: INITIALIZE_DIR_CHANGE_TO_DOWN\n"));
          }
          #endif // DEBUG_SERVICE_REQUEST_QUEUE
        } else {
          if ((el_state != SLOW_START_DOWN) && (el_state != NORMAL_DOWN)) {
            if (el_slowstart_active) {
              el_state = INITIALIZE_SLOW_START_DOWN;
            } else { el_state = INITIALIZE_NORMAL_DOWN; };
          }
        }
        el_request_queue_state = NONE;
        el_last_rotate_initiation = millis();
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        if (debug_mode) {
          control_port->println();
        }
        #endif // DEBUG_SERVICE_REQUEST_QUEUE
        break; // REQUEST_DOWN

      case (REQUEST_STOP):
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        if (debug_mode) {
          debug.print(F("REQUEST_STOP\n"));
        }
        #endif // DEBUG_SERVICE_REQUEST_QUEUE
        change_tracking(DEACTIVATE_ALL);
        #ifdef FEATURE_PARK
          deactivate_park();
        #endif // FEATURE_PARK
        if (el_state != IDLE) {
          if (el_slowdown_active) {
            if ((el_state == TIMED_SLOW_DOWN_UP) || (el_state == TIMED_SLOW_DOWN_DOWN) || (el_state == SLOW_DOWN_UP) || (el_state == SLOW_DOWN_DOWN)) {  // if we're already in timed slow down and we get another stop, do a hard stop
              rotator(DEACTIVATE, UP, 27);
              rotator(DEACTIVATE, DOWN, 27);
              el_state = IDLE;
              el_request_queue_state = NONE;
            }
            if ((el_state == SLOW_START_UP) || (el_state == NORMAL_UP)) {
              el_state = INITIALIZE_TIMED_SLOW_DOWN_UP;
              el_request_queue_state = IN_PROGRESS_TIMED;
              el_last_rotate_initiation = millis();
            }
            if ((el_state == SLOW_START_DOWN) || (el_state == NORMAL_DOWN)) {
              el_state = INITIALIZE_TIMED_SLOW_DOWN_DOWN;
              el_request_queue_state = IN_PROGRESS_TIMED;
              el_last_rotate_initiation = millis();
            }
          } else {
            rotator(DEACTIVATE, UP, 28);
            rotator(DEACTIVATE, DOWN, 28);
            el_state = IDLE;
            el_request_queue_state = NONE;
          }
        } else {
          el_request_queue_state = NONE; // nothing to do, we're already in IDLE state
        }
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        if (debug_mode) {
          control_port->println();
        }
        #endif // DEBUG_SERVICE_REQUEST_QUEUE
        break; // REQUEST_STOP

      case (REQUEST_KILL):
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        if (debug_mode) {
          debug.print(F("REQUEST_KILL\n"));
        }
        #endif // DEBUG_SERVICE_REQUEST_QUEUE
        change_tracking(DEACTIVATE_ALL);
        #ifdef FEATURE_PARK
          deactivate_park();
        #endif // FEATURE_PARK
        rotator(DEACTIVATE, UP, 29);
        rotator(DEACTIVATE, DOWN, 29);
        el_state = IDLE;
        el_request_queue_state = NONE;
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        if (debug_mode) {
          control_port->println();
        }
        #endif // DEBUG_SERVICE_REQUEST_QUEUE
        break; // REQUEST_KILL
    } /* switch */

    #ifdef FEATURE_LCD_DISPLAY
    //if (el_request_queue_state != IN_QUEUE) {push_lcd_update = 1;}
    #endif //FEATURE_LCD_DISPLAY

  } // (el_request_queue_state == IN_QUEUE)
  #endif // FEATURE_ELEVATION_CONTROL


} /* service_request_queue */

// --------------------------------------------------------------
void check_for_dirty_configuration(){

  #ifdef DEBUG_PROCESSES
    service_process_debug(DEBUG_PROCESSES_PROCESS_ENTER,PROCESS_CHECK_FOR_DIRTY_CONFIGURATION);
  #endif     

  static unsigned long last_config_write_time = 0;

  if ((configuration_dirty) && ((millis() - last_config_write_time) > ((unsigned long)EEPROM_WRITE_DIRTY_CONFIG_TIME * 1000))) {
    write_settings_to_eeprom();
    last_config_write_time = millis();
  }

  #ifdef DEBUG_PROCESSES
    service_process_debug(DEBUG_PROCESSES_PROCESS_EXIT,PROCESS_CHECK_FOR_DIRTY_CONFIGURATION);
  #endif     

}

// --------------------------------------------------------------
byte current_az_state(){

  if ((az_state == SLOW_START_CW) || (az_state == NORMAL_CW) || (az_state == SLOW_DOWN_CW) || (az_state == TIMED_SLOW_DOWN_CW)) {
    return ROTATING_CW;
  }
  if ((az_state == SLOW_START_CCW) || (az_state == NORMAL_CCW) || (az_state == SLOW_DOWN_CCW) || (az_state == TIMED_SLOW_DOWN_CCW)) {
    return ROTATING_CCW;
  }
  return NOT_DOING_ANYTHING;

}
// --------------------------------------------------------------
#ifdef FEATURE_ELEVATION_CONTROL
byte current_el_state(){

  if ((el_state == SLOW_START_UP) || (el_state == NORMAL_UP) || (el_state == SLOW_DOWN_UP) || (el_state == TIMED_SLOW_DOWN_UP)) {
    return ROTATING_UP;
  }
  if ((el_state == SLOW_START_DOWN) || (el_state == NORMAL_DOWN) || (el_state == SLOW_DOWN_DOWN) || (el_state == TIMED_SLOW_DOWN_DOWN)) {
    return ROTATING_DOWN;
  }
  return NOT_DOING_ANYTHING;

}
#endif // FEATURE_ELEVATION_CONTROL
// --------------------------------------------------------------
#ifdef FEATURE_AZ_POSITION_PULSE_INPUT
void az_position_pulse_interrupt_handler(){


 #ifdef DEBUG_POSITION_PULSE_INPUT
  // az_position_pule_interrupt_handler_flag++;
  az_pulse_counter++;
  #endif // DEBUG_POSITION_PULSE_INPUT

  if (current_az_state() == ROTATING_CW) {
    az_position_pulse_input_azimuth += (float)AZ_POSITION_PULSE_DEG_PER_PULSE;
    last_known_az_state = ROTATING_CW;
  } else {
    if (current_az_state() == ROTATING_CCW) {
      az_position_pulse_input_azimuth -= (float)AZ_POSITION_PULSE_DEG_PER_PULSE;
      last_known_az_state = ROTATING_CCW;
    } else {
          #ifndef OPTION_PULSE_IGNORE_AMBIGUOUS_PULSES
      if (last_known_az_state == ROTATING_CW) {
        az_position_pulse_input_azimuth += (float)AZ_POSITION_PULSE_DEG_PER_PULSE;
      } else {
        if (last_known_az_state == ROTATING_CCW) {
          az_position_pulse_input_azimuth -= (float)AZ_POSITION_PULSE_DEG_PER_PULSE;
        }
      }
            #endif // OPTION_PULSE_IGNORE_AMBIGUOUS_PULSES
            #ifdef DEBUG_POSITION_PULSE_INPUT
      az_pulse_counter_ambiguous++;
            #endif // DEBUG_POSITION_PULSE_INPUT
    }
  }

  #ifdef OPTION_AZ_POSITION_PULSE_HARD_LIMIT
    if (az_position_pulse_input_azimuth < configuration.azimuth_starting_point) {
      az_position_pulse_input_azimuth = configuration.azimuth_starting_point;
    }
    if (az_position_pulse_input_azimuth > (configuration.azimuth_starting_point + configuration.azimuth_rotation_capability)) {
      az_position_pulse_input_azimuth = (configuration.azimuth_starting_point + configuration.azimuth_rotation_capability);
    }
    #else
    if (az_position_pulse_input_azimuth < 0) {
      az_position_pulse_input_azimuth += 360.0;
    }
    if (az_position_pulse_input_azimuth >= 360) {
      az_position_pulse_input_azimuth -= 360.0;
    }
  #endif // OPTION_AZ_POSITION_PULSE_HARD_LIMIT

} /* az_position_pulse_interrupt_handler */
#endif // FEATURE_AZ_POSITION_PULSE_INPUT
// --------------------------------------------------------------
#ifdef FEATURE_ELEVATION_CONTROL
#ifdef FEATURE_EL_POSITION_PULSE_INPUT
void el_position_pulse_interrupt_handler(){

  #ifdef DEBUG_POSITION_PULSE_INPUT
  // el_position_pule_interrupt_handler_flag++;
  el_pulse_counter++;
  #endif // DEBUG_POSITION_PULSE_INPUT



  #ifdef OPTION_EL_PULSE_DEBOUNCE //---------------------------------------------
  if ((millis()-last_el_pulse_debounce) > EL_POSITION_PULSE_DEBOUNCE) {
    if (current_el_state() == ROTATING_UP) {
      el_position_pulse_input_elevation += EL_POSITION_PULSE_DEG_PER_PULSE;
      last_known_el_state = ROTATING_UP;
    } else {
      if (current_el_state() == ROTATING_DOWN) {
        el_position_pulse_input_elevation -= EL_POSITION_PULSE_DEG_PER_PULSE;
        last_known_el_state = ROTATING_DOWN;
      } else {
        #ifndef OPTION_PULSE_IGNORE_AMBIGUOUS_PULSES
        if (last_known_el_state == ROTATING_UP) {
          el_position_pulse_input_elevation += EL_POSITION_PULSE_DEG_PER_PULSE;
        } else {
          if (last_known_el_state == ROTATING_DOWN) {
            el_position_pulse_input_elevation -= EL_POSITION_PULSE_DEG_PER_PULSE;
          }
        }
        #endif // OPTION_PULSE_IGNORE_AMBIGUOUS_PULSES
        #ifdef DEBUG_POSITION_PULSE_INPUT
        el_pulse_counter_ambiguous++;
        #endif // DEBUG_POSITION_PULSE_INPUT
      }
    }
    last_el_pulse_debounce = millis();
  }

  #else //OPTION_EL_PULSE_DEBOUNCE -----------------------



  if (current_el_state() == ROTATING_UP) {
    el_position_pulse_input_elevation += EL_POSITION_PULSE_DEG_PER_PULSE;
    last_known_el_state = ROTATING_UP;
  } else {
    if (current_el_state() == ROTATING_DOWN) {
      el_position_pulse_input_elevation -= EL_POSITION_PULSE_DEG_PER_PULSE;
      last_known_el_state = ROTATING_DOWN;
    } else {
      #ifndef OPTION_PULSE_IGNORE_AMBIGUOUS_PULSES
      if (last_known_el_state == ROTATING_UP) {
        el_position_pulse_input_elevation += EL_POSITION_PULSE_DEG_PER_PULSE;
      } else {
        if (last_known_el_state == ROTATING_DOWN) {
          el_position_pulse_input_elevation -= EL_POSITION_PULSE_DEG_PER_PULSE;
        }
      }
      #endif // OPTION_PULSE_IGNORE_AMBIGUOUS_PULSES
      #ifdef DEBUG_POSITION_PULSE_INPUT
      el_pulse_counter_ambiguous++;
      #endif // DEBUG_POSITION_PULSE_INPUT
    }
  }
  #endif //OPTION_EL_PULSE_DEBOUNCE --------------------------

  #ifdef OPTION_EL_POSITION_PULSE_HARD_LIMIT
  if (el_position_pulse_input_elevation < 0) {
    el_position_pulse_input_elevation = 0;
  }
  if (el_position_pulse_input_elevation > ELEVATION_MAXIMUM_DEGREES) {
    el_position_pulse_input_elevation = ELEVATION_MAXIMUM_DEGREES;
  }
  #endif // OPTION_EL_POSITION_PULSE_HARD_LIMIT


} /* el_position_pulse_interrupt_handler */
#endif // FEATURE_EL_POSITION_PULSE_INPUT
#endif // FEATURE_ELEVATION_CONTROL
// --------------------------------------------------------------------------
// --------------------------------------------------------------------------
#ifdef FEATURE_AZIMUTH_CORRECTION
float correct_azimuth(float azimuth_in){

  if (sizeof(azimuth_calibration_from) != sizeof(azimuth_calibration_to)) {
    return azimuth_in;
  }
  for (unsigned int x = 0; x < (sizeof(azimuth_calibration_from) - 2); x++) {
    if ((azimuth_in >= azimuth_calibration_from[x]) && (azimuth_in <= azimuth_calibration_from[x + 1])) {
      //return (map(azimuth_in * 10, azimuth_calibration_from[x] * 10, azimuth_calibration_from[x + 1] * 10, azimuth_calibration_to[x] * 10, azimuth_calibration_to[x + 1] * 10)) / 10.0;
      return (azimuth_in - azimuth_calibration_from[x]) * (azimuth_calibration_to[x+1] - azimuth_calibration_to[x]) / (azimuth_calibration_from[x + 1] - azimuth_calibration_from[x]) + azimuth_calibration_to[x];
    }
  }
  return(azimuth_in);

}
#endif // FEATURE_AZIMUTH_CORRECTION
// --------------------------------------------------------------------------
#ifdef FEATURE_ELEVATION_CORRECTION
float correct_elevation(float elevation_in){


  if (sizeof(elevation_calibration_from) != sizeof(elevation_calibration_to)) {
    return elevation_in;
  }
  for (int x = 0; x < (sizeof(elevation_calibration_from) - 2); x++) {
    if ((elevation_in >= elevation_calibration_from[x]) && (elevation_in <= elevation_calibration_from[x + 1])) {
      // changed this from map() 2015-03-28 due to it blowing up at compile time in Arduino 1.6.1
      return (elevation_in - elevation_calibration_from[x]) * (elevation_calibration_to[x+1] - elevation_calibration_to[x]) / (elevation_calibration_from[x + 1] - elevation_calibration_from[x]) + elevation_calibration_to[x];
    }
  }

  return(elevation_in);


}
#endif // FEATURE_ELEVATION_CORRECTION
// --------------------------------------------------------------------------
#ifdef FEATURE_JOYSTICK_CONTROL
void check_joystick(){

  int joystick_x = 0;
  int joystick_y = 0;

  static int joystick_resting_x = 0;
  static int joystick_resting_y = 0;

  static unsigned long last_joystick_az_action_time = 0;

  static byte joystick_azimuth_rotation = NOT_DOING_ANYTHING;

  #ifdef FEATURE_ELEVATION_CONTROL
  static byte joystick_elevation_rotation = NOT_DOING_ANYTHING;
  static unsigned long last_joystick_el_action_time = 0;
  #endif // FEATURE_ELEVATION_CONTROL

  if ((joystick_resting_x == 0) || (joystick_resting_y == 0)) {  // initialize the resting readings if this is our first time here

    joystick_resting_x = analogReadEnhanced(pin_joystick_x);
    joystick_resting_y = analogReadEnhanced(pin_joystick_y);

  } else {

    joystick_x = analogReadEnhanced(pin_joystick_x);
    joystick_y = analogReadEnhanced(pin_joystick_y);

    if ((millis() - last_joystick_az_action_time) > JOYSTICK_WAIT_TIME_MS) {
      #ifdef DEBUG_JOYSTICK
      static unsigned long last_debug_joystick_status = 0;

      if ((debug_mode) && ((millis() - last_debug_joystick_status) > 1000)) {
        debug.print("check_joystick: x: ");
        debug.print(joystick_x);
        debug.print("\ty: ");
        control_port->println(joystick_y);
        last_debug_joystick_status = millis();
      }
      #endif // DEBUG_JOYSTICK

      #ifndef OPTION_JOYSTICK_REVERSE_X_AXIS
      if ((joystick_resting_x - joystick_x) < (joystick_resting_x * -0.2)) {   // left
      #else
      if ((joystick_resting_x - joystick_x) > (joystick_resting_x * 0.2)) {
      #endif
        #ifdef DEBUG_JOYSTICK
        if (debug_mode) {
          control_port->println("check_joystick: L");
        }
          #endif // DEBUG_JOYSTICK
        if (current_az_state() != ROTATING_CCW) {
          submit_request(AZ, REQUEST_CCW, 0, 1);
        }
        joystick_azimuth_rotation = ROTATING_CCW;
        last_joystick_az_action_time = millis();

      } else {
        #ifndef OPTION_JOYSTICK_REVERSE_X_AXIS
        if ((joystick_resting_x - joystick_x) > (joystick_resting_x * 0.2)) {  // right
        #else
        if ((joystick_resting_x - joystick_x) < (joystick_resting_x * -0.2)) {
        #endif
          #ifdef DEBUG_JOYSTICK
          if (debug_mode) {
            control_port->println("check_joystick: R");
          }
            #endif // DEBUG_JOYSTICK
          if (current_az_state() != ROTATING_CW) {
            submit_request(AZ, REQUEST_CW, 0, 2);
          }
          joystick_azimuth_rotation = ROTATING_CW;
          last_joystick_az_action_time = millis();

        } else { // joystick is in X axis resting position
          if (joystick_azimuth_rotation != NOT_DOING_ANYTHING) {
            if (current_az_state() != NOT_DOING_ANYTHING) {
              submit_request(AZ, REQUEST_STOP, 0, 3);
              last_joystick_az_action_time = millis();
            }
            joystick_azimuth_rotation = NOT_DOING_ANYTHING;
          }
        }

      }

    }

    #ifdef FEATURE_ELEVATION_CONTROL
    if ((millis() - last_joystick_el_action_time) > JOYSTICK_WAIT_TIME_MS) {   
      #ifndef OPTION_JOYSTICK_REVERSE_Y_AXIS
      if ((joystick_resting_y - joystick_y) > (joystick_resting_y * 0.2)) {  // down
        #else
      if ((joystick_resting_y - joystick_y) < (joystick_resting_y * -0.2)) {
          #endif
          #ifdef DEBUG_JOYSTICK
        if (debug_mode) {
          control_port->println("check_joystick: D");
        }
          #endif // DEBUG_JOYSTICK
        if (current_el_state() != ROTATING_DOWN) {
          submit_request(EL, REQUEST_DOWN, 0, 4);
        }
        joystick_elevation_rotation = ROTATING_DOWN;
        last_joystick_el_action_time = millis();
      } else {
            #ifndef OPTION_JOYSTICK_REVERSE_Y_AXIS
        if ((joystick_resting_y - joystick_y) < (joystick_resting_y * -0.2)) { // up
          #else
        if ((joystick_resting_y - joystick_y) > (joystick_resting_y * 0.2)) {
            #endif
            #ifdef DEBUG_JOYSTICK
          if (debug_mode) {
            control_port->println("check_joystick: U");
          }
            #endif // DEBUG_JOYSTICK
          if (current_el_state() != ROTATING_UP) {
            submit_request(EL, REQUEST_UP, 0, 5);
          }
          joystick_elevation_rotation = ROTATING_UP;
          last_joystick_el_action_time = millis();

        } else {  // Y axis is in resting position
          if (joystick_elevation_rotation != NOT_DOING_ANYTHING) {
            if (current_el_state() != NOT_DOING_ANYTHING) {
              submit_request(EL, REQUEST_STOP, 0, 6);
              last_joystick_el_action_time = millis();
            }
            joystick_elevation_rotation = NOT_DOING_ANYTHING;
          }
        }
      }
      

    }
    #endif // FEATURE_ELEVATION_CONTROL

  }


} /* check_joystick */
#endif // FEATURE_JOYSTICK_CONTROL
// --------------------------------------------------------------------------

#ifdef FEATURE_ROTATION_INDICATOR_PIN
void service_rotation_indicator_pin(){


  static byte rotation_indication_pin_state = 0;
  static unsigned long time_rotation_went_inactive = 0;

  #ifdef FEATURE_ELEVATION_CONTROL
  if ((!rotation_indication_pin_state) && ((az_state != IDLE) || (el_state != IDLE))) {
    #else
  if ((!rotation_indication_pin_state) && ((az_state != IDLE))) {
      #endif
    if (rotation_indication_pin) {
      digitalWriteEnhanced(rotation_indication_pin, ROTATION_INDICATOR_PIN_ACTIVE_STATE);
    }
    rotation_indication_pin_state = 1;
      #ifdef DEBUG_ROTATION_INDICATION_PIN
    if (debug_mode) {
      debug.print(F("service_rotation_indicator_pin: active\n"));
    }
      #endif
  }

    #ifdef FEATURE_ELEVATION_CONTROL
  if ((rotation_indication_pin_state) && (az_state == IDLE) && (el_state == IDLE)) {
      #else
  if ((rotation_indication_pin_state) && (az_state == IDLE)) {
        #endif
    if (time_rotation_went_inactive == 0) {
      time_rotation_went_inactive = millis();
    } else {
      if ((millis() - time_rotation_went_inactive) >= (((unsigned long)ROTATION_INDICATOR_PIN_TIME_DELAY_SECONDS * 1000) + ((unsigned long)ROTATION_INDICATOR_PIN_TIME_DELAY_MINUTES * 60 * 1000))) {
        if (rotation_indication_pin) {
          digitalWriteEnhanced(rotation_indication_pin, ROTATION_INDICATOR_PIN_INACTIVE_STATE);
        }
        rotation_indication_pin_state = 0;
        time_rotation_went_inactive = 0;
        #ifdef DEBUG_ROTATION_INDICATION_PIN
          if (debug_mode) {
            debug.print(F("service_rotation_indicator_pin: inactive\n"));
          }
        #endif
      }
    }
  }


} /* service_rotation_indicator_pin */
      #endif // FEATURE_ROTATION_INDICATOR_PIN
// --------------------------------------------------------------
#ifdef FEATURE_PARK
void deactivate_park(){

  park_status = NOT_PARKED;
  park_serial_initiated = 0;
}
#endif // FEATURE_PARK

// --------------------------------------------------------------
#ifdef FEATURE_PARK
void initiate_park(){

  #ifdef DEBUG_PARK
    debug.print(F("initiate_park: park initiated\n"));
  #endif // DEBUG_PARK

  byte park_initiated = 0;

  change_tracking(DEACTIVATE_ALL);

  if (abs(raw_azimuth - configuration.park_azimuth) > (AZIMUTH_TOLERANCE)) {
    submit_request(AZ, REQUEST_AZIMUTH_RAW, configuration.park_azimuth, 7);
    park_initiated = 1;
  }

  #ifdef FEATURE_ELEVATION_CONTROL
    if (abs(elevation - configuration.park_elevation) > (ELEVATION_TOLERANCE)) {
      submit_request(EL, REQUEST_ELEVATION, configuration.park_elevation, 8);
      park_initiated = 1;
    }
  #endif // FEATURE_ELEVATION_CONTROL

  if (park_initiated) {
    park_status = PARK_INITIATED;
  } else {
    park_status = PARKED;
  }

} /* initiate_park */
  #endif // FEATURE_PARK

// --------------------------------------------------------------
#ifdef FEATURE_PARK
void service_park(){


  #ifdef DEBUG_LOOP
    control_port->println("service_park()");
    control_port->flush();
  #endif // DEBUG_LOOP

  static byte last_park_status = NOT_PARKED;

  static unsigned long time_first_detect_not_parked = 0;

  if (park_status == PARKED) {

    #if !defined(FEATURE_ELEVATION_CONTROL)
      if (abs(raw_azimuth - configuration.park_azimuth) > (AZIMUTH_TOLERANCE)) {
        if (time_first_detect_not_parked == 0){
          time_first_detect_not_parked = millis();
        } else {
          if ((millis() - time_first_detect_not_parked) > NOT_PARKED_DETECT_TIME_MS){
            park_status = NOT_PARKED;
            #ifdef DEBUG_PARK
              debug.println(F("service_park: NOT_PARKED"));
            #endif // DEBUG_PARK  
            time_first_detect_not_parked = 0;
          }
        }     
      }

    #else

      if ((abs(elevation - configuration.park_elevation) > (ELEVATION_TOLERANCE)) || (abs(raw_azimuth - configuration.park_azimuth) > (AZIMUTH_TOLERANCE))){
        if (time_first_detect_not_parked == 0){
          time_first_detect_not_parked = millis();
        } else {        
          if ((millis() - time_first_detect_not_parked) > NOT_PARKED_DETECT_TIME_MS){
            park_status = NOT_PARKED;
            #ifdef DEBUG_PARK
              debug.println(F("service_park: NOT_PARKED"));
            #endif // DEBUG_PARK  
            time_first_detect_not_parked = 0;
          }
        }  
      }

    #endif // FEATURE_ELEVATION_CONTROL

  }


  // if (park_status == PARKED) {
  //   if (abs(raw_azimuth - configuration.park_azimuth) > (AZIMUTH_TOLERANCE)) {
  //     park_status = NOT_PARKED;
  //     #ifdef DEBUG_PARK
  //       debug.println(F("service_park: az NOT_PARKED"));
  //     #endif // DEBUG_PARK      
  //   }
  //   #ifdef FEATURE_ELEVATION_CONTROL
  //   if (abs(elevation - configuration.park_elevation) > (ELEVATION_TOLERANCE)) {
  //     park_status = NOT_PARKED;
  //     #ifdef DEBUG_PARK
  //       debug.println(F("service_park: el NOT_PARKED"));
  //     #endif // DEBUG_PARK          
  //   }
  //   #endif // FEATURE_ELEVATION_CONTROL
  // }


  if (park_status != last_park_status) {
    switch (park_status) {
      case NOT_PARKED:
        if (park_in_progress_pin) {
          digitalWriteEnhanced(park_in_progress_pin, LOW);
        }
        if (parked_pin) {
          digitalWriteEnhanced(parked_pin, LOW);
        }
        #ifdef DEBUG_PARK
          debug.print(F("service_park: park_in_progress_pin: LOW  parked_pin: LOW\n"));
        #endif // DEBUG_PARK
        break;
      case PARK_INITIATED:
        if (park_in_progress_pin) {
          digitalWriteEnhanced(park_in_progress_pin, HIGH);
        }
        if (parked_pin) {
          digitalWriteEnhanced(parked_pin, LOW);
        }
        #ifdef DEBUG_PARK
          debug.print(F("service_park: park_in_progress_pin: HIGH  parked_pin: LOW\n"));
        #endif // DEBUG_PARK
        break;
      case PARKED:
        if (park_in_progress_pin) {
          digitalWriteEnhanced(park_in_progress_pin, LOW);
        }
        if (parked_pin) {
          digitalWriteEnhanced(parked_pin, HIGH);
        }
        if (park_serial_initiated) {
        #if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
          control_port->println(F("Parked."));
        #endif
          park_serial_initiated = 0;
        }
        #ifdef DEBUG_PARK
          debug.print(F("service_park: park_in_progress_pin: LOW  parked_pin: HIGH\n"));
        #endif // DEBUG_PARK
        break;
    } /* switch */
  }

  last_park_status = park_status;

} /* service_park */
#endif // FEATURE_PARK

// --------------------------------------------------------------

#ifdef FEATURE_LIMIT_SENSE
void check_limit_sense(){

  #ifdef DEBUG_PROCESSES
    service_process_debug(DEBUG_PROCESSES_PROCESS_ENTER,PROCESS_MISC_ADMIN);
  #endif    

  static byte az_limit_tripped = 0;

  #ifdef FEATURE_ELEVATION_CONTROL
  static byte el_limit_tripped = 0;
  #endif // FEATURE_ELEVATION_CONTROL

  if (az_limit_sense_pin) {
    if (digitalReadEnhanced(az_limit_sense_pin) == 0) {
      if (!az_limit_tripped) {
        submit_request(AZ, REQUEST_KILL, 0, 9);
        az_limit_tripped = 1;
        #ifdef DEBUG_LIMIT_SENSE
          debug.print(F("check_limit_sense: az limit tripped\n"));
        #endif // DEBUG_LIMIT_SENSE
      }
    } else {
      az_limit_tripped = 0;
    }
  }

    #ifdef FEATURE_ELEVATION_CONTROL
  if (el_limit_sense_pin) {
    if (digitalReadEnhanced(el_limit_sense_pin) == 0) {
      if (!el_limit_tripped) {
        submit_request(EL, REQUEST_KILL, 0, 10);
        el_limit_tripped = 1;
        #ifdef DEBUG_LIMIT_SENSE
          debug.print(F("check_limit_sense: el limit tripped\n"));
        #endif // DEBUG_LIMIT_SENSE
      }
    } else {
      el_limit_tripped = 0;
    }
  }
      #endif // FEATURE_ELEVATION_CONTROL

  #ifdef DEBUG_PROCESSES
    service_process_debug(DEBUG_PROCESSES_PROCESS_EXIT,PROCESS_MISC_ADMIN);
  #endif    

} /* check_limit_sense */
#endif // FEATURE_LIMIT_SENSE

// --------------------------------------------------------------
#ifdef FEATURE_AZ_POSITION_INCREMENTAL_ENCODER
void az_position_incremental_encoder_interrupt_handler(){

  byte rotation_result = 0;
  byte current_phase_a = digitalReadEnhanced(az_incremental_encoder_pin_phase_a);
  byte current_phase_b = digitalReadEnhanced(az_incremental_encoder_pin_phase_b);
  byte current_phase_z = digitalReadEnhanced(az_incremental_encoder_pin_phase_z);

  #ifdef DEBUG_AZ_POSITION_INCREMENTAL_ENCODER
    az_position_incremental_encoder_interrupt++;
  #endif // DEBUG_AZ_POSITION_INCREMENTAL_ENCODER

  if ((az_3_phase_encoder_last_phase_a_state != current_phase_a) || (az_3_phase_encoder_last_phase_b_state != current_phase_b)) {
    if (az_3_phase_encoder_last_phase_a_state == LOW) {
      rotation_result++;
    }
    rotation_result = rotation_result << 1;
    if (az_3_phase_encoder_last_phase_b_state == LOW) {
      rotation_result++;
    }
    rotation_result = rotation_result << 1;
    if (current_phase_a == LOW) {
      rotation_result++;
    }
    rotation_result = rotation_result << 1;
    if (current_phase_b == LOW) {
      rotation_result++;
    }
    switch (rotation_result) {
      case B0010: //az_incremental_encoder_position++; break;
      case B1011: //az_incremental_encoder_position++; break;
      case B1101: //az_incremental_encoder_position++; break;
      case B0100: az_incremental_encoder_position++; break;

      case B0001: //az_incremental_encoder_position--; break;
      case B0111: //az_incremental_encoder_position--; break;
      case B1110: //az_incremental_encoder_position--; break;
      case B1000: az_incremental_encoder_position--; break;
    }


  if (az_incremental_encoder_position > ((long(AZ_POSITION_INCREMENTAL_ENCODER_PULSES_PER_REV*4.) - 1) * 2)) {
    az_incremental_encoder_position = 0;
  }
  if (az_incremental_encoder_position < 0) {
    az_incremental_encoder_position = ((long(AZ_POSITION_INCREMENTAL_ENCODER_PULSES_PER_REV*4.) - 1) * 2);
  }

  #ifndef OPTION_SCANCON_2RMHF3600_INC_ENCODER
    if ((current_phase_a == LOW) && (current_phase_b == LOW) && (current_phase_z == LOW)) {
      if ((az_incremental_encoder_position < long((AZ_POSITION_INCREMENTAL_ENCODER_PULSES_PER_REV*4.) / 2)) || (az_incremental_encoder_position > long((AZ_POSITION_INCREMENTAL_ENCODER_PULSES_PER_REV*4.) * 1.5))) {
        az_incremental_encoder_position = AZ_INCREMENTAL_ENCODER_ZERO_PULSE_POSITION;
      } else {
        az_incremental_encoder_position = long(AZ_POSITION_INCREMENTAL_ENCODER_PULSES_PER_REV*4.);
      }
    }
  #else
    if ((current_phase_a == HIGH) && (current_phase_b == HIGH) && (current_phase_z == HIGH)) {
      if ((az_incremental_encoder_position < long((AZ_POSITION_INCREMENTAL_ENCODER_PULSES_PER_REV*4.) / 2)) || (az_incremental_encoder_position > long((AZ_POSITION_INCREMENTAL_ENCODER_PULSES_PER_REV*4.) * 1.5))) {
        az_incremental_encoder_position = AZ_INCREMENTAL_ENCODER_ZERO_PULSE_POSITION;
      } else {
        az_incremental_encoder_position = long(AZ_POSITION_INCREMENTAL_ENCODER_PULSES_PER_REV*4.);
      }
    }
  #endif //OPTION_SCANCON_2RMHF3600_INC_ENCODER      
  az_3_phase_encoder_last_phase_a_state = current_phase_a;
  az_3_phase_encoder_last_phase_b_state = current_phase_b;

  }

  if (!read_azimuth_lock){
    read_azimuth(1);
    if(!service_rotation_lock){
      service_rotation();
    }
  }
  


} /* az_position_incremental_encoder_interrupt_handler */
#endif // FEATURE_AZ_POSITION_INCREMENTAL_ENCODER
// --------------------------------------------------------------

#if defined(FEATURE_EL_POSITION_INCREMENTAL_ENCODER) && defined(FEATURE_ELEVATION_CONTROL)
void el_position_incremental_encoder_interrupt_handler(){

  byte rotation_result = 0;
  byte current_phase_a = digitalReadEnhanced(el_incremental_encoder_pin_phase_a);
  byte current_phase_b = digitalReadEnhanced(el_incremental_encoder_pin_phase_b);
  byte current_phase_z = digitalReadEnhanced(el_incremental_encoder_pin_phase_z);

  #ifdef DEBUG_EL_POSITION_INCREMENTAL_ENCODER
    el_position_incremental_encoder_interrupt++;
  #endif // DEBUG_EL_POSITION_INCREMENTAL_ENCODER

  if ((el_3_phase_encoder_last_phase_a_state != current_phase_a) || (el_3_phase_encoder_last_phase_b_state != current_phase_b)) {
    if (el_3_phase_encoder_last_phase_a_state == LOW) {
      rotation_result++;
    }
    rotation_result = rotation_result << 1;
    if (el_3_phase_encoder_last_phase_b_state == LOW) {
      rotation_result++;
    }
    rotation_result = rotation_result << 1;
    if (current_phase_a == LOW) {
      rotation_result++;
    }
    rotation_result = rotation_result << 1;
    if (current_phase_b == LOW) {
      rotation_result++;
    }
    switch (rotation_result) {
      case B0010: //el_incremental_encoder_position++; break;
      case B1011: //el_incremental_encoder_position++; break;
      case B1101: //el_incremental_encoder_position++; break;
      case B0100: el_incremental_encoder_position++; break;

      case B0001: //el_incremental_encoder_position--; break;
      case B0111: //el_incremental_encoder_position--; break;
      case B1110: //el_incremental_encoder_position--; break;
      case B1000: el_incremental_encoder_position--; break;
    }

    

    #ifndef OPTION_SCANCON_2RMHF3600_INC_ENCODER
      if ((current_phase_a == LOW) && (current_phase_b == LOW) && (current_phase_z == LOW)) {
        el_incremental_encoder_position = EL_INCREMENTAL_ENCODER_ZERO_PULSE_POSITION;
      } else {

        if (el_incremental_encoder_position < 0) {
          el_incremental_encoder_position = (long(EL_POSITION_INCREMENTAL_ENCODER_PULSES_PER_REV*4.) - 1L);
        }

        if (el_incremental_encoder_position >= long(EL_POSITION_INCREMENTAL_ENCODER_PULSES_PER_REV*4.)) {
          el_incremental_encoder_position = 0;
        }  

      } 
    #else
      if ((current_phase_a == HIGH) && (current_phase_b == HIGH) && (current_phase_z == HIGH)) {
        el_incremental_encoder_position = EL_INCREMENTAL_ENCODER_ZERO_PULSE_POSITION;
      } else {
        if (el_incremental_encoder_position < 0) {
          el_incremental_encoder_position = (long(EL_POSITION_INCREMENTAL_ENCODER_PULSES_PER_REV*4L) - 1L);
        }
        if (el_incremental_encoder_position >= long(EL_POSITION_INCREMENTAL_ENCODER_PULSES_PER_REV*4L)) {
          el_incremental_encoder_position = 0;
        }  
      } 
    #endif //OPTION_SCANCON_2RMHF3600_INC_ENCODER

  


    el_3_phase_encoder_last_phase_a_state = current_phase_a;
    el_3_phase_encoder_last_phase_b_state = current_phase_b;

  }

  if (!read_elevation_lock){
    read_elevation(1);
    if(!service_rotation_lock){
      service_rotation();
    }
  }


} /* el_position_incremental_encoder_interrupt_handler */
  #endif // defined(FEATURE_EL_POSITION_INCREMENTAL_ENCODER) && defined(FEATURE_ELEVATION_CONTROL)

// --------------------------------------------------------------

void pinModeEnhanced(uint8_t pin, uint8_t mode){

  #if !defined(FEATURE_MASTER_WITH_SERIAL_SLAVE) && !defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE)
    pinMode(pin, mode);
  #else
    if (pin < 100) {
      pinMode(pin, mode);
    } else {
      submit_remote_command(REMOTE_UNIT_DHL_COMMAND, pin, mode);
    }
  #endif // !defined(FEATURE_MASTER_WITH_SERIAL_SLAVE) && !defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE)

}

// --------------------------------------------------------------

void digitalWriteEnhanced(uint8_t pin, uint8_t writevalue){



  #if !defined(FEATURE_MASTER_WITH_SERIAL_SLAVE) && !defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE)
    digitalWrite(pin, writevalue);
  #else
    if (pin < 100) {
      digitalWrite(pin, writevalue);
    } else {
      submit_remote_command(REMOTE_UNIT_DHL_COMMAND, pin, writevalue);
    }
  #endif // !defined(FEATURE_MASTER_WITH_SERIAL_SLAVE) && !defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE)

}

// --------------------------------------------------------------

int digitalReadEnhanced(uint8_t pin){

  return digitalRead(pin);

}

// --------------------------------------------------------------

int analogReadEnhanced(uint8_t pin){


  #ifdef FEATURE_ADC_RESOLUTION12
    analogReadResolution(12);
  #endif

  #ifdef OPTION_EXTERNAL_ANALOG_REFERENCE
    analogReference(EXTERNAL);
  #endif //OPTION_EXTERNAL_ANALOG_REFERENCE
  return analogRead(pin);

}

// --------------------------------------------------------------


void analogWriteEnhanced(uint8_t pin, int writevalue){


  #if !defined(FEATURE_MASTER_WITH_SERIAL_SLAVE) && !defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE)
    analogWrite(pin, writevalue);
  #else
    if (pin < 100) {
      analogWrite(pin, writevalue);
    } else {
      submit_remote_command(REMOTE_UNIT_AW_COMMAND, pin, writevalue);
    }
  #endif // !defined(FEATURE_MASTER_WITH_SERIAL_SLAVE) && !defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE)

}


// --------------------------------------------------------------

// #if defined(FEATURE_MASTER_WITH_SERIAL_SLAVE) || defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE)
// void take_care_of_pending_remote_command(){

//   // if there's a command already sent to the remote and we're awaiting the response, service the serial buffer and the queue

//   unsigned long start_time = millis();

//   while ((remote_unit_command_submitted) && ((millis() - start_time) < 200)) {
//     #if defined(FEATURE_MASTER_WITH_SERIAL_SLAVE)
//     check_serial();
//     #endif //defined(FEATURE_MASTER_WITH_SERIAL_SLAVE)
//     #if defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE)
//     service_ethernet();
//     #endif //defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE)
//     service_remote_communications_incoming_buffer();
//   }


// }
// #endif // defined(FEATURE_MASTER_WITH_SERIAL_SLAVE) || defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE)
// --------------------------------------------------------------




void port_flush(){

  
  #if defined(CONTROL_PORT_MAPPED_TO) && (defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION))
    control_port->flush();
  #endif //CONTROL_PORT_MAPPED_TO

  #if defined(FEATURE_MASTER_WITH_SERIAL_SLAVE)
    REMOTE_PORT.flush();
  #endif

  #if defined(GPS_PORT_MAPPED_TO) && defined(FEATURE_GPS)
    GPS_PORT.flush();
  #endif //defined(GPS_PORT_MAPPED_TO) && defined(FEATURE_GPS)
  

}
// --------------------------------------------------------------
#ifdef FEATURE_POWER_SWITCH
void service_power_switch(){

  static byte power_switch_state = 1;

  #ifdef FEATURE_ELEVATION_CONTROL
  if ((az_state != IDLE) || (el_state != IDLE)){
    last_activity_time = millis();
  }
  #else //FEATURE_ELEVATION_CONTROL
  if (az_state != IDLE){
    last_activity_time = millis();
  }
  #endif //FEATURE_ELEVATION_CONTROL


  if ((millis()-last_activity_time) > ((unsigned long)60000 * (unsigned long)POWER_SWITCH_IDLE_TIMEOUT)) {
    if (power_switch_state){ 
      digitalWriteEnhanced(power_switch, LOW);
      power_switch_state = 0;
    }
  } else {
    if (!power_switch_state){ 
      digitalWriteEnhanced(power_switch, HIGH);
      power_switch_state = 1;
    }
  }


}
#endif //FEATURE_POWER_SWITCH


//------------------------------------------------------
char *coordinates_to_maidenhead(float latitude_degrees,float longitude_degrees){

  static char temp_string[8] = "";

  latitude_degrees += 90.0;
  longitude_degrees += 180.0;

  temp_string[0] = (int(longitude_degrees/20)) + 65;
  temp_string[1] = (int(latitude_degrees/10)) + 65;
  temp_string[2] = (int((longitude_degrees - int(longitude_degrees/20)*20)/2)) + 48;
  temp_string[3] = (int(latitude_degrees - int(latitude_degrees/10)*10)) + 48;
  temp_string[4] = (int((longitude_degrees - (int(longitude_degrees/2)*2)) / (5.0/60.0))) + 97;
  temp_string[5] = (int((latitude_degrees - (int(latitude_degrees/1)*1)) / (2.5/60.0))) + 97;
  temp_string[6] = 0;

  return temp_string;

}

//------------------------------------------------------
void maidenhead_to_coordinates(char* grid, float* latitude_degrees,float* longitude_degrees){

  /*  Base code contributed by Adam Maurer VK4GHZ 14-JUL-2020

      Input: grid, four or six characters (i.e. FN20, FN20EV, fn20, fn20ev, Fn20eV)
      Output: latitude_degrees, longitude_degrees

  */

  float alpha1 = 0;
  float alpha2 = 0;
  byte alpha3 = 0;
  byte alpha4 = 0;
  float alpha5 = 0;
  float alpha6 = 0;

  const int x_step = 20; // Number of Horizontal degrees EW across a major field
  const int y_step = 10; // Number of Vertical degrees NS across a major field

  // Uppercase everything
  for (int x = 0;x < 6;x++){
    grid[x] = toupper(grid[x]);
  }

  // Do we have a subsquare?  If not, fudge it in the middle of the square
  if (!((grid[4] > 64) && (grid[4] < 89) && (grid[5] > 64) && (grid[5] < 89))){
    grid[4] = 77;
    grid[5] = 77;
  }

  // Convert Alphas to numeric values to calculate with
  // A=65... R=82... X=88

  alpha1 = (grid[0]-65) * x_step;
  alpha2 = (grid[1]-65) * y_step;
  alpha3 = (grid[2]-48) * (x_step/10);
  alpha4 = (grid[3]-48) * (y_step/10);

  // Each Field is subdivided into 24 x 24 sub squares
  alpha5 = ((grid[4]-65) * (x_step/240.0)) + (x_step/480.0); 
  alpha6 = ((grid[5]-65) * (y_step/240.0)) + (y_step/480.0);

  *longitude_degrees = alpha1 + alpha3 + alpha5 - 180.0;

  *latitude_degrees = (alpha2 + alpha4 + alpha6) - 90.0;

}

//------------------------------------------------------
float calculate_target_bearing(float source_latitude,float source_longitude,float target_latitude,float target_longitude){


  //  Base code contributed by Adam Maurer VK4GHZ 14-JUL-2020

  float teta1 = radians(source_latitude);
  float teta2 = radians(target_latitude);
  float delta2 = radians(target_longitude-source_longitude);

  
  float y = sin(delta2) * cos(teta2);
  float x = cos(teta1) * sin(teta2) - sin(teta1) * cos(teta2) * cos(delta2);
  float targetBearing = atan2(y,x);
  targetBearing = degrees(targetBearing);// radians to degrees
  targetBearing = (((int)targetBearing + 360) % 360 ); 

  return targetBearing;

}

//------------------------------------------------------

#ifdef FEATURE_ANALOG_OUTPUT_PINS
void service_analog_output_pins(){

  static int last_azimith_voltage_out = 0;
  int azimuth_voltage_out = map(azimuth,0,360,0,255);
  if (last_azimith_voltage_out != azimuth_voltage_out){
    analogWriteEnhanced(pin_analog_az_out,azimuth_voltage_out);
    last_azimith_voltage_out = azimuth_voltage_out;
  }

  #ifdef FEATURE_ELEVATION_CONTROL
  static int last_elevation_voltage_out = 0;
  int elevation_voltage_out = map(elevation,0,ANALOG_OUTPUT_MAX_EL_DEGREES,0,255);
  if (last_elevation_voltage_out != elevation_voltage_out){
    analogWriteEnhanced(pin_analog_el_out,elevation_voltage_out);
    last_elevation_voltage_out = elevation_voltage_out;
  }
  #endif //FEATURE_ELEVATION_CONTROL

}
#endif //FEATURE_ANALOG_OUTPUT_PINS




//-------------------------------------------------------



#ifdef FEATURE_AUTOCORRECT
void submit_autocorrect(byte axis,float heading){

  #ifdef DEBUG_AUTOCORRECT
  debug.print("submit_autocorrect: ");
  #endif //DEBUG_AUTOCORRECT

  if (axis == AZ){
    autocorrect_state_az = AUTOCORRECT_WATCHING_AZ;
    autocorrect_az = heading;
    autocorrect_az_submit_time = millis();

    #ifdef DEBUG_AUTOCORRECT
    debug.print("AZ: ");
    #endif //DEBUG_AUTOCORRECT

  }


  #ifdef FEATURE_ELEVATION_CONTROL
  if (axis == EL){
    autocorrect_state_el = AUTOCORRECT_WATCHING_EL;
    autocorrect_el = heading;
    autocorrect_el_submit_time = millis();

    #ifdef DEBUG_AUTOCORRECT
    debug.print("EL: ");
    #endif //DEBUG_AUTOCORRECT

  }
  #endif //FEATURE_ELEVATION_CONTROL

  #ifdef DEBUG_AUTOCORRECT
  debug.print(heading,2);
  debug.println("");
  #endif //DEBUG_AUTOCORRECT

}
#endif //FEATURE_AUTOCORRECT



// --------------------------------------------------------------
//#if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_ANCILLARY_PIN_CONTROL) || defined(UNDER_DEVELOPMENT_REMOTE_UNIT_COMMANDS)
byte get_analog_pin(byte pin_number){

  byte return_output = 0;

  switch (pin_number) {
    case 0: return_output = A0; break;
    case 1: return_output = A1; break;
    case 2: return_output = A2; break;
    case 3: return_output = A3; break;
    case 4: return_output = A4; break;
    case 5: return_output = A5; break;
    case 6: return_output = A6; break;
  }

  return return_output;

}
//#endif // FEATURE_REMOTE_UNIT_SLAVE

// *************************************** stuff below here has issues moving to .h files - need to work on this **********************


// -------------------------------------------------------------

// --------------------------------------------------------------
void strconditionalcpy(char *__dst, const char *__src, byte do_it){

  if (do_it){
    strcpy(__dst,__src);
  } else {
    strcpy(__dst,"");
  }
}

// --------------------------------------------------------------
byte process_backslash_command(byte input_buffer[], int input_buffer_index, byte source_port, byte include_response_code, char * return_string, byte input_source){

  strcpy(return_string,"");
  float tempfloat = 0;
  byte hit_decimal = 0;
  byte x = 0;

  #if defined(FEATURE_PARK) && defined(FEATURE_NEXTION_DISPLAY)
    char workstring1[32];
    char workstring2[32];
  #endif

  #if !defined(OPTION_SAVE_MEMORY_EXCLUDE_REMOTE_CMDS)
    float heading = 0;
  #endif 

  #ifdef FEATURE_CLOCK
    int temp_year = 0;
    byte temp_month = 0;
    byte temp_day = 0;
    byte temp_minute = 0;
    byte temp_hour = 0;
    byte temp_second = 0;
    byte negative_flag = 0;
  #endif // FEATURE_CLOCK

  #if defined(FEATURE_MOON_TRACKING) || defined(FEATURE_SUN_TRACKING)
    char grid[10] = "";
    byte hit_error = 0;
  #endif // defined(FEATURE_MOON_TRACKING) || defined(FEATURE_SUN_TRACKING)

  #if defined(FEATURE_AZ_POSITION_ROTARY_ENCODER) || defined(FEATURE_AZ_POSITION_PULSE_INPUT) || defined(FEATURE_AZ_POSITION_ROTARY_ENCODER_USE_PJRC_LIBRARY) || defined(FEATURE_AZ_POSITION_HH12_AS5045_SSI_RELATIVE)
    float new_azimuth = 9999;
  #endif

  #if defined(FEATURE_ELEVATION_CONTROL) && (defined(FEATURE_EL_POSITION_ROTARY_ENCODER) || defined(FEATURE_EL_POSITION_PULSE_INPUT) || defined(FEATURE_EL_POSITION_ROTARY_ENCODER_USE_PJRC_LIBRARY))
    float new_elevation = 9999;
  #endif

  #if defined(FEATURE_SATELLITE_TRACKING)
    unsigned long tle_upload_start_time;
    byte got_return = 0;
    byte tle_char_read = 0;  
    byte end_of_eeprom_was_hit = 0;
    byte write_char_to_tle_file_area_result = 0;
    byte tle_serial_buffer[2001];
    unsigned int tle_serial_buffer_in_pointer = 0;
    unsigned int tle_serial_buffer_out_pointer = 0;
    char satellite_to_find[17];
    byte get_line_result = 0;
    byte tle_line_number;
    byte line_char_counter;
    byte last_tle_char_read = 0;    
  #endif

  #if defined(FEATURE_AUTOPARK)
    byte valid_input_autopark = 0;
  #endif  

  byte brake_az_disabled;

  char temp_string[20] = "";

  switch (input_buffer[1]) {
  #if !defined(OPTION_SAVE_MEMORY_EXCLUDE_BACKSLASH_CMDS)
  #if defined(FEATURE_AZ_POSITION_ROTARY_ENCODER) || defined(FEATURE_AZ_POSITION_PULSE_INPUT) || defined(FEATURE_AZ_POSITION_ROTARY_ENCODER_USE_PJRC_LIBRARY) || defined(FEATURE_AZ_POSITION_HH12_AS5045_SSI_RELATIVE)
    case 'A':      // \Ax[x][x] - manually set azimuth
      new_azimuth = 9999;
      switch (input_buffer_index) {
        case 3:
          new_azimuth = (input_buffer[2] - 48);
          break;
        case 4:
          new_azimuth = ((input_buffer[2] - 48) * 10) + (input_buffer[3] - 48);
          break;
        case 5:
          new_azimuth = ((input_buffer[2] - 48) * 100) + ((input_buffer[3] - 48) * 10) + (input_buffer[4] - 48);
          break;
      }
      if ((new_azimuth >= 0) && (new_azimuth < 360)) {
        azimuth = new_azimuth;
        configuration.last_azimuth = new_azimuth;
        raw_azimuth = new_azimuth;
        configuration_dirty = 1;
        strcpy_P(return_string, (const char*) F("Azimuth set to "));
        dtostrf(new_azimuth, 0, 0, temp_string);
        strcat(return_string, temp_string);
      } else {
        strcpy_P(return_string, (const char*) F("Error.  Format: \\Ax[x][x] "));
      }
      break;
   #else // defined(FEATURE_AZ_POSITION_ROTARY_ENCODER) || defined(FEATURE_AZ_POSITION_PULSE_INPUT)
    case 'A':      // \Ax[xxx][.][xxxx] - manually set azimuth
      place_multiplier = 1;
      for (int x = input_buffer_index - 1; x > 1; x--) {
        if (char(input_buffer[x]) != '.') {
          tempfloat += (input_buffer[x] - 48) * place_multiplier;
          place_multiplier = place_multiplier * 10;
        } else {
          decimalplace = x;
        }
      }
      if (decimalplace) {
        tempfloat = tempfloat / pow(10, (input_buffer_index - decimalplace - 1));
      }
      if ((tempfloat >= 0) && (tempfloat <= 360)) {
        configuration.azimuth_offset = 0;
        read_azimuth(1);
        configuration.azimuth_offset = tempfloat - raw_azimuth;
        configuration.azimuth_starting_point = configuration.azimuth_starting_point + abs(configuration.azimuth_offset);      
        configuration_dirty = 1;
        strcpy_P(return_string, (const char*) F("Azimuth calibrated to "));
        dtostrf(tempfloat, 0, 2, temp_string);
        strcat(return_string, temp_string);
      } else {
        strcpy(return_string, "Error.");
      }

      break;
   #endif // defined(FEATURE_AZ_POSITION_ROTARY_ENCODER) || defined(FEATURE_AZ_POSITION_PULSE_INPUT)

    case 'I':        // \Ix[x][x] - set az starting point
      tempfloat = 0;
      for (int x = 2;x < input_buffer_index;x++){
        if(input_buffer[x] == '.'){
          hit_decimal = 10;
        } else {
          if (hit_decimal > 0){
            tempfloat = tempfloat + ((float)(input_buffer[x] - 48) / (float)hit_decimal);
            hit_decimal = hit_decimal * 10;
          } else {
            tempfloat = (tempfloat * 10) + (input_buffer[x] - 48);
          }
        }
      }   

      strcpy_P(return_string, (const char*) F("Azimuth starting point ")); 
      if ((tempfloat > 0) || (input_buffer_index > 2)){
        configuration.azimuth_starting_point = tempfloat;
        configuration_dirty = 1;
        strcat_P(return_string, (const char*) F("set to "));
      } else {
        strcat_P(return_string, (const char*) F("is "));
      }
      dtostrf(configuration.azimuth_starting_point, 0, 0, temp_string);
      strcat(return_string, temp_string);      
      break;

    case 'J':        // \Jx[x][x][x][x] - set az rotation capability
      tempfloat = 0;
      for (int x = 2;x < input_buffer_index;x++){
        if(input_buffer[x] == '.'){
          hit_decimal = 10;
        } else {
          if (hit_decimal > 0){
            tempfloat = tempfloat + ((float)(input_buffer[x] - 48) / (float)hit_decimal);
            hit_decimal = hit_decimal * 10;
          } else {
            tempfloat = (tempfloat * 10) + (input_buffer[x] - 48);
          }
        }
      }   
      strcpy_P(return_string, (const char*) F("Azimuth rotation capability ")); 
      if (tempfloat > 0){
        configuration.azimuth_rotation_capability = tempfloat;
        configuration_dirty = 1;
        strcat_P(return_string, (const char*) F("set to "));
      } else {
        strcat_P(return_string, (const char*) F("is "));
      }
      dtostrf(configuration.azimuth_rotation_capability, 0, 0, temp_string);
      strcat(return_string, temp_string);

      break;

    case 'K':          // \Kx   - Force disable the az brake even if a pin is defined (x: 0 = enable, 1 = disable)
      brake_az_disabled = 2;
      if (input_buffer_index == 2) {
        brake_az_disabled = configuration.brake_az_disabled;
      } else {
          switch (input_buffer[2]) {
            case '0': brake_az_disabled = 0; break;
            case '1': brake_az_disabled = 1; break;
          }
      }
      if ((brake_az_disabled >=0) && (brake_az_disabled <= 1)) {
        if (input_buffer_index > 2) {
          configuration.brake_az_disabled = brake_az_disabled;
          configuration_dirty = 1;
        }
        strcpy(return_string, "Az brake ");
        strcat(return_string, (brake_az_disabled ? "disabled." : "enabled."));
      } else {
        strcpy(return_string, "Error.");
      }
      break;

    #if defined(FEATURE_ELEVATION_CONTROL)
      #if defined(FEATURE_EL_POSITION_ROTARY_ENCODER) || defined(FEATURE_EL_POSITION_PULSE_INPUT)  || defined(FEATURE_EL_POSITION_ROTARY_ENCODER_USE_PJRC_LIBRARY)
        case 'B':      // \Bx[x][x] - manually set elevation
          new_elevation = 9999;
          switch (input_buffer_index) {
            case 3:
              new_elevation = (input_buffer[2] - 48);
              break;
            case 4:
              new_elevation = ((input_buffer[2] - 48) * 10) + (input_buffer[3] - 48);
              break;
            case 5:
              new_elevation = ((input_buffer[2] - 48) * 100) + ((input_buffer[3] - 48) * 10) + (input_buffer[4] - 48);
              break;
          }
          if ((new_elevation >= 0) && (new_elevation <= 180)) {
            elevation = new_elevation;
            configuration.last_elevation = new_elevation;
            configuration_dirty = 1;
            strcpy_P(return_string, (const char*) F("Elevation set to "));
            dtostrf(new_elevation, 0, 0, temp_string);
            strcat(return_string, temp_string);
          } else {
            strcpy_P(return_string, (const char*) F("Error.  Format: \\Bx[x][x]"));
          }
          break;
      #else // defined(FEATURE_EL_POSITION_ROTARY_ENCODER) || defined(FEATURE_EL_POSITION_PULSE_INPUT)
        case 'B':      // \Bx[xxx][.][xxxx] - manually set elevation
          place_multiplier = 1;
          for (int x = input_buffer_index - 1; x > 1; x--) {
            if (char(input_buffer[x]) != '.') {
              tempfloat += (input_buffer[x] - 48) * place_multiplier;
              place_multiplier = place_multiplier * 10;
            } else {
              decimalplace = x;
            }
          }
          if (decimalplace) {
            tempfloat = tempfloat / pow(10, (input_buffer_index - decimalplace - 1));
          }
          if ((tempfloat >= 0) && (tempfloat <= 180)) {
            configuration.elevation_offset = 0;
            read_elevation(1);
            configuration.elevation_offset = tempfloat - elevation;
            configuration_dirty = 1;
            strcpy_P(return_string, (const char*) F("Elevation calibrated to "));
            dtostrf(tempfloat, 0, 2, temp_string);
            strcat(return_string, temp_string);
          } else {
            strcpy_P(return_string, (const char*) F("Error."));
          }
          break;
      #endif // defined(FEATURE_EL_POSITION_ROTARY_ENCODER) || defined(FEATURE_EL_POSITION_PULSE_INPUT)
    #endif //FEATURE_ELEVATION_CONTROL

    #ifdef FEATURE_CLOCK
    case 'C':         // show clock
      #if defined(OPTION_USE_OLD_TIME_CODE)
      update_time();
      #endif
      sprintf(return_string, "%s", timezone_modified_clock_string());
      break;

    case 'O':         // set clock UTC time
      temp_year = ((input_buffer[2] - 48) * 1000) + ((input_buffer[3] - 48) * 100) + ((input_buffer[4] - 48) * 10) + (input_buffer[5] - 48);
      temp_month = ((input_buffer[6] - 48) * 10) + (input_buffer[7] - 48);
      temp_day = ((input_buffer[8] - 48) * 10) + (input_buffer[9] - 48);
      temp_hour = ((input_buffer[10] - 48) * 10) + (input_buffer[11] - 48);
      temp_minute = ((input_buffer[12] - 48) * 10) + (input_buffer[13] - 48);
      if (input_buffer_index == 16){
        temp_second = ((input_buffer[14] - 48) * 10) + (input_buffer[15] - 48);
      } else {
        temp_second = 0;
      }
      if ( (temp_year > 2020) && (temp_year < 2070) &&
           (temp_month > 0) && (temp_month < 13) &&
           (temp_day > 0) && (temp_day < 32) &&
           (temp_hour >= 0) && (temp_hour < 24) &&
           (temp_minute >= 0) && (temp_minute < 60) &&
           (temp_second >= 0) && (temp_second < 60) &&
           ((input_buffer_index == 14) || (input_buffer_index == 16)) ){
        #if defined(OPTION_USE_OLD_TIME_CODE)
          set_clock.year = temp_year;
          set_clock.month = temp_month;
          set_clock.day = temp_day;
          set_clock.hours = temp_hour;
          set_clock.minutes = temp_minute;
          set_clock.seconds = temp_second;
          millis_at_last_calibration = millis();
        #else //#if defined(OPTION_USE_OLD_TIME_CODE)
          setTime(temp_hour, temp_minute, temp_second, temp_day, temp_month, temp_year);
        #endif //#if defined(OPTION_USE_OLD_TIME_CODE)

        #if defined(FEATURE_SATELLITE_TRACKING)
          satellite_array_data_ready = 0;
        #endif

        // Update the Realtime Clock if we have one

        #if defined(FEATURE_RTC_DS1307)
          #if defined(DEBUG_RTC)
            debug.println("process_backslash_command: setting DS1307 RTC time");
          #endif
          rtc.adjust(DateTime(temp_year, temp_month, temp_day, temp_hour, temp_minute, temp_second));
          #if defined(DEBUG_RTC)
            debug.println("process_backslash_command: set DS1307 RTC time");
          #endif          
        #endif // defined(FEATURE_RTC_DS1307)
        #if defined(FEATURE_RTC_PCF8583)
          #if defined(DEBUG_RTC)
            debug.println("process_backslash_command: setting PCF8583 RTC time");
          #endif        
          rtc.year = temp_year;
          rtc.month = temp_month;
          rtc.day = temp_day;
          rtc.hour  = temp_hour;
          rtc.minute = temp_minute;
          rtc.second = temp_second;
          rtc.set_time();
          #if defined(DEBUG_RTC)
            debug.println("process_backslash_command: set PCF8583 RTC time");
          #endif             
        #endif // defined(FEATURE_RTC_PCF8583)
        #if defined(FEATURE_RTC_TEENSY)
          #if defined(DEBUG_RTC)
            debug.println("process_backslash_command: setting Teensy RTC time");
          #endif        
          tmElements_t temp_t;
          temp_t.Year = temp_year - 1970;
          temp_t.Month = temp_month;
          temp_t.Day = temp_day;
          temp_t.Hour = temp_hour;
          temp_t.Minute = temp_minute;
          temp_t.Second = temp_second;
          Teensy3Clock.set(makeTime(temp_t));
          #if defined(DEBUG_RTC)
            debug.println("process_backslash_command: set Teensy RTC time");
          #endif             
        #endif //FEATURE_RTC_TEENSY

        #if (!defined(FEATURE_RTC_DS1307) && !defined(FEATURE_RTC_PCF8583) && !defined(FEATURE_RTC_TEENSY))
          strcpy(return_string, "Clock set to ");
          #if defined(OPTION_USE_OLD_TIME_CODE)
          update_time();
          #endif //OPTION_USE_OLD_TIME_CODE
          strcat(return_string, timezone_modified_clock_string());
        #else
          strcpy_P(return_string, (const char*) F("Internal clock and RTC set to "));
          #if defined(OPTION_USE_OLD_TIME_CODE)
          update_time();
          #endif //#if defined(OPTION_USE_OLD_TIME_CODE)
          strcat(return_string, timezone_modified_clock_string());
        #endif
      } else {
        strcpy_P(return_string, (const char*) F("Error. Usage: \\OYYYYMMDDHHmm[ss]"));
      }
      break;

    case 'V': //  \Vx[xxx][.][xxxx]   Set time zone offset
      negative_flag = 0;
      place_multiplier = 1;
      for (int x = input_buffer_index - 1; x > 1; x--) {
        if (char(input_buffer[x]) == '-') {
          negative_flag = 1;
        } else {
          if (char(input_buffer[x]) != '.') {
            tempfloat += (input_buffer[x] - 48) * place_multiplier;
            place_multiplier = place_multiplier * 10;
          } else {
            decimalplace = x;
          }
        }
      }
      if (decimalplace) {
        tempfloat = tempfloat / pow(10, (input_buffer_index - decimalplace - 1));
      }
      if (negative_flag){tempfloat = tempfloat * -1.0;}
      if ((tempfloat >= -24.0) && (tempfloat <= 24.0)) {
        configuration.clock_timezone_offset = tempfloat;
        configuration_dirty = 1;
        strcpy_P(return_string, (const char*) F("Timezone offset set to "));
        dtostrf(tempfloat, 0, 2, temp_string);
        strcat(return_string, temp_string);
      } else {
        strcpy_P(return_string, (const char*) F("Error."));
      }
      break;
    #endif // FEATURE_CLOCK

    case 'D':                                                                      // \D - Debug
      if (debug_mode & source_port) {
        debug_mode = debug_mode & (~source_port);
      } else {
        debug_mode = debug_mode | source_port;
      } 

      if (input_buffer_index == 3){
        x = (input_buffer[2] - 48);
      } else {
        if (input_buffer_index == 4){
          x = ((input_buffer[2] - 48) * 10) + (input_buffer[3] - 48);
        } else {
          if (input_buffer_index == 5){
            x = ((input_buffer[2] - 48) * 100) + ((input_buffer[3] - 48) * 10) + (input_buffer[4] - 48);              
          } else {
            x = 3;
          }
        }
      }
      periodic_debug_dump_time_seconds = x;
      break;

    case 'E':                                                                      // \E - Initialize eeprom
      initialize_eeprom_with_defaults();
      strcpy_P(return_string, (const char*) F("Initialized eeprom, resetting unit in 5 seconds..."));
      reset_the_unit = 1;
      break;

    #if defined(FEATURE_LCD_DISPLAY)
      case 'H':
        //k3ngdisplay.clear();
        k3ngdisplay.redraw();
        break;
    #endif  

    case 'Q':                                                                      // \Q - Save settings in the EEPROM and restart
      write_settings_to_eeprom();
      strcpy_P(return_string, (const char*) F("Settings saved in EEPROM, resetting unit in 5 seconds..."));
      reset_the_unit = 1;
      break;

    case 'L':                                                                      // \L - rotate to long path
      if (azimuth < (180)) {
        submit_request(AZ, REQUEST_AZIMUTH, (azimuth + (180)), 15);
      } else {
        submit_request(AZ, REQUEST_AZIMUTH, (azimuth - (180)), 16);
      }
      break;

 #if defined(FEATURE_MOON_TRACKING) || defined(FEATURE_SUN_TRACKING)
    case 'G':   // G - set coordinates using grid square
      if (isalpha(input_buffer[2])) {
        grid[0] = input_buffer[2];
      } else { hit_error = 1; }
      if (isalpha(input_buffer[3])) {
        grid[1] = input_buffer[3];
      } else { hit_error = 1; }
      if (isdigit(input_buffer[4])) {
        grid[2] = input_buffer[4];
      } else { hit_error = 1; }
      if (isdigit(input_buffer[5])) {
        grid[3] = input_buffer[5];
      } else { hit_error = 1; }
      if (isalpha(input_buffer[6])) {
        grid[4] = input_buffer[6];
      } else { hit_error = 1; }
      if (isalpha(input_buffer[7])) {
        grid[5] = input_buffer[7];
      } else { hit_error = 1; }
      if ((input_buffer_index != 8) || (hit_error)) {
        strcpy_P(return_string, (const char*) F("Error.  Usage \\Gxxxxxx"));
      } else {
        grid2deg(grid, &longitude, &latitude);
        strcpy_P(return_string, (const char*) F("Coordinates set to: "));
        dtostrf(latitude, 0, 4, temp_string);
        strcat(return_string, temp_string);
        strcat(return_string, " ");
        dtostrf(longitude, 0, 4, temp_string);
        strcat(return_string, temp_string);
      }
      break;
 #endif // defined(FEATURE_MOON_TRACKING) || defined(FEATURE_SUN_TRACKING)

  #ifdef FEATURE_MOON_TRACKING
    case 'M':
      switch (input_buffer[2]) {
        case '0':
          change_tracking(DEACTIVATE_MOON_TRACKING);
          stop_rotation();
          strcpy_P(return_string, (const char*) F("Moon tracking deactivated."));
          break;
        case '1':
          change_tracking(ACTIVATE_MOON_TRACKING);          
          strcpy_P(return_string, (const char*) F("Moon tracking activated."));
          break;
        default: //strcpy(return_string, "Error."); 
          update_moon_position();
          strcpy_P(return_string, (const char*) F("Moon: AZ:"));
          dtostrf(moon_azimuth,0,2,temp_string);
          strcat(return_string, temp_string);
          strcat_P(return_string, (const char*) F(" EL:"));
          dtostrf(moon_elevation,0,2,temp_string);
          strcat(return_string, temp_string);       
        break;
      }
      break;
  #endif // FEATURE_MOON_TRACKING

  #if defined(FEATURE_MASTER_WITH_SERIAL_SLAVE) || defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE)
    case 'R':
      strcpy_P(return_string, (const char*) F("Remote port rx sniff o"));
      if (remote_port_rx_sniff) {
        remote_port_rx_sniff = 0;
        strcat(return_string, "ff");
      } else {
        remote_port_rx_sniff = 1;
        strcat(return_string, "n");
      }
      break;
    case 'S':
      #if defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE)
      ethernetslavelinkclient0.print(ETHERNET_PREAMBLE);
      #endif    
      for (int x = 2; x < input_buffer_index; x++) {
        #if defined(FEATURE_MASTER_WITH_SERIAL_SLAVE)
        REMOTE_PORT.write(input_buffer[x]);
        #endif
        #if defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE)
        ethernetslavelinkclient0.write(input_buffer[x]);
        #endif
        if (remote_port_tx_sniff) {
          control_port->write(input_buffer[x]);
        }
      }
      #if defined(FEATURE_MASTER_WITH_SERIAL_SLAVE)
      REMOTE_PORT.write(13);
      #endif
      #if defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE)
      ethernetslavelinkclient0.write(13);
      #endif      
      if (remote_port_tx_sniff) {
        control_port->write(13);
      }
      break;
    case 'T':
      strcpy_P(return_string,(const char*) F( "Remote port tx sniff o"));
      if (remote_port_tx_sniff) {
        remote_port_tx_sniff = 0;
        strcat(return_string, "ff");
      } else {
        remote_port_tx_sniff = 1;
        strcat(return_string, "n");
      }
      break;
    case 'Z':
      strcpy_P(return_string, (const char*) F("Suspend auto remote commands o"));
      if (suspend_remote_commands) {
        suspend_remote_commands = 0;
        strcat(return_string, "ff");
      } else {
        suspend_remote_commands = 1;
        strcat(return_string, "n");
      }
      break;
  #endif // defined(FEATURE_MASTER_WITH_SERIAL_SLAVE) || defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE)

  #ifdef FEATURE_SUN_TRACKING
    case 'U':     // activate / deactivate sun tracking
      switch (input_buffer[2]) {
        case '0':
          change_tracking(DEACTIVATE_SUN_TRACKING);
          stop_rotation();
          strcpy_P(return_string, (const char*) F("Sun tracking deactivated."));
          break;
        case '1':
          change_tracking(ACTIVATE_SUN_TRACKING);
          strcpy_P(return_string, (const char*) F("Sun tracking activated."));       
          break;
        default: //strcpy(return_string, "Error."); 
          strcpy_P(return_string, (const char*) F("Sun: AZ:"));
          dtostrf(sun_azimuth,0,2,temp_string);
          strcat(return_string, temp_string);
          strcat_P(return_string, (const char*) F(" EL:"));
          dtostrf(sun_elevation,0,2,temp_string);
          strcat(return_string, temp_string);         
          break;
      }
      break;

  #endif // FEATURE_SUN_TRACKING

  #if !defined(FEATURE_CALIBRATION)
    case 'X':
      switch (toupper(input_buffer[2])) {
        #if defined(FEATURE_SUN_TRACKING)
        case 'S': 
          update_sun_position();
          if (calibrate_az_el(sun_azimuth, sun_elevation)) {
            strcpy(return_string, az_el_calibrated_string());
          } else {
            strcpy_P(return_string, (const char*) F("Error."));
          }
          break;
        #endif // FEATURE_SUN_TRACKING
        #if defined(FEATURE_MOON_TRACKING)
        case 'M':
          update_moon_position();
          if (calibrate_az_el(moon_azimuth, moon_elevation)) {
            strcpy(return_string, az_el_calibrated_string());
          } else {
            strcpy_P(return_string, (const char*) F("Error."));
          }
          break;
        #endif // FEATURE_MOON_TRACKING
        case '0':
          configuration.azimuth_starting_point = configuration.azimuth_starting_point - abs(configuration.azimuth_offset);
          configuration.azimuth_offset = 0;
          configuration.elevation_offset = 0;
          configuration_dirty = 1;
          strcpy_P(return_string, (const char*) F("Cleared calibration offsets"));
          break;
        default: strcpy(return_string, "?>"); break;


      } //switch (toupper(input_buffer[2]))
      break; //case 'X':

  #else //FEATURE_CALIBRATION

    case 'X':
      #if defined(FEATURE_SUN_TRACKING)
        if ((input_buffer_index == 3) && (toupper(input_buffer[2]) == 'S')){
          update_sun_position();
          if (calibrate_az_el(sun_azimuth, sun_elevation)) {
            strcpy(return_string, az_el_calibrated_string());
          } else {
            strcpy_P(return_string, (const char*) F("Error."));
          }
        }
      #endif // FEATURE_SUN_TRACKING

      #if defined(FEATURE_MOON_TRACKING)
        if ((input_buffer_index == 3) && (toupper(input_buffer[2]) == 'M')){
          update_moon_position();
          if (calibrate_az_el(moon_azimuth, moon_elevation)) {
            strcpy(return_string, az_el_calibrated_string());
          } else {
            strcpy_P(return_string, (const char*) F("Error."));
          }
        }
      #endif // FEATURE_MOON_TRACKING
        
      if ((input_buffer_index == 3) && (toupper(input_buffer[2]) == '0')){
        initialize_calibration_tables();
        configuration_dirty = 1;
        strcpy_P(return_string, (const char*) F("Cleared calibration tables")); 
      } 

      if (input_buffer_index == 2){   // no arguments, print the calibration table
        control_port->println(F("\n\rCalibration Tables\n\r"));
        control_port->println(F("AZ\n\rIn\t\tOut"));
        for (byte x = 0;x < CALIBRATION_POINTS;x++){
          if (configuration.calibration_az_flag[x] > 0){
            control_port->print(configuration.calibration_az_from[x],2);
            control_port->print("\t");
            if (configuration.calibration_az_from[x] > 360.0){
              control_port->print("(");
              control_port->print(configuration.calibration_az_from[x]-360.0,2);
              control_port->print(")");
            }
            control_port->print("\t");
            control_port->print(configuration.calibration_az_to[x],2);
            if (configuration.calibration_az_to[x] > 360.0){
              control_port->print("(");
              control_port->print(configuration.calibration_az_to[x]-360.0,2);
              control_port->print(")");
            }
            control_port->print("\t");            
            control_port->print("\t"); 
            switch (configuration.calibration_el_flag[x]){
              case 1:control_port->println(F("default"));break;
              case 2:control_port->println(F("sun"));break;
              case 3:control_port->println(F("moon"));break;
              case 4:control_port->println(F("user"));break;
            }
          }
        }
        control_port->println(F("\n\rEL\n\rIn\tOut"));  
        for (byte x = 0;x < CALIBRATION_POINTS;x++){
          if (configuration.calibration_el_flag[x] > 0){
            control_port->print(configuration.calibration_el_from[x],2);
            control_port->print("\t");
            control_port->print(configuration.calibration_el_to[x],2);   
            control_port->print("\t");
            switch (configuration.calibration_el_flag[x]){
              case 1:control_port->println(F("default"));break;
              case 2:control_port->println(F("sun"));break;
              case 3:control_port->println(F("moon"));break;
              case 4:control_port->println(F("user"));break;
            }
          }            
        }
        //zzzzzzzz
      }      
      break; //case 'X'

    #endif //FEATURE_CALIBRATION

    #ifdef FEATURE_PARK
    case 'P':    // Park, PA and PE commands - set / query park azimuth and elevation
      temp_int = 999;
      // control_port->print(F("input_buffer_index:"));
      // control_port->println(input_buffer_index);
      if (input_buffer_index == 2){ // no parameters, it's a park initiation
        strcpy_P(return_string, (const char*) F("Parking..."));
        initiate_park();
        park_serial_initiated = 1;
      } else {
        if ((input_buffer[2] == 'A') || (input_buffer[2] == 'E')){ 
          if (input_buffer_index == 3){ // PA or PE, no parameters, it's a query
            #if defined(FEATURE_NEXTION_DISPLAY)
            if (input_source == SOURCE_NEXTION){
              strcpy_P(workstring1,(const char*) F("Park Az: "));
              dtostrf(configuration.park_azimuth, 1, 0, workstring2);
              strcat(workstring1,workstring2);
              #if defined(FEATURE_ELEVATION_CONTROL)
                strcat_P(workstring1,(const char*) F(" El: "));
                dtostrf(configuration.park_elevation, 1, 0, workstring2);
                strcat(workstring1,workstring2);
              #endif
              //request_transient_message(workstring1,1,5000);
            } else {
              #if defined(FEATURE_ELEVATION_CONTROL)
                control_port->print(F("Park Azimuth: "));
                control_port->print(configuration.park_azimuth);
                control_port->print(F(" Elevation: "));
                control_port->println(configuration.park_elevation); 
              #else
                control_port->print(F("Park Azimuth: "));
                control_port->println(configuration.park_azimuth);
              #endif 
            }  
            #else
              #if defined(FEATURE_ELEVATION_CONTROL)
                control_port->print(F("Park Azimuth: "));
                control_port->print(configuration.park_azimuth);
                control_port->print(F(" Elevation: "));
                control_port->println(configuration.park_elevation); 
              #else
                control_port->print(F("Park Azimuth: "));
                control_port->println(configuration.park_azimuth);
              #endif 
            #endif      
          } else {
            if (input_buffer_index == 4){
              temp_int = (input_buffer[3] - 48);
            } else {
              if (input_buffer_index == 5){
                temp_int = ((input_buffer[3] - 48) * 10) + (input_buffer[4] - 48);
              } else {
                if (input_buffer_index == 6){
                  temp_int = ((input_buffer[3] - 48) * 100) + ((input_buffer[4] - 48) * 10) + (input_buffer[5] - 48);              
                } else {
                  control_port->print(F("Error."));
                }
              }
            }
          }
        }
      }
      if (temp_int != 999){
        control_port->print(F("Park "));
        if (input_buffer[2] == 'A'){
          configuration.park_azimuth = temp_int;
          control_port->print(F("Azimuth"));
        } else {
          configuration.park_elevation = temp_int;
          control_port->print(F("Elevation"));
        }
        configuration_dirty = 1;
        control_port->print(F(" set to "));
        control_port->println(temp_int);
      }
      break;
      #endif // FEATURE_PARK

    #ifdef FEATURE_ANCILLARY_PIN_CONTROL
    case 'N':      // \Nxx - turn pin on; xx = pin number
      if ((((input_buffer[2] > 47) && (input_buffer[2] < 58)) || (toupper(input_buffer[2]) == 'A')) && (input_buffer[3] > 47) && (input_buffer[3] < 58) && (input_buffer_index == 4)) {
        byte pin_value = 0;
        if (toupper(input_buffer[2]) == 'A') {
          pin_value = get_analog_pin(input_buffer[3] - 48);
        } else {
          pin_value = ((input_buffer[2] - 48) * 10) + (input_buffer[3] - 48);
        }
        pinModeEnhanced(pin_value, OUTPUT);
        digitalWriteEnhanced(pin_value, HIGH);
        strcpy(return_string, "OK");
      } else {
        strcpy(return_string, "Error");
      }
      break;
    case 'F':      // \Fxx - turn pin off; xx = pin number
      if ((((input_buffer[2] > 47) && (input_buffer[2] < 58)) || (toupper(input_buffer[2]) == 'A')) && (input_buffer[3] > 47) && (input_buffer[3] < 58) && (input_buffer_index == 4)) {
        byte pin_value = 0;
        if (toupper(input_buffer[2]) == 'A') {
          pin_value = get_analog_pin(input_buffer[3] - 48);
        } else {
          pin_value = ((input_buffer[2] - 48) * 10) + (input_buffer[3] - 48);
        }
        pinModeEnhanced(pin_value, OUTPUT);
        digitalWriteEnhanced(pin_value, LOW);
        strcpy_P(return_string, (const char*) F("OK"));
      } else {
        strcpy_P(return_string, (const char*) F("Error"));
      }
      break;
    case 'W':    // \Wxxyyy - turn on pin PWM; xx = pin number, yyy = PWM value (0-255)
      if (((input_buffer[2] > 47) && (input_buffer[2] < 58)) && (input_buffer[3] > 47) && (input_buffer[3] < 58)  && (input_buffer_index == 7)) {
        byte pin_value = 0;
        if (toupper(input_buffer[2]) == 'A') {
          pin_value = get_analog_pin(input_buffer[3] - 48);
        } else {
          pin_value = ((input_buffer[2] - 48) * 10) + (input_buffer[3] - 48);
        }
        int write_value = ((input_buffer[4] - 48) * 100) + ((input_buffer[5] - 48) * 10) + (input_buffer[6] - 48);
        if ((write_value >= 0) && (write_value < 256)) {
          pinModeEnhanced(pin_value, OUTPUT);
          analogWriteEnhanced(pin_value, write_value);
          strcpy_P(return_string, (const char*) F("OK"));
        } else {
          strcpy_P(return_string, (const char*) F("Error"));
        }
      } else {
        strcpy_P(return_string, (const char*) F("Error"));
      }
      break;
  #endif // FEATURE_ANCILLARY_PIN_CONTROL

  #if defined(FEATURE_AUTOPARK)

    case 'Y':
        if (input_buffer_index == 2){ // query command \Y
          if (configuration.autopark_active){
            strcpy_P(return_string, (const char*) F("Autopark is on, timer: "));
            dtostrf(configuration.autopark_time_minutes, 0, 0, temp_string);
            strcat(return_string, temp_string);
            strcat_P(return_string, (const char*) F(" minute"));
            if (configuration.autopark_time_minutes > 1){
              strcat(return_string, "s");
            }
          } else {
            strcpy_P(return_string, (const char*) F("Autopark is off"));
          }
        }
        if (input_buffer_index == 3){  // command \Yx
          if ((input_buffer[2] > 47) && (input_buffer[2] < 58)){
            if (input_buffer[2] == 48){       // deactivate command \Y0 
              configuration.autopark_time_minutes = 0; 
              valid_input_autopark = 1;
            }  
            if (input_buffer[2] != 48){  // set command \Yx
              configuration.autopark_time_minutes = input_buffer[2] - 48;
              valid_input_autopark = 1;
            }
          } else {
            strcpy(return_string, "Error");
          }
        }
        if (input_buffer_index == 4){  // set command \Yxx
          if ((input_buffer[2] > 47) && (input_buffer[2] < 58) && (input_buffer[3] > 47) && (input_buffer[3] < 58)){
              configuration.autopark_time_minutes = ((input_buffer[2] - 48) * 10) + (input_buffer[3] - 48);
              valid_input_autopark = 1;
          } else {
            strcpy_P(return_string, (const char*) F("Error"));
          }
        }         
        if (input_buffer_index == 5){ // set command \Yxxx
          if ((input_buffer[2] > 47) && (input_buffer[2] < 58) && (input_buffer[3] > 47) && (input_buffer[3] < 58) && (input_buffer[4] > 47) && (input_buffer[4] < 58)){
              configuration.autopark_time_minutes = ((input_buffer[2] - 48) * 100) + ((input_buffer[3] - 48) * 10) + (input_buffer[4] - 48);
              valid_input_autopark = 1;
          } else {
            strcpy_P(return_string, (const char*) F("Error"));
          }
        }   
        if (input_buffer_index == 6){ // set command \Yxxxxx
          if ((input_buffer[2] > 47) && (input_buffer[2] < 58) && (input_buffer[3] > 47) && (input_buffer[3] < 58) && (input_buffer[4] > 47) && (input_buffer[4] < 58)  && (input_buffer[5] > 47) && (input_buffer[5] < 58)){
              configuration.autopark_time_minutes = ((input_buffer[2] - 48) * 1000) + ((input_buffer[3] - 48) * 100) + ((input_buffer[4] - 48) * 10) + (input_buffer[5] - 48);
              valid_input_autopark = 1;
          } else {
            strcpy_P(return_string, (const char*) F("Error"));
          }
        } 
        if (valid_input_autopark){
          if (configuration.autopark_time_minutes > 0){
            strcpy_P(return_string, (const char*) F("Autopark on, timer: "));
            dtostrf(configuration.autopark_time_minutes, 0, 0, temp_string);
            strcat(return_string, temp_string);
            strcat_P(return_string, (const char*) F(" minute"));
            if (configuration.autopark_time_minutes > 1){
              strcat(return_string, "s");
            }
            configuration.autopark_active = 1;
          } else {
            strcpy_P(return_string, (const char*) F("Autopark off"));
            configuration.autopark_active = 0;
          }
          last_activity_time_autopark = millis();          
          configuration_dirty = 1;
        }                  
      break;
  #endif

  case '+':
    if (configuration.azimuth_display_mode == AZ_DISPLAY_MODE_OVERLAP_PLUS){
      configuration.azimuth_display_mode = AZ_DISPLAY_MODE_NORMAL;
      strcpy_P(return_string,(const char*) F("Azimuth Display Mode: Normal"));
    } else {
      if (configuration.azimuth_display_mode == AZ_DISPLAY_MODE_RAW){
        configuration.azimuth_display_mode = AZ_DISPLAY_MODE_OVERLAP_PLUS;
        strcpy_P(return_string,(const char*) F("Azimuth Display Mode: +Overlap"));
      } else {
        if (configuration.azimuth_display_mode == AZ_DISPLAY_MODE_NORMAL){
          configuration.azimuth_display_mode = AZ_DISPLAY_MODE_RAW;
          strcpy_P(return_string,(const char*) F("Azimuth Display Mode: Raw Degrees"));
        }
      }
    }
    configuration_dirty = 1;
    break;


      #if defined(FEATURE_AUDIBLE_ALERT)
        case '-':  // audible alert query and control
          if (input_buffer_index == 2){
            control_port->print(F("Audible alerts are "));
            if (configuration.audible_alert_enabled){
              control_port->print(F("en"));
            } else {
              control_port->print(F("dis"));
            }
            control_port->println(F("abled."));
            control_port->print(F("AZ target alert: o"));
            if (configuration.audible_alert_enabled_az_target){
              control_port->println(F("n."));
            } else {
              control_port->println(F("ff."));
            }
            control_port->print(F("EL target alert: o"));
            if (configuration.audible_alert_enabled_el_target){
              control_port->println(F("n."));
            } else {
              control_port->println(F("ff."));
            }
            if (configuration.audible_alert_enabled){
              control_port->print(F("Audible alert is o"));
              if (audible_alert(AUDIBLE_ALERT_SERVICE)){
                control_port->println(F("n."));
              } else {
                control_port->println(F("ff."));
              }
            }
          } else {
            if (input_buffer_index == 3){
              if (input_buffer[2] == '0'){
                configuration.audible_alert_enabled = 0;
                configuration_dirty = 1;
                control_port->println(F("Audible alerts disabled."));
              }
              if (input_buffer[2] == '1'){
                configuration.audible_alert_enabled = 1;
                configuration_dirty = 1;
                control_port->println(F("Audible alerts enabled."));
              }              
              if (input_buffer[2] == 'X'){
                audible_alert(AUDIBLE_ALERT_SILENCE);
                control_port->println(F("Audible alert silenced."));
              }
              if (input_buffer[2] == '~'){
                audible_alert(AUDIBLE_ALERT_MANUAL_ACTIVATE);
                control_port->println(F("Audible alert activated."));
              }  
            } else {
              if (input_buffer_index == 4){
                if (input_buffer[2] == 'A'){
                  if (input_buffer[3] == '0'){
                    configuration.audible_alert_enabled_az_target = 0;
                    configuration_dirty = 1;
                    control_port->println(F("AZ target audible alert disabled."));
                  }
                  if (input_buffer[3] == '1'){
                    configuration.audible_alert_enabled_az_target = 1;
                    configuration_dirty = 1;
                    control_port->println(F("AZ target audible alert enabled."));
                  }
                }
                if (input_buffer[2] == 'E'){
                  if (input_buffer[3] == '0'){
                    configuration.audible_alert_enabled_el_target = 0;
                    configuration_dirty = 1;
                    control_port->println(F("EL target audible alert disabled."));
                  }
                  if (input_buffer[3] == '1'){
                    configuration.audible_alert_enabled_el_target = 1;
                    configuration_dirty = 1;
                    control_port->println(F("EL target audible alert enabled."));
                  }
                }
              }

            }
          }
          break;
      #endif

// TODO : one big status query command    

  #if !defined(OPTION_SAVE_MEMORY_EXCLUDE_EXTENDED_COMMANDS)

    case '?':
      if (include_response_code){ 
        strcpy(return_string, "\\!??");  //  \\??xxyy - failed response back
      }
      if (input_buffer_index == 4){
        if ((input_buffer[2] == 'F') && (input_buffer[3] == 'S')) {  // \?FS - Full Status
          strconditionalcpy(return_string, "\\!OKFS",include_response_code);
          // AZ
          if (raw_azimuth < 100) {
            strcat(return_string,"0");
          }
          if (raw_azimuth < 10) {
            strcat(return_string,"0");
          }
          dtostrf(raw_azimuth,0,6,temp_string);
          strcat(return_string,temp_string);
          strcat(return_string,",");
          // EL
          #if defined(FEATURE_ELEVATION_CONTROL)
            if (elevation >= 0) {
              strcat(return_string,"+");
            } else {
              strcat(return_string,"-");
            }
            if (abs(elevation) < 100) {
              strcat(return_string,"0");
            }
            if (abs(elevation) < 10) {
              strcat(return_string,"0");
            }
            dtostrf(float(abs(elevation)),0,6,temp_string);
            strcat(return_string,temp_string); 
          #endif //  FEATURE_ELEVATION_CONTROL
          strcat(return_string,",");
          // AS
          dtostrf(az_state, 0, 0, temp_string);
          strcat(return_string, temp_string); 
          strcat(return_string,",");
          // ES
          #if defined(FEATURE_ELEVATION_CONTROL)
          dtostrf(el_state, 0, 0, temp_string);
          strcat(return_string, temp_string); 
          #endif
          strcat(return_string,",");                    

          // RC
          #ifdef FEATURE_GPS
            if (latitude < 0){strcat(return_string,"-");} else {strcat(return_string,"+");}
            dtostrf(abs(latitude),0,4,temp_string);
            strcat(return_string,temp_string);         
            strcat(return_string,",");
            if (longitude < 0){strcat(return_string,"-");} else {strcat(return_string,"+");}
            if (longitude < 100){strcat(return_string,"0");}
            dtostrf(abs(longitude),0,4,temp_string);
            strcat(return_string,temp_string); 
          #endif //FEATURE_GPS
          strcat(return_string,","); 
           // GS    
          #ifdef FEATURE_CLOCK
            if (clock_status == GPS_SYNC){                
              strcat(return_string,"1");
            } else {
              strcat(return_string,"0");
            }        
          #endif //FEATURE_CLOCK 
          strcat(return_string,","); 

          #ifdef FEATURE_CLOCK
            #if defined(OPTION_USE_OLD_TIME_CODE)
            update_time();
            #endif
            strcat(return_string,timezone_modified_clock_string());
          #endif //FEATURE_CLOCK 

          strcat(return_string,";");


        }
        if ((input_buffer[2] == 'A') && (input_buffer[3] == 'Z')) {  // \?AZ - query AZ
          strconditionalcpy(return_string, "\\!OKAZ", include_response_code);
          if (raw_azimuth < 100) {
            strcat(return_string,"0");
          }
          if (raw_azimuth < 10) {
            strcat(return_string,"0");
          }
          dtostrf(raw_azimuth,0,6,temp_string);
          strcat(return_string,temp_string); 
        }
        if ((input_buffer[2] == 'E') && (input_buffer[3] == 'L')) {  // \?EL - query EL
          #ifdef FEATURE_ELEVATION_CONTROL
            strconditionalcpy(return_string, "\\!OKFS", include_response_code);    
            if (elevation >= 0) {
              strcat(return_string,"+");
            } else {
              strcat(return_string,"-");
            }
            if (abs(elevation) < 100) {
              strcat(return_string,"0");
            }
            if (abs(elevation) < 10) {
              strcat(return_string,"0");
            }
            dtostrf(float(abs(elevation)),0,6,temp_string);
            strcat(return_string,temp_string); 
          #else // FEATURE_ELEVATION_CONTROL
            strcpy(return_string, "\\!??EL");
          #endif //FEATURE_ELEVATION_CONTROL 
        }
        if ((input_buffer[2] == 'A') && (input_buffer[3] == 'S')) {  // \?AS - AZ status
          strconditionalcpy(return_string, "\\!OKAS", include_response_code);
          dtostrf(az_state, 0, 0, temp_string);
          strcat(return_string, temp_string); 
        }  
        if ((input_buffer[2] == 'E') && (input_buffer[3] == 'S')) {  // \?ES - EL Status
          #ifdef FEATURE_ELEVATION_CONTROL
            strconditionalcpy(return_string, "\\!OKES", include_response_code);        
            dtostrf(el_state, 0, 0, temp_string);
            strcat(return_string, temp_string);
          #else // FEATURE_ELEVATION_CONTROL  
            strcpy(return_string, "\\!??ES");
          #endif //FEATURE_ELEVATION_CONTROL              
        }   
        if ((input_buffer[2] == 'P') && (input_buffer[3] == 'G')) {  // \?PG - Ping        
          strcpy(return_string, "\\!OKPG");     
        }    
          
        if ((input_buffer[2] == 'R') && (input_buffer[3] == 'L')) {  // \?RL - rotate left
          submit_request(AZ, REQUEST_CCW, 0, 121);
          strconditionalcpy(return_string, "\\!OKRL", include_response_code);     
        }     
        if ((input_buffer[2] == 'R') && (input_buffer[3] == 'R')) {  // \?RR - rotate right
          submit_request(AZ, REQUEST_CW, 0, 122);
          strconditionalcpy(return_string, "\\!OKRR", include_response_code);      
        }   
        if ((input_buffer[2] == 'R') && (input_buffer[3] == 'U')) {  //  \?RU - elevate up
          submit_request(EL, REQUEST_UP, 0, 129);
          strconditionalcpy(return_string, "\\!OKRU", include_response_code);       
        } 
        if ((input_buffer[2] == 'R') && (input_buffer[3] == 'D')) {  // \?RD - elevate down
          submit_request(EL, REQUEST_DOWN, 0, 130);
          strconditionalcpy(return_string, "\\!OKRD", include_response_code);       
        }  
        #ifdef FEATURE_GPS
          if ((input_buffer[2] == 'R') && (input_buffer[3] == 'C')) {  // \?RC - Read coordinates
            strconditionalcpy(return_string, "\\!OKRC", include_response_code);        
            if (latitude < 0){strcat(return_string,"-");} else {strcat(return_string,"+");}
            dtostrf(abs(latitude),0,4,temp_string);
            strcat(return_string,temp_string);         
            strcat(return_string," ");
            if (longitude < 0){strcat(return_string,"-");} else {strcat(return_string,"+");}
            if (longitude < 100){strcat(return_string,"0");}
            dtostrf(abs(longitude),0,4,temp_string);
            strcat(return_string,temp_string); 
          }
          if ((input_buffer[2] == 'R') && (input_buffer[3] == 'G')) {  // \?RG - Read grid square
            strconditionalcpy(return_string, "\\!OKRG", include_response_code);      
            strcat(return_string,coordinates_to_maidenhead(latitude,longitude));
          }
        #endif //FEATURE_GPS
        #ifdef FEATURE_CLOCK
          if ((input_buffer[2] == 'G') && (input_buffer[3] == 'S')) { // \?GS - query GPS sync
            strconditionalcpy(return_string,"\\!OKGS", include_response_code);
            if (clock_status == GPS_SYNC){                
              strcat(return_string,"1");
            } else {
              strcat(return_string,"0");
            }        
          }
        #endif //FEATURE_CLOCK 

        if ((input_buffer[2] == 'S') && (input_buffer[3] == 'A')) {  // \?SA - stop azimuth rotation
          submit_request(AZ, REQUEST_STOP, 0, 124);
          strconditionalcpy(return_string,"\\!OKSA", include_response_code);
        }   
        if ((input_buffer[2] == 'S') && (input_buffer[3] == 'E')) {  // \?SE - stop elevation rotation
          #ifdef FEATURE_ELEVATION_CONTROL
            submit_request(EL, REQUEST_STOP, 0, 125);
          #endif
          strconditionalcpy(return_string,"\\!OKSE", include_response_code);
        } 
        if ((input_buffer[2] == 'S') && (input_buffer[3] == 'S')) {  // \?SS - stop all rotation
          submit_request(AZ, REQUEST_STOP, 0, 124);
          #ifdef FEATURE_ELEVATION_CONTROL
            submit_request(EL, REQUEST_STOP, 0, 125);
          #endif
          strconditionalcpy(return_string,"\\!OKSS", include_response_code);
        }   

        if ((input_buffer[2] == 'C') && (input_buffer[3] == 'L')) {  // \?CL - read the clock
          #ifdef FEATURE_CLOCK
            strconditionalcpy(return_string,"\\!OKCL", include_response_code);
            #if defined(OPTION_USE_OLD_TIME_CODE)
            update_time();
            #endif //#if defined(OPTION_USE_OLD_TIME_CODE)
            strcat(return_string,timezone_modified_clock_string());
          #else //FEATURE_CLOCK
            strconditionalcpy(return_string,"\\!??CL", include_response_code);
          #endif //FEATURE_CLOCK
        }

        if ((input_buffer[2] == 'R') && (input_buffer[3] == 'B')) {  // \?RB - reboot
          reset_the_unit = 1;
          // wdt_enable(WDTO_30MS); while (1) {}
        }

        if ((input_buffer[2] == 'C') && (input_buffer[3] == 'V')) {  // \?CV Code Verson
          strconditionalcpy(return_string,"\\!OKCV", include_response_code);
          strcat(return_string,CODE_VERSION);
        }

        if ((input_buffer[2] == 'A') && (input_buffer[3] == 'O')) {  // \?AO - Azimuth Full CCW Calibration
          read_azimuth(1);
          configuration.analog_az_full_ccw = analog_az;
          write_settings_to_eeprom();        
          strconditionalcpy(return_string,"\\!OKAO", include_response_code);
        }

        if ((input_buffer[2] == 'A') && (input_buffer[3] == 'F')) {  // \?AF - Azimuth Full CW Calibration
          read_azimuth(1);
          configuration.analog_az_full_cw = analog_az;
          write_settings_to_eeprom();        
          strconditionalcpy(return_string,"\\!OKAF", include_response_code);
        }
  
        #if defined(FEATURE_ELEVATION_CONTROL)
          if ((input_buffer[2] == 'E') && (input_buffer[3] == 'O')) {  // \?EO - Elevation Full DOWN Calibration
            read_elevation(1);
            configuration.analog_el_0_degrees = analog_el;
            write_settings_to_eeprom();        
            strconditionalcpy(return_string,"\\!OKEO", include_response_code);
          }

          if ((input_buffer[2] == 'E') && (input_buffer[3] == 'F')) {  // \?EF - Elevation Full UP Calibration
            read_elevation(1);
            configuration.analog_el_max_elevation = analog_el;
            write_settings_to_eeprom();        
            strconditionalcpy(return_string,"\\!OKEF", include_response_code);
          }
        #endif  

        #if defined(FEATURE_NEXTION_DISPLAY)
          if ((input_buffer[2] == 'N') && (input_buffer[3] == 'G')) {  // \?NG - output Nextion gSC variable immediately on Nextion por
            output_nextion_gSC_variable();
            if (input_source != SOURCE_NEXTION){
               strconditionalcpy(return_string,"\\!OKNG", include_response_code);
            }
          }
        #endif
      } //if (input_buffer_index == 4)
 
    if (input_buffer_index == 6){
      if ((input_buffer[2] == 'D') && (input_buffer[3] == 'O')) {  // \?DOxx - digital pin initialize as output; xx = pin # (01, 02, A0,etc.)
        if ((((input_buffer[4] > 47) && (input_buffer[4] < 58)) || (toupper(input_buffer[4]) == 'A')) && (input_buffer[5] > 47) && (input_buffer[5] < 58)) {
          byte pin_value = 0;
          if (toupper(input_buffer[4]) == 'A') {
            pin_value = get_analog_pin(input_buffer[4] - 48);
          } else {
            pin_value = ((input_buffer[4] - 48) * 10) + (input_buffer[5] - 48);
          }
          strconditionalcpy(return_string,"\\!OKDO", include_response_code);
          pinModeEnhanced(pin_value, OUTPUT);
        }
      }

      if ((input_buffer[2] == 'D') && ((input_buffer[3] == 'H') || (input_buffer[3] == 'L'))) { // \?DLxx - digital pin write low; xx = pin #   \?DHxx - digital pin write high; xx = pin # 
        if ((((input_buffer[4] > 47) && (input_buffer[4] < 58)) || (toupper(input_buffer[4]) == 'A')) && (input_buffer[5] > 47) && (input_buffer[5] < 58)) {
          byte pin_value = 0;
          if (toupper(input_buffer[4]) == 'A') {
            pin_value = get_analog_pin(input_buffer[5] - 48);
          } else {
            pin_value = ((input_buffer[4] - 48) * 10) + (input_buffer[5] - 48);
          }
          if (input_buffer[3] == 'H') {
            digitalWriteEnhanced(pin_value, HIGH);
            strconditionalcpy(return_string,"\\!OKDH", include_response_code);
          } else {
            digitalWriteEnhanced(pin_value, LOW);
            strconditionalcpy(return_string,"\\!OKDL", include_response_code);
          }
        }
      }





/*

Not implemented yet:

\\SWxy - serial write byte; x = serial port # (0, 1, 2, 3), y = byte to write
\\SDx - deactivate serial read event; x = port #
\\SSxyyyyyy... - serial write string; x = port #, yyyy = string of characters to send (variable length)
\\SAx - activate serial read event; x = port #

*/


      if ((input_buffer[2] == 'D') && (input_buffer[3] == 'I')) {  // \?DIxx - digital pin initialize as input; xx = pin #
        if ((((input_buffer[4] > 47) && (input_buffer[4] < 58)) || (toupper(input_buffer[4]) == 'A')) && (input_buffer[5] > 47) && (input_buffer[5] < 58)) {
          byte pin_value = 0;
          if (toupper(input_buffer[4]) == 'A') {
            pin_value = get_analog_pin(input_buffer[5] - 48);
          } else {
            pin_value = ((input_buffer[4] - 48) * 10) + (input_buffer[5] - 48);
          }
          pinModeEnhanced(pin_value, INPUT);
          strconditionalcpy(return_string,"\\!OKDI", include_response_code);
        }
      }

      if ((input_buffer[2] == 'D') && (input_buffer[3] == 'P')) {  // \?DPxx - digital pin initialize as input with pullup; xx = pin #
        if ((((input_buffer[4] > 47) && (input_buffer[4] < 58)) || (toupper(input_buffer[4]) == 'A')) && (input_buffer[5] > 47) && (input_buffer[5] < 58)) {
          byte pin_value = 0;
          if (toupper(input_buffer[4]) == 'A') {
            pin_value = get_analog_pin(input_buffer[5] - 48);
          } else {
            pin_value = ((input_buffer[4] - 48) * 10) + (input_buffer[5] - 48);
          }
          pinModeEnhanced(pin_value, INPUT);
          digitalWriteEnhanced(pin_value, HIGH);
          strconditionalcpy(return_string,"\\!OKDP", include_response_code);
        }
      }

      if ((input_buffer[2] == 'D') && (input_buffer[3] == 'R')) {  // \?DRxx - digital pin read; xx = pin #
        if ((((input_buffer[4] > 47) && (input_buffer[4] < 58)) || (toupper(input_buffer[4]) == 'A')) && (input_buffer[5] > 47) && (input_buffer[5] < 58)) {
          byte pin_value = 0;
          if (toupper(input_buffer[4]) == 'A') {
            pin_value = get_analog_pin(input_buffer[5] - 48);
          } else {
            pin_value = ((input_buffer[4] - 48) * 10) + (input_buffer[5] - 48);
          }
          byte pin_read = digitalReadEnhanced(pin_value);
          strconditionalcpy(return_string,"\\!OKDR", include_response_code);
          dtostrf((input_buffer[4]-48),0,0,temp_string);
          strcat(return_string,temp_string);              
          dtostrf((input_buffer[5]-48),0,0,temp_string);
          strcat(return_string,temp_string);  
          if (pin_read) {
            strcat(return_string,"1");
          } else {
            strcat(return_string,"0");
          }
        }
      }
      if ((input_buffer[2] == 'A') && (input_buffer[3] == 'R')) {  //  \?ARxx - analog pin read; xx = pin #
        if ((((input_buffer[4] > 47) && (input_buffer[4] < 58)) || (toupper(input_buffer[4]) == 'A')) && (input_buffer[5] > 47) && (input_buffer[5] < 58)) {
          byte pin_value = 0;
          if (toupper(input_buffer[4]) == 'A') {
            pin_value = get_analog_pin(input_buffer[5] - 48);
          } else {
            pin_value = ((input_buffer[4] - 48) * 10) + (input_buffer[5] - 48);
          }
          int pin_read = analogReadEnhanced(pin_value);
          strconditionalcpy(return_string,"\\!OKAR", include_response_code);
          if (toupper(input_buffer[4]) == 'A') {
            strcat(return_string,"A");
          } else {
            dtostrf((input_buffer[4]-48),0,0,temp_string);
            strcat(return_string,temp_string);
          }
                        
          dtostrf((input_buffer[5]-48),0,0,temp_string);
          strcat(return_string,temp_string);  
          if (pin_read < 1000) {
            strcat(return_string,"0");
          }
          if (pin_read < 100) {
            strcat(return_string,"0");
          }
          if (pin_read < 10) {
            strcat(return_string,"0");
          }
          dtostrf(pin_read,0,0,temp_string);
          strcat(return_string,temp_string);             
        }
      }

      if ((input_buffer[2] == 'N') && (input_buffer[3] == 'T')) { // \?NTxx - no tone; xx = pin #
          byte pin_value = 0;
          if (toupper(input_buffer[4]) == 'A') {
            pin_value = get_analog_pin(input_buffer[5] - 48);
          } else {
            pin_value = ((input_buffer[4] - 48) * 10) + (input_buffer[5] - 48);
          }
          noTone(pin_value);
          strconditionalcpy(return_string,"\\!OKNT", include_response_code);
        }
    }  //if ((input_buffer_index == 6)

    int hit_decimal = 0;

    /*
        \?TSxxxx            - set satellite tracking check interval (mS)
        \?TUxxxx            - set sun tracking check interval (mS)
        \?TMxxxx            - set moon tracking check interval (mS)

        \?TXxxxx            - set satellite rotation interval (mS)
        \?TYxxxx            - set sun rotation interval (mS)
        \?TZxxxx            - set moon rotation interval (mS)

        \?TAx[.]x            - set satellite degrees difference threshold
        \?TBx[.]x            - set sun degrees difference threshold
        \?TCx[.]x            - set moon degrees difference threshold
    */     

    //ZZZZZZ
    #if defined(FEATURE_SATELLITE_TRACKING) || defined(FEATURE_MOON_TRACKING) || defined(FEATURE_SUN_TRACKING)
      if (input_buffer[2] == 'T'){
        if ((input_buffer[3] == 'S') || (input_buffer[3] == 'U') || (input_buffer[3] == 'M') ||
            (input_buffer[3] == 'X') || (input_buffer[3] == 'Y') || (input_buffer[3] == 'Z'))
        {
          temp_int = 0;
          for (int x = 4;x < input_buffer_index;x++){
            temp_int = (temp_int * 10) + (input_buffer[x] - 48);
          }   
          #if defined(FEATURE_SATELLITE_TRACKING)
            if (input_buffer[3] == 'S'){
              configuration.tracking_sat_check_frequency_ms = temp_int;
              strconditionalcpy(return_string,"\\!OKTS", include_response_code);
              configuration_dirty = 1;
            }
          #endif
          #if defined(FEATURE_SUN_TRACKING)
            if (input_buffer[3] == 'U'){
              configuration.tracking_sun_check_frequency_ms = temp_int;
              strconditionalcpy(return_string,"\\!OKTU", include_response_code);
              configuration_dirty = 1;
            }
          #endif
          #if defined(FEATURE_MOON_TRACKING)
            if (input_buffer[3] == 'M'){
              configuration.tracking_moon_check_frequency_ms = temp_int;
              strconditionalcpy(return_string,"\\!OKTM", include_response_code);
              configuration_dirty = 1;
            }
          #endif
          #if defined(FEATURE_SATELLITE_TRACKING)
            if (input_buffer[3] == 'X'){
              configuration.tracking_sat_minimum_rotation_interval_ms = temp_int;
              strconditionalcpy(return_string,"\\!OKTX", include_response_code);
              configuration_dirty = 1;
            }
          #endif  
          #if defined(FEATURE_SUN_TRACKING)
            if (input_buffer[3] == 'Y'){
              configuration.tracking_sun_minimum_rotation_interval_ms = temp_int;
              strconditionalcpy(return_string,"\\!OKTY", include_response_code);
              configuration_dirty = 1;
            }
          #endif  
          #if defined(FEATURE_MOON_TRACKING)
            if (input_buffer[3] == 'Z'){
              configuration.tracking_moon_minimum_rotation_interval_ms = temp_int;
              strconditionalcpy(return_string,"\\!OKTZ", include_response_code);
              configuration_dirty = 1;
            }  
          #endif          

        }
        if ((input_buffer[3] == 'A') || (input_buffer[3] == 'B') || (input_buffer[3] == 'C')){
          hit_decimal = 0;
          tempfloat = 0;
          for (int x = 4;x < input_buffer_index;x++){
            if(input_buffer[x] == '.'){
              hit_decimal = 10;
            } else {
              if (hit_decimal > 0){
                tempfloat = tempfloat + ((float)(input_buffer[x] - 48) / (float)hit_decimal);
                hit_decimal = hit_decimal * 10;
              } else {
                tempfloat = (tempfloat * 10) + (input_buffer[x] - 48);
              }
            }
          }  
          #if defined(FEATURE_SATELLITE_TRACKING)
            if (input_buffer[3] == 'A'){
              configuration.tracking_sat_degrees_difference_threshold = tempfloat;
              strconditionalcpy(return_string,"\\!OKTA", include_response_code);
              configuration_dirty = 1;
            }
          #endif
          #if defined(FEATURE_SUN_TRACKING)
            if (input_buffer[3] == 'B'){
              configuration.tracking_sun_degrees_difference_threshold = tempfloat;
              strconditionalcpy(return_string,"\\!OKTB", include_response_code);
              configuration_dirty = 1;
            }
          #endif
          #if defined(FEATURE_MOON_TRACKING)
            if (input_buffer[3] == 'C'){
              configuration.tracking_moon_degrees_difference_threshold = tempfloat;
              strconditionalcpy(return_string,"\\!OKTC", include_response_code);
              configuration_dirty = 1;
            }
          #endif
        }  

      }

    #endif  //defined(FEATURE_SATELLITE_TRACKING) || defined(FEATURE_MOON_TRACKING) || defined(FEATURE_SUN_TRACKING)

    if ((input_buffer[2] == 'G') && (input_buffer[3] == 'A')) {  // \?GAxxx.x - go to AZ xxx.x
      heading = 0;
      for (int x = 4;x < input_buffer_index;x++){
        if(input_buffer[x] == '.'){
          hit_decimal = 10;
        } else {
          if (hit_decimal > 0){
            heading = heading + ((float)(input_buffer[x] - 48) / (float)hit_decimal);
            hit_decimal = hit_decimal * 10;
          } else {
            heading = (heading * 10) + (input_buffer[x] - 48);
          }
        }
      }   
      // debug.print("process_backslash_command: heading:");
      // debug.println(heading);      
      if ((heading >= 0) && (heading < 451)) {  
        submit_request(AZ, REQUEST_AZIMUTH, (heading), 136);
        strconditionalcpy(return_string,"\\!OKGA", include_response_code);
      } else {
        strconditionalcpy(return_string,"\\!??GA", include_response_code);
      }
    }  
    if ((input_buffer[2] == 'G') && (input_buffer[3] == 'E')) {  // \?GExxx.x - go to EL
      #ifdef FEATURE_ELEVATION_CONTROL
        heading = 0;
        for (int x = 4;x < input_buffer_index;x++){
          if(input_buffer[x] == '.'){
            hit_decimal = 10;
          } else {
            if (hit_decimal > 0){
              heading = heading + ((float)(input_buffer[x] - 48) / (float)hit_decimal);
              hit_decimal = hit_decimal * 10;
            } else {
              heading = (heading * 10) + (input_buffer[x] - 48);
            }
          }
        }   
        // debug.print("process_backslash_command: heading:");
        // debug.println(heading);  
        if ((heading >= 0) && (heading < 181)) {
          submit_request(EL, REQUEST_ELEVATION, (heading), 37);
          strconditionalcpy(return_string,"\\!OKGE", include_response_code);
        } else {
          strconditionalcpy(return_string,"\\!??GE", include_response_code);
        }
      #else 
        strconditionalcpy(return_string,"\\!OKGE", include_response_code);  
      #endif // #FEATURE_ELEVATION_CONTROL  
    } 


    // \?CGxxxx or \?CGxxxxxx  - Convert grid to coordinates
    if (((input_buffer_index == 8) || (input_buffer_index == 10)) && ((input_buffer[2] == 'C') && (input_buffer[3] == 'G'))){    
      strconditionalcpy(return_string,"\\!OKCG", include_response_code);
      temp_string[0] = input_buffer[4];
      temp_string[1] = input_buffer[5];
      temp_string[2] = input_buffer[6];
      temp_string[3] = input_buffer[7];
      if (input_buffer_index == 10){  // grab the subsquare if it was provided
        temp_string[4] = input_buffer[8];
        temp_string[5] = input_buffer[9];        
      } else {
        temp_string[4] = 0;
      }
      float temp_latitude = 0;
      float temp_longitude = 0;

      maidenhead_to_coordinates(temp_string,&temp_latitude,&temp_longitude);

      if (temp_latitude < 0){strcat(return_string,"-");} else {strcat(return_string,"+");}
      dtostrf(abs(temp_latitude),0,6,temp_string);
      strcat(return_string,temp_string);         
      strcat(return_string," ");
      if (temp_longitude < 0){strcat(return_string,"-");} else {strcat(return_string,"+");}
      if (temp_longitude < 100){strcat(return_string,"0");}
      dtostrf(abs(temp_longitude),0,6,temp_string);
      strcat(return_string,temp_string); 
    } // \?CGxxxx or \?CGxxxxxx  - Convert grid to coordinates




    //#if defined(FEATURE_MOON_TRACKING) || defined(FEATURE_SUN_TRACKING) || defined(FEATURE_CLOCK) || defined(FEATURE_GPS) || defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(OPTION_DISPLAY_ALT_HHMM_CLOCK_AND_MAIDENHEAD) || defined(OPTION_DISPLAY_CONSTANT_HHMMSS_CLOCK_AND_MAIDENHEAD)
      // \?GCxxxx xxxx  - go to coordinate target (rotate azimuth)
      if ((input_buffer[2] == 'G') && (input_buffer[3] == 'C')){ 

        float temp_latitude = 0;
        float temp_longitude = 0;
        byte hit_space = 0;
        byte latitude_negative = 0;
        byte longitude_negative = 0;

        for (int x = 4;x < input_buffer_index;x++){
          if(input_buffer[x] == ' '){
            hit_space = 1;
            hit_decimal = 0;
          } else {
            if (hit_space){
              // parsing longitude
              if(input_buffer[x] == '.'){   
                hit_decimal = 10;  
              } else {
                if(input_buffer[x] == '-'){
                  longitude_negative = 1;
                } else {
                  if (hit_decimal > 0){
                    temp_longitude = temp_longitude + ((float)(input_buffer[x] - 48) / (float)hit_decimal);
                    hit_decimal = hit_decimal * 10;
                  } else {
                    temp_longitude = (temp_longitude * 10) + (input_buffer[x] - 48);
                  }
                }
              }
            } else {
              // parsing latitude
              if(input_buffer[x] == '.'){   
                hit_decimal = 10;  
              } else {
                if(input_buffer[x] == '-'){
                  latitude_negative = 1;
                } else {
                  if (hit_decimal > 0){
                    temp_latitude = temp_latitude + ((float)(input_buffer[x] - 48) / (float)hit_decimal);
                    hit_decimal = hit_decimal * 10;
                  } else {
                    temp_latitude = (temp_latitude * 10) + (input_buffer[x] - 48);
                  }
                }
              }
            }
          }
        }  
        if ((temp_latitude >= 0) && (temp_latitude <= 90) && (temp_longitude >= 0) && (temp_longitude <= 180)){
          if (latitude_negative){temp_latitude = temp_latitude * -1.0;}
          if (longitude_negative){temp_longitude = temp_longitude * -1.0;}
          strconditionalcpy(return_string,"\\!OKCC", include_response_code);
          float temp_bearing = calculate_target_bearing(latitude,longitude,temp_latitude,temp_longitude);
          dtostrf(temp_bearing,0,DISPLAY_DECIMAL_PLACES,temp_string);
          strcat(return_string,temp_string);
          submit_request(AZ, REQUEST_AZIMUTH, temp_bearing, DBG_BACKSLASH_GC_CMD);
        }
      } 
    //#endif 


    //#if defined(FEATURE_MOON_TRACKING) || defined(FEATURE_SUN_TRACKING) || defined(FEATURE_CLOCK) || defined(FEATURE_GPS) || defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(OPTION_DISPLAY_ALT_HHMM_CLOCK_AND_MAIDENHEAD) || defined(OPTION_DISPLAY_CONSTANT_HHMMSS_CLOCK_AND_MAIDENHEAD)
      // \?GTxxxx or \?GTxxxxxx  - Go to (rotate azimuth) to grid target
      if (((input_buffer_index == 8) || (input_buffer_index == 10)) && ((input_buffer[2] == 'G') && (input_buffer[3] == 'T'))){    
        temp_string[0] = input_buffer[4];
        temp_string[1] = input_buffer[5];
        temp_string[2] = input_buffer[6];
        temp_string[3] = input_buffer[7];
        if (input_buffer_index == 10){  // grab the subsquare if it was provided
          temp_string[4] = input_buffer[8];
          temp_string[5] = input_buffer[9];        
        } else {
          temp_string[4] = 0;
        }
        float temp_latitude = 0;
        float temp_longitude = 0;

        maidenhead_to_coordinates(temp_string,&temp_latitude,&temp_longitude);

        if ((temp_latitude >= -90) && (temp_latitude <= 90) && (temp_longitude >= -180) && (temp_longitude <= 180)){
          strconditionalcpy(return_string,"\\!OKGT", include_response_code);
          float temp_bearing = calculate_target_bearing(latitude,longitude,temp_latitude,temp_longitude);
          dtostrf(temp_bearing,0,DISPLAY_DECIMAL_PLACES,temp_string);
          strcat(return_string,temp_string);
          submit_request(AZ, REQUEST_AZIMUTH, temp_bearing, DBG_BACKSLASH_GT_CMD);
        }
      }  
    //#endif 

  
    //#if defined(FEATURE_MOON_TRACKING) || defined(FEATURE_SUN_TRACKING) || defined(FEATURE_CLOCK) || defined(FEATURE_GPS) || defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(OPTION_DISPLAY_ALT_HHMM_CLOCK_AND_MAIDENHEAD) || defined(OPTION_DISPLAY_CONSTANT_HHMMSS_CLOCK_AND_MAIDENHEAD)
      // \?BGxxxx or \?BGxxxxxx  - Calculate bearing to target grid
      if (((input_buffer_index == 8) || (input_buffer_index == 10)) && ((input_buffer[2] == 'B') && (input_buffer[3] == 'G'))){    
        temp_string[0] = input_buffer[4];
        temp_string[1] = input_buffer[5];
        temp_string[2] = input_buffer[6];
        temp_string[3] = input_buffer[7];
        if (input_buffer_index == 10){  // grab the subsquare if it was provided
          temp_string[4] = input_buffer[8];
          temp_string[5] = input_buffer[9];        
        } else {
          temp_string[4] = 0;
        }
        float temp_latitude = 0;
        float temp_longitude = 0;

        maidenhead_to_coordinates(temp_string,&temp_latitude,&temp_longitude);

        if ((temp_latitude >= -90) && (temp_latitude <= 90) && (temp_longitude >= -180) && (temp_longitude <= 180)){
          strconditionalcpy(return_string,"\\!OKBG", include_response_code);
          dtostrf(calculate_target_bearing(latitude,longitude,temp_latitude,temp_longitude),0,DISPLAY_DECIMAL_PLACES,temp_string);
          strcat(return_string,temp_string);
        }
      }  
    //#endif 

    //#if defined(FEATURE_MOON_TRACKING) || defined(FEATURE_SUN_TRACKING) || defined(FEATURE_CLOCK) || defined(FEATURE_GPS) || defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(OPTION_DISPLAY_ALT_HHMM_CLOCK_AND_MAIDENHEAD) || defined(OPTION_DISPLAY_CONSTANT_HHMMSS_CLOCK_AND_MAIDENHEAD)
      // \?BCxxxx xxxx  - Calculate bearing to target coordinates
      if ((input_buffer[2] == 'B') && (input_buffer[3] == 'C')){ 

        float temp_latitude = 0;
        float temp_longitude = 0;
        byte hit_space = 0;
        byte latitude_negative = 0;
        byte longitude_negative = 0;

        for (int x = 4;x < input_buffer_index;x++){
          if(input_buffer[x] == ' '){
            hit_space = 1;
            hit_decimal = 0;
          } else {
            if (hit_space){
              // parsing longitude
              if(input_buffer[x] == '.'){   
                hit_decimal = 10;  
              } else {
                if(input_buffer[x] == '-'){
                  longitude_negative = 1;
                } else {
                  if (hit_decimal > 0){
                    temp_longitude = temp_longitude + ((float)(input_buffer[x] - 48) / (float)hit_decimal);
                    hit_decimal = hit_decimal * 10;
                  } else {
                    temp_longitude = (temp_longitude * 10) + (input_buffer[x] - 48);
                  }
                }
              }
            } else {
              // parsing latitude
              if(input_buffer[x] == '.'){   
                hit_decimal = 10;  
              } else {
                if(input_buffer[x] == '-'){
                  latitude_negative = 1;
                } else {
                  if (hit_decimal > 0){
                    temp_latitude = temp_latitude + ((float)(input_buffer[x] - 48) / (float)hit_decimal);
                    hit_decimal = hit_decimal * 10;
                  } else {
                    temp_latitude = (temp_latitude * 10) + (input_buffer[x] - 48);
                  }
                }
              }
            }
          }
        }  
        if ((temp_latitude >= 0) && (temp_latitude <= 90) && (temp_longitude >= 0) && (temp_longitude <= 180)){
          if (latitude_negative){temp_latitude = temp_latitude * -1.0;}
          if (longitude_negative){temp_longitude = temp_longitude * -1.0;}
          strconditionalcpy(return_string,"\\!OKBC", include_response_code);
          dtostrf(calculate_target_bearing(latitude,longitude,temp_latitude,temp_longitude),0,DISPLAY_DECIMAL_PLACES,temp_string);
          strcat(return_string,temp_string);
        }
      } 
    //#endif 




    // \?CCxxxx xxxx  - Convert coordinates to grid
    if ((input_buffer[2] == 'C') && (input_buffer[3] == 'C')){ 

      float temp_latitude = 0;
      float temp_longitude = 0;
      byte hit_space = 0;
      byte latitude_negative = 0;
      byte longitude_negative = 0;

      for (int x = 4;x < input_buffer_index;x++){
        if(input_buffer[x] == ' '){
          hit_space = 1;
          hit_decimal = 0;
        } else {

          if (hit_space){
            // parsing longitude
            if(input_buffer[x] == '.'){   
              hit_decimal = 10;  
            } else {
              if(input_buffer[x] == '-'){
                longitude_negative = 1;
              } else {
                if (hit_decimal > 0){
                  temp_longitude = temp_longitude + ((float)(input_buffer[x] - 48) / (float)hit_decimal);
                  hit_decimal = hit_decimal * 10;
                } else {
                  temp_longitude = (temp_longitude * 10) + (input_buffer[x] - 48);
                }
              }
            }

          } else {

            // parsing latitude
            if(input_buffer[x] == '.'){   
              hit_decimal = 10;  
            } else {
              if(input_buffer[x] == '-'){
                latitude_negative = 1;
              } else {
                if (hit_decimal > 0){
                  temp_latitude = temp_latitude + ((float)(input_buffer[x] - 48) / (float)hit_decimal);
                  hit_decimal = hit_decimal * 10;
                } else {
                  temp_latitude = (temp_latitude * 10) + (input_buffer[x] - 48);
                }
              }
            }
          }
        }


      }   
    
      if ((temp_latitude >= 0) && (temp_latitude <= 90) && (temp_longitude >= 0) && (temp_longitude <= 180)){
        if (latitude_negative){temp_latitude = temp_latitude * -1.0;}
        if (longitude_negative){temp_longitude = temp_longitude * -1.0;}
        strconditionalcpy(return_string,"\\!OKCC", include_response_code);
        strcat(return_string,coordinates_to_maidenhead(temp_latitude,temp_longitude));
      }

    }               
    


    if (input_buffer_index == 9) {

      // if ((input_buffer[2] == 'G') && (input_buffer[3] == 'A')) {  // \?GAxxx.x - go to AZ xxx.x
      //   heading = ((input_buffer[4] - 48) * 100.) + ((input_buffer[5] - 48) * 10.) + (input_buffer[6] - 48.) + ((input_buffer[8] - 48) / 10.);     
      //   if (((heading >= 0) && (heading < 451))  && (input_buffer[7] == '.')) {
      //     submit_request(AZ, REQUEST_AZIMUTH, (heading), 136);
      //     strconditionalcpy(return_string,"\\!OKGA", include_response_code);
      //   } else {
      //     strconditionalcpy(return_string,"\\!??GA", include_response_code);
      //   }
      // }  
      // if ((input_buffer[2] == 'G') && (input_buffer[3] == 'E')) {  // \?GExxx.x - go to EL
      //   #ifdef FEATURE_ELEVATION_CONTROL
      //     heading = ((input_buffer[4] - 48) * 100.) + ((input_buffer[5] - 48) * 10.) + (input_buffer[5] - 48) + ((input_buffer[8] - 48) / 10.);
      //     if (((heading >= 0) && (heading < 181)) && (input_buffer[7] == '.')) {
      //       submit_request(EL, REQUEST_ELEVATION, (heading), 37);
      //       strconditionalcpy(return_string,"\\!OKGE", include_response_code);
      //     } else {
      //       strconditionalcpy(return_string,"\\!??GE", include_response_code);
      //     }
      //   #else 
      //     strconditionalcpy(return_string,"\\!OKGE", include_response_code);  
      //   #endif // #FEATURE_ELEVATION_CONTROL  
      // } 


      if ((input_buffer[2] == 'A') && (input_buffer[3] == 'W')) {  // \?AWxxyyy - analog pin write; xx = pin #, yyy = value to write (0 - 255)
        if ((((input_buffer[4] > 47) && (input_buffer[4] < 58)) || (toupper(input_buffer[4]) == 'A')) && (input_buffer[5] > 47) && (input_buffer[5] < 58)) {
          byte pin_value = 0;
          if (toupper(input_buffer[4]) == 'A') {
            pin_value = get_analog_pin(input_buffer[5] - 48);
          } else {
            pin_value = ((input_buffer[4] - 48) * 10) + (input_buffer[5] - 48);
          }
          int write_value = ((input_buffer[6] - 48) * 100) + ((input_buffer[7] - 48) * 10) + (input_buffer[8] - 48);
          if ((write_value >= 0) && (write_value < 256)) {
            analogWriteEnhanced(pin_value, write_value);
            strconditionalcpy(return_string,"\\!OKAW", include_response_code);
          }
        }
      }
    } //if (input_buffer_index == 9)

    if (input_buffer_index == 10) {
      if ((input_buffer[2] == 'D') && (input_buffer[3] == 'T')) { // \?DTxxyyyy - digital pin tone output; xx = pin #, yyyy = frequency
        if ((((input_buffer[4] > 47) && (input_buffer[4] < 58)) || (toupper(input_buffer[4]) == 'A')) && (input_buffer[5] > 47) && (input_buffer[5] < 58)) {
          byte pin_value = 0;
          if (toupper(input_buffer[4]) == 'A') {
            pin_value = get_analog_pin(input_buffer[5] - 48);
          } else {
            pin_value = ((input_buffer[4] - 48) * 10) + (input_buffer[5] - 48);
          }
          int write_value = ((input_buffer[6] - 48) * 1000) + ((input_buffer[7] - 48) * 100) + ((input_buffer[8] - 48) * 10) + (input_buffer[9] - 48);
          if ((write_value >= 0) && (write_value <= 9999)) {
            tone(pin_value, write_value);
            strconditionalcpy(return_string,"\\!OKDT", include_response_code);

          }
        }
      }
    }  //if (input_buffer_index == 10)


      break; //case '\\'



  #endif  //!defined(OPTION_SAVE_MEMORY_EXCLUDE_EXTENDED_COMMANDS)

  #endif //!defined(OPTION_SAVE_MEMORY_EXCLUDE_BACKSLASH_CMDS)


  } // switch 

  return(0);
} // process_backslash_command

//-----------------------------------------------------------------------

#ifdef FEATURE_EASYCOM_EMULATION
void process_easycom_command(byte * easycom_command_buffer, int easycom_command_buffer_index, byte source_port, char * return_string){


  /* Easycom protocol implementation
   *
   * Implemented commands:
   *
   * Command      Meaning     Parameters
   * -------      -------     ----------
   *
   * ML           Move Left
   * MR           Move Right
   * MU           Move Up
   * MD           Move Down
   * SA           Stop azimuth moving
   * SE           Stop elevation moving
   *
   * VE           Request Version
   * AZ           Query azimuth
   * AZx.x        Rotate to Azimuth
   * AZxx.x       Rotate to Azimuth
   * AZxxx.x      Rotate to Azimuth
   * EL           Request Elevation
   * ELx.x        Rotate to Elevation
   * ELxx.x       Rotate to Elevation
   * ELxxx.x      Rotate to Elevation
   *
   *
   * Commands are executed upon space, carriage return, or line feed
   * 
   * Reference: https://www.qsl.net/dh1ngp/onlinehelpft100/Rotator_control_with_Easycomm.htm
   * 
   */



  char tempstring[11] = "";
  float heading = -1;
  strcpy(return_string,"");

  switch (easycom_command_buffer[0]) { // look at the first character of the command
    #if defined(OPTION_HAMLIB_EASYCOM_AZ_EL_COMMAND_HACK) && defined(FEATURE_ELEVATION_CONTROL)  
    case 'Z':
      //strcpy(return_string,"+");
      strcpy(return_string,"AZ");
      dtostrf(azimuth,0,1,tempstring);
      strcat(return_string,tempstring);
      //if (elevation >= 0){
        //strcat(return_string,"+");
        strcat(return_string," EL");
      //}
      dtostrf(elevation,0,1,tempstring);      
      strcat(return_string,tempstring);
      break;
    #endif //OPTION_HAMLIB_EASYCOM_AZ_EL_COMMAND_HACK
    case 'A':  // AZ
      if (easycom_command_buffer[1] == 'Z') {  // format is AZx.x or AZxx.x or AZxxx.x (why didn't they make it fixed length?)
        switch (easycom_command_buffer_index) {
          case 2:
            //strcpy(return_string,"AZ");
            strcpy(return_string,"+");
            dtostrf(azimuth,0,1,tempstring);
            strcat(return_string,tempstring);
            return;
            break;
          case 5: // format AZx.x
            heading = float(easycom_command_buffer[2] - 48) + (float(easycom_command_buffer[4] - 48) / 10.);
            break;
          case 6: // format AZxx.x
            heading = (float(easycom_command_buffer[2] - 48) * 10.) + float(easycom_command_buffer[3] - 48) + (float(easycom_command_buffer[5] - 48) / 10.);
            break;
          case 7: // format AZxxx.x
            heading = (float(easycom_command_buffer[2] - 48) * 100.) + (float(easycom_command_buffer[3] - 48) * 10.) + float(easycom_command_buffer[4] - 48.) + (float(easycom_command_buffer[6] - 48) / 10.);
            break;
            // default: control_port->println("?"); break;
        }
        if (((heading >= 0) && (heading < 451))  && (easycom_command_buffer[easycom_command_buffer_index - 2] == '.')) {
          submit_request(AZ, REQUEST_AZIMUTH, heading, 36);
        } else {
          strcpy(return_string,"?");
        }
      } else {
        strcpy(return_string,"?");
      }
      break;
      #ifdef FEATURE_ELEVATION_CONTROL
    case 'E':  // EL
      if (easycom_command_buffer[1] == 'L') {
        switch (easycom_command_buffer_index) {
          case 2:
            //strcpy(return_string,"EL");
            if (elevation >= 0){
              strcpy(return_string,"+");
            }
            dtostrf(elevation,0,1,tempstring);
            strcat(return_string,tempstring);            
            return;
            break;
          case 5: // format ELx.x
            heading = float(easycom_command_buffer[2] - 48) + (float(easycom_command_buffer[4] - 48) / 10.);
            break;
          case 6: // format ELxx.x
            heading = (float(easycom_command_buffer[2] - 48) * 10.) + float(easycom_command_buffer[3] - 48) + (float(easycom_command_buffer[5] - 48) / 10.);
            break;
          case 7: // format ELxxx.x
            heading = (float(easycom_command_buffer[2] - 48) * 100.) + (float(easycom_command_buffer[3] - 48) * 10.) + float(easycom_command_buffer[4] - 48) + (float(easycom_command_buffer[6] - 48) / 10.);
            break;
            // default: control_port->println("?"); break;
        }
        if (((heading >= 0) && (heading < 181)) && (easycom_command_buffer[easycom_command_buffer_index - 2] == '.')) {
          submit_request(EL, REQUEST_ELEVATION, (heading), 37);
        } else {
          strcpy(return_string,"?");
        }
      } else {
        strcpy(return_string,"?");
      }
      break;
      #endif // #FEATURE_ELEVATION_CONTROL
    case 'S':  // SA or SE - stop azimuth, stop elevation
      switch (easycom_command_buffer[1]) {
        case 'A':
          submit_request(AZ, REQUEST_STOP, 0, 38);
          break;
        #ifdef FEATURE_ELEVATION_CONTROL
        case 'E':
          submit_request(EL, REQUEST_STOP, 0, 39);
          break;
        #endif // FEATURE_ELEVATION_CONTROL
        default: strcpy(return_string,"?"); break;
      }
      break;
    case 'M':  // ML, MR, MU, MD - move left, right, up, down
      switch (easycom_command_buffer[1]) {
        case 'L': // ML - move left
          submit_request(AZ, REQUEST_CCW, 0, 40);
          break;
        case 'R': // MR - move right
          submit_request(AZ, REQUEST_CW, 0, 41);
          break;
        #ifdef FEATURE_ELEVATION_CONTROL
        case 'U': // MU - move up
          submit_request(EL, REQUEST_UP, 0, 42);
          break;
        case 'D': // MD - move down
          submit_request(EL, REQUEST_DOWN, 0, 43);
          break;
        #endif // FEATURE_ELEVATION_CONTROL
        default: strcpy(return_string,"?"); break;
      }
      break;
    case 'V': // VE - version query
      if (easycom_command_buffer[1] == 'E') {
        strcpy(return_string,"VE002");
      }                                                                       // not sure what to send back, sending 002 because this is easycom version 2?
      break;
    default: strcpy(return_string,"?"); break;
  } /* switch */



} /* easycom_serial_commmand */
#endif // FEATURE_EASYCOM_EMULATION




//-----------------------------------------------------------------------

#ifdef FEATURE_DCU_1_EMULATION
void process_dcu_1_command(byte * dcu_1_command_buffer, int dcu_1_command_buffer_index, byte source_port, byte command_termination, char * return_string){



  /* DCU-1 protocol implementation


    AP1###;  = set azimuth target,  ### = 0 to 359
    AP1###\r = rotate to azimuth target,  ### = 0 to 359

    AM1; or AM1\r = execute rotation

    ; (alone) = stop rotation

    AI1; or AI1\r = report azimuth

   */


  strcpy(return_string,"?");
  static int dcu_1_azimuth_target_set = -1;
  int temp_heading = 0;
  char tempstring[5];

  // ; command - stop rotation
  if (dcu_1_command_buffer[0] == ';'){
    strcpy(return_string,"OK");
    submit_request(AZ, REQUEST_STOP, 0, DBG_PROCESS_DCU_1);
    return;  
  }

  // AP1 command - set rotation target (if terminated with semicolon), or initiate immediate rotation (if terminated with carriage return)
  if (dcu_1_command_buffer[0] == 'A'){
    if ((dcu_1_command_buffer[1] == 'P') && (dcu_1_command_buffer[2] == '1') && (dcu_1_command_buffer_index == 6)){
      temp_heading = ((dcu_1_command_buffer[3] - 48) * 100) + ((dcu_1_command_buffer[4] - 48) * 10) + (dcu_1_command_buffer[5] - 48);
      if ((temp_heading > -1) && (temp_heading < 360)){
        strcpy(return_string,"OK");
        if (command_termination == DCU_1_SEMICOLON){
          dcu_1_azimuth_target_set = temp_heading;
          return;
        } else {
          submit_request(AZ, REQUEST_AZIMUTH, temp_heading, DBG_PROCESS_DCU_1);
          return;
        }
      }
    }

    // AM1 command - initiate rotation
    if ((dcu_1_command_buffer[1] == 'M') && (dcu_1_command_buffer[2] == '1')  && (dcu_1_command_buffer_index == 3) && (dcu_1_azimuth_target_set > -1)){
      submit_request(AZ, REQUEST_AZIMUTH, dcu_1_azimuth_target_set, DBG_PROCESS_DCU_1);
      dcu_1_azimuth_target_set = -1;
      strcpy(return_string,"OK");
      return;
    }    

    // AI1 command - report azimuth
    if ((dcu_1_command_buffer[1] == 'I') && (dcu_1_command_buffer[2] == '1')){
      submit_request(AZ, REQUEST_AZIMUTH, dcu_1_azimuth_target_set, DBG_PROCESS_DCU_1);
      strcpy(return_string,";");
      dtostrf(int(azimuth),0,0,tempstring);
      if (int(azimuth) < 10) {
        strcat(return_string,"0");
      }
      if (int(azimuth) < 100) {
        strcat(return_string,"0");
      }
      strcat(return_string,tempstring);      
      return;
    }    

  }


} 
#endif // FEATURE_DCU_1_EMULATION



// --------------------------------------------------------------
#ifdef FEATURE_REMOTE_UNIT_SLAVE
void process_remote_slave_command(byte * slave_command_buffer, int slave_command_buffer_index, byte source_port, char * return_string){


/*
 *
 * This implements a protocol for host unit to remote unit communications
 *
 *
 * Remote Slave Unit Protocol Reference
 *
 *  PG - ping
 *  AZ - read azimuth  (returns AZxxx.xxxxxx)
 *  EL - read elevation (returns ELxxx.xxxxxx)
 *  RC - read coordinates (returns RC+xx.xxxx -xxx.xxxx)
 *  GS - query GPS status (returns GS0 (no sync) or GS1 (sync))
 *  DOxx - digital pin initialize as output;
 *  DIxx - digital pin initialize as input
 *  DPxx - digital pin initialize as input with pullup
 *  DRxx - digital pin read
 *  DLxx - digital pin write low
 *  DHxx - digital pin write high
 *  DTxxyyyy - digital pin tone output
 *  NTxx - no tone
 *  ARxx - analog pin read
 *  AWxxyyy - analog pin write
 *  SWxy - serial write byte
 *  SDx - deactivate serial read event; x = port #
 *  SSxyyyyyy... - serial write string; x = port #, yyyy = string of characters to send
 *  SAx - activate serial read event; x = port #
 *  RB - reboot
 *  CL - return clock date and time
 *  
 *  New commands 2021-12-19
 *  RL - rotate left
 *  RR - rotate right
 *  RU - rotate up
 *  RD - rotate down
 *  RA - stop azimuth rotation
 *  RE - stop elevation rotation
 *  RS - stop all rotation
 *
 * Responses
 *
 *  ER - report an error (remote to host only)
 *  EV - report an event (remote to host only)
 *  OK - report success (remote to host only)
 *  CS - report a cold start (remote to host only)
 *
 * Error Codes
 *
 *  ER01 - Serial port buffer timeout
 *  ER02 - Command syntax error
 *
 * Events
 *
 *  EVSxy - Serial port read event; x = serial port number, y = byte returned
 *
 *
 */



  byte command_good = 0;
  strcpy(return_string,"");
  char tempstring[25] = "";

  if (slave_command_buffer_index < 2) {
    strcpy(return_string,"ER02");  // we don't have enough characters - syntax error
  } else {

    #ifdef DEBUG_PROCESS_SLAVE
    debug.print("serial_serial_buffer: command_string: ");
    debug.print((char*)slave_command_buffer);
    debug.print("$ slave_command_buffer_index: ");
    debug.print(slave_command_buffer_index);
    debug.print("\n");
    #endif // DEBUG_PROCESS_SLAVE

    if (((slave_command_buffer[0] == 'S') && (slave_command_buffer[1] == 'S')) && (slave_command_buffer[2] > 47) && (slave_command_buffer[2] < 53)) { // this is a variable length command
      command_good = 1;
      for (byte x = 3; x < slave_command_buffer_index; x++) {
        switch (slave_command_buffer[2] - 48) {
          case 0: control_port->write(slave_command_buffer[x]); break;
          #if defined(FEATURE_MASTER_WITH_SERIAL_SLAVE)
          case 1: REMOTE_PORT.write(slave_command_buffer[x]); break;
          #endif
        }
      }
    }

    if (slave_command_buffer_index == 2) {

      #ifdef FEATURE_CLOCK
      if ((slave_command_buffer[0] == 'C') && (slave_command_buffer[1] == 'L')) {
        strcpy(return_string,"CL");
        #if defined(OPTION_USE_OLD_TIME_CODE)
          update_time();
        #endif
        strcat(return_string,timezone_modified_clock_string());
        command_good = 1;
      }
      #endif //FEATURE_CLOCK

//zzzzzz

      if ((slave_command_buffer[0] == 'R') && (slave_command_buffer[1] == 'L')) {                    // RL - rotate left
        strcpy(return_string,"RL");
        submit_request(AZ, REQUEST_CCW, 0, DBG_PROCESS_REMOTE_SLAVE_COMMAND);
        command_good = 1;
      }
      if ((slave_command_buffer[0] == 'R') && (slave_command_buffer[1] == 'R')) {                    // RR - rotate left
        strcpy(return_string,"RR");
        submit_request(AZ, REQUEST_CW, 0, DBG_PROCESS_REMOTE_SLAVE_COMMAND);
        command_good = 1;
      }
      if ((slave_command_buffer[0] == 'R') && (slave_command_buffer[1] == 'U')) {                    // RU - rotate up
        strcpy(return_string,"RU");
        submit_request(EL, REQUEST_UP, 0, DBG_PROCESS_REMOTE_SLAVE_COMMAND);
        command_good = 1;
      }
      if ((slave_command_buffer[0] == 'R') && (slave_command_buffer[1] == 'D')) {                    // RD - rotate down
        strcpy(return_string,"RD");
        submit_request(EL, REQUEST_DOWN, 0, DBG_PROCESS_REMOTE_SLAVE_COMMAND);
        command_good = 1;
      }
      if ((slave_command_buffer[0] == 'R') && (slave_command_buffer[1] == 'A')) {                    // RA - stop azimuth rotation
        strcpy(return_string,"RA");
        submit_request(AZ, REQUEST_STOP, 0, DBG_PROCESS_REMOTE_SLAVE_COMMAND);
        command_good = 1;
      }
      if ((slave_command_buffer[0] == 'R') && (slave_command_buffer[1] == 'E')) {                    // RE - stop elevation rotation
        strcpy(return_string,"RE");
        submit_request(EL, REQUEST_STOP, 0, DBG_PROCESS_REMOTE_SLAVE_COMMAND);
        command_good = 1;
      }
      if ((slave_command_buffer[0] == 'R') && (slave_command_buffer[1] == 'S')) {                    // RS - stop all rotation
        strcpy(return_string,"RS");
        submit_request(AZ, REQUEST_STOP, 0, DBG_PROCESS_REMOTE_SLAVE_COMMAND);
        submit_request(EL, REQUEST_STOP, 0, DBG_PROCESS_REMOTE_SLAVE_COMMAND);
        command_good = 1;
      }      

      #ifdef FEATURE_GPS
      if ((slave_command_buffer[0] == 'R') && (slave_command_buffer[1] == 'C')) {                    // RC - read coordinates
        strcpy(return_string,"RC");
        if (latitude < 0){strcat(return_string,"-");} else {strcat(return_string,"+");}
        dtostrf(abs(latitude),0,4,tempstring);
        strcat(return_string,tempstring);         
        strcat(return_string," ");
        if (longitude < 0){strcat(return_string,"-");} else {strcat(return_string,"+");}
        if (longitude < 100){strcat(return_string,"0");}
        dtostrf(abs(longitude),0,4,tempstring);
        strcat(return_string,tempstring);        
        command_good = 1;
      }   
      #ifdef FEATURE_CLOCK
      if ((slave_command_buffer[0] == 'G') && (slave_command_buffer[1] == 'S')) {                    // GS - query GPS sync
        strcpy(return_string,"GS");
        if (clock_status == GPS_SYNC){                
          strcat(return_string,"1");
        } else {
          strcat(return_string,"0");
        }        
        command_good = 1;
      }
      #endif //FEATURE_CLOCK                 
      #endif //FEATURE_GPS      

      // PM - ping from master, respond back with a ping
      if ((slave_command_buffer[0] == 'P') && (slave_command_buffer[1] == 'M')) {
        strcpy(return_string,"PM");
        command_good = 1;
      }   
      // PM - ping sent from remote pinged back by master
      if ((slave_command_buffer[0] == 'P') && (slave_command_buffer[1] == 'R')) {
        command_good = 1;
        last_ping_receive_time = millis();
      }    

      if ((slave_command_buffer[0] == 'R') && (slave_command_buffer[1] == 'B')) {
        reset_the_unit = 1;
        // wdt_enable(WDTO_30MS); while (1) {
        // }
      }                                                                        // RB - reboot
      if ((slave_command_buffer[0] == 'A') && (slave_command_buffer[1] == 'Z')) {
        strcpy(return_string,"AZ");
        //if (raw_azimuth < 1000) {
        //  strcat(return_string,"0");
        //}
        if (raw_azimuth < 100) {
          strcat(return_string,"0");
        }
        if (raw_azimuth < 10) {
          strcat(return_string,"0");
        }
        dtostrf(raw_azimuth,0,6,tempstring);
        strcat(return_string,tempstring);          
        command_good = 1;
      }
      #ifdef FEATURE_ELEVATION_CONTROL
      if ((slave_command_buffer[0] == 'E') && (slave_command_buffer[1] == 'L')) {
        strcpy(return_string,"EL");
        if (elevation >= 0) {
          strcat(return_string,"+");
        } else {
          strcat(return_string,"-");
        }
        //if (abs(elevation) < 1000) {
        //  strcat(return_string,"0");
        //}
        if (abs(elevation) < 100) {
          strcat(return_string,"0");
        }
        if (abs(elevation) < 10) {
          strcat(return_string,"0");
        }
        dtostrf(abs(elevation),0,6,tempstring);
        strcat(return_string,tempstring);            
        command_good = 1;
      }
        #endif // FEATURE_ELEVATION_CONTROL
    } // end of two byte commands



    if (slave_command_buffer_index == 3) {
      if (((slave_command_buffer[0] == 'S') && (slave_command_buffer[1] == 'A')) & (slave_command_buffer[2] > 47) && (slave_command_buffer[2] < 53)) {
        serial_read_event_flag[slave_command_buffer[2] - 48] = 1;
        command_good = 1;
        strcpy(return_string,"OK");
      }
      if (((slave_command_buffer[0] == 'S') && (slave_command_buffer[1] == 'D')) & (slave_command_buffer[2] > 47) && (slave_command_buffer[2] < 53)) {
        serial_read_event_flag[slave_command_buffer[2] - 48] = 0;
        command_good = 1;
        strcpy(return_string,"OK");
      }

    }


    if (slave_command_buffer_index == 4) {
      if ((slave_command_buffer[0] == 'S') && (slave_command_buffer[1] == 'W')) { // Serial Write command
        switch (slave_command_buffer[2]) {
          case '0': control_port->write(slave_command_buffer[3]); command_good = 1; break;
          #if defined(FEATURE_MASTER_WITH_SERIAL_SLAVE)
          case '1': REMOTE_PORT.write(slave_command_buffer[3]); command_good = 1; break;
          #endif
        }
      }

      if ((slave_command_buffer[0] == 'D') && (slave_command_buffer[1] == 'O')) {
        if ((((slave_command_buffer[2] > 47) && (slave_command_buffer[2] < 58)) || (toupper(slave_command_buffer[2]) == 'A')) && (slave_command_buffer[3] > 47) && (slave_command_buffer[3] < 58)) {
          command_good = 1;
          byte pin_value = 0;
          if (toupper(slave_command_buffer[2]) == 'A') {
            pin_value = get_analog_pin(slave_command_buffer[3] - 48);
          } else {
            pin_value = ((slave_command_buffer[2] - 48) * 10) + (slave_command_buffer[3] - 48);
          }
          #ifdef DEBUG_PROCESS_SLAVE
          debug.print("service_serial_buffer: pin_value: ");
          debug.print(pin_value);
          #endif // DEBUG_PROCESS_SLAVE
          strcpy(return_string,"OK");
          pinModeEnhanced(pin_value, OUTPUT);
        }
      }

      if ((slave_command_buffer[0] == 'D') && (slave_command_buffer[1] == 'H')) {
        if ((((slave_command_buffer[2] > 47) && (slave_command_buffer[2] < 58)) || (toupper(slave_command_buffer[2]) == 'A')) && (slave_command_buffer[3] > 47) && (slave_command_buffer[3] < 58)) {
          command_good = 1;
          byte pin_value = 0;
          if (toupper(slave_command_buffer[2]) == 'A') {
            pin_value = get_analog_pin(slave_command_buffer[3] - 48);
          } else {
            pin_value = ((slave_command_buffer[2] - 48) * 10) + (slave_command_buffer[3] - 48);
          }
          digitalWriteEnhanced(pin_value, HIGH);
          strcpy(return_string,"OK");
        }
      }

      if ((slave_command_buffer[0] == 'D') && (slave_command_buffer[1] == 'L')) {
        if ((((slave_command_buffer[2] > 47) && (slave_command_buffer[2] < 58)) || (toupper(slave_command_buffer[2]) == 'A')) && (slave_command_buffer[3] > 47) && (slave_command_buffer[3] < 58)) {
          command_good = 1;
          byte pin_value = 0;
          if (toupper(slave_command_buffer[2]) == 'A') {
            pin_value = get_analog_pin(slave_command_buffer[3] - 48);
          } else {
            pin_value = ((slave_command_buffer[2] - 48) * 10) + (slave_command_buffer[3] - 48);
          }
          digitalWriteEnhanced(pin_value, LOW);
          strcpy(return_string,"OK");
        }
      }

      if ((slave_command_buffer[0] == 'D') && (slave_command_buffer[1] == 'I')) {
        if ((((slave_command_buffer[2] > 47) && (slave_command_buffer[2] < 58)) || (toupper(slave_command_buffer[2]) == 'A')) && (slave_command_buffer[3] > 47) && (slave_command_buffer[3] < 58)) {
          command_good = 1;
          byte pin_value = 0;
          if (toupper(slave_command_buffer[2]) == 'A') {
            pin_value = get_analog_pin(slave_command_buffer[3] - 48);
          } else {
            pin_value = ((slave_command_buffer[2] - 48) * 10) + (slave_command_buffer[3] - 48);
          }
          pinModeEnhanced(pin_value, INPUT);
          strcpy(return_string,"OK");
        }
      }

      if ((slave_command_buffer[0] == 'D') && (slave_command_buffer[1] == 'P')) {
        if ((((slave_command_buffer[2] > 47) && (slave_command_buffer[2] < 58)) || (toupper(slave_command_buffer[2]) == 'A')) && (slave_command_buffer[3] > 47) && (slave_command_buffer[3] < 58)) {
          command_good = 1;
          byte pin_value = 0;
          if (toupper(slave_command_buffer[2]) == 'A') {
            pin_value = get_analog_pin(slave_command_buffer[3] - 48);
          } else {
            pin_value = ((slave_command_buffer[2] - 48) * 10) + (slave_command_buffer[3] - 48);
          }
          pinModeEnhanced(pin_value, INPUT);
          digitalWriteEnhanced(pin_value, HIGH);
          strcpy(return_string,"OK");
        }
      }

      if ((slave_command_buffer[0] == 'D') && (slave_command_buffer[1] == 'R')) {
        if ((((slave_command_buffer[2] > 47) && (slave_command_buffer[2] < 58)) || (toupper(slave_command_buffer[2]) == 'A')) && (slave_command_buffer[3] > 47) && (slave_command_buffer[3] < 58)) {
          command_good = 1;
          byte pin_value = 0;
          if (toupper(slave_command_buffer[2]) == 'A') {
            pin_value = get_analog_pin(slave_command_buffer[3] - 48);
          } else {
            pin_value = ((slave_command_buffer[2] - 48) * 10) + (slave_command_buffer[3] - 48);
          }
          byte pin_read = digitalReadEnhanced(pin_value);
          strcpy(return_string,"DR");
          dtostrf((slave_command_buffer[2]-48),0,0,tempstring);
          strcat(return_string,tempstring);              
          dtostrf((slave_command_buffer[3]-48),0,0,tempstring);
          strcat(return_string,tempstring);  
          if (pin_read) {
            strcat(return_string,"1");
          } else {
            strcat(return_string,"0");
          }
        }
      }
      if ((slave_command_buffer[0] == 'A') && (slave_command_buffer[1] == 'R')) {
        if ((((slave_command_buffer[2] > 47) && (slave_command_buffer[2] < 58)) || (toupper(slave_command_buffer[2]) == 'A')) && (slave_command_buffer[3] > 47) && (slave_command_buffer[3] < 58)) {
          command_good = 1;
          byte pin_value = 0;
          if (toupper(slave_command_buffer[2]) == 'A') {
            pin_value = get_analog_pin(slave_command_buffer[3] - 48);
          } else {
            pin_value = ((slave_command_buffer[2] - 48) * 10) + (slave_command_buffer[3] - 48);
          }
          int pin_read = analogReadEnhanced(pin_value);
          strcpy(return_string,"AR");
          if (toupper(slave_command_buffer[2]) == 'A') {
            strcat(return_string,"A");
          } else {
            dtostrf((slave_command_buffer[2]-48),0,0,tempstring);
            strcat(return_string,tempstring);
          }
                        
          dtostrf((slave_command_buffer[3]-48),0,0,tempstring);
          strcat(return_string,tempstring);  
          if (pin_read < 1000) {
            strcat(return_string,"0");
          }
          if (pin_read < 100) {
            strcat(return_string,"0");
          }
          if (pin_read < 10) {
            strcat(return_string,"0");
          }
          dtostrf(pin_read,0,0,tempstring);
          strcat(return_string,tempstring);             
        }
      }

      if ((slave_command_buffer[0] == 'N') && (slave_command_buffer[1] == 'T')) {
        if ((((slave_command_buffer[2] > 47) && (slave_command_buffer[2] < 58)) || (toupper(slave_command_buffer[2]) == 'A')) && (slave_command_buffer[3] > 47) && (slave_command_buffer[3] < 58)) {
          command_good = 1;
          byte pin_value = 0;
          if (toupper(slave_command_buffer[2]) == 'A') {
            pin_value = get_analog_pin(slave_command_buffer[3] - 48);
          } else {
            pin_value = ((slave_command_buffer[2] - 48) * 10) + (slave_command_buffer[3] - 48);
          }
          noTone(pin_value);
          strcpy(return_string,"OK");
        }
      }

    } // if (slave_command_buffer_index == 4)

    if (slave_command_buffer_index == 7) {
      if ((slave_command_buffer[0] == 'A') && (slave_command_buffer[1] == 'W')) {
        if ((((slave_command_buffer[2] > 47) && (slave_command_buffer[2] < 58)) || (toupper(slave_command_buffer[2]) == 'A')) && (slave_command_buffer[3] > 47) && (slave_command_buffer[3] < 58)) {
          byte pin_value = 0;
          if (toupper(slave_command_buffer[2]) == 'A') {
            pin_value = get_analog_pin(slave_command_buffer[3] - 48);
          } else {
            pin_value = ((slave_command_buffer[2] - 48) * 10) + (slave_command_buffer[3] - 48);
          }
          int write_value = ((slave_command_buffer[4] - 48) * 100) + ((slave_command_buffer[5] - 48) * 10) + (slave_command_buffer[6] - 48);
          if ((write_value >= 0) && (write_value < 256)) {
            analogWriteEnhanced(pin_value, write_value);
            strcpy(return_string,"OK");
            command_good = 1;
          }
        }
      }
    }

    if (slave_command_buffer_index == 8) {
      if ((slave_command_buffer[0] == 'D') && (slave_command_buffer[1] == 'T')) {
        if ((((slave_command_buffer[2] > 47) && (slave_command_buffer[2] < 58)) || (toupper(slave_command_buffer[2]) == 'A')) && (slave_command_buffer[3] > 47) && (slave_command_buffer[3] < 58)) {
          byte pin_value = 0;
          if (toupper(slave_command_buffer[2]) == 'A') {
            pin_value = get_analog_pin(slave_command_buffer[3] - 48);
          } else {
            pin_value = ((slave_command_buffer[2] - 48) * 10) + (slave_command_buffer[3] - 48);
          }
          int write_value = ((slave_command_buffer[4] - 48) * 1000) + ((slave_command_buffer[5] - 48) * 100) + ((slave_command_buffer[6] - 48) * 10) + (slave_command_buffer[7] - 48);
          if ((write_value >= 0) && (write_value <= 9999)) {
            tone(pin_value, write_value);
            strcpy(return_string,"OK");
            command_good = 1;
          }
        }
      }
    }


    if (!command_good) {
      strcpy(return_string,"ER0289");
    }
  }

  slave_command_buffer_index = 0;

}
#endif //FEATURE_REMOTE_UNIT_SLAVE

// --------------------------------------------------------------


#ifdef FEATURE_YAESU_EMULATION
void process_yaesu_command(byte * yaesu_command_buffer, int yaesu_command_buffer_index, byte source_port, char * return_string){



    char tempstring[11] = "";
    int parsed_value = 0;
  
    int parsed_elevation = 0;
  

    #ifdef FEATURE_TIMED_BUFFER
      int parsed_value2 = 0;
    #endif //FEATURE_TIMED_BUFFER

    strcpy(return_string,"");

    switch (yaesu_command_buffer[0]) {          // look at the first character of the command
      case 'C':                                // C - return current azimuth
        #ifdef DEBUG_PROCESS_YAESU
          if (debug_mode) {
            debug.print("yaesu_serial_command: C\n");
          }
        #endif // DEBUG_PROCESS_YAESU
        #ifdef OPTION_DELAY_C_CMD_OUTPUT
        delay(400);
        #endif    
        //strcpy(return_string,"");
        #ifndef OPTION_GS_232B_EMULATION
        strcat(return_string,"+0");
        #else
        strcat(return_string,"AZ=");
        #endif
        dtostrf(int(azimuth),0,0,tempstring);
        if (int(azimuth) < 10) {
          strcat(return_string,"0");
        }
        if (int(azimuth) < 100) {
          strcat(return_string,"0");
        }
        strcat(return_string,tempstring);
      
        #ifdef FEATURE_ELEVATION_CONTROL
        #ifndef OPTION_C_COMMAND_SENDS_AZ_AND_EL
        if ((yaesu_command_buffer[1] == '2') && (yaesu_command_buffer_index > 1)) {     // did we get the C2 command?
        #endif


        #ifndef OPTION_GS_232B_EMULATION
          if (elevation < 0) {
            strcat(return_string,"-0");
          } else {
            strcat(return_string,"+0");
          }
        #endif

        #ifdef OPTION_GS_232B_EMULATION
          strcat(return_string,"EL=");
        #endif

        dtostrf(int(elevation),0,0,tempstring);
        if (int(elevation) < 10) {
          strcat(return_string,("0"));
        }
        if (int(elevation) < 100) {
          strcat(return_string,"0");
        }
        strcat(return_string,tempstring);

        #ifndef OPTION_C_COMMAND_SENDS_AZ_AND_EL
        } else {
          //strcat(return_string,"\n");
        }
        #endif // OPTION_C_COMMAND_SENDS_AZ_AND_EL
        #endif // FEATURE_ELEVATION_CONTROL
      
        #ifndef FEATURE_ELEVATION_CONTROL
          if ((yaesu_command_buffer[1] == '2') && (yaesu_command_buffer_index > 1)) {     // did we get the C2 command?
            #ifndef OPTION_GS_232B_EMULATION
            strcat(return_string,"+0000");    // return a dummy elevation since we don't have the elevation feature turned on
            #else
            strcat(return_string,"EL=000");
            #endif
          } else {
            //strcat(return_string,"\n");
          }
        #endif // FEATURE_ELEVATION_CONTROL   
        break;
        
        
        //-----------------end of C command-----------------
        
      #ifdef FEATURE_AZ_POSITION_POTENTIOMETER
        case 'F': // F - full scale calibration
          #ifdef DEBUG_PROCESS_YAESU
            if (debug_mode) {
              debug.print("yaesu_serial_command: F\n");
            }
          #endif // DEBUG_PROCESS_YAESU
        
          #ifdef FEATURE_ELEVATION_CONTROL
            if ((yaesu_command_buffer[1] == '2') && (yaesu_command_buffer_index > 1)) {     // did we get the F2 command?

              clear_serial_buffer();

              #if defined(OPTION_ALLOW_ROTATIONAL_AND_CONFIGURATION_CMDS_AT_BOOT_UP)
                if (source_port == CONTROL_PORT0){
                  control_port->println(F("Elevate to 180 (or max elevation) and send keystroke..."));
                }
                get_keystroke();
                read_elevation(1);
                configuration.analog_el_max_elevation = analog_el;
                write_settings_to_eeprom();
                strcpy(return_string,"Wrote to memory");
              #else
                if (millis() > ROTATIONAL_AND_CONFIGURATION_CMD_IGNORE_TIME_MS){ 
                  if (source_port == CONTROL_PORT0){
                    control_port->println(F("Elevate to 180 (or max elevation) and send keystroke..."));
                  }
                  get_keystroke();
                  read_elevation(1);
                  configuration.analog_el_max_elevation = analog_el;
                  write_settings_to_eeprom();
                  strcpy(return_string,"Wrote to memory");
                }
              #endif  // OPTION_ALLOW_ROTATIONAL_AND_CONFIGURATION_CMDS_AT_BOOT_UP

              return;
            }
          #endif  //FEATURE_ELEVATION_CONTROL
        
          clear_serial_buffer();

          #if defined(OPTION_ALLOW_ROTATIONAL_AND_CONFIGURATION_CMDS_AT_BOOT_UP)
            if (source_port == CONTROL_PORT0){
              control_port->println(F("Rotate to full CW and send keystroke..."));
              get_keystroke();
            }
            read_azimuth(1);
            configuration.analog_az_full_cw = analog_az;
            write_settings_to_eeprom();
            strcpy(return_string,"Wrote to memory");  
          #else
            if (millis() > ROTATIONAL_AND_CONFIGURATION_CMD_IGNORE_TIME_MS){
              if (source_port == CONTROL_PORT0){
                control_port->println(F("Rotate to full CW and send keystroke..."));
                get_keystroke();
              }
              read_azimuth(1);
              configuration.analog_az_full_cw = analog_az;
              write_settings_to_eeprom();
              strcpy(return_string,"Wrote to memory");  
            }
          #endif  // OPTION_ALLOW_ROTATIONAL_AND_CONFIGURATION_CMDS_AT_BOOT_UP

          break;
        #endif // FEATURE_AZ_POSITION_POTENTIOMETER

      case 'H': print_help(source_port); break;                     // H - print help - depricated

      case 'L':  // L - manual left (CCW) rotation
        #ifdef DEBUG_PROCESS_YAESU
          if (debug_mode) {
            debug.print("yaesu_serial_command: L\n");
          }
        #endif // DEBUG_PROCESS_YAESU


        #if defined(OPTION_ALLOW_ROTATIONAL_AND_CONFIGURATION_CMDS_AT_BOOT_UP)
          submit_request(AZ, REQUEST_CCW, 0, 21);
          //strcpy(return_string,"\n");
          #ifdef FEATURE_PARK
            deactivate_park();
          #endif // FEATURE_PARK
        #else
          if (millis() > ROTATIONAL_AND_CONFIGURATION_CMD_IGNORE_TIME_MS){
            submit_request(AZ, REQUEST_CCW, 0, 21);
            //strcpy(return_string,"\n");
            #ifdef FEATURE_PARK
            deactivate_park();
            #endif // FEATURE_PARK
          }  
        #endif //OPTION_ALLOW_ROTATIONAL_AND_CONFIGURATION_CMDS_AT_BOOT_UP

        break;
        
      #ifdef FEATURE_AZ_POSITION_POTENTIOMETER
        case 'O':  // O - offset calibration
          #ifdef DEBUG_PROCESS_YAESU
            if (debug_mode) {
              debug.print("yaesu_serial_command: O\n");
            }
          #endif // DEBUG_PROCESS_YAESU

          #ifdef FEATURE_ELEVATION_CONTROL
            if ((yaesu_command_buffer[1] == '2') && (yaesu_command_buffer_index > 1)) {     // did we get the O2 command?       
              clear_serial_buffer();

              #if defined(OPTION_ALLOW_ROTATIONAL_AND_CONFIGURATION_CMDS_AT_BOOT_UP)
                if (source_port == CONTROL_PORT0){  
                  control_port->println(F("Elevate to 0 degrees and send keystroke..."));
                }
                get_keystroke();
                read_elevation(1);
                configuration.analog_el_0_degrees = analog_el;
                write_settings_to_eeprom();
                strcpy(return_string,"Wrote to memory");
              #else
                if (millis() > ROTATIONAL_AND_CONFIGURATION_CMD_IGNORE_TIME_MS){
                  if (source_port == CONTROL_PORT0){  
                    control_port->println(F("Elevate to 0 degrees and send keystroke..."));
                  }
                  get_keystroke();
                  read_elevation(1);
                  configuration.analog_el_0_degrees = analog_el;
                  write_settings_to_eeprom();
                  strcpy(return_string,"Wrote to memory"); 
                }             
              #endif //OPTION_ALLOW_ROTATIONAL_AND_CONFIGURATION_CMDS_AT_BOOT_UP

              return;
            }
          #endif // FEATURE_ELEVATION_CONTROL
        
          clear_serial_buffer();  
          
          #if defined(OPTION_ALLOW_ROTATIONAL_AND_CONFIGURATION_CMDS_AT_BOOT_UP)
            if (source_port == CONTROL_PORT0){    
              control_port->println(F("Rotate to full CCW and send keystroke..."));
            }
            get_keystroke();
            read_azimuth(1);
            configuration.analog_az_full_ccw = analog_az;
            write_settings_to_eeprom();
            strcpy(return_string,"Wrote to memory");
          #else
            if (millis() > ROTATIONAL_AND_CONFIGURATION_CMD_IGNORE_TIME_MS){
              if (source_port == CONTROL_PORT0){    
                control_port->println(F("Rotate to full CCW and send keystroke..."));
              }
              get_keystroke();
              read_azimuth(1);
              configuration.analog_az_full_ccw = analog_az;
              write_settings_to_eeprom();
              strcpy_P(return_string,(const char*) F("Wrote to memory"));
            }
          #endif

          break;
        #endif // FEATURE_AZ_POSITION_POTENTIOMETER
      
      case 'R':  // R - manual right (CW) rotation
        #ifdef DEBUG_PROCESS_YAESU
          if (debug_mode) {
            debug.print("yaesu_serial_command: R\n");
          }
        #endif // DEBUG_PROCESS_YAESU

        #if defined(OPTION_ALLOW_ROTATIONAL_AND_CONFIGURATION_CMDS_AT_BOOT_UP)
          submit_request(AZ, REQUEST_CW, 0, 22);
          strcpy(return_string,"\n");
          #ifdef FEATURE_PARK
            deactivate_park();
          #endif // FEATURE_PARK
        #else
          if (millis() > ROTATIONAL_AND_CONFIGURATION_CMD_IGNORE_TIME_MS){
            submit_request(AZ, REQUEST_CW, 0, 22);
            strcpy(return_string,"\n");
            #ifdef FEATURE_PARK
              deactivate_park();
            #endif // FEATURE_PARK
          }
        #endif  

        break;
        
      case 'A':  // A - CW/CCW rotation stop
        #ifdef DEBUG_PROCESS_YAESU
          if (debug_mode) {
            debug.print(F("yaesu_serial_command: A\n"));
          }
        #endif // DEBUG_PROCESS_YAESU
        submit_request(AZ, REQUEST_STOP, 0, 23);
        //strcpy(return_string,"\n");
        #ifdef FEATURE_PARK
          deactivate_park();
        #endif // FEATURE_PARK
        break;
        
      case 'S':         // S - all stop
        #ifdef DEBUG_PROCESS_YAESU
          if (debug_mode) {
            debug.print(F("yaesu_serial_command: S\n"));
          }
        #endif // DEBUG_PROCESS_YAESU
        stop_rotation();
        #ifdef FEATURE_TIMED_BUFFER
          clear_timed_buffer();
        #endif // FEATURE_TIMED_BUFFER
        //strcpy(return_string,"");
        #ifdef FEATURE_PARK
          deactivate_park();
        #endif // FEATURE_PARK
        break;
        
      case 'M': // M - auto azimuth rotation
        #ifdef DEBUG_PROCESS_YAESU
        if (debug_mode) {
          debug.print(F("yaesu_serial_command: M\n"));
        }
        #endif // DEBUG_PROCESS_YAESU
        #ifdef FEATURE_PARK
        deactivate_park();
        #endif // FEATURE_PARK
  
        if (yaesu_command_buffer_index > 4) {  // if there are more than 4 characters in the command buffer, we got a timed interval command
          #ifdef FEATURE_TIMED_BUFFER
            clear_timed_buffer();
            parsed_value = ((int(yaesu_command_buffer[1]) - 48) * 100) + ((int(yaesu_command_buffer[2]) - 48) * 10) + (int(yaesu_command_buffer[3]) - 48);
            if ((parsed_value > 0) && (parsed_value < 1000)) {
              timed_buffer_interval_value_seconds = parsed_value;
              for (int x = 5; x < yaesu_command_buffer_index; x = x + 4) {
                parsed_value = ((int(yaesu_command_buffer[x]) - 48) * 100) + ((int(yaesu_command_buffer[x + 1]) - 48) * 10) + (int(yaesu_command_buffer[x + 2]) - 48);
                if ((parsed_value >= 0) && (parsed_value <= (configuration.azimuth_starting_point + configuration.azimuth_rotation_capability))) {  // is it a valid azimuth?
                  timed_buffer_azimuths[timed_buffer_number_entries_loaded] = parsed_value;
                  timed_buffer_number_entries_loaded++;
                  timed_buffer_status = LOADED_AZIMUTHS;
                  if (timed_buffer_number_entries_loaded > TIMED_INTERVAL_ARRAY_SIZE) {   // is the array full?
                    submit_request(AZ, REQUEST_AZIMUTH, timed_buffer_azimuths[0], 26);  // array is full, go to the first azimuth
                    timed_buffer_entry_pointer = 1;
                    return;
                  }
                } else {   // we hit an invalid bearing
                  timed_buffer_status = EMPTY;
                  timed_buffer_number_entries_loaded = 0;
                  strcpy(return_string,"?>");  // error
                  return;
                }
              }
              submit_request(AZ, REQUEST_AZIMUTH, timed_buffer_azimuths[0], 27);   // go to the first azimuth
              timed_buffer_entry_pointer = 1;       
            } else {
              strcpy(return_string,"?>");  // error
            }
          #else
            strcpy(return_string,"?>");
          #endif // FEATURE_TIMED_BUFFER
          return;
        } else {                         // if there are four characters, this is just a single direction setting
          if (yaesu_command_buffer_index == 4) {
            parsed_value = ((int(yaesu_command_buffer[1]) - 48) * 100) + ((int(yaesu_command_buffer[2]) - 48) * 10) + (int(yaesu_command_buffer[3]) - 48);
            #ifdef FEATURE_TIMED_BUFFER
              clear_timed_buffer();
            #endif // FEATURE_TIMED_BUFFER
            if ((parsed_value >= 0) && (parsed_value <= (configuration.azimuth_starting_point + configuration.azimuth_rotation_capability))) {
              #if defined(OPTION_ALLOW_ROTATIONAL_AND_CONFIGURATION_CMDS_AT_BOOT_UP)
                submit_request(AZ, REQUEST_AZIMUTH, (parsed_value), 28);
              #else
                if (millis() > ROTATIONAL_AND_CONFIGURATION_CMD_IGNORE_TIME_MS){
                  submit_request(AZ, REQUEST_AZIMUTH, (parsed_value), 28);
                }
              #endif
              return;
            }
          }
        }
        strcpy(return_string,"?>");      
        break;
        
      #ifdef FEATURE_TIMED_BUFFER
      case 'N': // N - number of loaded timed interval entries
        #ifdef DEBUG_PROCESS_YAESU
          if (debug_mode) {
            debug.print("yaesu_serial_command: N\n");
          }
        #endif // DEBUG_PROCESS_YAESU
        #ifdef FEATURE_PARK
          deactivate_park();
        #endif // FEATURE_PARK
        sprintf(return_string,"%d",timed_buffer_number_entries_loaded);
        break;
        #endif // FEATURE_TIMED_BUFFER
        
      #ifdef FEATURE_TIMED_BUFFER
      case 'T': // T - initiate timed tracking
        #if defined(OPTION_ALLOW_ROTATIONAL_AND_CONFIGURATION_CMDS_AT_BOOT_UP)
          initiate_timed_buffer(source_port);
        #else
          if (millis() > ROTATIONAL_AND_CONFIGURATION_CMD_IGNORE_TIME_MS){
            initiate_timed_buffer(source_port);
          }
        #endif
        #ifdef FEATURE_PARK
          deactivate_park();
        #endif // FEATURE_PARK
        break;           
        #endif // FEATURE_TIMED_BUFFER
        
      case 'X':  // X - azimuth speed change
        #ifdef DEBUG_PROCESS_YAESU
        if (debug_mode) {
          debug.print(F("yaesu_serial_command: X\n"));
        }
        #endif // DEBUG_PROCESS_YAESU
        
        
        if (yaesu_command_buffer_index > 1) {
          switch (yaesu_command_buffer[1]) {
            case '4':
              normal_az_speed_voltage = PWM_SPEED_VOLTAGE_X4;
              update_az_variable_outputs(PWM_SPEED_VOLTAGE_X4);
              #if defined(FEATURE_ELEVATION_CONTROL) && defined(OPTION_EL_SPEED_FOLLOWS_AZ_SPEED)
              normal_el_speed_voltage = PWM_SPEED_VOLTAGE_X4;
              update_el_variable_outputs(PWM_SPEED_VOLTAGE_X4);
              #endif
              strcpy_P(return_string,(const char*) F("Speed X4"));
              break;
            case '3':
              normal_az_speed_voltage = PWM_SPEED_VOLTAGE_X3;
              update_az_variable_outputs(PWM_SPEED_VOLTAGE_X3);
              #if defined(FEATURE_ELEVATION_CONTROL) && defined(OPTION_EL_SPEED_FOLLOWS_AZ_SPEED)
              normal_el_speed_voltage = PWM_SPEED_VOLTAGE_X3;
              update_el_variable_outputs(PWM_SPEED_VOLTAGE_X3);
              #endif
              strcpy_P(return_string,(const char*) F("Speed X3"));
              break;
            case '2':
              normal_az_speed_voltage = PWM_SPEED_VOLTAGE_X2;
              update_az_variable_outputs(PWM_SPEED_VOLTAGE_X2);
              #if defined(FEATURE_ELEVATION_CONTROL) && defined(OPTION_EL_SPEED_FOLLOWS_AZ_SPEED)
              normal_el_speed_voltage = PWM_SPEED_VOLTAGE_X2;
              update_el_variable_outputs(PWM_SPEED_VOLTAGE_X2);
              #endif
              strcpy_P(return_string,(const char*) F("Speed X2"));
              break;
            case '1':
              normal_az_speed_voltage = PWM_SPEED_VOLTAGE_X1;
              update_az_variable_outputs(PWM_SPEED_VOLTAGE_X1);
              #if defined(FEATURE_ELEVATION_CONTROL) && defined(OPTION_EL_SPEED_FOLLOWS_AZ_SPEED)
              normal_el_speed_voltage = PWM_SPEED_VOLTAGE_X1;
              update_el_variable_outputs(PWM_SPEED_VOLTAGE_X1);
              #endif
              strcpy_P(return_string,(const char*) F("Speed X1"));
              break;
            default: strcpy_P(return_string,(const char*) F("?>")); break;
          } /* switch */
        } else {
          strcpy_P(return_string,(const char*) F("?>"));
        }
        break;
        
      #ifdef FEATURE_ELEVATION_CONTROL
        case 'U':  // U - manual up rotation
          #ifdef DEBUG_PROCESS_YAESU
              if (debug_mode) {
                debug.print(F("yaesu_serial_command: U\n"));
              }
          #endif // DEBUG_PROCESS_YAESU
          #ifdef FEATURE_PARK
            deactivate_park();
          #endif // FEATURE_PARK
          #if defined(OPTION_ALLOW_ROTATIONAL_AND_CONFIGURATION_CMDS_AT_BOOT_UP)
            submit_request(EL, REQUEST_UP, 0, 29);
          #else
            if (millis() > ROTATIONAL_AND_CONFIGURATION_CMD_IGNORE_TIME_MS){
              submit_request(EL, REQUEST_UP, 0, 29);
            }
          #endif
          //strcpy(return_string,"\n");
          break;
          
        case 'D':  // D - manual down rotation
          #ifdef DEBUG_PROCESS_YAESU
            if (debug_mode) {
              debug.print("yaesu_serial_command: D\n");
            }
          #endif // DEBUG_PROCESS_YAESU
          #ifdef FEATURE_PARK
            deactivate_park();
          #endif // FEATURE_PARK
          #if defined(OPTION_ALLOW_ROTATIONAL_AND_CONFIGURATION_CMDS_AT_BOOT_UP)
            submit_request(EL, REQUEST_DOWN, 0, 30);
          #else
            if (millis() > ROTATIONAL_AND_CONFIGURATION_CMD_IGNORE_TIME_MS){
              submit_request(EL, REQUEST_DOWN, 0, 30);
            }
          #endif
          //strcpy(return_string,"\n");
          break;
          
        case 'E':  // E - stop elevation rotation
          #ifdef DEBUG_PROCESS_YAESU
            if (debug_mode) {
              debug.print(F("yaesu_serial_command: E\n"));
            }
          #endif // DEBUG_PROCESS_YAESU
          #ifdef FEATURE_PARK
            deactivate_park();
          #endif // FEATURE_PARK
          submit_request(EL, REQUEST_STOP, 0, 31);
          //strcpy(return_string,"\n");
          break;
          
        case 'B': // B - return current elevation
          #ifndef OPTION_GS_232B_EMULATION
            if (elevation < 0) {
              strcat(return_string,"-0");
            } else {
              strcat(return_string,"+0");
            }
          #else
            strcat(return_string,"EL=");
          #endif //OPTION_GS_232B_EMULATION
          dtostrf(int(elevation),0,0,tempstring);
          if (int(elevation) < 10) {
            strcat(return_string,("0"));
          }
          if (int(elevation) < 100) {
            strcat(return_string,"0");
          }
          strcat(return_string,tempstring);
        break;        
      
      #endif /* ifdef FEATURE_ELEVATION_CONTROL */
      
      case 'W':  // W - auto elevation rotation
        #ifdef DEBUG_PROCESS_YAESU
          if (debug_mode) {
            debug.print(F("yaesu_serial_command: W\n"));
          }
        #endif // DEBUG_PROCESS_YAESU

        #ifdef FEATURE_PARK
          deactivate_park();
        #endif // FEATURE_PARK
        
        
        // parse out W command
        // Short Format: WXXX YYY            XXX = azimuth YYY = elevation
        // Long Format : WSSS XXX YYY        SSS = timed interval   XXX = azimuth    YYY = elevation

        if (yaesu_command_buffer_index > 8) {  // if there are more than 4 characters in the command buffer, we got a timed interval command
          #if defined(FEATURE_TIMED_BUFFER) && defined(FEATURE_ELEVATION_CONTROL) 
            parsed_value = ((int(yaesu_command_buffer[1]) - 48) * 100) + ((int(yaesu_command_buffer[2]) - 48) * 10) + (int(yaesu_command_buffer[3]) - 48);
            if ((parsed_value > 0) && (parsed_value < 1000)) {
              timed_buffer_interval_value_seconds = parsed_value;
              for (int x = 5; x < yaesu_command_buffer_index; x = x + 8) {
                parsed_value = ((int(yaesu_command_buffer[x]) - 48) * 100) + ((int(yaesu_command_buffer[x + 1]) - 48) * 10) + (int(yaesu_command_buffer[x + 2]) - 48);
                parsed_value2 = ((int(yaesu_command_buffer[x + 4]) - 48) * 100) + ((int(yaesu_command_buffer[x + 5]) - 48) * 10) + (int(yaesu_command_buffer[x + 6]) - 48);
                if ((parsed_value > -1) && (parsed_value < 361) && (parsed_value2 > -1) && (parsed_value2 < 181)) {  // is it a valid azimuth?
                  timed_buffer_azimuths[timed_buffer_number_entries_loaded] = (parsed_value);
                  timed_buffer_elevations[timed_buffer_number_entries_loaded] = (parsed_value2);
                  timed_buffer_number_entries_loaded++;
                  timed_buffer_status = LOADED_AZIMUTHS_ELEVATIONS;
                  if (timed_buffer_number_entries_loaded > TIMED_INTERVAL_ARRAY_SIZE) {   // is the array full?
                    x = yaesu_command_buffer_index;  // array is full, go to the first azimuth and elevation
        
                  }
                } else {   // we hit an invalid bearing
                  timed_buffer_status = EMPTY;
                  timed_buffer_number_entries_loaded = 0;
                  strcpy(return_string,"?>");  // error
                  return;
                }
              }
            }
            timed_buffer_entry_pointer = 1;             // go to the first bearings
            parsed_value = timed_buffer_azimuths[0];
            parsed_elevation = timed_buffer_elevations[0];
          #else /* ifdef FEATURE_TIMED_BUFFER FEATURE_ELEVATION_CONTROL*/
            strcpy(return_string,"?>");
          #endif // FEATURE_TIMED_BUFFER FEATURE_ELEVATION_CONTROL
        } else {
          // this is a short form W command, just parse the azimuth and elevation and initiate rotation
          parsed_value = (((int(yaesu_command_buffer[1]) - 48) * 100) + ((int(yaesu_command_buffer[2]) - 48) * 10) + (int(yaesu_command_buffer[3]) - 48));
          parsed_elevation = (((int(yaesu_command_buffer[5]) - 48) * 100) + ((int(yaesu_command_buffer[6]) - 48) * 10) + (int(yaesu_command_buffer[7]) - 48));
        }
      
        #ifndef FEATURE_ELEVATION_CONTROL

          if ((parsed_value >= 0) && (parsed_value <= ((configuration.azimuth_starting_point + configuration.azimuth_rotation_capability)))) {
          //if ((parsed_value >= 0) && (parsed_value <= 360)) {
            submit_request(AZ, REQUEST_AZIMUTH, parsed_value, 32);
          } else {
            #ifdef DEBUG_PROCESS_YAESU
              if (debug_mode) {
                debug.print(F("process_yaesu_command: W cmd az error"));
              }
            #endif // DEBUG_PROCESS_YAESU
            strcpy(return_string,"?>");      // bogus elevation - return and error and don't do anything
          }
        
        #else
          if ((parsed_value >= 0) && (parsed_value <= ((configuration.azimuth_starting_point + configuration.azimuth_rotation_capability))) && (parsed_elevation >= 0) && (parsed_elevation <= (ELEVATION_MAXIMUM_DEGREES))) {
          #if defined(OPTION_ALLOW_ROTATIONAL_AND_CONFIGURATION_CMDS_AT_BOOT_UP)
            submit_request(AZ, REQUEST_AZIMUTH, parsed_value, 33);
            submit_request(EL, REQUEST_ELEVATION, parsed_elevation, 34);
          #else
            if (millis() > ROTATIONAL_AND_CONFIGURATION_CMD_IGNORE_TIME_MS){
              submit_request(AZ, REQUEST_AZIMUTH, parsed_value, 33);
              submit_request(EL, REQUEST_ELEVATION, parsed_elevation, 34);              
            }
          #endif
        } else {
          #ifdef DEBUG_PROCESS_YAESU
            if (debug_mode) {
              debug.print(F("process_yaesu_command: W cmd az/el error"));
            }
          #endif // DEBUG_PROCESS_YAESU
          strcpy(return_string,"?>");      // bogus elevation - return and error and don't do anything
        } 
        #endif // FEATURE_ELEVATION_CONTROL
        
        
        break;
        
      #ifdef OPTION_GS_232B_EMULATION
        case 'P':  // P - switch between 360 and 450 degree mode
          #if defined(OPTION_ALLOW_ROTATIONAL_AND_CONFIGURATION_CMDS_AT_BOOT_UP)
            if ((yaesu_command_buffer[1] == '3') && (yaesu_command_buffer_index > 2)) {  // P36 command
              configuration.azimuth_rotation_capability = 360;
              strcpy_P(return_string,(const char*) F("Mode 360 degree"));
              configuration_dirty = 1;
            } else {
              if ((yaesu_command_buffer[1] == '4') && (yaesu_command_buffer_index > 2)) { // P45 command
                configuration.azimuth_rotation_capability = 450;
                strcpy_P(return_string,(const char*) F("Mode 450 degree"));
                configuration_dirty = 1;
              } else {
                strcpy_P(return_string,(const char*) F("?>"));
              }
            }
          #else
            if (millis() > ROTATIONAL_AND_CONFIGURATION_CMD_IGNORE_TIME_MS){
              if ((yaesu_command_buffer[1] == '3') && (yaesu_command_buffer_index > 2)) {  // P36 command
                configuration.azimuth_rotation_capability = 360;
                strcpy_P(return_string,(const char*) F("Mode 360 degree"));
                // write_settings_to_eeprom();
              } else {
                if ((yaesu_command_buffer[1] == '4') && (yaesu_command_buffer_index > 2)) { // P45 command
                  configuration.azimuth_rotation_capability = 450;
                  strcpy_P(return_string,(const char*) F("Mode 450 degree"));
                  // write_settings_to_eeprom();
                } else {
                  strcpy_P(return_string,(const char*) F("?>"));
                }
              }
            }
          #endif  

        
        break;                 
        case 'Z':                                           // Z - Starting point toggle
          #if defined(OPTION_ALLOW_ROTATIONAL_AND_CONFIGURATION_CMDS_AT_BOOT_UP)
            if (azimuth_starting_point == 180) {
              azimuth_starting_point = 0;
              strcpy(return_string,"N");
            } else {
              azimuth_starting_point = 180;
              strcpy(return_string,"S");
            }
            strcat(return_string," Center");
          #else
            if (millis() > ROTATIONAL_AND_CONFIGURATION_CMD_IGNORE_TIME_MS){
              if (configuration.azimuth_starting_point == 180) {
                configuration.azimuth_starting_point = 0;
                strcpy(return_string,"N");
              } else {
                configuration.azimuth_starting_point = 180;
                strcpy(return_string,"S");
              }
              strcat_P(return_string,(const char*) F(" Center"));
            }
          #endif

          break;
        #endif
        
      default:
        strcpy_P(return_string,(const char*) F("?>"));
        #ifdef DEBUG_PROCESS_YAESU
          if (debug_mode) {
            debug.print(F("process_yaesu_command: yaesu_command_buffer_index: "));
            debug.print(yaesu_command_buffer_index);
            for (int debug_x = 0; debug_x < yaesu_command_buffer_index; debug_x++) {
              debug.print(F("process_yaesu_command: yaesu_command_buffer["));
              debug.print(debug_x);
              debug.print("]: ");
              debug.print(yaesu_command_buffer[debug_x]);
              debug.print(" ");
              debug.write(yaesu_command_buffer[debug_x]);
              debug.print("\n");;
            }
          }
        #endif // DEBUG_PROCESS_YAESU
    } /* switch */

} /* yaesu_serial_command */
  #endif // FEATURE_YAESU_EMULATION
// --------------------------------------------------------------

        
#ifdef FEATURE_WIFI
void initialize_wifi(){

  #ifdef DEBUG_WIFI
    control_port->println("initialize_wifi()");
  #endif // DEBUG_WIFI

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long wifi_connect_start_time = millis();
  while ((WiFi.status() != WL_CONNECTED) && ((millis() - wifi_connect_start_time) < WIFI_CONNECT_TIMEOUT_MS)) {
    delay(250);
  }

  if (WiFi.status() == WL_CONNECTED) {
    control_port->print("initialize_wifi: connected, IP: ");
    control_port->println(WiFi.localIP());
    wifi_server0.begin();
  } else {
    control_port->println("initialize_wifi: connection failed, WiFi control port not available");
  }

}
#endif //FEATURE_WIFI
// --------------------------------------------------------------

#ifdef FEATURE_WIFI
void service_wifi(){

  #ifdef DEBUG_LOOP
    control_port->println("service_wifi()");
    control_port->flush();
  #endif // DEBUG_LOOP

  byte incoming_byte = 0;
  char return_string[100] = "";
  static byte first_connect_occurred = 0;
  static long last_received_byte0 = 0;

  // clear things out if we received a partial message and it's been awhile
  if ((wifi_port_buffer_index) && ((millis()-last_received_byte0) > WIFI_MESSAGE_TIMEOUT_MS)){
    wifi_port_buffer_index = 0;
  }

  if (!wifi_client0.connected()){
    wifi_client0 = wifi_server0.available();
    first_connect_occurred = 0;
  }

  if (wifi_client0.connected()){

    if (!first_connect_occurred){  // clean out the cruft that's alway spit out on first connect
      while(wifi_client0.available()){wifi_client0.read();}
      first_connect_occurred = 1;
      return;
    }

    if (wifi_client0.available() > 0){        // the client has sent something
      incoming_byte = wifi_client0.read();
      last_received_byte0 = millis();

      #ifdef DEBUG_WIFI
      debug.print("service_wifi: client:") ;
      debug.print(" char:");
      debug.print((char) incoming_byte);
      debug.print("\n");
      #endif //DEBUG_WIFI

      if ((incoming_byte > 96) && (incoming_byte < 123)) {  // uppercase it
        incoming_byte = incoming_byte - 32;
      }

      if ((incoming_byte != 10) && (incoming_byte != 13)) { // add it to the buffer if it's not a line feed or carriage return
        wifi_port_buffer[wifi_port_buffer_index] = incoming_byte;
        wifi_port_buffer_index++;
      }

      if (((incoming_byte == 13) || (wifi_port_buffer_index >= COMMAND_BUFFER_SIZE)) && (wifi_port_buffer_index > 0)){  // do we have a carriage return?
        if ((wifi_port_buffer[0] == '\\') || (wifi_port_buffer[0] == '/')) {
          process_backslash_command(wifi_port_buffer, wifi_port_buffer_index, WIFI_PORT0, INCLUDE_RESPONSE_CODE, return_string, SOURCE_CONTROL_PORT);
        } else {
          #ifdef FEATURE_YAESU_EMULATION
            process_yaesu_command(wifi_port_buffer,wifi_port_buffer_index,WIFI_PORT0,return_string);
          #endif //FEATURE_YAESU_EMULATION
        }
        wifi_client0.println(return_string);
        wifi_port_buffer_index = 0;
      }

    }
  }

}
#endif //FEATURE_WIFI

//-------------------------------------------------------

void stop_rotation(){

  submit_request(AZ, REQUEST_STOP, 0, DBG_STOP_ROTATION);
  #ifdef FEATURE_ELEVATION_CONTROL
    submit_request(EL, REQUEST_STOP, 0, DBG_STOP_ROTATION);
  #endif


}

//-------------------------------------------------------

void change_tracking(byte action){

  #if defined(FEATURE_MOON_TRACKING) || defined(FEATURE_SUN_TRACKING) || defined(FEATURE_SATELLITE_TRACKING)
  switch(action){

    case DEACTIVATE_ALL: 
      #if defined(FEATURE_MOON_TRACKING)
        moon_tracking_active = 0;
      #endif
      #if defined(FEATURE_SUN_TRACKING) 
        sun_tracking_active = 0;
      #endif
      #if defined(FEATURE_SATELLITE_TRACKING)
        satellite_tracking_active = 0;
      #endif
      break;

    case DEACTIVATE_MOON_TRACKING:
      #if defined(FEATURE_MOON_TRACKING)
        moon_tracking_active = 0;   
      #endif    
      break;   

    case DEACTIVATE_SUN_TRACKING:
      #if defined(FEATURE_SUN_TRACKING) 
        sun_tracking_active = 0;      
      #endif    
      break;    

    case DEACTIVATE_SATELLITE_TRACKING:
      #if defined(FEATURE_SATELLITE_TRACKING)
        satellite_tracking_active = 0;     
      #endif    
      break;    

    case ACTIVATE_MOON_TRACKING:
      #if defined(FEATURE_MOON_TRACKING)
        moon_tracking_active = 1;
      #endif
      #if defined(FEATURE_SUN_TRACKING) 
        sun_tracking_active = 0;
      #endif
      #if defined(FEATURE_SATELLITE_TRACKING)
        satellite_tracking_active = 0;
      #endif    
      break;    

    case ACTIVATE_SUN_TRACKING:
      #if defined(FEATURE_MOON_TRACKING)
        moon_tracking_active = 0;
      #endif
      #if defined(FEATURE_SUN_TRACKING) 
        sun_tracking_active = 1;
      #endif
      #if defined(FEATURE_SATELLITE_TRACKING)
        satellite_tracking_active = 0;
      #endif    
      break;  

    case ACTIVATE_SATELLITE_TRACKING:
      #if defined(FEATURE_MOON_TRACKING)
        moon_tracking_active = 0;
      #endif
      #if defined(FEATURE_SUN_TRACKING) 
        sun_tracking_active = 0;
      #endif
      #if defined(FEATURE_SATELLITE_TRACKING)
        satellite_tracking_active = 1;
      #endif    
      break;    

  }

  #endif //defined(FEATURE_MOON_TRACKING) || defined(FEATURE_SUN_TRACKING) || defined(FEATURE_SATELLITE_TRACKING)
}



//-------------------------------------------------------



//------------------------------------------------------

#if defined(FEATURE_TEST_DISPLAY_AT_STARTUP)

void test_display(){
  
  char tempchar[12] = "";
  int display_number = 1;


  k3ngdisplay.print_top_left("1");
  k3ngdisplay.service(1);
  delay(150);  
  k3ngdisplay.print_top_right("2");
  k3ngdisplay.service(1);
  delay(150);   
  k3ngdisplay.print_bottom_left("3");
  k3ngdisplay.service(1);
  delay(150);   
  k3ngdisplay.print_bottom_right("4"); 
  k3ngdisplay.service(1);
  delay(2000);
  k3ngdisplay.clear();

  for (int y = 0; y < LCD_ROWS;y++){
    for (int x = 0; x < LCD_COLUMNS;x++){
      dtostrf(display_number,0,0,tempchar);
      k3ngdisplay.print(tempchar,x,y);
      k3ngdisplay.service(1);
      delay(100);
      display_number++;
      if (display_number > 9){display_number = 0;}
    }
  }
  
  delay(2000);
}


#endif //FEATURE_TEST_DISPLAY_AT_STARTUP



//------------------------------------------------------
#if defined(FEATURE_AUTOPARK)
void service_autopark(){

  #ifdef DEBUG_LOOP
    control_port->println("service_autopark()");
    control_port->flush();
  #endif // DEBUG_LOOP


  byte autopark_inhibited = 0;

  #if defined(FEATURE_ELEVATION_CONTROL)
    if ((az_state != IDLE) || (el_state != IDLE) || (park_status == PARKED)){
      last_activity_time_autopark = millis();
    }
  #else
    if ((az_state != IDLE) || (park_status == PARKED)){
      last_activity_time_autopark = millis();
    }
  #endif

  if (pin_autopark_timer_reset){
    if (digitalReadEnhanced(pin_autopark_timer_reset) == LOW){
      last_activity_time_autopark = millis();
      if (park_status == PARK_INITIATED){
        deactivate_park();
        stop_rotation();
      }      
    }
  }

  if (pin_autopark_disable){
    if (digitalReadEnhanced(pin_autopark_disable) == LOW){
      autopark_inhibited = 1;
      last_activity_time_autopark = millis();
      if (park_status == PARK_INITIATED){
        deactivate_park();
        stop_rotation();   
      }
    }
  }

  if ((configuration.autopark_active) && (!autopark_inhibited) && ((millis() - last_activity_time_autopark) > (long(configuration.autopark_time_minutes) * 60000L)) 
    && ((park_status != PARK_INITIATED) || (park_status != PARKED))) {
    #if defined(DEBUG_PARK)
      debug.print(F("service_autopark: initiating park\n"));
    #endif
    initiate_park();
  }


}
#endif //FEATURE_AUTOPARK
//------------------------------------------------------

void service_process_debug(byte action,byte process_id){

  #if defined(DEBUG_PROCESSES)

    

    static unsigned long last_process_start_time[PROCESS_TABLE_SIZE];
    static unsigned long process_cumulative_time[PROCESS_TABLE_SIZE];
    static unsigned long last_output = 0;

    last_process_start_time[PROCESS_DEBUG] = micros();

    switch(action){
      case DEBUG_PROCESSES_SERVICE:

        // PROCESS_LOOP (0) is a special case

        process_cumulative_time[PROCESS_LOOP] = process_cumulative_time[PROCESS_LOOP] + (micros() - last_process_start_time[PROCESS_LOOP]);
        last_process_start_time[PROCESS_LOOP] = micros();

        if (((millis() - last_output) > 3000) && (debug_mode)){
          debug.println("\r\npid\tprocess\t\t\t\t%\tmicros");
          for (int x = 0;x < PROCESS_TABLE_SIZE;x++){
            if (process_cumulative_time[x] > 0){
              control_port->print(x);
              control_port->print("\t");
              switch(x){
                case PROCESS_LOOP: control_port->print("loop()\t\t\t"); break;
                case PROCESS_READ_HEADINGS: control_port->print("read_headings\t\t"); break;
                case PROCESS_CHECK_SERIAL: control_port->print("check_serial\t\t"); break;
                case PROCESS_SERVICE_NEXTION: control_port->print("service_nextion\t\t"); break;
                case PROCESS_UPDATE_LCD_DISPLAY: control_port->print("update_lcd_display\t"); break;
                case PROCESS_SERVICE_ROTATION: control_port->print(F("service_rotation\t")); break;
                case PROCESS_UPDATE_SUN_POSITION: control_port->print(F("update_sun_position\t")); break;
                case PROCESS_UPDATE_MOON_POSITION: control_port->print(F("update_moon_position\t")); break;
                case PROCESS_UPDATE_TIME: control_port->print(F("update_time\t\t")); break;
                case PROCESS_SERVICE_GPS: control_port->print(F("service_gps\t\t")); break;
                case PROCESS_CHECK_FOR_DIRTY_CONFIGURATION: control_port->print(F("check_for_dirty_configuration")); break;
                case PROCESS_CHECK_BUTTONS: control_port->print(F("check_buttons\t\t")); break;
                case PROCESS_MISC_ADMIN: control_port->print("misc_admin\t\t"); break;
                case PROCESS_DEBUG: control_port->print("debug\t\t\t"); break;
              }
              control_port->print("\t");
              control_port->print(((float)process_cumulative_time[x]/(float)micros())*100.0,0);
              control_port->print("%");                
              control_port->print("\t");
              control_port->println(process_cumulative_time[x]);
            }
            
          }
          control_port->println("\r\n");
          last_output = millis();
        }

        break;

      case DEBUG_PROCESSES_PROCESS_ENTER:
        last_process_start_time[process_id] = micros();
        break;


      case DEBUG_PROCESSES_PROCESS_EXIT:
        process_cumulative_time[process_id] = process_cumulative_time[process_id] + (micros() - last_process_start_time[process_id]);
        break;    


    }

    process_cumulative_time[PROCESS_DEBUG] = process_cumulative_time[PROCESS_DEBUG] + (micros() - last_process_start_time[PROCESS_DEBUG]);

  #endif //DEBUG_PROCESSES


}

//------------------------------------------------------
//------------------------------------------------------

void run_this_once(){


  #ifdef DEBUG_LOOP
    control_port->println(F("run_this_once()"));
    control_port->flush();
  #endif // DEBUG_LOOP  

  #if defined(DEBUG_TEST_POLAR_TO_CARTESIAN)
    double x = 0;
    double y = 0;
    int el = 0;
    control_port->println("\r\naz\tel\tx\ty");
    for (int az = 0;az < 360;az = az + 5){
      el = 0;
      convert_polar_to_cartesian(COORDINATE_PLANE_UPPER_LEFT_ORIGIN,az,el,&x,&y);
      control_port->print(az);
      control_port->print("\t");
      control_port->print(el);
      control_port->print("\t");    
      control_port->print(x);
      control_port->print("\t");
      control_port->println(y);
    }
  #endif //DEBUG_TEST_POLAR_TO_CARTESIAN


}

//------------------------------------------------------

void send_vt100_code(char* code_to_send){
  
  #if defined(OPTION_CLI_VT100)

    control_port->write(0x1B);  //ESCape
    control_port->print(code_to_send);



  #endif //OPTION_CLI_VT100

}

//------------------------------------------------------
#if defined(FEATURE_SATELLITE_TRACKING)
  void warm_initialize_satellite_array(){

    for (int x = 0;x < SATELLITE_LIST_LENGTH;x++){
      if ((satellite[x].status != 255) && (strlen(satellite[x].name) > 2)){
        satellite[x].next_pass_max_el = 0;
        satellite[x].status = satellite[x].status & B11111000; // clear aos (1), timeout (2), and AOS/LOS state change (4) flags
        cleartime(&satellite[x].next_aos);
        cleartime(&satellite[x].next_los);
        satellite[x].azimuth = 0;
        satellite[x].elevation = 0;
        satellite[x].next_aos_az = 0;
        satellite[x].next_los_az = 0;
        satellite[x].longitude = 0;
        satellite[x].latitude = 0; 
        satellite[x].order = 254;  // 254 = valid slot, but no order placed on it yet             
      }
    }            
            
  }
#endif //FEATURE_SATELLITE_TRACKING

//------------------------------------------------------

#if defined(FEATURE_MASTER_WITH_SERIAL_SLAVE) || defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE)
byte submit_remote_command(byte remote_command_to_send, byte parm1, int parm2){

  #ifdef FEATURE_MASTER_WITH_ETHERNET_SLAVE
    char ethernet_send_string[32];
    char temp_string[32];
  #endif //FEATURE_MASTER_WITH_ETHERNET_SLAVE

  #ifdef FEATURE_MASTER_WITH_ETHERNET_SLAVE
    if (ethernetslavelinkclient0_state != ETHERNET_SLAVE_CONNECTED){return 0;}
  #endif //FEATURE_MASTER_WITH_ETHERNET_SLAVE

  // if ((remote_unit_command_submitted && ((remote_command_to_send == REMOTE_UNIT_AZ_COMMAND) || (remote_command_to_send == REMOTE_UNIT_EL_COMMAND) || (remote_command_to_send == REMOTE_UNIT_CL_COMMAND))) || suspend_remote_commands) {
  if (/*remote_unit_command_submitted ||*/ suspend_remote_commands) {
    return 0;
  } else {
    switch (remote_command_to_send) {
      case REMOTE_UNIT_CL_COMMAND:
        #ifdef FEATURE_MASTER_WITH_SERIAL_SLAVE
          REMOTE_PORT.println("CL");
        #endif //FEATURE_MASTER_WITH_SERIAL_SLAVE
        #ifdef FEATURE_MASTER_WITH_ETHERNET_SLAVE
          ethernet_slave_link_send("CL");
        #endif //FEATURE_MASTER_WITH_ETHERNET_SLAVE
        #if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
          if (remote_port_tx_sniff) {control_port->println("CL");}
        #endif
        // remote_unit_command_submitted = REMOTE_UNIT_CL_COMMAND;
        break;


      case REMOTE_UNIT_PM_COMMAND:
        #ifdef FEATURE_MASTER_WITH_SERIAL_SLAVE
          REMOTE_PORT.println("PM");
        #endif //FEATURE_MASTER_WITH_SERIAL_SLAVE
        #ifdef FEATURE_MASTER_WITH_ETHERNET_SLAVE
          ethernet_slave_link_send("PM");
        #endif //FEATURE_MASTER_WITH_ETHERNET_SLAVE
        #if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
          if (remote_port_tx_sniff) {control_port->println("PM");}
        #endif
        //remote_unit_command_submitted = REMOTE_UNIT_PG_COMMAND;
        break;

      // case REMOTE_UNIT_AZ_COMMAND:
      //   #ifdef FEATURE_MASTER_WITH_SERIAL_SLAVE
      //     REMOTE_PORT.println("AZ");
      //   #endif //FEATURE_MASTER_WITH_SERIAL_SLAVE
      //   #ifdef FEATURE_MASTER_WITH_ETHERNET_SLAVE
      //     ethernet_slave_link_send("AZ");
      //   #endif //FEATURE_MASTER_WITH_ETHERNET_SLAVE
      //   #if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
      //     if (remote_port_tx_sniff) {control_port->println("AZ");}
      //   #endif
      //   remote_unit_command_submitted = REMOTE_UNIT_AZ_COMMAND;
      //   break;

      // case REMOTE_UNIT_EL_COMMAND:
      //   #ifdef FEATURE_MASTER_WITH_SERIAL_SLAVE
      //     REMOTE_PORT.println("EL");
      //   #endif //FEATURE_MASTER_WITH_SERIAL_SLAVE
      //   #ifdef FEATURE_MASTER_WITH_ETHERNET_SLAVE
      //     ethernet_slave_link_send("EL");
      //   #endif //FEATURE_MASTER_WITH_ETHERNET_SLAVE        
      //   #if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
      //     if (remote_port_tx_sniff) {control_port->println("EL");}
      //   #endif
      //   remote_unit_command_submitted = REMOTE_UNIT_EL_COMMAND;
      //   break;


      case REMOTE_UNIT_AW_COMMAND:
        #ifdef FEATURE_MASTER_WITH_SERIAL_SLAVE
          // take_care_of_pending_remote_command();
          REMOTE_PORT.print("AW");
          #if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
            if (remote_port_tx_sniff) {control_port->print("AW");}
          #endif          
          parm1 = parm1 - 100;   // pin number
          if (parm1 < 10) {
            REMOTE_PORT.print("0");
            #if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
              if (remote_port_tx_sniff) {control_port->print("0");}
            #endif
          }
          REMOTE_PORT.print(parm1);
          #if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
            if (remote_port_tx_sniff) {control_port->print(parm1);}
          #endif          
          if (parm2 < 10) {
            REMOTE_PORT.print("0");
            #if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
              if (remote_port_tx_sniff) {control_port->print("0");}
            #endif            
          }
          if (parm2 < 100) {
            REMOTE_PORT.print("0");
            #if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
              if (remote_port_tx_sniff) {control_port->print("0");}
            #endif            
          }
          REMOTE_PORT.println(parm2);
          #if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
            if (remote_port_tx_sniff) {control_port->println(parm2);}
          #endif                
        #endif //FEATURE_MASTER_WITH_SERIAL_SLAVE

        #ifdef FEATURE_MASTER_WITH_ETHERNET_SLAVE
          // take_care_of_pending_remote_command();
          strcpy(ethernet_send_string,"AW");
          parm1 = parm1 - 100;   // pin number
          if (parm1 < 10) {strcat(ethernet_send_string,"0");}
          dtostrf(parm1,0,0,temp_string);
          if (parm2 < 10) {strcat(ethernet_send_string,"0");}
          if (parm2 < 100) {strcat(ethernet_send_string,"0");}
          dtostrf(parm2,0,0,temp_string);
          strcat(ethernet_send_string,temp_string);
          ethernet_slave_link_send(ethernet_send_string);
        #endif //FEATURE_MASTER_WITH_ETHERNET_SLAVE

        // remote_unit_command_submitted = REMOTE_UNIT_OTHER_COMMAND;
        break;

      case REMOTE_UNIT_DHL_COMMAND:
        #ifdef FEATURE_MASTER_WITH_SERIAL_SLAVE
          // take_care_of_pending_remote_command();
          REMOTE_PORT.print("D");
          #if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
            if (remote_port_tx_sniff) {control_port->print("D");}
          #endif              
          if (parm2 == HIGH) {
            REMOTE_PORT.print("H");
            #if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
              if (remote_port_tx_sniff) {control_port->print("H");}
            #endif                
          } else {
            REMOTE_PORT.print("L");
            #if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
              if (remote_port_tx_sniff) {control_port->print("L");}
            #endif                
          }
          parm1 = parm1 - 100;
          if (parm1 < 10) {
            REMOTE_PORT.print("0");
            #if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
              if (remote_port_tx_sniff) {control_port->print("0");}
            #endif                
          }
          REMOTE_PORT.println(parm1);
          #if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
            if (remote_port_tx_sniff) {control_port->println(parm1);}
          #endif              
        #endif //FEATURE_MASTER_WITH_SERIAL_SLAVE


        #ifdef FEATURE_MASTER_WITH_ETHERNET_SLAVE
          // take_care_of_pending_remote_command();
          strcpy(ethernet_send_string,"D");
          if (parm2 == HIGH) {strcat(ethernet_send_string,"H");} else {strcat(ethernet_send_string,"L");}
          parm1 = parm1 - 100;
          if (parm1 < 10) {strcat(ethernet_send_string,"0");}
          dtostrf(parm1,0,0,temp_string);
          strcat(ethernet_send_string,temp_string);
          ethernet_slave_link_send(ethernet_send_string);
        #endif //FEATURE_MASTER_WITH_ETHERNET_SLAVE

        // remote_unit_command_submitted = REMOTE_UNIT_OTHER_COMMAND;

        break;

      case REMOTE_UNIT_DOI_COMMAND:
        #ifdef FEATURE_MASTER_WITH_SERIAL_SLAVE
          // take_care_of_pending_remote_command();
          REMOTE_PORT.print("D");
          #if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
            if (remote_port_tx_sniff) {control_port->print("D");}
          #endif                
          if (parm2 == OUTPUT) {
            REMOTE_PORT.print("O");
            #if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
              if (remote_port_tx_sniff) {control_port->print("O");}
            #endif                  
          } else {
            REMOTE_PORT.print("I");}
            #if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
              if (remote_port_tx_sniff) {control_port->print("I");}
            #endif                  
          parm1 = parm1 - 100;
          if (parm1 < 10) {
            REMOTE_PORT.print("0");
            #if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
              if (remote_port_tx_sniff) {control_port->print("0");}
            #endif                  
          }
          REMOTE_PORT.println(parm1);
            #if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
              if (remote_port_tx_sniff) {control_port->println(parm1);}
            #endif                
          // remote_unit_command_submitted = REMOTE_UNIT_OTHER_COMMAND;
          // get_remote_port_ok_response();
        #endif //FEATURE_MASTER_WITH_SERIAL_SLAVE

        #ifdef FEATURE_MASTER_WITH_ETHERNET_SLAVE
          // take_care_of_pending_remote_command();
          strcpy(ethernet_send_string,"D");
          if (parm2 == OUTPUT) {strcat(ethernet_send_string,"O");} else {strcat(ethernet_send_string,"I");}
          parm1 = parm1 - 100;
          if (parm1 < 10) {strcat(ethernet_send_string,"0");}
          dtostrf(parm1,0,0,temp_string);
          strcat(ethernet_send_string,temp_string);
          ethernet_slave_link_send(ethernet_send_string);
          // remote_unit_command_submitted = REMOTE_UNIT_OTHER_COMMAND;
        #endif //FEATURE_MASTER_WITH_ETHERNET_SLAVE

        break;

      case REMOTE_UNIT_GS_COMMAND:
        #ifdef FEATURE_MASTER_WITH_SERIAL_SLAVE
          REMOTE_PORT.println("GS");
        #endif //FEATURE_MASTER_WITH_SERIAL_SLAVE
        #ifdef FEATURE_MASTER_WITH_ETHERNET_SLAVE
          ethernet_slave_link_send("GS");
        #endif //FEATURE_MASTER_WITH_ETHERNET_SLAVE
        #if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
          if (remote_port_tx_sniff) {control_port->println("GS");}
        #endif
        // remote_unit_command_submitted = REMOTE_UNIT_GS_COMMAND;
        break;

      case REMOTE_UNIT_RC_COMMAND:    
        #ifdef FEATURE_MASTER_WITH_SERIAL_SLAVE
          REMOTE_PORT.println("RC");
        #endif //FEATURE_MASTER_WITH_SERIAL_SLAVE
        #ifdef FEATURE_MASTER_WITH_ETHERNET_SLAVE
          ethernet_slave_link_send("RC");
        #endif //FEATURE_MASTER_WITH_ETHERNET_SLAVE
        #if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
          if (remote_port_tx_sniff) {control_port->println("RC");}
        #endif
        // remote_unit_command_submitted = REMOTE_UNIT_RC_COMMAND;
        break;

      // zzzzzz

      case REMOTE_UNIT_RL_COMMAND:
        #ifdef FEATURE_MASTER_WITH_SERIAL_SLAVE
          REMOTE_PORT.println("RL");
        #endif //FEATURE_MASTER_WITH_SERIAL_SLAVE
        #ifdef FEATURE_MASTER_WITH_ETHERNET_SLAVE
          ethernet_slave_link_send("RL");
        #endif //FEATURE_MASTER_WITH_ETHERNET_SLAVE
        #if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
          if (remote_port_tx_sniff) {control_port->println("RL");}
        #endif
        // remote_unit_command_submitted = REMOTE_UNIT_RL_COMMAND;
        break;


      case REMOTE_UNIT_RR_COMMAND:
        #ifdef FEATURE_MASTER_WITH_SERIAL_SLAVE
          REMOTE_PORT.println("RR");
        #endif //FEATURE_MASTER_WITH_SERIAL_SLAVE
        #ifdef FEATURE_MASTER_WITH_ETHERNET_SLAVE
          ethernet_slave_link_send("RR");
        #endif //FEATURE_MASTER_WITH_ETHERNET_SLAVE
        #if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
          if (remote_port_tx_sniff) {control_port->println("RR");}
        #endif
        // remote_unit_command_submitted = REMOTE_UNIT_RR_COMMAND;
        break;

      case REMOTE_UNIT_RD_COMMAND:
        #ifdef FEATURE_MASTER_WITH_SERIAL_SLAVE
          REMOTE_PORT.println("RD");
        #endif //FEATURE_MASTER_WITH_SERIAL_SLAVE
        #ifdef FEATURE_MASTER_WITH_ETHERNET_SLAVE
          ethernet_slave_link_send("RD");
        #endif //FEATURE_MASTER_WITH_ETHERNET_SLAVE
        #if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
          if (remote_port_tx_sniff) {control_port->println("RD");}
        #endif
        // remote_unit_command_submitted = REMOTE_UNIT_RD_COMMAND;
        break;        

      case REMOTE_UNIT_RU_COMMAND:
        #ifdef FEATURE_MASTER_WITH_SERIAL_SLAVE
          REMOTE_PORT.println("RU");
        #endif //FEATURE_MASTER_WITH_SERIAL_SLAVE
        #ifdef FEATURE_MASTER_WITH_ETHERNET_SLAVE
          ethernet_slave_link_send("RU");
        #endif //FEATURE_MASTER_WITH_ETHERNET_SLAVE
        #if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
          if (remote_port_tx_sniff) {control_port->println("RU");}
        #endif
        // remote_unit_command_submitted = REMOTE_UNIT_RU_COMMAND;
        break;

      case REMOTE_UNIT_RA_COMMAND:
        #ifdef FEATURE_MASTER_WITH_SERIAL_SLAVE
          REMOTE_PORT.println("RA");
        #endif //FEATURE_MASTER_WITH_SERIAL_SLAVE
        #ifdef FEATURE_MASTER_WITH_ETHERNET_SLAVE
          ethernet_slave_link_send("RA");
        #endif //FEATURE_MASTER_WITH_ETHERNET_SLAVE
        #if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
          if (remote_port_tx_sniff) {control_port->println("RA");}
        #endif
        // remote_unit_command_submitted = REMOTE_UNIT_RA_COMMAND;
        break;

      case REMOTE_UNIT_RE_COMMAND:
        #ifdef FEATURE_MASTER_WITH_SERIAL_SLAVE
          REMOTE_PORT.println("RE");
        #endif //FEATURE_MASTER_WITH_SERIAL_SLAVE
        #ifdef FEATURE_MASTER_WITH_ETHERNET_SLAVE
          ethernet_slave_link_send("RE");
        #endif //FEATURE_MASTER_WITH_ETHERNET_SLAVE
        #if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
          if (remote_port_tx_sniff) {control_port->println("RE");}
        #endif
        // remote_unit_command_submitted = REMOTE_UNIT_RE_COMMAND;
        break;

      case REMOTE_UNIT_RS_COMMAND:
        #ifdef FEATURE_MASTER_WITH_SERIAL_SLAVE
          REMOTE_PORT.println("RS");
        #endif //FEATURE_MASTER_WITH_SERIAL_SLAVE
        #ifdef FEATURE_MASTER_WITH_ETHERNET_SLAVE
          ethernet_slave_link_send("RS");
        #endif //FEATURE_MASTER_WITH_ETHERNET_SLAVE
        #if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
          if (remote_port_tx_sniff) {control_port->println("RS");}
        #endif
        // remote_unit_command_submitted = REMOTE_UNIT_RS_COMMAND;
        break;

    } /* switch */
    // last_remote_unit_command_time = millis();
    // remote_unit_command_results_available = 0;
    return 1;
  }



} /* submit_remote_command */
#endif // defined(FEATURE_MASTER_WITH_SERIAL_SLAVE) || defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE)

// --------------------------------------------------------------------------

#if defined(FEATURE_MASTER_WITH_SERIAL_SLAVE) || defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE)
  byte is_ascii_number(byte char_in){

    if ((char_in > 47) && (char_in < 58)) {
      return 1;
    } else {
      return 0;
    }

  }
#endif // defined(FEATURE_MASTER_WITH_SERIAL_SLAVE) || defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE)
// --------------------------------------------------------------------------
#if defined(FEATURE_MASTER_WITH_SERIAL_SLAVE) || defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE)
  void service_remote_communications_incoming_buffer(){


    #if defined(FEATURE_CLOCK) && defined(OPTION_SYNC_MASTER_CLOCK_TO_SLAVE)
      int temp_year = 0;
      byte temp_month = 0;
      byte temp_day = 0;
      byte temp_minute = 0;
      byte temp_hour = 0;
      byte temp_sec = 0;
    #endif // defined(FEATURE_CLOCK) && defined(OPTION_SYNC_MASTER_CLOCK_TO_SLAVE)

    float temp_float_latitude = 0;
    float temp_float_longitude = 0;



    byte good_data = 0;

    if (remote_unit_port_buffer_carriage_return_flag) {

      #ifdef DEBUG_SVC_REMOTE_COMM_INCOMING_BUFFER
        debug.print("service_remote_communications_incoming_buffer: remote_unit_port_buffer_index: ");
        debug.print(remote_unit_port_buffer_index);
        debug.print(" buffer: ");
        for (int x = 0; x < remote_unit_port_buffer_index; x++) {
          debug.write((char*)remote_unit_port_buffer[x]);
          debug.println("$");
        }
      #endif // DEBUG_SVC_REMOTE_COMM_INCOMING_BUFFER


      // Ping for Master Response
      if ((remote_unit_port_buffer[0] == 'P') && (remote_unit_port_buffer[1] == 'M')) {
        last_ping_receive_time = millis();
        good_data = 1;
      }

      // OK Response - Still Needed?
      if ((remote_unit_port_buffer[0] == 'O') && (remote_unit_port_buffer[1] == 'K')) {
        good_data = 1;
      }

      //REMOTE_UNIT_RC_COMMAND:  //RC+40.9946 -075.6596
      #if defined(OPTION_SYNC_MASTER_COORDINATES_TO_SLAVE)
        if ((remote_unit_port_buffer[0] == 'R') && (remote_unit_port_buffer[1] == 'C') && (remote_unit_port_buffer[5] == '.') && (remote_unit_port_buffer[10] == ' ') && (remote_unit_port_buffer[15] == '.')){
          temp_float_latitude = ((remote_unit_port_buffer[3]-48)*10) + (remote_unit_port_buffer[4]-48) + ((remote_unit_port_buffer[6]-48)/10.0) + ((remote_unit_port_buffer[7]-48)/100.0) + ((remote_unit_port_buffer[8]-48)/1000.0) + ((remote_unit_port_buffer[9]-48)/10000.0);
          if (remote_unit_port_buffer[2] == '-') {
            temp_float_latitude = temp_float_latitude * -1;
          }
          temp_float_longitude = ((remote_unit_port_buffer[12]-48)*100) + ((remote_unit_port_buffer[13]-48)*10) + (remote_unit_port_buffer[14]-48) + ((remote_unit_port_buffer[16]-48)/10.0)+ ((remote_unit_port_buffer[17]-48)/100.0) + ((remote_unit_port_buffer[18]-48)/1000.0) + ((remote_unit_port_buffer[19]-48)/10000.0);
          if (remote_unit_port_buffer[11] == '-') {
            temp_float_longitude = temp_float_longitude * -1;
          }
          if ((temp_float_latitude <= 90) && (temp_float_latitude >= -90) && (temp_float_longitude <= 180) && (temp_float_longitude >= -180)){
            latitude = temp_float_latitude;
            longitude = temp_float_longitude;
            #ifdef DEBUG_SYNC_MASTER_COORDINATES_TO_SLAVE
              debug.println("service_remote_communications_incoming_buffer: coordinates synced to slave");
            #endif //DEBUG_SYNC_MASTER_COORDINATES_TO_SLAVE              
          }         
          good_data = 1;
        }
      #endif //OPTION_SYNC_MASTER_COORDINATES_TO_SLAVE

      // REMOTE_UNIT_GS_COMMAND:
      #if defined(OPTION_SYNC_MASTER_CLOCK_TO_SLAVE) && defined(FEATURE_CLOCK)
        if ((remote_unit_port_buffer[0] == 'G') && (remote_unit_port_buffer[1] == 'S')){
          if (remote_unit_port_buffer[2] == '1'){
            if (clock_status == SLAVE_SYNC) {clock_status = SLAVE_SYNC_GPS;}
            good_data = 1;
          } else {
            if (remote_unit_port_buffer[2] == '0') {good_data = 1;}
          }
        }
      #endif // defined(OPTION_SYNC_MASTER_CLOCK_TO_SLAVE) && defined(FEATURE_CLOCK)


      // REMOTE_UNIT_CL_COMMAND:
      if ((remote_unit_port_buffer[0] == 'C') && (remote_unit_port_buffer[1] == 'L') && (remote_unit_port_buffer[12] == ' ') && (remote_unit_port_buffer[21] == 'Z')){
        #if defined(FEATURE_CLOCK) && defined(OPTION_SYNC_MASTER_CLOCK_TO_SLAVE)
          temp_year = ((remote_unit_port_buffer[2] - 48) * 1000) + ((remote_unit_port_buffer[3] - 48) * 100) + ((remote_unit_port_buffer[4] - 48) * 10) + (remote_unit_port_buffer[5] - 48);
          temp_month = ((remote_unit_port_buffer[7] - 48) * 10) + (remote_unit_port_buffer[8] - 48);
          temp_day = ((remote_unit_port_buffer[10] - 48) * 10) + (remote_unit_port_buffer[11] - 48);
          temp_hour = ((remote_unit_port_buffer[13] - 48) * 10) + (remote_unit_port_buffer[14] - 48);
          temp_minute = ((remote_unit_port_buffer[16] - 48) * 10) + (remote_unit_port_buffer[17] - 48);
          temp_sec = ((remote_unit_port_buffer[19] - 48) * 10) + (remote_unit_port_buffer[20] - 48);
          if ((temp_year > 2013) && (temp_year < 2070) && (temp_month > 0) && (temp_month < 13) && (temp_day > 0) && (temp_day < 32) && (temp_hour < 24) && (temp_minute < 60) && (temp_sec < 60)){
            #if defined(OPTION_USE_OLD_TIME_CODE)
              set_clock.year = temp_year;
              set_clock.month = temp_month;
              set_clock.day = temp_day;
              set_clock.hours = temp_hour;
              set_clock.minutes = temp_minute;
              set_clock.seconds = temp_sec;
              millis_at_last_calibration = millis();
              //update_time();
            #else //OPTION_USE_OLD_TIME_CODE
              setTime(temp_hour, temp_minute, temp_sec, temp_day, temp_month, temp_year);
            #endif //OPTION_USE_OLD_TIME_CODE
            #ifdef DEBUG_SYNC_MASTER_CLOCK_TO_SLAVE
              debug.println("service_remote_communications_incoming_buffer: clock synced to slave clock");
            #endif //DEBUG_SYNC_MASTER_CLOCK_TO_SLAVE
            good_data = 1;
            clock_synced_to_remote = 1;
            if (clock_status == FREE_RUNNING) {clock_status = SLAVE_SYNC;}
          } else {
            #ifdef DEBUG_SYNC_MASTER_CLOCK_TO_SLAVE
              debug.println("service_remote_communications_incoming_buffer: slave clock sync error");
            #endif //DEBUG_SYNC_MASTER_CLOCK_TO_SLAVE  
            if ((clock_status == SLAVE_SYNC) || (clock_status == SLAVE_SYNC_GPS)) {clock_status = FREE_RUNNING;}   
          }
        #endif // defined(FEATURE_CLOCK) && defined(OPTION_SYNC_MASTER_CLOCK_TO_SLAVE)
        #if !defined(FEATURE_CLOCK) || !defined(OPTION_SYNC_MASTER_CLOCK_TO_SLAVE)
          good_data = 1;
        #endif //!defined(FEATURE_CLOCK) || !defined(OPTION_SYNC_MASTER_CLOCK_TO_SLAVE)
      } else {
        #if defined(FEATURE_CLOCK) && defined(OPTION_SYNC_MASTER_CLOCK_TO_SLAVE)
          #if defined(DEBUG_SYNC_MASTER_CLOCK_TO_SLAVE)
            debug.print("service_remote_communications_incoming_buffer: REMOTE_UNIT_CL_COMMAND format error.  remote_unit_port_buffer_index: ");
            debug.print(remote_unit_port_buffer_index);
            debug.println("");
          #endif // defined(DEBUG_SYNC_MASTER_CLOCK_TO_SLAVE)
        if ((clock_status == SLAVE_SYNC) || (clock_status == SLAVE_SYNC_GPS)) {clock_status = FREE_RUNNING;} 
      #endif // defined(FEATURE_CLOCK) && defined(OPTION_SYNC_MASTER_CLOCK_TO_SLAVE)          
      }

      if ((remote_unit_port_buffer_index == 13) && (remote_unit_port_buffer[0] == 'A') && (remote_unit_port_buffer[1] == 'Z') &&
          (is_ascii_number(remote_unit_port_buffer[2])) && (is_ascii_number(remote_unit_port_buffer[3])) && (is_ascii_number(remote_unit_port_buffer[4])) && (is_ascii_number(remote_unit_port_buffer[6]))  && (is_ascii_number(remote_unit_port_buffer[7])) && (is_ascii_number(remote_unit_port_buffer[8])) && (is_ascii_number(remote_unit_port_buffer[9])) && (is_ascii_number(remote_unit_port_buffer[10])) && (is_ascii_number(remote_unit_port_buffer[11]))) {
        remote_unit_azimuth_float = float((remote_unit_port_buffer[2] - 48) * 100) + float((remote_unit_port_buffer[3] - 48) * 10) + float(remote_unit_port_buffer[4] - 48) + (float(remote_unit_port_buffer[6] - 48) / (float)10.0) + (float(remote_unit_port_buffer[7] - 48) / (float)100.0) + (float(remote_unit_port_buffer[8] - 48) / (float)1000.0) + (float(remote_unit_port_buffer[9] - 48) / (float)10000.0) + (float(remote_unit_port_buffer[10] - 48) / (float)100000.0) + (float(remote_unit_port_buffer[11] - 48) / (float)1000000.0);
        good_data = 1;
      }

      if ((remote_unit_port_buffer_index == 14) && (remote_unit_port_buffer[0] == 'E') && (remote_unit_port_buffer[1] == 'L') &&
          (is_ascii_number(remote_unit_port_buffer[3])) && (is_ascii_number(remote_unit_port_buffer[4])) && (is_ascii_number(remote_unit_port_buffer[5])) && (is_ascii_number(remote_unit_port_buffer[7])) && (is_ascii_number(remote_unit_port_buffer[8])) && (is_ascii_number(remote_unit_port_buffer[9])) && (is_ascii_number(remote_unit_port_buffer[10])) && (is_ascii_number(remote_unit_port_buffer[11])) && (is_ascii_number(remote_unit_port_buffer[12]))) {
        remote_unit_elevation_float = (float(remote_unit_port_buffer[3] - 48) * 100) + float((remote_unit_port_buffer[4] - 48) * 10) + float(remote_unit_port_buffer[5] - 48) + (float(remote_unit_port_buffer[7] - 48) / (float)10.0)  + (float(remote_unit_port_buffer[8] - 48) / (float)100.0)  + (float(remote_unit_port_buffer[9] - 48) / (float)1000.0)  + (float(remote_unit_port_buffer[10] - 48) / (float)10000.0)  + (float(remote_unit_port_buffer[11] - 48) / (float)100000.0)  + (float(remote_unit_port_buffer[12] - 48) / (float)1000000.0);
        if (remote_unit_port_buffer[2] == '+') {
          good_data = 1;
        }
        if (remote_unit_port_buffer[2] == '-') {
          remote_unit_elevation_float = remote_unit_elevation_float * (float)-1.0;
          good_data = 1;
        }
      }


      if (good_data) {

        remote_unit_good_results++;

        // #ifdef DEBUG_SVC_REMOTE_COMM_INCOMING_BUFFER
        //   debug.print("service_remote_communications_incoming_buffer: remote_unit_command_results_available: ");
        //   debug.print(remote_unit_command_results_available);
        //   debug.print(" remote_unit_command_result_float: ");
        //   debug.print(remote_unit_command_result_float,2);
        //   debug.println("");
        // #endif // DEBUG_SVC_REMOTE_COMM_INCOMING_BUFFER
      } else {
        #ifdef DEBUG_SVC_REMOTE_COMM_INCOMING_BUFFER_BAD_DATA
          debug.print("service_remote_communications_incoming_buffer:");
          // switch (remote_unit_command_submitted) {
          //   case REMOTE_UNIT_AZ_COMMAND: debug.print("REMOTE_UNIT_AZ_COMMAND"); break;
          //   case REMOTE_UNIT_EL_COMMAND: debug.print("REMOTE_UNIT_EL_COMMAND"); break;
          //   case REMOTE_UNIT_OTHER_COMMAND: debug.print("REMOTE_UNIT_OTHER_COMMAND"); break;
          //   default: debug.print("UNDEFINED"); break;
          // }
          debug.print(" buffer_index:");
          debug.print(remote_unit_port_buffer_index);
          debug.print(" buffer: ");
          for (int x = 0; x < remote_unit_port_buffer_index; x++) {
            debug.write((char*)remote_unit_port_buffer[x]);
          }
          debug.println("$");
        #endif // DEBUG_SVC_REMOTE_COMM_INCOMING_BUFFER_BAD_DATA


        // remote_unit_command_results_available = 0;
        remote_unit_bad_results++;
      }
      // remote_unit_command_submitted = 0;

      remote_unit_port_buffer_carriage_return_flag = 0;
      remote_unit_port_buffer_index = 0;
    }

    // has a command timed out?
    // if ((remote_unit_command_submitted) && ((millis() - last_remote_unit_command_time) > REMOTE_UNIT_COMMAND_TIMEOUT_MS)){

    //   #if defined(FEATURE_CLOCK) && defined(OPTION_SYNC_MASTER_CLOCK_TO_SLAVE)
    //     if ((remote_unit_command_submitted == REMOTE_UNIT_CL_COMMAND) && ((clock_status == SLAVE_SYNC) || (clock_status == SLAVE_SYNC_GPS))){
    //       clock_status = FREE_RUNNING;
    //     }
    //   #endif //defined(FEATURE_CLOCK) && defined(OPTION_SYNC_MASTER_CLOCK_TO_SLAVE)

    //   remote_unit_command_timeouts++;
    //   remote_unit_command_submitted = 0;
    //   remote_unit_port_buffer_index = 0;
      
    // }

    // have characters been in the buffer for some time but no carriage return?
    if ((remote_unit_port_buffer_index) /*&& (!remote_unit_command_submitted)*/ && ((millis() - serial1_last_receive_time) > REMOTE_UNIT_COMMAND_TIMEOUT_MS)) {
      remote_unit_port_buffer_index = 0;
      remote_unit_incoming_buffer_timeouts++;
    }

  } /* service_remote_communications_incoming_buffer */

#endif // defined(FEATURE_MASTER_WITH_SERIAL_SLAVE) || defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE)


//------------------------------------------------------

#if defined(FEATURE_MASTER_WITH_SERIAL_SLAVE) || defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE) || defined(FEATURE_REMOTE_UNIT_SLAVE)
  void service_master_remote_link_state(){

    static unsigned long last_ping_send_time = 0;

    #if defined(FEATURE_MASTER_WITH_SERIAL_SLAVE) || defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE)

      if (((float)(millis() - (float)last_ping_send_time) > ((float)MASTER_REMOTE_LINK_PING_TIME_MS * (float)0.8)) || (master_remote_link_state == MASTER_REMOTE_LINK_DOWN)){
        submit_remote_command(REMOTE_UNIT_PM_COMMAND,0,0);
        last_ping_send_time = millis();
      }

      if ((millis() - last_ping_receive_time) < MASTER_REMOTE_LINK_PING_TIME_MS){
        master_remote_link_state = MASTER_REMOTE_LINK_UP;
      } else {
        master_remote_link_state = MASTER_REMOTE_LINK_DOWN;
      }

    #endif //defined(FEATURE_MASTER_WITH_SERIAL_SLAVE) || defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE)


    #if defined(FEATURE_REMOTE_UNIT_SLAVE)

      if (((float)(millis() - (float)last_ping_send_time) > ((float)MASTER_REMOTE_LINK_PING_TIME_MS * (float)0.8)) || (master_remote_link_state == MASTER_REMOTE_LINK_DOWN)){
        // submit_remote_command(REMOTE_UNIT_PM_COMMAND,0,0);
        last_ping_send_time = millis();
      }

      if ((millis() - last_ping_receive_time) < MASTER_REMOTE_LINK_PING_TIME_MS){
        master_remote_link_state = MASTER_REMOTE_LINK_UP;
      } else {
        master_remote_link_state = MASTER_REMOTE_LINK_DOWN;
      }


    #endif //defined(FEATURE_REMOTE_UNIT_SLAVE)


  }



#endif //defined(FEATURE_MASTER_WITH_SERIAL_SLAVE) || defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE)



// that's all, folks !
