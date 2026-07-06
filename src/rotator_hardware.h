/* rotator_hardware.h

   This fork targets a single board: the Arduino Nano ESP32. There is no
   hardware-profile picker here anymore (upstream supports many AVR/Teensy/etc.
   boards via HARDWARE_* profiles in this file; this fork does not).

*/

#if !defined(rotator_hardware_h)   // can't touch this
#define rotator_hardware_h         // can't touch this

// The Nano ESP32's "Serial" is native USB-CDC (class USBCDC), not a HardwareSerial UART.
#define CONTROL_PORT_SERIAL_PORT_CLASS USBCDC

#endif //!defined(rotator_hardware_h) stop.  hammer time.
