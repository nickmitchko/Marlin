/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifdef __AVR__

#include "../../inc/MarlinConfig.h"
#include "HAL.h"

// ------------------------
// Public Variables
// ------------------------

//uint8_t MCUSR;

// ------------------------
// Public functions
// ------------------------

void HAL_init() {
  // Init Servo Pins
  #define INIT_SERVO(N) OUT_WRITE(SERVO##N##_PIN, LOW)
  #if HAS_SERVO_0
    INIT_SERVO(0);
  #endif
  #if HAS_SERVO_1
    INIT_SERVO(1);
  #endif
  #if HAS_SERVO_2
    INIT_SERVO(2);
  #endif
  #if HAS_SERVO_3
    INIT_SERVO(3);
  #endif
}

// Jan 30 2020 -- implemented digitalRead and digitalWrite which will vastly improve the performance of the HAL layer here
// Warning -- This is deemed unsafe :) make sure you have the correct #Defines for these to take effect :)
#ifdef __FAST_DIGITAL_OPS

/**
 * digitalRead -- Faster implementation which will save a few cycles for reads and writes
 *                Please note that this is only for Arduino platforms with AVR
 * See original Arduino AVR implementation here: https://github.com/arduino/ArduinoCore-avr/blob/master/cores/arduino/wiring_digital.c#L165
 */
void digitalRead(uint8_t pin) {
	uint8_t bit = digitalPinToBitMask(pin);
	uint8_t port = digitalPinToPort(pin);
  if (*portInputRegister(port) & bit) return HIGH;
	return LOW;
}

/**
 * digitalWrite -- Faster implementation which will save a few cycles for writes
 *                Please note that this is only for Arduino platforms with AVR
 * See original Arduino AVR implementation here: https://github.com/arduino/ArduinoCore-avr/blob/master/cores/arduino/wiring_digital.c#L138
 */
void digitalWrite(uint8_t pin, uint8_t val) {
  // Skip the pin to timer, Skip the pin to port
  uint8_t port = digitalPinToPort(pin);
  volatile uint8_t *out = portOutputRegister(port);
  
  uint8_t oldSREG = SREG;
  __asm__ __volatile__ ("cli");
  if (val == LOW) {
	 *out &= ~bit;
	} else {
	 *out |= bit;
	}
  SREG = oldSREG;
  
}

#endif

#if ENABLED(SDSUPPORT)

  #include "../../sd/SdFatUtil.h"
  int freeMemory() { return SdFatUtil::FreeRam(); }

#else // !SDSUPPORT

extern "C" {
  extern char __bss_end;
  extern char __heap_start;
  extern void* __brkval;

  int freeMemory() {
    int free_memory;
    if ((int)__brkval == 0)
      free_memory = ((int)&free_memory) - ((int)&__bss_end);
    else
      free_memory = ((int)&free_memory) - ((int)__brkval);
    return free_memory;
  }
}

#endif // !SDSUPPORT

#endif // __AVR__
