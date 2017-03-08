/* Arduino Slow Software I2C Master 
   Copyright (c) 2017 Bernhard Nebel.

   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public License
   as published by the Free Software Foundation; either version 3 of
   the License, or (at your option) any later version.

   This library is distributed in the hope that it will be useful, but
   WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with this library; if not, write to the Free Software
   Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301
   USA
*/
/* This is basically 90% new and re-written code as I made it work on ESP8266
   with Arduino, but being based on the version by Bernhard Nebel, I contribute it
   back to the open source community.  I have tested it with Digole 400x240 Touch LCD
   and Seeed Studio Grove Color Sensor running concurrently.  I had to create
   a detailed I2C state map in a spreadsheet to ensure perfect compliance with
   the I2C standard while ensuring compatibility with ESP8266 Arduino compatibility.
   Implements extreme clock stretching in a ESP8266 Arduino compatible way that
   ensures fastest I2C clocking while avoiding hogging the CPU or causing WDT resets.
   Could handle greater than 3 second timeout errors better and pass them through
   the companion SlowSoftWire library also by Bernhard Nebel, which I also forked
   and slightly modified for greater compatibility with other Arduino libraries.
   License terms LGPL continue.
   
   Copyright (c) 2017 Shannon Bailey.
*/
	
#include <SlowSoftI2CMaster.h>

#define SET_SDA_LOW() \
 pinMode(_sda, OUTPUT); \
 _sda_set = LOW;

#define SET_SCL_LOW() \
 pinMode(_scl, OUTPUT); \
 _scl_set = LOW;

#define SET_SDA_HIGH() \
 pinMode(_sda, INPUT); \
 _sda_set = HIGH;

#define SET_SCL_HIGH() \
 pinMode(_scl, INPUT); \
 _scl_set = HIGH;


SlowSoftI2CMaster::SlowSoftI2CMaster(uint8_t sda, uint8_t scl) {
  _sda = sda;
  _scl = scl;
}

// Init function. Needs to be called once in the beginning.
// Returns false if SDA or SCL are low, which probably means 
// a I2C bus lockup or that the lines are not pulled up.
bool SlowSoftI2CMaster::i2c_init(void) {

  SET_SCL_LOW();
  SET_SDA_LOW();
  digitalWrite(_scl, LOW);
  digitalWrite(_sda, LOW);
  SET_SDA_HIGH();
//  SET_SCL_HIGH();
  delayMicroseconds(DELAY);

  for (uint8_t i = 0; i < 28; i++) {
    SET_SCL_LOW();
    delayMicroseconds(DELAY);
    SET_SCL_HIGH();
    delayMicroseconds(DELAY);
  }
  i2c_stop();

  if ((digitalRead(_sda) == LOW) || (digitalRead(_scl) == LOW) )
    return false;
  return true;
}

// Start transfer function: <addr> is the 8-bit I2C address
// (including the R/W bit). 
// Return: true if the slave replies with an "acknowledge", false otherwise.
bool SlowSoftI2CMaster::i2c_start(uint8_t addr) {
  if (_scl_set == LOW) {
    if (_sda_set == HIGH) {
      delayMicroseconds(DELAY);
      SET_SCL_HIGH();
    } else {
      delayMicroseconds(DELAY);
      SET_SDA_HIGH();
      delayMicroseconds(DELAY);
      SET_SCL_HIGH();   
    }
  }
  delayMicroseconds(DELAY);
  SET_SDA_LOW();              // Start
  delayMicroseconds(DELAY);
  SET_SCL_LOW();              // Master Ready
  return i2c_write(addr);
}

// Repeated start function: After having claimed the bus with a start condition,
// you can address another or the same chip again without an intervening 
// stop condition.
// Return: true if the slave replies with an "acknowledge", false otherwise
bool SlowSoftI2CMaster::i2c_rep_start(uint8_t addr) {

  return i2c_start(addr);
}

// Issue a stop condition, freeing the bus.
void SlowSoftI2CMaster::i2c_stop(void) {
  if (_sda_set == HIGH) {
    if (_scl_set == HIGH) {
      SET_SCL_LOW();
      delayMicroseconds(DELAY);
      }
    SET_SDA_LOW();
    delayMicroseconds(DELAY);
  }
  if (_scl_set == LOW) {
    SET_SCL_HIGH();
    delayMicroseconds(DELAY);
  }
  SET_SDA_HIGH();               // Stop
  delayMicroseconds(DELAY * 2);
}

// Write one byte to the slave chip that had been addressed
// by the previous start call. <value> is the byte to be sent.
// Return: true if the slave replies with an "acknowledge", false otherwise
bool SlowSoftI2CMaster::i2c_write(uint8_t value) {
  for (uint8_t curr = 0X80; curr != 0; curr >>= 1) {
    if (curr & value) {
      SET_SDA_HIGH();
    } else {
      SET_SDA_LOW();
    }
    delayMicroseconds(DELAY);    
    SET_SCL_HIGH();
    delayMicroseconds(DELAY);

    if (ClockStretch() )
      return 0;

    SET_SCL_LOW();
  }

  // get Ack or Nak
  SET_SDA_HIGH();
  delayMicroseconds(DELAY);  
  SET_SCL_HIGH();
  delayMicroseconds(DELAY/2);

  if (ClockStretch() )
    return 0;

  uint8_t ack = digitalRead(_sda);

  SET_SCL_LOW();
  delayMicroseconds(DELAY);

  return ack == 0;
}

// Read one byte. If <last> is true, we send a NAK after having received 
// the byte in order to terminate the read sequence. 
uint8_t SlowSoftI2CMaster::i2c_read(bool last) {
  uint8_t value = 0;

  SET_SDA_HIGH();
  delayMicroseconds(DELAY);

  for (uint8_t i = 0; i < 8; i++) {
    value <<= 1;  
    SET_SCL_HIGH();
    delayMicroseconds(DELAY/2);

    if (ClockStretch() )
      return 0xFF;

    if (digitalRead(_sda))
       value |= 1;

    SET_SCL_LOW();
    delayMicroseconds(DELAY);
  }

  if (last) {
    SET_SDA_HIGH();
  } else {
    SET_SDA_LOW();
  }

  SET_SCL_HIGH();
  delayMicroseconds(DELAY);

  if (ClockStretch() )
    return 0xFF;

  SET_SCL_LOW();
  delayMicroseconds(DELAY);

  return value;
}

// Original clock stretch uses delay(1), but new code uses no delay for first 3 milliseconds
// providing immediate clock edge response within the first 3 milliseconds of clock stretch
// after which it reverts to delay(1) for three seconds, then times out
//  for (int x=0; (digitalRead(_scl) == LOW) && (x < 10000); x++) delay(1);
bool SlowSoftI2CMaster::ClockStretch() {
  unsigned long milliSeconds = 2 + millis();
  unsigned long milli3000    = 3000 + milliSeconds;

  while (digitalRead(_scl) == LOW) {  // provide tight clock stretch timing response for first 3 milliseconds
    if (milliSeconds > millis() ) {
      ;
    } else {
      if (milli3000 > millis() ) {
        delay(1); // start using delay(1) to give other tasks on ESP8266 time to run
      } else {
        return 1;  // after three seconds, just return so code exits
      }
    } 
  }          
  return 0;
}
