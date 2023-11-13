/*
  Copyright (c) 2016 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA


  Modified by Erik K. Nyquist for the Arduino Zero Stage Metronome project. These
  changes were found to be needed in order to ensure clean playback of metronome
  beep sounds without any unpleasant pops/clicks. Specifically, disabling the I2S
  hardware via 'i2sd.disable()' after all samples for a sound have been sent
  seemed to eliminate unwanted artifacts.

  * Renamed I2S class to ModifiedI2S so as not to be confused with original I2S lib
  * Added ModifiedI2S.enable(), ModifiedI2S.disable(), and
    ModifiedI2S.remainingBytesToTransmit() functions.
*/

#ifndef _I2S_H_INCLUDED
#define _I2S_H_INCLUDED

#include <Arduino.h>

#include "I2SDoubleBuffer.h"

#define I2S_HAS_SET_BUFFER_SIZE 1

typedef enum {
  I2S_PHILIPS_MODE,
  I2S_RIGHT_JUSTIFIED_MODE,
  I2S_LEFT_JUSTIFIED_MODE
} i2s_mode_t;

class ModifiedI2SClass : public Stream
{
public:
  // the device index and pins must map to the "COM" pads in Table 6-1 of the datasheet 
  ModifiedI2SClass(uint8_t deviceIndex, uint8_t clockGenerator, uint8_t sdPin, uint8_t sckPin, uint8_t fsPin);

  // the SCK and FS pins are driven as outputs using the sample rate
  int begin(int mode, long sampleRate, int bitsPerSample);
  // the SCK and FS pins are inputs, other side controls sample rate
  int begin(int mode, int bitsPerSample);
  void end();

  // from Stream
  virtual int available();
  virtual int read();
  virtual int peek();
  virtual void flush();

  // from Print
  virtual size_t write(uint8_t);
  virtual size_t write(const uint8_t *buffer, size_t size);

  virtual int availableForWrite();

  int read(void* buffer, size_t size);

  size_t write(int);
  size_t write(int32_t);
  size_t write(const void *buffer, size_t size);

  void onTransmit(void(*)(void));
  void onReceive(void(*)(void));

  void setBufferSize(int bufferSize);

  void enable();
  void disable();
  int remainingBytesToTransmit();

private:
  int begin(int mode, long sampleRate, int bitsPerSample, bool driveClock);

  void enableClock(int divider);
  void disableClock();

  void enableTransmitter();
  void enableReceiver();

  void onTransferComplete(void);

  static void onDmaTransferComplete(int);

private:
  typedef enum {
    I2S_STATE_IDLE,
    I2S_STATE_TRANSMITTER,
    I2S_STATE_RECEIVER
  } i2s_state_t;

  static int _beginCount;

  uint8_t _deviceIndex;
  uint8_t _clockGenerator;
  uint8_t _sdPin;
  uint8_t _sckPin;
  uint8_t _fsPin;

  i2s_state_t _state;
  int _dmaChannel;
  int _bitsPerSample;

  volatile bool _dmaTransferInProgress;
  volatile bool _enabled;
  I2SDoubleBuffer _doubleBuffer;

  void (*_onTransmit)(void);
  void (*_onReceive)(void);
};

// "I2S" is already defined by the CMSIS device, undefine it so the ModifiedI2SClass
// instance can be called I2S
#undef I2S

#if I2S_INTERFACES_COUNT > 0
extern ModifiedI2SClass ModifiedI2S;
#else
#error "I2S is not supported on your board!"
#endif

#endif
