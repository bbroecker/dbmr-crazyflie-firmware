/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2016 Bitcraze AB
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * locodeck.h: Dwm1000 deck driver.
 */

#ifndef __LOCODECK_H__
#define __LOCODECK_H__

#include "libdw1000.h"
#include "stabilizer_types.h"

#define SPEED_OF_LIGHT 299792458.0
// Timestamp counter frequency
#define LOCODECK_TS_FREQ (499.2e6 * 128)



typedef enum uwbEvent_e {
  eventTimeout,
  eventPacketReceived,
  eventPacketSent,
  eventReceiveTimeout,
  eventReceiveFailed,
  eventTokenHandling,
} uwbEvent_t;

#define LOCODECK_NR_OF_DEV 8

typedef uint64_t locoAddress_t;

typedef struct {
  const uint64_t antennaDelay;
  const int rangingFailedThreshold;

  const uint16_t myID;
  const locoAddress_t dw1000Addresses[LOCODECK_NR_OF_DEV];

  point_t anchorPosition[LOCODECK_NR_OF_DEV];
  bool anchorPositionOk;

  float distance[LOCODECK_NR_OF_DEV];
  float pressures[LOCODECK_NR_OF_DEV];
  int failedRanging[LOCODECK_NR_OF_DEV];
  volatile uint16_t rangingState;
} lpsAlgoOptions_t;

// Callback for one uwb algorithm
typedef struct uwbAlgorithm_s {
  void (*init)(dwDevice_t *dev, lpsAlgoOptions_t* options);
  uint32_t (*onEvent)(dwDevice_t *dev, uwbEvent_t event);
} uwbAlgorithm_t;

#include <FreeRTOS.h>

#define MAX_TIMEOUT portMAX_DELAY

#endif // __LOCODECK_H__
