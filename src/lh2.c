#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "system.h"
#include "deck.h"
#include "log.h"
#include "param.h"
#include "statsCnt.h"

#include "config.h"
#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "LH"
#include "debug.h"
#include "uart1.h"
#include "lh_bootloader.h"

#include "pulse_processor.h"
#include "lighthouse.h"

#include "estimator.h"
#include "estimator_kalman.h"

#define RADIANS(X) ((X/180.0f)*PI)


static baseStationGeometry_t lh2Geometry[2]  = {
{.origin = {-2.033134, 0.266750, 2.334647, }, .mat = {{0.783252, -0.011617, 0.621597, }, {-0.002057, 0.999772, 0.021277, }, {-0.621702, -0.017943, 0.783048, }, }},
{.origin = {0.012081, 1.541809, 2.274207, }, .mat = {{-0.013548, 0.999692, -0.020775, }, {-0.757842, -0.023820, -0.652004, }, {-0.652298, 0.006911, 0.757931, }, }},
};

#define STR2(x) #x
#define STR(x) STR2(x)

#define INCBIN(name, file) \
    __asm__(".section .rodata\n" \
            ".global incbin_" STR(name) "_start\n" \
            ".align 4\n" \
            "incbin_" STR(name) "_start:\n" \
            ".incbin \"" file "\"\n" \
            \
            ".global incbin_" STR(name) "_end\n" \
            ".align 1\n" \
            "incbin_" STR(name) "_end:\n" \
            ".byte 0\n" \
            ".align 4\n" \
            STR(name) "Size:\n" \
            ".int incbin_" STR(name) "_end - incbin_" STR(name) "_start\n" \
    ); \
    extern const __attribute__((aligned(4))) void* incbin_ ## name ## _start; \
    extern const void* incbin_ ## name ## _end; \
    extern const int name ## Size; \
    static const __attribute__((used)) unsigned char* name = (unsigned char*) & incbin_ ## name ## _start; \

INCBIN(bitstream, "blobs/lighthouse.bin");

typedef struct frame_s {
  bool sync;
  
  int sensor;
  
  // Envelope
  uint32_t timestamp;
  uint16_t width;

  // Raw beam data
  uint32_t beamData;

  // Decoded beam data
  uint32_t offset;
  bool modeFound;
  int mode;
  int slowbit;
} frame_t;

static bool getFrame(frame_t *frame)
{
  static char data[12];
  int nSync = 0;

  for(int i=0; i<12; i++) {
    uart1Getchar(&data[i]);
    if (data[i] == 0xff) {
      nSync += 1;
    }
  }

  memset(frame, 0, sizeof(*frame));

  frame->sync = (nSync == 12);

  memcpy(&frame->timestamp, &data[9], 3);
  memcpy(&frame->beamData, &data[6], 3);
  memcpy(&frame->offset, &data[3], 3);
  memcpy(&frame->width, &data[1], 2);
  frame->sensor = data[0] & 0x03;
  frame->modeFound = (data[0]&0x80) == 0;
  frame->mode = (data[0] >> 3) & 0x1f;
  frame->slowbit = (data[0]>>2)&0x01;
  
  return true;
}

static void checkVersionAndBoot()
{

  uint8_t bootloaderVersion = 0;
  lhblGetVersion(&bootloaderVersion);
  DEBUG_PRINT("Lighthouse bootloader version: %d\n", bootloaderVersion);

  // Wakeup mem
  lhblFlashWakeup();
  vTaskDelay(M2T(1));

  // Checking the bitstreams are identical
  // Also decoding bitstream version for console
  static char deckBitstream[65];
  lhblFlashRead(LH_FW_ADDR, 64, (uint8_t*)deckBitstream);
  deckBitstream[64] = 0;
  int deckVersion = strtol(&deckBitstream[2], NULL, 10);
  int embeddedVersion = strtol((char*)&bitstream[2], NULL, 10);

  bool identical = true;
  for (int i=0; i<=bitstreamSize; i+=64) {
    int length = ((i+64)<bitstreamSize)?64:bitstreamSize-i;
    lhblFlashRead(LH_FW_ADDR + i, length, (uint8_t*)deckBitstream);
    if (memcmp(deckBitstream, &bitstream[i], length)) {
      DEBUG_PRINT("Fail comparing firmware\n");
      identical = false;
      break;
    }
  }

  if (identical == false) {
    DEBUG_PRINT("Deck has version %d and we embeed version %d\n", deckVersion, embeddedVersion);
    DEBUG_PRINT("Updating deck with embedded version!\n");

    // Erase LH deck FW
    lhblFlashEraseFirmware();

    // Flash LH deck FW
    if (lhblFlashWriteFW((uint8_t*)bitstream, bitstreamSize)) {
      DEBUG_PRINT("FW updated [OK]\n");
    } else {
      DEBUG_PRINT("FW updated [FAILED]\n");
    }
  }

  // Launch LH deck FW
  DEBUG_PRINT("Firmware version %d verified, booting deck!\n", deckVersion);
  lhblBootToFW();
}



static void calculateAE(float firstBeam, float secondBeam, float* azimuth, float* elevation)
{
  *azimuth = ((firstBeam + secondBeam) / 2) - RADIANS(180);
  float p = RADIANS(60);
  float beta = (secondBeam - firstBeam) - RADIANS(120);
  *elevation = atanf(sinf(beta/2)/tanf(p/2));
}

static const uint32_t periods[] = {959000, 957000,
                                   953000, 949000,
                                   947000, 943000,
                                   941000, 939000,
                                   937000, 929000,
                                   919000, 911000,
                                   907000, 901000,
                                   893000, 887000};

static float angles[2][2];
static vec3d position;
float position_delta;
static positionMeasurement_t ext_pos;

static void lighthouseTask(void *param)
{
  bool synchronized = false;
  int syncCounter = 0;
  char c;
  static frame_t frame;
  static uint32_t lastMeasurements[16] = {0,};

  systemWaitStart();

  // Boot the deck firmware
  checkVersionAndBoot();

  while(1) {
    // Synchronize
    syncCounter = 0;
    while (!synchronized) {

      uart1Getchar(&c);
      if (c == 0xff) {
        syncCounter += 1;
      } else {
        syncCounter = 0;
      }
      synchronized = syncCounter == 12;
    }

    DEBUG_PRINT("Synchronized!\n");

    // Receive data until being desynchronized
    synchronized = getFrame(&frame);
    while(synchronized) {
      if (frame.sync) {
        synchronized = getFrame(&frame);
        continue;
      }

      if (frame.offset != 0) {
        if (frame.offset > (lastMeasurements[frame.mode] + 10000)) {
          float firstBeam = ((lastMeasurements[frame.mode] * 8.0f) / periods[frame.mode]) * 2 * PI;
          float secondBeam = ((frame.offset * 8.0f) / periods[frame.mode]) * 2 * PI;
          if (frame.mode <= 1) {
            calculateAE(firstBeam, secondBeam, &angles[frame.mode][0], &angles[frame.mode][1]);
            lighthouseGeometryGetPositionFromRayIntersection(lh2Geometry, angles[0], angles[1], position, &position_delta);

            ext_pos.x = position[0];
            ext_pos.y = position[1];
            ext_pos.z = position[2];
            ext_pos.stdDev = 0.01;
            estimatorEnqueuePosition(&ext_pos);
          }
        }
        lastMeasurements[frame.mode] = frame.offset;
      }

      synchronized = getFrame(&frame);
      if (frame.sync) {
        synchronized = getFrame(&frame);
        continue;
      }
    }
  }
}

static bool isInit = false;

static void lighthouseInit(DeckInfo *info)
{
  if (isInit) return;

  uart1Init(230400);
  lhblInit(I2C1_DEV);

  xTaskCreate(lighthouseTask, LIGHTHOUSE_TASK_NAME,
              2*configMINIMAL_STACK_SIZE, NULL, LIGHTHOUSE_TASK_PRI, NULL);

  isInit = true;
}


static const DeckDriver lighthouse_deck = {
  .vid = 0xBC,
  .pid = 0x10,
  .name = "bcLighthouse4",

  .usedGpio = 0,  // FIXME: set the used pins
  .requiredEstimator = kalmanEstimator,

  .init = lighthouseInit,
};

DECK_DRIVER(lighthouse_deck);

LOG_GROUP_START(lh2)
LOG_ADD(LOG_FLOAT, azimuth0, &angles[0][0])
LOG_ADD(LOG_FLOAT, elevation0, &angles[0][1])
LOG_ADD(LOG_FLOAT, azimuth1, &angles[1][0])
LOG_ADD(LOG_FLOAT, elevation1, &angles[1][1])
LOG_ADD(LOG_FLOAT, x, &position[0])
LOG_ADD(LOG_FLOAT, y, &position[1])
LOG_ADD(LOG_FLOAT, z, &position[2])
LOG_GROUP_STOP(lh2)
