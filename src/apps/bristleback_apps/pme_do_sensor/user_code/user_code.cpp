#include "user_code.h"
#include "FreeRTOS.h"
#include "OrderedSeparatorLineParser.h"
#include "app_pub_sub.h"
#include "array_utils.h"
#include "avgSampler.h"
#include "bm_os.h"
#include "bm_printf.h"
#include "bsp.h"
#include "configuration.h"
#include "debug.h"
#include "device_info.h"
#include "htuSampler.h"
#include "io.h"
#include "lwip/inet.h"
#include "payload_uart.h"
#include "pme_dissolved_oxygen_msg.h"
#include "pme_do_sensor.h"
#include "sensorSampler.h"
#include "serial.h"
#include "spotter.h"
#include "stm32_rtc.h"
#include "task_priorities.h"
#include "uptime.h"
#include "usart.h"
#include "util.h"
#include <ctime>
#include <inttypes.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>

#define LAST_WIPE_EPOCH_KEY "lastWipeEpochS";

/* TODO
- Add power switching to microDOT when sensing is not ready
- User/System config to allow customer to set measurement intervals and toggle spotter_log for uploading data
- Use cbor buffers to upload data rather than raw payloads (cbor works but not implemented yet)
- Comment code
- Remove or comment out debugging printf's
- Follow as bm_pub_wl is developed
- Pull barometric pressure from spotter to use for DOsat% calc
*/



static PmeSensor pmeSensor;

static char pmeDoTopic[BM_TOPIC_MAX_LEN]; // DO
static int pmeDoTopicStrLen;              // DO

static char pmeWipeTopic[BM_TOPIC_MAX_LEN]; // Wipe
static int pmeWipeTopicStrLen;              // Wipe

bool firstDo;

// Defines the max buffer size for the pme sensor message
static constexpr uint32_t pmeSensorDataMsgMaxSize = 256;

// Variables for measurements and timing
static uint32_t lastWipeEpochS = 0;
static uint64_t lastDoMeasurementUptimeSec = 0;
static uint32_t doIntervalSec = 600;
static uint32_t wipeIntervalSec = 14400;

// Function to create the topic string for pme DO measurement data
static int createPmeDoMeasurementDataTopic(void) { // DO measurement
  int topicStrLen = snprintf(pmeDoTopic, BM_TOPIC_MAX_LEN,
                             "sensor/%016" PRIx64 "/pme/pme_do_data", getNodeId());
  configASSERT(topicStrLen > 0 && topicStrLen < BM_TOPIC_MAX_LEN);
  return topicStrLen;
}

static int createPmeWipeDataTopic(void) { // Wipe
  int topicStrLen = snprintf(pmeWipeTopic, BM_TOPIC_MAX_LEN,
                             "sensor/%016" PRIx64 "/pme/pme_wipe_data", getNodeId());
  configASSERT(topicStrLen > 0 && topicStrLen < BM_TOPIC_MAX_LEN);
  return topicStrLen;
}

void debugTx(void) {}

void setup(void) {
  pmeSensor.init();
  pmeDoTopicStrLen = createPmeDoMeasurementDataTopic();
  pmeWipeTopicStrLen = createPmeWipeDataTopic();
  lastWipeEpochS = loadLastWipeEpoch();
  printf("Last wipe time: %lu\n", lastWipeEpochS);
  // Ensure Vbus stable before enabling Vout
  IOWrite(&BB_VBUS_EN, 0);
  bm_delay(50);
  // Turn off LEDs
  ledAllOff();
  //Set first DO measurement flag to true to trigger startup DO measurement
  firstDo = true;
}

void loop(void) {
  RTCTimeAndDate_t time_and_date = {};
  bool rtcIsSet = rtcGet(&time_and_date);
  uint64_t epochTimeSec = (rtcGetMicroSeconds(&time_and_date) / 1000000);
  //used for debugging RTC intervals
  // printf("&&& epochTimeSec: %llu\n", epochTimeSec);    
  uint64_t currentUptimeSec = uptimeGetMs() / 1000;
  // Debugging timers
  // printf("currentUptimeSec: %llu\n",currentUptimeSec);
  // printf("currentUptimeMs: %" PRIu64 ", uptimeGetMs: %" PRIu64 ", remainingWipeTime: %" PRIu64 ", remainingDoTime: %" PRIu64 "\n", currentUptimeMs, uptimeGetMs(), remainingWipeTime, remainingDoTime);
  // Perform a DO measurement
  static PmeDissolvedOxygenMsg::Data d;
  uint64_t elapsedDoSec = currentUptimeSec - lastDoMeasurementUptimeSec;
  if (elapsedDoSec >= doIntervalSec || firstDo) {
    
    if (firstDo) {
      firstDo = false; //Turn off initial DO flag
    }

    if (pmeSensor.getDoData(d)) {
      static uint8_t cborBuf[pmeSensorDataMsgMaxSize];
      size_t encodedLen = 0;
      if (PmeDissolvedOxygenMsg::encode(d, cborBuf, sizeof(cborBuf), &encodedLen) ==
          CborNoError) {
        printf("#  DOT Encoding success! | Topic: %s, cborBuf: %d, \n", pmeDoTopic,
               cborBuf); // Debugging
        bm_pub_wl(pmeDoTopic, pmeDoTopicStrLen, cborBuf, encodedLen, 0,
                  BM_COMMON_PUB_SUB_VERSION);
        //optional with config, todo
        if (true) {
          debugTx();
        }
      } else {
        printf("!  Failed to encode DOT measurement data message\n");
      }
    } else {
      printf("!  Failed to perform DOT measurement\n");
    }
    lastDoMeasurementUptimeSec = currentUptimeSec;
  } else {
    printf("DOT timer: %llu of %i seconds\n", elapsedDoSec, doIntervalSec);
  }

  ////////
  // Wipe
  ////////

  //debugging epoch timers
  // printf("&&&& lastwipetime %lu, epochtimesec %llu, wipeintervalsec %lu\n", lastWipeEpochS, epochTimeSec, wipeIntervalSec);

  if (rtcIsSet) {
    // Check if enough time has passed since the last wipe
    uint64_t elapsedWipe = epochTimeSec - lastWipeEpochS;
    if (elapsedWipe >= wipeIntervalSec) {
      // Perform a Wipe
      static PmeWipeMsg::Data w;
      saveLastWipeEpoch(epochTimeSec);
      lastWipeEpochS = epochTimeSec;
      if (pmeSensor.getWipeData(w)) {
        static uint8_t cborBuf[pmeSensorDataMsgMaxSize];
        size_t encodedLen = 0;
        if (true) {
          debugTx();
        }
        if (PmeWipeMsg::encode(w, cborBuf, sizeof(cborBuf), &encodedLen) == CborNoError) {
          bm_pub_wl(pmeDoTopic, pmeDoTopicStrLen, cborBuf, encodedLen, 0,
                    BM_COMMON_PUB_SUB_VERSION);
          printf("#  WIPE Encoding success! | Topic: %s, cborBuf: %d, \n", pmeDoTopic,
                 cborBuf); // Debugging
          //optional with config, todo
        } else {
          printf("!  Failed to encode WIPE data message\n");
        }
      }
    } else {
      printf("Wipe timer: %llu of %i seconds\n", elapsedWipe,
             wipeIntervalSec);
    }
  } else {
    printf("!  Not wiping; RTC is not set!\n");
  }
  bm_delay(1000);
}