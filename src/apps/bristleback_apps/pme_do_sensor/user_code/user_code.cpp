#include "user_code.h"
#include "configuration.h"
#include "bm_network.h"
#include "bm_printf.h"
#include "bm_pubsub.h"
#include "bsp.h"
#include <ctime>
#include "debug.h"
#include "lwip/inet.h"
#include "payload_uart.h"
#include "stm32_rtc.h"
#include "task_priorities.h"
#include "uptime.h"
#include "usart.h"
#include "util.h"
#include "sensorSampler.h"
#include "avgSampler.h"
#include "array_utils.h"
#include "app_pub_sub.h"
#include "htuSampler.h"
#include <stdint.h>
#include <string.h>
#include "pme_do_sensor.h"
#include "FreeRTOS.h"
#include "serial.h" 
#include "OrderedSeparatorLineParser.h"
#include "device_info.h"
#include "io.h"
#include <sys/types.h>
#include <ctime>
#include "pme_dissolved_oxygen_msg.h"
#include <inttypes.h>

// app_main passes a handle to the config partitions in NVM.
extern cfg::Configuration *userConfigurationPartition;
extern cfg::Configuration *systemConfigurationPartition;

static PmeSensor pmeSensor;

// Defines the max buffer size for the pme sensor message (P.F.)
static constexpr uint32_t pmeSensorDataMsgMaxSize = 256;

// Variables for measurements and timing
static float currentUptimeS = 0;
static uint64_t lastWipeTime = 0;
static uint64_t lastDoMeasurementTime = 0;

static uint8_t doMeasurementIntervalSec = 10;
static uint8_t wipeIntervalSec = 60;

static char pmeDoTopic[BM_TOPIC_MAX_LEN]; // DO
static int pmeDoTopicStrLen; // DO

static char pmeWipeTopic[BM_TOPIC_MAX_LEN]; // Wipe
static int pmeWipeTopicStrLen; // Wipe

// Function to create the topic string for pme DO measurement data (P.F.)
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

void setup(void) {
  // Perform pme sensor setup which includes PLUART setup (P.F.)
  configASSERT(systemConfigurationPartition);
  pmeSensor.init();
  pmeDoTopicStrLen = createPmeDoMeasurementDataTopic();
  pmeWipeTopicStrLen = createPmeWipeDataTopic();
  
  IOWrite(&BB_VBUS_EN, 0);
  // Ensure Vbus stable before enable Vout with a 5ms delay.
  vTaskDelay(pdMS_TO_TICKS(5));
  // Enable Vout, 12V by default.
  IOWrite(&BB_PL_BUCK_EN, 0);
  // Turn off LEDs
  IOWrite(&LED_BLUE, 0);
  IOWrite(&LED_GREEN, 0);
  IOWrite(&LED_RED, 0);
}

void loop(void) {
  currentUptimeS = uptimeGetMs()/1000.0;
  float remainingDoTime = (currentUptimeS - lastDoMeasurementTime);
  float remainingWipeTime = (currentUptimeS - lastWipeTime);
  // printf("currentUptimeMs: %" PRIu64 ", uptimeGetMs: %" PRIu64 ", remainingWipeTime: %" PRIu64 ", remainingDoTime: %" PRIu64 "\n", currentUptimeMs, uptimeGetMs(), remainingWipeTime, remainingDoTime);
  // Perform a DO measurement
  static PmeDissolvedOxygenMsg::Data d;
  if (remainingDoTime >= doMeasurementIntervalSec) {
    if (pmeSensor.getDoData(d)) {
      static uint8_t cborBuf[pmeSensorDataMsgMaxSize];
      size_t encodedLen = 0;
      if (PmeDissolvedOxygenMsg::encode(d, cborBuf, sizeof(cborBuf), &encodedLen) == CborNoError) {
        bm_pub_wl(pmeDoTopic, pmeDoTopicStrLen, cborBuf, encodedLen, 0);
        printf("### DO Encoding success! | Topic: %s, cborBuf: %d, \n", pmeDoTopic, cborBuf); // Debugging 
      } else {
        printf("Failed to encode DO measurement data message\n");
      }
    } else {
      printf("Failed to perform DO measurement\n");
    }
    lastDoMeasurementTime = currentUptimeS;
  } else {
    printf("DO interval not yet reached... %.1f of %i seconds\n", remainingDoTime, doMeasurementIntervalSec);
  }

  // Check if enough time has passed since the last wipe
  if (remainingWipeTime >= wipeIntervalSec) {
    // Perform a Wipe
    static PmeWipeMsg::Data w;
    pmeSensor.getWipeData(w);
    static uint8_t cborBuf[pmeSensorDataMsgMaxSize];
    size_t encodedLen = 0;
    if (PmeWipeMsg::encode(w, cborBuf, sizeof(cborBuf), &encodedLen) == CborNoError) {
      bm_pub_wl(pmeDoTopic, pmeDoTopicStrLen, cborBuf, encodedLen, 0);
      printf("### WIPE Encoding success! | Topic: %s, cborBuf: %d, \n", pmeDoTopic, cborBuf); // Debugging 
    } else {
      printf("Failed to encode WIPE data message\n");
    }
    lastWipeTime = currentUptimeS;
  } else {
    printf("Wipe interval not yet reached... %.1f of %i seconds\n", remainingWipeTime, wipeIntervalSec);
  }
  vTaskDelay(1000);
}