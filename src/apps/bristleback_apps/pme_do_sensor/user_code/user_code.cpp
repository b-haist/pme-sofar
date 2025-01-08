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

#define LAST_WIPE_EPOCH_KEY "lastWipeEpochS"
#define LAST_DO_EPOCH_KEY "lastDoEpochS"

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

void saveLastWipeEpoch(uint64_t lastWipeEpochS) {
    systemConfigurationPartition->setConfig(LAST_WIPE_EPOCH_KEY, strlen(LAST_WIPE_EPOCH_KEY), reinterpret_cast<const uint8_t*>(&lastWipeEpochS), sizeof(lastWipeEpochS));
    systemConfigurationPartition->saveConfig(false);
}

uint64_t loadLastWipeEpoch() {
    uint64_t lastWipeEpochS = 0;
    size_t valueLen = sizeof(lastWipeEpochS);
    if (!systemConfigurationPartition->getConfig(LAST_WIPE_EPOCH_KEY, strlen(LAST_WIPE_EPOCH_KEY), reinterpret_cast<uint8_t*>(&lastWipeEpochS), valueLen)) {
        printf("LAST_WIPE_EPOCH_KEY not found. Initializing to 0.\n");
        saveLastWipeEpoch(0); // Save default value
    }
    return lastWipeEpochS;
}

void saveLastDoEpoch(uint64_t lastDoEpochS) {
    systemConfigurationPartition->setConfig(LAST_DO_EPOCH_KEY, strlen(LAST_DO_EPOCH_KEY), reinterpret_cast<const uint8_t*>(&lastDoEpochS), sizeof(lastDoEpochS));
    systemConfigurationPartition->saveConfig(false);
}

uint64_t loadLastDoEpoch() {
    uint64_t lastDoEpochS = 0;
    size_t valueLen = sizeof(lastDoEpochS);
    if (!systemConfigurationPartition->getConfig(LAST_DO_EPOCH_KEY, strlen(LAST_DO_EPOCH_KEY), reinterpret_cast<uint8_t*>(&lastDoEpochS), valueLen)) {
        printf("LAST_DO_EPOCH_KEY not found. Initializing to 0.\n");
        saveLastDoEpoch(0); // Save default value
    }
    return lastDoEpochS;
}




void setup(void) {
    configASSERT(systemConfigurationPartition);
    pmeSensor.init();

    pmeDoTopicStrLen = createPmeDoMeasurementDataTopic();
    pmeWipeTopicStrLen = createPmeWipeDataTopic();
    
    // Load last DO and wipe epochs
    lastDoMeasurementTime = loadLastDoEpoch();
    lastWipeTime = loadLastWipeEpoch();

    // Debugging
    printf("Loaded last DO measurement time: %llu\n", lastDoMeasurementTime);
    printf("Loaded last wipe time: %llu\n", lastWipeTime);

    // Ensure Vbus stable before enabling Vout
    IOWrite(&BB_VBUS_EN, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    IOWrite(&BB_PL_BUCK_EN, 0);

    // Turn off LEDs
    IOWrite(&LED_BLUE, 0);
    IOWrite(&LED_GREEN, 0);
    IOWrite(&LED_RED, 0);
}




void loop(void) {
  printf("last do epoch: %llu\n", loadLastDoEpoch());
  printf("last wipe epoch: %llu\n", loadLastWipeEpoch());
  currentUptimeS = uptimeGetMs()/1000.0;
  float remainingDoTime = (currentUptimeS - lastDoMeasurementTime);
  float remainingWipeTime = (currentUptimeS - lastWipeTime);
  // printf("currentUptimeMs: %" PRIu64 ", uptimeGetMs: %" PRIu64 ", remainingWipeTime: %" PRIu64 ", remainingDoTime: %" PRIu64 "\n", currentUptimeMs, uptimeGetMs(), remainingWipeTime, remainingDoTime);
  // Perform a DO measurement
  static PmeDissolvedOxygenMsg::Data d; 
  if (remainingDoTime >= doMeasurementIntervalSec) {
    RTCTimeAndDate_t time_and_date = {};
    rtcGet(&time_and_date);
    saveLastDoEpoch(rtcGetMicroSeconds(&time_and_date) / 1000);
    if (pmeSensor.getDoData(d)) {
      static uint8_t cborBuf[pmeSensorDataMsgMaxSize];
      size_t encodedLen = 0;
      if (PmeDissolvedOxygenMsg::encode(d, cborBuf, sizeof(cborBuf), &encodedLen) == CborNoError) {
        bm_pub_wl(pmeDoTopic, pmeDoTopicStrLen, cborBuf, encodedLen, 0);
        printf("### DO Encoding success! | Topic: %s, cborBuf: %d, \n", pmeDoTopic, cborBuf); // Debugging 
      } 
      else {
        printf("Failed to encode DO measurement data message\n");
      }
    } 
    else {
      printf("Failed to perform DO measurement\n");
    }
    lastDoMeasurementTime = currentUptimeS;
  } 
  else {
    printf("DO interval not yet reached... %.1f of %i seconds\n", remainingDoTime, doMeasurementIntervalSec);
  }

  // Check if enough time has passed since the last wipe
  if (remainingWipeTime >= wipeIntervalSec) {
    // Perform a Wipe
    static PmeWipeMsg::Data w;
    RTCTimeAndDate_t time_and_date = {};
    rtcGet(&time_and_date);
    saveLastWipeEpoch(rtcGetMicroSeconds(&time_and_date) / 1000);
    if(pmeSensor.getWipeData(w)) {
      static uint8_t cborBuf[pmeSensorDataMsgMaxSize];
      size_t encodedLen = 0;
      if (PmeWipeMsg::encode(w, cborBuf, sizeof(cborBuf), &encodedLen) == CborNoError) {
        bm_pub_wl(pmeDoTopic, pmeDoTopicStrLen, cborBuf, encodedLen, 0);
        printf("### WIPE Encoding success! | Topic: %s, cborBuf: %d, \n", pmeDoTopic, cborBuf); // Debugging 
      } 
      else {
        printf("Failed to encode WIPE data message\n");
      }
    }
    lastWipeTime = currentUptimeS;
  } 
  else {
    printf("Wipe interval not yet reached... %.1f of %i seconds\n", remainingWipeTime, wipeIntervalSec);
  }
  vTaskDelay(1000);
}