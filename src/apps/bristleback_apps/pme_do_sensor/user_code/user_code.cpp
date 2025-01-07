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


// app_main passes a handle to the config partitions in NVM.
extern cfg::Configuration *userConfigurationPartition;
extern cfg::Configuration *systemConfigurationPartition;

static PmeSensor pme_sensor;

//Defines the max buffer size for the pme sensor message (P.F.)
static constexpr u_int32_t PME_SENSOR_DATA_MSG_MAX_SIZE = 256;

// Variables for measurements and timing
static uint64_t currentUptimeMs = 0;
static uint64_t lastWipeTime = 0;
static uint64_t lastDoMeasurementTime = 0;

uint8_t DO_MEASUREMENT_INTERVAL_SEC = 10;
static constexpr uint32_t WIPE_INTERVAL_SEC = 60;

static char pme_do_topic[BM_TOPIC_MAX_LEN]; //DO
static int pme_do_topic_str_len; //DO

static char pme_wipe_topic[BM_TOPIC_MAX_LEN]; //Wipe
static int pme_wipe_topic_str_len; //Wipe


// Function to create the topic string for pme DO measurement data (P.F.)
static int createPmeDoMeasurementDataTopic(void) {    //DO measurement
  int topic_str_len = snprintf(pme_do_topic, BM_TOPIC_MAX_LEN,
                               "sensor/%016" PRIx64 "/pme/pme_do_data", getNodeId());
  configASSERT(topic_str_len > 0 && topic_str_len < BM_TOPIC_MAX_LEN);
  return topic_str_len;
}

static int createPmeWipeDataTopic(void) {    //Wipe
  int topic_str_len = snprintf(pme_wipe_topic, BM_TOPIC_MAX_LEN,
                               "sensor/%016" PRIx64 "/pme/pme_wipe_data", getNodeId());
  configASSERT(topic_str_len > 0 && topic_str_len < BM_TOPIC_MAX_LEN);
  return topic_str_len;
}

void setup(void) {
  //Perform pme sensor setup which includes PLUART setup (P.F.)
  configASSERT(systemConfigurationPartition);
  pme_sensor.init();
  pme_do_topic_str_len = createPmeDoMeasurementDataTopic();
  pme_wipe_topic_str_len = createPmeWipeDataTopic();
  
  IOWrite(&BB_VBUS_EN, 0);
  // ensure Vbus stable before enable Vout with a 5ms delay.
  vTaskDelay(pdMS_TO_TICKS(5));
  // enable Vout, 12V by default.
  IOWrite(&BB_PL_BUCK_EN, 0);
  // turn off LEDs
  IOWrite(&LED_BLUE, 0);
  IOWrite(&LED_GREEN, 0);
  IOWrite(&LED_RED, 0);
}


void loop(void) {
  currentUptimeMs = uptimeGetMs();
  uint64_t remainingDoTime = (currentUptimeMs - lastDoMeasurementTime);
  uint64_t remainingWipeTime = (currentUptimeMs - lastWipeTime);
  printf("currentUptimeMs: %llu, uptimeGetMs: %llu, remainingWipeTime: %llu, remainingDOTime: %llu\n", currentUptimeMs, uptimeGetMs(), remainingWipeTime, remainingDoTime);
  // ################
  // ###    DO    ###
  // ################
  // Perform a DO measurement
  static PmeDissolvedOxygenMsg::Data d;
  if (remainingDoTime >= (DO_MEASUREMENT_INTERVAL_SEC * 1000)) {
    if (pme_sensor.getDoData(d)) {
      static uint8_t cbor_buf[PME_SENSOR_DATA_MSG_MAX_SIZE];
      size_t encoded_len = 0;
      if (PmeDissolvedOxygenMsg::encode(d, cbor_buf, sizeof(cbor_buf), &encoded_len) == CborNoError) {
        bm_pub_wl(pme_do_topic, pme_do_topic_str_len, cbor_buf, encoded_len, 0);
        printf("### DO Encoding success! | Topic: %s, cbor_buf: %d, \n", pme_do_topic, cbor_buf); //debugging 
      }
      else {
        printf("Failed to encode DO measurement data message\n");
      }
    } 
    else {
      printf("Failed to perform DO measurement\n");
    }
    lastDoMeasurementTime = currentUptimeMs;
  }
  else {
    printf("DO interval not yet reached... %llu of %llu seconds\n", (remainingDoTime/1000), DO_MEASUREMENT_INTERVAL_SEC);
  }

// #####################
// ###    Wiping     ###
// #####################
// Check if enough time has passed since the last wipe
if (remainingWipeTime >= (WIPE_INTERVAL_SEC * 1000)) {
    // Perform a Wipe
    static PmeWipeMsg::Data w;
    pme_sensor.getWipeData(w);
    static uint8_t cbor_buf[PME_SENSOR_DATA_MSG_MAX_SIZE];
    size_t encoded_len = 0;
        if (PmeWipeMsg::encode(w, cbor_buf, sizeof(cbor_buf), &encoded_len) == CborNoError) {
            bm_pub_wl(pme_do_topic, pme_do_topic_str_len, cbor_buf, encoded_len, 0);
            printf("### WIPE Encoding success! | Topic: %s, cbor_buf: %d, \n", pme_do_topic, cbor_buf); //debugging 
        } 
        else {
            printf("Failed to encode WIPE data message\n");
        }
    lastWipeTime = currentUptimeMs;
} 
else {
    printf("Wipe interval not yet reached... %llu of %llu seconds\n", (remainingWipeTime/1000), WIPE_INTERVAL_SEC);}

vTaskDelay(10000);
}