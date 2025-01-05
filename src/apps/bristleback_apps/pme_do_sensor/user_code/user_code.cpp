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
//extern cfg::Configuration *hardwareConfigurationPartition;
static PmeSensor pme_sensor;

//holds the unix eopch time of the last wipe (P.F.)
// changed from int to float to match member function in configuration.h (B.H)
//static float lastWipeTime = 0;
//Line terminator for userConfigurationPartition (B.H.)
//static u_int32_t line_term_config = 13; // Carraige  Return, CR, 0x0D

// Define DO measurement interval (e.g., 600 seconds = 10 minutes)
static constexpr uint32_t DO_MEASUREMENT_INTERVAL_SEC = 600;

// Variable to store the last DO measurement time (in Unix epoch seconds)
// static uint64_t lastDoMeasurementTime = 0;


//Defines the max buffer size for the pme sensor message (P.F.)
static constexpr u_int32_t PME_SENSOR_DATA_MSG_MAX_SIZE = 256;

//Defines variables used in topic generation (P.F.)
static char pme_do_topic[BM_TOPIC_MAX_LEN]; //DO
static int pme_do_topic_str_len; //DO

// static char pme_wipe_topic[BM_TOPIC_MAX_LEN]; //Wipe
// static int pme_wipe_topic_str_len; //Wipe


// Function to create the topic string for pme DO measurement data (P.F.)
static int createPmeDoMeasurementDataTopic(void) {    //DO measurement
  int topic_str_len = snprintf(pme_do_topic, BM_TOPIC_MAX_LEN,
                               "sensor/%016" PRIx64 "/pme/pme_do_data", getNodeId());
  configASSERT(topic_str_len > 0 && topic_str_len < BM_TOPIC_MAX_LEN);
  return topic_str_len;
}
/*
static int createPmeWipeDataTopic(void) {    //Wipe
  int topic_str_len = snprintf(pme_wipe_topic, BM_TOPIC_MAX_LEN,
                               "sensor/%016" PRIx64 "/pme/pme_wipe_data", getNodeId());
  configASSERT(topic_str_len > 0 && topic_str_len < BM_TOPIC_MAX_LEN);
  return topic_str_len;
}
*/

void setup(void) {
  /* USER ONE-TIME SETUP CODE GOES HERE */
  // Retrieve user-set config values out of NVM.
  //hardwareConfigurationPartition->getConfig("plUartBaudRate", strlen("plUartBaudRate"), lastWipeTime);
  //userConfigurationPartition->getConfig("plUartLineTerm", strlen("plUartLineTerm"), line_term_config);
  
  //Perform pme sensor setup which includes PLUART setup (P.F.)
  pme_sensor.init();
  pme_do_topic_str_len = createPmeDoMeasurementDataTopic();
  
  IOWrite(&BB_VBUS_EN, 0);
  // ensure Vbus stable before enable Vout with a 5ms delay.
  vTaskDelay(pdMS_TO_TICKS(5));
  // enable Vout, 12V by default.
  IOWrite(&BB_PL_BUCK_EN, 0);
  // turn off LEDs
  IOWrite(&LED_BLUE, 0);
  IOWrite(&LED_GREEN, 0);
  IOWrite(&LED_RED, 0);
  // Write code to request sensor S/N and save to config 
  /*trigger Initial DO measurement*/
  static PmeDissolvedOxygenMsg::Data d;
  if (pme_sensor.getDoData(d)) {
    static uint8_t cbor_buf[PME_SENSOR_DATA_MSG_MAX_SIZE];
    size_t encoded_len = 0;
    if (PmeDissolvedOxygenMsg::encode(d, cbor_buf, sizeof(cbor_buf), &encoded_len) == CborNoError) {
      bm_pub_wl(pme_do_topic, pme_do_topic_str_len, cbor_buf, encoded_len, 0);
    } 
    else {
      printf("Failed to encode DO measurement data message\n");
    }
  }
  /*record RTC at time of measurement as latest DO measurement time*/
}


void loop(void) {
  // timing testing
  // Get the current RTC time
  // RTCTimeAndDate_t time_and_date = {};
  // rtcGet(&time_and_date);
  // char rtcTimeBuffer[32];
  // rtcPrint(rtcTimeBuffer, NULL);
  // bm_fprintf(0, "payload_data.log", USE_TIMESTAMP, "tick: %llu, rtc: %s\n", uptimeGetMs(), rtcTimeBuffer);
  // // Convert RTC time to Unix epoch
  // uint64_t currentUnixTime = rtcGetMicroSeconds(&time_and_date);
  // Check if enough time has passed since the last DO measurement
  // if (currentUnixTime - lastDoMeasurementTime >= DO_MEASUREMENT_INTERVAL_SEC) {
  // Update the last measurement time
  // lastDoMeasurementTime = currentUnixTime;
  // printf("DO measurement performed and published at %u (Unix time)\n", currentUnixTime);
  // Perform a DO measurement
    static PmeDissolvedOxygenMsg::Data d;
    if (pme_sensor.getDoData(d)) {
      static uint8_t cbor_buf[PME_SENSOR_DATA_MSG_MAX_SIZE];
      size_t encoded_len = 0;
      if (PmeDissolvedOxygenMsg::encode(d, cbor_buf, sizeof(cbor_buf), &encoded_len) == CborNoError) {
        bm_pub_wl(pme_do_topic, pme_do_topic_str_len, cbor_buf, encoded_len, 0);
        // printf("DO encoding performed and published!\n"); //debugging
        printf("### DO Encoding success! | Topic: %s, cbor_buf: %d, \n", pme_do_topic, cbor_buf); //debugging  
      }
      else {
        printf("Failed to encode DO measurement data message\n");
      }
    } 
    else {
      printf("Failed to perform DO measurement\n");
    }
  //}
  //If time to perform a DO measurement
  //Compare last DO measurement time to current time. If difference is greater than DO interval, perform DO measurement.

  //If time to perform a Wipe
  //Retrieve last wipe time from NVM and compare it to current time. If difference is greater than wipe interval, perform wipe.
  // static PmeWipeMsg::Data w;
  // if (pme_sensor.getWipeData(w)) {
  //   static uint8_t cbor_buf[PME_SENSOR_DATA_MSG_MAX_SIZE];
  //   size_t encoded_len = 0;
  //   if (PmeWipeMsg::encode(w, cbor_buf, sizeof(cbor_buf), &encoded_len) == CborNoError) {
  //     bm_pub_wl(pme_wipe_topic, pme_wipe_topic_str_len, cbor_buf, encoded_len, 0);
  //   } else {
  //     printf("Failed to encode Wipe data message\n");
  //   }
  // }
  vTaskDelay(10000);
}