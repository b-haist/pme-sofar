#include "user_code.h"
#include "bm_network.h"
#include "bm_printf.h"
#include "bm_pubsub.h"
#include "bsp.h"
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
#include "configuration.h"
#include "serial.h" 
#include "device_info.h"

#define LED_ON_TIME_MS 50
#define LED_PERIOD_MS 30000
#define DEFAULT_BAUD_RATE 9600
#define DEFAULT_LINE_TERM 13 // Carraige Return, CR, 0x0D
#define BYTES_CLUSTER_MS 50  // used for console printing convenience
#define DEFAULT_UART_MODE MODE_RS232
#define DEFAULT_SENSOR_AGG_PERIOD_MIN (0.5) // let's start with 30s for quick testing
// Converted to milliseconds.
#define SENSOR_AGG_PERIOD_MS ((double)DEFAULT_SENSOR_AGG_PERIOD_MIN * 60.0 * 1000.0)
// How long should our sample buffer arrays be?
// Assume we have a 2s polling period.
#define MAX_SENSOR_SAMPLES (((uint64_t)SENSOR_AGG_PERIOD_MS / 2000))


// Structure to store aggregated statistics from one of our sensors.
typedef struct {
  uint16_t sample_count;
  float mean;
  float stdev;
} __attribute__((__packed__)) sensorStatAgg_t;



// app_main passes a handle to the config partitions in NVM.
extern cfg::Configuration *userConfigurationPartition;
extern cfg::Configuration *systemConfigurationPartition;

// variables to store configurations retrieved from NVM
//static u_int32_t baud_rate_config = DEFAULT_BAUD_RATE;
//static u_int32_t line_term_config = DEFAULT_LINE_TERM;
//static u_int32_t bm_log_enable = true;
// Publish once per minute
static constexpr uint32_t PUBLISH_PERIOD_MS = (60000);
// The topic type is a identifier for the topic to encode different data types, we don't need to worry about this value for the tutorial,
// other than that it must be the same for the publisher and subscriber.
static constexpr uint32_t EXAMPLE_PUBLISH_TOPIC_TYPE = (1);
// The topic version is a version number for the topic, in case things change we don't need to worry about this value for the tutorial,
// other than that it must be the same for the publisher and subscriber.
static constexpr uint32_t EXAMPLE_PUBLISH_TOPIC_VERSION = (1);
// This is the topic to publish to, the subscriber will need to know this topic to receive the data (see the application sub_example)
static const char *const EXAMPLE_PUBLISH_TOPIC = "pubsub_example";
// static AveragingSampler oxygen_stats;
// static AveragingSampler temp_stats;

// A timer variable we can set to trigger a pulse on LED2 when we get payload serial data
//static int32_t ledLinePulse = -1;

// A buffer to put Rx data from our payload sensor into.
char payload_buffer[2048];


//Seapoint implementation
static constexpr uint32_t BM_SEAPOINT_TURBIDITY_DATA_MSG_MAX_SIZE = 256;

extern cfg::Configuration *systemConfigurationPartition;
static SeapointTurbiditySensor seapoint_turbidity_sensor;
static char seapoint_turbidity_topic[BM_TOPIC_MAX_LEN];
static int seapoint_turbidity_topic_str_len;

static int createSeapointTurbidityDataTopic(void) {
  int topic_str_len = snprintf(seapoint_turbidity_topic, BM_TOPIC_MAX_LEN,
                               "sensor/%016" PRIx64 "/sofar/seapoint_turbidity_data", getNodeId());
  configASSERT(topic_str_len > 0 && topic_str_len < BM_TOPIC_MAX_LEN);
  return topic_str_len;
}

void setup(void) {
  configASSERT(systemConfigurationPartition);
  seapoint_turbidity_sensor.init();
  seapoint_turbidity_topic_str_len = createSeapointTurbidityDataTopic();
  IOWrite(&BB_VBUS_EN, 0);
  vTaskDelay(pdMS_TO_TICKS(500)); // Wait for Vbus to stabilize
  IOWrite(&BB_PL_BUCK_EN, 0);
}

void loop(void) {
  // Read and handle line from sensor
  static BmSeapointTurbidityDataMsg::Data d;
  if (seapoint_turbidity_sensor.getData(d)) {
    static uint8_t cbor_buf[BM_SEAPOINT_TURBIDITY_DATA_MSG_MAX_SIZE];
    size_t encoded_len = 0;
    if (BmSeapointTurbidityDataMsg::encode(d, cbor_buf, sizeof(cbor_buf), &encoded_len) == CborNoError) {
      bm_pub_wl(seapoint_turbidity_topic, seapoint_turbidity_topic_str_len, cbor_buf, encoded_len, 0);
    } else {
      printf("Failed to encode turbidity data message\n");
    }
  }
}


// PLUART write command to take measurement
//static constexpr char queryMDOT[] = "MDOT\r";
//static void querySensor(void) {
//    PLUART::write((uint8_t *)queryMDOT, strlen(queryMDOT));
//}



/*
void setup(void) {
  // USER ONE-TIME SETUP CODE GOES HERE
  // Retrieve user-set config values out of NVM.
  userConfigurationPartition->getConfig("plUartBaudRate", strlen("plUartBaudRate"), baud_rate_config);
  userConfigurationPartition->getConfig("plUartLineTerm", strlen("plUartLineTerm"), line_term_config);
  systemConfigurationPartition->getConfig("sensorBmLogEnable", strlen("sensorBmLogEnable"), bm_log_enable);
  // Setup the UART – the on-board serial driver that talks to the RS232 transceiver.
  PLUART::init(USER_TASK_PRIORITY);
  // Baud set per expected baud rate of the sensor.
  PLUART::setBaud(baud_rate_config);
  // Enable passing raw bytes to user app.
  PLUART::setUseByteStreamBuffer(true);
  // Enable parsing lines and passing to user app.
  /// Warning: PLUART only stores a single line at a time. If your attached payload sends lines
  /// faster than the app reads them, they will be overwritten and data will be lost.
  PLUART::setUseLineBuffer(true);
  // Set a line termination character per protocol of the sensor.
  PLUART::setTerminationCharacter((char)line_term_config);
  // Turn on the UART.
  PLUART::enable();
  // Enable the input to the Vout power supply.
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
*/

/*
void loop(void) {
  static int64_t readingBytesTimer = -1;
  vTaskDelay(10000);
  // USER LOOP CODE GOES HERE

  Print out the values from the humidity & temperature sensor
  float oxygen, temperature = 0.0;
  if (htuSamplerGetLatest(oxygen, temperature)) {
    oxygen_stats.addSample(oxygen);
    temp_stats.addSample(temperature);
    printf("###DO-temp from user code: DO: %f, T:%f\n", oxygen, temperature);
  }
  Print out the values from the humidity & temperature sensor

  querySensor();
  // Read a line if it is available
  // Note - PLUART::setUseLineBuffer must be set true in setup to enable lines.
  if (PLUART::lineAvailable()) { 
    // Shortcut the raw bytes cluster completion so the parsed line will be on a new console line
    if (readingBytesTimer > -1) {
      //printf("\n");
      readingBytesTimer = -1;
    }

    uint16_t read_len = PLUART::readLine(payload_buffer, sizeof(payload_buffer));
    // Get the RTC if available
    RTCTimeAndDate_t time_and_date = {};
    rtcGet(&time_and_date);
    char rtcTimeBuffer[32];
    rtcPrint(rtcTimeBuffer, &time_and_date);

    // Based on configuration, print the payload data to a file, to the bm_printf console, and to the printf console. 
    if (bm_log_enable) {
      bm_fprintf(0, "payload_data.log", USE_TIMESTAMP, "tick: %llu, rtc: %s, line: %.*s\n",
                 uptimeGetMs(), rtcTimeBuffer, read_len, payload_buffer);
    }
    bm_printf(0, "[payload] | tick: %llu, rtc: %s, line: %.*s\n", uptimeGetMs(), rtcTimeBuffer, read_len, payload_buffer);
    printf("[payload-line] | tick: %llu, rtc: %s, line: %.*s\n", uptimeGetMs(), rtcTimeBuffer, read_len, payload_buffer);
    //uint8_t tx_data[sizeof()];
    if(spotter_tx_data(payload_buffer, read_len, BM_NETWORK_TYPE_CELLULAR_IRI_FALLBACK)){
      printf("%llut - %s | Successfully sent Spotter transmit data request\n", uptimeGetMs(), rtcTimeBuffer);
    }
    else {
        printf("%llut - %s | ERR Failed to send Spotter transmit data request!\n", uptimeGetMs(), rtcTimeBuffer);
    }
    ledLinePulse = uptimeGetMs(); // trigger a pulse on LED
  }


// We have 3 sensor channels - oxygen, temperature, and q
static const uint8_t NUM_SENSORS = 3;
// Create an array of stat agg structs. 1st will be oxygen, 2nd will be  temperature, 3rd will be q
sensorStatAgg_t report_stats[NUM_SENSORS] = {};
// Pack the statistics into the structs
report_stats[0].sample_count = temp_stats.getNumSamples();
report_stats[0].mean = temp_mean;
report_stats[0].stdev = temp_stdev;
report_stats[1].sample_count = hum_stats.getNumSamples();
report_stats[1].mean = hum_mean;
report_stats[1].stdev = hum_stdev;
report_stats[2].sample_count = temp_stats.getNumSamples();
report_stats[2].mean = temp_mean;
report_stats[2].stdev = temp_stdev;
// Copy the structs into a raw bytes buffer and send
*/

/*
uint8_t tx_data[sizeof(sensorStatAgg_t) * NUM_SENSORS] = {};
for (uint8_t i = 0; i < NUM_SENSORS; i++) {
  memcpy(tx_data + sizeof(sensorStatAgg_t) * i, reinterpret_cast<uint8_t *>(&report_stats[i]), sizeof(sensorStatAgg_t));
}
if(spotter_tx_data(tx_data, sizeof(sensorStatAgg_t) * NUM_SENSORS, BM_NETWORK_TYPE_CELLULAR_IRI_FALLBACK)){
  printf("%llut - %s | Successfully sent Spotter transmit data request\n", uptimeGetMs(), rtcTimeBuffer);
}
else {
  printf("%llut - %s | ERR Failed to send Spotter transmit data request!\n", uptimeGetMs(), rtcTimeBuffer);
}



  // Read a cluster of Rx bytes if available
  // -- A timer is used to try to keep clusters of bytes (say from lines) in the same output.
  // Note - PLUART::setUseByteStreamBuffer must be set true in setup to enable bytes.
  if (readingBytesTimer == -1 && PLUART::byteAvailable()) {
    // Get the RTC if available
    RTCTimeAndDate_t time_and_date = {};
    rtcGet(&time_and_date);
    char rtcTimeBuffer[32];
    rtcPrint(rtcTimeBuffer, &time_and_date);
    // printf("[payload-bytes] | tick: %llu, rtc: %s, bytes:", uptimeGetMs(), rtcTimeBuffer);
    // not very readable, but it's a compact trick to overload our timer variable with a -1 flag
    readingBytesTimer = (int64_t)((u_int32_t)uptimeGetMs());
  }

   
  while (PLUART::byteAvailable()) {
    readingBytesTimer = (int64_t)((u_int32_t)uptimeGetMs());
    uint8_t byte_read = PLUART::readByte();
    printf("%02X ", byte_read);
  }
  
  
  if (readingBytesTimer > -1 && (u_int32_t)uptimeGetMs() - (u_int32_t)readingBytesTimer >= BYTES_CLUSTER_MS) {
    //printf("\n");
    readingBytesTimer = -1;
  }

  // start pub_example 
  // https://github.com/bristlemouth/bm_protocol/blob/cfc6c5e03fb575c761a322ce7a658d4a75e9ef50/src/apps/bm_devkit/pub_example/user_code/user_code.cpp
  static uint64_t publishTimer = uptimeGetMs();
  // Publish data every PUBLISH_PERIOD_MS milliseconds.
  if (uptimeGetMs() - publishTimer >= PUBLISH_PERIOD_MS) {
    // This is a data buffer to hold the data to be published.
    // It is larger than necessary to demonstrate the use of the bm_pub function.
    static uint8_t data_buffer[128];
    // Clear the data buffer
    memset(data_buffer, 0, sizeof(data_buffer));
    // Print a simple message into the data buffer to publish to the topic.
    sprintf((char *)data_buffer, "[%" PRId64 " ms] Hello World!", uptimeGetMs());
    // Publish the data buffer to the topic.
    if (!bm_pub(EXAMPLE_PUBLISH_TOPIC, data_buffer, sizeof(data_buffer),
                EXAMPLE_PUBLISH_TOPIC_TYPE, EXAMPLE_PUBLISH_TOPIC_VERSION)) {
      printf("Failed to publish message\n");
    } 
    else {
      printf("Published message to network: %s\n", data_buffer);
    }
    // Increment the publish timer
    publishTimer += PUBLISH_PERIOD_MS;
  }
  // end pub_example





// LED Function
  static bool led2State = false;
  /// Flash the LED blue if we received Rx data
  if (!led2State && ledLinePulse > -1) {
    IOWrite(&LED_BLUE, 1);
    led2State = true;
  }
  // If LED2 has been on for LED_ON_TIME_MS, turn it off.
  else if (led2State && ((u_int32_t)uptimeGetMs() - ledLinePulse >= LED_ON_TIME_MS)) {
    IOWrite(&LED_BLUE, 0);
    ledLinePulse = -1;
    led2State = false;
  }

  /// Blink the LED green periodically for sign of life.
  static u_int32_t ledPulseTimer = uptimeGetMs();
  static u_int32_t ledOnTimer = 0;
  static bool led1State = false;
  // Turn LED1 on green every LED_PERIOD_MS milliseconds.
  if (!led1State && ((u_int32_t)uptimeGetMs() - ledPulseTimer >= LED_PERIOD_MS)) {
    IOWrite(&LED_GREEN, 1);
    ledOnTimer = uptimeGetMs();
    ledPulseTimer += LED_PERIOD_MS;
    led1State = true;
  }
  // If LED1 has been on for LED_ON_TIME_MS milliseconds, turn it off.
  else if (led1State && ((u_int32_t)uptimeGetMs() - ledOnTimer >= LED_ON_TIME_MS)) {
    IOWrite(&LED_GREEN, 0);
    led1State = false;
  }
}
*/