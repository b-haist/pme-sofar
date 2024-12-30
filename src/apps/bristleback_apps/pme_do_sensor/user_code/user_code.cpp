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
#include "OrderedSeparatorLineParser.h"

#define LED_ON_TIME_MS 50
#define LED_PERIOD_MS 5000
#define DEFAULT_BAUD_RATE 9600
#define DEFAULT_LINE_TERM 13 // Carraige Return, CR, 0x0D
#define BYTES_CLUSTER_MS 50  // used for console printing convenience
#define DEFAULT_UART_MODE MODE_RS232
#define DEFAULT_SENSOR_AGG_PERIOD_MIN (5)
// Converted to milliseconds.
#define SENSOR_AGG_PERIOD_MS ((double)DEFAULT_SENSOR_AGG_PERIOD_MIN * 60.0 * 1000.0)
// How long should our sample buffer arrays be?
// Assume we have a 2s polling period.
#define MAX_SENSOR_SAMPLES (((uint64_t)SENSOR_AGG_PERIOD_MS / 2000))



// Structures to store readings for our on-board sensors.
typedef struct {
  uint16_t sample_count; // Keep track of how many samples we have in the buffer.
  double* values; // Pointer to the buffer. We'll allocate memory later.
} __attribute__((__packed__)) sensorStatDataDouble_t;

typedef struct {
  uint16_t sample_count; // Keep track of how many samples we have in the buffer.
  uint64_t* values; // Pointer to the buffer. We'll allocate memory later.
} __attribute__((__packed__)) sensorStatDataUint64_t;

typedef struct {
  uint16_t sample_count;
  float mean;
  float stdev;
} __attribute__((__packed__)) sensorStatAgg_t;

// PMEData_t structure 
typedef struct {
    uint8_t time;
    uint8_t battery;
    double temp_value;
    double do_value;
    double q_value;
} __attribute__((__packed__))  PMEData_t;

PMEData_t pmeData = {};

float temp_mean = 0;
float do_mean = 0;
float q_mean = 0;

// We'll allocate memory for the values buffers later.
sensorStatDataUint64_t time_stats = {};
sensorStatDataUint64_t battery_stats = {};
sensorStatDataDouble_t temp_stats = {};
sensorStatDataDouble_t do_stats = {};
sensorStatDataDouble_t q_stats = {};

// app_main passes a handle to the config partitions in NVM.
extern cfg::Configuration *userConfigurationPartition;
extern cfg::Configuration *systemConfigurationPartition;

// variables to store configurations retrieved from NVM
static u_int32_t baud_rate_config = DEFAULT_BAUD_RATE;
static u_int32_t line_term_config = DEFAULT_LINE_TERM;
static u_int32_t bm_log_enable = true;
// Publish once per minute
static constexpr uint32_t PUBLISH_PERIOD_MS = (300000);
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
static const char* separator = ",";
static unsigned int maxLineLen = 256;
static const ValueType PARSER_VALUE_TYPE[] = {TYPE_DOUBLE, TYPE_DOUBLE, TYPE_DOUBLE, TYPE_DOUBLE, TYPE_DOUBLE};
OrderedSeparatorLineParser parser(separator, maxLineLen, PARSER_VALUE_TYPE, 5);

// A timer variable we can set to trigger a pulse on LED2 when we get payload serial data
static int32_t ledLinePulse = -1;

// A buffer to put Rx data from our payload sensor into.
char payload_buffer[2048];

char rtcTimeBuffer[32] = {}; // Let's get UTC time string for convenience
uint64_t this_uptime = 0;
uint64_t net_time = 0;

static constexpr char queryMDOT[] = "MDOT\r";

void fillPMEStruct(){
    pmeData.time = parser.getValue(0).data.double_val;
    pmeData.battery = parser.getValue(1).data.double_val;
    pmeData.temp_value = parser.getValue(2).data.double_val;
    pmeData.do_value = parser.getValue(3).data.double_val;
    pmeData.q_value = parser.getValue(4).data.double_val;
}

void addDataToStatsBuffers(){
    //time_stats.values[time_stats.sample_count++] = parser.getValue(0).data.double_val;
    //battery_stats.values[battery_stats.sample_count++] = parser.getValue(1).data.double_val;
    temp_stats.values[temp_stats.sample_count++] = parser.getValue(2).data.double_val;
    do_stats.values[do_stats.sample_count++] = parser.getValue(3).data.double_val;
    q_stats.values[q_stats.sample_count++] = parser.getValue(4).data.double_val;
    if(temp_stats.sample_count >= MAX_SENSOR_SAMPLES){
      printf("   ### Aggregating data; there are %lu readings! ###\n   ### CLEARING BUFFER, RESETTING SAMPLE COUNT ###\n", MAX_SENSOR_SAMPLES);
      printf("   ### Mean temp: %.3f, Mean DO: %.3f, Mean Q: %.3f\n", getMean(temp_stats.values, temp_stats.sample_count), getMean(do_stats.values, do_stats.sample_count), getMean(q_stats.values, q_stats.sample_count));
      temp_stats.sample_count = 0;
      do_stats.sample_count = 0;
      q_stats.sample_count = 0;
    }
}

void setup(void) {
  parser.init();
  /* USER ONE-TIME SETUP CODE GOES HERE */
  // Retrieve user-set config values out of NVM.
  userConfigurationPartition->getConfig("plUartBaudRate", strlen("plUartBaudRate"), baud_rate_config);
  userConfigurationPartition->getConfig("plUartLineTerm", strlen("plUartLineTerm"), line_term_config);
  systemConfigurationPartition->getConfig("sensorBmLogEnable", strlen("sensorBmLogEnable"), bm_log_enable);
  // Setup the UART – the on-board serial driver that talks to the RS232 transceiver.
  PLUART::init(USER_TASK_PRIORITY);
  // Baud set per expected baud rate the sensor.
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
  time_stats.sample_count = 0;
  battery_stats.sample_count = 0;
  temp_stats.sample_count = 0;
  do_stats.sample_count = 0;
  q_stats.sample_count = 0;
  // Allocate memory for statistical buffers
  temp_stats.values = (double *)malloc(MAX_SENSOR_SAMPLES * sizeof(double));
  do_stats.values = (double *)malloc(MAX_SENSOR_SAMPLES * sizeof(double));
  q_stats.values = (double *)malloc(MAX_SENSOR_SAMPLES * sizeof(double));
}


void loop(void) {
  /* USER LOOP CODE GOES HERE */
  static int64_t readingBytesTimer = -1;
  PLUART::write((uint8_t *)queryMDOT, strlen(queryMDOT));
  // Read a line if it is available
  if (PLUART::lineAvailable()) {
    uint16_t read_len = PLUART::readLine(payload_buffer, sizeof(payload_buffer));
    // Get the RTC if available
    RTCTimeAndDate_t time_and_date = {};
    rtcGet(&time_and_date);
    char rtcTimeBuffer[32];
    rtcPrint(rtcTimeBuffer, &time_and_date);
    
    if (parser.parseLine(payload_buffer, read_len)) {
      printf("---> parsed values: %.3f | %.3f | %.3f | %.3f | %.3f\n", parser.getValue(0).data, parser.getValue(1).data, parser.getValue(2).data, parser.getValue(3).data, parser.getValue(4).data);
      fillPMEStruct(); //No args. This uses global parser to update global nortkeData
      addDataToStatsBuffers(); // No args, updates all the buffers with latest data from parser.
      temp_mean = (float) getMean(temp_stats.values, temp_stats.sample_count);
      do_mean = (float) getMean(do_stats.values, do_stats.sample_count);
      q_mean = (float) getMean(q_stats.values, q_stats.sample_count);
      printf("---> [temp]:                        %.3f                     mean: %.3f,    sample ct: %d, time elapsed (sec): %llu\n", temp_stats.values[temp_stats.sample_count-1], temp_mean, temp_stats.sample_count, uptimeGetMs()/1000);
      printf("---> [do]:                                   %.3f             mean: %.3f,     sample ct: %d, time elapsed (sec): %llu\n", do_stats.values[do_stats.sample_count-1], do_mean, do_stats.sample_count, uptimeGetMs()/1000);
      printf("---> [q]:                                            %.3f     mean: %.3f,     sample ct: %d, time elapsed (sec): %llu\n", q_stats.values[q_stats.sample_count-1], q_mean, q_stats.sample_count, uptimeGetMs()/1000);
    }
    
    bm_printf(0, "[payload] | tick: %llu, rtc: %s, line: %.*s\n", uptimeGetMs(), rtcTimeBuffer, read_len, payload_buffer);
    printf("[payload-line] | tick: %llu, rtc: %s, line: %.*s\n", uptimeGetMs(), rtcTimeBuffer, read_len, payload_buffer);

    rtcPrint(rtcTimeBuffer, NULL);
    this_uptime = uptimeGetMs();
    //printf("batt v from userProcessLine | count: %u/%lu batt v: %f \n", battery_stats.sample_count, MAX_SENSOR_SAMPLES-10, pmeData.battery);

    if (readingBytesTimer > -1) {
      //printf("\n");
      readingBytesTimer = -1;
    }

    // Based on configuration, print the payload data to a file, to the bm_printf console, and to the printf console. 
    if (bm_log_enable) {
      bm_fprintf(0, "payload_data.log", USE_TIMESTAMP, "tick: %llu, rtc: %s, line: %.*s\n",
                  uptimeGetMs(), rtcTimeBuffer, read_len, payload_buffer);
    }

    /* Don't bombard at this stage
    if(spotter_tx_data(payload_buffer, read_len, BM_NETWORK_TYPE_CELLULAR_IRI_FALLBACK)){
      printf("%llut - %s | Successfully sent Spotter transmit data request\n", uptimeGetMs(), rtcTimeBuffer);
    }
    */
   
    else {
        printf("%llut - %s | ERR Failed to send Spotter transmit data request!\n", uptimeGetMs(), rtcTimeBuffer);
    }
    ledLinePulse = uptimeGetMs(); // trigger a pulse on LED
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
  
  if (readingBytesTimer > -1 && (u_int32_t)uptimeGetMs() - (u_int32_t)readingBytesTimer >= BYTES_CLUSTER_MS) {
    //printf("\n");
    readingBytesTimer = -1;
  }

//FIX LED FUNCTION; LED REMAINS DARK BLUE & DOES NOT FLASH
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

  // Query interval to read microDOT; make sure this matches the denominator of MAX_SENSOR_SAMPLES
  vTaskDelay(2000);
}