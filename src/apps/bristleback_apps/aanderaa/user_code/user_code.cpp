#include "avgSampler.h"
#include "debug.h"
#include "util.h"
#include "user_code.h"
#include "bsp.h"
#include "stm32_rtc.h"
#include "task_priorities.h"
#include "uptime.h"
#include "bm_printf.h"
#include "bm_pubsub.h"
#include "lwip/inet.h"
#include "bm_network.h"
#include "usart.h"
#include "payload_uart.h"
#include "OrderedKVPLineParser.h"
#include "array_utils.h"
#include "sensors.h"

#define LED_ON_TIME_MS 20
#define LED_PERIOD_MS 1000

/// For Turning Text Into Numbers
#ifdef AANDERAA_BLUE
#define NUM_PARAMS_TO_AGG 10 // Need to manually adjust current_data, valueTypes and keys when changing
#else
#define NUM_PARAMS_TO_AGG 9 // Need to manually adjust current_data, valueTypes and keys when changing
#endif
// Skip the first couple of readings for now. It takes time for stuff to init,
// and we're not parsing the "*" characters to infer errors yet.
#define N_SKIP_SAMPLES 2
uint8_t samples_skipped = 0;
bool ready_to_send = true; // Flip this off after sending to make it a one-shot.
#define DEFAULT_CURRENT_AGG_PERIOD_MIN (3.0) // Aggregate after 3 minutes.
// Global variable to store configuration setting. We'll retrieve it in setup().
float current_agg_period_min = DEFAULT_CURRENT_AGG_PERIOD_MIN;
#define CURRENT_SAMPLE_PERIOD_MS 2000 // Aanderaa "interval" config setting.
#define CURRENT_AGG_PERIOD_MS ((double)current_agg_period_min * 60 * 1000)
#define N_SAMPLES_PAD 10 // Extra sample padding to account for timing slop.
#define MAX_CURRENT_SAMPLES (((uint64_t)CURRENT_AGG_PERIOD_MS / CURRENT_SAMPLE_PERIOD_MS) + N_SAMPLES_PAD)
typedef struct {
  uint16_t sample_count;
  float min;
  float max;
  float mean;
  float stdev;
} __attribute__((__packed__)) currentData_t;
// create a working instance here, we'll allocate memory for values later.
#ifdef AANDERAA_BLUE
AveragingSampler current_data[10];
#else
AveragingSampler current_data[9];
#endif
char* stats_print_buffer; // Buffer to store debug print data for the stats aggregation.
RTCTimeAndDate_t statsStartRtc = {}; // Timestampt that tracks the start of aggregation periods.

// app_main passes a handle to the user config partition in NVM.
extern cfg::Configuration* userConfigurationPartition;

/*
 * Setup a LineParser to turn the ASCII serial data from the Aanderaa into numbers
 * Lines from the Aanderaa sensor look like:
 *    TODO - detail */
const char lineHeader[] = "MEASUREMENT";
// Let's use 64 bit doubles for everything.
//   We've got luxurious amounts of RAM on this chip, and it's much easier to avoid roll-overs and precision issues
//   by using it vs. troubleshooting them because we prematurely optimized things.
ValueType valueTypes[] = {
    TYPE_DOUBLE,
    TYPE_DOUBLE,
    TYPE_DOUBLE,
    TYPE_DOUBLE,
    TYPE_DOUBLE,
    TYPE_DOUBLE,
    TYPE_DOUBLE,
    TYPE_DOUBLE,
    TYPE_DOUBLE,
#ifdef AANDERAA_BLUE
    TYPE_DOUBLE
#endif
};
const char * keys[] = {
    "Abs Speed[cm/s]",
    "Direction[Deg.M]",
    "North[cm/s]",
    "East[cm/s]",
    "Heading[Deg.M]",
    "Tilt X[Deg]",
    "Tilt Y[Deg]",
    "Ping Count",
    "Abs Tilt[Deg]",
#ifdef AANDERAA_BLUE
    "Memory Used[Bytes]"
#endif
};

char payload_buffer[2048];

// Declare the parser here with separator, buffer length, value types array, and number of values per line.
//   We'll initialize the parser later in setup to allocate all the memory we'll need.
OrderedKVPLineParser parser("\t", 750, valueTypes, NUM_PARAMS_TO_AGG, keys, lineHeader);

// A timer variable we can set to trigger a pulse on LED2 when we get payload serial data
static int32_t ledLinePulse = -1;


currentData_t aggregateStats(AveragingSampler &sampler) {
  currentData_t ret_stats = {};
  if ( sampler.getNumSamples() > 0) {
    ret_stats.mean = sampler.getMean();
    ret_stats.stdev = sampler.getStd(ret_stats.mean);
    ret_stats.min = sampler.getMin();
    ret_stats.max = sampler.getMax();
    ret_stats.sample_count = sampler.getNumSamples();
    /// NOTE - Reset the buffer.
    sampler.clear();
  }
  return ret_stats;
}

void setup(void) {
  /* USER ONE-TIME SETUP CODE GOES HERE */
  // Retrieve user-set config values out of NVM.
  userConfigurationPartition->getConfig("currentAggPeriodMin",
                                        strlen("currentAggPeriodMin"),
                                        current_agg_period_min);

  // Allocate memory for pressure data buffer.
  for (uint8_t i=0; i<NUM_PARAMS_TO_AGG; i++) {
    current_data[i].initBuffer(MAX_CURRENT_SAMPLES);
  }
  stats_print_buffer = static_cast<char *>(pvPortMalloc(sizeof(char) * 1500)); // Trial and error, should be plenty of space
  // Initialize our LineParser, which will allocated any needed memory for parsing.
  parser.init();
  // Setup the UART – the on-board serial driver that talks to the RS232 transceiver.
  PLUART::init(USER_TASK_PRIORITY);
  // Baud set per expected baud rate of the sensor.
#ifdef AANDERAA_BLUE
  PLUART::setBaud(115200);
#else
  PLUART::setBaud(9600);
#endif
  // Set a line termination character per protocol of the sensor.
  PLUART::setTerminationCharacter('\n');
  // Turn on the UART.
  PLUART::enable();
  // Enable the input to the Vout power supply.
  IOWrite(&BB_VBUS_EN, 0);
  // ensure Vbus stable before enable Vout with a 5ms delay.
  vTaskDelay(pdMS_TO_TICKS(5));
  // enable Vout, 12V by default.
  IOWrite(&BB_PL_BUCK_EN, 0);
  rtcGet(&statsStartRtc);
}

void loop(void) {
  /* USER LOOP CODE GOES HERE */
  /// This aggregates BMDK sensor readings into stats, and sends them along to Spotter
  static uint32_t sensorStatsTimer = uptimeGetMs();
  static uint32_t statsStartTick = uptimeGetMs();
  if (ready_to_send && (uint32_t)uptimeGetMs() - sensorStatsTimer >= CURRENT_AGG_PERIOD_MS) {
    /// set ready_to_send false when sending to setup as a 2 minute one-shot to save data costs.
    ready_to_send = false;
    sensorStatsTimer = uptimeGetMs();
    // create additional buffers for convenience, we won't allocate values arrays.
#ifdef AANDERAA_BLUE
    currentData_t current_tx_data[10] = {{}, {}, {}, {}, {}, {}, {}, {}, {}, {}};
#else
    currentData_t current_tx_data[9] = {{}, {}, {}, {}, {}, {}, {}, {}, {}};
#endif

    for (uint8_t i=0; i<NUM_PARAMS_TO_AGG; i++) {
      current_tx_data[i] = aggregateStats(current_data[i]);
    }

    char rtcTimeBuffer[32] = {};
    rtcPrint(rtcTimeBuffer, &statsStartRtc);
    uint32_t statsEndTick = uptimeGetMs();
    int buffer_offset = sprintf(stats_print_buffer,
            "rtc_start: %s, tick_start: %u, tick_end: %u | ",
            rtcTimeBuffer, statsStartTick, statsEndTick);
    for (uint8_t i=0; i<NUM_PARAMS_TO_AGG; i++) {
      buffer_offset += sprintf(stats_print_buffer + buffer_offset,
                               "key: %s, n: %u, min: %.4f, max: %.4f, mean: %.4f, std: %.4f | ",
                               keys[i],
                               current_tx_data[i].sample_count,
                               current_tx_data[i].min,
                               current_tx_data[i].max,
                               current_tx_data[i].mean,
                               current_tx_data[i].stdev
      );
    }
    printf("DEBUG - wrote %d chars to buffer.", buffer_offset);
    bm_fprintf(0, "aanderaa_agg.log", "%s\n", stats_print_buffer);
    bm_printf(0, "[aanderaa-agg] | %s", stats_print_buffer);
    printf("[aanderaa-agg] | %s\n", stats_print_buffer);
    // Update variables tracking start time of agg period in ticks and RTC.
    statsStartTick = statsEndTick;
    RTCTimeAndDate_t time_and_date = {};
    rtcGet(&time_and_date);
    statsStartRtc = time_and_date;

    /// Remote Tx using spotter/transmit-data
    // Setup raw bytes for Remote Tx data buffer.
    //    We don't need the full sizeof(currentData_t), don't need pointer to values.
    static const uint8_t N_STAT_ELEM_BYTES = 18;
    static const uint16_t N_TX_DATA_BYTES = N_STAT_ELEM_BYTES * NUM_PARAMS_TO_AGG;
    uint8_t tx_data[N_TX_DATA_BYTES] = {};
//    // Iterate through all of our aggregated stats, and copy data to our tx buffer.
    for (uint8_t i = 0; i < NUM_PARAMS_TO_AGG; i++) {
      memcpy(tx_data + N_STAT_ELEM_BYTES*i, reinterpret_cast<uint8_t *>(&current_tx_data[i]), N_STAT_ELEM_BYTES);
    }
//    //
    if(spotter_tx_data(tx_data, N_TX_DATA_BYTES, BM_NETWORK_TYPE_CELLULAR_IRI_FALLBACK)){
      printf("%" PRIu64 "t - %s | Sucessfully sent %" PRIu16 " bytes Spotter transmit data request\n", uptimeGetMs(), rtcTimeBuffer, N_TX_DATA_BYTES);
    }
    else {
      printf("%" PRIu64 "t - %s | Failed to send %" PRIu16 " bytes Spotter transmit data request\n", uptimeGetMs(), rtcTimeBuffer, N_TX_DATA_BYTES);
    }
  }

  /// Blink green LED when data is received from the payload UART.
  static bool led2State = false;
  // If LED_GREEN is off and the ledLinePulse flag is set, turn it on.
  if (!led2State && ledLinePulse > -1) {
    IOWrite(&LED_GREEN, BB_LED_ON);
    led2State = true;
  } // If LED_GREEN has been on for LED_ON_TIME_MS, turn it off.
  else if (led2State && ((u_int32_t)uptimeGetMs() - ledLinePulse >= LED_ON_TIME_MS)) {
    IOWrite(&LED_GREEN, BB_LED_OFF);
    ledLinePulse = -1;
    led2State = false;
  }

  // Blink red LED on a simple timer.
  static u_int32_t ledPulseTimer = uptimeGetMs();
  static u_int32_t ledOnTimer = 0;
  static bool led1State = false;
  // Turn LED_RED on every LED_PERIOD_MS milliseconds.
  if (!led1State && ((u_int32_t)uptimeGetMs() - ledPulseTimer >= LED_PERIOD_MS)) {
    IOWrite(&LED_RED, BB_LED_ON);
    ledOnTimer = uptimeGetMs();
    ledPulseTimer += LED_PERIOD_MS;
    led1State = true;
  } // If LED_RED has been on for LED_ON_TIME_MS milliseconds, turn it off.
  else if (led1State && ((u_int32_t)uptimeGetMs() - ledOnTimer >= LED_ON_TIME_MS)) {
    IOWrite(&LED_RED, BB_LED_OFF);
    led1State = false;
  }

  // If there is a line to read, read it ad then parse it
  if (PLUART::lineAvailable()) {

    uint16_t read_len = PLUART::readLine(payload_buffer, sizeof(payload_buffer));

    char rtcTimeBuffer[32] = {};
    rtcPrint(rtcTimeBuffer, NULL);
    bm_fprintf(0, "aanderaa_raw.log", "tick: %" PRIu64 ", rtc: %s, line: %.*s\n", uptimeGetMs(), rtcTimeBuffer, read_len, payload_buffer);
    bm_printf(0, "[aanderaa] | tick: %" PRIu64 ", rtc: %s, line: %.*s", uptimeGetMs(), rtcTimeBuffer, read_len, payload_buffer);
    printf("[aanderaa] | tick: %" PRIu64 ", rtc: %s, line: %.*s\n", uptimeGetMs(), rtcTimeBuffer, read_len, payload_buffer);

    ledLinePulse = uptimeGetMs(); // trigger a pulse on LED2
    // Now when we get a line of text data, our LineParser turns it into numeric values.
    if (!parser.parseLine(payload_buffer, read_len)) {
      printf("Error parsing line!\n");
      return;
    }
    if (samples_skipped < N_SKIP_SAMPLES) {
      samples_skipped++;
      printf("Skipping samples during init.\n");
      return;
    }
    // Now let's aggregate those values into statistics
    if (current_data[0].getNumSamples() >= MAX_CURRENT_SAMPLES) {
      printf("ERR - No more room in current reading buffer, already have %d readings!\n", MAX_CURRENT_SAMPLES);
      return;
    }

    printf("parsed values:\n");
    for (uint8_t i=0; i<NUM_PARAMS_TO_AGG; i++) {
      double param_reading = parser.getValue(i).data.double_val;
      current_data[i].addSample(param_reading);
      printf("\t%s | value: %f, count: %u/%d, min: %f, max: %f\n", keys[i], param_reading, current_data[i].getNumSamples(), MAX_CURRENT_SAMPLES-N_SAMPLES_PAD, current_data[i].getMin(), current_data[i].getMax());
    }
  }
}
