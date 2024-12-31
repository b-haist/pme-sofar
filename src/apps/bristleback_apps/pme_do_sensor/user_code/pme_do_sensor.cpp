#include <cstdint>
#include <cstddef>
#include "FreeRTOS.h"
#include "bm_printf.h"
#include "configuration.h"
#include "payload_uart.h"
#include "pme_do_sensor.h"
#include "serial.h"
#include "stm32_rtc.h"
#include "task_priorities.h"
#include "uptime.h"
#include "util.h"

extern cfg::Configuration *systemConfigurationPartition;

//Function to request a DO measurement from the microDOT sensor. (P.F.)
static constexpr char queryMDOT[] = "MDOT\r";
//Function to request a wipe from the microDOT sensor. (P.F.)
static constexpr char queryWIPE[] = "WIPE\r";

/**
 * @brief Initializes the Seapoint Turbidity Sensor.
 *
 * This function performs several operations to prepare the sensor for use:
 * - Asserts that the system configuration partition is available.
 * - Initializes the parser.
 * - Retrieves the sensor log enable configuration.
 * - Initializes the Payload UART (PLUART) with a specified task priority.
 * - Sets the baud rate for the PLUART to 115200, which is the rate expected by the Seapoint turbidity sensor.
 * - Disables the use of raw byte stream buffer in the PLUART.
 * - Enables the use of line buffer in the PLUART. Note that the PLUART only stores a single line at a time.
 *   If the attached payload sends lines faster than the app reads them, they will be overwritten and data will be lost.
 * - Sets a line termination character for the PLUART according to the protocol of the sensor.
 * - Enables the PLUART, effectively turning on the UART.
 */
void PmeSensor::init() {
  configASSERT(systemConfigurationPartition);
  _parser.init();
  systemConfigurationPartition->getConfig(SENSOR_BM_LOG_ENABLE, strlen(SENSOR_BM_LOG_ENABLE),
                                          _sensorBmLogEnable);
  printf("sensorBmLogEnable: %" PRIu32 "\n", _sensorBmLogEnable);

  PLUART::init(USER_TASK_PRIORITY);
  // Baud set to 9600, which is expected by the microDOT sensor
  PLUART::setBaud(BAUD_RATE);
  // Disable passing raw bytes to user app.
  PLUART::setUseByteStreamBuffer(false);
  /// Warning: PLUART only stores a single line at a time. If your attached payload sends lines
  /// faster than the app reads them, they will be overwritten and data will be lost.
  PLUART::setUseLineBuffer(true);
  // Set a line termination character per protocol of the sensor.
  PLUART::setTerminationCharacter(LINE_TERM);
  // Turn on the UART.
  PLUART::enable();
}

/**
 * @brief Retrieves DO data from the microDOT Sensor. (P.F.)
 *
 * This function checks if a line of data is available from the sensor. If available, it reads the line into a buffer.
 * It then logs the data along with the current system uptime and RTC time.
 * The function then attempts to parse the data from the buffer. If the parsing is successful and the data is of the correct type,
 * it populates the passed PmeDissolvedOxygenMsg::Data structure with the parsed data and the current system time.
 *
 * @param d Reference to a PmeDissolvedOxygenMsg::Data structure where the parsed data will be stored.
 * @return Returns true if data was successfully retrieved and parsed, false otherwise.
 */
bool PmeSensor::getDoData(PmeDissolvedOxygenMsg::Data &d) {
  bool success = false;
  PLUART::write((uint8_t *)queryMDOT, sizeof(queryMDOT));
  vTaskDelay(250);
  if (PLUART::lineAvailable()) {
    uint16_t read_len = PLUART::readLine(_payload_buffer, sizeof(_payload_buffer));

    RTCTimeAndDate_t time_and_date = {};
    rtcGet(&time_and_date);
    char rtc_time_str[32] = {};
    rtcPrint(rtc_time_str, NULL);

    if (_sensorBmLogEnable) {
      bm_fprintf(0, PME_DO_RAW_LOG, USE_TIMESTAMP, "tick: %" PRIu64 ", rtc: %s, line: %.*s\n",
                 uptimeGetMs(), rtc_time_str, read_len, _payload_buffer);
    }
    bm_printf(0, "DOT | tick: %" PRIu64 ", rtc: %s, line: %.*s", uptimeGetMs(),
              rtc_time_str, read_len, _payload_buffer);
    printf("DOT | tick: %" PRIu64 ", rtc: %s, line: %.*s\n", uptimeGetMs(),
           rtc_time_str, read_len, _payload_buffer);

    if (_parser.parseLine(_payload_buffer, read_len)) {
      Value temp_signal = _parser.getValue(2);
      Value do_signal = _parser.getValue(3);
      Value q_signal = _parser.getValue(4);
      if ( temp_signal.type != TYPE_DOUBLE || do_signal.type != TYPE_DOUBLE || q_signal.type != TYPE_DOUBLE) {
        printf("Parsed invalid DOT data: temp_signal: %d, do_signal: %d, q_signal: %d\n", temp_signal.type, do_signal.type, q_signal.type);
      } else {
        d.header.reading_time_utc_ms = rtcGetMicroSeconds(&time_and_date) / 1000;
        d.header.reading_uptime_millis = uptimeGetMs();
        d.temperature_deg_c = temp_signal.data.double_val;
        d.do_mg_per_l = do_signal.data.double_val;
        d.quality = q_signal.data.double_val;
        // DO Sat% not available at this time; setting to NULL causes error (converting NULL to double)
        //d.do_saturation_pct = 0;
        success = true;
      }
    } else {
      printf("Failed to parse DOT data\n");
    }
  }
  return success;
}

/**
 * @brief Retrieves Wipe data from the microDOT Sensor. (P.F.)
 *
 * This function requests a wipe and then waits for a line of data from the sensor for a set period. If available, it reads the line into a buffer.
 * It then logs the data along with the current system uptime and RTC time.
 * If data was not available, it logs a message indicating that no data was received.
 * The function then attempts to parse the data from the buffer. If the parsing is successful and the data is of the correct type,
 * it populates the passed PmeWipeMsg::Data structure with the parsed data and the current system time.
 *
 * @param d Reference to a PmeWipeMsg::Data structure where the parsed data will be stored.
 * @return Returns true if data was successfully retrieved and parsed, false otherwise.
 */
bool PmeSensor::getWipeData(PmeWipeMsg::Data &d) {
  bool success = false;
  PLUART::write((uint8_t *)queryWIPE, sizeof(queryWIPE));
  vTaskDelay(250);
  //Determine how to continue checking for a line of data from the sensor until a timeout occurs (20 seconds). (P.F.)
  if (PLUART::lineAvailable()) {
    uint16_t read_len = PLUART::readLine(_payload_buffer, sizeof(_payload_buffer));

    RTCTimeAndDate_t time_and_date = {};
    rtcGet(&time_and_date);
    char rtc_time_str[32] = {};
    rtcPrint(rtc_time_str, NULL);

    if (_sensorBmLogEnable) {
      bm_fprintf(0, PME_DO_RAW_LOG, USE_TIMESTAMP, "tick: %" PRIu64 ", rtc: %s, line: %.*s\n",
                 uptimeGetMs(), rtc_time_str, read_len, _payload_buffer);
    }
    bm_printf(0, "DOT | tick: %" PRIu64 ", rtc: %s, line: %.*s", uptimeGetMs(),
              rtc_time_str, read_len, _payload_buffer);
    printf("DOT | tick: %" PRIu64 ", rtc: %s, line: %.*s\n", uptimeGetMs(),
           rtc_time_str, read_len, _payload_buffer);

    // Expect readout in format +90 (wipe_current_mean_mA), +5 (wipe_duration_s)
    if (_parser.parseLine(_payload_buffer, read_len)) {
      Value wipe_current_mean_mA = _parser.getValue(0);
      Value wipe_duration = _parser.getValue(1);
      if (wipe_current_mean_mA.type != TYPE_DOUBLE || wipe_duration.type != TYPE_DOUBLE) { 
        printf("Parsed invalid DOT data: wipe_current_mean_mA: %d, wipe_duration (sec): %d\n", wipe_current_mean_mA.type, wipe_duration.type);
        } 
      else {
        d.header.reading_time_utc_ms = rtcGetMicroSeconds(&time_and_date) / 1000;
        d.header.reading_uptime_millis = uptimeGetMs();
        d.wipe_current_mean_ma = wipe_current_mean_mA.data.double_val;
        d.wipe_duration_s = wipe_duration.data.double_val;
        success = true;
        }
    } 
    else {
      printf("Failed to parse DOT data\n");
    }
  }
  return success;
}

/**
 * @brief Flushes the data from the sensor driver.
 */
void PmeSensor::flush(void) { PLUART::reset(); }
