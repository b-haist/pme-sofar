#pragma once
#include <stdint.h>
#include "OrderedSeparatorLineParser.h"
#include "pme_dissolved_oxygen_msg.h"
#include "pme_wipe_msg.h"

class PmeSensor {
  public:
    PmeSensor()
        : _DOTparser(",", 256, DOT_PARSER_VALUE_TYPE, 4),
          _WIPEparser(",", 256, WIPE_PARSER_VALUE_TYPE, 6) {};
    void init();
    bool getDoData(PmeDissolvedOxygenMsg::Data &d);
    bool getWipeData(PmeWipeMsg::Data &w);
    bool getSN(PmeDissolvedOxygenMsg::Data &s);
    void flush();

  public:
    static constexpr char PME_DO_RAW_LOG[] = "pme_do_raw.log";
    static constexpr char PME_WIPE_RAW_LOG[] = "pme_wipe_raw.log";

  private:
    static constexpr u_int32_t BAUD_RATE = 9600;
    static constexpr char LINE_TERM = '\r';
    static constexpr ValueType DOT_PARSER_VALUE_TYPE[] = {TYPE_DOUBLE, TYPE_DOUBLE, TYPE_DOUBLE, TYPE_DOUBLE};
    static constexpr ValueType WIPE_PARSER_VALUE_TYPE[] = {TYPE_DOUBLE, TYPE_DOUBLE, TYPE_DOUBLE, TYPE_DOUBLE, TYPE_DOUBLE, TYPE_DOUBLE};
    static constexpr char SENSOR_BM_LOG_ENABLE[] = "sensorBmLogEnable";

  private:
    u_int32_t _sensorBmLogEnable = 0;
    OrderedSeparatorLineParser _DOTparser;
    OrderedSeparatorLineParser _WIPEparser;
    char _DOTpayload_buffer[2048];
    char _WIPEpayload_buffer[2048];
    char _SNpayload_buffer[2048];
};