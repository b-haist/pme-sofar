#pragma once
#include <stdint.h>
#include "OrderedSeparatorLineParser.h"
#include "pme_dissolved_oxygen_msg.h"
#include "pme_wipe_msg.h"

class PmeSensor {
  public:
    PmeSensor()
        : _parser(",", 256, PARSER_VALUE_TYPE, 5){};
    void init();
    bool getDoData(PmeDissolvedOxygenMsg::Data &d);
    bool getWipeData(PmeWipeMsg::Data &w);
    bool getSN(PmeDissolvedOxygenMsg::Data &s);
    void flush(void);

  public:
    static constexpr char PME_DO_RAW_LOG[] = "pme_do_raw.log";
    static constexpr char PME_WIPE_RAW_LOG[] = "pme_wipe_raw.log";

  private:
    static constexpr u_int32_t BAUD_RATE = 9600;
    static constexpr char LINE_TERM = '\r';
    static constexpr ValueType PARSER_VALUE_TYPE[] = {TYPE_DOUBLE, TYPE_DOUBLE, TYPE_DOUBLE, TYPE_DOUBLE, TYPE_DOUBLE};
    static constexpr char SENSOR_BM_LOG_ENABLE[] = "sensorBmLogEnable";

  private:
    u_int32_t _sensorBmLogEnable = 0;
    OrderedSeparatorLineParser _parser;
    char _payload_buffer[2048];
};