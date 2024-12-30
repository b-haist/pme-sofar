#pragma once
#include <stdint.h>
#include "OrderedSeparatorLineParser.h"
#include "bm_seapoint_turbidity_data_msg.h"

class PmeDoSensor {
  public:
    PmeDoSensor()
        : _parser(",", 256, PARSER_VALUE_TYPE, 5){};
    void init();
    bool getData(BmSeapointTurbidityDataMsg::Data &d);
    void flush(void);

  public:
    static constexpr char SEAPOINT_TURBIDITY_RAW_LOG[] = "seapoint_turbidity_raw.log";

  private:
    static constexpr uint32_t BAUD_RATE = 9600;
    static constexpr char LINE_TERM = '\r';
    static constexpr ValueType PARSER_VALUE_TYPE[] = {TYPE_DOUBLE, TYPE_DOUBLE, TYPE_DOUBLE, TYPE_DOUBLE, TYPE_DOUBLE};
    static constexpr char SENSOR_BM_LOG_ENABLE[] = "sensorBmLogEnable";

  private:
    uint32_t _sensorBmLogEnable = 0;
    OrderedSeparatorLineParser _parser;
    char _payload_buffer[2048];
};