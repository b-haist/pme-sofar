//
// Created by Uma Arthika Katikapalli on 10/7/24.
// Using seapoint turbidity sensor bristleback app as a reference
//
#include "user_code.h"
#include "bm_network.h"
#include "bm_printf.h"
#include "bm_pubsub.h"
#include "bristlefin.h"
#include "bsp.h"
#include "debug.h"
#include "lwip/inet.h"
#include "payload_uart.h"
#include "sensors.h"
#include "stm32_rtc.h"
#include "task_priorities.h"
#include "uptime.h"
#include "usart.h"
#include "util.h"
#include "exo3s_sensor.h"
#include "bristlefin.h"

#include "stm32_io.h"


void SondeEXO3sSensor::init() {
  printf("Sonde Init");
//  configASSERT(systemConfigurationPartition);
//  _parser.init();
//  systemConfigurationPartition->getConfig(SENSOR_BM_LOG_ENABLE, strlen(SENSOR_BM_LOG_ENABLE),
//                                          _sensorBmLogEnable);
//  printf("sensorBmLogEnable: %" PRIu32 "\n", _sensorBmLogEnable);

  PLUART::init(USER_TASK_PRIORITY);
  PLUART::setBaud(1200);
  PLUART::setUseByteStreamBuffer(false);
  PLUART::setUseLineBuffer(true);
  PLUART::setEvenParity();
  PLUART::set7bitDatawidth();
//  PLUART::enableDataInversion();
  PLUART::setTerminationCharacter(LINE_TERM);
  PLUART::enableTransactions(Bristlefin::sdi12Tx, Bristlefin::sdi12Rx);
  PLUART::enable();
}

void SondeEXO3sSensor::sdi_wake(int delay) {
  unsigned long timeStart;
  timeStart = uptimeGetMs();
  while(uptimeGetMs() < timeStart + delay);
  printf("sdi_wake\n");
//  PLUART::enable();	// re enable Serial
}


void SondeEXO3sSensor::sdi_break(void) {
  flush();
  unsigned long timeStart;
  STM32Pin_t *pin = (STM32Pin_t *)(PLUART::uart_handle).txPin->pin;
  PLUART::disable();
  LL_GPIO_SetOutputPin((GPIO_TypeDef *)pin->gpio, pin->pinmask);
  timeStart = uptimeGetMs();
  while(uptimeGetMs() < timeStart + breakTimeMin);
  LL_GPIO_ResetOutputPin((GPIO_TypeDef *)pin->gpio, pin->pinmask);
  timeStart = uptimeGetMs();
  while(uptimeGetMs() < timeStart + 9);
  printf("sdi_break\n");
  LL_GPIO_SetPinMode((GPIO_TypeDef *)pin->gpio, pin->pinmask, LL_GPIO_MODE_ALTERNATE);
//  init();	// re enable Serial
  PLUART::enable();
//  flush();
}


void SondeEXO3sSensor::sdi_transmit(const char *ptr) {
  PLUART::startTransaction();
  sdi_break();
  PLUART::write((uint8_t *)ptr, strlen(ptr));
  PLUART::endTransaction(50);
  printf("sdi_transmit_DONE\n");
//  delay(delayAfterTransmit);
}

uint8_t SondeEXO3sSensor::invertData(uint8_t data) {
  return ~data;  // Bitwise NOT to invert all bits
}

char SondeEXO3sSensor::sdi_receive(void) {

  char payload_buffer[256];
  printf("sdi_receive\n");
  vTaskDelay(pdMS_TO_TICKS(10));

//  while (!PLUART::lineAvailable()){
//    vTaskDelay(pdMS_TO_TICKS(5));
//    printf("stuck here\n");
//  }

  uint16_t read_len = PLUART::readLine(payload_buffer, sizeof(payload_buffer));
  printf("Read Line ----- %.*s\n", read_len, payload_buffer);
  printf("sdi_receive_DONE\n");
  return 0;
}

char SondeEXO3sSensor::sdi_cmd(const char *cmd) {
//  char index,cnt,i;
//  char* ptr;
//  char* xptr;
//  float fvalue[5];
  char result = sdiSuccess;
  sdi_transmit(cmd);
//  result = sdi_receive();

  return result;
}


//bool SondeEXO3sSensor::readData(char *buffer) {
//  buffer = 0;
//  return false;
//}

/**
 * @brief Flushes the data from the sensor driver.
 */
void SondeEXO3sSensor::flush(void) { PLUART::reset(); }