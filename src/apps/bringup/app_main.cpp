
// Includes from CubeMX Generated files
#include "main.h"

// Peripheral
#include "main.h"
#include "adc.h"
// #include "gpdma.h"
#include "i2c.h"
#include "icache.h"
#include "iwdg.h"
// #include "rtc.h"
#include "spi.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

// Includes for FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

#include "bm_l2.h"
#include "bristlemouth.h"
#include "bootloader_helper.h"
#include "bsp.h"
#include "cli.h"
#include "debug.h"
#include "debug_gpio.h"
#include "debug_i2c.h"
#include "debug_memfault.h"
#include "debug_spi.h"
#include "debug_sys.h"
#include "debug_uart.h"
#include "gpioISR.h"
#include "htu21d.h"
#include "io.h"
#include "lpm.h"
#include "memfault_platform_core.h"
#include "pca9535.h"
#include "serial.h"
#include "serial_console.h"
#include "usb.h"
#include "watchdog.h"

#ifdef USE_BOOTLOADER
#include "mcuboot_cli.h"
#endif

#include <stdio.h>

static void defaultTask(void *parameters);

SerialHandle_t lpuart1 = {
  .device = LPUART1,
  .name = "payload",
  .txPin = &PAYLOAD_TX,
  .rxPin = &PAYLOAD_RX,
  .txStreamBuffer = NULL,
  .rxStreamBuffer = NULL,
  .txBufferSize = 128,
  .rxBufferSize = 2048,
  .rxBytesFromISR = serialGenericRxBytesFromISR,
  .getTxBytesFromISR = serialGenericGetTxBytesFromISR,
  .processByte = NULL,
  .data = NULL,
  .enabled = false,
  .flags = 0,
};

// Serial console (when no usb present)
SerialHandle_t usart1 = {
  .device = USART1,
  .name = "debug",
  .txPin = &DEBUG_TX,
  .rxPin = &DEBUG_RX,
  .txStreamBuffer = NULL,
  .rxStreamBuffer = NULL,
  .txBufferSize = 4096,
  .rxBufferSize = 512,
  .rxBytesFromISR = serialGenericRxBytesFromISR,
  .getTxBytesFromISR = serialGenericGetTxBytesFromISR,
  .processByte = NULL,
  .data = NULL,
  .enabled = false,
  .flags = 0,
};

// Serial console USB device
SerialHandle_t usbCLI   = {
  .device = (void *)0, // Using CDC 0
  .name = "vcp-cli",
  .txPin = NULL,
  .rxPin = NULL,
  .txStreamBuffer = NULL,
  .rxStreamBuffer = NULL,
  .txBufferSize = 1024,
  .rxBufferSize = 512,
  .rxBytesFromISR = NULL,
  .getTxBytesFromISR = NULL,
  .processByte = NULL,
  .data = NULL,
  .enabled = false,
  .flags = 0,
};

// "bristlemouth" USB serial - Use TBD
SerialHandle_t usbPcap   = {
  .device = (void *)1, // Using CDC 1
  .name = "vcp-bm",
  .txPin = NULL,
  .rxPin = NULL,
  .txStreamBuffer = NULL,
  .rxStreamBuffer = NULL,
  .txBufferSize = 2048,
  .rxBufferSize = 64,
  .rxBytesFromISR = NULL,
  .getTxBytesFromISR = NULL,
  .processByte = NULL,
  .data = NULL,
  .enabled = false,
  .flags = 0,
};


static const DebugI2C_t debugI2CInterfaces[] = {
  {1, &i2c1}
};

static const DebugSPI_t debugSPIInterfaces[] = {
  {1, &spi1},
  {2, &spi2},
  {3, &spi3},
};

// TODO - move this to some debug file?
static const DebugGpio_t debugGpioPins[] = {
  {"adin_nss", &ADIN_NSS, GPIO_OUT},
  {"adin_int", &ADIN_INT, GPIO_IN},
  {"adin_pwr", &ADIN_PWR, GPIO_OUT},
  {"gpio1", &GPIO1, GPIO_OUT},
  {"gpio2", &GPIO2, GPIO_IN},
  {"bm_int", &BM_INT, GPIO_IN},
  {"bm_cs", &BM_CS, GPIO_OUT},
  {"flash_cs", &FLASH_CS, GPIO_OUT},
  {"boot_led", &BOOT_LED, GPIO_IN},
  {"exp_led_g1", &EXP_LED_G1, GPIO_OUT},
  {"exp_led_r1", &EXP_LED_R1, GPIO_OUT},
  {"exp_led_g2", &EXP_LED_G2, GPIO_OUT},
  {"exp_led_r2", &EXP_LED_R2, GPIO_OUT},
  {"vusb_detect", &VUSB_DETECT, GPIO_IN},
  {"adin_rst", &ADIN_RST, GPIO_OUT},
  {"exp_gpio03", &EXP_GPIO3, GPIO_OUT},
  {"exp_gpio04", &EXP_GPIO4, GPIO_OUT},
  {"exp_gpio05", &EXP_GPIO5, GPIO_OUT},
  {"exp_gpio06", &EXP_GPIO6, GPIO_OUT},
  {"exp_gpio07", &EXP_GPIO7, GPIO_OUT},
  {"exp_gpio08", &EXP_GPIO8, GPIO_OUT},
  {"exp_gpio09", &EXP_GPIO9, GPIO_OUT},
  {"exp_gpio10", &EXP_GPIO10, GPIO_OUT},
  {"exp_gpio11", &EXP_GPIO11, GPIO_OUT},
  {"exp_gpio12", &EXP_GPIO12, GPIO_OUT},
};

extern "C" void USART1_IRQHandler(void) {
  serialGenericUartIRQHandler(&usart1);
}

extern "C" int main(void) {

  // Before doing anything, check if we should enter ROM bootloader
  enterBootloaderIfNeeded();

  HAL_Init();

  SystemClock_Config();
  SystemPower_Config_ext();
  MX_GPIO_Init();
  MX_LPUART1_UART_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_ICACHE_Init();
  // MX_RTC_Init();
  MX_IWDG_Init();

  usbMspInit();

  // rtcInit();

  // Enable hardfault on divide-by-zero
  SCB->CCR |= 0x10;

  // Initialize low power manager
  lpmInit();

  // Inhibit low power mode during boot process
  lpmPeripheralActive(LPM_BOOT);

  BaseType_t rval = xTaskCreate(
              defaultTask,
              "Default",
              // TODO - verify stack size
              128 * 4,
              NULL,
              // Start with very high priority during boot then downgrade
              // once done initializing everything
              2,
              NULL);
  configASSERT(rval == pdTRUE);

  // Start FreeRTOS scheduler
  vTaskStartScheduler();

  /* We should never get here as control is now taken by the scheduler */

  while (1){};
}

static void defaultTask( void *parameters ) {

  (void)parameters;

  startIWDGTask();
  startSerial();

  startCLI();

  // Use USB for serial console if USB is connected on boot
  // Otherwise use ST-Link serial port
  if(usb_is_connected()) {
    startSerialConsole(&usbCLI);
    // Serial device will be enabled automatically when console connects
    // so no explicit serialEnable is required
  } else {
    startSerialConsole(&usart1);
    serialEnable(&usart1);
  }

  gpioISRStartTask();

  memfault_platform_boot();
  memfault_platform_start();

  bspInit();

  usbInit();

  debugSysInit();
  debugGpioInit(debugGpioPins, sizeof(debugGpioPins)/sizeof(DebugGpio_t));
  debugI2CInit(debugI2CInterfaces, sizeof(debugI2CInterfaces)/sizeof(DebugI2C_t));
  debugSPIInit(debugSPIInterfaces, sizeof(debugSPIInterfaces)/sizeof(DebugSPI_t));
  debugMemfaultInit(&usart1);

#ifdef USE_BOOTLOADER
  mcubootCliInit();
#endif

  // Commenting out while we test usart1
  // lpmPeripheralInactive(LPM_BOOT);

  while(1) {
    vTaskDelay(1000);
  }

}