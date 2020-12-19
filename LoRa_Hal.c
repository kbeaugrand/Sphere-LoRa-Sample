#include <errno.h>
#include <stdio.h> 
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>

#include <applibs/log.h>
#include <applibs/uart.h>
#include <applibs/gpio.h>

#include "peripheral_utilities.h"
#include "LoRa_ChipConfig.h"

static int UART_FD;
static int RST_FD;
static int CS_FD;

/** @defgroup LORA_HAL_UART HAL UART Interface */             /** @{ */

/**
 * @brief Map UART Function Pointers
 */
bool LoRa_hal_uartMap(void) {
  // Create a UART_Config object, open the UART and set up UART event handler
  UART_Config uartConfig;
  UART_InitConfig(&uartConfig);
    
  uartConfig.baudRate = 57600;
  uartConfig.dataBits = UART_DataBits_Eight;
  uartConfig.parity = UART_Parity_None;
  uartConfig.stopBits = UART_StopBits_One;
  uartConfig.flowControl = UART_FlowControl_None;

  UART_FD = UART_Open(LORA_UART_RXTX, &uartConfig);

  if (UART_FD == -1) {
    Log_Debug("ERROR: Could not open UART: %s (%d).\n", strerror(errno), errno);
    return false;
  }

  return true;
}

/**
 * @brief Map UART GPIO Pointers (CS, RST Pin)
 */
bool LoRa_hal_gpio_gpioMap(void){ 
  RST_FD = GPIO_OpenAsOutput(LORA_UART_RST, GPIO_OutputMode_PushPull, GPIO_Value_High);

  if (RST_FD == -1) {
    Log_Debug("ERROR: Could not open rst GPIO: %s (%d).\n", strerror(errno), errno);
    return false;
  }

  CS_FD = GPIO_OpenAsOutput(LORA_UART_CS, GPIO_OutputMode_PushPull, GPIO_Value_Low);

  if (CS_FD == -1) {
      Log_Debug("ERROR: Could not open cst GPIO: %s (%d).\n", strerror(errno), errno);
      return false;
  }

  return true;
}

/**
 * @brief Closes the LoRa UAR and GPIO Pointers
 */
void LoRa_hal_close(void)
{
  CloseFdAndPrintError(UART_FD, "LORA_UART_RXTX");
  CloseFdAndPrintError(CS_FD, "LORA_UART_CS");
  CloseFdAndPrintError(RST_FD, "LORA_UART_RST");
}

/**
 * @brief Sets the CS Pin at the input level
 */
void LoRa_hal_gpio_csSet(uint8_t input){ 
  GPIO_SetValue(CS_FD, input);
}

/**
 * @brief Sets the RST Pin at the input level
 */
void LoRa_hal_gpio_rstSet(uint8_t input){ 
  GPIO_SetValue(RST_FD, input);
}

/**
 * @brief hal_uartWrite
 *
 * @param[in] input tx data byte
 *
 * Function writes one byte on UART.
 */
void LoRa_hal_uartWrite(uint8_t input) {
  write(UART_FD, &input, 1);
}

/**
 * @brief hal_uartRead
 *
 * @return rx data byte
 *
 * Function reads one byte.
 */
ssize_t LoRa_hal_uartRead(uint8_t *ret)
{
  return read(UART_FD, ret, 1);
}
