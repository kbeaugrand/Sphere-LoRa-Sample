
#pragma once

#include <stdint.h>

/**
 * @brief Map UART Function Pointers
 */
bool LoRa_hal_uartMap(void);

/**
 * @brief Closes the LoRa UAR and GPIO Pointers
 */
void LoRa_hal_close(void);

/**
 * @brief Map UART GPIO Pointers (CS, RST Pin)
 */
bool LoRa_hal_gpio_gpioMap(void);

/**
 * @brief Sets the CS Pin at the input level
 */
void LoRa_hal_gpio_csSet(uint8_t input);

/**
 * @brief Sets the RST Pin at the input level
 */
void LoRa_hal_gpio_rstSet(uint8_t input);

/**
 * @brief hal_uartWrite
 *
 * @param[in] input tx data byte
 *
 * Function writes one byte on UART.
 */
void LoRa_hal_uartWrite(uint8_t input);

/**
 * @brief hal_uartRead
 *
 * @return rx data byte
 *
 * Function reads one byte.
 */
ssize_t LoRa_hal_uartRead(uint8_t *ret);