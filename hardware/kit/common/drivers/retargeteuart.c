/***************************************************************************//**
 * @file
 * @brief Provide stdio retargeting to EUART on BG22
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/

#include <stdio.h>
#include "em_device.h"
#include "em_cmu.h"
#include "em_core.h"
#include "em_gpio.h"
#include "retargetserial.h"

#if defined(HAL_CONFIG)
#include "retargetserialhalconfig.h"
#else
#include "retargetserialconfig.h"
#endif

/***************************************************************************//**
 * @addtogroup kitdrv
 * @{
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup RetargetIo
 * @{
 ******************************************************************************/

#include "em_eusart.h"

/* Receive buffer */
#ifndef RXBUFSIZE
#define RXBUFSIZE    8                          /**< Buffer size for RX */
#endif
static volatile int     rxReadIndex  = 0;       /**< Index in buffer to be read */
static volatile int     rxWriteIndex = 0;       /**< Index in buffer to be written to */
static volatile int     rxCount      = 0;       /**< Keeps track of how much data which are stored in the buffer */
static volatile uint8_t rxBuffer[RXBUFSIZE];    /**< Buffer to store data */
static uint8_t          LFtoCRLF    = 0;        /**< LF to CRLF conversion disabled */
static bool             initialized = false;    /**< Initialize UART/LEUART */

/**************************************************************************//**
 * @brief Disable RX interrupt
 *****************************************************************************/
static void disableRxInterrupt()
{
	EUSART_IntDisable(EUART0, EUSART_IF_RXFLIF);
}

/**************************************************************************//**
 * @brief Enable RX interrupt
 *****************************************************************************/
static void enableRxInterrupt()
{
	EUSART_IntEnable(EUART0, EUSART_IF_RXFLIF);
}

/**************************************************************************//**
 * @brief UART/LEUART IRQ Handler
 *****************************************************************************/
void EUART0_RX_IRQHandler(void)
{
	uint32_t flags = EUSART_IntGet(EUART0);
	if (flags & EUSART_IF_RXFLIF) {
		if (rxCount < RXBUFSIZE) {
			/* There is room for data in the RX buffer so we store the data. */
			rxBuffer[rxWriteIndex] = EUSART_Rx(EUART0);
			rxWriteIndex++;
			rxCount++;
			if (rxWriteIndex == RXBUFSIZE) {
				rxWriteIndex = 0;
			}
		} else {
			/* The RX buffer is full so we must wait for the RETARGET_ReadChar()
			 * function to make some more room in the buffer. RX interrupts are
			 * disabled to let the ISR exit. The RX interrupt will be enabled in
			 * RETARGET_ReadChar(). */
			disableRxInterrupt();
		}
	}
	EUSART_IntClear(EUART0, flags);
	// gecko_external_signal(1);
}

/**************************************************************************//**
 * @brief UART/LEUART toggle LF to CRLF conversion
 * @param on If non-zero, automatic LF to CRLF conversion will be enabled
 *****************************************************************************/
void RETARGET_SerialCrLf(int on)
{
  if (on) {
    LFtoCRLF = 1;
  } else {
    LFtoCRLF = 0;
  }
}

/**************************************************************************//**
 * @brief Intializes UART/LEUART
 *****************************************************************************/
void RETARGET_SerialInit(void)
{
	/* Enable peripheral clocks */
	CMU->EUART0CLKCTRL = 2;  // EM23GRPACLK is clocking UART
	/* Configure GPIO pins */
	CMU_ClockEnable(cmuClock_GPIO, true);
	/* To avoid false start, configure output as high */
	GPIO_PinModeSet(RETARGET_TXPORT, RETARGET_TXPIN, gpioModePushPull, 1);
	GPIO_PinModeSet(RETARGET_RXPORT, RETARGET_RXPIN, gpioModeInputPull, 1);

	EUSART_UartInit_TypeDef init = EUSART_UART_INIT_DEFAULT_LF;
	/* Enable DK RS232/UART switch */
	RETARGET_PERIPHERAL_ENABLE();

	CMU_ClockEnable(cmuClock_EUART0, true);

	/* Configure USART for basic async operation */
	init.enable = eusartDisable;
	EUSART_UartInitLf(EUART0, &init);

	/* Enable pins at correct UART/USART location. */
	GPIO->EUARTROUTE[0].ROUTEEN = GPIO_EUART_ROUTEEN_TXPEN;
	GPIO->EUARTROUTE[0].TXROUTE =
			(RETARGET_TXPORT << _GPIO_EUART_TXROUTE_PORT_SHIFT)
			| (RETARGET_TXPIN << _GPIO_EUART_TXROUTE_PIN_SHIFT);
	GPIO->EUARTROUTE[0].RXROUTE =
			(RETARGET_RXPORT << _GPIO_EUART_RXROUTE_PORT_SHIFT)
			| (RETARGET_RXPIN << _GPIO_EUART_RXROUTE_PIN_SHIFT);


	/* Clear previous RX interrupts */
	EUSART_IntClear(EUART0, EUSART_IF_RXFLIF);
	NVIC_ClearPendingIRQ(EUART0_RX_IRQn);

	/* Enable RX interrupts */
	EUSART_IntEnable(EUART0, EUSART_IF_RXFLIF);
	NVIC_EnableIRQ(EUART0_RX_IRQn);

	/* Finally enable it */
	EUSART_Enable(EUART0, eusartEnable);

#if !defined(__CROSSWORKS_ARM) && defined(__GNUC__)
	setvbuf(stdout, NULL, _IONBF, 0);   /*Set unbuffered mode for stdout (newlib)*/
#endif

  initialized = true;
}

/**************************************************************************//**
 * @brief Receive a byte from USART/LEUART and put into global buffer
 * @return -1 on failure, or positive character integer on sucesss
 *****************************************************************************/
int RETARGET_ReadChar(void)
{
  int c = -1;
  CORE_DECLARE_IRQ_STATE;

  if (initialized == false) {
    RETARGET_SerialInit();
  }

  CORE_ENTER_ATOMIC();
  if (rxCount > 0) {
    c = rxBuffer[rxReadIndex];
    rxReadIndex++;
    if (rxReadIndex == RXBUFSIZE) {
      rxReadIndex = 0;
    }
    rxCount--;
    /* Unconditionally enable the RX interrupt. RX interrupts are disabled when
     * a buffer full condition is entered. This way flow control can be handled
     * automatically by the hardware. */
    enableRxInterrupt();
  }

  CORE_EXIT_ATOMIC();

  return c;
}

/**************************************************************************//**
 * @brief Transmit single byte to USART/LEUART
 * @param c Character to transmit
 * @return Transmitted character
 *****************************************************************************/
int RETARGET_WriteChar(char c)
{
  if (initialized == false) {
    RETARGET_SerialInit();
  }

  /* Add CR or LF to CRLF if enabled */
  if (LFtoCRLF && (c == '\n')) {
	  EUSART_Tx(EUART0, '\r');
  }
  EUSART_Tx(EUART0, c);

  return c;
}

/**************************************************************************//**
 * @brief Enable hardware flow control. (RTS + CTS)
 * @return true if hardware flow control was enabled and false otherwise.
 *****************************************************************************/
bool RETARGET_SerialEnableFlowControl(void)
{
  GPIO_PinModeSet(RETARGET_CTSPORT, RETARGET_CTSPIN, gpioModeInputPull, 0);
  GPIO_PinModeSet(RETARGET_RTSPORT, RETARGET_RTSPIN, gpioModePushPull, 0);
  GPIO->EUARTROUTE[0].CTSROUTE = (RETARGET_CTSPORT << _GPIO_EUART_CTSROUTE_PORT_SHIFT)
					| (RETARGET_CTSPIN << _GPIO_EUART_CTSROUTE_PIN_SHIFT);
  GPIO->EUARTROUTE[0].RTSROUTE = (RETARGET_RTSPORT << _GPIO_EUART_RTSROUTE_PORT_SHIFT)
							| (RETARGET_RTSPIN << _GPIO_EUART_RTSROUTE_PIN_SHIFT);
  GPIO->EUARTROUTE[0].ROUTEEN |= GPIO_EUART_ROUTEEN_RTSPEN;
  return true;
}

/**************************************************************************//**
 * @brief Flush UART/LEUART
 *****************************************************************************/
void RETARGET_SerialFlush(void)
{
  while(!(EUART0->STATUS & EUSART_STATUS_TXIDLE));
}

/** @} (end group RetargetIo) */
/** @} (end group kitdrv) */
