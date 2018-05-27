/******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  *
  * Licensed under ST MYLIBERTY SOFTWARE LICENSE AGREEMENT (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/myliberty
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied,
  * AND SPECIFICALLY DISCLAIMING THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, AND NON-INFRINGEMENT.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
******************************************************************************/
/*
 *      PROJECT:
 *      $Revision: $
 *      LANGUAGE:  ANSI C
 */

/*! \file
 *
 *  \author Wolfgang Reichart
 *          M. Arpa
 *
 *  \brief UART streaming driver implementation.
 *
 */

/*
 ******************************************************************************
 * INCLUDES
 ******************************************************************************
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "st_stream.h"
#include "st_errno.h"
#include "uart_driver.h"
#include "stream_driver.h"
#include "stream_dispatcher.h"
#include "logger.h"

typedef enum {
	RX_IDLE=0,
	RX_HEADER_RECEIVED,
	RX_PAYLOAD_RECEIVED,
	RX_PKT_RECEIVED,
	RX_STATE_CNT
} RX_STATE;

/*
 *
 ******************************************************************************
 * LOCAL VARIABLES
 ******************************************************************************
 */
static uint8_t initalized = false;
static uint8_t rxState = RX_IDLE;
static uint8_t uartRxBuffer[USB_HID_REPORT_SIZE];
static uint8_t uartTxBuffer[USB_HID_REPORT_SIZE];

static uint8_t * rxBuffer;   /* INFO: buffer location is set in StreamInitialize */
static uint8_t * txBuffer;   /* INFO: buffer location is set in StreamInitialize */

static uint8_t txTid;
static uint8_t rxTid;
static uint16_t rxSize;
static uint8_t * rxEnd; /* pointer to next position where to copy the received data */


#define ioLedOn()     ;
#define ioLedOff()    ;

#define RX_TIMEOUT_MS 5
/*
 ******************************************************************************
 * GLOBAL FUNCTIONS
 ******************************************************************************
 */

void uartStreamInitialize (uint8_t * rxBuf, uint8_t * txBuf)
{

  /* setup pointers to buffers */
  rxBuffer = rxBuf;
  txBuffer = txBuf;
  /* so far we have received nothing */
  txTid = 0;
  rxTid = 0;
  rxSize = 0;
  rxEnd = rxBuffer;
  initalized = true;
}

void uartStreamConnect (void)
{
  /* dummy implementation */
}

void uartStreamDisconnect (void)
{
  /* dummy implementation */
}

uint8_t uartStreamReady (void)
{
  return initalized;
}

void uartStreamPacketProcessed ( uint16_t rxed )
{
  /*
   * FIXME: knowing that a memmove() is not the most performant variant,
   * it is used here to keep the code simpler.
   * If we run into performance issues on the PIC, this can be replaced
   * by pointer operations.
   */
  rxed += ST_STREAM_HEADER_SIZE;
  /* decrease remaining data length by length of consumed packet */
  rxSize -= rxed;

  memmove( rxBuffer, rxBuffer + rxed, rxSize );
  /* correct end pointer to the new end of the buffer */
  rxEnd = rxBuffer + rxSize;
}

int8_t uartStreamHasAnotherPacket ( )
{
  return (  rxSize >= ST_STREAM_HEADER_SIZE
            && rxSize >= ( ST_STREAM_DR_GET_RX_LENGTH( rxBuffer ) + ST_STREAM_HEADER_SIZE )
         );
}

uint16_t uartStreamOldFormatRequest ( )
{
  uint8_t protocol = uartRxBuffer[ 2 ];
  uint8_t toTx = uartRxBuffer[ 3 ];

  ioLedOn();

  /* send back the special answer */
  if ( toTx > 0 || ( protocol & 0x40 ) ) { /* response was required */

    /* wait here (and before copying the IN-buffer) until the USBInHandle is free again */


    uartTxBuffer[ 0 ] = ST_STREAM_COMPATIBILITY_TID;
    uartTxBuffer[ 1 ] = 0x03; /* payload */
    uartTxBuffer[ 2 ] = protocol;
    uartTxBuffer[ 3 ] = 0xFF; /* status = failed -> wrong protocol version */
    uartTxBuffer[ 4 ] = 0x00; /* no data will be sent back */

    /* initiate transfer now */
    uartTxNBytes(CTRL_UART, uartTxBuffer, USB_HID_REPORT_SIZE);
  }

  return 0;
}

uint16_t uartStreamReceive ( )
{
	static uint32_t rxStartTick=0;
	uint8_t lastState = rxState;
	uint32_t rxAvailable = uartRxBytesReadyForReceive(CTRL_UART);


	if ( (rxState>RX_IDLE) && (lastState == rxState))
	{
		uint32_t tick = HAL_GetTick();
		if (( (tick<rxStartTick) && (tick>RX_TIMEOUT_MS) ) ||
			  (tick>rxStartTick+RX_TIMEOUT_MS) )
		{
			platformLog("Killed transaction with spent time %d (%d, %d)ms\r\n", tick-rxStartTick,tick,rxStartTick);
			// Timeout: Reset everything
			uartRxNBytes(CTRL_UART, uartRxBuffer, rxAvailable);
			rxState = RX_IDLE;
			rxSize = 0;
			rxEnd = rxBuffer;
		}
	}
	if (rxAvailable == 0)
	{
		return 0;
	}
	if (rxState == RX_IDLE)
	{
		rxStartTick = HAL_GetTick();
		if (rxAvailable >= UART_HEADER_SIZE)
		{
			uint32_t rxLen = uartRxNBytes(CTRL_UART, uartRxBuffer, UART_HEADER_SIZE);
			rxAvailable -= rxLen;
	        rxTid = UART_TID( uartRxBuffer );
			rxState = RX_HEADER_RECEIVED;
			platformLog("Start transaction @ %d ms\r\n", rxStartTick);
		}
	}
	if ( ( rxState == RX_HEADER_RECEIVED) && (rxAvailable>0))
	{
         /*
           * Read the uart  buffer into the local buffer.
           * If received data is not as long as header and payload size
           * do not return any data
           */
        uint16_t rxToRead;
        uint16_t payload = UART_GET_PAYLOAD_SIZE( uartRxBuffer );

#if 0
        if ( UART_STATUS( uartRxBuffer ) != 0 ) {
			/* this is a request in the old format */
			/* in the old format at this position we had the protocol id - which was never 0
				 in the new format here this uint8_t is reserved and 0 when sent from host to device */
			return uartStreamOldFormatRequest( );
        }
#endif
        ioLedOn();
        rxToRead = rxAvailable;
        if (rxAvailable+rxSize >= payload)
        {
        	rxToRead = payload-rxSize;
			rxState = RX_PAYLOAD_RECEIVED;
        }
        rxAvailable -= rxToRead;
        uartRxNBytes(CTRL_UART, rxEnd, rxToRead);
        rxSize += rxToRead;
        rxEnd += rxToRead;
	}
	if ( rxState == RX_PAYLOAD_RECEIVED )
	{
		/* Do not check for available bit as all bits are maybe already read. */
		uint16_t packetSize = ST_STREAM_DR_GET_RX_LENGTH( rxBuffer ) + ST_STREAM_HEADER_SIZE;
		if (rxAvailable+rxSize >= packetSize)
		{
			rxAvailable = packetSize-rxSize;
			rxState = RX_PKT_RECEIVED;
		}
		else
		{
			uartRxNBytes(CTRL_UART, rxEnd, rxAvailable);
			rxSize += rxAvailable;
			rxEnd += rxAvailable;
		}
	}
	if ( rxState == RX_PKT_RECEIVED )
	{
		ioLedOff();
		uint32_t tick = HAL_GetTick();
		rxEnd = rxBuffer;   /* next time we receive new data, we start over at buffer start */
        rxState = RX_IDLE;
		platformLog("Finished transaction @ %d ms (%d ms)\r\n", tick, tick-rxStartTick);
        return rxSize;
    }
	if (lastState == rxState)
	{
		uint32_t tick = HAL_GetTick();
		if (( (tick<rxStartTick) && (tick>RX_TIMEOUT_MS) ) ||
			  (tick>rxStartTick+RX_TIMEOUT_MS) )
		{
			platformLog("Killed transaction with spent time %d ms\r\n", tick-rxStartTick);
			// Timeout: Reset everything
			uartRxNBytes(CTRL_UART, uartRxBuffer, rxAvailable);
			rxState = RX_IDLE;
			rxSize = 0;
			rxEnd = rxBuffer;
		}

	}
    /* indicate that we did not receive anything - try next time again */
    return 0;
}

void uartStreamTransmit ( uint16_t packetSize )
{
	if (packetSize>0)
	{
		ioLedOn( );

		/* wait here (and before copying the IN-buffer) until the Uart is free again */
		while ( uartMaxTxBytes(CTRL_UART) == 0 )
		  ;
		/* generate a new tid for tx */
		UART_GENERATE_TID_FOR_TX( rxTid, txTid );

		/* TX-packet setup */
		UART_TID( uartTxBuffer )          = txTid;
		UART_STATUS( uartTxBuffer )       = StreamDispatcherGetLastError();
		UART_SET_PAYLOAD_SIZE( uartTxBuffer, packetSize);

		/* copy data to uart buffer */
		memcpy( UART_PAYLOAD( uartTxBuffer ), txBuffer, packetSize );

		/* initiate transfer now */
		uartTxNBytes(CTRL_UART, uartTxBuffer, UART_HEADER_SIZE+packetSize);

		ioLedOff( );
	}
}
