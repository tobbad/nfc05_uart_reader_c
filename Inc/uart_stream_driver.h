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
 *
 *  \brief UART streaming driver declarations.
 *
 */

/*!
 *
 *
 */
#ifndef _UART_STREAM_DRIVER_H
#define _UART_STREAM_DRIVER_H

/*
 ******************************************************************************
 * INCLUDES
 ******************************************************************************
 */
#include <stdint.h>
#include "st_stream.h"
#include "stream_driver.h"

/*
 ******************************************************************************
 * GLOBAL FUNCTIONS
 ******************************************************************************
 */
typedef enum {
    UART_OK = 0,
} UART_RES;

/*!
 *****************************************************************************
 *  \brief  initializes the UART Stream driver variables
 *
 *  This function takes care for proper initialisation of buffers, variables, etc.
 *
 *  \param rxBuf : buffer where received packets will be copied into
 *  \param txBuf : buffer where to be transmitted packets will be copied into
 *****************************************************************************
 */
void uartStreamInitialize (uint8_t * rxBuf, uint8_t * txBuf);

/*!
 *****************************************************************************
 *  \brief  initialises the uart and connects to host
 *****************************************************************************
 */
void uartStreamConnect(void);

/*!
 *****************************************************************************
 *  \brief  disconnects from host
 *****************************************************************************
 */
void uartStreamDisconnect(void);

/*!
 *****************************************************************************
 *  \brief  returns 1 if stream init is finished
 *
 *  \return 0=init has not finished yet, 1=stream has been initialized
 *****************************************************************************
 */
uint8_t uartStreamReady(void);

/*!
 *****************************************************************************
 *  \brief  tells the stream driver that the packet has been processed and can
 *   be moved from the rx buffer
 *
 *  \param rxed : number of bytes which have been processed
 *****************************************************************************
 */
void uartStreamPacketProcessed (uint16_t rxed);

/*!
 *****************************************************************************
 *  \brief  returns 1 if another packet is available in buffer
 *
 *  \return 0=no packet available in buffer, 1=another packet available
 *****************************************************************************
 */
int8_t uartStreamHasAnotherPacket (void);

/*!
 *****************************************************************************
 *  \brief checks if there is data received on the UART device from the host
 *  and copies the received data into a local buffer
 *
 *  Checks if the UART device has data received from the host, copies this
 *  data into a local buffer. The data in the local buffer is than interpreted
 *  as a packet (with header, rx-length and tx-length). As soon as a full
 *  packet is received the function returns non-null.
 *
 *  \return 0 = nothing to process, >0 at least 1 packet to be processed
 *****************************************************************************
 */
uint16_t uartStreamReceive (void);

/*!
 *****************************************************************************
 *  \brief checks if there is data to be transmitted from the UART device to
 *  the host.
 *
 *  Checks if there is data waiting to be transmitted to the host. Copies this
 *  data from a local buffer to the uart buffer and transmits this buffer.
 *  If more than 1 uart report is needed to transmit the data, the function
 *  waits until the first one is sent, and than refills the uart buffer with
 *  the next chunk of data and transmits the buffer again. And so on, until
 *  all data is sent.
 *
 *  \param [in] totalTxSize: the size of the data to be transmitted (the UART
 *  header is not included)
 *****************************************************************************
 */
void uartStreamTransmit( uint16_t totalTxSize );

#endif // _UART_STREAM_DRIVER_H

