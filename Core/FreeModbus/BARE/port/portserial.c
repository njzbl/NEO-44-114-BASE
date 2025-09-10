/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id$
 */

#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "stm32g0xx_hal.h"

#include "main.h"
#include "usart.h"
#include "gpio.h"

/* ----------------------- static functions ---------------------------------*/
// static void prvvUARTTxReadyISR( void );
// static void prvvUARTRxISR( void );

/* ----------------------- Start implementation -----------------------------*/
void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
  if(xRxEnable)
	{
			HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_RESET); //MAX485�����͵�ƽΪ����ģʽ
		__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);		 //ʹ�ܽ��ռĴ����ǿ��ж�
		
  }
	else
    {
        __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);		//���ܽ��ռĴ����ǿ��ж�		
        HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_SET); //MAX485���� �ߵ�ƽΪ����ģʽ
    }
    if (xTxEnable)
    {
        //HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_SET); //MAX485���� �ߵ�ƽΪ����ģʽ
        __HAL_UART_ENABLE_IT(&huart1, UART_IT_TXE);      //ʹ�ܷ�������ж�
			
    }
    else
    {
       __HAL_UART_DISABLE_IT(&huart1, UART_IT_TXE);   //���ܷ�������ж�
    }
}

    /* If xRXEnable enable serial receive interrupts. If xTxENable enable
     * transmitter empty interrupts.
     */


BOOL
xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
	
		//MX_USART2_UART_Init();

		//MX_GPIO_Init();
		return TRUE;
}

BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
	 HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_SET);
		//USART2->TDR = ucByte;
	if(HAL_UART_Transmit(&huart1,(uint8_t *)&ucByte,1,1) != HAL_OK)
		return FALSE;
		else
    /* Put a byte in the UARTs transmit buffer. This function is called
     * by the protocol stack if pxMBFrameCBTransmitterEmpty( ) has been
     * called. */
    return TRUE;
}

BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
		HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_RESET);
	 //*pucByte = (USART2->RDR & (uint16_t)0x00FF);
    /* Return the byte in the UARTs receive buffer. This function is called
     * by the protocol stack after pxMBFrameCBByteReceived( ) has been called.
     */
	if(HAL_UART_Receive(&huart1,(uint8_t *)pucByte,1,1)!= HAL_OK)
		return FALSE;
	else
    return TRUE;
}

/* Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call 
 * xMBPortSerialPutByte( ) to send the character.
 */
void prvvUARTTxReadyISR( void )
{
    pxMBFrameCBTransmitterEmpty(  );
	
}

/* Create an interrupt handler for the receive interrupt for your target
 * processor. This function should then call pxMBFrameCBByteReceived( ). The
 * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
 * character.
 */
void prvvUARTRxISR( void )
{
    pxMBFrameCBByteReceived(  );
}


