/*!
 *****************************************************************************
 * @file:    Communications.c
 * @brief:
 * @version: $Revision$
 * @date:    $Date$
 *-----------------------------------------------------------------------------
 *
Copyright (c) 2015-2017 Analog Devices, Inc.

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
  - Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
  - Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
  - Modified versions of the software must be conspicuously marked as such.
  - This software is licensed solely and exclusively for use with processors
    manufactured by or for Analog Devices, Inc.
  - This software may not be combined or merged with other code in any manner
    that would cause the software to become subject to terms and conditions
    which differ from those listed here.
  - Neither the name of Analog Devices, Inc. nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.
  - The use of this software may or may not infringe the patent rights of one
    or more patent holders.  This license does not release you from the
    requirement that you obtain separate licenses from these patent holders
    to use this software.

THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
TITLE, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
NO EVENT SHALL ANALOG DEVICES, INC. OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, PUNITIVE OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, DAMAGES ARISING OUT OF CLAIMS OF INTELLECTUAL
PROPERTY RIGHTS INFRINGEMENT; PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

/***************************** Include Files **********************************/
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>

// #include "ADuCM360.h"
// #include "SpiLib.h"
// #include "DioLib.h"
// #include "UrtLib.h"

#include "Communication.h"

#include "spi.h"
#include "gpio.h"
/************************* Functions Definitions ******************************/

/**
   @brief Writes a data, a command or a register to the LCD or to ACC via SPI.

   @param ui8address - ACC register address
   @param ui8Data - value to be written in 1 register write
   @param ui8Data2 - 2nd value to be written in 2 register write
   @enMode enWriteData - write mode

   @return none

**/
void SPI_Write(uint8_t ui8address, uint8_t ui8Data, uint8_t ui8Data2, enWriteData enMode)
{
   uint8_t ui8writeAddress;
   ui8writeAddress = ((ui8address << 1) | ADXL355_WRITE);
   uint8_t dat[3];
   if (enMode == SPI_WRITE_ONE_REG)
   {

      HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET); /* Select accelerometer */

      //SpiFifoFlush(pADI_SPI0, SPICON_TFLUSH_EN, SPICON_RFLUSH_EN); /* Flush Tx and Rx FIFOs */

      //SpiTx(pADI_SPI0, ui8writeAddress); /* Send register address */

      //SpiTx(pADI_SPI0, ui8Data); /* Send value to be written */

      dat[0] = ui8writeAddress;
      dat[1] = ui8Data;
      HAL_SPI_Transmit(&hspi2, dat, 2, 500);

      // while ((SpiSta(pADI_SPI0) & SPI0STA_RXFSTA_TWOBYTES) != SPI0STA_RXFSTA_TWOBYTES)
      //    ; /* Wait until 3 bytes are received */

      HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET); /* Deselect accelerometer */
   }
   if (enMode == SPI_WRITE_TWO_REG)
   {
      HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET); /* Select accelerometer */

      //SpiFifoFlush(pADI_SPI0, SPICON_TFLUSH_EN, SPICON_RFLUSH_EN); /* Flush Tx and Rx FIFOs */

      // SpiTx(pADI_SPI0, ui8writeAddress); /* Send register address */

      // SpiTx(pADI_SPI0, ui8Data); /* Send 1st value to be written */

      // SpiTx(pADI_SPI0, ui8Data2); /* Send 2nd value to be written */

      dat[0] = ui8writeAddress;
      dat[1] = ui8Data;
      dat[2] = ui8Data2;
      HAL_SPI_Transmit(&hspi2, dat, 3, 500);

      // while ((SpiSta(pADI_SPI0) & SPI0STA_RXFSTA_THREEBYTES) != SPI0STA_RXFSTA_THREEBYTES)
      //    ; /* Wait until 3 bytes are received */

      HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET); /* Deselect accelerometer */
   }
}

/**
   @brief Reads a specified register or two registers address in the accelerometer via SPI.

   @param ui8address - register address
   @param enRegs - register number

   @return reading result

**/
uint32_t SPI_Read(uint8_t ui8address, enRegsNum enRegs)
{
   uint8_t dat[4];
   uint32_t ui32Result = 0;
   dat[0] = ((ui8address << 1) | ADXL355_READ);

   HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET); /* Select accelerometer */

   if (enRegs == SPI_READ_ONE_REG)
   {
      HAL_SPI_TransmitReceive(&hspi1, dat, dat, 2, 500);
      ui32Result=dat[1];
   }
   if (enRegs == SPI_READ_TWO_REG)
   { /* Only used for Temp & X,Y,Z offset and threshold registers*/
      HAL_SPI_TransmitReceive(&hspi1, dat, dat, 3, 500);
      ui32Result = ((dat[1] << 8) | dat[2]); /* Set read result*/
   }
   if (enRegs == SPI_READ_THREE_REG)
   { /* Only used for X,Y,Z axis data registers*/
      HAL_SPI_TransmitReceive(&hspi1, dat, dat, 4, 500);
      ui32Result = ((dat[1] << 16) | (dat[2] << 8) | dat[3]); /* Set read result*/
   }

   HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET); /* Deselect accelerometer */
   return ui32Result;
}