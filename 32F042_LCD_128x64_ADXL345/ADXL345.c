/*
 *
 * Modified by Alex Sedyshev for STM32F042x6.. (CMSIS )
 *  https://github.com/AlekseySedyshev
 * 
*/
/***************************************************************************
 *   @file   ADXL345.c
 *   @brief  Implementation of ADXL345 Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
****************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
********************************************************************************
 *   SVN Revision: 684
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "stm32f0xx.h"
#include "ADXL345.h"

void 			writeReg8(unsigned char reg, unsigned char value)																						{//Write a 8-bit register
	while(I2C1->ISR & I2C_ISR_BUSY){}; //I2C_ISR_BUSY
	I2C1->CR2 &= ~(I2C_CR2_NBYTES)  & (~I2C_CR2_SADD) & (~I2C_CR2_RD_WRN); 	
 	I2C1->CR2 |= (ADXL345_ADDRESS << I2C_CR2_SADD_Pos) | (0x80 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
	while((I2C1->ISR & I2C_ISR_TC)){}; 	
	while(!(I2C1->ISR & I2C_ISR_TXE)){}; I2C1->TXDR = reg;	while((I2C1->ISR & I2C_ISR_TC)){}; 	 
	while(!(I2C1->ISR & I2C_ISR_TXE)){}; I2C1->TXDR = value;	while((I2C1->ISR & I2C_ISR_TC)){}; 
	while(!(I2C1->ISR & I2C_ISR_TXE)){}; // TX buf Empty
	I2C1->CR2 |= I2C_CR2_STOP;
	while(!(I2C1->ISR & I2C_ISR_STOPF)){};
	I2C1->ICR |= I2C_ICR_STOPCF;	
}
unsigned char 	readReg8(unsigned char reg)																														{//Read an 8-bit register
	
	while(I2C1->ISR & I2C_ISR_BUSY){}; //I2C_ISR_BUSY
	I2C1->CR2 &= ~(I2C_CR2_NBYTES)  & (~I2C_CR2_SADD) & (~I2C_CR2_RD_WRN); 	
 	I2C1->CR2 |= (ADXL345_ADDRESS << I2C_CR2_SADD_Pos) | (0x80 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
	while((I2C1->ISR & I2C_ISR_TC)){}; 	
	while(!(I2C1->ISR & I2C_ISR_TXE)){}; I2C1->TXDR = reg;	while((I2C1->ISR & I2C_ISR_TC)){};
	while(!(I2C1->ISR & I2C_ISR_TXE)){}; // TX buf Empty
	I2C1->CR2 |= I2C_CR2_STOP;
	while(!(I2C1->ISR & I2C_ISR_STOPF)){};
	I2C1->ICR |= I2C_ICR_STOPCF;	
	while(I2C1->ISR & I2C_ISR_BUSY){}; //I2C_ISR_BUSY
	I2C1->CR2 &= ~(I2C_CR2_NBYTES)  & (~I2C_CR2_SADD);	
	I2C1->CR2 |= I2C_CR2_RD_WRN;																				// Direction - data in
	I2C1->CR2 |= (ADXL345_ADDRESS << I2C_CR2_SADD_Pos) | (1 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
	while((I2C1->ISR & I2C_ISR_TC)){}; 
	while(!(I2C1->ISR & I2C_ISR_RXNE)){};		
	unsigned char value=I2C1->RXDR;
	while(!(I2C1->ISR & I2C_ISR_TXE)){};
	I2C1->CR2 |= I2C_CR2_STOP;
	while(!(I2C1->ISR & I2C_ISR_STOPF)){};
	I2C1->ICR |= I2C_ICR_STOPCF;
	I2C1->CR2 &=(~I2C_CR2_RD_WRN) &(~I2C_CR2_NACK);
	return value; 		
}





void readMulti(unsigned char reg, unsigned char * dst, unsigned char count)																	{// readMulti
	while(I2C1->ISR & I2C_ISR_BUSY){}; //I2C_ISR_BUSY
	I2C1->CR2 &= ~(I2C_CR2_NBYTES)  & (~I2C_CR2_SADD) & (~I2C_CR2_RD_WRN); 	
 	I2C1->CR2 |= (ADXL345_ADDRESS << I2C_CR2_SADD_Pos) | (0xff << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
	while((I2C1->ISR & I2C_ISR_TC)){}; 	
	while(!(I2C1->ISR & I2C_ISR_TXE)){}; I2C1->TXDR = reg;	while((I2C1->ISR & I2C_ISR_TC)){}; 	
	while(!(I2C1->ISR & I2C_ISR_TXE)){};
	I2C1->CR2 |= I2C_CR2_STOP;
	while(!(I2C1->ISR & I2C_ISR_STOPF)){};
	I2C1->ICR |= I2C_ICR_STOPCF;
	while(I2C1->ISR & I2C_ISR_BUSY){}; //I2C_ISR_BUSY	
	I2C1->CR2 &= ~(I2C_CR2_NBYTES)  & (~I2C_CR2_SADD);	
	I2C1->CR2 |= I2C_CR2_RD_WRN;																				// Direction - data in
	I2C1->CR2 |= (ADXL345_ADDRESS << I2C_CR2_SADD_Pos) | (count << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
	while((I2C1->ISR & I2C_ISR_TC)){}; 
	while (count-- > 0) {
		
		while(!(I2C1->ISR & I2C_ISR_RXNE)){};
		*(dst++) = I2C1->RXDR;
	}
	while(!(I2C1->ISR & I2C_ISR_TXE)){};
	I2C1->CR2 |= I2C_CR2_STOP;
	while(!(I2C1->ISR & I2C_ISR_STOPF)){};
	I2C1->ICR |= I2C_ICR_STOPCF;
	I2C1->CR2 &=(~I2C_CR2_RD_WRN) &(~I2C_CR2_NACK);
}

/***************************************************************************//**
 * @brief Initializes the communication peripheral and checks if the ADXL345
 *          part is present.
 *
 * @return status - Result of the initialization procedure.
 *                  Example: 0x0 - I2C peripheral was not initialized or
 *                                 ADXL345 part is not present.
 *                           0x1 - I2C peripheral is initialized and ADXL345
 *                                 part is present.
*******************************************************************************/
unsigned char ADXL345_Init(void)	{
  if(readReg8(ADXL345_DEVID)!= ADXL345_ID) { return 0; }
		
	ADXL345_SetPowerMode(0) ;
	//(ADXL345_RANGE_PM_2G /ADXL345_RANGE_PM_4G /ADXL345_RANGE_PM_8G /ADXL345_RANGE_PM_16G
	//    | ADXL345_INT_INVERT |
	ADXL345_SetDataFormat(  ADXL345_RANGE(ADXL345_RANGE_PM_2G) | ADXL345_INT_INVERT | ADXL345_FULL_RES  ) ;//0x6b
	ADXL345_SetPowerMode(1) ;
	return 1;
}

/***************************************************************************//**
 * @brief Places the device into standby/measure mode.
 *
 * @param pwrMode - Power mode.
 *                    Example: 0x0 - standby mode.
 *                             0x1 - measure mode.
 *
 * @return None.
*******************************************************************************/
void ADXL345_SetPowerMode(unsigned char pwrMode)		{
    
    if (pwrMode) {
				writeReg8(ADXL345_POWER_CTL, readReg8(ADXL345_POWER_CTL) | ADXL345_PCTL_MEASURE );
		}
		else {
				writeReg8(ADXL345_POWER_CTL, readReg8(ADXL345_POWER_CTL)  & (~ADXL345_PCTL_MEASURE) );
		}
    
}

/***************************************************************************//**
 * @brief Reads the output data of each axis.
 *
 * @param x - X-axis's output data.
 * @param y - Y-axis's output data.
 * @param z - Z-axis's output data.
 *
 * @return None.
*******************************************************************************/
void ADXL345_GetXyz(short* x, short* y, short* z)	{

	unsigned char data_array[6];
	readMulti(ADXL345_DATAX0, data_array, 6);
			
		*x =(short) ( data_array[1] << 8 | data_array[0]);
    *y =(short) ( data_array[3] << 8 | data_array[2]);
    *z =(short) ( data_array[5] << 8 | data_array[4]);

}

/***************************************************************************//**
 * @brief Enables/disables the tap detection.
 *
 * @param tapType - Tap type (none, single, double).
 *                    Example: 0x0 - disables tap detection.
 *                             ADXL345_SINGLE_TAP - enables single tap detection.
 *                             ADXL345_DOUBLE_TAP - enables double tap detection.
 * @param tapAxes - Axes which participate in tap detection.
 *                    Example: 0x0 - disables axes participation.
 *                             ADXL345_TAP_X_EN - enables x-axis participation.
 *                             ADXL345_TAP_Y_EN - enables y-axis participation.
 *                             ADXL345_TAP_Z_EN - enables z-axis participation.
 * @param tapDur - Tap duration.
 * @param tapLatent - Tap latency.
 * @param tapWindow - Tap window. 
 * @param tapThresh - Tap threshold.
 * @param tapInt - Interrupts pin.
 *                   Example: 0x0 - interrupts on INT1 pin.
 *                            ADXL345_SINGLE_TAP - single tap interrupts on
 *                                                 INT2 pin.
 *                            ADXL345_DOUBLE_TAP - double tap interrupts on
 *                                                 INT2 pin.
 *
 * @return None.
*******************************************************************************/
void ADXL345_SetTapDetection(unsigned char tapType, unsigned char tapAxes, unsigned char tapDur, unsigned char tapLatent,
                             unsigned char tapWindow, unsigned char tapThresh, unsigned char tapInt) {
    unsigned char newTapAxes    = 0;
    unsigned char newIntMap     = 0;
    unsigned char newIntEnable  = 0;
    
    newTapAxes = (readReg8(ADXL345_TAP_AXES) & ~(ADXL345_TAP_X_EN | ADXL345_TAP_Y_EN | ADXL345_TAP_Z_EN));
    writeReg8(ADXL345_TAP_AXES, newTapAxes | tapAxes);
	
    writeReg8(ADXL345_DUR, tapDur);
    writeReg8(ADXL345_LATENT, tapLatent);
    writeReg8(ADXL345_WINDOW, tapWindow);
    writeReg8(ADXL345_THRESH_TAP, tapThresh);
    
		newIntMap = (readReg8(ADXL345_INT_MAP) & ~(ADXL345_SINGLE_TAP | ADXL345_DOUBLE_TAP));
    writeReg8(ADXL345_INT_MAP, newIntMap | tapInt);
	
    newIntEnable = (readReg8(ADXL345_INT_ENABLE) & ~(ADXL345_SINGLE_TAP | ADXL345_DOUBLE_TAP));
    writeReg8(ADXL345_INT_ENABLE, newIntEnable | tapType);
}

/***************************************************************************//**
 * @brief Enables/disables the activity detection.
 *
 * @param actOnOff - Enables/disables the activity detection.
 *                     Example: 0x0 - disables the activity detection.
 *                              0x1 - enables the activity detection.
 * @param actAxes - Axes which participate in detecting activity.
 *                    Example: 0x0 - disables axes participation.
 *                             ADXL345_ACT_X_EN - enables x-axis participation.
 *                             ADXL345_ACT_Y_EN - enables y-axis participation.
 *                             ADXL345_ACT_Z_EN - enables z-axis participation.
 * @param actAcDc - Selects dc-coupled or ac-coupled operation.
 *                    Example: 0x0 - dc-coupled operation.
 *                             ADXL345_ACT_ACDC - ac-coupled operation.
 * @param actThresh - Threshold value for detecting activity.
 * @patam actInt - Interrupts pin.
 *                   Example: 0x0 - activity interrupts on INT1 pin.
 *                            ADXL345_ACTIVITY - activity interrupts on INT2 pin.
 *
 * @return None.
*******************************************************************************/
void ADXL345_SetActivityDetection(unsigned char actOnOff, unsigned char actAxes, unsigned char actAcDc,
                                  unsigned char actThresh,   unsigned char actInt)	{
    unsigned char newActInactCtl    = 0;
    unsigned char newIntMap         = 0;
    unsigned char newIntEnable      = 0;
    
    newActInactCtl = (readReg8(ADXL345_INT_ENABLE) & ~(ADXL345_ACT_ACDC | ADXL345_ACT_X_EN | ADXL345_ACT_Y_EN | ADXL345_ACT_Z_EN));
    writeReg8(ADXL345_ACT_INACT_CTL, newActInactCtl | (actAcDc | actAxes));
    
		writeReg8(ADXL345_THRESH_ACT, actThresh);
    
		newIntMap = (readReg8(ADXL345_INT_MAP) &(~ADXL345_ACTIVITY));
    writeReg8(ADXL345_INT_MAP, newIntMap | actInt);
    
	  newIntEnable = (readReg8(ADXL345_INT_ENABLE) & (~ADXL345_ACTIVITY));
    writeReg8(ADXL345_INT_ENABLE, newIntEnable | (ADXL345_ACTIVITY * actOnOff));
}

/***************************************************************************//**
 * @brief Enables/disables the inactivity detection.
 *
 * @param inactOnOff - Enables/disables the inactivity detection.
 *                       Example: 0x0 - disables the inactivity detection.
 *                                0x1 - enables the inactivity detection.
 * @param inactAxes - Axes which participate in detecting inactivity.
 *                      Example: 0x0 - disables axes participation.
 *                               ADXL345_INACT_X_EN - enables x-axis.
 *                               ADXL345_INACT_Y_EN - enables y-axis.
 *                               ADXL345_INACT_Z_EN - enables z-axis.
 * @param inactAcDc - Selects dc-coupled or ac-coupled operation.
 *                      Example: 0x0 - dc-coupled operation.
 *                               ADXL345_INACT_ACDC - ac-coupled operation.
 * @param inactThresh - Threshold value for detecting inactivity.
 * @param inactTime - Inactivity time.
 * @patam inactInt - Interrupts pin.
 *                     Example: 0x0 - inactivity interrupts on INT1 pin.
 *                              ADXL345_INACTIVITY - inactivity interrupts on
 *                                                   INT2 pin.
 *
 * @return None.
*******************************************************************************/
void ADXL345_SetInactivityDetection(unsigned char inactOnOff, unsigned char inactAxes, unsigned char inactAcDc,
                                    unsigned char inactThresh, unsigned char inactTime, unsigned char inactInt)		{
    unsigned char newActInactCtl    = 0;
    unsigned char newIntMap         = 0;
    unsigned char newIntEnable      = 0;
    
    newActInactCtl = (readReg8(ADXL345_INT_ENABLE) & ~(ADXL345_INACT_ACDC | ADXL345_INACT_X_EN | ADXL345_INACT_Y_EN | ADXL345_INACT_Z_EN));
    writeReg8(ADXL345_ACT_INACT_CTL, (newActInactCtl | (inactAcDc | inactAxes)));
    
		writeReg8(ADXL345_THRESH_INACT, inactThresh);
    writeReg8(ADXL345_TIME_INACT, inactTime);
    
		newIntMap = readReg8(ADXL345_INT_MAP) & ~(ADXL345_INACTIVITY);
    writeReg8(ADXL345_INT_MAP, newIntMap | inactInt);
   
		newIntEnable = readReg8(ADXL345_INT_ENABLE) & ~(ADXL345_INACTIVITY);
    writeReg8(ADXL345_INT_ENABLE, newIntEnable | (ADXL345_INACTIVITY * inactOnOff));
}

/***************************************************************************//**
 * @brief Enables/disables the free-fall detection.
 *
 * @param ffOnOff - Enables/disables the free-fall detection.
 *                    Example: 0x0 - disables the free-fall detection.
 *                             0x1 - enables the free-fall detection.
 * @param ffThresh - Threshold value for free-fall detection.
 * @param ffTime - Time value for free-fall detection.
 * @param ffInt - Interrupts pin.
 *                  Example: 0x0 - free-fall interrupts on INT1 pin.
 *                           ADXL345_FREE_FALL - free-fall interrupts on INT2 pin.
 *
 * @return None.
*******************************************************************************/
void ADXL345_SetFreeFallDetection(unsigned char ffOnOff, unsigned char ffThresh,
                                  unsigned char ffTime,  unsigned char ffInt) {
    unsigned char oldIntMap     = 0;
    unsigned char newIntMap     = 0;
    unsigned char oldIntEnable  = 0;
    unsigned char newIntEnable  = 0;
    
    writeReg8(ADXL345_THRESH_FF, ffThresh);
    writeReg8(ADXL345_TIME_FF, ffTime);
	
    oldIntMap = readReg8(ADXL345_INT_MAP);
    newIntMap = oldIntMap & ~(ADXL345_FREE_FALL);
    newIntMap = newIntMap | ffInt;
    writeReg8(ADXL345_INT_MAP, newIntMap);
    oldIntEnable = readReg8(ADXL345_INT_ENABLE);
    newIntEnable = oldIntEnable & ~ADXL345_FREE_FALL;
    newIntEnable = newIntEnable | (ADXL345_FREE_FALL * ffOnOff);
    writeReg8(ADXL345_INT_ENABLE, newIntEnable);
}

/***************************************************************************//**
 * @brief Calibrates the accelerometer.
 *
 * @param xOffset - X-axis's offset.
 * @param yOffset - Y-axis's offset.
 * @param zOffset - Z-axis's offset.
 *
 * @return None.
*******************************************************************************/
void ADXL345_SetOffset(unsigned char xOffset,
                       unsigned char yOffset,
                       unsigned char zOffset)	{
    writeReg8(ADXL345_OFSX, xOffset);
    writeReg8(ADXL345_OFSY, yOffset);
    writeReg8(ADXL345_OFSZ, yOffset);
}


void ADXL345_SetDataFormat(unsigned char format)	{
    writeReg8(ADXL345_DATA_FORMAT, format);
}
