/* --COPYRIGHT--,BSD
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//*****************************************************************************
//
//usci_b_i2c.c - Driver for the I2C Module.
//
//*****************************************************************************
#include "inc/hw_regaccess.h"
#include "assert.h"
#include "usci_b_i2c.h"
#ifdef  __IAR_SYSTEMS_ICC__
#include "deprecated/IAR/msp430xgeneric.h"
#else
#include "deprecated/CCS/msp430xgeneric.h"
#endif

//*****************************************************************************
//
//! Initializes the I2C Master block.
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param selectClockSource is the clocksource.
//!         Valid values are
//!         \b USCI_B_I2C_CLOCKSOURCE_ACLK
//!         \b USCI_B_I2C_CLOCKSOURCE_SMCLK
//! \param i2cClk is the rate of the clock supplied to the I2C module.
//! \param dataRate set up for selecting data transfer rate.
//!         Valid values are
//!         \b USCI_B_I2C_SET_DATA_RATE_400KBPS
//!         \b USCI_B_I2C_SET_DATA_RATE_100KBPS
//!
//! This function initializes operation of the I2C Master block.  Upon
//! successful initialization of the I2C block, this function will have set the
//! bus speed for the master; however I2C module is still disabled till
//! USCI_B_I2C_enable is invoked
//!
//! If the parameter \e dataRate is USCI_B_I2C_SET_DATA_RATE_400KBPS, then the master
//! block will be set up to transfer data at 400 kbps; otherwise, it will be
//! set up to transfer data at 100 kbps.
//!
//! Modified bits are \b UCMST,UCMODE_3,\b UCSYNC of \b UCBxCTL0 register
//!                   \b UCSSELx, \b UCSWRST, of \b UCBxCTL1 register
//!                   \b UCBxBR0 and \b UCBxBR1 regsiters
//! \return None.
//
//*****************************************************************************
void USCI_B_I2C_masterInit (uint32_t baseAddress,
    uint8_t selectClockSource,
    uint32_t i2cClk,
    uint32_t dataRate)
{
    uint16_t preScalarValue;

    assert((USCI_B_I2C_CLOCKSOURCE_ACLK == selectClockSource) ||
        (USCI_B_I2C_CLOCKSOURCE_SMCLK == selectClockSource)
        );

    assert((USCI_B_I2C_SET_DATA_RATE_400KBPS == dataRate) ||
        (USCI_B_I2C_SET_DATA_RATE_100KBPS == dataRate)
        );

    //Disable the USCI module and clears the other bits of control register
    HWREG8(baseAddress + OFS_UCBxCTL1) = UCSWRST;

    /*
     * Configure as I2C master mode.
     * UCMST = Master mode
     * UCMODE_3 = I2C mode
     * UCSYNC = Synchronous mode
     */
    HWREG8(baseAddress + OFS_UCBxCTL0) = UCMST + UCMODE_3 + UCSYNC;

    //Configure I2C clock source
    HWREG8(baseAddress + OFS_UCBxCTL1) = (selectClockSource + UCSWRST );

    /*
     * Compute the clock divider that achieves the fastest speed less than or
     * equal to the desired speed.  The numerator is biased to favor a larger
     * clock divider so that the resulting clock is always less than or equal
     * to the desired clock, never greater.
     */
    preScalarValue = (unsigned short)(i2cClk / dataRate);
    HWREG16(baseAddress + OFS_UCBxBRW) = preScalarValue;
}

//*****************************************************************************
//
//! Initializes the I2C Slave block.
//!
//! \param baseAddress is the base address of the I2C Slave module.
//! \param slaveAddress 7-bit slave address
//!
//! This function initializes operation of the I2C as a Slave mode.  Upon
//! successful initialization of the I2C blocks, this function will have set
//! the slave address but the I2C module is still disabled till USCI_B_I2C_enable
//! is invoked.
//!
//! The parameter slaveAddress is the value that will be compared against the
//! slave address sent by an I2C master.
//! Modified buts are \b UCMODE_3, \b UCSYNC of \b UCBxCTL0 register
//!                   \b UCSWRST of \b UCBxCTL1 register
//!                   \b UCBxI2COA register
//!
//! \return None.
//
//*****************************************************************************
void USCI_B_I2C_slaveInit (uint32_t baseAddress,
    uint8_t slaveAddress
    )
{
    //Disable the USCI module
    HWREG8(baseAddress + OFS_UCBxCTL1) |= UCSWRST;

    //Confiugre I2C as Slave and Synchronous mode
    HWREG8(baseAddress + OFS_UCBxCTL0) = UCMODE_3 + UCSYNC;

    //Set up the slave address.
    HWREG16(baseAddress + OFS_UCBxI2COA) = slaveAddress;
}

//*****************************************************************************
//
//! Enables the I2C block.
//!
//! \param baseAddress is the base address of the USCI I2C module.
//!
//! This will enable operation of the I2C block.
//! Modified bits are \b UCSWRST of \b UCBxCTL1 register.
//!
//! \return None.
//
//*****************************************************************************
void USCI_B_I2C_enable (uint32_t baseAddress)
{
    //Reset the UCSWRST bit to enable the USCI Module
    HWREG8(baseAddress + OFS_UCBxCTL1) &= ~(UCSWRST);
}

//*****************************************************************************
//
//! Disables the I2C block.
//!
//! \param baseAddress is the base address of the USCI I2C module.
//!
//! This will disable operation of the I2C block.
//! Modified bits are \b UCSWRST of \b UCBxCTL1 register.
//!
//! \return None.
//
//*****************************************************************************
void USCI_B_I2C_disable (uint32_t baseAddress)
{
    //Set the UCSWRST bit to disable the USCI Module
    HWREG8(baseAddress + OFS_UCBxCTL1) |= UCSWRST;
}

//*****************************************************************************
//
//! Sets the address that the I2C Master will place on the bus.
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param slaveAddress 7-bit slave address
//!
//! This function will set the address that the I2C Master will place on the
//! bus when initiating a transaction.
//! Modified bits are \b UCSWRST of \b UCBxCTL1 register
//!                   \b UCBxI2CSA register
//!
//! \return None.
//
//*****************************************************************************
void USCI_B_I2C_setSlaveAddress (uint32_t baseAddress,
    uint8_t slaveAddress
    )
{
    //Set the address of the slave with which the master will communicate.
    HWREG16(baseAddress + OFS_UCBxI2CSA) = (slaveAddress);
}

//*****************************************************************************
//
//! Sets the mode of the I2C device
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param receive indicates whether module is in transmit/receive mode
//!
//! When the receive parameter is set to USCI_B_I2C_TRANSMIT_MODE, the address will
//! indicate that the I2C module is in receive mode; otherwise, the I2C module
//! is in send mode. Valid values are
//!     \b USCI_B_I2C_TRANSMIT_MODE
//!     \b USCI_B_I2C_RECEIVE_MODE [Default value]
//! Modified bits are \b UCTR of \b UCBxCTL1 register
//!
//! \return None.
//
//*****************************************************************************
void USCI_B_I2C_setMode (uint32_t baseAddress,
    uint8_t mode
    )
{
    assert((USCI_B_I2C_TRANSMIT_MODE == mode) ||
        (USCI_B_I2C_RECEIVE_MODE == mode)
        );

    HWREG8(baseAddress + OFS_UCBxCTL1) &= ~USCI_B_I2C_TRANSMIT_MODE;
    HWREG8(baseAddress + OFS_UCBxCTL1) |= mode;
}

//*****************************************************************************
//
//! Transmits a byte from the I2C Module.
//!
//! \param baseAddress is the base address of the I2C module.
//! \param transmitData data to be transmitted from the I2C module
//!
//! This function will place the supplied data into I2C trasmit data register
//! to start transmission
//! Modified bit is \b UCBxTXBUF register
//!
//! \return None.
//
//*****************************************************************************
void USCI_B_I2C_slaveDataPut (uint32_t baseAddress,
    uint8_t transmitData
    )
{
    //Send single byte data.
    HWREG8(baseAddress + OFS_UCBxTXBUF) = transmitData;
}

//*****************************************************************************
//
//! Receives a byte that has been sent to the I2C Module.
//!
//! \param baseAddress is the base address of the I2C module.
//!
//! This function reads a byte of data from the I2C receive data Register.
//!
//! \return Returns the byte received from by the I2C module, cast as an
//! uint8_t.
//! Modified bit is \b UCBxRXBUF register
//
//*****************************************************************************
uint8_t USCI_B_I2C_slaveDataGet (uint32_t baseAddress)
{
    //Read a byte.
    return (HWREG8(baseAddress + OFS_UCBxRXBUF));
}

//*****************************************************************************
//
//! Indicates whether or not the I2C bus is busy.
//!
//! \param baseAddress is the base address of the I2C module.
//!
//! This function returns an indication of whether or not the I2C bus is
//! busy.This function checks the status of the bus via UCBBUSY bit in
//! UCBxSTAT register.
//!
//! \return Returns USCI_B_I2C_BUS_BUSY if the I2C Master is busy; otherwise, returns
//! USCI_B_I2C_BUS_NOT_BUSY.
//
//*****************************************************************************
uint8_t USCI_B_I2C_isBusBusy (uint32_t baseAddress)
{
    //Return the bus busy status.
    return (HWREG8(baseAddress + OFS_UCBxSTAT) & UCBBUSY);
}

//*****************************************************************************
//
//! DEPRECATED - Function may be removed in future release. 
//! Indicates whether or not the I2C module is busy.
//!
//! \param baseAddress is the base address of the I2C module.
//!
//! This function returns an indication of whether or not the I2C module is
//! busy transmitting or receiving data. This function checks if the Transmit
//! or receive flag is set.
//!
//! \return Returns USCI_B_I2C_BUS_BUSY if the I2C module is busy; otherwise, returns
//! USCI_B_I2C_BUS_NOT_BUSY.
//
//*****************************************************************************
uint8_t USCI_B_I2C_isBusy (uint32_t baseAddress)
{
    //Return the busy status.
    if ((HWREG8(baseAddress + OFS_UCBxIFG) & (UCTXIFG + UCRXIFG))){
        return (USCI_B_I2C_BUS_BUSY);
    } else   {
        return (USCI_B_I2C_BUS_NOT_BUSY);
    }
}

//*****************************************************************************
//
//! Indicates whether STOP got sent.
//!
//! \param baseAddress is the base address of the I2C module.
//!
//! This function returns an indication of whether or not STOP got sent
//! This function checks the status of the bus via UCTXSTP bit in
//! UCBxCTL1 register.
//!
//! \return Returns USCI_B_I2C_STOP_SEND_COMPLETE if the I2C Master
//!			finished sending STOP; 
//!			otherwise, returns  USCI_B_I2C_SENDING_STOP.
//
//*****************************************************************************
uint8_t USCI_B_I2C_masterIsStopSent (uint32_t baseAddress)
{
    //Return the bus busy status.
    return (HWREG8(baseAddress + OFS_UCBxCTL1) & UCTXSTP);
}
//*****************************************************************************
//
//! Indicates whether START got sent.
//!
//! \param baseAddress is the base address of the I2C module.
//!
//! This function returns an indication of whether or not START got sent
//! This function checks the status of the bus via UCTXSTT bit in
//! UCBxCTL1 register.
//!
//! \return Returns USCI_B_I2C_START_SEND_COMPLETE if the I2C Master
//!			finished sending START; otherwise, returns 
//!			USCI_B_I2C_SENDING_START.
//
//*****************************************************************************
uint8_t USCI_B_I2C_masterIsStartSent (uint32_t baseAddress)
{
    //Return if master has sent start
    return (HWREG8(baseAddress + OFS_UCBxCTL1) & UCTXSTT);
}
//*****************************************************************************
//
//! This function is used by the Master module to initiate START
//!
//! \param baseAddress is the base address of the I2C Master module.
//!
//! This function is used by the Master module to initiate STOP
//!
//! Modified bits are UCTXSTT bit of UCBxCTL1.
//!
//! \return None.
//
//*****************************************************************************
void USCI_B_I2C_masterSendStart (uint32_t baseAddress)
{
    HWREG8(baseAddress + OFS_UCBxCTL1) |= UCTXSTT;
}
//*****************************************************************************
//
//! Enables individual I2C interrupt sources.
//!
//! \param baseAddress is the base address of the I2C module.
//! \param interruptFlags is the bit mask of the interrupt sources to be enabled.
//!
//! Enables the indicated I2C interrupt sources.  Only the sources that
//! are enabled can be reflected to the processor interrupt; disabled sources
//! have no effect on the processor. <b>Does not clear interrupt flags.</b>
//!
//! The mask parameter is the logical OR of any of the following:
//!
//! - \b USCI_B_I2C_STOP_INTERRUPT - STOP condition interrupt
//! - \b USCI_B_I2C_START_INTERRUPT - START condition interrupt
//! - \b USCI_B_I2C_RECEIVE_INTERRUPT -Receive interrupt
//! - \b USCI_B_I2C_TRANSMIT_INTERRUPT - Transmit interrupt
//! - \b USCI_B_I2C_NAK_INTERRUPT - Not-acknowledge interrupt
//! - \b USCI_B_I2C_ARBITRATIONLOST_INTERRUPT - Arbitration lost interrupt
//!
//! Modified registers are UCBxIFG and OFS_UCBxIE.
//!
//! \return None.
//
//*****************************************************************************
void USCI_B_I2C_enableInterrupt (uint32_t baseAddress,
    uint8_t mask
    )
{
    assert( 0x00 == ( mask & ~(USCI_B_I2C_STOP_INTERRUPT +
                               USCI_B_I2C_START_INTERRUPT +
                               USCI_B_I2C_RECEIVE_INTERRUPT +
                               USCI_B_I2C_TRANSMIT_INTERRUPT +
                               USCI_B_I2C_NAK_INTERRUPT +
                               USCI_B_I2C_ARBITRATIONLOST_INTERRUPT))
        );

    //Enable the interrupt masked bit
    HWREG16(baseAddress + OFS_UCBxIE) |= mask;
}

//*****************************************************************************
//
//! Disables individual I2C interrupt sources.
//!
//! \param baseAddress is the base address of the I2C module.
//! \param mask is the bit mask of the interrupt sources to be
//! disabled.
//!
//! Disables the indicated I2C interrupt sources.  Only the sources that
//! are enabled can be reflected to the processor interrupt; disabled sources
//! have no effect on the processor.
//!
//! The mask parameter is the logical OR of any of the following:
//!
//! - \b USCI_B_I2C_STOP_INTERRUPT - STOP condition interrupt
//! - \b USCI_B_I2C_START_INTERRUPT - START condition interrupt
//! - \b USCI_B_I2C_RECEIVE_INTERRUPT -Receive interrupt
//! - \b USCI_B_I2C_TRANSMIT_INTERRUPT - Transmit interrupt
//! - \b USCI_B_I2C_NAK_INTERRUPT - Not-acknowledge interrupt
//! - \b USCI_B_I2C_ARBITRATIONLOST_INTERRUPT - Arbitration lost interrupt
//!
//! Modified register is \b UCBxIE.
//!
//! \return None.
//
//*****************************************************************************
void USCI_B_I2C_disableInterrupt (uint32_t baseAddress,
    uint8_t mask
    )
{
    assert( 0x00 == ( mask & ~(USCI_B_I2C_STOP_INTERRUPT +
                               USCI_B_I2C_START_INTERRUPT +
                               USCI_B_I2C_RECEIVE_INTERRUPT +
                               USCI_B_I2C_TRANSMIT_INTERRUPT +
                               USCI_B_I2C_NAK_INTERRUPT +
                               USCI_B_I2C_ARBITRATIONLOST_INTERRUPT))
        );

    //Disable the interrupt masked bit
    HWREG8(baseAddress + OFS_UCBxIE) &= ~(mask);
}

//*****************************************************************************
//
//! Clears I2C interrupt sources.
//!
//! \param baseAddress is the base address of the I2C Slave module.
//! \param mask is a bit mask of the interrupt sources to be cleared.
//!
//! The I2C interrupt source is cleared, so that it no longer asserts.
//! The highest interrupt flag is automatically cleared when an interrupt vector
//! generator is used.
//!
//! The mask parameter has the same definition as the mask
//! parameter to USCI_B_I2C_enableInterrupt().
//!
//! Modified register is \b UCBxIFG.
//!
//! \return None.
//
//*****************************************************************************
void USCI_B_I2C_clearInterruptFlag (uint32_t baseAddress,
    uint8_t mask
    )
{
    assert( 0x00 == ( mask & ~(USCI_B_I2C_STOP_INTERRUPT +
                               USCI_B_I2C_START_INTERRUPT +
                               USCI_B_I2C_RECEIVE_INTERRUPT +
                               USCI_B_I2C_TRANSMIT_INTERRUPT +
                               USCI_B_I2C_NAK_INTERRUPT +
                               USCI_B_I2C_ARBITRATIONLOST_INTERRUPT))
        );
    //Clear the I2C interrupt source.
    HWREG8(baseAddress + OFS_UCBxIFG) &= ~(mask);
}

//*****************************************************************************
//
//! Gets the current I2C interrupt status.
//!
//! \param baseAddress is the base address of the I2C module.
//! \param mask is the masked interrupt flag status to be returned.
//!
//! This returns the interrupt status for the I2C  module based on which
//! flag is passed. mask parameter can be either any of the following
//! selection.
//!
//! - \b USCI_B_I2C_STOP_INTERRUPT - STOP condition interrupt
//! - \b USCI_B_I2C_START_INTERRUPT - START condition interrupt
//! - \b USCI_B_I2C_RECEIVE_INTERRUPT -Receive interrupt
//! - \b USCI_B_I2C_TRANSMIT_INTERRUPT - Transmit interrupt
//! - \b USCI_B_I2C_NAK_INTERRUPT - Not-acknowledge interrupt
//! - \b USCI_B_I2C_ARBITRATIONLOST_INTERRUPT - Arbitration lost interrupt
//!
//! Modified register is \b UCBxIFG.
//!
//! \returns the masked status of the interrupt flag
//
//*****************************************************************************
uint8_t USCI_B_I2C_getInterruptStatus (uint32_t baseAddress,
    uint8_t mask
    )
{
    assert( 0x00 == ( mask & ~(USCI_B_I2C_STOP_INTERRUPT +
                               USCI_B_I2C_START_INTERRUPT +
                               USCI_B_I2C_RECEIVE_INTERRUPT +
                               USCI_B_I2C_TRANSMIT_INTERRUPT +
                               USCI_B_I2C_NAK_INTERRUPT +
                               USCI_B_I2C_ARBITRATIONLOST_INTERRUPT))
        );
    //Return the interrupt status of the request masked bit.
    return (HWREG8(baseAddress + OFS_UCBxIFG) & mask);
}

//*****************************************************************************
//
//! Does single byte transmission from Master to Slave
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param txData is the data byte to be transmitted
//!
//! This function is used by the Master module to send a single byte.
//! This function
//! - Sends START
//! - Transmits the byte to the Slave
//! - Sends STOP
//!
//! Modified registers are \b UCBxIE, \b UCBxCTL1, \b UCBxIFG, \b UCBxTXBUF,
//! \b UCBxIE
//!
//! \return None.
//
//*****************************************************************************
void USCI_B_I2C_masterSendSingleByte (uint32_t baseAddress,
    uint8_t txData
    )
{
    //Store current TXIE status
    uint8_t txieStatus = HWREG8(baseAddress + OFS_UCBxIE) & UCTXIE;

    //Disable transmit interrupt enable
    HWREG8(baseAddress + OFS_UCBxIE) &= ~(UCTXIE);

    //Send start condition.
    HWREG8(baseAddress + OFS_UCBxCTL1) |= UCTR + UCTXSTT;

    //Poll for transmit interrupt flag.
    while (!(HWREG8(baseAddress + OFS_UCBxIFG) & UCTXIFG)) ;

    //Send single byte data.
    HWREG8(baseAddress + OFS_UCBxTXBUF) = txData;

    //Poll for transmit interrupt flag.
    while (!(HWREG8(baseAddress + OFS_UCBxIFG) & UCTXIFG)) ;

    //Send stop condition.
    HWREG8(baseAddress + OFS_UCBxCTL1) |= UCTXSTP;

    //Clear transmit interrupt flag before enabling interrupt again
    HWREG8(baseAddress + OFS_UCBxIFG) &= ~(UCTXIFG);

    //Reinstate transmit interrupt enable
    HWREG8(baseAddress + OFS_UCBxIE) |= txieStatus;
}

//*****************************************************************************
//
//! Does single byte transmission from Master to Slave with timeout
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param txData is the data byte to be transmitted
//! \param timeout is the amount of time to wait until giving up
//!
//! This function is used by the Master module to send a single byte.
//! This function
//! - Sends START
//! - Transmits the byte to the Slave
//! - Sends STOP
//!
//! Modified registers are \b UCBxIE, \b UCBxCTL1, \b UCBxIFG, \b UCBxTXBUF,
//! \b UCBxIE
//!
//! \return STATUS_SUCCESS or STATUS_FAILURE of the transmission process.
//
//*****************************************************************************
unsigned short USCI_B_I2C_masterSendSingleByteWithTimeout (uint32_t baseAddress,
    uint8_t txData,
    uint32_t timeout
    )
{
	// Creating variable for second timeout scenario
	uint32_t timeout2 = timeout;
	
	assert(timeout > 0);
		
    //Store current TXIE status
    uint8_t txieStatus = HWREG8(baseAddress + OFS_UCBxIE) & UCTXIE;

    //Disable transmit interrupt enable
    HWREG8(baseAddress + OFS_UCBxIE) &= ~(UCTXIE);

    //Send start condition.
    HWREG8(baseAddress + OFS_UCBxCTL1) |= UCTR + UCTXSTT;

    //Poll for transmit interrupt flag.
    while ((!(HWREG8(baseAddress + OFS_UCBxIFG) & UCTXIFG)) && timeout--) ;
    
    //Check if transfer timed out
    if (timeout == 0){
    	return (STATUS_FAIL);
    }

    //Send single byte data.
    HWREG8(baseAddress + OFS_UCBxTXBUF) = txData;

   	//Poll for transmit interrupt flag.
    while ((!(HWREG8(baseAddress + OFS_UCBxIFG) & UCTXIFG)) && timeout2--) ;
    
    //Check if transfer timed out
    if (timeout2 == 0){
    	return (STATUS_FAIL);
    }
    
    //Send stop condition.
    HWREG8(baseAddress + OFS_UCBxCTL1) |= UCTXSTP;

    //Clear transmit interrupt flag before enabling interrupt again
    HWREG8(baseAddress + OFS_UCBxIFG) &= ~(UCTXIFG);

    //Reinstate transmit interrupt enable
    HWREG8(baseAddress + OFS_UCBxIE) |= txieStatus;
    
    return (STATUS_SUCCESS);
}

//*****************************************************************************
//
//! Starts multi-byte transmission from Master to Slave
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param txData is the first data byte to be transmitted
//!
//! This function is used by the Master module to send a single byte.
//! This function
//! - Sends START
//! - Transmits the first data byte of a multi-byte transmission to the Slave
//!
//! Modified registers are \b UCBxIE, \b UCBxCTL1, \b UCBxIFG, \b UCBxTXBUF,
//! \b UCBxIE
//!
//! \return None.
//
//*****************************************************************************
void USCI_B_I2C_masterMultiByteSendStart (uint32_t baseAddress,
    uint8_t txData
    )
{
    //Store current transmit interrupt enable
    uint8_t txieStatus = HWREG8(baseAddress + OFS_UCBxIE) & UCTXIE;


    //Disable transmit interrupt enable
    HWREG8(baseAddress + OFS_UCBxIE) &= ~(UCTXIE);

    //Send start condition.
    HWREG8(baseAddress + OFS_UCBxCTL1) |= UCTR +  UCTXSTT;

    //Poll for transmit interrupt flag.
    while (!(HWREG8(baseAddress + OFS_UCBxIFG) & UCTXIFG)) ;

    //Send single byte data.
    HWREG8(baseAddress + OFS_UCBxTXBUF) = txData;

    //Reinstate transmit interrupt enable
    HWREG8(baseAddress + OFS_UCBxIE) |= txieStatus;
}

//*****************************************************************************
//
//! Starts multi-byte transmission from Master to Slave with timeout
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param txData is the first data byte to be transmitted
//! \param timeout is the amount of time to wait until giving up
//!
//! This function is used by the Master module to send a single byte.
//! This function
//! - Sends START
//! - Transmits the first data byte of a multi-byte transmission to the Slave
//!
//! Modified registers are \b UCBxIE, \b UCBxCTL1, \b UCBxIFG, \b UCBxTXBUF,
//! \b UCBxIE
//!
//! \return STATUS_SUCCESS or STATUS_FAILURE of the transmission process.
//
//*****************************************************************************
unsigned short USCI_B_I2C_masterMultiByteSendStartWithTimeout (uint32_t baseAddress,
    uint8_t txData, 
    uint32_t timeout
    )
{
	assert(timeout == 0);
	
    //Store current transmit interrupt enable
    uint8_t txieStatus = HWREG8(baseAddress + OFS_UCBxIE) & UCTXIE;

    //Disable transmit interrupt enable
    HWREG8(baseAddress + OFS_UCBxIE) &= ~(UCTXIE);

    //Send start condition.
    HWREG8(baseAddress + OFS_UCBxCTL1) |= UCTR +  UCTXSTT;

    //Poll for transmit interrupt flag.
    while ((!(HWREG8(baseAddress + OFS_UCBxIFG) & UCTXIFG)) && timeout--) ;

	//Check if transfer timed out
    if (timeout == 0){
    	return (STATUS_FAIL);
    }

    //Send single byte data.
    HWREG8(baseAddress + OFS_UCBxTXBUF) = txData;

    //Reinstate transmit interrupt enable
    HWREG8(baseAddress + OFS_UCBxIE) |= txieStatus;

    return(STATUS_SUCCESS);
}

//*****************************************************************************
//
//! Continues multi-byte transmission from Master to Slave
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param txData is the next data byte to be transmitted
//! \param timeout is the amount of time to wait until giving up
//!
//! This function is used by the Master module continue each byte of a
//! multi-byte trasmission. This function
//! - Transmits each data byte of a multi-byte transmission to the Slave
//!
//! Modified registers are \b UCBxTXBUF
//!
//! \return None.
//
//*****************************************************************************
void USCI_B_I2C_masterMultiByteSendNext (uint32_t baseAddress,
    uint8_t txData
    )
{
    //If interrupts are not used, poll for flags
    if (!(HWREG8(baseAddress + OFS_UCBxIE) & UCTXIE)){
        //Poll for transmit interrupt flag.
        while (!(HWREG8(baseAddress + OFS_UCBxIFG) & UCTXIFG)) ;
    }

    //Send single byte data.
    HWREG8(baseAddress + OFS_UCBxTXBUF) = txData;
}

//*****************************************************************************
//
//! Continues multi-byte transmission from Master to Slave with timeout
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param txData is the next data byte to be transmitted
//! \param timeout is the amount of time to wait until giving up
//!
//! This function is used by the Master module continue each byte of a
//! multi-byte trasmission. This function
//! - Transmits each data byte of a multi-byte transmission to the Slave
//!
//! Modified registers are \b UCBxTXBUF
//!
//! \return STATUS_SUCCESS or STATUS_FAILURE of the transmission process.
//
//*****************************************************************************
unsigned short USCI_B_I2C_masterMultiByteSendNextWithTimeout (uint32_t baseAddress,
    uint8_t txData, 
    uint32_t timeout
    )
{
	assert(timeout == 0);
		
    //If interrupts are not used, poll for flags
    if (!(HWREG8(baseAddress + OFS_UCBxIE) & UCTXIE)){
        //Poll for transmit interrupt flag.
    	while ((!(HWREG8(baseAddress + OFS_UCBxIFG) & UCTXIFG)) && timeout--);

		//Check if transfer timed out
        if (timeout == 0){
        	return (STATUS_FAIL);
        }
    }

    //Send single byte data.
    HWREG8(baseAddress + OFS_UCBxTXBUF) = txData;

    return(STATUS_SUCCESS);
}

//*****************************************************************************
//
//! Finishes multi-byte transmission from Master to Slave
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param txData is the last data byte to be transmitted in a multi-byte
//! tramsission
//!
//! This function is used by the Master module to send the last byte and STOP.
//! This function
//! - Transmits the last data byte of a multi-byte transmission to the Slave
//! - Sends STOP
//!
//! Modified registers are \b UCBxTXBUF and \b UCBxCTL1.
//!
//! \return None.
//
//*****************************************************************************
void USCI_B_I2C_masterMultiByteSendFinish (uint32_t baseAddress,
    uint8_t txData
    )
{
    //If interrupts are not used, poll for flags
    if (!(HWREG8(baseAddress + OFS_UCBxIE) & UCTXIE)){
        //Poll for transmit interrupt flag.
        while (!(HWREG8(baseAddress + OFS_UCBxIFG) & UCTXIFG)) ;
    }

    //Send single byte data.
    HWREG8(baseAddress + OFS_UCBxTXBUF) = txData;

    //Poll for transmit interrupt flag.
    while (!(HWREG8(baseAddress + OFS_UCBxIFG) & UCTXIFG)) ;

    //Send stop condition.
    HWREG8(baseAddress + OFS_UCBxCTL1) |= UCTXSTP;
}

//*****************************************************************************
//
//! Finishes multi-byte transmission from Master to Slave with timeout
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param txData is the last data byte to be transmitted in a multi-byte
//! tramsission
//! \param timeout is the amount of time to wait until giving up
//!
//! This function is used by the Master module to send the last byte and STOP.
//! This function
//! - Transmits the last data byte of a multi-byte transmission to the Slave
//! - Sends STOP
//!
//! Modified registers are \b UCBxTXBUF and \b UCBxCTL1.
//!
//! \return STATUS_SUCCESS or STATUS_FAILURE of the transmission process.
//
//*****************************************************************************
unsigned short USCI_B_I2C_masterMultiByteSendFinishWithTimeout (uint32_t baseAddress,
    uint8_t txData,
    uint32_t timeout
    )
{
	uint32_t timeout2 = timeout;
	
	assert(timeout == 0);
	
    //If interrupts are not used, poll for flags
    if (!(HWREG8(baseAddress + OFS_UCBxIE) & UCTXIE)){
        //Poll for transmit interrupt flag.
        while ((!(HWREG8(baseAddress + OFS_UCBxIFG) & UCTXIFG)) && timeout--) ;
        
        //Check if transfer timed out
        if (timeout == 0){
        	return (STATUS_FAIL);
        }
    }

    //Send single byte data.
    HWREG8(baseAddress + OFS_UCBxTXBUF) = txData;

    //Poll for transmit interrupt flag.
    while ((!(HWREG8(baseAddress + OFS_UCBxIFG) & UCTXIFG)) && timeout2--) ;

	//Check if transfer timed out
    if (timeout2 == 0){
    	return (STATUS_FAIL);
    }

    //Send stop condition.
    HWREG8(baseAddress + OFS_UCBxCTL1) |= UCTXSTP;
    
    return(STATUS_SUCCESS);
}

//*****************************************************************************
//
//! Send STOP byte at the end of a multi-byte transmission from Master to Slave
//!
//! \param baseAddress is the base address of the I2C Master module.
//!
//! This function is used by the Master module send STOP at the end of a
//! multi-byte trasmission
//!
//! This function
//! - Send a STOP after current transmission is complete
//!
//! Modified bits are \b UCTXSTP bit of \b UCBxCTL1.
//! \return None.
//
//*****************************************************************************
void USCI_B_I2C_masterMultiByteSendStop (uint32_t baseAddress)
{
    //If interrupts are not used, poll for flags
    if (!(HWREG8(baseAddress + OFS_UCBxIE) & UCTXIE)){
        //Poll for transmit interrupt flag.
        while (!(HWREG8(baseAddress + OFS_UCBxIFG) & UCTXIFG)) ;
    }

    //Send stop condition.
    HWREG8(baseAddress + OFS_UCBxCTL1) |= UCTXSTP;
}

//*****************************************************************************
//
//! Send STOP byte at the end of a multi-byte transmission from Master to Slave 
//! with timeout
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param timeout is the amount of time to wait until giving up
//!
//! This function is used by the Master module send STOP at the end of a
//! multi-byte trasmission
//!
//! This function
//! - Send a STOP after current transmission is complete
//!
//! Modified bits are \b UCTXSTP bit of \b UCBxCTL1.
//! \return STATUS_SUCCESS or STATUS_FAILURE of the transmission process.
//
//*****************************************************************************
unsigned short USCI_B_I2C_masterMultiByteSendStopWithTimeout (uint32_t baseAddress, 
	uint32_t timeout)
{
	
	assert(timeout == 0);
	
    //If interrupts are not used, poll for flags
    if (!(HWREG8(baseAddress + OFS_UCBxIE) & UCTXIE)){
        //Poll for transmit interrupt flag.
    	while ((!(HWREG8(baseAddress + OFS_UCBxIFG) & UCTXIFG)) && timeout--) ;

		//Check if transfer timed out
        if (timeout == 0){
        	return (STATUS_FAIL);
        }
    }

    //Send stop condition.
    HWREG8(baseAddress + OFS_UCBxCTL1) |= UCTXSTP;

    return (STATUS_SUCCESS);
}

//*****************************************************************************
//
//! Starts multi-byte reception at the Master end
//!
//! \param baseAddress is the base address of the I2C Master module.
//!
//! This function is used by the Master module initiate reception of a single
//! byte. This function
//! - Sends START
//!
//! Modified bits are \b UCTXSTT bit of \b UCBxCTL1.
//! \return None.
//
//*****************************************************************************
void USCI_B_I2C_masterMultiByteReceiveStart (uint32_t baseAddress)
{
    //Set USCI in Receive mode
    HWREG8(baseAddress + OFS_UCBxCTL1) &= ~UCTR;
    //Send start
    HWREG8(baseAddress + OFS_UCBxCTL1) |= UCTXSTT;
}

//*****************************************************************************
//
//! Starts multi-byte reception at the Master end one byte at a time
//!
//! \param baseAddress is the base address of the I2C Master module.
//!
//! This function is used by the Master module to receive each byte of a
//! multi-byte reception
//! This function reads currently received byte
//!
//! Modified register is \b UCBxRXBUF.
//! \return Received byte at Master end.
//
//*****************************************************************************
uint8_t USCI_B_I2C_masterMultiByteReceiveNext (uint32_t baseAddress)
{
    return (HWREG8(baseAddress + OFS_UCBxRXBUF));
}

//*****************************************************************************
//
//! Finishes multi-byte reception at the Master end
//!
//! \param baseAddress is the base address of the I2C Master module.
//!
//! This function is used by the Master module to initiate completion of a
//! multi-byte reception
//! This function
//! - Receives the current byte and initiates the STOP from Master to Slave
//!
//! Modified bits are \b UCTXSTP bit of \b UCBxCTL1.
//!
//! \return Received byte at Master end.
//
//*****************************************************************************
uint8_t USCI_B_I2C_masterMultiByteReceiveFinish (uint32_t baseAddress)
{
    //Send stop condition.
    HWREG8(baseAddress + OFS_UCBxCTL1) |= UCTXSTP;

    //Wait for Stop to finish
    while (HWREG8(baseAddress + OFS_UCBxCTL1) & UCTXSTP)

    // Wait for RX buffer
    while (!(HWREG8(baseAddress + OFS_UCBxIFG) & UCRXIFG)) ;

    //Capture data from receive buffer after setting stop bit due to
    //MSP430 I2C critical timing.
    return (HWREG8(baseAddress + OFS_UCBxRXBUF));
}

//*****************************************************************************
//
//! Finishes multi-byte reception at the Master end with timeout
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param txData is a pointer to the location to store the recieved byte at 
//!      master end
//! \param timeout is the amount of time to wait until giving up
//!
//! This function is used by the Master module to initiate completion of a
//! multi-byte reception
//! This function
//! - Receives the current byte and initiates the STOP from Master to Slave
//!
//! Modified bits are \b UCTXSTP bit of \b UCBxCTL1.
//!
//! \return STATUS_SUCCESS or STATUS_FAILURE of the transmission process.
//
//*****************************************************************************
unsigned short USCI_B_I2C_masterMultiByteReceiveFinishWithTimeout (uint32_t baseAddress,
	uint8_t *txData,
    uint32_t timeout
    )
{
	uint32_t timeout2 = timeout;
	
    //Send stop condition.
    HWREG8(baseAddress + OFS_UCBxCTL1) |= UCTXSTP;

    //Wait for Stop to finish
    while ((HWREG8(baseAddress + OFS_UCBxCTL1) & UCTXSTP) && timeout--);
    
    //Check if transfer timed out
    if (timeout == 0){
    	return (STATUS_FAIL);
    }
    
    // Wait for RX buffer
    while ((!(HWREG8(baseAddress + OFS_UCBxIFG) & UCRXIFG)) && timeout2--) ;

    //Check if transfer timed out
    if (timeout2 == 0){
    	return (STATUS_FAIL);
    }
    
    //Capture data from receive buffer after setting stop bit due to
    //MSP430 I2C critical timing.
    *txData = (HWREG8(baseAddress + OFS_UCBxRXBUF));
    
    return (STATUS_SUCCESS);
}

//*****************************************************************************
//
//! Sends the STOP at the end of a multi-byte reception at the Master end
//!
//! \param baseAddress is the base address of the I2C Master module.
//!
//! This function is used by the Master module to initiate STOP
//!
//! Modified bits are UCTXSTP bit of UCBxCTL1.
//!
//! \return None.
//
//*****************************************************************************
void USCI_B_I2C_masterMultiByteReceiveStop (uint32_t baseAddress)
{
    //Send stop condition.
    HWREG8(baseAddress + OFS_UCBxCTL1) |= UCTXSTP;
}

//*****************************************************************************
//
//! Initiates a single byte Reception at the Master End
//!
//! \param baseAddress is the base address of the I2C Master module.
//!
//! This function sends a START and STOP immediately to indicate Single byte
//! reception
//!
//! Modified bits are \b UCTXSTT and \b UCTXSTP of \b UCBxCTL1, GIE
//!
//! \return None.
//
//*****************************************************************************
void USCI_B_I2C_masterSingleReceiveStart (uint32_t baseAddress)
{
    //local variable to store GIE status
    uint16_t gieStatus;

    //Store current SR register
    gieStatus = __get_SR_register() & GIE;

    //Disable global interrupt
    __disable_interrupt();

	//Set USCI in Receive mode
    HWREG8(baseAddress + OFS_UCBxCTL1) &= ~UCTR;
 
    //Send start condition.
    HWREG8(baseAddress + OFS_UCBxCTL1) |= UCTXSTT;

    //Poll for Start bit to complete
    while (HWREG8(baseAddress + OFS_UCBxCTL1) & UCTXSTT) ;

    //Send stop condition.
    HWREG8(baseAddress + OFS_UCBxCTL1) |= UCTXSTP;

    //Reinstate SR register
    __bis_SR_register(gieStatus);
}

//*****************************************************************************
//
//! Initiates a single byte Reception at the Master End with timeout
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param timeout is the amount of time to wait until giving up
//!
//! This function sends a START and STOP immediately to indicate Single byte
//! reception
//!
//! Modified bits are \b UCTXSTT and \b UCTXSTP of \b UCBxCTL1, GIE
//!
//! \return STATUS_SUCCESS or STATUS_FAILURE of the transmission process.
//
//*****************************************************************************
unsigned short USCI_B_I2C_masterSingleReceiveStartWithTimeout (uint32_t baseAddress,
	uint32_t timeout
	)
{
    //local variable to store GIE status
    uint16_t gieStatus;
    
    assert(timeout == 0);

    //Store current SR register
    gieStatus = __get_SR_register() & GIE;

    //Disable global interrupt
    __disable_interrupt();

	//Set USCI in Receive mode
    HWREG8(baseAddress + OFS_UCBxCTL1) &= ~UCTR;
 
    //Send start condition.
    HWREG8(baseAddress + OFS_UCBxCTL1) |= UCTXSTT;

    //Poll for Start bit to complete
    while ((!(HWREG8(baseAddress + OFS_UCBxIFG) & UCTXSTT)) && timeout--) ;
    
    //Check if transfer timed out
    if (timeout == 0){
    	return (STATUS_FAIL);
    }

    //Send stop condition.
    HWREG8(baseAddress + OFS_UCBxCTL1) |= UCTXSTP;

    //Reinstate SR register
    __bis_SR_register(gieStatus);
    
    return (STATUS_SUCCESS);
}

//*****************************************************************************
//
//! Receives a byte that has been sent to the I2C Master Module.
//!
//! \param baseAddress is the base address of the I2C module.
//!
//! This function reads a byte of data from the I2C receive data Register.
//!
//! \return Returns the byte received from by the I2C module, cast as an
//! uint8_t.
//
//*****************************************************************************
uint8_t USCI_B_I2C_masterSingleReceive (uint32_t baseAddress)
{
    //Read a byte.
    return (HWREG8(baseAddress + OFS_UCBxRXBUF));
}

//*****************************************************************************
//
//! Returns the address of the RX Buffer of the I2C for the DMA module.
//!
//! \param baseAddress is the base address of the I2C module.
//!
//! Returns the address of the I2C RX Buffer. This can be used in conjunction
//! with the DMA to store the received data directly to memory.
//!
//! \return NONE
//
//*****************************************************************************
uint32_t USCI_B_I2C_getReceiveBufferAddressForDMA (uint32_t baseAddress)
{
    return ( baseAddress + OFS_UCBxRXBUF );
}

//*****************************************************************************
//
//! Returns the address of the TX Buffer of the I2C for the DMA module.
//!
//! \param baseAddress is the base address of the I2C module.
//!
//! Returns the address of the I2C TX Buffer. This can be used in conjunction
//! with the DMA to obtain transmitted data directly from memory.
//!
//! \return NONE
//
//*****************************************************************************
uint32_t USCI_B_I2C_getTransmitBufferAddressForDMA (uint32_t baseAddress)
{
    return ( baseAddress + OFS_UCBxTXBUF );
}

//*****************************************************************************
//
//Close the Doxygen group.
//! @}
//
//*****************************************************************************
