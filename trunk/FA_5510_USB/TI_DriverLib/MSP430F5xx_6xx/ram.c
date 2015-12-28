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
//ram.c - Driver for the RAM Controller Module.
//
//*****************************************************************************
#include "inc/hw_regaccess.h"
#include "assert.h"
#include "ram.h"
#ifdef  __IAR_SYSTEMS_ICC__
#include "deprecated/IAR/msp430xgeneric.h"
#else
#include "deprecated/CCS/msp430xgeneric.h"
#endif

//*****************************************************************************
//
//! Set specified RAM sector off
//!
//! \param baseAddress is the base address of the Timer module.
//! \param sector is specified sector to be set off.
//! Valid values are a logical OR of
//!         \b RAM_SECTOR0
//!         \b RAM_SECTOR1
//!         \b RAM_SECTOR2
//!         \b RAM_SECTOR3
//!         \b RAM_SECTOR4
//!         \b RAM_SECTOR5
//!         \b RAM_SECTOR6
//!         \b RAM_SECTOR7
//!
//! Modified register RCCTL0
//!
//! \return None
//
//*****************************************************************************
void RAM_setSectorOff (uint32_t baseAddress,
    uint8_t sector
    )
{
    assert(0x00 == sector & (~(RAM_SECTOR0 +
                               RAM_SECTOR1 +
                               RAM_SECTOR2 +
                               RAM_SECTOR3 
                               )
                             )

        );
    //Write key to start write to RCCTL0 and sector
    HWREG16(baseAddress + OFS_RCCTL0) = (RCKEY + sector);
}

//*****************************************************************************
//
//! Get RAM sector ON/OFF status
//!
//! \param baseAddress is the base address of the Timer module.
//! \param sector is specified sector
//! Valid values are a logical OR of
//!         \b RAM_SECTOR0
//!         \b RAM_SECTOR1
//!         \b RAM_SECTOR2
//!         \b RAM_SECTOR3
//!         \b RAM_SECTOR4
//!         \b RAM_SECTOR5
//!         \b RAM_SECTOR6
//!         \b RAM_SECTOR7
//!
//! Modified register RCCTL0
//!
//! \return Mask of the sector state that are ON
//
//*****************************************************************************
uint8_t RAM_getSectorState (uint32_t baseAddress,
    uint8_t sector
    )
{
    assert(0x00 == sector & (~(RAM_SECTOR0 +
                               RAM_SECTOR1 +
                               RAM_SECTOR2 +
                               RAM_SECTOR3 
                               )
                             )

        );
    return (HWREG8(baseAddress + OFS_RCCTL0_L) & sector);
}

//*****************************************************************************
//
//Close the Doxygen group.
//! @}
//
//*****************************************************************************
