//*****************************************************************************
//
//  am_hal_otp.c
//! @file
//!
//! @brief Functions for OTP functions
//!
//! @addtogroup
//! @ingroup apollo4hal
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2020, Ambiq Micro, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// Third party software included in this distribution is subject to the
// additional license terms as defined in the /docs/licenses directory.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision b0-release-20201110-564-g8433a2a39 of the AmbiqSuite Development Package.
//
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"

/* poll on the AIB acknowledge bit */
#define AM_HAL_OTP_WAIT_ON_AIB_ACK_BIT()  \
    while (CRYPTO->AIBFUSEPROGCOMPLETED_b.AIBFUSEPROGCOMPLETED == 0)

static uint32_t validate_otp_offset(uint32_t offset)
{
    if ((offset & 0x3) || (offset > (AM_REG_OTP_SIZE - 4)) || (offset < AM_REG_OTP_SBL_WPROT0_O))
    {
        return AM_HAL_STATUS_OUT_OF_RANGE;
    }
    else
    {
        return AM_HAL_STATUS_SUCCESS;
    }
}

//*****************************************************************************
//
//! @brief  Read OTP word
//!
//! @param  pVal -  Pointer to word for returned data
//! @param  offset -  word aligned offset in OTP to be read
//!
//! This will retrieve the OTP information
//!
//! @return Returns AM_HAL_STATUS_SUCCESS on success
//
//*****************************************************************************
uint32_t am_hal_otp_read_word(uint32_t offset, uint32_t *pVal)
{
    uint32_t status = AM_HAL_STATUS_SUCCESS;
    if ((PWRCTRL->DEVPWRSTATUS_b.PWRSTCRYPTO == 0) || (CRYPTO->HOSTCCISIDLE_b.HOSTCCISIDLE == 0))
    {
        // Crypto is not accessible
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
    status = validate_otp_offset(offset);
    if (status == AM_HAL_STATUS_SUCCESS)
    {
        if (!pVal)
        {
            return AM_HAL_STATUS_INVALID_ARG;
        }
        *pVal = AM_REGVAL(AM_REG_OTP_BASEADDR + offset);
    }
    return status;
}

//*****************************************************************************
//
//! @brief  Write OTP word
//!
//! @param  value -  value to be written
//! @param  offset -  word aligned offset in OTP to be read
//!
//! This will write a word to the supplied offset in the OTP
//!
//! @return Returns AM_HAL_STATUS_SUCCESS on success
//
//*****************************************************************************
uint32_t am_hal_otp_write_word(uint32_t offset, uint32_t value)
{
    uint32_t status = AM_HAL_STATUS_SUCCESS;
    if ((PWRCTRL->DEVPWRSTATUS_b.PWRSTCRYPTO == 0) || (CRYPTO->HOSTCCISIDLE_b.HOSTCCISIDLE == 0))
    {
        // Crypto is not accessible
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
    status = validate_otp_offset(offset);
    if (status == AM_HAL_STATUS_SUCCESS)
    {
        AM_REGVAL(AM_REG_OTP_BASEADDR + offset) = value;
        AM_HAL_OTP_WAIT_ON_AIB_ACK_BIT();
        // Read back the value to compare
        if (AM_REGVAL(AM_REG_OTP_BASEADDR + offset) != value)
        {
            status = AM_HAL_STATUS_FAIL;
        }
    }
    return status;
}
