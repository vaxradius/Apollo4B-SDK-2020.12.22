//*****************************************************************************
//
//  am_hal_mcu_sysctrl.h
//! @file
//!
//! @brief Functions for interfacing with the M4F system control registers
//!
//! @addtogroup sysctrl4 System Control (SYSCTRL)
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
#ifndef AM_HAL_MCU_SYSCTRL_H
#define AM_HAL_MCU_SYSCTRL_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// Write flush - This function will hold the bus until all queued write
// operations have completed, thereby guaranteeing that all writes have
// been flushed.
//
//*****************************************************************************
#if 0
#define SYNC_READ       0x5FFF0000      // TODO - FIXME: Not required for Apollo4
extern volatile uint32_t g_ui32BusWriteFlush;
#define am_hal_sysctrl_bus_write_flush()                                \
    if ( 1 )                                                            \
    {                                                                   \
        uint32_t *pui32Flush = (uint32_t*)SYNC_READ;                    \
        g_ui32BusWriteFlush = *pui32Flush;                              \
    }
#endif

#define am_hal_sysctrl_membarrier()                                     \
    if (1)                                                              \
    {                                                                   \
        __DMB();                                                        \
        am_hal_sysctrl_bus_write_flush();                               \
    }

// This Write will begin only after all previous load/store operations are done
#define am_hal_sysctrl_membarrier_write(addr, data)                     \
    if (1)                                                              \
    {                                                                   \
        am_hal_sysctrl_membarrier();                                    \
        *((uint32_t *)(addr)) = (data);                                 \
    }

// This Read will be performed before any subsequent load/store operations are done
#define am_hal_sysctrl_membarrier_read(addr)                            \
    if (1)                                                              \
    {                                                                   \
        volatile uint32_t ui32tmp;                                      \
        ui32tmp1 = *((uint32_t *)(addr));                               \
        am_hal_sysctrl_membarrier();                                    \
        g_ui32BusWriteFlush = ui32tmp;                                  \
    }


//*****************************************************************************
//
// External function definitions
//
//*****************************************************************************
extern void am_hal_sysctrl_fpu_enable(void);
extern void am_hal_sysctrl_fpu_disable(void);
extern void am_hal_sysctrl_fpu_stacking_enable(bool bLazy);
extern void am_hal_sysctrl_fpu_stacking_disable(void);
extern void am_hal_sysctrl_aircr_reset(void);
#ifdef __cplusplus
}
#endif

#endif // AM_HAL_MCU_SYSCTRL_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

