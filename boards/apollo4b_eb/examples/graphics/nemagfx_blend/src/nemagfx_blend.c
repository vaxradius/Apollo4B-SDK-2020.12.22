//*****************************************************************************
//
//! @file nemagfx_blend.c
//!
//! @brief NemaGFX example.
//! brief Example that demonstrates blend feature
//! Blending requires a series of calculations between the source (foreground)
//! and destination (background)color fragments for producing the final color,
//! which will be written in memory.This example use a constent table inside
//! most of the supported blending mode.demonstrates each more every 1 second.
//! the dst color is nema_rgba(0xff, 0, 0, 0x80), which is red color with 50%
//! alpha blending, the src color is nema_rgba(0, 0, 0xff, 0x80), which is blue
//! color with 50% alpha blending.
//!
//! Printing takes place over the ITM at 1M Baud.

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

//*****************************************************************************
//
// This application has a large number of common include files. For
// convenience, we'll collect them all together in a single header and include
// that everywhere.
//
//*****************************************************************************
#include "nemagfx_blend.h"
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"
#include "nema_hal.h"

//#define AM_DEBUG_PRINTF

extern int fillrect_grad(void);

//*****************************************************************************
//
// Enable printing to the console.
//
//*****************************************************************************
void
enable_print_interface(void)
{
    //
    // Initialize a debug printing interface.
    //
    am_bsp_debug_printf_enable();
}

//*****************************************************************************
//
// Enable printing to the console.
//
//*****************************************************************************
void
disable_print_interface(void)
{
    //
    // Initialize a debug printing interface.
    //
    am_bsp_debug_printf_disable();
    am_hal_itm_disable();
}


//*****************************************************************************
//
// Main Function
//
//*****************************************************************************
int
main(void)
{
    //
    // Initialize the printf interface for ITM/SWO output.
    //
    am_bsp_itm_printf_enable();
    //
    // Clear the terminal and print the banner.
    //
    am_util_stdio_terminal_clear();

    //
    // If you want to change display configurations in example, please change them here as below.
    //
    // g_eDispType = RM67162_DSI;
    // g_sDsiCfg.ui8NumLanes = 1;
    // g_sDsiCfg.eDbiWidth = AM_HAL_DSI_DBI_WIDTH_8;
    // g_sDsiCfg.eDsiFreq = AM_HAL_DSI_FREQ_TRIM_X12;

    if (g_sDispCfg[g_eDispType].eInterface == IF_DSI)
    {
        //
        // Enable DSI power and configure DSI clock.
        //
        am_hal_dsi_init();
    }
    else
    {
        if (g_sDispCfg[g_eDispType].bUseDPHYPLL == true)
        {
            am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_DISPCLKSEL_DPHYPLL, NULL);
            am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_DCCLK_ENABLE, NULL);
            am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_PLLCLKSEL_HFRC12, NULL);
            am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_PLLCLK_ENABLE, NULL);
        }
        else
        {
            am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_DISPCLKSEL_HFRC96, NULL);
            am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_DCCLK_ENABLE, NULL);
        }
    }

    //
    // Global interrupt enable
    //
    am_hal_interrupt_master_enable();

    am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_GFX);
    am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_DISP);
    if (g_sDispCfg[g_eDispType].bUseDPHYPLL == true)
    {
        am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_DISPPHY);
    }

    //
    // Initialize the printf interface for ITM output
    //
    am_bsp_debug_printf_enable();
    //
    // Enable printing to the console.
    //
#ifdef AM_DEBUG_PRINTF
    enable_print_interface();
#endif

    CPU->DAXICFG_b.BUFFERENABLE = CPU_DAXICFG_BUFFERENABLE_ONE;  // only enable a single buffer

    PWRCTRL->SSRAMPWREN = PWRCTRL_SSRAMPWREN_PWRENSSRAM_ALL;  // enable all
    while(PWRCTRL->SSRAMPWRST == 0);

    am_hal_timer_config_t       TimerConfig;
    am_hal_timer_default_config_set(&TimerConfig);
    TimerConfig.eInputClock = AM_HAL_TIMER_CLOCK_HFRC_DIV16;
    TimerConfig.eFunction = AM_HAL_TIMER_FN_CONTINUOUS;
    am_hal_timer_config(0, &TimerConfig);
    am_hal_timer_start(0);


#ifdef BAREMETAL
    test_blend();
#else /* BAREMETAL */
    //
    // Initialize plotting interface.
    //
    am_util_debug_printf("FreeRTOS NemaGFX_SW Transition Effect Example\n");

    //
    // Run the application.
    //
    run_tasks();
#endif /* BAREMETAL*/

    //
    // We shouldn't ever get here.
    //
    while (1)
    {
    }

}

