//*****************************************************************************
//
//! @file radio_task.c
//!
//! @brief Task to handle radio operation.
//!
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
// Global includes for this project.
//
//*****************************************************************************
#include "ble_freertos_txpower_ctrl.h"

//*****************************************************************************
//
// WSF standard includes.
//
//*****************************************************************************
#include "wsf_types.h"
#include "wsf_trace.h"
#include "wsf_buf.h"
#include "wsf_timer.h"

//*****************************************************************************
//
// Includes for operating the ExactLE stack.
//
//*****************************************************************************
#include "hci_handler.h"
#include "dm_handler.h"
#include "l2c_handler.h"
#include "att_handler.h"
#include "smp_handler.h"
#include "l2c_api.h"
#include "att_api.h"
#include "smp_api.h"
#include "app_api.h"
#include "hci_core.h"
#include "hci_drv.h"
#include "hci_drv_apollo.h"
#include "hci_drv_cooper.h"
#include "hci_apollo_config.h"

#include "wsf_msg.h"

//*****************************************************************************
//
//
//*****************************************************************************
#include "app_ui.h"

//*****************************************************************************
//
// Radio task handle.
//
//*****************************************************************************
TaskHandle_t radio_task_handle;

//*****************************************************************************
//
// Function prototypes
//
//*****************************************************************************
void exactle_stack_init(void);
void button_handler(wsfEventMask_t event, wsfMsgHdr_t *pMsg);
void setup_buttons(void);
//*****************************************************************************
//
// Timer for buttons.
//
//*****************************************************************************
wsfHandlerId_t ButtonHandlerId;
wsfTimer_t ButtonTimer;

txPowerLevel_t  tx_power_level = TX_POWER_LEVEL_PLUS_6P0_dBm;
uint32_t        dtm_in_progress = false;

//*****************************************************************************
//
// WSF buffer pools.
//
//*****************************************************************************
#define WSF_BUF_POOLS               4

// Important note: the size of g_pui32BufMem should includes both overhead of internal
// buffer management structure, wsfBufPool_t (up to 16 bytes for each pool), and pool
// description (e.g. g_psPoolDescriptors below).

// Memory for the buffer pool
#if defined(USE_NONBLOCKING_HCI)
AM_SHARED_RW static uint32_t g_pui32BufMem[
        (WSF_BUF_POOLS*16
         + 16*8 + 32*4 + 64*6 + 280*8) / sizeof(uint32_t)];
#else
static uint32_t g_pui32BufMem[
        (WSF_BUF_POOLS*16
         + 16*8 + 32*4 + 64*6 + 280*8) / sizeof(uint32_t)];
#endif

// Default pool descriptor.
static wsfBufPoolDesc_t g_psPoolDescriptors[WSF_BUF_POOLS] =
{
    {  16,  8 },
    {  32,  4 },
    {  64,  6 },
    { 280,  8 }
};

//*****************************************************************************
//
// Tracking variable for the scheduler timer.
//
//*****************************************************************************

void radio_timer_handler(void);



//*****************************************************************************
//
// Poll the buttons.
//
//*****************************************************************************
void button_handler(wsfEventMask_t event, wsfMsgHdr_t *pMsg)
{
    uint32_t ui32ButtonTimer = 10 * 1000;

    //
    // Restart the button timer.
    //
#if AM_BSP_NUM_BUTTONS
    ui32ButtonTimer = 10;
#endif
    WsfTimerStartMs(&ButtonTimer, ui32ButtonTimer);

#if AM_BSP_NUM_BUTTONS
    //
    // Every time we get a button timer tick, check all of our buttons.
    //
    am_devices_button_array_tick(am_bsp_psButtons, AM_BSP_NUM_BUTTONS);

    //
    // If we got a a press, do something with it.
    //
    if ( am_devices_button_released(am_bsp_psButtons[0]) )
    {
        AppUiBtnTest(APP_UI_BTN_1_SHORT);
#endif

        HciVscSetRfPowerLevelEx(tx_power_level);

        switch ( tx_power_level )
        {
            case TX_POWER_LEVEL_MINUS_20P0_dBm:
                am_util_debug_printf("Current Tx Power is -20.0 dBm\n");
                tx_power_level = TX_POWER_LEVEL_MINUS_15P0_dBm;
                break;
            case TX_POWER_LEVEL_MINUS_15P0_dBm:
                am_util_debug_printf("Current Tx Power is -15.0 dBm\n");
                tx_power_level = TX_POWER_LEVEL_MINUS_10P0_dBm;
                break;
            case TX_POWER_LEVEL_MINUS_10P0_dBm:
                am_util_debug_printf("Current Tx Power is -10.0 dBm\n");
                tx_power_level = TX_POWER_LEVEL_MINUS_5P0_dBm;
                break;
            case TX_POWER_LEVEL_MINUS_5P0_dBm:
                am_util_debug_printf("Current Tx Power is -5.0 dBm\n");
                tx_power_level = TX_POWER_LEVEL_0P0_dBm;
                break;
            case TX_POWER_LEVEL_0P0_dBm:
                am_util_debug_printf("Current Tx Power is 0.0 dBm\n");
                tx_power_level = TX_POWER_LEVEL_PLUS_3P0_dBm;
                break;
            case TX_POWER_LEVEL_PLUS_3P0_dBm:
                am_util_debug_printf("Current Tx Power is +3.0 dBm\n");
                tx_power_level = TX_POWER_LEVEL_PLUS_4P0_dBm;
                break;
            case TX_POWER_LEVEL_PLUS_4P0_dBm:
                am_util_debug_printf("Current Tx Power is +4.0 dBm\n");
                tx_power_level = TX_POWER_LEVEL_PLUS_6P0_dBm;
                break;
            case TX_POWER_LEVEL_PLUS_6P0_dBm:
                am_util_debug_printf("Current Tx Power is +6.0 dBm\n");
                tx_power_level = TX_POWER_LEVEL_MINUS_20P0_dBm;
                break;
            default:
                am_util_debug_printf("Invalid Tx power level\n");
                break;
        }
#if AM_BSP_NUM_BUTTONS
    }

    if ( am_devices_button_released(am_bsp_psButtons[1]) )
    {
        AppUiBtnTest(APP_UI_BTN_2_SHORT);

        if ( dtm_in_progress )
        {
            HciLeTestEndCmd();
            dtm_in_progress = false;
            am_util_debug_printf("Transmitter test ended\n");
        }
        else
        {
            // Resetting controller first.
            HciResetCmd();

            // HciLeTransmitterTestCmd
            HciLeTransmitterTestCmd(0, 255, 7);

            dtm_in_progress = true;
            am_util_debug_printf("Transmitter test started\n");
        }
    }
#endif
}

//*****************************************************************************
//
// Sets up a button interface.
//
//*****************************************************************************
void setup_buttons(void)
{
#if AM_BSP_NUM_BUTTONS
    //
    // Enable the buttons for user interaction.
    //
    am_devices_button_array_init(am_bsp_psButtons, AM_BSP_NUM_BUTTONS);
#endif
    ButtonTimer.handlerId = ButtonHandlerId;
    WsfTimerStartSec(&ButtonTimer, 2);
}


//*****************************************************************************
//
// Initialization for the ExactLE stack.
//
//*****************************************************************************
void
exactle_stack_init(void)
{
    wsfHandlerId_t handlerId;
    uint16_t       wsfBufMemLen;
    //
    // Set up timers for the WSF scheduler.
    //
    WsfOsInit();
    WsfTimerInit();

    //
    // Initialize a buffer pool for WSF dynamic memory needs.
    //
    wsfBufMemLen = WsfBufInit(sizeof(g_pui32BufMem), (uint8_t *)g_pui32BufMem, WSF_BUF_POOLS,
               g_psPoolDescriptors);

    if (wsfBufMemLen > sizeof(g_pui32BufMem))
    {
        am_util_debug_printf("Memory pool is too small by %d\r\n",
                             wsfBufMemLen - sizeof(g_pui32BufMem));
    }

    //
    // Initialize the WSF security service.
    //
    SecInit();
    SecAesInit();
    SecCmacInit();
    SecEccInit();

    //
    // Set up callback functions for the various layers of the ExactLE stack.
    //
    handlerId = WsfOsSetNextHandler(HciHandler);
    HciHandlerInit(handlerId);

    handlerId = WsfOsSetNextHandler(DmHandler);
    DmDevVsInit(0);
    DmAdvInit();
    DmConnInit();
    DmConnSlaveInit();
    DmSecInit();
    DmSecLescInit();
    DmPrivInit();
    DmHandlerInit(handlerId);

    handlerId = WsfOsSetNextHandler(L2cSlaveHandler);
    L2cSlaveHandlerInit(handlerId);
    L2cInit();
    L2cSlaveInit();

    handlerId = WsfOsSetNextHandler(AttHandler);
    AttHandlerInit(handlerId);
    AttsInit();
    AttsIndInit();
    AttcInit();

    handlerId = WsfOsSetNextHandler(SmpHandler);
    SmpHandlerInit(handlerId);
    SmprInit();
    SmprScInit();
    HciSetMaxRxAclLen(251);

    handlerId = WsfOsSetNextHandler(AppHandler);
    AppHandlerInit(handlerId);


    ButtonHandlerId = WsfOsSetNextHandler(button_handler);

    handlerId = WsfOsSetNextHandler(HciDrvHandler);
    HciDrvHandlerInit(handlerId);
}

//*****************************************************************************
//
// GPIO interrupt handler.
//
//*****************************************************************************
void
am_cooper_irq_isr(void)
{
    uint32_t    ui32IntStatus;

    AM_CRITICAL_BEGIN
    am_hal_gpio_interrupt_irq_status_get(AM_COOPER_IRQn, false, &ui32IntStatus);
    am_hal_gpio_interrupt_irq_clear(AM_COOPER_IRQn, ui32IntStatus);
    AM_CRITICAL_END
    am_hal_gpio_interrupt_service(AM_COOPER_IRQn, ui32IntStatus);
}

//*****************************************************************************
//
// UART interrupt handler.
//
//*****************************************************************************
void
am_uart_isr(void)
{
    uint32_t ui32Status;

    //
    // Read and save the interrupt status, but clear out the status register.
    //
    ui32Status = UARTn(0)->MIS;
    UARTn(0)->IEC = ui32Status;

}

//*****************************************************************************
//
// Perform initial setup for the radio task.
//
//****************************************************************************/
void
RadioTaskSetup(void)
{
    am_util_debug_printf("RadioTask: setup\r\n");

    NVIC_SetPriority(COOPER_IOM_IRQn, NVIC_configMAX_SYSCALL_INTERRUPT_PRIORITY);
#if defined(COOPER_QFN)
    NVIC_SetPriority(GPIO0_001F_IRQn, NVIC_configMAX_SYSCALL_INTERRUPT_PRIORITY);
#else
    NVIC_SetPriority(GPIO0_203F_IRQn, NVIC_configMAX_SYSCALL_INTERRUPT_PRIORITY);
#endif
}


/*************************************************************************************************/
/*!
 *  \fn     TxPowerCtrlDmCback
 *
 *  \brief  Application DM callback.
 *
 *  \param  pDmEvt  DM callback event
 *
 *  \return None.
 */
/*************************************************************************************************/
static void TxPowerCtrlDmCback(dmEvt_t *pDmEvt)
{
  // do nothing
}

//*****************************************************************************
//
// Short Description.
//
//*****************************************************************************
void
RadioTask(void *pvParameters)
{
#if WSF_TRACE_ENABLED == TRUE
    //
    // Enable ITM
    //
    am_util_debug_printf("Starting wicentric trace:\n\n");
#endif

    //
    // Boot the radio.
    //
    HciDrvRadioBoot(1);

    //
    // Initialize the main ExactLE stack.
    //
    exactle_stack_init();

    // uncomment the following to set custom Bluetooth address here
    // {
    //     uint8_t bd_addr[6] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66};
    //     HciVscSetCustom_BDAddr(&bd_addr);
    // }

    //
    // Prep the buttons for use
    //

    setup_buttons();

    //
    // Need to register DM callback for Vendor specific command's event
    //

    DmRegister(TxPowerCtrlDmCback);

    am_util_debug_printf("Usage as below:\n");
#if AM_BSP_NUM_BUTTONS
    am_util_debug_printf("Short press Button 1 to enter/exit DTM tx test\n");
    am_util_debug_printf("Short press Button 0 to change tx power level from high to low.\n");
#else
    am_util_debug_printf("10s timer to switch between tx power levels\n");
    // Resetting controller first.
    HciResetCmd();

    // HciLeTransmitterTestCmd
    HciLeTransmitterTestCmd(0, 255, 7);

    am_util_debug_printf("Transmitter test started\n");
#endif

    while (1)
    {

        //
        // Calculate the elapsed time from our free-running timer, and update
        // the software timers in the WSF scheduler.
        //
        wsfOsDispatcher();

    }
}
