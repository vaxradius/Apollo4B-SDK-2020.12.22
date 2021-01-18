//*****************************************************************************
//
//! @file hci_drv_cooper.c
//!
//! @brief HCI driver interface.
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
#include "am_util.h"
#include "am_bsp.h"

#include "wsf_types.h"
#include "wsf_timer.h"
#include "bstream.h"
#include "wsf_msg.h"
#include "wsf_cs.h"
#include "hci_drv.h"
#include "hci_drv_apollo.h"
#include "hci_tr_apollo.h"
#include "hci_core.h"
#include "dm_api.h"

#include "hci_drv_cooper.h"
#include "am_devices_cooper.h"
#include "hci_dbg_trc.h"

#include <string.h>

//*****************************************************************************
//
// Configurable buffer sizes.
//
//*****************************************************************************
#define HCI_DRV_MAX_TX_PACKET           524 // the max packet of SBL to controller is 512 plus 12 bytes header
#define HCI_DRV_MAX_RX_PACKET           256

//*****************************************************************************
//
// Configurable error-detection thresholds.
//
//*****************************************************************************

// Buffer for non-blocking transactions
#if ((defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)) && (defined(USE_NONBLOCKING_HCI)))
AM_SHARED_RW uint32_t DMATCBBuffer[HCI_DRV_MAX_TX_PACKET / 4];
#else
uint32_t DMATCBBuffer[HCI_DRV_MAX_TX_PACKET / 4];
#endif

// set the BLE MAC address to a special value
uint8_t g_BLEMacAddress[6] = {0};

//*****************************************************************************
//
// Global variables.
//
//*****************************************************************************
// BLE module handle
void *g_IomDevHdl;
void *pvHciSpiHandle;

// Global handle used to send BLE events about the Hci driver layer.
wsfHandlerId_t g_HciDrvHandleID = 0;

//*****************************************************************************
//
// Structure for holding outgoing HCI packets.
//
//*****************************************************************************

// Buffers for HCI read data.
#if ((defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)) && (defined(USE_NONBLOCKING_HCI)))
AM_SHARED_RW uint32_t g_pui32ReadBuffer[HCI_DRV_MAX_RX_PACKET / 4];
#else
uint32_t g_pui32ReadBuffer[HCI_DRV_MAX_RX_PACKET / 4];
#endif
uint8_t *g_pui8ReadBuffer = (uint8_t *) g_pui32ReadBuffer;

uint32_t g_ui32NumBytes = 0;
uint32_t g_consumed_bytes = 0;

#if defined(USE_NONBLOCKING_HCI)
void hciDrvWriteCallback(void *pCallbackCtxt);
void hciDrvReadCallback(void *pCallbackCtxt);
#endif
//*****************************************************************************
//
// Events for the HCI driver interface.
//
//*****************************************************************************
#define BLE_TRANSFER_NEEDED_EVENT                   0x01

//*****************************************************************************
//
// Error-handling wrapper macro.
//
//*****************************************************************************
#define ERROR_CHECK_VOID(status)                                              \
    {                                                                         \
        uint32_t ui32ErrChkStatus;                                            \
        if (0 != (ui32ErrChkStatus = (status)))                               \
        {                                                                     \
            am_util_debug_printf("ERROR_CHECK_VOID "#status "\n");            \
            error_check(ui32ErrChkStatus);                                    \
            return;                                                           \
        }                                                                     \
    }

#define ERROR_RETURN(status, retval)                                          \
    if ((status))                                                             \
    {                                                                         \
        error_check(status);                                                  \
        return (retval);                                                      \
    }

#define ERROR_RECOVER(status)                                                 \
    if ((status))                                                             \
    {                                                                         \
        error_check(status);                                                  \
        HciDrvRadioShutdown();                                                \
        HciDrvRadioBoot(0);                                                   \
        DmDevReset();                                                         \
        return;                                                               \
    }

//*****************************************************************************
//
// Debug section.
//
//*****************************************************************************
#if 1
#define CRITICAL_PRINT(...)                                                   \
    do                                                                        \
    {                                                                         \
        AM_CRITICAL_BEGIN;                                                    \
        am_util_debug_printf(__VA_ARGS__);                                    \
        AM_CRITICAL_END;                                                      \
    } while (0)
#else
#define CRITICAL_PRINT(...)
#endif

//*****************************************************************************
//
// Function pointer for redirecting errors
//
//*****************************************************************************
hci_drv_error_handler_t g_hciDrvErrorHandler = 0;
static uint32_t g_ui32FailingStatus = 0;

//*****************************************************************************
//
// By default, errors will be printed. If there is an error handler defined,
// they will be sent there intead.
//
//*****************************************************************************
static void
error_check(uint32_t ui32Status)
{
    //
    // Don't do anything unless there's an error.
    //
    if (ui32Status)
    {
        //
        // Set the global error status. If there's an error handler function,
        // call it. Otherwise, just print the error status and wait.
        //
        g_ui32FailingStatus = ui32Status;

        if (g_hciDrvErrorHandler)
        {
            g_hciDrvErrorHandler(g_ui32FailingStatus);
        }
        else
        {
            CRITICAL_PRINT("Error detected: 0x%08x\n", g_ui32FailingStatus);
        }
    }
}

//*****************************************************************************
//
// Simple interrupt handler to call
//
// Note: These two lines need to be added to the exactle initialization
// function at the beginning of all Cordio applications:
//
//     handlerId = WsfOsSetNextHandler(HciDrvHandler);
//     HciDrvHandler(handlerId);
//
//*****************************************************************************
#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
static void HciDrvIntService(void *pArg)
#else
void
HciDrvIntService(void)
#endif
{
    //
    // Send an event to get processed in the HCI handler.
    //
    WsfSetEvent(g_HciDrvHandleID, BLE_TRANSFER_NEEDED_EVENT);
}

#if ((defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)) && (!defined(COOPER_QFN)))
static void ClkReqIntService(void *pArg)
{
    if(am_devices_cooper_clkreq_read(g_IomDevHdl))
    {
        // Power up the 32MHz Crystal
        am_hal_mcuctrl_control(AM_HAL_MCUCTRL_CONTROL_EXTCLK32M_KICK_START, 0);
        am_devices_cooper_clkreq_change_dir(AM_HAL_GPIO_PIN_INTDIR_HI2LO);
        // Notify Cooper
        //am_hal_gpio_state_write(AM_DEVICES_COOPER_CLKACK_PIN, AM_HAL_GPIO_OUTPUT_SET);
    }
    else
    {
        am_hal_mcuctrl_control(AM_HAL_MCUCTRL_CONTROL_EXTCLK32M_DISABLE, 0);
        am_devices_cooper_clkreq_change_dir(AM_HAL_GPIO_PIN_INTDIR_LO2HI);
        // Clear and power down
        // am_hal_gpio_state_write(AM_DEVICES_COOPER_CLKACK_PIN, AM_HAL_GPIO_OUTPUT_CLEAR);
    }
}

#endif

//*****************************************************************************
//
// Boot the radio.
//
//*****************************************************************************
uint32_t
HciDrvRadioBoot(bool bColdBoot)
{
    uint32_t ui32Status = AM_DEVICES_COOPER_STATUS_ERROR;

    am_devices_cooper_config_t stCooperConfig;
    stCooperConfig.ui32ClockFreq = COOPER_IOM_FREQ;
    stCooperConfig.pNBTxnBuf = DMATCBBuffer;
    stCooperConfig.ui32NBTxnBufLength = sizeof(DMATCBBuffer) / 4;

    //
    // Initialize the SPI module.
    //
    ui32Status = am_devices_cooper_init(SPI_MODULE, &stCooperConfig, &g_IomDevHdl, &pvHciSpiHandle);

    if (AM_DEVICES_COOPER_STATUS_SUCCESS != ui32Status)
    {
        return AM_HAL_STATUS_FAIL;
    }
    //
    // Set the BLE TX Output power to 0dBm.
    //
    //am_devices_cooper_tx_power_set(g_IomDevHdl, 0x8);

    #if defined(USE_NONBLOCKING_HCI)
        am_hal_iom_interrupt_clear(pvHciSpiHandle, AM_HAL_IOM_INT_DCMP | AM_HAL_IOM_INT_CMDCMP);
        am_hal_iom_interrupt_enable(pvHciSpiHandle, AM_HAL_IOM_INT_DCMP | AM_HAL_IOM_INT_CMDCMP);
        NVIC_EnableIRQ(COOPER_IOM_IRQn);
    #endif

#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
    uint32_t IntNum = AM_DEVICES_COOPER_IRQ_PIN;
    am_hal_gpio_state_write(AM_DEVICES_COOPER_IRQ_PIN, AM_HAL_GPIO_OUTPUT_CLEAR);
    am_hal_gpio_interrupt_register(AM_HAL_GPIO_INT_CHANNEL_0, AM_DEVICES_COOPER_IRQ_PIN, HciDrvIntService, NULL);
    am_hal_gpio_interrupt_control(AM_HAL_GPIO_INT_CHANNEL_0,
                                  AM_HAL_GPIO_INT_CTRL_INDV_ENABLE,
                                  (void *)&IntNum);
#if !defined(COOPER_QFN)
    IntNum = AM_DEVICES_COOPER_CLKREQ_PIN;
    am_hal_gpio_state_write(AM_DEVICES_COOPER_CLKREQ_PIN, AM_HAL_GPIO_OUTPUT_CLEAR);
    am_hal_gpio_interrupt_register(AM_HAL_GPIO_INT_CHANNEL_0, AM_DEVICES_COOPER_CLKREQ_PIN, ClkReqIntService, NULL);
    am_hal_gpio_interrupt_control(AM_HAL_GPIO_INT_CHANNEL_0,
                                  AM_HAL_GPIO_INT_CTRL_INDV_ENABLE,
                                  (void *)&IntNum);
#endif
#else
    AM_HAL_GPIO_MASKCREATE(GpioIntMask);
    am_hal_gpio_interrupt_clear( AM_HAL_GPIO_MASKBIT(pGpioIntMask, AM_DEVICES_COOPER_IRQ_PIN));
    am_hal_gpio_interrupt_register(AM_DEVICES_COOPER_IRQ_PIN, HciDrvIntService);
    am_hal_gpio_interrupt_enable(AM_HAL_GPIO_MASKBIT(pGpioIntMask, AM_DEVICES_COOPER_IRQ_PIN));
#endif
    NVIC_EnableIRQ(AM_COOPER_IRQn);

    // When it's bColdBoot, it will use Apollo's Device ID to form Bluetooth address.
    if (bColdBoot)
    {
        am_hal_mcuctrl_device_t sDevice;
        am_hal_mcuctrl_info_get(AM_HAL_MCUCTRL_INFO_DEVICEID, &sDevice);

        // Bluetooth address formed by ChipID1 (32 bits) and ChipID0 (8-23 bits).
        memcpy(g_BLEMacAddress, &sDevice.ui32ChipID1, sizeof(sDevice.ui32ChipID1));
        // ui32ChipID0 bit 8-31 is test time during chip manufacturing
        g_BLEMacAddress[4] = (sDevice.ui32ChipID0 >> 8) & 0xFF;
        g_BLEMacAddress[5] = (sDevice.ui32ChipID0 >> 16) & 0xFF;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Shut down the BLE core.
//
//*****************************************************************************
void
HciDrvRadioShutdown(void)
{
#if defined(USE_NONBLOCKING_HCI)
    NVIC_DisableIRQ(COOPER_IOM_IRQn);
#endif
#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
    uint32_t IntNum = AM_DEVICES_COOPER_IRQ_PIN;
    am_hal_gpio_interrupt_control(AM_HAL_GPIO_INT_CHANNEL_0,
                                  AM_HAL_GPIO_INT_CTRL_INDV_DISABLE,
                                  (void *)&IntNum);
#if !defined(COOPER_QFN)
	IntNum = AM_DEVICES_COOPER_CLKREQ_PIN;
    am_hal_gpio_interrupt_control(AM_HAL_GPIO_INT_CHANNEL_0,
                                  AM_HAL_GPIO_INT_CTRL_INDV_DISABLE,
                                  (void *)&IntNum);
#endif
#else
    AM_HAL_GPIO_MASKCREATE(GpioIntMask);
    am_hal_gpio_interrupt_disable(AM_HAL_GPIO_MASKBIT(pGpioIntMask, AM_DEVICES_COOPER_IRQ_PIN));
#endif

    am_devices_cooper_term(g_IomDevHdl);
}

//*****************************************************************************
//
// Function used by the BLE stack to send HCI messages to the BLE controller.
//
// Internally, the Cordio BLE stack will allocate memory for an HCI message,
//
//*****************************************************************************

uint16_t
hciDrvWrite(uint8_t type, uint16_t len, uint8_t *pData)
{
    uint32_t ui32ErrorStatus = 0;

#if defined(USE_NONBLOCKING_HCI)
    ui32ErrorStatus = am_devices_cooper_nonblocking_write(g_IomDevHdl, type, (uint32_t*)pData, (uint16_t)len, hciDrvWriteCallback, NULL);
#else
    ui32ErrorStatus = am_devices_cooper_blocking_write(g_IomDevHdl, type, (uint32_t*)pData, (uint16_t)len);
#endif
    if(AM_DEVICES_COOPER_STATUS_SUCCESS == ui32ErrorStatus)
    {
        return len;
    }
    else
    {
        return 0;
    }
}

//*****************************************************************************
//
// Save the handler ID of the HciDrvHandler so we can send it events through
// the WSF task system.
//
// Note: These two lines need to be added to the exactle initialization
// function at the beginning of all Cordio applications:
//
//     handlerId = WsfOsSetNextHandler(HciDrvHandler);
//     HciDrvHandler(handlerId);
//
//*****************************************************************************
void
HciDrvHandlerInit(wsfHandlerId_t handlerId)
{
    g_HciDrvHandleID = handlerId;
}

#if defined(USE_NONBLOCKING_HCI)
//*****************************************************************************
//
// This function determines what to do when a write operation completes.
//
//*****************************************************************************
void hciDrvWriteCallback(void *pCallbackCtxt)
{
    //TODO
}

//*****************************************************************************
//
// This function determines what to do when a read operation completes.
//
//*****************************************************************************
void hciDrvReadCallback(void *pCallbackCtxt)
{
#ifdef ENABLE_BLE_CTRL_TRACE
        if(*g_pui8ReadBuffer == HCI_TRACE_TYPE)
        {
        #if 0
            {
              char tmp[400] = {0};
              int i = 0;
              int tmp_len = g_ui32NumBytes;
              if(tmp_len > 100) tmp_len = 100;

              for(i=0; i<tmp_len; i++)
                  sprintf(tmp+(i*3), "%02x ", g_pui8ReadBuffer[i]);

              am_util_debug_printf("Rx dbg trace: %s, len :%d\n", tmp, g_ui32NumBytes);
            }
        #endif

            hci_process_trace_data(g_pui8ReadBuffer+1, g_ui32NumBytes-1);
            g_consumed_bytes = g_ui32NumBytes;
        }
        else
#endif
    {
        g_consumed_bytes = hciTrSerialRxIncoming(g_pui8ReadBuffer, g_ui32NumBytes);
        if(g_consumed_bytes != g_ui32NumBytes)
        {
            // we need to come back again.
            WsfSetEvent(g_HciDrvHandleID, BLE_TRANSFER_NEEDED_EVENT);
        }
    }
}
#endif

//*****************************************************************************
//
// Event handler for HCI-related events.
//
// This handler can perform HCI reads or writes, and keeps the actions in the
// correct order.
//
//*****************************************************************************
void
HciDrvHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg)
{
    uint32_t ui32ErrorStatus = 0;

    //
    // Check to see if we read any bytes over the HCI interface that we haven't
    // already sent to the BLE stack.
    //
    if (g_ui32NumBytes > g_consumed_bytes)
    {
        //
        // If we have any bytes saved, we should send them to the BLE stack
        // now.
        //
        g_consumed_bytes += hciTrSerialRxIncoming(g_pui8ReadBuffer + g_consumed_bytes,
                                                  g_ui32NumBytes - g_consumed_bytes);

        //
        // If the stack doesn't accept all of the bytes we had, we will need to
        // keep the event set and come back later. Otherwise, we can just reset
        // our variables and exit the loop.
        //
        if (g_consumed_bytes != g_ui32NumBytes)
        {
            WsfSetEvent(g_HciDrvHandleID, BLE_TRANSFER_NEEDED_EVENT);
            return;
        }
        else
        {
            g_ui32NumBytes   = 0;
            g_consumed_bytes = 0;
        }
    }

    // Reset
    g_ui32NumBytes = 0;
#if defined(USE_NONBLOCKING_HCI)
    ui32ErrorStatus = am_devices_cooper_nonblocking_read(g_IomDevHdl, g_pui32ReadBuffer, &g_ui32NumBytes, hciDrvReadCallback, NULL);
#else
    ui32ErrorStatus = am_devices_cooper_blocking_read(g_IomDevHdl, g_pui32ReadBuffer, &g_ui32NumBytes);
#endif
    if (g_ui32NumBytes > HCI_DRV_MAX_RX_PACKET)
    {
        CRITICAL_PRINT("ERROR: Trying to receive an HCI packet larger than the hci driver buffer size (needs %d bytes of space).",
                       g_ui32NumBytes);
        ERROR_CHECK_VOID(HCI_DRV_RX_PACKET_TOO_LARGE);
    }
    else if ( ui32ErrorStatus != AM_HAL_STATUS_SUCCESS)
    {
        //
        // If the read didn't succeed for some physical reason, we need
        // to know. We shouldn't get failures here. We checked the IRQ
        // signal before calling the read function, and this driver
        // only contains a single call to the blocking read function,
        // so there shouldn't be any physical reason for the read to
        // fail.
        //
        CRITICAL_PRINT("HCI READ failed with status %d. Try recording with a logic analyzer to catch the error.\n",
                       ui32ErrorStatus);
        ERROR_RECOVER(ui32ErrorStatus);
    }
    else
    {
#if defined(USE_NONBLOCKING_HCI)
#else
    #ifdef ENABLE_BLE_CTRL_TRACE
        if(*g_pui8ReadBuffer == HCI_TRACE_TYPE)
        {
            #if 0
            {
              char tmp[400] = {0};
              int i = 0;
              int tmp_len = g_ui32NumBytes;
              if(tmp_len > 100) tmp_len = 100;

              for(i=0; i<tmp_len; i++)
                  sprintf(tmp+(i*3), "%02x ", g_pui8ReadBuffer[i]);

              am_util_debug_printf("Rx dbg trace: %s, len :%d\n", tmp, g_ui32NumBytes);
            }
            #endif

            hci_process_trace_data(g_pui8ReadBuffer+1, g_ui32NumBytes-1);
            g_consumed_bytes = g_ui32NumBytes;
        }
        else
    #endif
        {
            //
            // Pass the data along to the stack. The stack should be able
            // to read as much data as we send it.  If it can't, we need to
            // know that.
            //
            g_consumed_bytes = hciTrSerialRxIncoming(g_pui8ReadBuffer, g_ui32NumBytes);

            if (g_consumed_bytes != g_ui32NumBytes)
            {

                // need to come back again
                WsfSetEvent(g_HciDrvHandleID, BLE_TRANSFER_NEEDED_EVENT);
            }
        }
#endif
    }
}

//*****************************************************************************
//
// Register an error handler for the HCI driver.
//
//*****************************************************************************
void
HciDrvErrorHandlerSet(hci_drv_error_handler_t pfnErrorHandler)
{
    g_hciDrvErrorHandler = pfnErrorHandler;
}

static bool g_tx_test_started;
static bool g_rx_test_started;

bool test_in_progress()
{
    if(g_tx_test_started || g_rx_test_started)
    {
        return true;
    }
    else
    {
        return false;
    }
}
/*************************************************************************************************/
/*!
 *  \fn     HciVscTransmitterTest
 *
 *  \brief  Vendor-specific command for start transmitter testing
 *
 *  \param  tx_test_v3 pointer to the transmitter testing parameter
 *
 *  \return None
 */
/*************************************************************************************************/
void HciVscTransmitterTest(hciLeTxTestV3Cmd_t *tx_test_v3)
{
    g_tx_test_started = true;
    
    HciLeTransmitterTestCmdV3(tx_test_v3);
}


/*************************************************************************************************/
/*!
 *  \fn     HciVscReceiverTest
 *
 *  \brief  Vendor-specific command for receiver testing
 *
 *  \param  rx_test_v3 pointer to the receiver testing parameter
 *
 *  \return None
 */
/*************************************************************************************************/
void HciVscReceiverTest(hciLeRxTestV3Cmd_t *rx_test_v3)
{
    g_rx_test_started = true;
    HciLeReceiverTestCmdV3(rx_test_v3);
}

/*************************************************************************************************/
/*!
 *  \fn     HciVscTransmitterTestEnd
 *
 *  \brief  Vendor-specific command for ending Radio transmitter testing.
 *
 *  \param  None
 *
 *  \return None
 */
/*************************************************************************************************/
void HciVscTestEnd(void)
{
    if(test_in_progress())
    {
        HciLeTestEndCmd();
    }

    g_rx_test_started = false;
    g_tx_test_started = false;
}

/*************************************************************************************************/
/*!
 *  \fn     HciVscSetRfPowerLevelEx
 *
 *  \brief  Vendor specific command for settting Radio transmit power level
 *          for Nationz.
 *
 *  \param  txPowerlevel    valid range from 0 to 15 in decimal.
 *
 *  \return true when success, otherwise false
 */
/*************************************************************************************************/
bool HciVscSetRfPowerLevelEx(txPowerLevel_t txPowerlevel)
{
    // make sure it's 8 bit
    uint8_t tx_power_level = (uint8_t)txPowerlevel;

    if(tx_power_level < TX_POWER_LEVEL_INVALID) {
      HciVendorSpecificCmd(HCI_VSC_SET_TX_POWER_LEVEL, sizeof(tx_power_level), &tx_power_level);
      return true;
    }
    else {
      return false;
    }
}

/*************************************************************************************************/
/*!
 *  \brief  read memory variable
 *
 *  \param  start_addr   Start address to read
 *  \param  size         Access size
 *  \param  length       Length to read
 *
 *  \return true when success, otherwise false
 */
/*************************************************************************************************/
bool HciVscReadMem(uint32_t start_addr, eMemAccess_type size,uint8_t length)
{
    hciRdMemCmd_t rdMemCmd =
    {
        .start_addr = start_addr,
        .type = size,
        .length = length,
    };

    if((length > MAX_MEM_ACCESS_SIZE)
        || ((size!=RD_8_Bit)&&(size!=RD_16_Bit)&&(size!=RD_32_Bit)))
    {
        return false;
    }

    HciVendorSpecificCmd(HCI_DBG_RD_MEM_CMD_OPCODE, 6, (uint8_t *)&rdMemCmd);

    return true;
}

/*************************************************************************************************/
/*!
 *  \brief  write memory variable
 *
 *  \param  start_addr   Start address to write
 *  \param  size         Access size
 *  \param  length       Length to write
 *  \param  data         Data to write
 *
 *  \return true when success, otherwise false
 */
/*************************************************************************************************/
bool HciVscWriteMem(uint32_t start_addr, eMemAccess_type size,uint8_t length, uint8_t *data)
{
    hciWrMemCmd_t wrMemCmd =
    {
        .start_addr = start_addr,
        .type = size,
        .length = length
    };

    if((length > MAX_MEM_ACCESS_SIZE)
        || ((size!=RD_8_Bit)&&(size!=RD_16_Bit)&&(size!=RD_32_Bit)))
    {
        return false;
    }

    memset(wrMemCmd.data, 0x0, MAX_MEM_ACCESS_SIZE);
    memcpy(wrMemCmd.data, data, length);

    HciVendorSpecificCmd(HCI_DBG_WR_MEM_CMD_OPCODE, 136, (uint8_t *)&wrMemCmd);

    return true;
}


/*************************************************************************************************/
/*!
 *  \brief  Get flash ID
 *
 *  \param  NULL
 *
 *  \return None
 */
/*************************************************************************************************/
void HciVscGetFlashId(void)
{
    HciVendorSpecificCmd(HCI_DBG_ID_FLASH_CMD_OPCODE, 0, NULL);
}


/*************************************************************************************************/
/*!
 *  \brief  Erase specifide flash space
 *
 *  \param  type     Flash type
 *  \param  offset   Start offset address
 *  \param  size     Size to erase
 *
 *  \return None
 */
/*************************************************************************************************/
void HciVscEraseFlash(uint8_t type, uint32_t offset,uint32_t size)
{
    hciErFlashCmd_t erFlashCmd =
    {
        .flashtype = type,
        .startoffset = offset,
        .size = size
    };

    HciVendorSpecificCmd(HCI_DBG_ER_FLASH_CMD_OPCODE, 9, (uint8_t *)&erFlashCmd);
}


/*************************************************************************************************/
/*!
 *  \brief  write flash
 *
 *  \param  type     Flash type
 *  \param  offset   Start offset address
 *  \param  length   Length to write
 *  \param  data     Data to write
 *
 *  \return true when success, otherwise false
 */
/*************************************************************************************************/
bool HciVscWriteFlash(uint8_t type, uint32_t offset,uint32_t length, uint8_t *data)
{
    hciWrFlashCmd_t wrFlashCmd =
    {
        .flashtype = type,
        .length = length,
        .startoffset = offset
    };

    if(data == NULL)
    {
        return false;
    }

    memset(wrFlashCmd.data, 0x0, MAX_FLASH_ACCESS_SIZE);
    memcpy(wrFlashCmd.data, data, length);

    HciVendorSpecificCmd(HCI_DBG_WR_FLASH_CMD_OPCODE, 140, (uint8_t *)&wrFlashCmd);

    return true;
}



/*************************************************************************************************/
/*!
 *  \brief  Read flash
 *
 *  \param  type     Flash type
 *  \param  offset   Start offset address
 *  \param  size     Size to read
 *
 *  \return true when success, otherwise false
 */
/*************************************************************************************************/
bool HciVscReadFlash(uint8_t type, uint32_t offset,uint32_t size)
{
    hciRdFlashCmd_t rdFlashCmd =
    {
        .flashtype = type,
        .startoffset = offset,
        .size = size
    };

    HciVendorSpecificCmd(HCI_DBG_RD_FLASH_CMD_OPCODE, 6, (uint8_t *)&rdFlashCmd);

    return true;
}

/*************************************************************************************************/
/*!
 *  \brief  Read Register value
 *
 *  \param  reg_addr  Register address to read
 *
 *  \return None
 */
/*************************************************************************************************/
void HciVscReadReg(uint32_t reg_addr)
{
    hciRegRdCmd_t rdRegCmd =
    {
        .addr = reg_addr
    };

    HciVendorSpecificCmd(HCI_DBG_REG_RD_CMD_OPCODE, 4, (uint8_t *)&rdRegCmd);
}

/*************************************************************************************************/
/*!
 *  \brief  Write Register value
 *
 *  \param  reg_addr   Register address to read
 *  \param  value      Value to write
 *
 *  \return None
 */
/*************************************************************************************************/
void HciVscWriteReg(uint32_t reg_addr, uint32_t value)
{
    hciRegWrCmd_t wrRegCmd =
    {
        .addr = reg_addr,
        .value = value
    };

    HciVendorSpecificCmd(HCI_DBG_REG_WR_CMD_OPCODE, 8, (uint8_t *)&wrRegCmd);
}

/*************************************************************************************************/
/*!
 *
 *  \brief  Vendor specific command for updating firmware
 *
 *  \param  update_fw    firmware type to update
 *
 *  \return None
 */
/*************************************************************************************************/
void HciVscUpdateFw(sbl_flag_e update_fw)
{
    HciVendorSpecificCmd(HCI_VSC_UPDATE_FW, 4, (uint8_t *)&update_fw);
}

/*************************************************************************************************/
/*!
 *  \brief  Get device ID
 *
 *  \param  NULL
 *
 *  \return None
 */
/*************************************************************************************************/
void HciVscGetDeviceId(void)
{
    HciVendorSpecificCmd(HCI_VSC_GET_DEVICE_ID, 0, NULL);
}

/*************************************************************************************************/
/*!
 *  \brief  platform reset
 *
 *  \param  reson   Reson to reset platform
 *
 *  \return None
 */
/*************************************************************************************************/
void HciVscPlfReset(ePlfResetReason_type reason)
{
    hciPlfResetCmd_t resetPlfCmd =
    {
        .reason = reason
    };

    HciVendorSpecificCmd(HCI_DBG_PLF_RESET_CMD_OPCODE, 1, (uint8_t *)&resetPlfCmd);
}

#ifdef ENABLE_BLE_CTRL_TRACE
/*************************************************************************************************/
/*!
 *  \brief  Set trace bitmap to enable which traces to output to host.
 *
 *  \param  bit_map    bit map for trace module
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciVscSetTraceBitMap(ble_trace_cfg bit_map)
{
    HciVendorSpecificCmd(HCI_VSC_SET_LOG_BITMAP, 4, (uint8_t *)&bit_map);
}
#endif

/*************************************************************************************************/
/*!
 *  \fn     HciVscSetCustom_BDAddr
 *
 *  \brief  This procedure is to set customer-provided Bluetooth address if needed.
 *
 *  \param  bd_addr  pointer to a bluetooth address customer allocates or NULL to use Apollo Device ID.
 *
 *  \return true when success
 */
/*************************************************************************************************/
bool_t HciVscSetCustom_BDAddr(uint8_t *bd_addr)
{
    uint8_t invalid_bd_addr[6] = {0};

    // When bd_addr is null, it will use Apollo's Device ID to form Bluetooth address.
    if ((bd_addr == NULL) || (memcmp(invalid_bd_addr, bd_addr, 6) == 0))
        return false;
    else {
        memcpy(g_BLEMacAddress, bd_addr, 6);
        return true;
    }
}

void HciVscUpdateBDAddress(void)
{
    HciVendorSpecificCmd(HCI_VSC_SET_BD_ADDR, 6, g_BLEMacAddress);
}


uint8_t nvds_data[NVDS_DATA_LEN]=
{
    NVDS_PARAMETER_MAGIC_NUMBER,
    NVDS_PARAMETER_SLEEP_ALGO_DUR,
    NVDS_PARAMETER_LPCLK_DRIFT
};

void HciVscUpdateNvdsParam(void)
{
    HciVendorSpecificCmd(HCI_VSC_UPDATE_NVDS, NVDS_DATA_LEN, nvds_data);
}

uint8_t ll_local_feats[LE_FEATS_LEN] = {0};

void HciVscUpdateLinklayerFeature(void)
{
    ll_local_feats[0] = (uint8_t)LL_FEATURES_BYTE0;
    ll_local_feats[1] = (uint8_t)(LL_FEATURES_BYTE1>>8);
    ll_local_feats[2] = (uint8_t)(LL_FEATURES_BYTE2>>16);
    ll_local_feats[3] = (uint8_t)(LL_FEATURES_BYTE3>>24);

    HciVendorSpecificCmd(HCI_VSC_UPDATE_LL_FEATURE, LE_FEATS_LEN, ll_local_feats);
}

