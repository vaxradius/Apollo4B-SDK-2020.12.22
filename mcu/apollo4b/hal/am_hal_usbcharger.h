//*****************************************************************************
//
//! @file am_hal_usbcharger.h
//!
//! @brief
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

#ifndef AM_HAL_USB_CHARGER_H_
#define AM_HAL_USB_CHARGER_H_

#include "am_mcu_apollo.h"

#ifdef __cplusplus
extern "C" {
#endif


//*****************************************************************************
//
//! @name USB charger physical connection status.
//! @brief USB charger data pin connection status.
//!
//! The Dm and Dp pins of the USB device have two connection status:
//! 1- Connected: When the Dm and Dp are connected (for SDP(Standard Downstream
//!    Port) and CDP(Charger Downstream Port) chargers only)
//! 2- Unknown: When the Dm and Dp are not connected to the USB cable or the
//!    charger type is DCP(Dedicated Charger Port).
//!
//! @{
//
//*****************************************************************************
typedef enum
{
    AM_HAL_USB_CHARGER_DATA_PIN_CONNECTED,
    AM_HAL_USB_CHARGER_DATA_PIN_CONNECTION_UNKNOWN,
}
am_hal_usb_charger_data_pin_status_e;

//*****************************************************************************
//
//! @name USB charger type
//! @brief Macro definitions for the USB charger type.
//!
//! These macros are used to indicate what kind of charger is connected to the USB port:
//!   1-SDP: Standard Downstream Port like a PC/laptop.
//!   2-CDP: Charging Downstream Port is a downstream port on a device that complies with the
//!          USB 2.0 definition of a host or a hub.
//!   3-DCP: A Dedicated Charging Port (DCP) is a downstream port on a device that outputs power
//!          through a USB connector, but is not capable of enumerating a downstream device.
//!
//! @{
//
//*****************************************************************************
typedef enum
{
    AM_HAL_USB_CHARGER_SDP,
    AM_HAL_USB_CHARGER_CDP,
    AM_HAL_USB_CHARGER_DCP,
    AM_HAL_USB_CHARGER_NO_CHARGER
} am_hal_usb_charger_type_e;


//*****************************************************************************
//
//! @name Vendor-specific comparator ref voltage
//! @brief The vendor-specific comparator ref voltage for DP/DM
//!
//! These macros are used to program the voltage comparator for DP/DM USB pins:
//! The reference voltage for the comparator for each pin can be 1.25V, 1.65V, 2.35V, and 3.10V.
//! The output of the comparators is connected to BCDETSTATUS.DMCOMPOUT and BCDETSTATUS.DPCOMPOUT.
//! @{
//
//*****************************************************************************
typedef enum
{
    AM_HAL_USB_CHARGER_SET_REF_VOL_TO_1P65  = 0x0,  // set comparator voltage to 1.65V
    AM_HAL_USB_CHARGER_SET_REF_VOL_TO_3P10V = 0x1,  // set comparator voltage to 3.10V
    AM_HAL_USB_CHARGER_SET_REF_VOL_TO_2P35  = 0x2,  // set comparator voltage to 2.35V
    AM_HAL_USB_CHARGER_SET_REF_VOL_TO_1P25V = 0x3   // set comparator voltage to 1.25V
} am_hal_usb_charger_comparator_reference_voltage_e;



//*****************************************************************************
//
//! @brief Bring the USB hardware out of reset
//!
//! @param - none.
//!
//! @return one of am_hal_status_e like AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
static inline void am_hal_usb_hardware_unreset(void)
{
    //
    // Unreset the USB controller and PHY. To make sure the USB can do enumuration, all the USB charger detection
    // circuit should be disconnected from the Dp/Dm pins.
    //
    USB->BCDETCRTL1 = (uint32_t)( ((uint32_t)1 << USB_BCDETCRTL1_USBSWRESET_Pos)    |
        (0 << USB_BCDETCRTL1_USBDCOMPEN_Pos)        |
        (0 << USB_BCDETCRTL1_USBDCOMPREF_Pos)       |
        (0 << USB_BCDETCRTL1_IDPSINKEN_Pos)         |
        (0 << USB_BCDETCRTL1_VDMSRCEN_Pos)          |
        (0 << USB_BCDETCRTL1_RDMPDWNEN_Pos)         |
        (0 << USB_BCDETCRTL1_VDPSRCEN_Pos)          |
        (0 << USB_BCDETCRTL1_IDPSRCEN_Pos)          |
        (0 << USB_BCDETCRTL1_IDMSINKEN_Pos)         |
        (0 << USB_BCDETCRTL1_BCWEAKPULLDOWNEN_Pos)  |
        (0 << USB_BCDETCRTL1_BCWEAKPULLUPEN_Pos) );
}

//*****************************************************************
//
// USB reset functions
//
//*****************************************************************

static inline void am_hal_usb_hardware_reset(void)
{
    //
    // Holds a USB controller and PHY in the reset for BC detection
    // After MCU hardware reset, the USB is always in reset.
    //
    USB->BCDETCRTL1_b.USBSWRESET = 0;
}

//*****************************************************************************
//
//! @brief enable Data Contact Detection(DCD) based on BC1.2 protocol
//!
//! @param - none.
//!
//! @return one of am_hal_status_e like AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************

static inline void  am_hal_usb_charger_enable_data_pin_contact_detection(void)
{
    //
    // Clear all other bits
    // Holds a USB controller and PHY in the reset for BC detection
    // enables DP current source as per USB BC 1.2 spec.
    // enables DM pull-down as per USB BC 1.2 spec.
    //
    USB->BCDETCRTL1 = (USB_BCDETCRTL1_USBSWRESET_Pos << 0) | USB_BCDETCRTL1_IDPSRCEN_Msk | USB_BCDETCRTL1_RDMPDWNEN_Msk;
}

//*****************************************************************************
//
//! @brief Check the connection status of the data pins(Dp/Dm)
//!
//! @param - none.
//!
//! @return one of am_hal_usb_charger_data_pin_status_e like AM_HAL_USB_CHARGER_DATA_PIN_CONNECTED
//
//*****************************************************************************
static inline am_hal_usb_charger_data_pin_status_e am_hal_usb_charger_data_pin_connection_status()
{
    return ( USB->BCDETSTATUS_b.DPATTACHED )? AM_HAL_USB_CHARGER_DATA_PIN_CONNECTED :AM_HAL_USB_CHARGER_DATA_PIN_CONNECTION_UNKNOWN;
}

//*****************************************************************************
//
//! @brief Enable SDP connection detection hardware
//!
//! @param - none.
//!
//! @return one of am_hal_status_e like AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
static inline void am_hal_usb_charger_enable_primary_detection(void)
{
    //
    // Clear all other bits
    // Holds a USB controller and PHY in the reset for BC detection
    // enables DM current sink as per USB BC 1.2 spec.
    // enables DP voltage source as per USB BC 1.2 spec.
    //
    USB->BCDETCRTL1 = (USB_BCDETCRTL1_USBSWRESET_Pos << 0) | USB_BCDETCRTL1_IDMSINKEN_Msk | USB_BCDETCRTL1_VDPSRCEN_Msk;
}


//*****************************************************************************
//
//! @name SDP USB charger detection
//! @brief detect SDP charger hardware connection
//! This function must be called in the primary phase of BC1.2 protocol, else the
//! return value is invalid.
//! Returns AM_HAL_USB_CHARGER_SDP_DETECTED when the SDP charger(such as a PC) is
//! detected.
//! Returns AM_HAL_USB_CHARGER_NO_CHARGER_DETECTED when the charger type is CDP or DCP.
//!
//! @param - none.
//!
//! @return one of am_hal_usb_charger_detection_status_e like AM_HAL_USB_CHARGER_SDP_DETECTED
//
//*****************************************************************************
static inline am_hal_usb_charger_type_e  am_hal_usb_charger_sdp_connection_status(void)
{
    //
    // During Primary Detection the PD shall turn on VDP_SRC and IDM_SINK. When a voltage of VDP_SRC is
    // applied to Dp, an SDP will continue pulling Dm low through RDM_DWN.
    // A PD shall compare the voltage on Dm with VDAT_REF. If Dm is less than VDAT_REF, then the PD is
    // allowed determining that it is attached to an SDP. Because the Dm is less than VDAT_REF, the
    // CPDETECTED would be zero.
    //
    return (USB->BCDETSTATUS_b.CPDETECTED == 0) ? AM_HAL_USB_CHARGER_SDP: AM_HAL_USB_CHARGER_NO_CHARGER;
}

//*****************************************************************************
//
//! @name detect CDP or DCP USB charger
//! @brief Enable CDP or DCP connection detection hardware
//!
//! Returns AM_HAL_USB_CHARGER_SDP_DETECTED when the SDP charger(such as a PC) is
//! detected.
//! Returns AM_HAL_USB_CHARGER_NO_CHARGER_DETECTED when the charger type is CDP or DCP.
//!
//! @param - none.
//!
//! @return one of am_hal_usb_charger_detection_status_e like AM_HAL_USB_CHARGER_SDP_DETECTED
//
//*****************************************************************************
static inline void am_hal_usb_charger_enable_secondary_detection(void)
{
    //
    // Secondary Detection can be used to distinguish between a DCP and a CDP. PDs that are not ready
    // to be enumerated within TSVLD_CON_PWD of detecting VBUS are required to implement Secondary
    // Detection. PDs that are ready to be enumerated are allowed to bypass Secondary Detection.
    // During Secondary Detection, a PD shall output VDM_SRC(0.5V) on Dm, turn on IDP_SINK, and compare the
    // voltage on Dp to VDAT_REF(0.25V). Since a DCP is required to short Dp to Dm through a resistance
    // of RDCP_DAT(200 ohm), the voltage on Dp will be close to VDM_SRC, which is above VDAT_REF.
    // If a PD detects that Dp is greater than VDAT_REF, it knows that it is attached to a DCP.
    //
    USB->BCDETCRTL1 = (USB_BCDETCRTL1_USBSWRESET_Pos << 0) | USB_BCDETCRTL1_VDMSRCEN_Msk | USB_BCDETCRTL1_IDPSINKEN_Msk;
}


//*****************************************************************************
//
//! @name enable CDP charger
//! @brief Enable the downstream device to draw current based on CDP charger
//!
//! This function should be called at the end of BC1.2 charger detection algorithm after
//! detecting a CDP charger.
//!
//! @param - none.
//!
//! @return one of am_hal_status_e like AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
static inline void am_hal_usb_charger_enable_cdp_charger(void)
{
    //
    // If a PD detects that Dp is less than VDAT_REF, it knows that it is attached to a CDP.
    // It is then required to turn off VDP_SRC and VDM_SRC, as shown in the Good Battery
    // Algorithm in Section 3.3.2, and is allowed to draw IDEV_CHG.
    // Note: Here all bits in BCDETCRTL1 are cleared to zero.
    //
    USB->BCDETCRTL1 = (USB_BCDETCRTL1_USBSWRESET_Pos << 0) | (USB_BCDETCRTL1_VDMSRCEN_Pos << 0) | (USB_BCDETCRTL1_VDPSRCEN_Pos << 0);
}


//*****************************************************************************
//
//! @name enable DCP charger
//! @brief Enable the downstream device to draw current based on DCP charger
//!
//! This function should be called at the end of BC1.2 charger detection algorithm after
//! detecting a DCP charger.
//!
//! @param - none.
//!
//! @return one of am_hal_status_e like AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
static inline void  am_hal_usb_charger_enable_dcp_charger(void)
{
    //
    // If a PD detects that Dp is greater than VDAT_REF, it knows that it is attached to a DCP.
    // It is then required to enable VDP_SRC or pull Dp to VDP_UP through RDP_UP, as defined in
    // the Good Battery Algorithm in Section 3.3.2.
    //
    USB->BCDETCRTL1 = (USB_BCDETCRTL1_USBSWRESET_Pos << 0) | (USB_BCDETCRTL1_VDPSRCEN_Pos << 1);
}


//*****************************************************************************
//
//! @name return CDP or DCP type
//! @brief detect CDP or DCP USB charger
//!
//! This function must be call in the secondary phase of BC1.2 protocol, else the
//! return value in invalid.
//! Returns AM_HAL_USB_CHARGER_DCP_DETECTED when the DCP charger is detected.
//! Returns AM_HAL_USB_CHARGER_CDP_DETECTED when the CDP charger is detected.
//!
//! @param - none.
//!
//! @return one of am_hal_usb_charger_detection_status_e like AM_HAL_USB_CHARGER_DCP_DETECTED
//
//*****************************************************************************
static inline am_hal_usb_charger_type_e  am_hal_usb_charger_cdp_or_dcp_connection_status(void)
{
    return ( USB->BCDETSTATUS_b.DCPDETECTED ) ? AM_HAL_USB_CHARGER_DCP : AM_HAL_USB_CHARGER_CDP;
}

//*****************************************************************************
//
//! @name Enable vendor-specific comparator ref voltage
//! @brief Enable vendor-specific comparator ref voltage
//!
//! @param am_hal_usb_charger_comparator_reference_voltage_e - voltage reference value
//!
//! @return one of am_hal_status_e like AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
static inline void am_hal_usb_charger_configure_vendore_comparators(am_hal_usb_charger_comparator_reference_voltage_e e)
{
    //
    // Make sure the USB logic held in reset
    //
    USB->BCDETCRTL1_b.USBSWRESET = 0;

    //
    // Enable the DM/DP comparators
    //
    USB->BCDETCRTL1_b.USBDCOMPEN = 1;
    USB->BCDETCRTL1_b.USBDCOMPREF = (unsigned) e;
}


//*****************************************************************************
//
//! @name Disable vendor-specific voltage comparator
//! @brief Disable vendore-specific voltage comparators for DP and DM pins
//!
//! @param - none.
//!
//! @return one of am_hal_status_e like AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
static inline void am_hal_usb_charger_disable_vendore_comparators(void)
{
    //
    // Disable the DM/DP comparators
    //
    USB->BCDETCRTL1_b.USBDCOMPEN = 0;
}


//*****************************************************************************
//
//! @name Read vendor-specific voltage comparator output
//! @brief Read vendore-specific voltage comparator for DP pins
//!
//! @param - none.
//!
//! @return the Dp comparator output value
//
//*****************************************************************************
static inline uint32_t am_hal_usb_charger_read_dp_comparator_output(void)
{
    return USB->BCDETSTATUS_b.DPCOMPOUT;
}

//*****************************************************************************
//
//! @name Read vendor-specific voltage comparator output
//! @brief Read vendore-specific voltage comparator for DM pins
//!
//! @param - none.
//!
//! @return the Dp comparator output value
//
//*****************************************************************************
static inline uint32_t am_hal_usb_charger_read_dm_comparator_output(void)
{
    return USB->BCDETSTATUS_b.DMCOMPOUT;
}

//*****************************************************************************
//
//! @name Enable the USB PHY reset override
//! @brief Enable the USB PHY reset override for charger detection
//!
//! @param - none.
//!
//! @return - none
//
//*****************************************************************************
static inline void am_hal_usb_enable_phy_reset_override(void)
{
    MCUCTRL->USBPHYRESET &= ~(MCUCTRL_USBPHYRESET_USBPHYPORRSTDIS_Msk | MCUCTRL_USBPHYRESET_USBPHYUTMIRSTDIS_Msk);
}

//*****************************************************************************
//
//! @name Disable the USB PHY reset override
//! @brief Disable the USB PHY reset override for USB enumeration
//!
//! @param - none.
//!
//! @return - none
//
//*****************************************************************************
static inline void am_hal_usb_disable_phy_reset_override(void)
{
    MCUCTRL->USBPHYRESET |= (MCUCTRL_USBPHYRESET_USBPHYPORRSTDIS_Msk | MCUCTRL_USBPHYRESET_USBPHYUTMIRSTDIS_Msk);
}


#ifdef __cplusplus
}
#endif

#endif /* AM_HAL_USB_CHARGER_H_ */
