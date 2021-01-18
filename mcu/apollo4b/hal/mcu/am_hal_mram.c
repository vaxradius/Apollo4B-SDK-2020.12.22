//*****************************************************************************
//
//  am_hal_mram.c
//! @file
//!
//! @brief BootROM Helper Function Table
//!
//! @addtogroup MRAM4 MRAM
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
#include "am_hal_bootrom_helper.h"
#include "regs/am_mcu_apollo4b_info0.h"
#include "regs/am_mcu_apollo4b_info1.h"


static uint32_t g_program_mram_tmc_tcycrd[] =
{
    0x0104F244, 0x0101F2C4, 0x604A22C3, 0x6CCB2205,
    0x0310F043, 0x650A64CB, 0x60CB2301, 0x610A2200,
    0x618A614A, 0x000FF000, 0xF043680B, 0x600B0308,
    0xF022680A, 0x600A0208, 0x6D0B2209, 0x2395F362,
    0x2206650B, 0xF3626D0B, 0x650B0304, 0xF042680A,
    0x600A0208, 0x680B6DCA, 0x0308F023, 0x600B0912,
    0x1002EA40, 0x200560C8, 0xF3606D0A, 0x650A0204,
    0xF0406808, 0x60080008, 0xF022680A, 0x600A0208,
    0x6D082200, 0x2095F36F, 0x60CA6508, 0x6D0A2005,
    0x0204F360, 0x6808650A, 0x0008F040, 0x680A6008,
    0x0208F022, 0x2200600A, 0xF0206CC8, 0x64C80010,
    0x4770604A
};

//*****************************************************************************
//
// Macros
//
//*****************************************************************************
#define MRAM_OVERRIDE()     program_mram_tmc_tcycrd(1)
#define MRAM_REVERT()       program_mram_tmc_tcycrd(0)

typedef void (*program_mram_tmc_tcycrd_t)(uint32_t N);

program_mram_tmc_tcycrd_t program_mram_tmc_tcycrd = (program_mram_tmc_tcycrd_t)((uint8_t *)g_program_mram_tmc_tcycrd + 1);


//*****************************************************************************
//
//! @brief This programs up to N words of the Main MRAM
//!
//! @param ui32ProgramKey - The programming key, AM_HAL_MRAM_PROGRAM_KEY.
//! @param pui32Src - Pointer to word aligned array of data to program into
//! the MRAM.
//! @param pui32Dst - Pointer to the words (4 byte) aligned MRAM location where
//! programming of the MRAM is to begin.
//! @param ui32NumWords - The number of words to be programmed.
//!
//! This function will program multiple words (4 byte) tuples in main MRAM.
//!
//! @note This function is provided only for convenience. It is most efficient
//! to operate on MRAM in be 4 words (16 byte) aligned multiples. Doing word
//! access will be very inefficient and should be avoided.
//!
//! @return 0 for success, non-zero for failure.
//!     Failing return code indicates:
//!    -1   invalid alignment for pui32Dst or ui32NumWords not 16 bye multiple
//!     1   ui32ProgramKey is invalid.
//!     2   pui32Dst is invalid.
//!     3   Flash addressing range would be exceeded.  That is, (pui32Dst +
//!         (ui32NumWords * 4)) is greater than the last valid address.
//!     4   pui32Src is invalid.
//!     5   pui32Src is invalid.
//!     6   Flash controller hardware timeout.
//
//*****************************************************************************
int
am_hal_mram_main_words_program(uint32_t ui32ProgramKey, uint32_t *pui32Src,
                               uint32_t *pui32Dst, uint32_t ui32NumWords)
{
    //
    // Use the new helper function to efficiently program the data in MRAM.
    //

    if ( (uint32_t)pui32Dst <= AM_HAL_MRAM_LARGEST_VALID_ADDR )
    {
        //
        // This helper function requires a word offset rather than an actual
        // address. Since MRAM addresses start at 0x0, we can convert the addr
        // into a word offset by simply dividing the destination address by 4.
        //
        pui32Dst = (uint32_t*)((uint32_t)pui32Dst / 4);
    }

    return g_am_hal_bootrom_helper.nv_program_main2(ui32ProgramKey, 1,
                                                    (uint32_t)pui32Src,
                                                    (uint32_t)pui32Dst,
                                                    ui32NumWords);
}

//*****************************************************************************
//
//! @brief This programs up to N words of the Main MRAM
//!
//! @param ui32ProgramKey - The programming key, AM_HAL_MRAM_PROGRAM_KEY.
//! @param pui32Src - Pointer to word aligned array of data to program into
//! the MRAM.
//! @param pui32Dst - Pointer to the 4 words (16 byte) aligned MRAM location where
//! programming of the MRAM is to begin.
//! @param ui32NumWords - The number of words to be programmed. This MUST be
//! a multiple of 4 (16 byte multiple)
//!
//! This function will program multiple 4 words (16 byte) tuples in main MRAM.
//!
//! @note THIS FUNCTION ONLY OPERATES ON 16 BYTE BLOCKS OF MAIN MRAM. The pDst
//! MUST be 4 words (16 byte) aligned, and ui32NumWords MUST be multiple of 4.
//!
//! @return 0 for success, non-zero for failure.
//!     Failing return code indicates:
//!    -1   invalid alignment for pui32Dst or ui32NumWords not 16 byte multiple
//!     1   ui32ProgramKey is invalid.
//!     2   pui32Dst is invalid.
//!     3   Flash addressing range would be exceeded.  That is, (pui32Dst +
//!         (ui32NumWords * 4)) is greater than the last valid address.
//!     4   pui32Src is invalid.
//!     5   pui32Src is invalid.
//!     6   Flash controller hardware timeout.
//
//*****************************************************************************
int
am_hal_mram_main_program(uint32_t ui32ProgramKey, uint32_t *pui32Src,
                         uint32_t *pui32Dst, uint32_t ui32NumWords)
{
    //
    // Check for pui32Dst & ui32NumWords
    //
    if (((uint32_t)pui32Dst & 0xf) || (ui32NumWords & 0x3))
    {
        return -1;
    }

    return am_hal_mram_main_words_program(ui32ProgramKey, pui32Src,
                                          pui32Dst, ui32NumWords);
}

//*****************************************************************************
//
//! @brief This Fills up to N words of the Main MRAM
//!
//! @param ui32ProgramKey - The programming key, AM_HAL_MRAM_PROGRAM_KEY.
//! @param ui32Value - 32-bit data value to fill into the MRAM
//! @param pui32Dst - Pointer to the 4 words (16 byte) aligned MRAM location where
//! programming of the MRAM is to begin.
//! @param ui32NumWords - The number of words to be programmed. This MUST be
//! a multiple of 4 (16 byte multiple)
//!
//! This function will fill multiple 16 byte tuples in main MRAM with specified pattern.
//!
//! @note THIS FUNCTION ONLY OPERATES ON 16 BYTE BLOCKS OF MAIN MRAM. The pDst
//! MUST be 4 words (16 byte) aligned, and ui32NumWords MUST be multiple of 4.
//!
//! @return 0 for success, non-zero for failure.
//!    -1   invalid alignment for pui32Dst or ui32NumWords not 16 byte multiple
//!     1   ui32InfoKey is invalid.
//!     2   ui32Offset is invalid.
//!     3   addressing range would be exceeded.  That is, (ui32Offset +
//!         (ui32NumWords * 4)) is greater than the last valid address.
//!     4   pui32Src is invalid.
//!     5   pui32Src is invalid.
//!     6   Flash controller hardware timeout.
//
//*****************************************************************************
int
am_hal_mram_main_fill(uint32_t ui32ProgramKey, uint32_t ui32Value,
                      uint32_t *pui32Dst, uint32_t ui32NumWords)
{
    //
    // Check for pui32Dst & ui32NumWords
    //
    if (((uint32_t)pui32Dst & 0xf) || (ui32NumWords & 0x3))
    {
        return -1;
    }

    //
    // Use the new helper function to efficiently fill MRAM with a value.
    //

    if ( (uint32_t)pui32Dst <= AM_HAL_MRAM_LARGEST_VALID_ADDR )
    {
        //
        // This helper function requires a word offset rather than an actual
        // address. Since MRAM addresses start at 0x0, we can convert the addr
        // into a word offset by simply dividing the destination address by 4.
        //
        pui32Dst = (uint32_t*)((uint32_t)pui32Dst / 4);
    }

    return g_am_hal_bootrom_helper.nv_program_main2(ui32ProgramKey, 0,
                                                    (uint32_t)ui32Value,
                                                    (uint32_t)pui32Dst,
                                                    ui32NumWords);
}

//*****************************************************************************
//
//! @brief This programs up to N words of the Main array on one MRAM.
//!
//! @param ui32InfoKey - The programming key, AM_HAL_MRAM_INFO_KEY.
//! @param pui32Src - Pointer to word aligned array of data to program into
//! INFO0
//! @param ui32Offset - Pointer to the word aligned INFO0 offset where
//! programming of the INFO0 is to begin.
//! @param ui32NumWords - The number of words to be programmed.
//!
//! This function will program multiple words in INFO0
//!
//! @return 0 for success, non-zero for failure.                                                6
//!     1   ui32InfoKey is invalid.
//!     2   ui32Offset is invalid.
//!     3   addressing range would be exceeded.  That is, (ui32Offset +
//!         (ui32NumWords * 4)) is greater than the last valid address.
//!     4   pui32Src is invalid.
//!     5   pui32Src is invalid.
//!     6   Flash controller hardware timeout.
//
//*****************************************************************************
int
am_hal_mram_info_program(uint32_t ui32InfoKey, uint32_t *pui32Src,
                         uint32_t ui32Offset, uint32_t ui32NumWords)
{
    int retval;

    MRAM_OVERRIDE();

    retval = g_am_hal_bootrom_helper.nv_program_info_area(ui32InfoKey,
                                pui32Src, ui32Offset, ui32NumWords);

    MRAM_REVERT();

    return retval;
}

//*****************************************************************************
//
//! @brief This function erases main MRAM + customer INFO space
//!
//! @param ui32BrickKey - The Brick key.
//!
//! This function erases main MRAM and customer INFOinstance
//! even if the customer INFO space is programmed to not be erasable. This
//! function completely erases the MRAM main and info instance and wipes the
//! SRAM. Upon completion of the erasure operations, it does a POI (power on
//! initialization) reset.
//!
//! @note This function enforces 128 bit customer key lock. The caller needs to assert
//! the Recovery Lock using am_hal_security_set_key() providing appropriate key.
//! Otherwise, the function will fail.  Therefore, always check for a return code.
//! If the function returns, a failure has occured.
//!
//! @return Does not return if successful.  Returns failure code otherwise.
//!     Failing return code indicates:
//!     0x00000001  ui32BrickKey is invalid.
//!     0x00000002  Recovery key lock not set.
//!     Other values indicate Internal error.
//
//*****************************************************************************
int
am_hal_mram_recovery(uint32_t ui32BrickKey)
{
    return g_am_hal_bootrom_helper.nv_recovery(ui32BrickKey);
}


//*****************************************************************************
//
//! @brief Read INFO data.
//!
//! This function implements a workaround required for Apollo4 B0 parts in
//! order to accurately read INFO space.
//!
//! @param ui32InfoSpace - 0 = Read INFO0.
//!                        1 = Only valid to read the customer visible area
//!                            of INFO1. If INFO1, the ui32WordOffset argument
//!                            must be 0x480 or greater (byte offset 0x1200).
//! @param ui32WordOffset - Desired word offset into INFO space.
//! @param ui32NumWords  - The number of words to be retrieved.
//! @param pui32Dst      - Pointer to the location where the INFO data
//!                        is to be copied to.
//!
//! @return 0 for success, non-zero for failure.                                                6
//
//*****************************************************************************
int
am_hal_mram_info_read(uint32_t ui32InfoSpace,
                      uint32_t ui32WordOffset,
                      uint32_t ui32NumWords,
                      uint32_t *pui32Dst)
{
    int retval = 0;
    uint32_t ux;
    uint32_t *pui32Info;

    if ( ui32InfoSpace == 0 )
    {
        if ( (ui32WordOffset >= (AM_HAL_INFO0_SIZE_BYTES / 4)) || ((ui32WordOffset + ui32NumWords) > (AM_HAL_INFO0_SIZE_BYTES / 4))  )
        {
            return 2;
        }
        pui32Info = (uint32_t*)(AM_REG_INFO0_BASEADDR + (ui32WordOffset * 4));
    }
    else if ( ui32InfoSpace == 1 )
    {
        if ( (ui32WordOffset < (AM_HAL_INFO1_VISIBLE_OFFSET / 4)) || (ui32WordOffset >= (AM_HAL_INFO1_SIZE_BYTES / 4)) || ((ui32WordOffset + ui32NumWords) > (AM_HAL_INFO1_SIZE_BYTES / 4))  )
        {
            return 2;
        }
        pui32Info = (uint32_t*)(AM_REG_INFO1_BASEADDR + (ui32WordOffset * 4));
    }
    else
    {
        return 1;
    }

    MRAM_OVERRIDE();

    for (ux = 0; ux < ui32NumWords; ux++ )
    {
        *pui32Dst++ = *pui32Info++;
    }

    MRAM_REVERT();

    return retval;
}
