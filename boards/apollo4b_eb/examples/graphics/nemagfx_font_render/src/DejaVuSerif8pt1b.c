//*****************************************************************************
//
//! @file DejaVuSerif8pt1b.c
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

#ifndef DEJAVUSERIF8PT1B_C
#define DEJAVUSERIF8PT1B_C

#include "DejaVuSerif8pt1b.h"

#ifndef NEMA_GPU_MEM
#define NEMA_GPU_MEM
#endif // NEMA_GPU_MEM

/* This will be read by the GPU only */
#if defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment = 16
const unsigned char DejaVuSerif8pt1bBitmaps[] =
#else
const unsigned char DejaVuSerif8pt1bBitmaps[]  __attribute__ ((aligned (16))) =
#endif
{
/* static uint8_t NEMA_GPU_MEM DejaVuSerif8pt1bBitmaps[] = { */
  // 0x20 - 0x7e
  0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0x00, 0xC0, 0xC0,
  0x90, 0x90, 0x90, 0x90, 0x0C, 0x80, 0x08, 0x80, 0x09, 0x80, 0x7F, 0xE0,
  0x11, 0x00, 0x13, 0x00, 0x13, 0x00, 0xFF, 0xC0, 0x32, 0x00, 0x26, 0x00,
  0x26, 0x00, 0x10, 0x10, 0x7E, 0x93, 0x91, 0xD0, 0x78, 0x1E, 0x13, 0x91,
  0xD3, 0x7C, 0x10, 0x10, 0x70, 0x40, 0x88, 0x40, 0x88, 0x80, 0x89, 0x80,
  0x89, 0x00, 0x72, 0x00, 0x02, 0x70, 0x04, 0x88, 0x0C, 0x88, 0x08, 0x88,
  0x10, 0x88, 0x10, 0x70, 0x1E, 0x00, 0x23, 0x00, 0x21, 0x00, 0x20, 0x00,
  0x30, 0x00, 0x48, 0x70, 0x84, 0x20, 0x82, 0x20, 0x81, 0x40, 0x80, 0x80,
  0x61, 0xC0, 0x3E, 0x38, 0x80, 0x80, 0x80, 0x80, 0x20, 0x40, 0x40, 0x80,
  0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x40, 0x40, 0x20, 0x80,
  0x40, 0x40, 0x40, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x40, 0x40,
  0x40, 0x80, 0x10, 0x92, 0x7C, 0x10, 0x7C, 0x92, 0x10, 0x04, 0x00, 0x04,
  0x00, 0x04, 0x00, 0x04, 0x00, 0x04, 0x00, 0xFF, 0xE0, 0x04, 0x00, 0x04,
  0x00, 0x04, 0x00, 0x04, 0x00, 0x04, 0x00, 0x40, 0x40, 0x40, 0x80, 0xF0,
  0xC0, 0xC0, 0x08, 0x18, 0x10, 0x10, 0x30, 0x20, 0x20, 0x20, 0x60, 0x40,
  0x40, 0xC0, 0x80, 0x3C, 0x42, 0x42, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81,
  0x42, 0x42, 0x3C, 0x20, 0x60, 0xA0, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,
  0x20, 0x20, 0xF8, 0x7C, 0xC2, 0x81, 0x01, 0x01, 0x01, 0x02, 0x04, 0x08,
  0x30, 0x61, 0xFF, 0x7C, 0xC3, 0x81, 0x01, 0x02, 0x1C, 0x02, 0x01, 0x01,
  0x81, 0xC2, 0x7C, 0x06, 0x00, 0x0E, 0x00, 0x0A, 0x00, 0x12, 0x00, 0x32,
  0x00, 0x22, 0x00, 0x42, 0x00, 0xC2, 0x00, 0xFF, 0x80, 0x02, 0x00, 0x02,
  0x00, 0x0F, 0x80, 0x7E, 0x40, 0x40, 0x40, 0x7C, 0x42, 0x01, 0x01, 0x01,
  0x81, 0xC2, 0x7C, 0x1E, 0x63, 0x41, 0x80, 0xBC, 0xC2, 0x81, 0x81, 0x81,
  0x81, 0x42, 0x3C, 0xFF, 0x81, 0x82, 0x02, 0x04, 0x04, 0x04, 0x08, 0x08,
  0x10, 0x10, 0x20, 0x3C, 0xC3, 0x81, 0x81, 0xC3, 0x3C, 0x42, 0x81, 0x81,
  0x81, 0x42, 0x3C, 0x3C, 0x42, 0x81, 0x81, 0x81, 0x81, 0x43, 0x3D, 0x01,
  0x82, 0xC6, 0x78, 0xC0, 0xC0, 0x00, 0x00, 0x00, 0xC0, 0xC0, 0xC0, 0xC0,
  0x00, 0x00, 0x00, 0x40, 0x40, 0xC0, 0x80, 0x00, 0x40, 0x03, 0xC0, 0x1E,
  0x00, 0x78, 0x00, 0xC0, 0x00, 0x70, 0x00, 0x1E, 0x00, 0x03, 0xC0, 0x00,
  0x40, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xC0, 0x80, 0x00, 0xF0,
  0x00, 0x1E, 0x00, 0x07, 0x80, 0x00, 0xC0, 0x03, 0x80, 0x1E, 0x00, 0xF0,
  0x00, 0x80, 0x00, 0x78, 0xC6, 0x82, 0x02, 0x06, 0x0C, 0x38, 0x20, 0x20,
  0x00, 0x60, 0x60, 0x07, 0xC0, 0x18, 0x30, 0x20, 0x18, 0x40, 0x0C, 0x8F,
  0x44, 0x98, 0xC4, 0x90, 0x44, 0x90, 0x4C, 0x98, 0xD8, 0x8F, 0x70, 0x40,
  0x00, 0x20, 0x00, 0x18, 0x20, 0x0F, 0xC0, 0x06, 0x00, 0x06, 0x00, 0x0B,
  0x00, 0x09, 0x00, 0x19, 0x00, 0x10, 0x80, 0x10, 0x80, 0x20, 0x80, 0x3F,
  0xC0, 0x20, 0x40, 0x40, 0x20, 0xE0, 0x70, 0xFE, 0x00, 0x43, 0x00, 0x41,
  0x00, 0x41, 0x00, 0x43, 0x00, 0x7E, 0x00, 0x41, 0x00, 0x40, 0x80, 0x40,
  0x80, 0x40, 0x80, 0x41, 0x80, 0xFE, 0x00, 0x1F, 0x80, 0x20, 0xC0, 0x40,
  0x40, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80,
  0x00, 0x40, 0x40, 0x20, 0x80, 0x1F, 0x00, 0xFE, 0x00, 0x41, 0x80, 0x40,
  0x40, 0x40, 0x60, 0x40, 0x20, 0x40, 0x20, 0x40, 0x20, 0x40, 0x20, 0x40,
  0x60, 0x40, 0x40, 0x41, 0x80, 0xFE, 0x00, 0xFF, 0x80, 0x40, 0x80, 0x40,
  0x80, 0x40, 0x00, 0x42, 0x00, 0x7E, 0x00, 0x42, 0x00, 0x40, 0x00, 0x40,
  0x00, 0x40, 0x80, 0x40, 0x80, 0xFF, 0x80, 0xFF, 0x80, 0x40, 0x80, 0x40,
  0x80, 0x40, 0x00, 0x42, 0x00, 0x7E, 0x00, 0x42, 0x00, 0x40, 0x00, 0x40,
  0x00, 0x40, 0x00, 0x40, 0x00, 0xF0, 0x00, 0x1F, 0xC0, 0x20, 0x60, 0x40,
  0x20, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0xE0, 0x80, 0x20, 0x80,
  0x20, 0x40, 0x20, 0x20, 0x60, 0x1F, 0x80, 0xE0, 0xE0, 0x40, 0x40, 0x40,
  0x40, 0x40, 0x40, 0x40, 0x40, 0x7F, 0xC0, 0x40, 0x40, 0x40, 0x40, 0x40,
  0x40, 0x40, 0x40, 0x40, 0x40, 0xE0, 0xE0, 0xE0, 0x40, 0x40, 0x40, 0x40,
  0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0xE0, 0x1E, 0x04, 0x04, 0x04, 0x04,
  0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x84, 0x88, 0x78, 0xE7, 0x80,
  0x42, 0x00, 0x44, 0x00, 0x48, 0x00, 0x50, 0x00, 0x60, 0x00, 0x50, 0x00,
  0x48, 0x00, 0x44, 0x00, 0x42, 0x00, 0x41, 0x00, 0xE0, 0xC0, 0xE0, 0x00,
  0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00,
  0x40, 0x00, 0x40, 0x00, 0x40, 0x80, 0x40, 0x80, 0xFF, 0x80, 0xE0, 0x1C,
  0x60, 0x18, 0x50, 0x28, 0x50, 0x28, 0x48, 0x48, 0x48, 0x48, 0x44, 0x88,
  0x44, 0x88, 0x43, 0x08, 0x43, 0x08, 0x40, 0x08, 0xE0, 0x1C, 0xE0, 0xE0,
  0x70, 0x40, 0x50, 0x40, 0x48, 0x40, 0x48, 0x40, 0x44, 0x40, 0x42, 0x40,
  0x42, 0x40, 0x41, 0x40, 0x40, 0xC0, 0x40, 0xC0, 0xE0, 0x40, 0x1F, 0x00,
  0x20, 0x80, 0x40, 0x40, 0x80, 0x20, 0x80, 0x20, 0x80, 0x20, 0x80, 0x20,
  0x80, 0x20, 0x80, 0x20, 0x40, 0x40, 0x20, 0x80, 0x1F, 0x00, 0xFE, 0x00,
  0x41, 0x00, 0x40, 0x80, 0x40, 0x80, 0x40, 0x80, 0x41, 0x00, 0x7E, 0x00,
  0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0xE0, 0x00, 0x1F, 0x00,
  0x20, 0x80, 0x40, 0x40, 0x80, 0x20, 0x80, 0x20, 0x80, 0x20, 0x80, 0x20,
  0x80, 0x20, 0x80, 0x20, 0x40, 0x40, 0x20, 0x80, 0x1F, 0x00, 0x02, 0x00,
  0x01, 0x00, 0x00, 0xC0, 0xFE, 0x00, 0x41, 0x00, 0x40, 0x80, 0x40, 0x80,
  0x40, 0x80, 0x41, 0x00, 0x7E, 0x00, 0x41, 0x00, 0x40, 0x80, 0x40, 0x80,
  0x40, 0x80, 0xE0, 0x60, 0x3E, 0xC3, 0x81, 0x80, 0x80, 0x70, 0x0E, 0x03,
  0x01, 0x81, 0xC3, 0x7C, 0xFF, 0x80, 0x88, 0x80, 0x88, 0x80, 0x08, 0x00,
  0x08, 0x00, 0x08, 0x00, 0x08, 0x00, 0x08, 0x00, 0x08, 0x00, 0x08, 0x00,
  0x08, 0x00, 0x1C, 0x00, 0xE0, 0xE0, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40,
  0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40,
  0x20, 0x80, 0x1F, 0x00, 0xF0, 0x70, 0x40, 0x20, 0x20, 0x40, 0x20, 0x40,
  0x20, 0x80, 0x10, 0x80, 0x10, 0x80, 0x09, 0x00, 0x09, 0x00, 0x09, 0x00,
  0x06, 0x00, 0x06, 0x00, 0xE1, 0x87, 0x41, 0x82, 0x41, 0x82, 0x22, 0x44,
  0x22, 0x44, 0x22, 0x44, 0x22, 0x44, 0x14, 0x28, 0x14, 0x28, 0x14, 0x28,
  0x08, 0x10, 0x08, 0x10, 0xF1, 0xE0, 0x20, 0x80, 0x11, 0x00, 0x11, 0x00,
  0x0A, 0x00, 0x04, 0x00, 0x0A, 0x00, 0x0A, 0x00, 0x11, 0x00, 0x11, 0x00,
  0x20, 0x80, 0xF1, 0xE0, 0xF1, 0xE0, 0x20, 0x80, 0x20, 0x80, 0x11, 0x00,
  0x0A, 0x00, 0x0A, 0x00, 0x04, 0x00, 0x04, 0x00, 0x04, 0x00, 0x04, 0x00,
  0x04, 0x00, 0x0E, 0x00, 0xFF, 0xC0, 0x80, 0x80, 0x81, 0x00, 0x03, 0x00,
  0x02, 0x00, 0x04, 0x00, 0x08, 0x00, 0x10, 0x00, 0x30, 0x00, 0x20, 0x40,
  0x40, 0x40, 0xFF, 0xC0, 0xE0, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
  0x80, 0x80, 0x80, 0x80, 0x80, 0xE0, 0x80, 0xC0, 0x40, 0x40, 0x60, 0x20,
  0x20, 0x20, 0x30, 0x10, 0x10, 0x18, 0x08, 0xE0, 0x20, 0x20, 0x20, 0x20,
  0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0xE0, 0x0C, 0x00, 0x1E,
  0x00, 0x21, 0x00, 0x40, 0x80, 0xFF, 0xC0, 0x60, 0x30, 0x3C, 0x46, 0x02,
  0x02, 0x7E, 0x82, 0x82, 0xC6, 0x7B, 0xC0, 0x00, 0x40, 0x00, 0x40, 0x00,
  0x5E, 0x00, 0x61, 0x00, 0x40, 0x80, 0x40, 0x80, 0x40, 0x80, 0x40, 0x80,
  0x40, 0x80, 0x61, 0x00, 0xDE, 0x00, 0x3C, 0x46, 0x82, 0x80, 0x80, 0x80,
  0x82, 0x44, 0x3C, 0x03, 0x00, 0x01, 0x00, 0x01, 0x00, 0x3D, 0x00, 0x43,
  0x00, 0x81, 0x00, 0x81, 0x00, 0x81, 0x00, 0x81, 0x00, 0x81, 0x00, 0x43,
  0x00, 0x3D, 0x80, 0x3C, 0x42, 0x81, 0x81, 0xFF, 0x80, 0x81, 0x42, 0x3C,
  0x3C, 0x44, 0x40, 0xF8, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0xF0,
  0x3D, 0x80, 0x43, 0x00, 0x81, 0x00, 0x81, 0x00, 0x81, 0x00, 0x81, 0x00,
  0x81, 0x00, 0x43, 0x00, 0x3D, 0x00, 0x01, 0x00, 0x42, 0x00, 0x3C, 0x00,
  0xC0, 0x00, 0x40, 0x00, 0x40, 0x00, 0x5E, 0x00, 0x63, 0x00, 0x41, 0x00,
  0x41, 0x00, 0x41, 0x00, 0x41, 0x00, 0x41, 0x00, 0x41, 0x00, 0xE3, 0x80,
  0x40, 0x40, 0x00, 0x00, 0xC0, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40,
  0xE0, 0x08, 0x08, 0x00, 0x00, 0x18, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,
  0x08, 0x08, 0x08, 0x88, 0x70, 0xC0, 0x40, 0x40, 0x4F, 0x44, 0x48, 0x50,
  0x70, 0x48, 0x44, 0x42, 0xE7, 0xC0, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40,
  0x40, 0x40, 0x40, 0x40, 0xE0, 0xDC, 0xE0, 0x63, 0x10, 0x42, 0x10, 0x42,
  0x10, 0x42, 0x10, 0x42, 0x10, 0x42, 0x10, 0x42, 0x10, 0xE7, 0x38, 0xDE,
  0x00, 0x63, 0x00, 0x41, 0x00, 0x41, 0x00, 0x41, 0x00, 0x41, 0x00, 0x41,
  0x00, 0x41, 0x00, 0xE3, 0x80, 0x3C, 0x42, 0x81, 0x81, 0x81, 0x81, 0x81,
  0x42, 0x3C, 0xDE, 0x00, 0x61, 0x00, 0x40, 0x80, 0x40, 0x80, 0x40, 0x80,
  0x40, 0x80, 0x40, 0x80, 0x61, 0x00, 0x5E, 0x00, 0x40, 0x00, 0x40, 0x00,
  0xE0, 0x00, 0x3D, 0x80, 0x43, 0x00, 0x81, 0x00, 0x81, 0x00, 0x81, 0x00,
  0x81, 0x00, 0x81, 0x00, 0x43, 0x00, 0x3D, 0x00, 0x01, 0x00, 0x01, 0x00,
  0x03, 0x80, 0xDE, 0x62, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0xF0, 0x7C,
  0x86, 0x82, 0xC0, 0x78, 0x06, 0x82, 0xC2, 0x7C, 0x40, 0x40, 0x40, 0xFC,
  0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x44, 0x38, 0xC3, 0x00, 0x41, 0x00,
  0x41, 0x00, 0x41, 0x00, 0x41, 0x00, 0x41, 0x00, 0x41, 0x00, 0x63, 0x00,
  0x3D, 0x80, 0xE3, 0x80, 0x41, 0x00, 0x41, 0x00, 0x22, 0x00, 0x22, 0x00,
  0x14, 0x00, 0x14, 0x00, 0x14, 0x00, 0x08, 0x00, 0xE2, 0x38, 0x42, 0x10,
  0x45, 0x10, 0x45, 0x10, 0x29, 0x20, 0x28, 0xA0, 0x28, 0xA0, 0x10, 0x40,
  0x10, 0x40, 0xE3, 0x80, 0x41, 0x00, 0x22, 0x00, 0x14, 0x00, 0x08, 0x00,
  0x14, 0x00, 0x22, 0x00, 0x41, 0x00, 0xE3, 0x80, 0xE7, 0x42, 0x42, 0x24,
  0x24, 0x28, 0x18, 0x18, 0x10, 0x10, 0xA0, 0xE0, 0xFE, 0x84, 0x0C, 0x08,
  0x10, 0x20, 0x62, 0x42, 0xFE, 0x18, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,
  0xC0, 0x60, 0x20, 0x20, 0x20, 0x20, 0x20, 0x18, 0x80, 0x80, 0x80, 0x80,
  0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
  0xC0, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x18, 0x20, 0x20, 0x20, 0x20,
  0x20, 0x20, 0xC0, 0x78, 0x40, 0x87, 0x80,
  // 0x370 - 0x3ff
  0xF0, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x40, 0x7F, 0xC0,
  0x60, 0x40, 0x60, 0x40, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0xF0, 0x00,
  0xC0, 0x40, 0x40, 0x7E, 0x40, 0x40, 0x40, 0x40, 0xFF, 0xE0, 0xC4, 0x60,
  0xC4, 0x60, 0xC4, 0x60, 0xC4, 0x60, 0xC4, 0x60, 0x04, 0x00, 0x04, 0x00,
  0x04, 0x00, 0x04, 0x00, 0x04, 0x00, 0x1F, 0x00, 0xFE, 0x92, 0x92, 0x92,
  0x10, 0x10, 0x10, 0x78, 0x40, 0x40, 0xC0, 0xC0, 0x40, 0x40, 0x40, 0xC0,
  0xF0, 0x20, 0x60, 0x60, 0x60, 0xE0, 0x61, 0x60, 0x61, 0x60, 0x62, 0x60,
  0x64, 0x60, 0x64, 0x60, 0x68, 0x60, 0x70, 0x60, 0x70, 0x60, 0x60, 0xF0,
  0xC3, 0x00, 0x47, 0x00, 0x47, 0x00, 0x4B, 0x00, 0x53, 0x00, 0x73, 0x00,
  0x63, 0x00, 0x43, 0x80, 0xFF, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81,
  0x81, 0x81, 0x81, 0x81, 0x81, 0xFF, 0xFF, 0x81, 0x81, 0x81, 0x81, 0x81,
  0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0xFF, 0xC0, 0xC0, 0x00, 0x10,
  0xEC, 0x86, 0x06, 0x02, 0x02, 0x86, 0x86, 0x7C, 0x3C, 0x46, 0x82, 0x98,
  0x98, 0x80, 0x82, 0x44, 0x3C, 0x10, 0xEC, 0x86, 0x36, 0x32, 0x02, 0x86,
  0x86, 0x7C, 0xC0, 0xC0, 0x00, 0x00, 0x00, 0x40, 0x40, 0xC0, 0x80, 0xFF,
  0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81,
  0xFF, 0xFF, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81,
  0x81, 0x81, 0xFF, 0xFF, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81,
  0x81, 0x81, 0x81, 0x81, 0xFF, 0xFF, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81,
  0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0xFF, 0xFF, 0x81, 0x81, 0x81, 0x81,
  0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0xFF, 0x30, 0x60, 0xC0,
  0x18, 0x30, 0x60, 0x00, 0xD8, 0xD8, 0x18, 0x00, 0x36, 0x00, 0x66, 0x00,
  0x0B, 0x00, 0x09, 0x00, 0x19, 0x00, 0x10, 0x80, 0x10, 0x80, 0x20, 0x80,
  0x3F, 0xC0, 0x20, 0x40, 0x40, 0x20, 0xE0, 0x70, 0xC0, 0xC0, 0x30, 0x00,
  0x6F, 0xF8, 0xC4, 0x08, 0x04, 0x08, 0x04, 0x00, 0x04, 0x20, 0x07, 0xE0,
  0x04, 0x20, 0x04, 0x00, 0x04, 0x00, 0x04, 0x08, 0x04, 0x08, 0x0F, 0xF8,
  0x30, 0x00, 0x6E, 0x0E, 0xC4, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,
  0x07, 0xFC, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,
  0x0E, 0x0E, 0x30, 0x6E, 0xC4, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,
  0x04, 0x04, 0x0E, 0xFF, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81,
  0x81, 0x81, 0x81, 0x81, 0xFF, 0x30, 0x00, 0x6F, 0x80, 0xD0, 0x40, 0x20,
  0x20, 0x40, 0x10, 0x40, 0x10, 0x40, 0x10, 0x40, 0x10, 0x40, 0x10, 0x40,
  0x10, 0x20, 0x20, 0x10, 0x40, 0x0F, 0x80, 0xFF, 0x81, 0x81, 0x81, 0x81,
  0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0xFF, 0x30, 0x00, 0x6F,
  0x1E, 0xC2, 0x08, 0x02, 0x08, 0x01, 0x10, 0x00, 0xA0, 0x00, 0xA0, 0x00,
  0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0xE0, 0x30,
  0x00, 0x6F, 0x80, 0xD0, 0x40, 0x20, 0x20, 0x40, 0x10, 0x40, 0x10, 0x40,
  0x10, 0x40, 0x10, 0x40, 0x10, 0x40, 0x20, 0x20, 0x20, 0x50, 0x50, 0x78,
  0xF0, 0x18, 0x30, 0x60, 0x00, 0xD8, 0xD8, 0x00, 0x00, 0xC0, 0x40, 0x40,
  0x40, 0x40, 0x40, 0x40, 0x30, 0x00, 0x06, 0x00, 0x06, 0x00, 0x0B, 0x00,
  0x09, 0x00, 0x19, 0x00, 0x10, 0x80, 0x10, 0x80, 0x20, 0x80, 0x3F, 0xC0,
  0x20, 0x40, 0x40, 0x20, 0xE0, 0x70, 0xFE, 0x00, 0x43, 0x00, 0x41, 0x00,
  0x41, 0x00, 0x43, 0x00, 0x7E, 0x00, 0x41, 0x00, 0x40, 0x80, 0x40, 0x80,
  0x40, 0x80, 0x41, 0x80, 0xFE, 0x00, 0xFF, 0x80, 0x60, 0x80, 0x60, 0x80,
  0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00,
  0x60, 0x00, 0x60, 0x00, 0xF8, 0x00, 0x08, 0x00, 0x0C, 0x00, 0x1C, 0x00,
  0x16, 0x00, 0x26, 0x00, 0x22, 0x00, 0x23, 0x00, 0x43, 0x00, 0x41, 0x80,
  0x41, 0x80, 0x80, 0x80, 0xFF, 0xC0, 0xFF, 0x80, 0x40, 0x80, 0x40, 0x80,
  0x40, 0x00, 0x42, 0x00, 0x7E, 0x00, 0x42, 0x00, 0x40, 0x00, 0x40, 0x00,
  0x40, 0x80, 0x40, 0x80, 0xFF, 0x80, 0xFF, 0xC0, 0x80, 0x80, 0x81, 0x00,
  0x03, 0x00, 0x02, 0x00, 0x04, 0x00, 0x08, 0x00, 0x10, 0x00, 0x30, 0x00,
  0x20, 0x40, 0x40, 0x40, 0xFF, 0xC0, 0xE0, 0xE0, 0x40, 0x40, 0x40, 0x40,
  0x40, 0x40, 0x40, 0x40, 0x7F, 0xC0, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40,
  0x40, 0x40, 0x40, 0x40, 0xE0, 0xE0, 0x1F, 0x00, 0x20, 0x80, 0x40, 0x40,
  0x80, 0x20, 0xA0, 0xA0, 0xBF, 0xA0, 0xA0, 0xA0, 0x80, 0x20, 0x80, 0x20,
  0x40, 0x40, 0x20, 0x80, 0x1F, 0x00, 0xE0, 0x40, 0x40, 0x40, 0x40, 0x40,
  0x40, 0x40, 0x40, 0x40, 0x40, 0xE0, 0xE7, 0x80, 0x42, 0x00, 0x44, 0x00,
  0x48, 0x00, 0x50, 0x00, 0x60, 0x00, 0x50, 0x00, 0x48, 0x00, 0x44, 0x00,
  0x42, 0x00, 0x41, 0x00, 0xE0, 0xC0, 0x04, 0x00, 0x06, 0x00, 0x0E, 0x00,
  0x0B, 0x00, 0x13, 0x00, 0x11, 0x00, 0x11, 0x80, 0x21, 0x80, 0x20, 0xC0,
  0x20, 0xC0, 0x40, 0x40, 0xE1, 0xF0, 0xE0, 0x1C, 0x60, 0x18, 0x50, 0x28,
  0x50, 0x28, 0x48, 0x48, 0x48, 0x48, 0x44, 0x88, 0x44, 0x88, 0x43, 0x08,
  0x43, 0x08, 0x40, 0x08, 0xE0, 0x1C, 0xE0, 0xE0, 0x70, 0x40, 0x50, 0x40,
  0x48, 0x40, 0x48, 0x40, 0x44, 0x40, 0x42, 0x40, 0x42, 0x40, 0x41, 0x40,
  0x40, 0xC0, 0x40, 0xC0, 0xE0, 0x40, 0xFF, 0x80, 0xFF, 0x80, 0x80, 0x80,
  0x00, 0x00, 0x22, 0x00, 0x3E, 0x00, 0x22, 0x00, 0x22, 0x00, 0x00, 0x00,
  0x80, 0x80, 0x80, 0x80, 0xFF, 0x80, 0x1F, 0x00, 0x20, 0x80, 0x40, 0x40,
  0x80, 0x20, 0x80, 0x20, 0x80, 0x20, 0x80, 0x20, 0x80, 0x20, 0x80, 0x20,
  0x40, 0x40, 0x20, 0x80, 0x1F, 0x00, 0xFF, 0xE0, 0x40, 0x40, 0x40, 0x40,
  0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40,
  0x40, 0x40, 0x40, 0x40, 0xE0, 0xE0, 0xFE, 0x00, 0x41, 0x00, 0x40, 0x80,
  0x40, 0x80, 0x40, 0x80, 0x41, 0x00, 0x7E, 0x00, 0x40, 0x00, 0x40, 0x00,
  0x40, 0x00, 0x40, 0x00, 0xE0, 0x00, 0xFF, 0x81, 0x81, 0x81, 0x81, 0x81,
  0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0xFF, 0xFF, 0x80, 0x60, 0x80,
  0x30, 0x80, 0x38, 0x00, 0x1C, 0x00, 0x0C, 0x00, 0x04, 0x00, 0x08, 0x00,
  0x10, 0x00, 0x20, 0x80, 0x40, 0x80, 0xFF, 0x80, 0xFF, 0x80, 0x88, 0x80,
  0x88, 0x80, 0x08, 0x00, 0x08, 0x00, 0x08, 0x00, 0x08, 0x00, 0x08, 0x00,
  0x08, 0x00, 0x08, 0x00, 0x08, 0x00, 0x1C, 0x00, 0xF1, 0xE0, 0x20, 0x80,
  0x20, 0x80, 0x11, 0x00, 0x0A, 0x00, 0x0A, 0x00, 0x04, 0x00, 0x04, 0x00,
  0x04, 0x00, 0x04, 0x00, 0x04, 0x00, 0x0E, 0x00, 0x1F, 0x00, 0x04, 0x00,
  0x1F, 0x80, 0x64, 0x40, 0xC4, 0x60, 0xC4, 0x60, 0xC4, 0x60, 0xC4, 0x60,
  0x44, 0x40, 0x34, 0x80, 0x0F, 0x00, 0x1F, 0x00, 0xF1, 0xE0, 0x20, 0x80,
  0x11, 0x00, 0x11, 0x00, 0x0A, 0x00, 0x04, 0x00, 0x0A, 0x00, 0x0A, 0x00,
  0x11, 0x00, 0x11, 0x00, 0x20, 0x80, 0xF1, 0xE0, 0xCF, 0x30, 0x66, 0x60,
  0x66, 0x60, 0x66, 0x60, 0x66, 0x60, 0x66, 0x60, 0x66, 0x60, 0x36, 0x40,
  0x1F, 0x80, 0x06, 0x00, 0x06, 0x00, 0x0F, 0x00, 0x1F, 0x00, 0x20, 0x80,
  0x40, 0x40, 0x80, 0x20, 0x80, 0x20, 0x80, 0x20, 0x80, 0x20, 0x80, 0x20,
  0x80, 0x40, 0x40, 0x40, 0xA0, 0xA0, 0xF1, 0xE0, 0xD8, 0xD8, 0x00, 0x70,
  0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x70, 0x1B,
  0x00, 0x1B, 0x00, 0x00, 0x00, 0xF1, 0xE0, 0x20, 0x80, 0x20, 0x80, 0x11,
  0x00, 0x0A, 0x00, 0x0A, 0x00, 0x04, 0x00, 0x04, 0x00, 0x04, 0x00, 0x04,
  0x00, 0x04, 0x00, 0x0E, 0x00, 0x06, 0x00, 0x0C, 0x00, 0x18, 0x00, 0x00,
  0x00, 0x10, 0x00, 0x6D, 0x00, 0xC5, 0x00, 0xC6, 0x00, 0x86, 0x00, 0xC2,
  0x00, 0xC6, 0x00, 0xC7, 0x00, 0x79, 0x80, 0x00, 0x00, 0x0C, 0x18, 0x30,
  0x00, 0x7C, 0x82, 0x80, 0xC0, 0x38, 0xC0, 0x82, 0xC6, 0x7C, 0x06, 0x0C,
  0x18, 0x00, 0xDE, 0x63, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x01,
  0x01, 0x01, 0x18, 0x30, 0x60, 0x00, 0x00, 0xC0, 0x40, 0x40, 0x40, 0x40,
  0x40, 0x40, 0x30, 0x00, 0x06, 0x0C, 0x18, 0x00, 0x36, 0x36, 0x00, 0x00,
  0xC6, 0x43, 0x43, 0x43, 0x43, 0x43, 0x66, 0x3C, 0x10, 0x00, 0x6D, 0x00,
  0xC5, 0x00, 0xC6, 0x00, 0x86, 0x00, 0xC2, 0x00, 0xC6, 0x00, 0xC7, 0x00,
  0x79, 0x80, 0x00, 0x00, 0x78, 0xC4, 0xC4, 0x84, 0x8C, 0xB8, 0x86, 0x82,
  0x82, 0x82, 0x86, 0xFC, 0x80, 0x80, 0x80, 0xC2, 0x63, 0x23, 0x22, 0x32,
  0x34, 0x18, 0x18, 0x18, 0x38, 0x38, 0x7E, 0x42, 0x40, 0x70, 0x6E, 0xC2,
  0xC3, 0x83, 0xC3, 0xC3, 0xC2, 0x7C, 0x7C, 0x82, 0x80, 0xC0, 0x38, 0xC0,
  0x82, 0xC6, 0x7C, 0xFE, 0x0C, 0x30, 0x60, 0x40, 0xC0, 0xC0, 0x80, 0xC0,
  0xC0, 0x40, 0x78, 0x06, 0x02, 0x0E, 0xDE, 0x63, 0x41, 0x41, 0x41, 0x41,
  0x41, 0x41, 0x41, 0x01, 0x01, 0x01, 0x3C, 0x42, 0x42, 0x81, 0x81, 0xFF,
  0x81, 0x81, 0x81, 0x42, 0x42, 0x3C, 0xC0, 0x40, 0x40, 0x40, 0x40, 0x40,
  0x40, 0x30, 0x00, 0xEF, 0x80, 0x46, 0x00, 0x4C, 0x00, 0x58, 0x00, 0x70,
  0x00, 0x48, 0x00, 0x44, 0x00, 0x46, 0x00, 0xEF, 0x80, 0xE0, 0x00, 0x90,
  0x00, 0x10, 0x00, 0x08, 0x00, 0x18, 0x00, 0x1C, 0x00, 0x14, 0x00, 0x34,
  0x00, 0x22, 0x00, 0x62, 0x00, 0x41, 0x00, 0xE3, 0x80, 0xC3, 0x00, 0x41,
  0x00, 0x41, 0x00, 0x41, 0x00, 0x41, 0x00, 0x41, 0x00, 0x41, 0x00, 0x63,
  0x00, 0x7D, 0x80, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0xC2, 0x43, 0x43,
  0x43, 0x46, 0x44, 0x58, 0x60, 0xFE, 0x30, 0x60, 0x60, 0x60, 0x3C, 0x70,
  0xC0, 0xC0, 0xC0, 0xC0, 0x78, 0x06, 0x02, 0x0E, 0x3C, 0x42, 0x81, 0x81,
  0x81, 0x81, 0x81, 0x42, 0x3C, 0xFF, 0x80, 0x41, 0x00, 0x41, 0x00, 0x41,
  0x00, 0x41, 0x00, 0x41, 0x00, 0x41, 0x00, 0x41, 0x00, 0xE3, 0x80, 0x10,
  0x6C, 0xC2, 0xC2, 0x83, 0x83, 0xC2, 0xC6, 0xBC, 0x80, 0x80, 0x80, 0x10,
  0x6E, 0xC2, 0xC0, 0x80, 0xC0, 0xC0, 0xC0, 0x78, 0x06, 0x02, 0x0E, 0x7F,
  0x80, 0xC2, 0x00, 0xC3, 0x00, 0x83, 0x00, 0xC3, 0x00, 0xC3, 0x00, 0xC2,
  0x00, 0x7C, 0x00, 0xFE, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1C, 0x00,
  0xC6, 0x43, 0x43, 0x43, 0x43, 0x43, 0x66, 0x3C, 0x47, 0x80, 0xCC, 0x80,
  0x88, 0x80, 0x88, 0x80, 0x88, 0x80, 0x88, 0x80, 0xC9, 0x80, 0x7F, 0x00,
  0x08, 0x00, 0x08, 0x00, 0x08, 0x00, 0x00, 0x00, 0x73, 0x80, 0x13, 0x00,
  0x12, 0x00, 0x0E, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x1C, 0x00, 0x1C, 0x00,
  0x34, 0x00, 0x34, 0x00, 0xF2, 0x80, 0x01, 0x00, 0xC4, 0x40, 0x44, 0xC0,
  0x44, 0xC0, 0x64, 0xC0, 0x64, 0xC0, 0x64, 0x80, 0x25, 0x80, 0x1F, 0x00,
  0x04, 0x00, 0x04, 0x00, 0x04, 0x00, 0x40, 0x40, 0xC0, 0x60, 0x84, 0x20,
  0x84, 0x20, 0x84, 0x20, 0x84, 0x60, 0xCE, 0x60, 0x7B, 0xC0, 0xD8, 0xD8,
  0x00, 0x00, 0xC0, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x30, 0x00, 0x36,
  0x36, 0x00, 0x00, 0xC6, 0x43, 0x43, 0x43, 0x43, 0x43, 0x66, 0x3C, 0x06,
  0x0C, 0x18, 0x00, 0x3C, 0x42, 0x81, 0x81, 0x81, 0x81, 0x81, 0x42, 0x3C,
  0x0C, 0x18, 0x30, 0x00, 0x00, 0xC6, 0x43, 0x43, 0x43, 0x43, 0x43, 0x66,
  0x3C, 0x01, 0x80, 0x03, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40,
  0x40, 0xC0, 0x60, 0x84, 0x20, 0x84, 0x20, 0x84, 0x20, 0x84, 0x60, 0xCE,
  0x60, 0x7B, 0xC0, 0xF1, 0xE0, 0x60, 0x80, 0x63, 0x00, 0x66, 0x00, 0x68,
  0x00, 0x70, 0x00, 0x78, 0x00, 0x6C, 0x00, 0x66, 0x00, 0x63, 0x00, 0x61,
  0x80, 0xF1, 0xC0, 0x0B, 0x80, 0x07, 0x00, 0x02, 0x00, 0x01, 0x00, 0x3C,
  0x46, 0xC2, 0xC6, 0xC4, 0xFC, 0x82, 0xC2, 0xC3, 0xC2, 0x42, 0x3C, 0x1E,
  0x00, 0x11, 0x00, 0x11, 0x00, 0x11, 0x80, 0xD9, 0x80, 0x4F, 0xC0, 0x41,
  0x80, 0x61, 0x80, 0x61, 0x80, 0x61, 0x00, 0x23, 0x00, 0x1E, 0x00, 0xC1,
  0x80, 0x32, 0x40, 0x1A, 0x40, 0x0C, 0x40, 0x0C, 0x00, 0x0C, 0x00, 0x0C,
  0x00, 0x0C, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x1F, 0x00, 0x30,
  0x00, 0x78, 0x30, 0xC6, 0x48, 0x03, 0x48, 0x01, 0x88, 0x01, 0x80, 0x01,
  0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x03,
  0xE0, 0x36, 0x00, 0x36, 0x00, 0x00, 0x00, 0xC1, 0x80, 0x32, 0x40, 0x1A,
  0x40, 0x0C, 0x40, 0x0C, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x0C,
  0x00, 0x0C, 0x00, 0x0C, 0x00, 0x1F, 0x00, 0x08, 0x00, 0x08, 0x00, 0x08,
  0x00, 0x08, 0x00, 0x7F, 0x00, 0xC9, 0x80, 0x88, 0x80, 0x88, 0x80, 0x88,
  0x80, 0x89, 0x80, 0xC9, 0x80, 0x7F, 0x00, 0x08, 0x00, 0x08, 0x00, 0x08,
  0x00, 0xFF, 0xF8, 0x60, 0x30, 0x42, 0x10, 0x42, 0x10, 0x42, 0x10, 0x42,
  0x30, 0x67, 0x30, 0x3D, 0xE0, 0x41, 0xA3, 0x22, 0x24, 0x3C, 0x3C, 0x68,
  0xC5, 0x87, 0x01, 0x03, 0x12, 0x0C, 0x1F, 0x00, 0x20, 0x80, 0x60, 0x40,
  0xC0, 0x60, 0xC0, 0x60, 0xC0, 0x60, 0xC0, 0x60, 0xC0, 0x60, 0xC0, 0x60,
  0x40, 0x40, 0x60, 0xC0, 0x1F, 0x00, 0x04, 0x00, 0x04, 0x00, 0x04, 0x00,
  0x10, 0x6E, 0xC2, 0xC3, 0x83, 0xC3, 0xC3, 0xC2, 0x7C, 0x18, 0x18, 0x18,
  0x1F, 0x00, 0x20, 0xC0, 0x60, 0x40, 0xC0, 0x40, 0xC0, 0x00, 0xC0, 0x00,
  0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0x40, 0x00, 0x60, 0x00, 0x1F, 0x00,
  0x00, 0xC0, 0x00, 0xC0, 0x01, 0x80, 0x02, 0x1E, 0x60, 0xC0, 0xC0, 0x80,
  0xC0, 0xC0, 0xC0, 0x78, 0x06, 0x02, 0x0E, 0xFF, 0x80, 0x40, 0x80, 0x40,
  0x80, 0x40, 0x00, 0x42, 0x00, 0x7E, 0x00, 0x42, 0x00, 0x40, 0x00, 0x40,
  0x00, 0x40, 0x00, 0x40, 0x00, 0xF0, 0x00, 0x0F, 0x19, 0x18, 0x10, 0x10,
  0x10, 0x1C, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0xF0, 0xC0, 0x40,
  0x40, 0x42, 0x46, 0x9E, 0xFA, 0xE2, 0x02, 0x04, 0x04, 0x06, 0x40, 0x40,
  0x40, 0x80, 0x80, 0xFE, 0x02, 0x04, 0x04, 0x04, 0x08, 0x08, 0x3E, 0x00,
  0xC1, 0x00, 0xC1, 0x80, 0x82, 0xC0, 0x02, 0x40, 0x04, 0xE0, 0x08, 0xE0,
  0x11, 0x60, 0x12, 0x60, 0x22, 0x40, 0x44, 0x40, 0x48, 0xC0, 0x00, 0x80,
  0x01, 0x80, 0x07, 0x00, 0x80, 0x78, 0x0C, 0x0E, 0x16, 0x26, 0x27, 0x4B,
  0x93, 0x02, 0x02, 0x02, 0xFF, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81,
  0x81, 0x81, 0x81, 0x81, 0x81, 0xFF, 0xFF, 0x81, 0x81, 0x81, 0x81, 0x81,
  0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0xFF, 0xFF, 0x81, 0x81, 0x81,
  0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0xFF, 0xFF, 0x81,
  0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0xFF,
  0xFF, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81,
  0x81, 0xFF, 0xFF, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81,
  0x81, 0x81, 0x81, 0xFF, 0xFF, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81,
  0x81, 0x81, 0x81, 0x81, 0x81, 0xFF, 0xFF, 0x81, 0x81, 0x81, 0x81, 0x81,
  0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0xFF, 0xFF, 0x81, 0x81, 0x81,
  0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0xFF, 0xFF, 0x81,
  0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0xFF,
  0xFF, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81,
  0x81, 0xFF, 0xFF, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81,
  0x81, 0x81, 0x81, 0xFF, 0xFF, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81,
  0x81, 0x81, 0x81, 0x81, 0x81, 0xFF, 0xFF, 0x81, 0x81, 0x81, 0x81, 0x81,
  0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0xFF, 0x41, 0xA3, 0x22, 0x24,
  0x3C, 0x3C, 0x68, 0xC5, 0x87, 0x00, 0x10, 0x6C, 0xC2, 0xC2, 0x83, 0x83,
  0xC2, 0xC6, 0xBC, 0x80, 0xC0, 0x7E, 0x3C, 0x46, 0x82, 0x80, 0x80, 0x80,
  0x82, 0x44, 0x3C, 0x08, 0x08, 0x00, 0x00, 0x18, 0x08, 0x08, 0x08, 0x08,
  0x08, 0x08, 0x08, 0x08, 0x08, 0x88, 0x70, 0x1F, 0x00, 0x20, 0x80, 0x60,
  0x40, 0xC0, 0x60, 0xC0, 0x60, 0xFF, 0xE0, 0xC0, 0x60, 0xC0, 0x60, 0xC0,
  0x60, 0x40, 0x40, 0x60, 0xC0, 0x1F, 0x00, 0x3C, 0x46, 0x82, 0x80, 0xF8,
  0x80, 0x82, 0x44, 0x3C, 0x78, 0xC4, 0x82, 0x02, 0x3E, 0x02, 0x82, 0x44,
  0x78, 0xE0, 0x00, 0x40, 0x00, 0x7E, 0x00, 0x41, 0x00, 0x40, 0x80, 0x40,
  0x80, 0x40, 0x80, 0x41, 0x00, 0x7E, 0x00, 0x40, 0x00, 0x40, 0x00, 0xE0,
  0x00, 0xE0, 0x00, 0x20, 0x00, 0x20, 0x00, 0x24, 0x00, 0x2B, 0x00, 0x31,
  0x80, 0x20, 0x80, 0x20, 0x80, 0x20, 0x80, 0x20, 0x80, 0x31, 0x80, 0x2F,
  0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x1F, 0x80, 0x20, 0xC0, 0x40,
  0x40, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80,
  0x00, 0x40, 0x40, 0x20, 0x80, 0x1F, 0x00, 0xE0, 0x1E, 0x70, 0x38, 0x58,
  0x78, 0x4C, 0x58, 0x46, 0x98, 0x43, 0x18, 0x40, 0x18, 0x40, 0x18, 0x40,
  0x18, 0x40, 0x18, 0x40, 0x18, 0xF0, 0x3E, 0xC3, 0x80, 0xE3, 0x00, 0xE5,
  0x00, 0xB5, 0x00, 0x99, 0x00, 0x81, 0x00, 0x81, 0x00, 0x83, 0xC0, 0x80,
  0x00, 0x80, 0x00, 0x80, 0x00, 0x08, 0x36, 0x43, 0x43, 0xC3, 0xC3, 0xC3,
  0xC2, 0xFE, 0xC0, 0xC0, 0xF8, 0x3E, 0x00, 0xC1, 0x00, 0xC0, 0x80, 0x80,
  0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0xC0,
  0x80, 0x41, 0x80, 0x3F, 0x00, 0x1F, 0x80, 0x20, 0xC0, 0x40, 0x40, 0x80,
  0x00, 0x80, 0x00, 0x8C, 0x00, 0x8C, 0x00, 0x80, 0x00, 0x80, 0x00, 0x40,
  0x40, 0x20, 0x80, 0x1F, 0x00, 0x3E, 0x00, 0xC1, 0x00, 0xC0, 0x80, 0x80,
  0xC0, 0x00, 0xC0, 0x18, 0xC0, 0x18, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0xC0,
  0x80, 0x41, 0x80, 0x3F, 0x00
};

/* This struct will be read by the CPU only */
static const nema_glyph_t DejaVuSerif8pt1bGlyphs0[] = {
  {     0,   0,   5,    0,    1 },   // 0x20 ' '
  {     0,   2,   6,    2,  -11 },   // 0x21 '!'
  {    12,   4,   7,    2,  -11 },   // 0x22 '"'
  {    16,  11,  13,    1,  -10 },   // 0x23 '#'
  {    38,   8,  10,    1,  -11 },   // 0x24 '$'
  {    52,  13,  15,    1,  -11 },   // 0x25 '%'
  {    76,  13,  14,    1,  -11 },   // 0x26 '&'
  {   100,   1,   4,    2,  -11 },   // 0x27 '''
  {   104,   3,   6,    1,  -11 },   // 0x28 '('
  {   119,   3,   6,    1,  -11 },   // 0x29 ')'
  {   134,   7,   8,    1,  -11 },   // 0x2A '*'
  {   141,  11,  13,    1,  -10 },   // 0x2B '+'
  {   163,   2,   5,    1,   -1 },   // 0x2C ','
  {   167,   4,   5,    1,   -4 },   // 0x2D '-'
  {   168,   2,   6,    2,   -1 },   // 0x2E '.'
  {   170,   5,   5,    0,  -11 },   // 0x2F '/'
  {   183,   8,  10,    1,  -11 },   // 0x30 '0'
  {   195,   5,  10,    2,  -11 },   // 0x31 '1'
  {   207,   8,  10,    1,  -11 },   // 0x32 '2'
  {   219,   8,  10,    1,  -11 },   // 0x33 '3'
  {   231,   9,  10,    1,  -11 },   // 0x34 '4'
  {   255,   8,  10,    1,  -11 },   // 0x35 '5'
  {   267,   8,  10,    1,  -11 },   // 0x36 '6'
  {   279,   8,  10,    1,  -11 },   // 0x37 '7'
  {   291,   8,  10,    1,  -11 },   // 0x38 '8'
  {   303,   8,  10,    1,  -11 },   // 0x39 '9'
  {   315,   2,   5,    2,   -6 },   // 0x3A ':'
  {   322,   2,   5,    1,   -6 },   // 0x3B ';'
  {   331,  10,  13,    2,   -8 },   // 0x3C '<'
  {   349,  10,  13,    2,   -6 },   // 0x3D '='
  {   357,  10,  13,    2,   -8 },   // 0x3E '>'
  {   375,   7,   9,    1,  -11 },   // 0x3F '?'
  {   387,  14,  16,    1,  -10 },   // 0x40 '@'
  {   415,  12,  12,    0,  -11 },   // 0x41 'A'
  {   439,   9,  12,    1,  -11 },   // 0x42 'B'
  {   463,  10,  12,    1,  -11 },   // 0x43 'C'
  {   487,  11,  13,    1,  -11 },   // 0x44 'D'
  {   511,   9,  12,    1,  -11 },   // 0x45 'E'
  {   535,   9,  11,    1,  -11 },   // 0x46 'F'
  {   559,  11,  13,    1,  -11 },   // 0x47 'G'
  {   583,  11,  13,    1,  -11 },   // 0x48 'H'
  {   607,   3,   5,    1,  -11 },   // 0x49 'I'
  {   619,   7,   6,   -2,  -11 },   // 0x4A 'J'
  {   634,  10,  11,    1,  -11 },   // 0x4B 'K'
  {   658,   9,  11,    1,  -11 },   // 0x4C 'L'
  {   682,  14,  16,    1,  -11 },   // 0x4D 'M'
  {   706,  11,  13,    1,  -11 },   // 0x4E 'N'
  {   730,  11,  13,    1,  -11 },   // 0x4F 'O'
  {   754,   9,  11,    1,  -11 },   // 0x50 'P'
  {   778,  11,  13,    1,  -11 },   // 0x51 'Q'
  {   808,  11,  12,    1,  -11 },   // 0x52 'R'
  {   832,   8,  10,    1,  -11 },   // 0x53 'S'
  {   844,   9,  11,    1,  -11 },   // 0x54 'T'
  {   868,  11,  13,    1,  -11 },   // 0x55 'U'
  {   892,  12,  12,    0,  -11 },   // 0x56 'V'
  {   916,  16,  16,    0,  -11 },   // 0x57 'W'
  {   940,  11,  11,    0,  -11 },   // 0x58 'X'
  {   964,  11,  11,    0,  -11 },   // 0x59 'Y'
  {   988,  10,  11,    1,  -11 },   // 0x5A 'Z'
  {  1012,   3,   6,    1,  -11 },   // 0x5B '['
  {  1026,   5,   5,    0,  -11 },   // 0x5C '\'
  {  1039,   3,   6,    1,  -11 },   // 0x5D ']'
  {  1053,  10,  13,    2,  -11 },   // 0x5E '^'
  {  1061,   8,   8,    0,    4 },   // 0x5F '_'
  {  1062,   4,   8,    2,  -12 },   // 0x60 '`'
  {  1065,   8,  10,    1,   -8 },   // 0x61 'a'
  {  1074,   9,  11,    1,  -11 },   // 0x62 'b'
  {  1098,   7,   9,    1,   -8 },   // 0x63 'c'
  {  1107,   9,  11,    1,  -11 },   // 0x64 'd'
  {  1131,   8,  10,    1,   -8 },   // 0x65 'e'
  {  1140,   6,   6,    1,  -11 },   // 0x66 'f'
  {  1152,   9,  11,    1,   -8 },   // 0x67 'g'
  {  1176,   9,  11,    1,  -11 },   // 0x68 'h'
  {  1200,   3,   5,    1,  -12 },   // 0x69 'i'
  {  1213,   5,   4,   -2,  -12 },   // 0x6A 'j'
  {  1229,   8,  10,    1,  -11 },   // 0x6B 'k'
  {  1241,   3,   5,    1,  -11 },   // 0x6C 'l'
  {  1253,  13,  15,    1,   -8 },   // 0x6D 'm'
  {  1271,   9,  11,    1,   -8 },   // 0x6E 'n'
  {  1289,   8,  10,    1,   -8 },   // 0x6F 'o'
  {  1298,   9,  11,    1,   -8 },   // 0x70 'p'
  {  1322,   9,  11,    1,   -8 },   // 0x71 'q'
  {  1346,   7,   8,    1,   -8 },   // 0x72 'r'
  {  1355,   7,   9,    1,   -8 },   // 0x73 's'
  {  1364,   6,   7,    1,  -11 },   // 0x74 't'
  {  1376,   9,  11,    1,   -8 },   // 0x75 'u'
  {  1394,   9,   9,    0,   -8 },   // 0x76 'v'
  {  1412,  13,  14,    0,   -8 },   // 0x77 'w'
  {  1430,   9,   9,    0,   -8 },   // 0x78 'x'
  {  1448,   8,   9,    1,   -8 },   // 0x79 'y'
  {  1460,   7,   8,    1,   -8 },   // 0x7A 'z'
  {  1469,   5,  10,    2,  -11 },   // 0x7B '{'
  {  1484,   1,   5,    2,  -11 },   // 0x7C '|'
  {  1500,   5,  10,    2,  -11 },   // 0x7D '}'
  {  1515,  10,  13,    2,   -5 },   // 0x7E '~'
  {  1519,   0,   0,    0,    0 }
};

/* This struct will be read by the CPU only */
static const nema_glyph_t DejaVuSerif8pt1bGlyphs1[] = {
  {  1519,  10,  12,    1,  -11 },   // 0x370
  {  1543,   7,   9,    1,   -7 },   // 0x371
  {  1551,  11,  11,    0,  -11 },   // 0x372
  {  1575,   7,   9,    1,   -7 },   // 0x373
  {  1583,   2,   4,    1,  -12 },   // 0x374
  {  1587,   2,   4,    1,    0 },   // 0x375
  {  1591,  12,  14,    1,  -11 },   // 0x376
  {  1615,   9,  11,    1,   -7 },   // 0x377
  {  1631,   8,  10,    1,  -10 },   // 0x378
  {  1645,   8,  10,    1,  -10 },   // 0x379
  {  1659,   2,   8,    3,    2 },   // 0x37A
  {  1662,   7,   9,    1,   -8 },   // 0x37B
  {  1671,   7,   9,    1,   -8 },   // 0x37C
  {  1680,   7,   9,    1,   -8 },   // 0x37D
  {  1689,   2,   5,    1,   -6 },   // 0x37E
  {  1698,   8,  10,    1,  -10 },   // 0x37F
  {  1712,   8,  10,    1,  -10 },   // 0x380
  {  1726,   8,  10,    1,  -10 },   // 0x381
  {  1740,   8,  10,    1,  -10 },   // 0x382
  {  1754,   8,  10,    1,  -10 },   // 0x383
  {  1768,   4,   8,    3,  -12 },   // 0x384
  {  1771,   5,   8,    2,  -15 },   // 0x385
  {  1777,  12,  12,    0,  -12 },   // 0x386
  {  1803,   2,   5,    1,   -6 },   // 0x387
  {  1805,  13,  14,    0,  -12 },   // 0x388
  {  1831,  15,  17,    0,  -12 },   // 0x389
  {  1857,   7,   9,    0,  -12 },   // 0x38A
  {  1870,   8,  10,    1,  -10 },   // 0x38B
  {  1884,  12,  13,    0,  -12 },   // 0x38C
  {  1910,   8,  10,    1,  -10 },   // 0x38D
  {  1924,  15,  14,    0,  -12 },   // 0x38E
  {  1950,  12,  14,    0,  -12 },   // 0x38F
  {  1976,   5,   6,    1,  -15 },   // 0x390
  {  1993,  12,  12,    0,  -11 },   // 0x391
  {  2017,   9,  12,    1,  -11 },   // 0x392
  {  2041,   9,  11,    1,  -11 },   // 0x393
  {  2065,  10,  12,    1,  -11 },   // 0x394
  {  2089,   9,  12,    1,  -11 },   // 0x395
  {  2113,  10,  11,    1,  -11 },   // 0x396
  {  2137,  11,  14,    1,  -11 },   // 0x397
  {  2161,  11,  13,    1,  -11 },   // 0x398
  {  2185,   3,   6,    1,  -11 },   // 0x399
  {  2197,  10,  12,    1,  -11 },   // 0x39A
  {  2221,  12,  12,    0,  -11 },   // 0x39B
  {  2245,  14,  16,    1,  -11 },   // 0x39C
  {  2269,  11,  14,    1,  -11 },   // 0x39D
  {  2293,   9,  11,    1,  -11 },   // 0x39E
  {  2317,  11,  13,    1,  -11 },   // 0x39F
  {  2341,  11,  14,    1,  -11 },   // 0x3A0
  {  2365,   9,  11,    1,  -11 },   // 0x3A1
  {  2389,   8,  10,    1,  -10 },   // 0x3A2
  {  2403,   9,  11,    1,  -11 },   // 0x3A3
  {  2427,   9,  11,    1,  -11 },   // 0x3A4
  {  2451,  11,  11,    0,  -11 },   // 0x3A5
  {  2475,  11,  13,    1,  -11 },   // 0x3A6
  {  2499,  11,  11,    0,  -11 },   // 0x3A7
  {  2523,  12,  14,    1,  -11 },   // 0x3A8
  {  2547,  11,  13,    1,  -11 },   // 0x3A9
  {  2571,   5,   6,    0,  -14 },   // 0x3AA
  {  2586,  11,  11,    0,  -14 },   // 0x3AB
  {  2616,   9,  11,    1,  -12 },   // 0x3AC
  {  2644,   7,   8,    1,  -12 },   // 0x3AD
  {  2657,   8,  10,    1,  -12 },   // 0x3AE
  {  2673,   5,   6,    1,  -12 },   // 0x3AF
  {  2687,   8,  10,    1,  -15 },   // 0x3B0
  {  2703,   9,  11,    1,   -8 },   // 0x3B1
  {  2723,   7,   9,    1,  -11 },   // 0x3B2
  {  2738,   8,  10,    1,   -7 },   // 0x3B3
  {  2749,   8,  10,    1,  -11 },   // 0x3B4
  {  2761,   7,   7,    1,   -8 },   // 0x3B5
  {  2770,   7,   9,    1,  -11 },   // 0x3B6
  {  2785,   8,  11,    1,   -8 },   // 0x3B7
  {  2797,   8,  10,    1,  -11 },   // 0x3B8
  {  2809,   4,   6,    1,   -7 },   // 0x3B9
  {  2818,   9,  10,    1,   -8 },   // 0x3BA
  {  2836,   9,  10,    1,  -11 },   // 0x3BB
  {  2860,   9,  11,    1,   -8 },   // 0x3BC
  {  2884,   8,  10,    1,   -7 },   // 0x3BD
  {  2892,   7,   9,    1,  -11 },   // 0x3BE
  {  2907,   8,  10,    1,   -8 },   // 0x3BF
  {  2916,   9,  11,    1,   -8 },   // 0x3C0
  {  2934,   8,   9,    1,   -8 },   // 0x3C1
  {  2946,   7,   9,    1,   -8 },   // 0x3C2
  {  2958,   9,  11,    1,   -7 },   // 0x3C3
  {  2974,   7,   9,    1,   -7 },   // 0x3C4
  {  2983,   8,  10,    1,   -7 },   // 0x3C5
  {  2991,  10,  11,    1,   -7 },   // 0x3C6
  {  3013,   9,  10,    0,   -8 },   // 0x3C7
  {  3039,  11,  13,    1,   -7 },   // 0x3C8
  {  3061,  11,  13,    1,   -7 },   // 0x3C9
  {  3077,   5,   6,    1,  -11 },   // 0x3CA
  {  3090,   8,  10,    1,  -11 },   // 0x3CB
  {  3102,   8,  10,    1,  -12 },   // 0x3CC
  {  3115,   8,  10,    1,  -12 },   // 0x3CD
  {  3128,  11,  13,    1,  -12 },   // 0x3CE
  {  3154,  11,  12,    1,  -11 },   // 0x3CF
  {  3186,   8,   9,    1,  -11 },   // 0x3D0
  {  3198,  10,  11,    1,  -11 },   // 0x3D1
  {  3222,  11,  11,    0,  -11 },   // 0x3D2
  {  3246,  14,  14,    0,  -12 },   // 0x3D3
  {  3272,  11,  11,    0,  -14 },   // 0x3D4
  {  3302,   9,  11,    1,  -11 },   // 0x3D5
  {  3332,  13,  13,    0,   -7 },   // 0x3D6
  {  3348,   8,  10,    1,   -8 },   // 0x3D7
  {  3361,  11,  13,    1,  -11 },   // 0x3D8
  {  3391,   8,  10,    1,   -8 },   // 0x3D9
  {  3403,  10,  12,    1,  -11 },   // 0x3DA
  {  3433,   7,   9,    1,   -9 },   // 0x3DB
  {  3446,   9,  11,    1,  -11 },   // 0x3DC
  {  3470,   8,   7,   -1,  -11 },   // 0x3DD
  {  3485,   7,   9,    1,  -11 },   // 0x3DE
  {  3497,   7,  11,    2,  -11 },   // 0x3DF
  {  3509,  11,  13,    1,  -11 },   // 0x3E0
  {  3539,   8,   9,    1,   -8 },   // 0x3E1
  {  3551,   8,  10,    1,  -10 },   // 0x3E2
  {  3565,   8,  10,    1,  -10 },   // 0x3E3
  {  3579,   8,  10,    1,  -10 },   // 0x3E4
  {  3593,   8,  10,    1,  -10 },   // 0x3E5
  {  3607,   8,  10,    1,  -10 },   // 0x3E6
  {  3621,   8,  10,    1,  -10 },   // 0x3E7
  {  3635,   8,  10,    1,  -10 },   // 0x3E8
  {  3649,   8,  10,    1,  -10 },   // 0x3E9
  {  3663,   8,  10,    1,  -10 },   // 0x3EA
  {  3677,   8,  10,    1,  -10 },   // 0x3EB
  {  3691,   8,  10,    1,  -10 },   // 0x3EC
  {  3705,   8,  10,    1,  -10 },   // 0x3ED
  {  3719,   8,  10,    1,  -10 },   // 0x3EE
  {  3733,   8,  10,    1,  -10 },   // 0x3EF
  {  3747,   8,  10,    1,   -8 },   // 0x3F0
  {  3757,   8,   9,    1,   -8 },   // 0x3F1
  {  3769,   7,   9,    1,   -8 },   // 0x3F2
  {  3778,   5,   5,   -2,  -12 },   // 0x3F3
  {  3794,  11,  13,    1,  -11 },   // 0x3F4
  {  3818,   7,   9,    1,   -8 },   // 0x3F5
  {  3827,   7,   9,    1,   -8 },   // 0x3F6
  {  3836,   9,  11,    1,  -11 },   // 0x3F7
  {  3860,   9,  10,    0,  -11 },   // 0x3F8
  {  3890,  10,  12,    1,  -11 },   // 0x3F9
  {  3914,  15,  16,    1,  -11 },   // 0x3FA
  {  3938,  10,  11,    1,   -7 },   // 0x3FB
  {  3960,   8,   9,    1,   -8 },   // 0x3FC
  {  3972,  10,  12,    1,  -11 },   // 0x3FD
  {  3996,  10,  12,    1,  -11 },   // 0x3FE
  {  4020,  10,  12,    1,  -11 },   // 0x3FF
  {  4044,   0,   0,    0,    0 }
};

/* This struct will be read by the CPU only */
static const nema_font_range_t DejaVuSerif8pt1b_ranges[] = {
  {0x00000020, 0x0000007e, DejaVuSerif8pt1bGlyphs0},
  {0x00000370, 0x000003ff, DejaVuSerif8pt1bGlyphs1},
  {0, 0, NULL}
};

/* This struct will be read by the CPU only */
nema_font_t DejaVuSerif8pt1b = {
  {
    .base_virt = (void *) DejaVuSerif8pt1bBitmaps,
    .base_phys = (uintptr_t) DejaVuSerif8pt1bBitmaps,
    .size      = (int) sizeof(DejaVuSerif8pt1bBitmaps)
  },
  DejaVuSerif8pt1b_ranges,
  (int)sizeof(DejaVuSerif8pt1bBitmaps),
  DejaVuSerif8pt1bBitmaps,
  0,
  5, 19, 16, 1
};
#endif // DEJAVUSERIF8PT1B_C

