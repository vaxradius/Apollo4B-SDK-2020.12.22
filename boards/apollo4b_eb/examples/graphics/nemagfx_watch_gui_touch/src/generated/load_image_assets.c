//*****************************************************************************
//
//! @file load_image_assets.c
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

#include "nema_core.h"

img_obj_t _watch_flower_390x390_tsc4;
img_obj_t _watch_huawei_390x390_tsc4;
img_obj_t _hour_rgba;
img_obj_t _minute_rgba;
img_obj_t _second_rgba;

img_obj_t *image_assets[5];

#include "assets/images/watch_flower_390x390_tsc4.h"
#include "assets/images/watch_huawei_390x390_tsc4.h"
#include "assets/images/hour_rgba.h"
#include "assets/images/minute_rgba.h"
#include "assets/images/second_rgba.h"

void load_image_assets(void)
{
    _watch_flower_390x390_tsc4.bo = nema_buffer_create(sizeof(watch_flower_390x390_tsc4));
    nema_buffer_map(&_watch_flower_390x390_tsc4.bo);
    nema_memcpy(_watch_flower_390x390_tsc4.bo.base_virt, watch_flower_390x390_tsc4, sizeof(watch_flower_390x390_tsc4));
    _watch_flower_390x390_tsc4.w = 456;
    _watch_flower_390x390_tsc4.h = 456;
    _watch_flower_390x390_tsc4.format = NEMA_TSC4;
    _watch_flower_390x390_tsc4.stride = -1;
    _watch_flower_390x390_tsc4.sampling_mode = 1;
    _watch_flower_390x390_tsc4.color = 0xff000000;
    image_assets[0] = &_watch_flower_390x390_tsc4;

    _watch_huawei_390x390_tsc4.bo = nema_buffer_create(sizeof(watch_huawei_390x390_tsc4));
    nema_buffer_map(&_watch_huawei_390x390_tsc4.bo);
    nema_memcpy(_watch_huawei_390x390_tsc4.bo.base_virt, watch_huawei_390x390_tsc4, sizeof(watch_huawei_390x390_tsc4));
    _watch_huawei_390x390_tsc4.w = 456;
    _watch_huawei_390x390_tsc4.h = 456;
    _watch_huawei_390x390_tsc4.format = NEMA_TSC4;
    _watch_huawei_390x390_tsc4.stride = -1;
    _watch_huawei_390x390_tsc4.sampling_mode = 1;
    _watch_huawei_390x390_tsc4.color = 0xff000000;
    image_assets[1] = &_watch_huawei_390x390_tsc4;

    _hour_rgba.bo = nema_buffer_create(sizeof(hour_rgba));
    nema_buffer_map(&_hour_rgba.bo);
    nema_memcpy(_hour_rgba.bo.base_virt, hour_rgba, sizeof(hour_rgba));
    _hour_rgba.w = 22;
    _hour_rgba.h = 104;
    _hour_rgba.format = NEMA_RGBA8888;
    _hour_rgba.stride = -1;
    _hour_rgba.sampling_mode = 1;
    _hour_rgba.color = 0xff000000;
    image_assets[2] = &_hour_rgba;

    _minute_rgba.bo = nema_buffer_create(sizeof(minute_rgba));
    nema_buffer_map(&_minute_rgba.bo);
    nema_memcpy(_minute_rgba.bo.base_virt, minute_rgba, sizeof(minute_rgba));
    _minute_rgba.w = 28;
    _minute_rgba.h = 153;
    _minute_rgba.format = NEMA_RGBA8888;
    _minute_rgba.stride = -1;
    _minute_rgba.sampling_mode = 1;
    _minute_rgba.color = 0xff000000;
    image_assets[3] = &_minute_rgba;

    _second_rgba.bo = nema_buffer_create(sizeof(second_rgba));
    nema_buffer_map(&_second_rgba.bo);
    nema_memcpy(_second_rgba.bo.base_virt, second_rgba, sizeof(second_rgba));
    _second_rgba.w = 15;
    _second_rgba.h = 150;
    _second_rgba.format = NEMA_RGBA8888;
    _second_rgba.stride = -1;
    _second_rgba.sampling_mode = 1;
    _second_rgba.color = 0xff000000;
    image_assets[4] = &_second_rgba;

}
