//*****************************************************************************
//
//! @file gitem_tree.c
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

#include "gitem.h"
#define NUM_GROUPS  1
#define NUM_POPUPS  0
#define TIMER_ANIM_PERIOD  16

int NUM_SCREENS[NUM_GROUPS] = {2};
int cur_screen_group = 0;
int cur_screen_node  = 0;

//Forward declaration of gitems and tree nodes
//---------------------------------
tree_node_t node_1Screen1;
gitem_t _1Screen1;
    tree_node_t node_26Watch_Face4;
    gitem_t _26Watch_Face4;
        tree_node_t node_27Watch_Face_Hand9;
        gitem_t _27Watch_Face_Hand9;
        tree_node_t node_28Watch_Face_Hand10;
        gitem_t _28Watch_Face_Hand10;
        tree_node_t node_29Watch_Face_Hand11;
        gitem_t _29Watch_Face_Hand11;
    tree_node_t node_18Icon_Button2;
    gitem_t _18Icon_Button2;
        tree_node_t node_19Image2;
        gitem_t _19Image2;
tree_node_t node_17Screen2;
gitem_t _17Screen2;
    tree_node_t node_20Icon_Button3;
    gitem_t _20Icon_Button3;
        tree_node_t node_21Image3;
        gitem_t _21Image3;

//---------------------------------

gitem_t _1Screen1 = {0, 0, 0, 0, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 1, 0xff000000, 0xff000000, 0xff4c4c4c, 0xff4c4c4c, 0, 0, 0, 0, 456, 456, 456, 456, 0, 0, 1, -1, -1, 1, 0, 0, 0, GITEM_SCREEN, 0, 0, 0, 0, 0, 1 };
gitem_t _26Watch_Face4 = {0, 0, 0, 0, 0.000, 0.000, 0.000, 24.000, 0.000, 360.000, 0.000, 26, 0x00ffffff, 0x00ffffff, 0x00000000, 0x00000000, 0, 0, 0, 0, 455, 455, 456, 456, 0, 0, -1, -1, -1, 1, 50, 50, 0, GITEM_WATCH_FACE, 0, 0, 0, 0, 0, 0 };
gitem_t _27Watch_Face_Hand9 = {0, 0, 0, 0, 0.000, 0.000, 0.000, 100.000, 0.000, 360.000, 0.000, 27, 0xff808080, 0xff808080, 0xff000000, 0xff000000, 50, 50, 50, 50, 45, 45, 99, 99, 0, 0, 2, -1, -1, 1, 23, 86, 0, GITEM_WATCH_FACE_HAND, 0, 0, 0, 0, 0, 1 };
gitem_t _28Watch_Face_Hand10 = {0, 0, 0, 0, 0.000, 0.000, 0.000, 100.000, 0.000, 360.000, 0.000, 28, 0xff808080, 0xff808080, 0xff000000, 0xff000000, 50, 50, 50, 50, 33, 33, 148, 148, 0, 0, 3, -1, -1, 1, 17, 133, 0, GITEM_WATCH_FACE_HAND, 0, 0, 0, 0, 0, 1 };
gitem_t _29Watch_Face_Hand11 = {0, 0, 0, 0, 0.000, 0.000, 0.000, 100.000, 0.000, 360.000, 0.000, 29, 0xff808080, 0xff808080, 0xff000000, 0xff000000, 50, 50, 50, 50, 21, 21, 256, 256, 0, 0, 4, -1, -1, 1, 12, 172, 0, GITEM_WATCH_FACE_HAND, 0, 0, 0, 0, 0, 1 };
gitem_t _18Icon_Button2 = {0, 0, 0, 0, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 18, 0x00969600, 0x00969600, 0xffb5b54c, 0xffb5b54c, 0, 0, 0, 0, 455, 455, 455, 455, 0, 0, -1, -1, -1, 1, 0, 0, 262, GITEM_ICON_BUTTON, 0, 0, 1, 0, 0, 0 };
gitem_t _19Image2 = {0, 0, 0, 0, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 19, 0x00000000, 0x00000000, 0xff000000, 0xff000000, 144, 144, 141, 141, 24, 24, 24, 24, 0, 0, -1, -1, -1, 1, 0, 0, 0, GITEM_IMG, 0, 0, 0, 0, 0, 1 };
gitem_t _17Screen2 = {0, 0, 0, 0, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 17, 0xff000000, 0xff000000, 0xff4c4c4c, 0xff4c4c4c, 0, 0, 0, 0, 456, 456, 456, 456, 0, 0, 0, -1, -1, 1, 0, 0, 0, GITEM_SCREEN, 0, 0, 0, 0, 0, 1 };
gitem_t _20Icon_Button3 = {0, 0, 0, 0, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 20, 0x00969600, 0x00969600, 0xffb5b54c, 0xffb5b54c, 1, 1, 0, 0, 455, 455, 455, 455, 0, 0, -1, -1, -1, 1, 0, 0, 262, GITEM_ICON_BUTTON, 0, 0, 1, 0, 0, 0 };
gitem_t _21Image3 = {0, 0, 0, 0, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 21, 0x00000000, 0x00000000, 0xff000000, 0xff000000, 235, 235, 131, 131, 24, 24, 24, 24, 0, 0, -1, -1, -1, 1, 0, 0, 0, GITEM_IMG, 0, 0, 0, 0, 0, 1 };

//---------------------------------


//Build tree nodes
//---------------------------------
tree_node_t node_1Screen1 = { &_1Screen1, 0, &node_26Watch_Face4, 0};
    tree_node_t node_26Watch_Face4 = { &_26Watch_Face4, &node_1Screen1, &node_27Watch_Face_Hand9, &node_18Icon_Button2};
        tree_node_t node_27Watch_Face_Hand9 = { &_27Watch_Face_Hand9, &node_26Watch_Face4, 0, &node_28Watch_Face_Hand10};
        tree_node_t node_28Watch_Face_Hand10 = { &_28Watch_Face_Hand10, &node_26Watch_Face4, 0, &node_29Watch_Face_Hand11};
        tree_node_t node_29Watch_Face_Hand11 = { &_29Watch_Face_Hand11, &node_26Watch_Face4, 0, 0};
    tree_node_t node_18Icon_Button2 = { &_18Icon_Button2, &node_1Screen1, &node_19Image2, 0};
        tree_node_t node_19Image2 = { &_19Image2, &node_18Icon_Button2, 0, 0};

//Build tree nodes
//---------------------------------
tree_node_t node_17Screen2 = { &_17Screen2, 0, &node_20Icon_Button3, 0};
    tree_node_t node_20Icon_Button3 = { &_20Icon_Button3, &node_17Screen2, &node_21Image3, 0};
        tree_node_t node_21Image3 = { &_21Image3, &node_20Icon_Button3, 0, 0};

//---------------------------------


tree_node_t *group0_tree_nodes[] =
{
    &node_1Screen1,
    &node_17Screen2,
};
tree_node_t **group_tree_nodes[NUM_GROUPS] = {group0_tree_nodes};

tree_node_t **screen_tree_nodes = group0_tree_nodes;

nema_transition_t group_effect[1] = {NEMA_TRANS_LINEAR_H};

uint8_t group_layout[1] = {0};


//---------------------------------


tree_node_t *popup_tree_nodes[] = {
0};

//---------------------------------

