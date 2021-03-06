import string
import datetime


memory_header_string = '''\
//*****************************************************************************
//
// file am_memory_map.h
//
// brief Memory map include file.
//
// This file is generated by "memory_map.py".
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) {copyrightyear}, Ambiq Micro
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
// This is part of the AmbiqSuite Development Package.
//
//*****************************************************************************

#ifndef AM_MEMORY_MAP_H
#define AM_MEMORY_MAP_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// Memory block locations.
//
//*****************************************************************************
${memory_locations}

//*****************************************************************************
//
// Memory block sizes (in bytes)
//
//*****************************************************************************
${memory_sizes}

#ifdef __cplusplus
}
#endif

#endif // AM_MEMORY_MAP_H
'''

currdatetime = datetime.datetime.now()
memory_header_template = string.Template(memory_header_string.format(copyrightyear=currdatetime.year))



memory_location_define = '#define AM_MEM_{name:36} ((void *) 0x{start:08X})'
memory_size_define = '#define AM_MEM_{name:36} {size}'


def generate(memory_sections):

    mapping = {
        'memory_locations': '\n'.join(write_memory_locations(memory_sections)),
        'memory_sizes': '\n'.join(write_memory_sizes(memory_sections)),
    }

    return memory_header_template.substitute(**mapping)


def write_memory_locations(memory_sections):
    section_names = sorted_sections(memory_sections)
    for n in section_names:
        mapping = {
            'name':  n,
            'start': memory_sections[n]['start'],
        }
        yield memory_location_define.format_map(mapping)


def write_memory_sizes(memory_sections):
    section_names = sorted_sections(memory_sections)
    for n in section_names:
        mapping = {
            'name':  n + '_SIZE',
            'size': memory_sections[n]['size'],
        }
        yield memory_size_define.format_map(mapping)


def sorted_sections(memory_sections):

    def start_address(section):
        return memory_sections[section]['start']

    return sorted(memory_sections.keys(), key=start_address)
