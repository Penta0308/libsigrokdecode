##
## This file is part of the libsigrokdecode project.
##
## Copyright (C) 2011-2014 Uwe Hermann <uwe@hermann-uwe.de>
##
## This program is free software; you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 2 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program; if not, see <http://www.gnu.org/licenses/>.
##

import sigrokdecode as srd
from common.srdhelper import bitpack
from math import floor, ceil

'''
OUTPUT_PYTHON format:

Packet:
[<ptype>, <rxtx>, <pdata>]

This is the list of <ptype>s and their respective <pdata> values:
 - 'STARTBIT': The data is the (integer) value of the start bit (0/1).
 - 'DATA': This is always a tuple containing two items:
   - 1st item: the (integer) value of the UART data. Valid values
     range from 0 to 511 (as the data can be up to 9 bits in size).
   - 2nd item: the list of individual data bits and their ss/es numbers.
 - 'PARITYBIT': The data is the (integer) value of the parity bit (0/1).
 - 'STOPBIT': The data is the (integer) value of the stop bit (0 or 1).
 - 'INVALID STARTBIT': The data is the (integer) value of the start bit (0/1).
 - 'INVALID STOPBIT': The data is the (integer) value of the stop bit (0/1).
 - 'PARITY ERROR': The data is a tuple with two entries. The first one is
   the expected parity value, the second is the actual parity value.
 - 'BREAK': The data is always 0.
 - 'FRAME': The data is always a tuple containing two items: The (integer)
   value of the UART data, and a boolean which reflects the validity of the
   UART frame.
 - 'IDLE': The data is always 0.

The <rxtx> field is 0 for RX packets, 1 for TX packets.
'''

# Used for differentiating between the two data directions.
RX = 0
TX = 1

class SamplerateError(Exception):
    pass

class ChannelError(Exception):
    pass

class Ann:
    RX_DATA, TX_DATA, RX_START, TX_START, RX_STOP, TX_STOP, RX_WARN, TX_WARN, \
    RX_DATA_BIT, TX_DATA_BIT, RX_BREAK, TX_BREAK, RX_PACKET, TX_PACKET = \
    range(14)

class Bin:
    RX, TX, RXTX = range(3)

class Decoder(srd.Decoder):
    api_version = 3
    id = 'emount'
    name = 'EMount'
    longname = 'Sony E mount decoder'
    desc = 'Asynchronous, serial bus.'
    license = 'gplv2+'
    inputs = ['logic']
    outputs = ['emount']
    tags = ['Embedded/industrial']
    channels = (
        {'id': 'rx', 'name': 'RX', 'desc': 'UART receive line'},
        {'id': 'tx', 'name': 'TX', 'desc': 'UART transmit line'},
        {'id': 'rxe', 'name': 'RXE', 'desc': 'UART receive active line'},
        {'id': 'txe', 'name': 'TXE', 'desc': 'UART transmit active line'},
    )
    optional_channels = tuple()
    options = (
        {'id': 'baudrate', 'desc': 'Baud rate', 'default': 750000},
        {'id': 'sample_point', 'desc': 'Sample point (%)', 'default': 50},
    )
    annotations = (
        ('rx-data', 'RX data'),
        ('tx-data', 'TX data'),
        ('rx-start', 'RX start bits'),
        ('tx-start', 'TX start bits'),
        ('rx-stop', 'RX stop bits'),
        ('tx-stop', 'TX stop bits'),
        ('rx-warnings', 'RX warnings'),
        ('tx-warnings', 'TX warnings'),
        ('rx-data-bits', 'RX data bits'),
        ('tx-data-bits', 'TX data bits'),
        ('rx-break', 'RX break'),
        ('tx-break', 'TX break'),
        ('rx-packet', 'RX packet'),
        ('tx-packet', 'TX packet'),
    )
    annotation_rows = (
        ('rx-data-bits', 'RX bits', (Ann.RX_DATA_BIT,)),
        ('rx-data-vals', 'RX data', (Ann.RX_DATA, Ann.RX_START, Ann.RX_STOP)),
        ('rx-warnings', 'RX warnings', (Ann.RX_WARN,)),
        ('rx-breaks', 'RX breaks', (Ann.RX_BREAK,)),
        ('rx-packets', 'RX packets', (Ann.RX_PACKET,)),
        ('tx-data-bits', 'TX bits', (Ann.TX_DATA_BIT,)),
        ('tx-data-vals', 'TX data', (Ann.TX_DATA, Ann.TX_START, Ann.TX_STOP)),
        ('tx-warnings', 'TX warnings', (Ann.TX_WARN,)),
        ('tx-breaks', 'TX breaks', (Ann.TX_BREAK,)),
        ('tx-packets', 'TX packets', (Ann.TX_PACKET,)),
    )
    binary = (
        ('rx', 'RX dump'),
        ('tx', 'TX dump'),
        ('rxtx', 'RX/TX dump'),
    )
    idle_state = ['WAIT FOR START BIT', 'WAIT FOR START BIT']

    def putx(self, rxtx, data):
        s, halfbit = self.startsample[rxtx], self.bit_width / 2.0
        self.put(s - floor(halfbit), self.samplenum + ceil(halfbit), self.out_ann, data)

    def putx_packet(self, rxtx, data):
        s, halfbit = self.ss_packet[rxtx], self.bit_width / 2.0
        self.put(s - floor(halfbit), self.samplenum + ceil(halfbit), self.out_ann, data)

    def putpx(self, rxtx, data):
        s, halfbit = self.startsample[rxtx], self.bit_width / 2.0
        self.put(s - floor(halfbit), self.samplenum + ceil(halfbit), self.out_python, data)

    def putg(self, data):
        s, halfbit = self.samplenum, self.bit_width / 2.0
        self.put(s - floor(halfbit), s + ceil(halfbit), self.out_ann, data)

    def putp(self, data):
        s, halfbit = self.samplenum, self.bit_width / 2.0
        self.put(s - floor(halfbit), s + ceil(halfbit), self.out_python, data)

    def putgse(self, ss, es, data):
        self.put(ss, es, self.out_ann, data)

    def putpse(self, ss, es, data):
        self.put(ss, es, self.out_python, data)

    def putbin(self, rxtx, data):
        s, halfbit = self.startsample[rxtx], self.bit_width / 2.0
        self.put(s - floor(halfbit), self.samplenum + ceil(halfbit), self.out_binary, data)

    def __init__(self):
        self.reset()

    def reset(self):
        self.samplerate = None
        self.frame_start = [-1, -1]
        self.frame_valid = [None, None]
        self.startbit = [-1, -1]
        self.cur_data_bit = [0, 0]
        self.datavalue = [0, 0]
        self.stopbit1 = [-1, -1]
        self.startsample = [-1, -1]
        self.state = ['WAIT FOR START BIT', 'WAIT FOR START BIT']
        self.databits = [[], []]
        self.break_start = [None, None]
        self.packet_cache = [[], []]
        self.ss_packet, self.es_packet = [None, None], [None, None]
        self.idle_start = [None, None]

    def start(self):
        self.out_python = self.register(srd.OUTPUT_PYTHON)
        self.out_binary = self.register(srd.OUTPUT_BINARY)
        self.out_ann = self.register(srd.OUTPUT_ANN)
        self.bw = (8 + 7) // 8

    def metadata(self, key, value):
        if key == srd.SRD_CONF_SAMPLERATE:
            self.samplerate = value
            # The width of one UART bit in number of samples.
            self.bit_width = float(self.samplerate) / float(self.options['baudrate'])

    def get_sample_point(self, rxtx, bitnum):
        # Determine absolute sample number of a bit slot's sample point.
        # Counts for UART bits start from 0 (0 = start bit, 1..x = data,
        # x+1 = parity bit (if used) or the first stop bit, and so on).
        # Accept a position in the range of 1-99% of the full bit width.
        # Assume 50% for invalid input specs for backwards compatibility.
        perc = self.options['sample_point'] or 50
        if not perc or perc not in range(1, 100):
            perc = 50
        perc /= 100.0
        bitpos = (self.bit_width - 1) * perc
        bitpos += self.frame_start[rxtx]
        bitpos += bitnum * self.bit_width
        return bitpos

    def wait_for_start_bit(self, rxtx, signal):
        # Save the sample number where the start bit begins.
        self.frame_start[rxtx] = self.samplenum
        self.frame_valid[rxtx] = True

        self.state[rxtx] = 'GET START BIT'

    def get_start_bit(self, rxtx, signal):
        self.startbit[rxtx] = signal

        # The startbit must be 0. If not, we report an error and wait
        # for the next start bit (assuming this one was spurious).
        if self.startbit[rxtx] != 0:
            self.putp(['INVALID STARTBIT', rxtx, self.startbit[rxtx]])
            self.putg([Ann.RX_WARN + rxtx, ['Frame error', 'Frame err', 'FE']])
            self.frame_valid[rxtx] = False
            es = self.samplenum + ceil(self.bit_width / 2.0)
            self.putpse(self.frame_start[rxtx], es, ['FRAME', rxtx,
                (self.datavalue[rxtx], self.frame_valid[rxtx])])
            self.state[rxtx] = 'WAIT FOR START BIT'
            return

        self.cur_data_bit[rxtx] = 0
        self.datavalue[rxtx] = 0
        self.startsample[rxtx] = -1

        self.putp(['STARTBIT', rxtx, self.startbit[rxtx]])
        self.putg([Ann.RX_START + rxtx, ['Start bit', 'Start', 'S']])

        self.state[rxtx] = 'GET DATA BITS'

    def handle_packet(self, rxtx):
        if self.ss_packet[rxtx] is not None:
            self.packet_cache[rxtx].append(self.datavalue[rxtx])

        if (self.datavalue[rxtx] == 0xF0) and (self.ss_packet[rxtx] is None):
            self.ss_packet[rxtx] = self.startsample[rxtx]
            self.packet_cache[rxtx] = [self.datavalue[rxtx],]

        if len(self.packet_cache[rxtx]) >= 3:
            pdlen = (self.packet_cache[rxtx][2] << 8) + self.packet_cache[rxtx][1]
            if (pdlen == len(self.packet_cache[rxtx])):
                if self.packet_cache[rxtx][-1] == 0x55:
                    self.es_packet[rxtx] = self.samplenum

                    pdclass = self.packet_cache[rxtx][3]
                    pdseqn = self.packet_cache[rxtx][4]
                    pdtype = self.packet_cache[rxtx][5]
                    pdraw = self.packet_cache[rxtx][6:-3]

                    s = "{:01X} {:02X} {:02X} ".format(pdclass, pdseqn, pdtype)
                    for b in pdraw:
                        s += self.format_value(b)
                        s += '.'
                    if s[-1] == '.':
                        s = s[:-1] # Drop trailing space.
                    self.putx_packet(rxtx, [Ann.RX_PACKET + rxtx, [s]])
                else:
                    # TODO: INVALID PACKET PROCESSING
                    pass
                self.packet_cache[rxtx] = []
                self.ss_packet[rxtx] = None

    def get_data_bits(self, rxtx, signal):
        # Save the sample number of the middle of the first data bit.
        if self.startsample[rxtx] == -1:
            self.startsample[rxtx] = self.samplenum

        self.putg([Ann.RX_DATA_BIT + rxtx, ['%d' % signal]])

        # Store individual data bits and their start/end samplenumbers.
        s, halfbit = self.samplenum, int(self.bit_width / 2)
        self.databits[rxtx].append([signal, s - halfbit, s + halfbit])

        # Return here, unless we already received all data bits.
        self.cur_data_bit[rxtx] += 1
        if self.cur_data_bit[rxtx] < 8:
            return

        # Convert accumulated data bits to a data value.
        bits = [b[0] for b in self.databits[rxtx]]
        self.datavalue[rxtx] = bitpack(bits)
        self.putpx(rxtx, ['DATA', rxtx,
            (self.datavalue[rxtx], self.databits[rxtx])])

        b = self.datavalue[rxtx]
        formatted = self.format_value(b)
        if formatted is not None:
            self.putx(rxtx, [rxtx, [formatted]])

        bdata = b.to_bytes(self.bw, byteorder='big')
        self.putbin(rxtx, [rxtx, bdata])
        self.putbin(rxtx, [2, bdata])

        self.handle_packet(rxtx)

        self.databits[rxtx] = []

        # Advance to either reception of the parity bit, or reception of
        # the STOP bits if parity is not applicable.
        self.state[rxtx] = 'GET STOP BITS'

    def format_value(self, v):
        # Format value 'v' according to configured options.
        # Reflects the user selected kind of representation, as well as
        # the number of data bits in the UART frames.

        # Padding with leading zeroes for hex/oct/bin formats, but
        # without a prefix for density -- since the format is user
        # specified, there is no ambiguity.
        digits = (8 + 4 - 1) // 4
        fmtchar = "X"
        if fmtchar is not None:
            fmt = "{{:0{:d}{:s}}}".format(digits, fmtchar)
            return fmt.format(v)

        return None

    # TODO: Currently only supports 1 stop bit.
    def get_stop_bits(self, rxtx, signal):
        self.stopbit1[rxtx] = signal

        # Stop bits must be 1. If not, we report an error.
        if self.stopbit1[rxtx] != 1:
            self.putp(['INVALID STOPBIT', rxtx, self.stopbit1[rxtx]])
            self.putg([Ann.RX_WARN + rxtx, ['Frame error', 'Frame err', 'FE']])
            self.frame_valid[rxtx] = False

        self.putp(['STOPBIT', rxtx, self.stopbit1[rxtx]])
        self.putg([Ann.RX_STOP + rxtx, ['Stop bit', 'Stop', 'T']])

        # Pass the complete UART frame to upper layers.
        es = self.samplenum + ceil(self.bit_width / 2.0)
        self.putpse(self.frame_start[rxtx], es, ['FRAME', rxtx,
            (self.datavalue[rxtx], self.frame_valid[rxtx])])

        self.state[rxtx] = 'WAIT FOR START BIT'
        self.idle_start[rxtx] = self.frame_start[rxtx] + self.frame_len_sample_count

    def handle_break(self, rxtx):
        self.putpse(self.frame_start[rxtx], self.samplenum,
                ['BREAK', rxtx, 0])
        self.putgse(self.frame_start[rxtx], self.samplenum,
                [Ann.RX_BREAK + rxtx, ['Break condition', 'Break', 'Brk', 'B']])
        self.state[rxtx] = 'WAIT FOR START BIT'

    def get_wait_cond(self, rxtx):
        # Return condititions that are suitable for Decoder.wait(). Those
        # conditions either match the falling edge of the START bit, or
        # the sample point of the next bit time.
        state = self.state[rxtx]
        if state == 'WAIT FOR START BIT':
            return {rxtx: 'f', rxtx + 2: 'h'}
        if state == 'GET START BIT':
            bitnum = 0
        elif state == 'GET DATA BITS':
            bitnum = 1 + self.cur_data_bit[rxtx]
        elif state == 'GET STOP BITS':
            bitnum = 1 + 8
        want_num = ceil(self.get_sample_point(rxtx, bitnum))
        return {'skip': want_num - self.samplenum}

    def get_idle_cond(self, rxtx):
        # Return a condition that corresponds to the (expected) end of
        # the next frame, assuming that it will be an "idle frame"
        # (constant high input level for the frame's length).
        if self.idle_start[rxtx] is None:
            return None
        end_of_frame = self.idle_start[rxtx] + self.frame_len_sample_count
        if end_of_frame < self.samplenum:
            return None
        return {'skip': end_of_frame - self.samplenum}


    def inspect_en(self, rxtx, signal, signale):
        if not signale:
            self.packet_cache[rxtx] = []
            self.ss_packet[rxtx] = None

    def inspect_sample(self, rxtx, signal, signale):

        # Inspect a sample returned by .wait() for the specified UART line.

        state = self.state[rxtx]
        if state == 'WAIT FOR START BIT':
            self.wait_for_start_bit(rxtx, signal)
        elif state == 'GET START BIT':
            self.get_start_bit(rxtx, signal)
        elif state == 'GET DATA BITS':
            self.get_data_bits(rxtx, signal)
        elif state == 'GET STOP BITS':
            self.get_stop_bits(rxtx, signal)

    def inspect_edge(self, rxtx, signal, signale):

        # Inspect edges, independently from traffic, to detect break conditions.
        if not signal:
            # Signal went low. Start another interval.
            self.break_start[rxtx] = self.samplenum
            return
        # Signal went high. Was there an extended period with low signal?
        if self.break_start[rxtx] is None:
            return
        diff = self.samplenum - self.break_start[rxtx]
        if diff >= self.break_min_sample_count:
            self.handle_break(rxtx)
        self.break_start[rxtx] = None

    def inspect_idle(self, rxtx, signal, signale):

        # Check each edge and each period of stable input (either level).
        # Can derive the "idle frame period has passed" condition.
        if not signal:
            # Low input, cease inspection.
            self.idle_start[rxtx] = None
            return
        # High input, either just reached, or still stable.
        if self.idle_start[rxtx] is None:
            self.idle_start[rxtx] = self.samplenum
        diff = self.samplenum - self.idle_start[rxtx]
        if diff < self.frame_len_sample_count:
            return
        ss, es = self.idle_start[rxtx], self.samplenum
        self.putpse(ss, es, ['IDLE', rxtx, 0])
        self.idle_start[rxtx] = self.samplenum

    def decode(self):
        if not self.samplerate:
            raise SamplerateError('Cannot decode without samplerate.')

        has_pin = [self.has_channel(ch) for ch in (RX, TX)]
        if not True in has_pin:
            raise ChannelError('Need at least one of TX or RX pins.')

        opt = self.options
        cond_data_idx = [None] * len(has_pin)

        # Determine the number of samples for a complete frame's time span.
        # A period of low signal (at least) that long is a break condition.
        frame_samples = 1 # START
        frame_samples += 8 # DATA
        frame_samples += 1 # STOP
        frame_samples *= self.bit_width
        self.frame_len_sample_count = ceil(frame_samples)
        self.break_min_sample_count = self.frame_len_sample_count
        cond_edge_idx = [None] * 2
        cond_idle_idx = [None] * 2
        cond_en_idx = [None] * 2

        while True:
            conds = []

            cond_data_idx[RX] = len(conds)
            conds.append(self.get_wait_cond(RX))
            cond_edge_idx[RX] = len(conds)
            conds.append({RX: 'e', RX + 2: 'h'})
            cond_idle_idx[RX] = None
            idle_cond = self.get_idle_cond(RX)
            if idle_cond:
                cond_idle_idx[RX] = len(conds)
                conds.append(idle_cond)

            cond_data_idx[TX] = len(conds)
            conds.append(self.get_wait_cond(TX))
            cond_edge_idx[TX] = len(conds)
            conds.append({TX: 'e', TX + 2: 'h'})
            cond_idle_idx[TX] = None
            idle_cond = self.get_idle_cond(TX)
            if idle_cond:
                cond_idle_idx[TX] = len(conds)
                conds.append(idle_cond)

            cond_en_idx[RX] = len(conds)
            conds.append({RX + 2: 'e'})
            cond_en_idx[TX] = len(conds)
            conds.append({TX + 2: 'e'})

            (rx, tx, rxe, txe) = self.wait(conds)
            if cond_en_idx[RX] is not None and self.matched[cond_en_idx[RX]]:
                self.inspect_en(RX, rx, rxe)
            if cond_en_idx[TX] is not None and self.matched[cond_en_idx[TX]]:
                self.inspect_en(TX, tx, txe)
            if cond_data_idx[RX] is not None and self.matched[cond_data_idx[RX]]:
                self.inspect_sample(RX, rx, rxe)
            if cond_edge_idx[RX] is not None and self.matched[cond_edge_idx[RX]]:
                self.inspect_edge(RX, rx, rxe)
                self.inspect_idle(RX, rx, rxe)
            if cond_idle_idx[RX] is not None and self.matched[cond_idle_idx[RX]]:
                self.inspect_idle(RX, rx, rxe)
            if cond_data_idx[TX] is not None and self.matched[cond_data_idx[TX]]:
                self.inspect_sample(TX, tx, txe)
            if cond_edge_idx[TX] is not None and self.matched[cond_edge_idx[TX]]:
                self.inspect_edge(TX, tx, txe)
                self.inspect_idle(TX, tx, txe)
            if cond_idle_idx[TX] is not None and self.matched[cond_idle_idx[TX]]:
                self.inspect_idle(TX, tx, txe)
