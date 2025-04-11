// Arcflash serial port implementation
// Copyright (C) 2025 Google LLC

// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

#ifndef ARCFLASH_SERIAL_H
#define ARCFLASH_SERIAL_H

#include <stdint.h>

// Serial port emulation addresses
#define SERIAL_TXD_LOW  0x3FFFFF0
#define SERIAL_TXD_HIGH 0x3FFFFF4
#define SERIAL_RXD      0x3FFFFF8

struct SerialPort {
    uint64_t last_bit_time;     // Time of last bit transition in CPU cycles
    uint32_t tx_shift_reg;      // Shift register for transmit
    uint32_t tx_bits_remaining;
    uint32_t tx_state;          // 0=idle, 1=sending
    uint32_t tx_last_bit_value; // Value last set on TXD

    uint32_t rx_shift_reg;      // Shift register for receive
    uint32_t rx_bits_remaining;
    uint32_t rx_next_char;      // Next character to send
    uint64_t rx_next_bit_time;  // Time to send next bit
    uint64_t last_char_start_time;
};

// Initialize serial port state
void arcflash_serial_init(struct SerialPort *sp);

// Handle memory reads at serial port addresses
uint32_t arcflash_serial_read(struct SerialPort *sp, uint32_t addr, uint64_t cycle_count);

#endif // ARCFLASH_SERIAL_H