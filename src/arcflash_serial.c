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

#include "arcflash_serial.h"
#include <stdio.h>
#include <stdlib.h>

// The amount 'tsc' increments per cycle
// TODO move this into a header and make arm.c use it
#define cyc_i (1ull << 32)

// CPU speed in MHz, set in main.c
extern int speed_mhz;

// Enable very verbose debugging
#define DEBUG_SHOW_EVERY_BIT 0

// Serial port baud rate
static const uint64_t baud_rate = 10000;

// Helper function to handle bit shifting for serial transmission
static void handle_tx_bits(struct SerialPort *sp, uint64_t cycle_count, uint64_t cycles_per_bit, int bit_value) {
    // Abort any fake character we're sending *to* the host, because it's busy.
    sp->rx_bits_remaining = 0;

    // We should never get here unless we're in the middle of a character.
    if (sp->tx_state != 1) {
        return;
    }

    uint64_t elapsed = cycle_count - sp->last_bit_time;
    uint32_t bits = elapsed / cycles_per_bit;
    // printf("handle_tx_bits: us elapsed %.1f\n", (double)elapsed / speed_mhz / cyc_i);
    // printf("handle_tx_bits: cycle_count=%llu, elapsed=%llu, bits=%u, bits_remaining=%u\n",
    //     cycle_count, elapsed, bits, sp->tx_bits_remaining);
    if (bits > sp->tx_bits_remaining + 5) {
        // printf("too slow, resetting\n");
        sp->tx_state = 0;
        return;
    }

    if (bits > 0) {
        // Shift in the appropriate bits
        for (int i = 0; i < bits; i++) {
            // printf("    shift %d; %d bits remaining\n", sp->tx_last_bit_value, sp->tx_bits_remaining);
            sp->tx_shift_reg >>= 1;
            if (sp->tx_last_bit_value) sp->tx_shift_reg |= 0x100;
            --sp->tx_bits_remaining;
            sp->last_bit_time += cycles_per_bit;
        }
    }

    if (sp->tx_bits_remaining == 1 && bit_value == 1) {
        // printf("Serial TX: got start of stop bit, going to assume this will work\n");
        --sp->tx_bits_remaining;
        sp->tx_shift_reg = (sp->tx_shift_reg >> 1) | 0x100;
    }

    if (sp->tx_bits_remaining == 0) {
        if ((sp->tx_shift_reg & 0x100) == 0) {
            // printf("Serial TX: framing error\n");
        }
        // Extract character from shift register
        uint8_t ch = sp->tx_shift_reg & 0xFF;
        printf("Serial TX: %c (0x%02X)\n", (ch >= 32 && ch <= 127) ? ch : '?', ch);
        sp->tx_state = 0;
    }
    sp->tx_last_bit_value = bit_value;
}

// Initialize serial port state
void arcflash_serial_init(struct SerialPort *sp) {
    sp->last_bit_time = 0;
    sp->tx_shift_reg = 0;
    sp->tx_bits_remaining = 0;
    sp->tx_state = 0;
    sp->rx_shift_reg = 0;
    sp->rx_bits_remaining = 0;
    sp->rx_next_char = 0;
    sp->rx_next_bit_time = 0;
}

// Handle memory reads at serial port addresses
uint32_t arcflash_serial_read(struct SerialPort *sp, uint32_t addr, uint64_t cycle_count) {
    // Calculate time for one bit at 10000 baud
    // cycles_per_bit = CPU_MHz * 1000000 / baud_rate
    const uint64_t cycles_per_bit = ((uint64_t)speed_mhz * 1000000 * cyc_i) / baud_rate;

    if (addr == SERIAL_TXD_LOW) {
        // printf("Serial TXD_LOW\n");
        // Start bit or data bit 0
        if (sp->tx_state == 0) {
            // Start new transmission
            sp->tx_state = 1;
            sp->tx_bits_remaining = 9; // 8 data + Stop
            sp->tx_shift_reg = 0;
            // Set last bit time halfway through the first bit, so we sample at the right point.
            sp->last_bit_time = cycle_count + cycles_per_bit / 2;
            // printf("New transmission starting at cycle %llu\n", cycle_count);
        } else {
            handle_tx_bits(sp, cycle_count, cycles_per_bit, 0);
        }
        return 0;
    }

    if (addr == SERIAL_TXD_HIGH) {
        // printf("Serial TXD_HIGH\n");
        handle_tx_bits(sp, cycle_count, cycles_per_bit, 1);
        return 0;
    }

    if (addr == SERIAL_RXD) {
        if (DEBUG_SHOW_EVERY_BIT) {
            printf("Serial RXD at %llu (%.1f us into char)\n",
                cycle_count,
                ((double)cycle_count - (double)sp->last_char_start_time) / speed_mhz / cyc_i);
        }
        // Generate random received characters
        if (cycle_count >= sp->rx_next_bit_time) {
            if (sp->rx_bits_remaining == 0) {
                uint32_t ch = sp->rx_next_char;
                printf("Serial RXD: Finished receiving character: %c (0x%02X)\n", (ch >= 32 && ch <= 127) ? ch : '?', ch);
                // Start new character
                // Being charitable, we'll pretend the host sampled RIGHT after
                // the falling edge of the start bit.
                sp->last_char_start_time = cycle_count;
                // Right now the "next bit" is the start bit.
                sp->rx_next_bit_time = cycle_count;
                sp->rx_next_char = 32 + (rand() % 95); // ASCII printable chars
                if (DEBUG_SHOW_EVERY_BIT) printf("Serial RXD: starting new character %02X\n", sp->rx_next_char);
                sp->rx_bits_remaining = 10;
                // Start + data + stop + idle
                sp->rx_shift_reg = (0x600 | (sp->rx_next_char << 1));
            }

            // Output next bit
            uint32_t bit = sp->rx_shift_reg & 1;
            sp->rx_shift_reg >>= 1;
            sp->rx_bits_remaining--;

            // Schedule next bit
            sp->rx_next_bit_time += cycles_per_bit;

            if (DEBUG_SHOW_EVERY_BIT) printf("--> %d\n", bit);
            return bit;
        }

        // Return last bit state
        if (DEBUG_SHOW_EVERY_BIT) printf("    %d\n", sp->rx_shift_reg & 1);
        return (sp->rx_shift_reg & 1);
    }

    return 0;
}