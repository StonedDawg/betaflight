/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "pg/pg.h"
#include "drivers/io_types.h"
#include "common/utils.h"

#define VRX_PINS 3

typedef struct vrxPinsConfig_s {
    ioTag_t ioTags[VRX_PINS];
    uint8_t inversion;
} vrxPinsConfig_t;

PG_DECLARE(vrxPinsConfig_t, vrxPinsConfig);

// Helpful macros

#define VRX_DIVERSITY_TOGGLE              vrxPinsToggle(0)
#define VRX_DIVERSITY_0                 vrxPinsSet(0, false)
#define VRX_DIVERSITY_1                  vrxPinsSet(0, true)

#define VRX_OSD_TOGGLE              vrxPinsToggle(1)
#define VRX_OSD_OFF                 vrxPinsSet(1, false)
#define VRX_OSD_ON                  vrxPinsSet(1, true)

#define VRX_LED_TOGGLE              vrxPinsToggle(2)
#define VRX_LED_OFF                 vrxPinsSet(2, false)
#define VRX_LED_ON                  vrxPinsSet(2, true)

void vrxPinsInit(const vrxPinsConfig_t *vrxPinsConfig);
void vrxPinsToggle(int pin);
void vrxPinsSet(int pin, bool state);

