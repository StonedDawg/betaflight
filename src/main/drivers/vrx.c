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

#include "platform.h"

#include "pg/pg_ids.h"

#include "drivers/io.h"
#include "io_impl.h"

#include "vrx.h"

PG_REGISTER_WITH_RESET_FN(vrxPinsConfig_t, vrxPinsConfig, PG_VRX_PINS_CONFIG, 0);

static IO_t vrxPins[VRX_PINS];
static uint8_t vrxPinsInversion = 0;

#ifndef VRX_DIVERSITY_SWITCH_PIN
#define vrxPins0_PIN NONE
#else
#define vrxPins0_PIN VRX_DIVERSITY_SWITCH_PIN
#endif

#ifndef VRX_OSD_SWITCH_PIN
#define vrxPins1_PIN NONE
#else
#define vrxPins1_PIN VRX_OSD_SWITCH_PIN
#endif

#ifndef VRX_LED_PIN
#define vrxPins2_PIN NONE
#else
#define vrxPins2_PIN VRX_LED_PIN
#endif

void pgResetFn_vrxPinsConfig(vrxPinsConfig_t *vrxPinsConfig)
{
    vrxPinsConfig->ioTags[0] = IO_TAG(vrxPins0_PIN);
    vrxPinsConfig->ioTags[1] = IO_TAG(vrxPins1_PIN);
    vrxPinsConfig->ioTags[2] = IO_TAG(vrxPins2_PIN);

    vrxPinsConfig->inversion = 0
#ifdef vrxPins0_INVERTED
    | BIT(0)
#endif
#ifdef vrxPins1_INVERTED
    | BIT(1)
#endif
#ifdef vrxPins2_INVERTED
    | BIT(2)
#endif
    ;
}

void vrxPinsInit(const vrxPinsConfig_t *vrxPinsConfig)
{
    vrxPinsInversion = vrxPinsConfig->inversion;
    for (int i = 0; i < VRX_PINS; i++) {
        if (vrxPinsConfig->ioTags[i]) {
            vrxPins[i] = IOGetByTag(vrxPinsConfig->ioTags[i]);
            IOInit(vrxPins[i], OWNER_VRX, RESOURCE_INDEX(i));
            IOConfigGPIO(vrxPins[i], IOCFG_OUT_PP);
        } else {
            vrxPins[i] = IO_NONE;
        }
    }

    VRX_DIVERSITY_0;
    VRX_OSD_OFF;
    VRX_LED_OFF;
}

void vrxPinsToggle(int pin)
{
    IOToggle(vrxPins[pin]);
}

void vrxPinsSet(int pin, bool on)
{
    const bool inverted = (1 << (pin)) & vrxPinsInversion;
    IOWrite(vrxPins[pin], on ? inverted : !inverted);
}
