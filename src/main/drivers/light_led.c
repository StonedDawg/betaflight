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

#include "light_led.h"



PG_REGISTER_WITH_RESET_FN(statusLedConfig_t, statusLedConfig, PG_STATUS_LED_CONFIG, 0);
PG_REGISTER_WITH_RESET_FN(vrxPinsConfig_t, vrxPinsConfig, PG_VRX_PINS_CONFIG, 0);
PG_REGISTER_WITH_RESET_FN(vrxBtnsConfig_t, vrxBtnsConfig, PG_VRX_BTNS_CONFIG, 0);

static IO_t vrxPins[VRX_PINS];
static IO_t vrxBtns[VRX_BTNS];
static uint8_t vrxPinsInversion = 0;
static uint8_t vrxBtnsInversion = 0;


static IO_t leds[STATUS_LED_NUMBER];
static uint8_t ledInversion = 0;

#ifndef VRX_DIVERSITY0_SWITCH_PIN
#define vrxPins0_PIN NONE
#else
#define vrxPins0_PIN VRX_DIVERSITY0_SWITCH_PIN
#endif

#ifndef VRX_DIVERSITY1_SWITCH_PIN
#define vrxPins1_PIN NONE
#else
#define vrxPins1_PIN VRX_DIVERSITY1_SWITCH_PIN
#endif

#ifndef VRX_OSD_SWITCH_PIN
#define vrxPins2_PIN NONE
#else
#define vrxPins2_PIN VRX_OSD_SWITCH_PIN
#endif

#ifndef VRX_LED0_PIN
#define vrxPins3_PIN NONE
#else
#define vrxPins3_PIN VRX_LED0_PIN
#endif

#ifndef VRX_LED1_PIN
#define vrxPins4_PIN NONE
#else
#define vrxPins4_PIN VRX_LED1_PIN
#endif

#ifndef VRX_BTN0_PIN
#define vrxBtns0_PIN NONE
#else
#define vrxBtns0_PIN VRX_BTN0_PIN
#endif

#ifndef VRX_BTN1_PIN
#define vrxBtns1_PIN NONE
#else
#define vrxBtns1_PIN VRX_BTN1_PIN
#endif

#ifndef VRX_BTN2_PIN
#define vrxBtns2_PIN NONE
#else
#define vrxBtns2_PIN VRX_BTN2_PIN
#endif

#ifndef LED0_PIN
#define LED0_PIN NONE
#endif

#ifndef LED1_PIN
#define LED1_PIN NONE
#endif

#ifndef LED2_PIN
#define LED2_PIN NONE
#endif

void pgResetFn_statusLedConfig(statusLedConfig_t *statusLedConfig)
{
    statusLedConfig->ioTags[0] = IO_TAG(LED0_PIN);
    statusLedConfig->ioTags[1] = IO_TAG(LED1_PIN);
    statusLedConfig->ioTags[2] = IO_TAG(LED2_PIN);

    statusLedConfig->inversion = 0
#ifdef LED0_INVERTED
    | BIT(0)
#endif
#ifdef LED1_INVERTED
    | BIT(1)
#endif
#ifdef LED2_INVERTED
    | BIT(2)
#endif
    ;
}
void pgResetFn_vrxPinsConfig(vrxPinsConfig_t *vrxPinsConfig)
{
    vrxPinsConfig->ioTags[0] = IO_TAG(vrxPins0_PIN);
    vrxPinsConfig->ioTags[1] = IO_TAG(vrxPins1_PIN);
    vrxPinsConfig->ioTags[2] = IO_TAG(vrxPins2_PIN);
    vrxPinsConfig->ioTags[3] = IO_TAG(vrxPins3_PIN);
    vrxPinsConfig->ioTags[4] = IO_TAG(vrxPins4_PIN);
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
#ifdef vrxPins3_INVERTED
    | BIT(3)
#endif
    ;
}

void pgResetFn_vrxBtnsConfig(vrxBtnsConfig_t *vrxBtnsConfig)
{
    vrxBtnsConfig->ioTags[0] = IO_TAG(vrxBtns0_PIN);
    vrxBtnsConfig->ioTags[1] = IO_TAG(vrxBtns1_PIN);
    vrxBtnsConfig->ioTags[2] = IO_TAG(vrxBtns2_PIN);

    vrxBtnsConfig->inversion = 0
#ifdef vrxBtns0_INVERTED
    | BIT(0)
#endif
#ifdef vrxBtns1_INVERTED
    | BIT(1)
#endif
#ifdef vrxBtns2_INVERTED
    | BIT(2)
#endif
    ;
}

void ledInit(const statusLedConfig_t *statusLedConfig)
{
    ledInversion = statusLedConfig->inversion;
    for (int i = 0; i < STATUS_LED_NUMBER; i++) {
        if (statusLedConfig->ioTags[i]) {
            leds[i] = IOGetByTag(statusLedConfig->ioTags[i]);
            IOInit(leds[i], OWNER_LED, RESOURCE_INDEX(i));
            IOConfigGPIO(leds[i], IOCFG_OUT_PP);
        } else {
            leds[i] = IO_NONE;
        }
    }

    LED0_OFF;
    LED1_OFF;
    LED2_OFF;
}

void ledToggle(int led)
{
    IOToggle(leds[led]);
}

void ledSet(int led, bool on)
{
    const bool inverted = (1 << (led)) & ledInversion;
    IOWrite(leds[led], on ? inverted : !inverted);
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
    VRX_LED0_OFF;
}
void vrxBtnsInit(const vrxBtnsConfig_t *vrxBtnsConfig)
{
    vrxBtnsInversion = vrxBtnsConfig->inversion;
    for (int i = 0; i < VRX_BTNS; i++) {
        if (vrxBtnsConfig->ioTags[i]) {
            vrxBtns[i] = IOGetByTag(vrxBtnsConfig->ioTags[i]);
            IOInit(vrxBtns[i], OWNER_VRX, RESOURCE_INDEX(i));
            IOConfigGPIO(vrxBtns[i], IOCFG_IPD);
        } else {
            vrxBtns[i] = IO_NONE;
        }
    }
}

void vrxPinsToggle(int pin)
{
    IOToggle(vrxPins[pin]);
}

void vrxPinsSet(int pin, bool state)
{
    const bool inverted = (1 << (pin)) & vrxPinsInversion;
    IOWrite(vrxPins[pin], state ? inverted : !inverted);
}
bool vrxPinsRead(int pin)
{
    return IORead(vrxPins[pin]);
}

void vrxDualSwitchSet(bool state)
{
    vrxPinsSet(0, state);
    vrxPinsSet(1, !state);
}

bool vrxBtnRead(int pin)
{
    return IORead(vrxBtns[pin]);
}
