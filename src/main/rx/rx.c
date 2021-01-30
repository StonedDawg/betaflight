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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include <string.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"
#include "common/utils.h"
#include "common/filter.h"

#include "config/config.h"
#include "config/config_reset.h"
#include "config/feature.h"

#include "drivers/adc.h"
#include "drivers/rx/rx_pwm.h"
#include "drivers/time.h"

#include "fc/rc_controls.h"
#include "fc/rc_modes.h"

#include "flight/failsafe.h"

#include "io/serial.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

#include "rx/rx.h"
#include "rx/pwm.h"
#include "rx/fport.h"
#include "rx/sbus.h"
#include "rx/spektrum.h"
#include "rx/srxl2.h"
#include "rx/sumd.h"
#include "rx/sumh.h"
#include "rx/msp.h"
#include "rx/xbus.h"
#include "rx/ibus.h"
#include "rx/jetiexbus.h"
#include "rx/crsf.h"
#include "rx/rx_spi.h"
#include "rx/targetcustomserial.h"


const char rcChannelLetters[] = "AERT12345678abcdefgh";

static uint16_t rssi1 = 0;                  // range: [0;1023]
static uint16_t rssi2 = 0;  
static int16_t rssi1Dbm = CRSF_RSSI_MIN;    // range: [-130,20]
static int16_t rssi2Dbm = CRSF_RSSI_MIN;    // range: [-130,20]
static timeUs_t lastMspRssiUpdateUs = 0;

static pt1Filter_t frameErrFilter;

#ifdef USE_RX_LINK_QUALITY_INFO
static uint16_t linkQuality = 0;
static uint8_t rfMode = 0;
#endif

#define MSP_RSSI_TIMEOUT_US 1500000   // 1.5 sec

#define RSSI_ADC_DIVISOR (4096 / 1024)
#define RSSI_OFFSET_SCALING (1024 / 100.0f)

#define RSSI_HYSTERESIS 5
rssiSource_e rssi1Source;
rssiSource_e rssi2Source;
linkQualitySource_e linkQualitySource;

static bool rxDataProcessingRequired = false;
static bool auxiliaryProcessingRequired = false;

static bool rxSignalReceived = false;
static bool rxFlightChannelsValid = false;
static bool rxIsInFailsafeMode = true;
static uint8_t rxChannelCount;

static timeUs_t rxNextUpdateAtUs = 0;
static uint32_t needRxSignalBefore = 0;
static uint32_t needRxSignalMaxDelayUs;
static uint32_t suspendRxSignalUntil = 0;
static uint8_t  skipRxSamples = 0;

static int16_t rcRaw[MAX_SUPPORTED_RC_CHANNEL_COUNT];     // interval [1000;2000]
int16_t rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];     // interval [1000;2000]
uint32_t rcInvalidPulsPeriod[MAX_SUPPORTED_RC_CHANNEL_COUNT];

#define MAX_INVALID_PULS_TIME    300
#define PPM_AND_PWM_SAMPLE_COUNT 3

#define DELAY_50_HZ (1000000 / 50)
#define DELAY_33_HZ (1000000 / 33)
#define DELAY_10_HZ (1000000 / 10)
#define DELAY_5_HZ (1000000 / 5)
#define SKIP_RC_ON_SUSPEND_PERIOD 1500000           // 1.5 second period in usec (call frequency independent)
#define SKIP_RC_SAMPLES_ON_RESUME  2                // flush 2 samples to drop wrong measurements (timing independent)

rxRuntimeState_t rxRuntimeState;
static uint8_t rcSampleIndex = 0;

PG_REGISTER_ARRAY_WITH_RESET_FN(rxChannelRangeConfig_t, NON_AUX_CHANNEL_COUNT, rxChannelRangeConfigs, PG_RX_CHANNEL_RANGE_CONFIG, 0);
void pgResetFn_rxChannelRangeConfigs(rxChannelRangeConfig_t *rxChannelRangeConfigs)
{
    // set default calibration to full range and 1:1 mapping
    for (int i = 0; i < NON_AUX_CHANNEL_COUNT; i++) {
        rxChannelRangeConfigs[i].min = PWM_RANGE_MIN;
        rxChannelRangeConfigs[i].max = PWM_RANGE_MAX;
    }
}

PG_REGISTER_ARRAY_WITH_RESET_FN(rxFailsafeChannelConfig_t, MAX_SUPPORTED_RC_CHANNEL_COUNT, rxFailsafeChannelConfigs, PG_RX_FAILSAFE_CHANNEL_CONFIG, 0);
void pgResetFn_rxFailsafeChannelConfigs(rxFailsafeChannelConfig_t *rxFailsafeChannelConfigs)
{
    for (int i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        rxFailsafeChannelConfigs[i].mode = (i < NON_AUX_CHANNEL_COUNT) ? RX_FAILSAFE_MODE_AUTO : RX_FAILSAFE_MODE_HOLD;
        rxFailsafeChannelConfigs[i].step = (i == THROTTLE)
            ? CHANNEL_VALUE_TO_RXFAIL_STEP(RX_MIN_USEC)
            : CHANNEL_VALUE_TO_RXFAIL_STEP(RX_MID_USEC);
    }
}

void resetAllRxChannelRangeConfigurations(rxChannelRangeConfig_t *rxChannelRangeConfig) {
    // set default calibration to full range and 1:1 mapping
    for (int i = 0; i < NON_AUX_CHANNEL_COUNT; i++) {
        rxChannelRangeConfig->min = PWM_RANGE_MIN;
        rxChannelRangeConfig->max = PWM_RANGE_MAX;
        rxChannelRangeConfig++;
    }
}

static uint16_t nullReadRawRC(const rxRuntimeState_t *rxRuntimeState, uint8_t channel)
{
    UNUSED(rxRuntimeState);
    UNUSED(channel);

    return PPM_RCVR_TIMEOUT;
}

static uint8_t nullFrameStatus(rxRuntimeState_t *rxRuntimeState)
{
    UNUSED(rxRuntimeState);

    return RX_FRAME_PENDING;
}

static bool nullProcessFrame(const rxRuntimeState_t *rxRuntimeState)
{
    UNUSED(rxRuntimeState);

    return true;
}

STATIC_UNIT_TESTED bool isPulseValid(uint16_t pulseDuration)
{
    return  pulseDuration >= rxConfig()->rx_min_usec &&
            pulseDuration <= rxConfig()->rx_max_usec;
}

#ifdef USE_SERIAL_RX
static bool serialRxInit(const rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState)
{
    bool enabled = false;
    switch (rxRuntimeState->serialrxProvider) {
#ifdef USE_SERIALRX_SRXL2
    case SERIALRX_SRXL2:
        enabled = srxl2RxInit(rxConfig, rxRuntimeState);
        break;
#endif
#ifdef USE_SERIALRX_SPEKTRUM
    case SERIALRX_SRXL:
    case SERIALRX_SPEKTRUM1024:
    case SERIALRX_SPEKTRUM2048:
        enabled = spektrumInit(rxConfig, rxRuntimeState);
        break;
#endif
#ifdef USE_SERIALRX_SBUS
    case SERIALRX_SBUS:
        enabled = sbusInit(rxConfig, rxRuntimeState);
        break;
#endif
#ifdef USE_SERIALRX_SUMD
    case SERIALRX_SUMD:
        enabled = sumdInit(rxConfig, rxRuntimeState);
        break;
#endif
#ifdef USE_SERIALRX_SUMH
    case SERIALRX_SUMH:
        enabled = sumhInit(rxConfig, rxRuntimeState);
        break;
#endif
#ifdef USE_SERIALRX_XBUS
    case SERIALRX_XBUS_MODE_B:
    case SERIALRX_XBUS_MODE_B_RJ01:
        enabled = xBusInit(rxConfig, rxRuntimeState);
        break;
#endif
#ifdef USE_SERIALRX_IBUS
    case SERIALRX_IBUS:
        enabled = ibusInit(rxConfig, rxRuntimeState);
        break;
#endif
#ifdef USE_SERIALRX_JETIEXBUS
    case SERIALRX_JETIEXBUS:
        enabled = jetiExBusInit(rxConfig, rxRuntimeState);
        break;
#endif
#ifdef USE_SERIALRX_CRSF
    case SERIALRX_CRSF:
        enabled = crsfRxInit(rxConfig, rxRuntimeState);
        break;
#endif
#ifdef USE_SERIALRX_TARGET_CUSTOM
    case SERIALRX_TARGET_CUSTOM:
        enabled = targetCustomSerialRxInit(rxConfig, rxRuntimeState);
        break;
#endif
#ifdef USE_SERIALRX_FPORT
    case SERIALRX_FPORT:
        enabled = fportRxInit(rxConfig, rxRuntimeState);
        break;
#endif
    default:
        enabled = false;
        break;
    }
    return enabled;
}
#endif

void rxInit(void)
{
    if (featureIsEnabled(FEATURE_RX_PARALLEL_PWM)) {
        rxRuntimeState.rxProvider = RX_PROVIDER_PARALLEL_PWM;
    } else if (featureIsEnabled(FEATURE_RX_PPM)) {
        rxRuntimeState.rxProvider = RX_PROVIDER_PPM;
    } else if (featureIsEnabled(FEATURE_RX_SERIAL)) {
        rxRuntimeState.rxProvider = RX_PROVIDER_SERIAL;
    } else if (featureIsEnabled(FEATURE_RX_MSP)) {
        rxRuntimeState.rxProvider = RX_PROVIDER_MSP;
    } else if (featureIsEnabled(FEATURE_RX_SPI)) {
        rxRuntimeState.rxProvider = RX_PROVIDER_SPI;
    } else {
        rxRuntimeState.rxProvider = RX_PROVIDER_NONE;
    }
    rxRuntimeState.serialrxProvider = rxConfig()->serialrx_provider;
    rxRuntimeState.rcReadRawFn = nullReadRawRC;
    rxRuntimeState.rcFrameStatusFn = nullFrameStatus;
    rxRuntimeState.rcProcessFrameFn = nullProcessFrame;
    rcSampleIndex = 0;
    needRxSignalMaxDelayUs = DELAY_10_HZ;

    for (int i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        rcData[i] = rxConfig()->midrc;
        rcInvalidPulsPeriod[i] = millis() + MAX_INVALID_PULS_TIME;
    }

    rcData[THROTTLE] = (featureIsEnabled(FEATURE_3D)) ? rxConfig()->midrc : rxConfig()->rx_min_usec;

    // Initialize ARM switch to OFF position when arming via switch is defined
    // TODO - move to rc_mode.c
    for (int i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
        const modeActivationCondition_t *modeActivationCondition = modeActivationConditions(i);
        if (modeActivationCondition->modeId == BOXARM && IS_RANGE_USABLE(&modeActivationCondition->range)) {
            // ARM switch is defined, determine an OFF value
            uint16_t value;
            if (modeActivationCondition->range.startStep > 0) {
                value = MODE_STEP_TO_CHANNEL_VALUE((modeActivationCondition->range.startStep - 1));
            } else {
                value = MODE_STEP_TO_CHANNEL_VALUE((modeActivationCondition->range.endStep + 1));
            }
            // Initialize ARM AUX channel to OFF value
            rcData[modeActivationCondition->auxChannelIndex + NON_AUX_CHANNEL_COUNT] = value;
        }
    }

    switch (rxRuntimeState.rxProvider) {
    default:

        break;
#ifdef USE_SERIAL_RX
    case RX_PROVIDER_SERIAL:
        {
            const bool enabled = serialRxInit(rxConfig(), &rxRuntimeState);
            if (!enabled) {
                rxRuntimeState.rcReadRawFn = nullReadRawRC;
                rxRuntimeState.rcFrameStatusFn = nullFrameStatus;
            }
        }

        break;
#endif

#ifdef USE_RX_MSP
    case RX_PROVIDER_MSP:
        rxMspInit(rxConfig(), &rxRuntimeState);
        needRxSignalMaxDelayUs = DELAY_5_HZ;

        break;
#endif

#ifdef USE_RX_SPI
    case RX_PROVIDER_SPI:
        {
            const bool enabled = rxSpiInit(rxSpiConfig(), &rxRuntimeState);
            if (!enabled) {
                rxRuntimeState.rcReadRawFn = nullReadRawRC;
                rxRuntimeState.rcFrameStatusFn = nullFrameStatus;
            }
        }

        break;
#endif

#if defined(USE_PWM) || defined(USE_PPM)
    case RX_PROVIDER_PPM:
    case RX_PROVIDER_PARALLEL_PWM:
        rxPwmInit(rxConfig(), &rxRuntimeState);

        break;
#endif
    }

#if defined(USE_ADC)
    if (featureIsEnabled(FEATURE_RSSI_ADC)) {
        rssi1Source = RSSI_SOURCE_ADC;
        rssi2Source = RSSI_SOURCE_ADC;
        } 
#endif
    

    // Setup source frame RSSI filtering to take averaged values every FRAME_ERR_RESAMPLE_US
    pt1FilterInit(&frameErrFilter, pt1FilterGain(GET_FRAME_ERR_LPF_FREQUENCY(rxConfig()->rssi_src_frame_lpf_period), FRAME_ERR_RESAMPLE_US/1000000.0));

    rxChannelCount = MIN(rxConfig()->max_aux_channel + NON_AUX_CHANNEL_COUNT, rxRuntimeState.channelCount);
}

bool rxIsReceivingSignal(void)
{
    return rxSignalReceived;
}

bool rxAreFlightChannelsValid(void)
{
    return rxFlightChannelsValid;
}

void suspendRxPwmPpmSignal(void)
{
#if defined(USE_PWM) || defined(USE_PPM)
    if (rxRuntimeState.rxProvider == RX_PROVIDER_PARALLEL_PWM || rxRuntimeState.rxProvider == RX_PROVIDER_PPM) {
        suspendRxSignalUntil = micros() + SKIP_RC_ON_SUSPEND_PERIOD;
        skipRxSamples = SKIP_RC_SAMPLES_ON_RESUME;
        failsafeOnRxSuspend(SKIP_RC_ON_SUSPEND_PERIOD);
    }
#endif
}

void resumeRxPwmPpmSignal(void)
{
#if defined(USE_PWM) || defined(USE_PPM)
    if (rxRuntimeState.rxProvider == RX_PROVIDER_PARALLEL_PWM || rxRuntimeState.rxProvider == RX_PROVIDER_PPM) {
        suspendRxSignalUntil = micros();
        skipRxSamples = SKIP_RC_SAMPLES_ON_RESUME;
        failsafeOnRxResume();
    }
#endif
}

#ifdef USE_RX_LINK_QUALITY_INFO
#define LINK_QUALITY_SAMPLE_COUNT 16

STATIC_UNIT_TESTED uint16_t updateLinkQualitySamples(uint16_t value)
{
    static uint16_t samplesLQ[LINK_QUALITY_SAMPLE_COUNT];
    static uint8_t sampleLQIndex = 0;
    static uint16_t sumLQ = 0;

    sumLQ += value - samplesLQ[sampleLQIndex];
    samplesLQ[sampleLQIndex] = value;
    sampleLQIndex = (sampleLQIndex + 1) % LINK_QUALITY_SAMPLE_COUNT;
    return sumLQ / LINK_QUALITY_SAMPLE_COUNT;
}

void rxSetRfMode(uint8_t rfModeValue)
{
    rfMode = rfModeValue;
}
#endif

static void setLinkQuality(bool validFrame, timeDelta_t currentDeltaTimeUs)
{
    static uint16_t rssi1Sum = 0;
    static uint16_t rssi1Count = 0;
    static uint16_t rssi2Sum = 0;
    static uint16_t rssi2Count = 0;
    static timeDelta_t resampleTimeUs = 0;

#ifdef USE_RX_LINK_QUALITY_INFO
    if (linkQualitySource != LQ_SOURCE_RX_PROTOCOL_CRSF) {
        // calculate new sample mean
        linkQuality = updateLinkQualitySamples(validFrame ? LINK_QUALITY_MAX_VALUE : 0);
    }
#endif

    if (rssi1Source == RSSI_SOURCE_FRAME_ERRORS) {
        resampleTimeUs += currentDeltaTimeUs;
        rssi1Sum += validFrame ? RSSI_MAX_VALUE : 0;
        rssi1Count++;

        if (resampleTimeUs >= FRAME_ERR_RESAMPLE_US) {
            setRssi1(rssi1Sum / rssi1Count, rssi1Source);
            rssi1Sum = 0;
            rssi1Count = 0;
            resampleTimeUs -= FRAME_ERR_RESAMPLE_US;
        }
    }
    if (rssi2Source == RSSI_SOURCE_FRAME_ERRORS) {
        resampleTimeUs += currentDeltaTimeUs;
        rssi2Sum += validFrame ? RSSI_MAX_VALUE : 0;
        rssi2Count++;

        if (resampleTimeUs >= FRAME_ERR_RESAMPLE_US) {
            setRssi2(rssi2Sum / rssi2Count, rssi2Source);
            rssi2Sum = 0;
            rssi2Count = 0;
            resampleTimeUs -= FRAME_ERR_RESAMPLE_US;
        }
    }
}

void setLinkQualityDirect(uint16_t linkqualityValue)
{
#ifdef USE_RX_LINK_QUALITY_INFO
    linkQuality = linkqualityValue;
#else
    UNUSED(linkqualityValue);
#endif
}

bool rxUpdateCheck(timeUs_t currentTimeUs, timeDelta_t currentDeltaTimeUs)
{
    bool signalReceived = false;
    bool useDataDrivenProcessing = true;

    switch (rxRuntimeState.rxProvider) {
    default:

        break;
#if defined(USE_PWM) || defined(USE_PPM)
    case RX_PROVIDER_PPM:
        if (isPPMDataBeingReceived()) {
            signalReceived = true;
            rxIsInFailsafeMode = false;
            needRxSignalBefore = currentTimeUs + needRxSignalMaxDelayUs;
            resetPPMDataReceivedState();
        }

        break;
    case RX_PROVIDER_PARALLEL_PWM:
        if (isPWMDataBeingReceived()) {
            signalReceived = true;
            rxIsInFailsafeMode = false;
            needRxSignalBefore = currentTimeUs + needRxSignalMaxDelayUs;
            useDataDrivenProcessing = false;
        }

        break;
#endif
    case RX_PROVIDER_SERIAL:
    case RX_PROVIDER_MSP:
    case RX_PROVIDER_SPI:
        {
            const uint8_t frameStatus = rxRuntimeState.rcFrameStatusFn(&rxRuntimeState);
            if (frameStatus & RX_FRAME_COMPLETE) {
                rxIsInFailsafeMode = (frameStatus & RX_FRAME_FAILSAFE) != 0;
                bool rxFrameDropped = (frameStatus & RX_FRAME_DROPPED) != 0;
                signalReceived = !(rxIsInFailsafeMode || rxFrameDropped);
                if (signalReceived) {
                    needRxSignalBefore = currentTimeUs + needRxSignalMaxDelayUs;
                }

                setLinkQuality(signalReceived, currentDeltaTimeUs);
            }

            if (frameStatus & RX_FRAME_PROCESSING_REQUIRED) {
                auxiliaryProcessingRequired = true;
            }
        }

        break;
    }

    if (signalReceived) {
        rxSignalReceived = true;
    } else if (currentTimeUs >= needRxSignalBefore) {
        rxSignalReceived = false;
    }

    if ((signalReceived && useDataDrivenProcessing) || cmpTimeUs(currentTimeUs, rxNextUpdateAtUs) > 0) {
        rxDataProcessingRequired = true;
    }

    return rxDataProcessingRequired || auxiliaryProcessingRequired; // data driven or 50Hz
}

#if defined(USE_PWM) || defined(USE_PPM)
static uint16_t calculateChannelMovingAverage(uint8_t chan, uint16_t sample)
{
    static int16_t rcSamples[MAX_SUPPORTED_RX_PARALLEL_PWM_OR_PPM_CHANNEL_COUNT][PPM_AND_PWM_SAMPLE_COUNT];
    static int16_t rcDataMean[MAX_SUPPORTED_RX_PARALLEL_PWM_OR_PPM_CHANNEL_COUNT];
    static bool rxSamplesCollected = false;

    const uint8_t currentSampleIndex = rcSampleIndex % PPM_AND_PWM_SAMPLE_COUNT;

    // update the recent samples and compute the average of them
    rcSamples[chan][currentSampleIndex] = sample;

    // avoid returning an incorrect average which would otherwise occur before enough samples
    if (!rxSamplesCollected) {
        if (rcSampleIndex < PPM_AND_PWM_SAMPLE_COUNT) {
            return sample;
        }
        rxSamplesCollected = true;
    }

    rcDataMean[chan] = 0;
    for (int sampleIndex = 0; sampleIndex < PPM_AND_PWM_SAMPLE_COUNT; sampleIndex++) {
        rcDataMean[chan] += rcSamples[chan][sampleIndex];
    }
    return rcDataMean[chan] / PPM_AND_PWM_SAMPLE_COUNT;
}
#endif

static uint16_t getRxfailValue(uint8_t channel)
{
    const rxFailsafeChannelConfig_t *channelFailsafeConfig = rxFailsafeChannelConfigs(channel);

    switch (channelFailsafeConfig->mode) {
    case RX_FAILSAFE_MODE_AUTO:
        switch (channel) {
        case ROLL:
        case PITCH:
        case YAW:
            return rxConfig()->midrc;
        case THROTTLE:
            if (featureIsEnabled(FEATURE_3D) && !IS_RC_MODE_ACTIVE(BOX3D) && !flight3DConfig()->switched_mode3d) {
                return rxConfig()->midrc;
            } else {
                return rxConfig()->rx_min_usec;
            }
        }

    FALLTHROUGH;
    default:
    case RX_FAILSAFE_MODE_INVALID:
    case RX_FAILSAFE_MODE_HOLD:
        return rcData[channel];

    case RX_FAILSAFE_MODE_SET:
        return RXFAIL_STEP_TO_CHANNEL_VALUE(channelFailsafeConfig->step);
    }
}

STATIC_UNIT_TESTED uint16_t applyRxChannelRangeConfiguraton(int sample, const rxChannelRangeConfig_t *range)
{
    // Avoid corruption of channel with a value of PPM_RCVR_TIMEOUT
    if (sample == PPM_RCVR_TIMEOUT) {
        return PPM_RCVR_TIMEOUT;
    }

    sample = scaleRange(sample, range->min, range->max, PWM_RANGE_MIN, PWM_RANGE_MAX);
    sample = constrain(sample, PWM_PULSE_MIN, PWM_PULSE_MAX);

    return sample;
}

static void readRxChannelsApplyRanges(void)
{
    for (int channel = 0; channel < rxChannelCount; channel++) {

        const uint8_t rawChannel = channel < RX_MAPPABLE_CHANNEL_COUNT ? rxConfig()->rcmap[channel] : channel;

        // sample the channel
        uint16_t sample = rxRuntimeState.rcReadRawFn(&rxRuntimeState, rawChannel);

        // apply the rx calibration
        if (channel < NON_AUX_CHANNEL_COUNT) {
            sample = applyRxChannelRangeConfiguraton(sample, rxChannelRangeConfigs(channel));
        }

        rcRaw[channel] = sample;
    }
}

static void detectAndApplySignalLossBehaviour(void)
{
    const uint32_t currentTimeMs = millis();

    const bool useValueFromRx = rxSignalReceived && !rxIsInFailsafeMode;

    DEBUG_SET(DEBUG_RX_SIGNAL_LOSS, 0, rxSignalReceived);
    DEBUG_SET(DEBUG_RX_SIGNAL_LOSS, 1, rxIsInFailsafeMode);

    rxFlightChannelsValid = true;
    for (int channel = 0; channel < rxChannelCount; channel++) {
        uint16_t sample = rcRaw[channel];

        const bool validPulse = useValueFromRx && isPulseValid(sample);

        if (validPulse) {
            rcInvalidPulsPeriod[channel] = currentTimeMs + MAX_INVALID_PULS_TIME;
        } else {
            if (cmp32(currentTimeMs, rcInvalidPulsPeriod[channel]) < 0) {
                continue;           // skip to next channel to hold channel value MAX_INVALID_PULS_TIME
            } else {
                sample = getRxfailValue(channel);   // after that apply rxfail value
                if (channel < NON_AUX_CHANNEL_COUNT) {
                    rxFlightChannelsValid = false;
                }
            }
        }
#if defined(USE_PWM) || defined(USE_PPM)
        if (rxRuntimeState.rxProvider == RX_PROVIDER_PARALLEL_PWM || rxRuntimeState.rxProvider == RX_PROVIDER_PPM) {
            // smooth output for PWM and PPM
            rcData[channel] = calculateChannelMovingAverage(channel, sample);
        } else
#endif
        {
            rcData[channel] = sample;
        }
    }

    if (rxFlightChannelsValid && !IS_RC_MODE_ACTIVE(BOXFAILSAFE)) {
        failsafeOnValidDataReceived();
    } else {
        rxIsInFailsafeMode = true;
        failsafeOnValidDataFailed();
        for (int channel = 0; channel < rxChannelCount; channel++) {
            rcData[channel] = getRxfailValue(channel);
        }
    }
    DEBUG_SET(DEBUG_RX_SIGNAL_LOSS, 3, rcData[THROTTLE]);
}

bool calculateRxChannelsAndUpdateFailsafe(timeUs_t currentTimeUs)
{
    if (auxiliaryProcessingRequired) {
        auxiliaryProcessingRequired = !rxRuntimeState.rcProcessFrameFn(&rxRuntimeState);
    }

    if (!rxDataProcessingRequired) {
        return false;
    }

    rxDataProcessingRequired = false;
    rxNextUpdateAtUs = currentTimeUs + DELAY_33_HZ;

    // only proceed when no more samples to skip and suspend period is over
    if (skipRxSamples || currentTimeUs <= suspendRxSignalUntil) {
        if (currentTimeUs > suspendRxSignalUntil) {
            skipRxSamples--;
        }

        return true;
    }

    readRxChannelsApplyRanges();
    detectAndApplySignalLossBehaviour();

    rcSampleIndex++;

    return true;
}

void parseRcChannels(const char *input, rxConfig_t *rxConfig)
{
    for (const char *c = input; *c; c++) {
        const char *s = strchr(rcChannelLetters, *c);
        if (s && (s < rcChannelLetters + RX_MAPPABLE_CHANNEL_COUNT)) {
            rxConfig->rcmap[s - rcChannelLetters] = c - input;
        }
    }
}

void setRssi1Direct(uint16_t newRssi, rssiSource_e source)
{
    if (source != rssi1Source) {
        return;
    }

    rssi1 = newRssi;
}

void setRssi2Direct(uint16_t newRssi, rssiSource_e source)
{
    if (source != rssi2Source) {
        return;
    }

    rssi2 = newRssi;
}
#define RSSI_SAMPLE_COUNT 16

static uint16_t updateRssi1Samples(uint16_t value)
{
    static uint16_t samples1[RSSI_SAMPLE_COUNT];
    static uint8_t sample1Index = 0;
    static unsigned sum1 = 0;

    sum1 += value - samples1[sample1Index];
    samples1[sample1Index] = value;
    sample1Index = (sample1Index + 1) % RSSI_SAMPLE_COUNT;
    return sum1 / RSSI_SAMPLE_COUNT;
}
static uint16_t updateRssi2Samples(uint16_t value)
{
    static uint16_t samples2[RSSI_SAMPLE_COUNT];
    static uint8_t sample2Index = 0;
    static unsigned sum2 = 0;

    sum2 += value - samples2[sample2Index];
    samples2[sample2Index] = value;
    sample2Index = (sample2Index + 1) % RSSI_SAMPLE_COUNT;
    return sum2 / RSSI_SAMPLE_COUNT;
}
void setRssi1(uint16_t rssiValue, rssiSource_e source)
{
    if (source != rssi1Source) {
        return;
    }

    // Filter RSSI value
    if (source == RSSI_SOURCE_FRAME_ERRORS) {
        rssi1 = pt1FilterApply(&frameErrFilter, rssiValue);
    } else {
        // calculate new sample mean
        rssi1 = updateRssi1Samples(rssiValue);
    }
}
void setRssi2(uint16_t rssiValue, rssiSource_e source)
{
    if (source != rssi2Source) {
        return;
    }

    // Filter RSSI value
    if (source == RSSI_SOURCE_FRAME_ERRORS) {
        rssi2 = pt1FilterApply(&frameErrFilter, rssiValue);
    } else {
        // calculate new sample mean
        rssi2 = updateRssi2Samples(rssiValue);
    }
}

void setRssiMsp(uint8_t newMspRssi)
{
    if (rssi1Source == RSSI_SOURCE_NONE) {
        rssi1Source = RSSI_SOURCE_MSP;
    }

    if (rssi1Source == RSSI_SOURCE_MSP) {
        rssi1 = ((uint16_t)newMspRssi) << 2;
        lastMspRssiUpdateUs = micros();
    }
}

static void updateRSSIPWM(void)
{
    // Read value of AUX channel as rssi
    int16_t pwmRssi = rcData[rxConfig()->rssi_channel - 1];

    // Range of rawPwmRssi is [1000;2000]. rssi should be in [0;1023];
    setRssi1Direct(scaleRange(constrain(pwmRssi, PWM_RANGE_MIN, PWM_RANGE_MAX), PWM_RANGE_MIN, PWM_RANGE_MAX, 0, RSSI_MAX_VALUE), RSSI_SOURCE_RX_CHANNEL);
}

int32_t activeReceiver = 0;
int32_t diversityTargetReceiver;

static void updateDiversity(timeUs_t currentTimeUs)
{
#ifndef USE_ADC
    UNUSED(currentTimeUs);
#else
int32_t nextReceiver = activeReceiver;

//          if (!EepromSettings.quadversity) {
            int16_t rssiDiff = (int16_t) rssi1 - (int16_t) rssi2;
            uint16_t rssiDiffAbs = abs(rssiDiff);
            int32_t  currentBestReceiver = activeReceiver;
            static uint32_t diversityHysteresis = 0;
            
            if (rssiDiff > 0) {
                currentBestReceiver = 0;
            } else if (rssiDiff < 0) {
                currentBestReceiver = 1;
            } else {
                currentBestReceiver = activeReceiver;
            }
                        if (rssiDiffAbs >= RSSI_HYSTERESIS) {
                if (currentBestReceiver == diversityTargetReceiver) {
                    if (currentTimeUs - diversityHysteresis < 0) {
                        nextReceiver = diversityTargetReceiver;
                    }
                } else {
                    diversityTargetReceiver = currentBestReceiver;
                    diversityHysteresis = currentTimeUs + DELAY_10_HZ;
                }
            } else {
                    diversityHysteresis = currentTimeUs + DELAY_10_HZ;
            }            
        #ifdef DIVERSITY_SWITCH_PIN
        IOToggle(DIVERSITY_SWITCH_PIN);
        #endif
    #endif
}
static void updateRSSI1ADC(timeUs_t currentTimeUs)
{
#ifndef USE_ADC
    UNUSED(currentTimeUs);
#else
    static uint32_t rssiUpdateAt1 = 0;

    if ((int32_t)(currentTimeUs - rssiUpdateAt1) < 0) {
        return;
    }
    rssiUpdateAt1 = currentTimeUs + DELAY_50_HZ;

    const uint16_t adcRssiSample1 = adcGetChannel(ADC_RSSI1);
    //const uint16_t adcRssiSample2 = adcGetChannel(ADC_RSSI2);
    
    uint16_t rssiValue1 = adcRssiSample1 / RSSI_ADC_DIVISOR;
    //uint16_t rssiValue2 = adcRssiSample2 / RSSI_ADC_DIVISOR;

    setRssi1(rssiValue1, RSSI_SOURCE_ADC);
    //setRssi2(rssiValue2, RSSI_SOURCE_ADC);
#endif
}
static void updateRSSI2ADC(timeUs_t currentTimeUs)
{
#ifndef USE_ADC
    UNUSED(currentTimeUs);
#else
    static uint32_t rssiUpdateAt2 = 0;

    if ((int32_t)(currentTimeUs - rssiUpdateAt2) < 0) {
        return;
    }
    rssiUpdateAt2 = currentTimeUs + DELAY_50_HZ;

    //const uint16_t adcRssiSample1 = adcGetChannel(ADC_RSSI1);
    const uint16_t adcRssiSample2 = adcGetChannel(ADC_RSSI2);
    
    //uint16_t rssiValue1 = adcRssiSample1 / RSSI_ADC_DIVISOR;
    uint16_t rssiValue2 = adcRssiSample2 / RSSI_ADC_DIVISOR;

    //setRssi1(rssiValue1, RSSI_SOURCE_ADC);
    setRssi2(rssiValue2, RSSI_SOURCE_ADC);
#endif
}

void updateRSSI(timeUs_t currentTimeUs)
{

        updateRSSI1ADC(currentTimeUs);
        updateRSSI2ADC(currentTimeUs);
        
}

uint16_t getRssi1(void)
{
    uint16_t rssiValue1 = rssi1;

    // RSSI_Invert option
    if (rxConfig()->rssi_invert) {
        rssiValue1 = RSSI_MAX_VALUE - rssiValue1;
    }

    return rxConfig()->rssi_scale / 100.0f * rssiValue1 + rxConfig()->rssi_offset * RSSI_OFFSET_SCALING;
}
uint16_t getRssi2(void)
{
    uint16_t rssiValue2 = rssi2;

    // RSSI_Invert option
    if (rxConfig()->rssi_invert) {
        rssiValue2 = RSSI_MAX_VALUE - rssiValue2;
    }

    return rxConfig()->rssi_scale / 100.0f * rssiValue2 + rxConfig()->rssi_offset * RSSI_OFFSET_SCALING;
}

uint8_t getRssi1Percent(void)
{
    return scaleRange(getRssi1(), 0, RSSI_MAX_VALUE, 0, 100);
}
uint8_t getRssi2Percent(void)
{
    return scaleRange(getRssi2(), 0, RSSI_MAX_VALUE, 0, 100);
}
int16_t getRssi1Dbm(void)
{
    return rssi1Dbm;
}

int16_t getRssi2Dbm(void)
{
    return rssi2Dbm;
}
#define RSSI_SAMPLE_COUNT_DBM 16

static int16_t updateRssi1DbmSamples(int16_t value)
{
    static int16_t samplesdbm1[RSSI_SAMPLE_COUNT_DBM];
    static uint8_t sampledbm1Index = 0;
    static int sumdbm1 = 0;

    sumdbm1 += value - samplesdbm1[sampledbm1Index];
    samplesdbm1[sampledbm1Index] = value;
    sampledbm1Index = (sampledbm1Index + 1) % RSSI_SAMPLE_COUNT_DBM;
    return sumdbm1 / RSSI_SAMPLE_COUNT_DBM;
}

void setRssi1Dbm(int16_t rssiDbmValue, rssiSource_e source)
{
    if (source != rssi1Source) {
        return;
    }

    rssi1Dbm = updateRssi1DbmSamples(rssiDbmValue);
}

void setRssi1DbmDirect(int16_t newRssiDbm, rssiSource_e source)
{
    if (source != rssi1Source) {
        return;
    }

    rssi1Dbm = newRssiDbm;
}
static int16_t updateRssi2DbmSamples(int16_t value)
{
    static int16_t samplesdbm2[RSSI_SAMPLE_COUNT_DBM];
    static uint8_t sampledbm2Index = 0;
    static int sumdbm2 = 0;

    sumdbm2 += value - samplesdbm2[sampledbm2Index];
    samplesdbm2[sampledbm2Index] = value;
    sampledbm2Index = (sampledbm2Index + 1) % RSSI_SAMPLE_COUNT_DBM;
    return sumdbm2 / RSSI_SAMPLE_COUNT_DBM;
}

void setRssi2Dbm(int16_t rssiDbmValue, rssiSource_e source)
{
    if (source != rssi2Source) {
        return;
    }

    rssi2Dbm = updateRssi2DbmSamples(rssiDbmValue);
}

void setRssi2DbmDirect(int16_t newRssiDbm, rssiSource_e source)
{
    if (source != rssi2Source) {
        return;
    }

    rssi2Dbm = newRssiDbm;
}

#ifdef USE_RX_LINK_QUALITY_INFO
uint16_t rxGetLinkQuality(void)
{
    return linkQuality;
}

uint8_t rxGetRfMode(void)
{
    return rfMode;
}

uint16_t rxGetLinkQualityPercent(void)
{
    return (linkQualitySource == LQ_SOURCE_RX_PROTOCOL_CRSF) ?  linkQuality : scaleRange(linkQuality, 0, LINK_QUALITY_MAX_VALUE, 0, 100);
}
#endif

uint16_t rxGetRefreshRate(void)
{
    return rxRuntimeState.rxRefreshRate;
}

bool isRssiConfigured(void)
{
    if(rssi1Source != RSSI_SOURCE_NONE && rssi2Source != RSSI_SOURCE_NONE){

    return 1;
    } else {
        return 0;
    }
}

timeDelta_t rxGetFrameDelta(timeDelta_t *frameAgeUs)
{
    static timeUs_t previousFrameTimeUs = 0;
    static timeDelta_t frameTimeDeltaUs = 0;

    if (rxRuntimeState.rcFrameTimeUsFn) {
        const timeUs_t frameTimeUs = rxRuntimeState.rcFrameTimeUsFn();

        *frameAgeUs = cmpTimeUs(micros(), frameTimeUs);

        const timeDelta_t deltaUs = cmpTimeUs(frameTimeUs, previousFrameTimeUs);
        if (deltaUs) {
            frameTimeDeltaUs = deltaUs;
            previousFrameTimeUs = frameTimeUs;
        }
    }

    return frameTimeDeltaUs;
}
