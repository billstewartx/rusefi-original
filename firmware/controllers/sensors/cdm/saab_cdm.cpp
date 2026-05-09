/**
 * @file    saab_cdm.cpp
 * @brief   Saab 55352173 Combustion Detection Module decoder — implementation
 *
 * See saab_cdm.h for full protocol and wiring documentation.
 *
 * Integration checklist:
 *   1. Add saab_cdm.cpp to firmware/controllers/sensors/sensors.mk
 *   2. Call initSaabCdm() from engine_controller.cpp
 *   3. Call saabCdm.onIgnitionFiring(cylIndex, angle) from spark_logic.cpp
 *   4. Call saabCdm.updateOutputChannels() from periodicSlowCallback()
 *   5. Add output channel fields to tunerstudio_outputs.h (see bottom of file)
 *   6. Add gauge definitions to rusefi.ini (see saab_cdm_gauges.ini)
 *
 * @date    2025
 * @author  (your name)
 *
 * This file is part of rusEfi - see http://rusefi.com
 * rusEfi is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.
 */

#include "pch.h"
#include "saab_cdm.h"
#include "efi_gpio.h"
#include "gpio/gpio_ext.h"
#include "scheduler.h"
#include "engine_state.h"
#include "sensor.h"

// ============================================================================
// Knock intensity thresholds (pulse counts per window → retard step)
// Tune these on your specific engine after scoping the CDM output.
// ============================================================================
static constexpr uint32_t KNOCK_THRESHOLD_LIGHT  =  3;   // pulses → light knock
static constexpr uint32_t KNOCK_THRESHOLD_MEDIUM =  7;   // pulses → medium knock
static constexpr uint32_t KNOCK_THRESHOLD_HEAVY  = 13;   // pulses → heavy knock

static constexpr float    KNOCK_RETARD_LIGHT_DEG  = -2.0f;
static constexpr float    KNOCK_RETARD_MEDIUM_DEG = -4.0f;
static constexpr float    KNOCK_RETARD_HEAVY_DEG  = -8.0f;

// Max knock pulse count used for normalising getKnockIntensity() to 0.0–1.0
static constexpr float    KNOCK_INTENSITY_MAX_PULSES = 20.0f;

// ============================================================================
// Global singleton
// ============================================================================
SaabCdmDecoder saabCdm;

// ============================================================================
// ISR trampoline statics
// The EXTI callbacks are plain C function pointers; we use static lambdas
// stored in a file-scope struct so they can capture 'this' via a pointer.
// ============================================================================
#if EFI_PROD_CODE
namespace {
    // Called from EXTI ISR — KEEP MINIMAL, NO BLOCKING
    // rusEFI EXTI callback signature is void(void*) — no timestamp parameter.
    // We call getTimeNowNt() inside to get the timestamp ourselves.
    // ExtiCallback signature: void(*)(void*, efitick_t)
    void knockEdgeCallback(void* /*arg*/, efitick_t timestamp) {
        bool pinHigh = palReadPad(getHwPort("cdmK", CDM_KNOCK_PIN),
                                  getHwPin("cdmK",  CDM_KNOCK_PIN));
        saabCdm.onKnockEdge(!pinHigh, timestamp);
    }

    void ion13EdgeCallback(void* /*arg*/, efitick_t timestamp) {
        bool pinHigh = palReadPad(getHwPort("cdm13", CDM_ION_13_PIN),
                                  getHwPin("cdm13",  CDM_ION_13_PIN));
        saabCdm.onIon13Edge(!pinHigh, timestamp);
    }

    void ion24EdgeCallback(void* /*arg*/, efitick_t timestamp) {
        bool pinHigh = palReadPad(getHwPort("cdm24", CDM_ION_24_PIN),
                                  getHwPin("cdm24",  CDM_ION_24_PIN));
        saabCdm.onIon24Edge(!pinHigh, timestamp);
    }
}   // namespace
#endif  // EFI_PROD_CODE

// ============================================================================
// SaabCdmDecoder::init
// ============================================================================
void SaabCdmDecoder::init() {
    if (m_initialized) {
        return;
    }

#if EFI_PROD_CODE
    efiSetPadMode("CDM_KNOCK",  CDM_KNOCK_PIN,  PAL_MODE_INPUT);
    efiSetPadMode("CDM_ION_13", CDM_ION_13_PIN, PAL_MODE_INPUT);
    efiSetPadMode("CDM_ION_24", CDM_ION_24_PIN, PAL_MODE_INPUT);

    efiExtiEnablePin("CDM_KNOCK",  CDM_KNOCK_PIN,  PAL_EVENT_MODE_BOTH_EDGES, knockEdgeCallback,  nullptr);
    efiExtiEnablePin("CDM_ION_13", CDM_ION_13_PIN, PAL_EVENT_MODE_BOTH_EDGES, ion13EdgeCallback, nullptr);
    efiExtiEnablePin("CDM_ION_24", CDM_ION_24_PIN, PAL_EVENT_MODE_BOTH_EDGES, ion24EdgeCallback, nullptr);

    efiPrintf("CDM: Saab 55352173 decoder initialised. KNOCK=%s ION13=%s ION24=%s",
              hwPortname(CDM_KNOCK_PIN),
              hwPortname(CDM_ION_13_PIN),
              hwPortname(CDM_ION_24_PIN));
#endif  // EFI_PROD_CODE

    m_initialized = true;
}

// ============================================================================
// SaabCdmDecoder::onIgnitionFiring
// ============================================================================
void SaabCdmDecoder::onIgnitionFiring(uint8_t cylinderIndex, float firingAngle) {
    if (cylinderIndex >= 4) {
        return;
    }

    (void)firingAngle;  // reserved for future crank-sync angle use
    efitick_t now = getTimeNowNt();
    float     rpm = Sensor::getOrZero(SensorType::Rpm);

    // Record firing context for angle calculations in ion bank processing
    m_lastFiringNt[cylinderIndex]  = now;
    m_lastFiringRpm[cylinderIndex] = rpm;

    // Reset ion bank for this cylinder's bank
    int bank = cylinderToBank(cylinderIndex);
    m_ionBank[bank].reset();

    // Reset knock pulse counter for this cylinder
    m_knock[cylinderIndex].reset();

    // Schedule knock window open and close, crank-angle-timed
    // At rpm > 0: usPerDeg = 1e6 / (rpm * 6)
    if (rpm < 200.0f) {
        return;  // not running, don't schedule
    }

    float usPerDeg      = 1e6f / (rpm * 6.0f);
    uint32_t openDelayUs  = (uint32_t)(CDM_KNOCK_WINDOW_START_DEG * usPerDeg);
    uint32_t closeDelayUs = (uint32_t)(CDM_KNOCK_WINDOW_END_DEG   * usPerDeg);

    // Knock window scheduling is only available on real hardware.
    // The simulator uses SleepExecutor which does not support scheduleByTimestamp.
#if EFI_PROD_CODE
    // 'executor' is the global SingleTimerExecutor defined in single_timer_executor.cpp
    // API: executor.schedule("msg", &scheduling_s, efitick_t timeNt, action_s)
    extern SingleTimerExecutor executor;
    uint8_t cyl = cylinderIndex;  // capture by value for lambdas

    executor.schedule(
        "CDM_KNOCK_OPEN",
        &m_knock[cyl].windowOpenEvent,
        now + US2NT(openDelayUs),
        { [this, cyl]() {
            m_activeKnockCylinder = cyl;
            m_knock[cyl].windowOpen = true;
        } }
    );

    executor.schedule(
        "CDM_KNOCK_CLOSE",
        &m_knock[cyl].windowCloseEvent,
        now + US2NT(closeDelayUs),
        { [this, cyl]() {
            m_knock[cyl].windowOpen = false;
            if (m_activeKnockCylinder == cyl) {
                m_activeKnockCylinder = -1;
            }
            applyKnockRetard(cyl);
        } }
    );
#else
    // Simulator fallback: immediately apply retard without a timed window
    (void)openDelayUs;
    (void)closeDelayUs;
    applyKnockRetard(cylinderIndex);
#endif
}

// ============================================================================
// SaabCdmDecoder::onKnockEdge  (called from EXTI ISR)
// ============================================================================
void SaabCdmDecoder::onKnockEdge(bool rising, efitick_t /*timestamp*/) {
    // Active-low: a FALLING edge = start of a 50us knock pulse.
    // Count falling edges inside the active window.
    if (!rising) {
        int8_t activeCyl = m_activeKnockCylinder;
        if (activeCyl >= 0 && m_knock[activeCyl].windowOpen) {
            m_knock[activeCyl].pulseCount++;
        }
    }
}

// ============================================================================
// SaabCdmDecoder::onIon13Edge / onIon24Edge  (called from EXTI ISR)
// ============================================================================
void SaabCdmDecoder::onIon13Edge(bool rising, efitick_t timestamp) {
    float rpm = Sensor::getOrZero(SensorType::Rpm);
    processIonEdge(m_ionBank[0], rising, timestamp, rpm);
}

void SaabCdmDecoder::onIon24Edge(bool rising, efitick_t timestamp) {
    float rpm = Sensor::getOrZero(SensorType::Rpm);
    processIonEdge(m_ionBank[1], rising, timestamp, rpm);
}

// ============================================================================
// SaabCdmDecoder::processIonEdge
// ============================================================================
void SaabCdmDecoder::processIonEdge(CdmIonBankState& bank,
                                     bool             rising,
                                     efitick_t        timestamp,
                                     float            rpm) {
    using State = CdmIonBankState::State;

    // Active-low logic: falling edge = CDM asserting (start of pulse),
    //                   rising edge  = CDM de-asserting (end of pulse)
    if (!rising) {
        // FALLING EDGE
        switch (bank.state) {
            case State::Idle:
                // Start of flame-front pulse
                bank.pulse1StartNt = timestamp;
                bank.state = State::InPulse1;
                break;
            case State::Between:
                // Start of pressure pulse
                bank.pulse2StartNt = timestamp;
                bank.state = State::InPulse2;
                break;
            default:
                // Unexpected edge — noise or signal overlap, reset
                bank.reset();
                break;
        }
    } else {
        // RISING EDGE
        switch (bank.state) {
            case State::InPulse1:
                // End of flame-front pulse
                bank.pulse1EndNt       = timestamp;
                bank.combustionDetected = true;
                bank.state             = State::Between;
                break;

            case State::InPulse2: {
                // End of pressure pulse — compute peak pressure angle
                bank.pulse2EndNt = timestamp;

                efitick_t pulse2DurationNt = bank.pulse2EndNt - bank.pulse2StartNt;
                bank.pulse2WidthMs = ntToMs(pulse2DurationNt);

                // Midpoint of pulse 2 in NT ticks relative to pulse2 start
                efitick_t midpointNt = bank.pulse2StartNt + pulse2DurationNt / 2;

                // Convert midpoint offset from firing event to crank degrees.
                // We use the most recent firing timestamp for the active cylinder
                // in this bank. Since we only have bank-level resolution here,
                // we use the bank index to pick the most recently fired cylinder.
                // This is approximate (±2–4° depending on RPM); a more accurate
                // implementation would correlate firing order precisely.
                //
                // Degrees ATDC = (midpointNt - firingNt) / NT_PER_SECOND * RPM * 6
                //
                // Pick the right cylinder firing NT: check both cylinders in bank
                // and use whichever fired most recently.
                int   bankIdx  = (&bank == &m_ionBank[0]) ? 0 : 1;
                int   cyl0     = (bankIdx == 0) ? 0 : 1;  // 0-based cyl index
                int   cyl1     = (bankIdx == 0) ? 2 : 3;
                efitick_t firingNt = (m_lastFiringNt[cyl0] > m_lastFiringNt[cyl1])
                                        ? m_lastFiringNt[cyl0]
                                        : m_lastFiringNt[cyl1];

                if (firingNt > 0 && rpm > 200.0f && midpointNt > firingNt) {
                    float elapsedS  = (float)(midpointNt - firingNt) / NT_PER_SECOND;
                    bank.peakPressureAngleDeg = elapsedS * rpm * 6.0f;
                }

                bank.state = State::Idle;
                break;
            }

            default:
                break;
        }
    }
}

// ============================================================================
// SaabCdmDecoder::applyKnockRetard
// Called when the knock window closes for a given cylinder.
// ============================================================================
void SaabCdmDecoder::applyKnockRetard(uint8_t cylinderIndex) {
    if (cylinderIndex >= 4) {
        return;
    }

    CdmCylinderKnockState& ks = m_knock[cylinderIndex];
    uint32_t pulses = ks.pulseCount;

    float retardStep = 0.0f;
    if (pulses >= KNOCK_THRESHOLD_HEAVY) {
        retardStep = KNOCK_RETARD_HEAVY_DEG;
        ks.cleanEventCount = 0;
    } else if (pulses >= KNOCK_THRESHOLD_MEDIUM) {
        retardStep = KNOCK_RETARD_MEDIUM_DEG;
        ks.cleanEventCount = 0;
    } else if (pulses >= KNOCK_THRESHOLD_LIGHT) {
        retardStep = KNOCK_RETARD_LIGHT_DEG;
        ks.cleanEventCount = 0;
    } else {
        // No knock — increment clean counter, recover timing
        ks.cleanEventCount++;
        if (ks.cleanEventCount >= CDM_KNOCK_FREE_EVENTS_FOR_RECOVERY) {
            retardStep = CDM_KNOCK_RECOVERY_DEG_PER_EVENT;  // positive = advance
        }
    }

    ks.retardDeg = ks.retardDeg + retardStep;

    // Clamp to allowed range
    if (ks.retardDeg < CDM_MAX_KNOCK_RETARD_DEG) {
        ks.retardDeg = CDM_MAX_KNOCK_RETARD_DEG;
    }
    if (ks.retardDeg > 0.0f) {
        ks.retardDeg = 0.0f;
    }

    // Push retard into the engine's knock state so the ignition scheduler
    // picks it up when computing next firing angle.
    // rusEFI stores per-cylinder knock retard in engine->knockController.
    // The existing KnockController::onKnockSenseCompleted() interface expects
    // a knock level and does its own retard calculation, so we write directly
    // to the cylinder's timing offset instead.
    //
    // NOTE: check your rusEFI version — the exact field path may differ.
    // In recent firmware this is engine->cylinders[N].ignitionOffset or
    // engine->engineState.timingAdvance with per-cylinder correction.
    // The safest cross-version approach is to use setPerCylinderTimingTrim().
    //
    // If your firmware version exposes setPerCylinderTimingTrim(cyl, deg):
    //     setPerCylinderTimingTrim(cylinderIndex, ks.retardDeg);
    //
    // If that function is not present in your version, the fallback is to
    // write to engine->knockController directly. Adjust as needed:
    //
    //     engine->knockController.knockRetard[cylinderIndex] = -ks.retardDeg;
    //
    // For now we use the documented per-cylinder trim interface:
#ifdef EFI_KNOCK_CONTROL
    engine->knockController.setKnockRetard(cylinderIndex, -ks.retardDeg);
#endif
}

// ============================================================================
// SaabCdmDecoder — public accessors
// ============================================================================

float SaabCdmDecoder::getKnockIntensity(uint8_t cylinderIndex) const {
    if (cylinderIndex >= 4) return 0.0f;
    float count = (float)m_knock[cylinderIndex].pulseCount;
    return clampF(0.0f, count / KNOCK_INTENSITY_MAX_PULSES, 1.0f);
}

float SaabCdmDecoder::getKnockRetardDeg(uint8_t cylinderIndex) const {
    if (cylinderIndex >= 4) return 0.0f;
    return m_knock[cylinderIndex].retardDeg;
}

float SaabCdmDecoder::getPeakPressureAngle(uint8_t cylinderIndex) const {
    if (cylinderIndex >= 4) return 0.0f;
    int bank = cylinderToBank(cylinderIndex);
    return m_ionBank[bank].peakPressureAngleDeg;
}

bool SaabCdmDecoder::isCombustionDetected(uint8_t cylinderIndex) const {
    if (cylinderIndex >= 4) return false;
    int bank = cylinderToBank(cylinderIndex);
    return m_ionBank[bank].combustionDetected;
}

// ============================================================================
// SaabCdmDecoder::updateOutputChannels
// Call from periodicSlowCallback() in engine_controller.cpp
// ============================================================================
void SaabCdmDecoder::updateOutputChannels() {
#if EFI_TUNER_STUDIO && defined(CDM_OUTPUT_CHANNELS_PRESENT)
    // These writes require the cdm fields to be added to output_channels_s.
    // See tunerstudio_outputs.h.addition for the struct fields to add.
    // Once added, define CDM_OUTPUT_CHANNELS_PRESENT in saab_cdm.h to enable.
    engine->outputChannels.cdmKnockIntensity[0] = getKnockIntensity(0);
    engine->outputChannels.cdmKnockIntensity[1] = getKnockIntensity(1);
    engine->outputChannels.cdmKnockIntensity[2] = getKnockIntensity(2);
    engine->outputChannels.cdmKnockIntensity[3] = getKnockIntensity(3);

    engine->outputChannels.cdmKnockRetard[0] = getKnockRetardDeg(0);
    engine->outputChannels.cdmKnockRetard[1] = getKnockRetardDeg(1);
    engine->outputChannels.cdmKnockRetard[2] = getKnockRetardDeg(2);
    engine->outputChannels.cdmKnockRetard[3] = getKnockRetardDeg(3);

    engine->outputChannels.cdmPeakPressureAngle[0] = getPeakPressureAngle(0);
    engine->outputChannels.cdmPeakPressureAngle[1] = getPeakPressureAngle(1);
    engine->outputChannels.cdmPeakPressureAngle[2] = getPeakPressureAngle(2);
    engine->outputChannels.cdmPeakPressureAngle[3] = getPeakPressureAngle(3);

    engine->outputChannels.cdmCombustionDetected[0] = isCombustionDetected(0) ? 1u : 0u;
    engine->outputChannels.cdmCombustionDetected[1] = isCombustionDetected(1) ? 1u : 0u;
    engine->outputChannels.cdmCombustionDetected[2] = isCombustionDetected(2) ? 1u : 0u;
    engine->outputChannels.cdmCombustionDetected[3] = isCombustionDetected(3) ? 1u : 0u;
#endif  // EFI_TUNER_STUDIO && CDM_OUTPUT_CHANNELS_PRESENT

    // Always clear combustion flags regardless of TS channel availability
    m_ionBank[0].combustionDetected = false;
    m_ionBank[1].combustionDetected = false;
}

// ============================================================================
// Top-level init
// ============================================================================
void initSaabCdm() {
    saabCdm.init();
}

// ============================================================================
// HOW TO WIRE IN TO THE REST OF THE FIRMWARE
// ============================================================================
//
// 1. engine_controller.cpp — near end of initEngineController():
//
//    #include "saab_cdm.h"
//    ...
//    initSaabCdm();
//
// 2. engine_controller.cpp — inside periodicSlowCallback():
//
//    saabCdm.updateOutputChannels();
//
// 3. spark_logic.cpp — inside the per-cylinder spark scheduling function
//    (wherever you call engine->scheduler.scheduleByTimestamp for ignition):
//
//    #include "saab_cdm.h"
//    ...
//    // After scheduling the spark:
//    saabCdm.onIgnitionFiring(cylinderIndex, firingAngleDeg);
//
// 4. tunerstudio_outputs.h — add to the TsOutputChannels struct:
//
//    // === Saab CDM ion sense outputs ===
//    float cdmKnockIntensity[4];      // 0.0–1.0 per cylinder
//    float cdmKnockRetard[4];         // degrees, <= 0
//    float cdmPeakPressureAngle[4];   // degrees ATDC (estimated)
//    bool  cdmCombustionDetected[4];  // combustion flag per cylinder
//
//    NOTE: if adding fields to TsOutputChannels changes PAGE_0_SIZE you will
//    need to regenerate the .ini file or adjust the size constant.
//    Prefer adding to the 'unused' reserved area that rusEFI maintains.
//
// 5. sensors.mk — add compilation:
//
//    CONTROLLERS_SENSORS_SRC += $(CONTROLLERS_SENSORS_DIR)/cdm/saab_cdm.cpp
//
// ============================================================================