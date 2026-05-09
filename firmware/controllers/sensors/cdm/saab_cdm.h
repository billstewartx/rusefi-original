/**
 * @file    saab_cdm.h
 * @brief   Saab 55352173 Combustion Detection Module decoder
 *
 * Decodes digital outputs from the Saab/GM Trionic 8 CDM (part 55352173)
 * for knock detection and combustion phase sensing on a 4-cylinder engine.
 *
 * CDM output protocol:
 *   Pin 3  (KNOCK)   : Burst of 50us active-low pulses. More pulses = more knock.
 *   Pin 4  (ION 1/3) : Two sequential wide active-low pulses per combustion event.
 *                      Pulse 1 = flame front. Pulse 2 = post-flame/pressure phase.
 *                      Midpoint of pulse 2 correlates with peak cylinder pressure.
 *   Pin 11 (ION 2/4) : Same structure for cylinders 2 and 4.
 *
 * All three outputs are active-low (idle HIGH, event LOW).
 * Pull-ups are on the CDM side: 0.8k (knock), 5k (ion x2).
 * All Proteus JS pins are 5V tolerant — connect directly.
 *
 * STM32 EXTI constraint: each EXTI line is shared across all GPIO ports.
 * Pins must have unique numeric suffixes. e.g. PG7, PG8, PE9 is valid;
 * PG7 and PE7 together is NOT valid (both on EXTI line 7).
 * Suggested safe combination: PG7, PG8, PG9 (all on port G, lines 7/8/9)
 * or e.g. PG7, PH8, PI9 — just no duplicate numeric suffix with any other
 * interrupt-driven input already in use on the board.
 *
 * Drop location in source tree:
 *   firmware/controllers/sensors/cdm/saab_cdm.h
 *   firmware/controllers/sensors/cdm/saab_cdm.cpp
 *
 * Add to firmware/controllers/sensors/sensors.mk:
 *   CONTROLLERS_SENSORS_SRC += $(CONTROLLERS_SENSORS_DIR)/cdm/saab_cdm.cpp
 *
 * Call initSaabCdm() from engine_controller.cpp near the end of
 * initEngineController(), after all GPIO and scheduler init is complete.
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

#pragma once

#include "global.h"
#include "efi_gpio.h"
#include "digital_input_exti.h"

// Uncomment this line ONLY after you have added the cdm fields to
// output_channels_s in tunerstudio_outputs.h (see tunerstudio_outputs.h.addition).
// Until then, leave it commented out — the firmware will compile and run fine
// without it, just without CDM gauges in TunerStudio.
// #define CDM_OUTPUT_CHANNELS_PRESENT

// ============================================================================
// Configuration — edit these to match your Proteus JS pin assignments.
// CRITICAL: Pins must have unique numeric suffixes to avoid STM32 EXTI
// conflicts. Check all other digital inputs on your board first.
// Suggested safe pins on Proteus: PG7 (JS3), PG8 (JS4), PG9 (JS5).
// ============================================================================
#ifndef CDM_KNOCK_PIN
    #define CDM_KNOCK_PIN   Gpio::G7    // CDM pin 3 — 50us knock pulse bursts
#endif
#ifndef CDM_ION_13_PIN
    #define CDM_ION_13_PIN  Gpio::G8    // CDM pin 4  — cylinders 1 & 3
#endif
#ifndef CDM_ION_24_PIN
    #define CDM_ION_24_PIN  Gpio::G9    // CDM pin 11 — cylinders 2 & 4
#endif

// Knock window: degrees ATDC over which to count knock pulses.
// The CDM knock window starts after the spark discharge ends (~2–5° ATDC)
// and knock energy arrives mostly in the 5–60° ATDC range.
#ifndef CDM_KNOCK_WINDOW_START_DEG
    #define CDM_KNOCK_WINDOW_START_DEG   5.0f
#endif
#ifndef CDM_KNOCK_WINDOW_END_DEG
    #define CDM_KNOCK_WINDOW_END_DEG    60.0f
#endif

// Maximum per-cylinder knock retard accumulation (degrees, negative)
#ifndef CDM_MAX_KNOCK_RETARD_DEG
    #define CDM_MAX_KNOCK_RETARD_DEG   -15.0f
#endif

// Recovery rate: degrees to advance per ignition event when no knock present
#ifndef CDM_KNOCK_RECOVERY_DEG_PER_EVENT
    #define CDM_KNOCK_RECOVERY_DEG_PER_EVENT  0.5f
#endif

// How many clean events before recovery begins
#ifndef CDM_KNOCK_FREE_EVENTS_FOR_RECOVERY
    #define CDM_KNOCK_FREE_EVENTS_FOR_RECOVERY  4
#endif

// ============================================================================
// Per-bank ion sense state
// ============================================================================

/**
 * Tracks pulse state for one CDM ion output bank (covers two cylinders).
 *
 * The CDM produces two pulses per combustion event on each bank pin:
 *   Pulse 1 (flame front): wide active-low pulse starting shortly after TDC
 *   Pulse 2 (pressure):    second wide active-low pulse; its midpoint
 *                          corresponds to peak cylinder pressure
 */
struct CdmIonBankState {
    // Pulse state machine
    enum class State : uint8_t {
        Idle     = 0,  // waiting for pulse 1
        InPulse1 = 1,  // inside flame-front pulse
        Between  = 2,  // between pulse 1 and pulse 2
        InPulse2 = 3,  // inside pressure pulse
    };

    State       state           = State::Idle;

    // Timestamps in NT ticks (getTimeNowNt())
    efitick_t   pulse1StartNt   = 0;
    efitick_t   pulse1EndNt     = 0;
    efitick_t   pulse2StartNt   = 0;
    efitick_t   pulse2EndNt     = 0;

    // Derived values, updated when pulse 2 ends
    float       peakPressureAngleDeg = 0.0f;  // estimated degrees ATDC
    float       pulse2WidthMs        = 0.0f;  // post-flame pulse width

    // Combustion detected flag (set on rising edge of pulse 1, cleared after read)
    volatile bool combustionDetected = false;

    void reset() {
        state = State::Idle;
        pulse1StartNt = pulse2StartNt = pulse1EndNt = pulse2EndNt = 0;
        combustionDetected = false;
    }
};

// ============================================================================
// Per-cylinder knock state
// ============================================================================

struct CdmCylinderKnockState {
    uint32_t    pulseCount          = 0;    // pulses counted in current window
    bool        windowOpen          = false;
    float       retardDeg           = 0.0f; // current accumulated retard (<=0)
    uint32_t    cleanEventCount     = 0;    // consecutive knock-free events

    // Scheduling handles for window open/close
    scheduling_s windowOpenEvent;
    scheduling_s windowCloseEvent;

    void reset() {
        pulseCount    = 0;
        windowOpen    = false;
        cleanEventCount = 0;
        // do NOT reset retardDeg — that persists across events intentionally
    }
};

// ============================================================================
// Main CDM decoder class
// ============================================================================

class SaabCdmDecoder {
public:
    /**
     * Initialise GPIO interrupts and scheduler hooks.
     * Call once from initEngineController(), after scheduler is ready.
     */
    void init();

    /**
     * Called from the ignition scheduling code when a spark fires.
     * Opens the knock window for the appropriate cylinder, resets ion state.
     *
     * @param cylinderIndex  0-based cylinder index (0=cyl1 ... 3=cyl4)
     * @param firingAngle    crank angle at which the spark fired
     */
    void onIgnitionFiring(uint8_t cylinderIndex, float firingAngle);

    // ---- Read-back accessors (safe to call from slow callbacks) ----

    /** Knock intensity 0.0–1.0 for a cylinder (0-based index) */
    float getKnockIntensity(uint8_t cylinderIndex) const;

    /** Accumulated timing retard for a cylinder in degrees (<= 0) */
    float getKnockRetardDeg(uint8_t cylinderIndex) const;

    /** Estimated peak cylinder pressure angle (degrees ATDC), per bank */
    float getPeakPressureAngle(uint8_t cylinderIndex) const;

    /** True if ion sense pulse sequence was detected this event */
    bool  isCombustionDetected(uint8_t cylinderIndex) const;

    /**
     * Push current CDM state into TunerStudio output channels.
     * Call from the slow callback (periodicSlowCallback).
     */
    void updateOutputChannels();

private:
    // ---- ISR callbacks (registered via efiExtiEnablePin) ----
    void onKnockEdge(bool rising, efitick_t timestamp);
    void onIon13Edge(bool rising, efitick_t timestamp);
    void onIon24Edge(bool rising, efitick_t timestamp);

    // ---- Knock window management ----
    void openKnockWindow(uint8_t cylinderIndex);
    void closeKnockWindow(uint8_t cylinderIndex);
    void applyKnockRetard(uint8_t cylinderIndex);

    // ---- Ion bank processing ----
    void processIonEdge(CdmIonBankState& bank,
                        bool             rising,
                        efitick_t        timestamp,
                        float            currentRpm);

    // ---- Helpers ----
    static int cylinderToBank(uint8_t cylinderIndex) {
        // Bank 0 = cylinders 0 and 2 (cyl 1 and 3 in 1-based)
        // Bank 1 = cylinders 1 and 3 (cyl 2 and 4 in 1-based)
        return (cylinderIndex == 1 || cylinderIndex == 3) ? 1 : 0;
    }

    static float ntToMs(efitick_t nt) {
        return (float)nt / (NT_PER_SECOND / 1000.0f);
    }

    // ---- State ----
    CdmIonBankState       m_ionBank[2];          // [0]=cyl1/3, [1]=cyl2/4
    CdmCylinderKnockState m_knock[4];             // per cylinder, 0-based

    // Timestamp of most recent ignition event per cylinder (for angle calc)
    efitick_t             m_lastFiringNt[4]  = {};
    float                 m_lastFiringRpm[4] = {};

    // Active cylinder index currently in its knock window (used by knock ISR)
    volatile int8_t       m_activeKnockCylinder = -1;

    bool                  m_initialized = false;
};

// Global singleton — defined in saab_cdm.cpp
extern SaabCdmDecoder saabCdm;

/**
 * Top-level init function — call from engine_controller.cpp
 */
void initSaabCdm();
