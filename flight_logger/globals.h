#pragma once
#include "config.h"

// -----------------------------------------------------------------------------
//  Shared mutable state — defined in globals.cpp
// -----------------------------------------------------------------------------
extern int            g_error_state;
extern bool           g_icm_ok;
extern bool           g_mpl_ok;
extern bool           g_display_ok;
extern float          g_alt_baseline_m;
extern unsigned long  g_wake_time_ms;
extern bool           g_landing_confirmed;
extern unsigned long  g_landing_time_ms;

// -----------------------------------------------------------------------------
//  Assertion helpers — inline so every module gets them without a link step
// -----------------------------------------------------------------------------

// Range assertion: records error code and returns false on violation.
inline bool assertRange(float value, float lo, float hi, int err_code) {
    if (value < lo || value > hi) {
        g_error_state = err_code;
        return false;
    }
    return true;
}

// Null-pointer assertion: records error code and returns false if ptr is NULL.
inline bool assertNotNull(const void* ptr, int err_code) {
    if (!ptr) {
        g_error_state = err_code;
        return false;
    }
    return true;
}
