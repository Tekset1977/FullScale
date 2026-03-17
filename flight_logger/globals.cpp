#include "globals.h"

int           g_error_state      = ERR_NONE;
bool          g_icm_ok           = false;
bool          g_mpl_ok           = false;
bool          g_display_ok       = false;
float         g_alt_baseline_m   = 0.0f;
unsigned long g_wake_time_ms     = 0UL;
bool          g_landing_confirmed = false;
unsigned long g_landing_time_ms  = 0UL;
