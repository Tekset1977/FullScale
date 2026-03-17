#pragma once
#include "config.h"
#include "globals.h"

// -----------------------------------------------------------------------------
//  Ring buffer instances — defined in buffers.cpp
// -----------------------------------------------------------------------------
extern IcmRingBuf g_icm_buf;
extern MplRingBuf g_mpl_buf;

// -----------------------------------------------------------------------------
//  Push / pop API
// -----------------------------------------------------------------------------
void icmBufPush(const IcmSample* s);
bool icmBufPop(IcmSample* s);

void mplBufPush(const MplSample* s);
bool mplBufPop(MplSample* s);
