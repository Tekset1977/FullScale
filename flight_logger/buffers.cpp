#include "buffers.h"

IcmRingBuf g_icm_buf = {{}, 0U, 0U, 0U};
MplRingBuf g_mpl_buf = {{}, 0U, 0U, 0U};

// -----------------------------------------------------------------------------
//  ICM ring buffer
// -----------------------------------------------------------------------------

// Push — overwrites oldest entry on overrun (non-fatal)
void icmBufPush(const IcmSample* s) {
    if (!assertNotNull(s, ERR_NULL_PTR)) { return; }

    g_icm_buf.data[g_icm_buf.head] = *s;
    g_icm_buf.head = (uint8_t)((g_icm_buf.head + 1U) % ICM_RING_SIZE);

    if (g_icm_buf.count < ICM_RING_SIZE) {
        g_icm_buf.count++;
    } else {
        g_icm_buf.tail = (uint8_t)((g_icm_buf.tail + 1U) % ICM_RING_SIZE);
#ifdef DEBUG
        Serial.println("ICM ring overrun — oldest sample dropped");
#endif
    }
}

// Pop — returns false when buffer is empty
bool icmBufPeek(IcmSample* s) {
    if (!assertNotNull(s, ERR_NULL_PTR)) { return false; }
    if (g_icm_buf.count == 0U) { return false; }

    *s = g_icm_buf.data[g_icm_buf.tail];
    return true;
}

bool icmBufCommitPeek(void) {
    if (g_icm_buf.count == 0U) { return false; }

    g_icm_buf.tail = (uint8_t)((g_icm_buf.tail + 1U) % ICM_RING_SIZE);
    g_icm_buf.count--;
    return true;
}

bool icmBufPop(IcmSample* s) {
    if (!assertNotNull(s, ERR_SENSOR_RANGE)) { return false; }
    if (g_icm_buf.count == 0U) { return false; }

    *s = g_icm_buf.data[g_icm_buf.tail];
    g_icm_buf.tail = (uint8_t)((g_icm_buf.tail + 1U) % ICM_RING_SIZE);
    g_icm_buf.count--;
    return true;
}

// -----------------------------------------------------------------------------
//  MPL ring buffer
// -----------------------------------------------------------------------------

// Push — same overwrite-on-full policy as ICM push
void mplBufPush(const MplSample* s) {
    if (!assertNotNull(s, ERR_SENSOR_RANGE)) { return; }

    g_mpl_buf.data[g_mpl_buf.head] = *s;
    g_mpl_buf.head = (uint8_t)((g_mpl_buf.head + 1U) % MPL_RING_SIZE);

    if (g_mpl_buf.count < MPL_RING_SIZE) {
        g_mpl_buf.count++;
    } else {
        g_mpl_buf.tail = (uint8_t)((g_mpl_buf.tail + 1U) % MPL_RING_SIZE);
#ifdef DEBUG
        Serial.println("MPL ring overrun — oldest sample dropped");
#endif
    }
}

// Pop — returns false when buffer is empty
bool mplBufPeek(MplSample* s) {
    if (!assertNotNull(s, ERR_NULL_PTR)) { return false; }
    if (g_mpl_buf.count == 0U) { return false; }

    *s = g_mpl_buf.data[g_mpl_buf.tail];
    return true;
}

bool mplBufCommitPeek(void) {
    if (g_mpl_buf.count == 0U) { return false; }

    g_mpl_buf.tail = (uint8_t)((g_mpl_buf.tail + 1U) % MPL_RING_SIZE);
    g_mpl_buf.count--;
    return true;
}

bool mplBufPop(MplSample* s) {
    if (!assertNotNull(s, ERR_SENSOR_RANGE)) { return false; }
    if (g_mpl_buf.count == 0U) { return false; }

    *s = g_mpl_buf.data[g_mpl_buf.tail];
    g_mpl_buf.tail = (uint8_t)((g_mpl_buf.tail + 1U) % MPL_RING_SIZE);
    g_mpl_buf.count--;
    return true;
}
