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

#include <math.h>
#include "platform.h"

#ifdef USE_INTERPOLATED_SP

#include "build/debug.h"

#include "common/maths.h"

#include "fc/rc.h"

#include "flight/pid.h"

#include "interpolated_setpoint.h"

static float setpointDeltaImpl[XYZ_AXIS_COUNT];
static float setpointDelta[XYZ_AXIS_COUNT];

typedef struct laggedMovingAverageCombined_s {
     laggedMovingAverage_t filter;
     float buf[4];
} laggedMovingAverageCombined_t;

laggedMovingAverageCombined_t  setpointDeltaAvg[XYZ_AXIS_COUNT];

static float prevSetpointSpeed[XYZ_AXIS_COUNT];
static float prevRawSetpoint[XYZ_AXIS_COUNT];
static float prevAcceleration[XYZ_AXIS_COUNT];
static float prevDeltaImpl[XYZ_AXIS_COUNT];
static float prevBoostAmount[XYZ_AXIS_COUNT];

static uint8_t ffStatus[XYZ_AXIS_COUNT];
static bool bigStep[XYZ_AXIS_COUNT];
static bool prevDuplicatePacket[XYZ_AXIS_COUNT];
static uint8_t averagingCount;

static float ffMaxRateLimit[XYZ_AXIS_COUNT];
static float ffMaxRate[XYZ_AXIS_COUNT];

void interpolatedSpInit(const pidProfile_t *pidProfile) {
    const float ffMaxRateScale = pidProfile->ff_max_rate_limit * 0.01f;
    averagingCount = pidProfile->ff_interpolate_sp;
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        ffMaxRate[i] = applyCurve(i, 1.0f);
        ffMaxRateLimit[i] = ffMaxRate[i] * ffMaxRateScale;
        laggedMovingAverageInit(&setpointDeltaAvg[i].filter, averagingCount, (float *)&setpointDeltaAvg[i].buf[0]);
    }
}

FAST_CODE_NOINLINE float interpolatedSpApply(int axis, bool newRcFrame, ffInterpolationType_t type) {

    if (newRcFrame) {
        float rawSetpoint = getRawSetpoint(axis);
        float absRawSetpoint = fabsf(rawSetpoint);
        const float rxInterval = getCurrentRxRefreshRate() * 1e-6f;
        const float rxRate = 1.0f / rxInterval;
        float setpointSpeed = (rawSetpoint - prevRawSetpoint[axis]) * rxRate;
        float absSetpointSpeed = fabsf(setpointSpeed);
        float absPrevSetpointSpeed = fabsf(prevSetpointSpeed[axis]);
        float setpointAccelerationModifier = 1.0f;

        if (setpointSpeed == 0 && absRawSetpoint < 0.98f * ffMaxRate[axis]) {
            // no movement, or sticks at max
            // the max stick check is needed to prevent interpolation when arriving at max sticks
            if (prevDuplicatePacket[axis] == true) {
                // force boost to zero
                setpointAccelerationModifier = 0.0f;
                ffStatus[axis] = 10;
            } else {
                // duplicate or identical data packet
                if (ffStatus[axis] == 3) {
                    // packet before the one before was a duplicate also
                    setpointSpeed = 0.5f * prevSetpointSpeed[axis];
                    setpointAccelerationModifier = 0.0f;
                    ffStatus[axis] = 4;
                } else {
                    if (bigStep[axis] == true) {
                        // previous movement was big eg alternate stepping from still, or dual gaps
                        // will have excess acceleration, zero out boost here and the next one
                        setpointAccelerationModifier = 0.0f;
                        ffStatus[axis] = 2;
                    } else {
                        // add acceleration only when sticks are moving briskly, to avoid just adding jitter
                        setpointSpeed =  prevSetpointSpeed[axis] + prevAcceleration[axis] * constrainf(absPrevSetpointSpeed / 3000.0f, 0.0f, 1.0f);
                        ffStatus[axis] = 1;
                    }
                }
            }
            prevDuplicatePacket[axis] = true;
        } else {
            prevDuplicatePacket[axis] = false;
            // we have movement
            if (ffStatus[axis] != 0) {
                if (ffStatus[axis] == 1) { // ie 1
                    // do something to steps after dropouts - will be too big
                    setpointSpeed *= 0.5f;
                    setpointAccelerationModifier = 0.0f;
                    ffStatus[axis] = 3; // watch for a sequential flat point
                } else if (ffStatus[axis] == 2) {
                    setpointAccelerationModifier = 0.0f;
                    ffStatus[axis] = 0;
                } else {
                    ffStatus[axis] = 0;
                }
            }
        }
        
        if (axis == FD_ROLL) {
            DEBUG_SET(DEBUG_FF_INTERPOLATED, 2, lrintf(setpointSpeed));
        }

        float setpointAcceleration = setpointSpeed - prevSetpointSpeed[axis];
        prevAcceleration[axis] = setpointAcceleration;

        // determine if speed changed a lot, to use when evaluating next packet
        if (absSetpointSpeed > 1.5f * absPrevSetpointSpeed) {
            bigStep[axis] = true;
        } else {
            bigStep[axis] = false;
        }

        // all values afterwards are small numbers
        setpointAcceleration *= pidGetDT();
        setpointDeltaImpl[axis] = setpointSpeed * pidGetDT();

        const float ffBoostFactor = pidGetFfBoostFactor();
        float boostAmount = 0.0f;
        if (ffBoostFactor != 0.0f) {
            // calculate boost and prevent kick-back spike at max deflection
            if (fabsf(rawSetpoint) < 0.95f * ffMaxRate[axis] || absSetpointSpeed > 3.0f * absPrevSetpointSpeed) {
                boostAmount = ffBoostFactor * setpointAcceleration * setpointAccelerationModifier;
            }
        }

        prevSetpointSpeed[axis] = setpointSpeed;
        prevRawSetpoint[axis] = rawSetpoint;

        if (axis == FD_ROLL) {
            DEBUG_SET(DEBUG_FF_INTERPOLATED, 0, lrintf(setpointDeltaImpl[axis] * 100));
            DEBUG_SET(DEBUG_FF_INTERPOLATED, 1, lrintf(setpointAcceleration * setpointAccelerationModifier * 100));
//            DEBUG_SET(DEBUG_FF_INTERPOLATED, 2, lrintf(((setpointDeltaImpl[axis] + boostAmount) * 100)));
            DEBUG_SET(DEBUG_FF_INTERPOLATED, 3, ffStatus[axis]);
        }

        // first order smoothing of boost to reduce jitter
        const float ffSmoothFactor = pidGetFfSmoothFactor();
        boostAmount = prevBoostAmount[axis] + ffSmoothFactor * (boostAmount - prevBoostAmount[axis]);
        prevBoostAmount[axis] = boostAmount;
        setpointDeltaImpl[axis] += boostAmount;

        // first order smoothing of FF (second order boost filtering since boost filtered twice)
        setpointDeltaImpl[axis] = prevDeltaImpl[axis] + ffSmoothFactor * (setpointDeltaImpl[axis] - prevDeltaImpl[axis]);
        prevDeltaImpl[axis] = setpointDeltaImpl[axis];

        // apply averaging
        if (type == FF_INTERPOLATE_ON) {
            setpointDelta[axis] = setpointDeltaImpl[axis];
        } else {
            setpointDelta[axis] = laggedMovingAverageUpdate(&setpointDeltaAvg[axis].filter, setpointDeltaImpl[axis]);
        }
    }
    return setpointDelta[axis];
}

FAST_CODE_NOINLINE float applyFfLimit(int axis, float value, float Kp, float currentPidSetpoint) {
    switch (axis) {
    case FD_ROLL:
        DEBUG_SET(DEBUG_FF_LIMIT, 0, value);

        break;
    case FD_PITCH:
        DEBUG_SET(DEBUG_FF_LIMIT, 1, value);

        break;
    }

    if (fabsf(currentPidSetpoint) <= ffMaxRateLimit[axis]) {
        value = constrainf(value, (-ffMaxRateLimit[axis] - currentPidSetpoint) * Kp, (ffMaxRateLimit[axis] - currentPidSetpoint) * Kp);
    } else {
        value = 0;
    }

    if (axis == FD_ROLL) {
        DEBUG_SET(DEBUG_FF_LIMIT, 2, value);
    }

    return value;
}

bool shouldApplyFfLimits(int axis)
{
    return ffMaxRateLimit[axis] != 0.0f && axis < FD_YAW;
}
#endif
