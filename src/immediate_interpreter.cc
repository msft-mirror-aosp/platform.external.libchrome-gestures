// Copyright (c) 2011 The Chromium OS Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "gestures/include/immediate_interpreter.h"

#include <algorithm>
#include <math.h>

#include "gestures/include/gestures.h"
#include "gestures/include/logging.h"

using std::make_pair;
using std::max;
using std::min;

namespace gestures {

namespace {

// TODO(adlr): make these configurable:
const float kPalmPressure = 100.0;

// Block movement for 40ms after fingers change
const stime_t kChangeTimeout = 0.04;

// Wait 200ms to lock into a gesture:
const stime_t kGestureEvaluationTimeout = 0.2;

// If two fingers have a pressure difference greater than this, we assume
// one is a thumb.
const float kTwoFingerPressureDiffThresh = 17.0;

// If two fingers are closer than this distance (in millimeters), they are
// eligible for two-finger scroll and right click.
const float kTwoFingersCloseDistanceThresh = 40.0;

// Consider scroll vs pointing when a finger has moved at least this distance
// (mm).
const float kTwoFingerScrollDistThresh = 2.0;

// If doing a scroll, only one finger needs to move. The other finger can move
// up to this distance in the opposite direction (mm).
const float kScrollStationaryFingerMaxDist = 1.0;

// Height of the bottom zone in millimeters
const float kBottomZoneSize = 10;

float MaxMag(float a, float b) {
  if (fabsf(a) > fabsf(b))
    return a;
  return b;
}
float MinMag(float a, float b) {
  if (fabsf(a) < fabsf(b))
    return a;
  return b;
}

}  // namespace {}

ImmediateInterpreter::ImmediateInterpreter() {
  prev_state_.fingers = NULL;
}

ImmediateInterpreter::~ImmediateInterpreter() {
  if (prev_state_.fingers) {
    free(prev_state_.fingers);
    prev_state_.fingers = NULL;
  }
}

Gesture* ImmediateInterpreter::SyncInterpret(HardwareState* hwstate) {
  if (!prev_state_.fingers) {
    Log("Must call SetHardwareProperties() before Push().");
    return 0;
  }

  bool have_gesture = false;

  if (prev_state_.finger_cnt && prev_state_.fingers[0].pressure &&
      hwstate->finger_cnt && hwstate->fingers[0].pressure) {
    // This and the previous frame has pressure, so create a gesture

    // For now, simple: only one possible gesture
    result_ = Gesture(kGestureMove,
                      prev_state_.timestamp,
                      hwstate->timestamp,
                      hwstate->fingers[0].position_x -
                      prev_state_.fingers[0].position_x,
                      hwstate->fingers[0].position_y -
                      prev_state_.fingers[0].position_y);
    have_gesture = true;
  }
  SetPrevState(*hwstate);
  return have_gesture ? &result_ : NULL;
}

// For now, require fingers to be in the same slots
bool ImmediateInterpreter::SameFingers(const HardwareState& hwstate) const {
  if (hwstate.finger_cnt != prev_state_.finger_cnt)
    return false;
  for (int i = 0; i < hwstate.finger_cnt; ++i) {
    if (hwstate.fingers[i].tracking_id != prev_state_.fingers[i].tracking_id)
      return false;
  }
  return true;
}

void ImmediateInterpreter::ResetSameFingersState(stime_t now) {
  palm_.clear();
  pending_palm_.clear();
  pointing_.clear();
  start_positions_.clear();
  changed_time_ = now;
}

void ImmediateInterpreter::UpdatePalmState(const HardwareState& hwstate) {
  for (short i = 0; i < hwstate.finger_cnt; i++) {
    const FingerState& fs = hwstate.fingers[i];
    bool prev_palm = palm_.find(fs.tracking_id) != palm_.end();
    bool prev_pointing = pointing_.find(fs.tracking_id) != pointing_.end();

    // Lock onto palm permanently.
    if (prev_palm)
      continue;

    // TODO(adlr): handle low-pressure palms at edge of pad by inserting them
    // into pending_palm_
    if (fs.pressure >= kPalmPressure) {
      palm_.insert(fs.tracking_id);
      pointing_.erase(fs.tracking_id);
      pending_palm_.erase(fs.tracking_id);
      continue;
    }
    if (prev_pointing)
      continue;
    pointing_.insert(fs.tracking_id);
  }
}

namespace {
struct GetGesturingFingersCompare {
  // Returns true if finger_a is strictly closer to keyboard than finger_b
  bool operator()(const FingerState* finger_a, const FingerState* finger_b) {
    return finger_a->position_y < finger_b->position_y;
  }
};
}  // namespace {}

set<short, kMaxGesturingFingers> ImmediateInterpreter::GetGesturingFingers(
    const HardwareState& hwstate) const {
  const size_t kMaxSize = 2;  // We support up to 2 finger gestures
  if (pointing_.size() <= kMaxSize ||
      (hw_props_.supports_t5r2 && pointing_.size() > 2))
    return pointing_;

  const FingerState* fs[hwstate.finger_cnt];
  for (size_t i = 0; i < hwstate.finger_cnt; ++i)
    fs[i] = &hwstate.fingers[i];

  GetGesturingFingersCompare compare;
  // Pull the kMaxSize FingerStates w/ the lowest position_y to the
  // front of fs[].
  std::partial_sort(fs, fs + kMaxSize, fs + hwstate.finger_cnt, compare);
  set<short, kMaxGesturingFingers> ret;
  ret.insert(fs[0]->tracking_id);
  ret.insert(fs[1]->tracking_id);
  return ret;
}

void ImmediateInterpreter::UpdateCurrentGestureType(
    const HardwareState& hwstate,
    const set<short, kMaxGesturingFingers>& gs_fingers) {
  if (hw_props_.supports_t5r2 && gs_fingers.size() > 2) {
    current_gesture_type_ = kGestureTypeScroll;
    return;
  }
  int num_gesturing = gs_fingers.size();
  if (num_gesturing == 0) {
    current_gesture_type_ = kGestureTypeNull;
  } else if (num_gesturing == 1) {
    current_gesture_type_ = kGestureTypeMove;
  } else if (num_gesturing == 2) {
    if (hwstate.timestamp - changed_time_ < kGestureEvaluationTimeout ||
        current_gesture_type_ == kGestureTypeNull) {
      const FingerState* fingers[] = {
        hwstate.GetFingerState(*gs_fingers.begin()),
        hwstate.GetFingerState(*(gs_fingers.begin() + 1))
      };
      if (!fingers[0] || !fingers[1]) {
        Log("Unable to find gesturing fingers!");
        return;
      }
      // See if two pointers are close together
      bool potential_two_finger_gesture =
          TwoFingersGesturing(*fingers[0],
                              *fingers[1]);
      if (!potential_two_finger_gesture) {
        current_gesture_type_ = kGestureTypeMove;
      } else {
        current_gesture_type_ = GetTwoFingerGestureType(*fingers[0],
                                                        *fingers[1]);
      }
    }
  } else {
    Log("TODO(adlr): support > 2 finger gestures.");
  }
}

bool ImmediateInterpreter::TwoFingersGesturing(
    const FingerState& finger1,
    const FingerState& finger2) const {
  // First, make sure the pressure difference isn't too great
  float pdiff = fabsf(finger1.pressure - finger2.pressure);
  if (pdiff > kTwoFingerPressureDiffThresh)
    return false;
  float xdist = fabsf(finger1.position_x - finger2.position_x);
  float ydist = fabsf(finger1.position_x - finger2.position_x);

  // Next, make sure distance between fingers isn't too great
  if ((xdist * xdist + ydist * ydist) >
      (kTwoFingersCloseDistanceThresh * kTwoFingersCloseDistanceThresh))
    return false;

  // Next, if fingers are vertically aligned and one is in the bottom zone,
  // consider that one a resting thumb (thus, do not scroll/right click)
  if (xdist < ydist && (FingerInDampenedZone(finger1) ||
                        FingerInDampenedZone(finger2)))
    return false;
  return true;
}

GestureType ImmediateInterpreter::GetTwoFingerGestureType(
    const FingerState& finger1,
    const FingerState& finger2) {
  // Compute distance traveled since fingers changed for each finger
  float dx1 = finger1.position_x -
      start_positions_[finger1.tracking_id].first;
  float dy1 = finger1.position_y -
      start_positions_[finger1.tracking_id].second;
  float dx2 = finger2.position_x -
      start_positions_[finger2.tracking_id].first;
  float dy2 = finger2.position_y -
      start_positions_[finger2.tracking_id].second;

  float large_dx = MaxMag(dx1, dx2);
  float large_dy = MaxMag(dy1, dy2);
  float small_dx = MinMag(dx1, dx2);
  float small_dy = MinMag(dy1, dy2);

  // Thresholds
  float scroll_thresh = kTwoFingerScrollDistThresh;

  if (fabsf(large_dx) > fabsf(large_dy)) {
    // consider horizontal scroll
    if (fabsf(large_dx) < scroll_thresh)
      return kGestureTypeNull;
    if (fabsf(small_dx) < kScrollStationaryFingerMaxDist)
      small_dx = 0.0;
    return ((large_dx * small_dx) >= 0.0) ?  // same direction
        kGestureTypeScroll : kGestureTypeNull;
  } else {
    // consider vertical scroll
    if (fabsf(large_dy) < scroll_thresh)
      return kGestureTypeNull;
    if (fabsf(small_dy) < kScrollStationaryFingerMaxDist)
      small_dy = 0.0;
    return ((large_dy * small_dy) >= 0.0) ?  // same direction
        kGestureTypeScroll : kGestureTypeNull;
  }
}

void ImmediateInterpreter::SetPrevState(const HardwareState& hwstate) {
  prev_state_.timestamp = hwstate.timestamp;
  prev_state_.finger_cnt = min(hwstate.finger_cnt, hw_props_.max_finger_cnt);
  memcpy(prev_state_.fingers,
         hwstate.fingers,
         prev_state_.finger_cnt * sizeof(FingerState));
}

bool ImmediateInterpreter::FingerInDampenedZone(
    const FingerState& finger) const {
  // TODO(adlr): cache thresh
  float thresh = hw_props_.bottom - kBottomZoneSize;
  return finger.position_y > thresh;
}

void ImmediateInterpreter::FillStartPositions(const HardwareState& hwstate) {
  for (short i = 0; i < hwstate.finger_cnt; i++)
    start_positions_[hwstate.fingers[i].tracking_id] =
        make_pair(hwstate.fingers[i].position_x, hwstate.fingers[i].position_y);
}
void ImmediateInterpreter::SetHardwareProperties(
    const HardwareProperties& hw_props) {
  hw_props_ = hw_props;
  if (prev_state_.fingers) {
    free(prev_state_.fingers);
    prev_state_.fingers = NULL;
  }
  prev_state_.fingers =
      reinterpret_cast<FingerState*>(calloc(hw_props_.max_finger_cnt,
                                            sizeof(FingerState)));
}

}  // namespace gestures
