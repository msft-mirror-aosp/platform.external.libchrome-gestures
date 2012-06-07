// Copyright (c) 2012 The Chromium OS Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "gestures/include/semi_mt_correcting_filter_interpreter.h"

#include <math.h>

#include "gestures/include/gestures.h"
#include "gestures/include/interpreter.h"
#include "gestures/include/logging.h"
#include "gestures/include/util.h"

namespace gestures {

SemiMtCorrectingFilterInterpreter::SemiMtCorrectingFilterInterpreter(
    PropRegistry* prop_reg, Interpreter* next)
    : last_id_(0),
      interpreter_enabled_(prop_reg, "SemiMT Correcting Filter Enable", 0),
      pressure_threshold_(prop_reg, "SemiMT Pressure Threshold", 30),
      hysteresis_pressure_(prop_reg, "SemiMT Hysteresis Pressure", 25),
      clip_non_linear_edge_(prop_reg, "SemiMT Clip Non Linear Area", 1),
      non_linear_top_(prop_reg, "SemiMT Non Linear Area Top", 1250.0),
      non_linear_bottom_(prop_reg, "SemiMT Non Linear Area Bottom", 4570.0),
      non_linear_left_(prop_reg, "SemiMT Non Linear Area Left", 1360.0),
      non_linear_right_(prop_reg, "SemiMT Non Linear Area Right", 5560.0),
      min_jump_distance_(prop_reg, "SemiMT Min Sensor Jump Distance", 150.0),
      max_jump_distance_(prop_reg, "SemiMT Max Sensor Jump Distance", 910.0),
      move_threshold_(prop_reg, "SemiMT Finger Move Threshold", 130.0),
      jump_threshold_(prop_reg, "SemiMT Finger Jump Distance", 260.0) {
  ClearHistory();
  next_.reset(next);
}

Gesture* SemiMtCorrectingFilterInterpreter::SyncInterpret(
    HardwareState* hwstate, stime_t* timeout) {

  if (is_semi_mt_device_) {
    if (interpreter_enabled_.val_) {
      LowPressureFilter(hwstate);
      AssignTrackingId(hwstate);
      if (clip_non_linear_edge_.val_)
        ClipNonLinearFingerPosition(hwstate);
      SuppressTwoToOneFingerJump(hwstate);
      SuppressOneToTwoFingerJump(hwstate);
      CorrectFingerPosition(hwstate);
      SuppressOneFingerJump(hwstate);
      SuppressSensorJump(hwstate);
      UpdateHistory(hwstate);
    } else {
      ClearHistory();
    }
  }
  return next_->SyncInterpret(hwstate, timeout);
}

Gesture* SemiMtCorrectingFilterInterpreter::HandleTimer(
    stime_t now, stime_t* timeout) {
  return next_->HandleTimer(now, timeout);
}

void SemiMtCorrectingFilterInterpreter::SetHardwareProperties(
    const HardwareProperties& hw_props) {
  is_semi_mt_device_ = hw_props.support_semi_mt;
  next_->SetHardwareProperties(hw_props);
}

void SemiMtCorrectingFilterInterpreter::UpdateHistory(HardwareState* hwstate) {
  if (prev_hwstate_.fingers) {
    prev2_hwstate_ = prev_hwstate_;
    std::copy(prev_hwstate_.fingers, prev_hwstate_.fingers + kMaxSemiMtFingers,
              prev2_fingers_);
    prev2_hwstate_.fingers = prev2_fingers_;
  }
  prev_hwstate_ = *hwstate;
  if (hwstate->fingers) {
    std::copy(hwstate->fingers, hwstate->fingers + kMaxSemiMtFingers,
              prev_fingers_);
    prev_hwstate_.fingers = prev_fingers_;
  }
}

void SemiMtCorrectingFilterInterpreter::ClearHistory() {
  memset(&prev_hwstate_, 0, sizeof(prev_hwstate_));
  memset(&prev2_hwstate_, 0, sizeof(prev2_hwstate_));
}

void SemiMtCorrectingFilterInterpreter::AssignTrackingId(
    HardwareState* hwstate) {
  if (hwstate->finger_cnt == 0) {
    return;
  } else if (prev_hwstate_.finger_cnt == 0) {
    for (size_t i = 0; i < hwstate->finger_cnt; i++)
      hwstate->fingers[i].tracking_id = last_id_++;
  } else if (prev_hwstate_.finger_cnt == 1 && hwstate->finger_cnt == 2) {
      int finger0_tid = prev_hwstate_.fingers[0].tracking_id;
      hwstate->fingers[0].tracking_id = finger0_tid;
      hwstate->fingers[1].tracking_id = last_id_;
      while (++last_id_ == finger0_tid);
  } else if (prev_hwstate_.finger_cnt == 2 && hwstate->finger_cnt == 1) {
    float dist_sq_prev_finger0 =
        DistSq(prev_hwstate_.fingers[0], hwstate->fingers[0]);
    float dist_sq_prev_finger1 =
        DistSq(prev_hwstate_.fingers[1], hwstate->fingers[0]);
    if (dist_sq_prev_finger0 < dist_sq_prev_finger1)
      hwstate->fingers[0].tracking_id = prev_hwstate_.fingers[0].tracking_id;
    else
      hwstate->fingers[0].tracking_id = prev_hwstate_.fingers[1].tracking_id;
  } else {  // hwstate->finger_cnt == prev_hwstate_.finger_cnt_
    for (size_t i = 0; i < hwstate->finger_cnt; i++)
      hwstate->fingers[i].tracking_id = prev_hwstate_.fingers[i].tracking_id;
  }
}

void SemiMtCorrectingFilterInterpreter::SwapFingerPatternX(
    HardwareState* hwstate) {
  // Update LR bits, i.e. swap position_x values.
  std::swap(hwstate->fingers[0].position_x,
            hwstate->fingers[1].position_x);
  current_pattern_ =
      static_cast<FingerPattern>(current_pattern_ ^ kSwapPositionX);
  hwstate->fingers[0].flags |= GESTURES_FINGER_WARP_X;
  hwstate->fingers[1].flags |= GESTURES_FINGER_WARP_X;
}

void SemiMtCorrectingFilterInterpreter::SwapFingerPatternY(
    HardwareState* hwstate) {
  // Update TB bits, i.e. swap position_y values.
  std::swap(hwstate->fingers[0].position_y,
            hwstate->fingers[1].position_y);
  current_pattern_ =
      static_cast<FingerPattern>(current_pattern_ ^ kSwapPositionY);
  hwstate->fingers[0].flags |= GESTURES_FINGER_WARP_Y;
  hwstate->fingers[1].flags |= GESTURES_FINGER_WARP_Y;
}

void SemiMtCorrectingFilterInterpreter::UpdateFingerPattern(
    HardwareState* hwstate, const FingerPosition& center) {
  size_t stationary_finger = 1 - moving_finger_;
  FingerPosition stationary_pos = start_pos_[stationary_finger];

  bool stationary_was_left =
      ((current_pattern_ & kFinger0OnLeft) && (stationary_finger == 0)) ||
      ((current_pattern_ & kFinger0OnRight) && (stationary_finger == 1));
  bool stationary_was_top =
      ((current_pattern_ & kFinger0OnTop) && (stationary_finger == 0)) ||
      ((current_pattern_ & kFinger0OnBottom) && (stationary_finger == 1));
  bool center_crossed_stationary_x =
      (stationary_was_left && (center.x < stationary_pos.x)) ||
      (!stationary_was_left && (center.x > stationary_pos.x));
  bool center_crossed_stationary_y =
      (stationary_was_top && (center.y < stationary_pos.y)) ||
      (!stationary_was_top && (center.y > stationary_pos.y));

  if (center_crossed_stationary_x)
    SwapFingerPatternX(hwstate);
  if (center_crossed_stationary_y)
    SwapFingerPatternY(hwstate);
  Log("current pattern:0x%X moving finger index:%zu", current_pattern_,
      moving_finger_);
}

void SemiMtCorrectingFilterInterpreter::InitCurrentPattern(
    HardwareState* hwstate, const FingerPosition& center) {
  bool finger0_on_left;
  bool finger0_on_top;

  if (prev_hwstate_.finger_cnt == 0)  {
    // TODO(cywang): Find a way to decide correct finger pattern when two new
    // fingers arrive in the same HardwareState.
    // As the Synaptics kernel driver of the profile-sensor touchpad always
    // reports the bottom-left-top-right pattern of the bounding box for two
    // fingers events. We can not determine what the correct finger pattern
    // will be. Assume what it should be from cmt for now.
    finger0_on_left = true;
    finger0_on_top = false;
  } else {  // prev_hwstate_.finger_cnt == 1
    finger0_on_left = (prev_hwstate_.fingers[0].position_x < center.x);
    finger0_on_top = (prev_hwstate_.fingers[0].position_y < center.y);
  }
  if (finger0_on_left) {
    current_pattern_ =
        finger0_on_top ? kTopLeftBottomRight : kBottomLeftTopRight;
  } else {
    current_pattern_ =
        finger0_on_top ? kTopRightBottomLeft : kBottomRightTopLeft;
  }
  Log("current pattern:0x%X ", current_pattern_);
}

void SemiMtCorrectingFilterInterpreter::UpdateAbsolutePosition(
    HardwareState* hwstate, const FingerPosition& center,
    float min_x, float min_y, float max_x, float max_y) {

  switch (current_pattern_) {
    case kTopLeftBottomRight:
      hwstate->fingers[0].position_x = min_x;
      hwstate->fingers[0].position_y = min_y;
      hwstate->fingers[1].position_x = max_x;
      hwstate->fingers[1].position_y = max_y;
      break;
    case kBottomLeftTopRight:
      hwstate->fingers[0].position_x = min_x;
      hwstate->fingers[0].position_y = max_y;
      hwstate->fingers[1].position_x = max_x;
      hwstate->fingers[1].position_y = min_y;
      break;
    case kTopRightBottomLeft:
      hwstate->fingers[0].position_x = max_x;
      hwstate->fingers[0].position_y = min_y;
      hwstate->fingers[1].position_x = min_x;
      hwstate->fingers[1].position_y = max_y;
      break;
    case kBottomRightTopLeft:
      hwstate->fingers[0].position_x = max_x;
      hwstate->fingers[0].position_y = max_y;
      hwstate->fingers[1].position_x = min_x;
      hwstate->fingers[1].position_y = min_y;
  }
}

void SemiMtCorrectingFilterInterpreter::SetPosition(
    FingerPosition* pos, HardwareState* hwstate) {
  for (size_t i = 0; i < hwstate->finger_cnt; i++) {
    pos[i].x = hwstate->fingers[i].position_x;
    pos[i].y = hwstate->fingers[i].position_y;
  }
}

void SemiMtCorrectingFilterInterpreter::ClipNonLinearFingerPosition(
    HardwareState* hwstate) {
  for (size_t i = 0; i < hwstate->finger_cnt; i++) {
    struct FingerState* finger = &hwstate->fingers[i];
    float non_linear_left = non_linear_left_.val_;
    float non_linear_right = non_linear_right_.val_;
    float non_linear_top = non_linear_top_.val_;
    float non_linear_bottom = non_linear_bottom_.val_;

    finger->position_x = std::min(non_linear_right,
        std::max(finger->position_x, non_linear_left));
    finger->position_y = std::min(non_linear_bottom,
        std::max(finger->position_y, non_linear_top));
  }
}

void SemiMtCorrectingFilterInterpreter::SuppressTwoToOneFingerJump(
    HardwareState* hwstate) {
  if (hwstate->finger_cnt != 1)
    return;

  // Warp finger motion for next two samples after 2F->1F transition
  if (prev_hwstate_.finger_cnt == 2 || prev2_hwstate_.finger_cnt == 2)
    hwstate->fingers[0].flags |=
        GESTURES_FINGER_WARP_X | GESTURES_FINGER_WARP_Y;
}

void SemiMtCorrectingFilterInterpreter::SuppressOneToTwoFingerJump(
    HardwareState* hwstate) {
  if (hwstate->finger_cnt != 2)
    return;

  // Warp finger motion for next two samples after 1F->2F transition
  if (prev_hwstate_.finger_cnt == 1 || prev2_hwstate_.finger_cnt == 1) {
    hwstate->fingers[0].flags |=
        GESTURES_FINGER_WARP_X | GESTURES_FINGER_WARP_Y;
    hwstate->fingers[1].flags |=
        GESTURES_FINGER_WARP_X | GESTURES_FINGER_WARP_Y;
  }
}

void SemiMtCorrectingFilterInterpreter::CorrectFingerPosition(
    HardwareState* hwstate) {
  if (hwstate->finger_cnt != 2)
    return;

  struct FingerState* finger0 = &hwstate->fingers[0];
  struct FingerState* finger1 = &hwstate->fingers[1];
  // kernel always reports (min_x, max_y) in finger0 and (max_x, min_y) in
  // finger1.
  float min_x = finger0->position_x;
  float max_x = finger1->position_x;
  float min_y = finger1->position_y;
  float max_y = finger0->position_y;
  FingerPosition center = { (min_x + max_x) / 2, (min_y + max_y) / 2 };

  if (prev_hwstate_.finger_cnt < 2)
    InitCurrentPattern(hwstate, center);
  UpdateAbsolutePosition(hwstate, center, min_x, min_y, max_x, max_y);
  // Detect the moving finger only if we have more than one report, i.e.
  // skip the first event for velocity calculation. Also, if the first finger
  // is clicking, then we enforce the arriving finger as the moving finger.
  if (prev_hwstate_.finger_cnt < 2) {
    // TODO(cywang): Fix the cases for which the moving finger is the one with
    // higher Y, especially for one-finger scroll vertically.

    // Assume the the moving finger is the one with lower Y. If both finger
    // arrives at the same time, i.e. the previous finger count is zero, then we
    // neither figure out the correct pattern nor make the moving_finger_ index
    // correct.
    moving_finger_ = (finger0->position_y < finger1->position_y) ? 0 : 1;
    SetPosition(start_pos_, hwstate);
  } else {
    UpdateFingerPattern(hwstate, center);
    size_t stationary_finger = 1 - moving_finger_;
    hwstate->fingers[stationary_finger].flags |= GESTURES_FINGER_WARP_X;
    hwstate->fingers[stationary_finger].flags |= GESTURES_FINGER_WARP_Y;
  }
}

void SemiMtCorrectingFilterInterpreter::LowPressureFilter(
    HardwareState* hwstate) {
  // The pressure value will be the same for both fingers for semi_mt device.
  // Therefore, we either keep or remove all fingers based on finger 0's
  // pressure.
  if (hwstate->finger_cnt == 0)
    return;
  float pressure = hwstate->fingers[0].pressure;
  if (((prev_hwstate_.finger_cnt == 0) &&
      (pressure < pressure_threshold_.val_)) ||
      ((prev_hwstate_.finger_cnt > 0) &&
      (pressure < hysteresis_pressure_.val_)))
    hwstate->finger_cnt = hwstate->touch_cnt = 0;
}

void SemiMtCorrectingFilterInterpreter::SuppressSensorJump(
    HardwareState* hwstate) {
  if (hwstate->finger_cnt != kMaxSemiMtFingers)
    return;
  // Skip the check for the first 2f report.
  if (prev_hwstate_.finger_cnt != kMaxSemiMtFingers) {
    memset(sensor_jump_, 0, sizeof(sensor_jump_));
    return;
  }
  for (size_t i = 0; i < hwstate->finger_cnt; i++) {
    struct FingerState *current = &hwstate->fingers[i];
    struct FingerState *prev =
        prev_hwstate_.GetFingerState(current->tracking_id);
    if (prev == NULL)
      continue;
    float FingerState::* const fields[] = { &FingerState::position_x,
                                            &FingerState::position_y };
    for (size_t j = 0 ; j < arraysize(fields); j++) {
      // Skip if there is a jump in previous report.
      if (sensor_jump_[i][j]) {
        sensor_jump_[i][j] = false;
        continue;
      }

      float FingerState::* const field = fields[j];
      float delta = current->*field - prev->*field;
      float abs_delta = fabsf(delta);
      if (abs_delta >= min_jump_distance_.val_ &&
          abs_delta <= max_jump_distance_.val_) {
        sensor_jump_[i][j] = true;
        // Shorten the jump by half.
        current->*field -= (delta / 2);
      }
    }
  }
}

// A previously stationary (or very slowly moving, i.e. motion < move_threshold)
// single finger that suddenly appears to jump by a large distance
// (> jump_threshold) looks suspiciously like drum roll. When we detect this,
// report its old position, but still save the amount that it moved. If the next
// sample shows that it has not continued to move at a reasonable speed
// (motion < half of the jump distance), then we assume that the jump was
// caused by drumroll, and report it as a new finger at its new position with a
// new tracking id.
void SemiMtCorrectingFilterInterpreter::SuppressOneFingerJump(
    HardwareState* hwstate) {
  if (hwstate->finger_cnt != 1)
    return;
  if (prev_hwstate_.finger_cnt != 1) {
    memset(one_finger_jump_distance_, 0, sizeof(one_finger_jump_distance_));
    return;
  }

  struct FingerState *current = &hwstate->fingers[0];
  struct FingerState *prev =
    prev_hwstate_.GetFingerState(current->tracking_id);
  if (prev == NULL)
    return;
  float FingerState::* const fields[] = { &FingerState::position_x,
                                          &FingerState::position_y };
  for (size_t j = 0 ; j < arraysize(fields); j++) {
    float FingerState::* const field = fields[j];
    float delta = current->*field - prev->*field;
    float abs_delta = fabsf(delta);
    if (one_finger_jump_distance_[j] != 0) {
      // one_finger_jump_distance is the motion that was suppressed for the
      // previous hwstate. If it is != 0, abs_delta includes this motion plus
      // the motion for the current hwstate. We check here that this new motion
      // is at least half as far as the previous motion and less than one and
      // half times of the previous motion. If so, it is a finger jump and
      // should be assigned a new tracking id, thereby indicating that it is a
      // new finger.
      float abs_jump = fabsf(one_finger_jump_distance_[j]);
      if (delta * one_finger_jump_distance_[j] >= 0 &&
          (abs_delta >= (0.5 * abs_jump) && abs_delta <= (1.5 * abs_jump))) {
        current->tracking_id = last_id_++;
      }
      one_finger_jump_distance_[j] = 0;
    } else if (abs_delta >= jump_threshold_.val_) {
      struct FingerState *prev2 =
          prev2_hwstate_.GetFingerState(current->tracking_id);
      float prev_delta = 0;
      if (prev2 != NULL)
        prev_delta = prev->*field - prev2->*field;
      // Big jump following small motion, so assume drum-roll and report
      // previous position. If we were wrong, we will fix on the next sample.
      if (fabsf(prev_delta) < move_threshold_.val_) {
        one_finger_jump_distance_[j] = delta;
        current->*field = prev->*field;
      }
    }
  }
}
}  // namespace gestures
