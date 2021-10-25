// Copyright 2021 The Chromium OS Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <gtest/gtest.h>  // for FRIEND_TEST

#include "gestures/include/filter_interpreter.h"
#include "gestures/include/gestures.h"
#include "gestures/include/prop_registry.h"
#include "gestures/include/tracer.h"

#ifndef GESTURES_INCLUDE_HAPTIC_BUTTON_GENERATOR_FILTER_INTERPRETER_H_
#define GESTURES_INCLUDE_HAPTIC_BUTTON_GENERATOR_FILTER_INTERPRETER_H_

// For haptic touchpads, the gesture library is responsible for determining the
// state of the "physical" button. This class determines the total force applied
// to the touchpad and sets the button state based on the user's force threshold
// preferences.

namespace gestures {

class HapticButtonGeneratorFilterInterpreter : public FilterInterpreter {
  FRIEND_TEST(HapticButtonGeneratorFilterInterpreterTest, SimpleTest);
  FRIEND_TEST(HapticButtonGeneratorFilterInterpreterTest, NotHapticTest);
 public:
  // Takes ownership of |next|:
  explicit HapticButtonGeneratorFilterInterpreter(PropRegistry* prop_reg,
                                                  Interpreter* next,
                                                  Tracer* tracer);
  virtual ~HapticButtonGeneratorFilterInterpreter() {}

  virtual void Initialize(const HardwareProperties* hwprops,
                          Metrics* metrics, MetricsProperties* mprops,
                          GestureConsumer* consumer);
 protected:
  virtual void SyncInterpretImpl(HardwareState* hwstate, stime_t* timeout);

 private:
  void HandleHardwareState(HardwareState* hwstate);

  static const size_t kMaxSensitivitySettings = 5;

  // All threshold values are in grams.
  const double down_thresholds_[kMaxSensitivitySettings] =
      {100.0, 125.0, 150.0, 175.0, 200.0};
  const double up_thresholds_[kMaxSensitivitySettings] =
      {80.0, 105.0, 130.0, 155.0, 180.0};

  IntProperty sensitivity_;  // [1..5]

  BoolProperty use_custom_thresholds_;
  DoubleProperty custom_down_threshold_;
  DoubleProperty custom_up_threshold_;

  BoolProperty enabled_;

  DoubleProperty force_scale_;
  DoubleProperty force_translate_;

  // Is the button currently down?
  bool button_down_;

  bool is_haptic_pad_;
};

}  // namespace gestures


#endif  // GESTURES_INCLUDE_HAPTIC_BUTTON_GENERATOR_FILTER_INTERPRETER_H_
