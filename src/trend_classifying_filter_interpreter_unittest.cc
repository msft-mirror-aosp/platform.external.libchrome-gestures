// Copyright 2022 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <gtest/gtest.h>

#include "include/trend_classifying_filter_interpreter.h"
#include "include/unittest_util.h"

namespace gestures {

class TrendClassifyingFilterInterpreterTest : public ::testing::Test {};

class TrendClassifyingFilterInterpreterTestInterpreter : public Interpreter {
 public:
  TrendClassifyingFilterInterpreterTestInterpreter()
      : Interpreter(NULL, NULL, false),
        handle_timer_called_(false) {}

  virtual void SyncInterpret(HardwareState* hwstate, stime_t* timeout) {
    EXPECT_NE(static_cast<HardwareState*>(NULL), hwstate);
    EXPECT_EQ(1, hwstate->finger_cnt);
    prev_ = hwstate->fingers[0];
  }

  virtual void HandleTimer(stime_t now, stime_t* timeout) {
    handle_timer_called_ = true;
  }

  FingerState prev_;
  bool handle_timer_called_;
};


TEST(TrendClassifyingFilterInterpreterTest, SimpleTest) {
  TrendClassifyingFilterInterpreterTestInterpreter* base_interpreter =
      new TrendClassifyingFilterInterpreterTestInterpreter;
  TrendClassifyingFilterInterpreter interpreter(NULL, base_interpreter, NULL);

  EXPECT_TRUE(interpreter.trend_classifying_filter_enable_.val_);

  EXPECT_FALSE(interpreter.second_order_enable_.val_);
  interpreter.second_order_enable_.val_ = true;
  EXPECT_TRUE(interpreter.second_order_enable_.val_);

  HardwareProperties hwprops = {
    0, 0, 100, 100,  // left, top, right, bottom
    1, 1,  // x res (pixels/mm), y res (pixels/mm)
    1, 1,  // scrn DPI X, Y
    -1,  // orientation minimum
    2,   // orientation maximum
    5, 5,  // max fingers, max_touch,
    0, 0, 1,  // t5r2, semi, button pad
    0, 0,  // has wheel, vertical wheel is high resolution
    0,  // haptic pad
  };
  TestInterpreterWrapper wrapper(&interpreter, &hwprops);

  EXPECT_FALSE(base_interpreter->handle_timer_called_);
  wrapper.HandleTimer(0.0, NULL);
  EXPECT_TRUE(base_interpreter->handle_timer_called_);

  FingerState finger_states[] = {
    // TM, Tm, WM, Wm, Press, Orientation, X, Y, TrID
    // Consistent movement for 16 frames
    {0, 0, 0, 0, 20, 0, 40, 20, 1, 0}, // 0
    {0, 0, 0, 0, 20, 0, 40, 25, 1, 0}, // 1
    {0, 0, 0, 0, 20, 0, 40, 30, 1, 0}, // 2
    {0, 0, 0, 0, 20, 0, 40, 35, 1, 0}, // 3
    {0, 0, 0, 0, 20, 0, 40, 40, 1, 0}, // 4
    {0, 0, 0, 0, 20, 0, 40, 45, 1, 0}, // 5
    {0, 0, 0, 0, 20, 0, 40, 50, 1, 0}, // 6
    {0, 0, 0, 0, 20, 0, 40, 55, 1, 0}, // 7
    {0, 0, 0, 0, 20, 0, 40, 60, 1, 0}, // 8
    {0, 0, 0, 0, 20, 0, 40, 65, 1, 0}, // 9
    {0, 0, 0, 0, 20, 0, 40, 70, 1, 0}, // 10
    {0, 0, 0, 0, 20, 0, 40, 75, 1, 0}, // 11
    {0, 0, 0, 0, 20, 0, 40, 80, 1, 0}, // 12
    {0, 0, 0, 0, 20, 0, 40, 85, 1, 0}, // 13
    {0, 0, 0, 0, 20, 0, 40, 90, 1, 0}, // 14
    {0, 0, 0, 0, 20, 0, 40, 95, 1, 0}, // 15
  };

  HardwareState hardware_states[] = {
    // time, buttons, finger count, touch count, finger states pointer
    make_hwstate(1.00, 0, 1, 1, &finger_states[0]),
    make_hwstate(1.01, 0, 1, 1, &finger_states[1]),
    make_hwstate(1.02, 0, 1, 1, &finger_states[2]),
    make_hwstate(1.03, 0, 1, 1, &finger_states[3]),
    make_hwstate(1.04, 0, 1, 1, &finger_states[4]),
    make_hwstate(1.05, 0, 1, 1, &finger_states[5]),
    make_hwstate(1.06, 0, 1, 1, &finger_states[6]),
    make_hwstate(1.07, 0, 1, 1, &finger_states[7]),
    make_hwstate(1.08, 0, 1, 1, &finger_states[8]),
    make_hwstate(1.09, 0, 1, 1, &finger_states[9]),
    make_hwstate(1.10, 0, 1, 1, &finger_states[10]),
    make_hwstate(1.11, 0, 1, 1, &finger_states[11]),
    make_hwstate(1.12, 0, 1, 1, &finger_states[12]),
    make_hwstate(1.13, 0, 1, 1, &finger_states[13]),
    make_hwstate(1.14, 0, 1, 1, &finger_states[14]),
    make_hwstate(1.15, 0, 1, 1, &finger_states[15]),
  };

  for (size_t i = 0; i < arraysize(hardware_states); i++) {
    HardwareState *hwstate = &hardware_states[i];
    wrapper.SyncInterpret(hwstate, NULL);

    for (short j = 0; i < hwstate->finger_cnt; i++) {
      FingerState *fs = &hwstate->fingers[j];
      EXPECT_EQ(fs->flags, 0);
    }
  }

  EXPECT_TRUE(interpreter.trend_classifying_filter_enable_.val_);
  EXPECT_TRUE(interpreter.second_order_enable_.val_);
  EXPECT_EQ(interpreter.min_num_of_samples_.val_, 6);
  EXPECT_EQ(interpreter.z_threshold_.val_, 2.5758293035489004);
}

}  // namespace gestures
