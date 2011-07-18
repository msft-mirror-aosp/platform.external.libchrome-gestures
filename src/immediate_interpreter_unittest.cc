// Copyright (c) 2011 The Chromium OS Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <gtest/gtest.h>

#include "gestures/include/gestures.h"
#include "gestures/include/immediate_interpreter.h"

namespace gestures {

class ImmediateInterpreterTest : public ::testing::Test {};

TEST(ImmediateInterpreterTest, MoveDownTest) {
  ImmediateInterpreter ii;
  HardwareProperties hwprops = {
    0,  // left edge
    0,  // top edge
    1000,  // right edge
    1000,  // bottom edge
    500,  // pixels/TP width
    500,  // pixels/TP height
    96,  // screen DPI x
    96,  // screen DPI y
    2,  // max fingers
    0,  // tripletap
    0,  // semi-mt
    1  // is button pad
  };

  FingerState finger_states[] = {
    // TM, Tm, WM, Wm, Press, Orientation, X, Y, TrID
    {0, 0, 0, 0, 1, 0, 10, 10, 1},
    {0, 0, 0, 0, 1, 0, 10, 20, 1},
    {0, 0, 0, 0, 1, 0, 20, 20, 1}
  };
  HardwareState hardware_states[] = {
    // time, finger count, finger states pointer
    { 200000, 0, 1, &finger_states[0] },
    { 210000, 0, 1, &finger_states[1] },
    { 220000, 0, 1, &finger_states[2] },
    { 230000, 0, 0, NULL },
    { 240000, 0, 0, NULL }
  };

  // Should fail w/o hardware props set
  EXPECT_EQ(NULL, ii.SyncInterpret(&hardware_states[0]));

  ii.SetHardwareProperties(hwprops);

  EXPECT_EQ(NULL, ii.SyncInterpret(&hardware_states[0]));

  Gesture* gs = ii.SyncInterpret(&hardware_states[1]);
  EXPECT_NE(reinterpret_cast<Gesture*>(NULL), gs);
  EXPECT_EQ(kGestureTypeMove, gs->type);
  EXPECT_EQ(0, gs->details.move.dx);
  EXPECT_EQ(10, gs->details.move.dy);
  EXPECT_EQ(200000, gs->start_time);
  EXPECT_EQ(210000, gs->end_time);

  gs = ii.SyncInterpret(&hardware_states[2]);
  EXPECT_NE(reinterpret_cast<Gesture*>(NULL), gs);
  EXPECT_EQ(kGestureTypeMove, gs->type);
  EXPECT_EQ(10, gs->details.move.dx);
  EXPECT_EQ(0, gs->details.move.dy);
  EXPECT_EQ(210000, gs->start_time);
  EXPECT_EQ(220000, gs->end_time);

  EXPECT_EQ(reinterpret_cast<Gesture*>(NULL),
            ii.SyncInterpret(&hardware_states[3]));
  EXPECT_EQ(reinterpret_cast<Gesture*>(NULL),
            ii.SyncInterpret(&hardware_states[4]));
}

TEST(ImmediateInterpreterTest, ScrollUpTest) {
  ImmediateInterpreter ii;
  HardwareProperties hwprops = {
    0,  // left edge
    0,  // top edge
    1000,  // right edge
    1000,  // bottom edge
    20,  // pixels/TP width
    20,  // pixels/TP height
    96,  // x screen DPI
    96,  // y screen DPI
    2,  // max fingers
    0,  // tripletap
    0,  // semi-mt
    1  // is button pad
  };

  FingerState finger_states[] = {
    // TM, Tm, WM, Wm, Press, Orientation, X, Y, TrID
    {0, 0, 0, 0, 1, 0, 400, 900, 1},
    {0, 0, 0, 0, 1, 0, 405, 900, 2},

    {0, 0, 0, 0, 1, 0, 400, 800, 1},
    {0, 0, 0, 0, 1, 0, 405, 800, 2},

    {0, 0, 0, 0, 1, 0, 400, 700, 1},
    {0, 0, 0, 0, 1, 0, 405, 700, 2},
  };
  HardwareState hardware_states[] = {
    // time, finger count, finger states pointer
    { 0.200000, 0, 2, &finger_states[0] },
    { 0.210000, 0, 2, &finger_states[2] },
    { 0.220000, 0, 2, &finger_states[4] }
  };

  ii.SetHardwareProperties(hwprops);

  EXPECT_EQ(NULL, ii.SyncInterpret(&hardware_states[0]));

  Gesture* gs = ii.SyncInterpret(&hardware_states[1]);
  ASSERT_NE(reinterpret_cast<Gesture*>(NULL), gs);
  EXPECT_EQ(kGestureTypeScroll, gs->type);
  EXPECT_EQ(0, gs->details.move.dx);
  EXPECT_EQ(-100, gs->details.move.dy);
  EXPECT_EQ(0.200000, gs->start_time);
  EXPECT_EQ(0.210000, gs->end_time);

  gs = ii.SyncInterpret(&hardware_states[2]);
  ASSERT_NE(reinterpret_cast<Gesture*>(NULL), gs);
  EXPECT_EQ(kGestureTypeScroll, gs->type);
  EXPECT_EQ(0, gs->details.move.dx);
  EXPECT_EQ(-100, gs->details.move.dy);
  EXPECT_EQ(0.210000, gs->start_time);
  EXPECT_EQ(0.220000, gs->end_time);
}

TEST(ImmediateInterpreterTest, SetHardwarePropertiesTwiceTest) {
  ImmediateInterpreter ii;
  HardwareProperties hwprops = {
    0,  // left edge
    0,  // top edge
    1000,  // right edge
    1000,  // bottom edge
    500,  // pixels/TP width
    500,  // pixels/TP height
    96,  // screen DPI x
    96,  // screen DPI y
    2,  // max fingers
    0,  // tripletap
    0,  // semi-mt
    1  // is button pad
  };
  ii.SetHardwareProperties(hwprops);
  hwprops.max_finger_cnt = 3;
  ii.SetHardwareProperties(hwprops);

  FingerState finger_states[] = {
    // TM, Tm, WM, Wm, Press, Orientation, X, Y, TrID
    {0, 0, 0, 0, 1, 0, 0, 0, 1},
    {0, 0, 0, 0, 1, 0, 0, 0, 2},
    {0, 0, 0, 0, 1, 0, 0, 0, 3},
    {0, 0, 0, 0, 0, 0, 0, 0, 4},
    {0, 0, 0, 0, 0, 0, 0, 0, 5}
  };
  HardwareState hardware_state = {
    // time, buttons, finger count, finger states pointer
    200000, 0, 5, &finger_states[0]
  };
  // This used to cause a crash:
  Gesture* gs = ii.SyncInterpret(&hardware_state);
  EXPECT_EQ(reinterpret_cast<Gesture*>(NULL), gs);
}

TEST(ImmediateInterpreterTest, SameFingersTest) {
  ImmediateInterpreter ii;
  HardwareProperties hwprops = {
    0,  // left edge
    0,  // top edge
    1000,  // right edge
    1000,  // bottom edge
    500,  // x pixels/TP width
    500,  // y pixels/TP height
    96,  // x screen DPI
    96,  // y screen DPI
    2,  // max fingers
    0,  // t5r2
    0,  // semi-mt
    1  // is button pad
  };
  ii.SetHardwareProperties(hwprops);

  FingerState finger_states[] = {
    // TM, Tm, WM, Wm, Press, Orientation, X, Y, TrID
    {0, 0, 0, 0, 1, 0, 0, 0, 1},
    {0, 0, 0, 0, 1, 0, 0, 0, 1},
    {0, 0, 0, 0, 1, 0, 0, 0, 2},
    {0, 0, 0, 0, 1, 0, 0, 0, 3},
    {0, 0, 0, 0, 1, 0, 0, 0, 4},
    {0, 0, 0, 0, 1, 0, 0, 0, 5}
  };
  HardwareState hardware_state[] = {
    // time, buttons, finger count, finger states pointer
    { 200000, 0, 1, &finger_states[0] },
    { 200001, 0, 1, &finger_states[1] },
    { 200001, 0, 2, &finger_states[1] },
    { 200001, 0, 2, &finger_states[2] },
  };

  ii.SetPrevState(hardware_state[0]);
  EXPECT_TRUE(ii.SameFingers(hardware_state[1]));
  EXPECT_FALSE(ii.SameFingers(hardware_state[2]));
  ii.SetPrevState(hardware_state[2]);
  EXPECT_TRUE(ii.SameFingers(hardware_state[2]));
  EXPECT_FALSE(ii.SameFingers(hardware_state[3]));
}

TEST(ImmediateInterpreterTest, PalmTest) {
  ImmediateInterpreter ii;
  HardwareProperties hwprops = {
    0,  // left edge
    0,  // top edge
    1000,  // right edge
    1000,  // bottom edge
    500,  // x pixels/TP width
    500,  // y pixels/TP height
    96,  // x screen DPI
    96,  // y screen DPI
    2,  // max fingers
    0,  // t5r2
    0,  // semi-mt
    1  // is button pad
  };
  ii.SetHardwareProperties(hwprops);

  const int kBig = 100;  // palm pressure
  const int kSml = 1;  // small, low pressure

  FingerState finger_states[] = {
    // TM, Tm, WM, Wm, Press, Orientation, X, Y, TrID
    {0, 0, 0, 0, kSml, 0, 600, 500, 1},
    {0, 0, 0, 0, kSml, 0, 500, 500, 2},

    {0, 0, 0, 0, kSml, 0, 600, 500, 1},
    {0, 0, 0, 0, kBig, 0, 500, 500, 2},

    {0, 0, 0, 0, kSml, 0, 600, 500, 1},
    {0, 0, 0, 0, kSml, 0, 500, 500, 2}
  };
  HardwareState hardware_state[] = {
    // time, buttons, finger count, finger states pointer
    { 200000, 0, 2, &finger_states[0] },
    { 200001, 0, 2, &finger_states[2] },
    { 200002, 0, 2, &finger_states[4] },
  };


  ii.UpdatePalmState(hardware_state[0]);
  EXPECT_TRUE(ii.pointing_.end() != ii.pointing_.find(1));
  EXPECT_TRUE(ii.pending_palm_.end() == ii.pending_palm_.find(1));
  EXPECT_TRUE(ii.palm_.end() == ii.palm_.find(1));
  EXPECT_TRUE(ii.pointing_.end() != ii.pointing_.find(2));
  EXPECT_TRUE(ii.pending_palm_.end() == ii.pending_palm_.find(2));
  EXPECT_TRUE(ii.palm_.end() == ii.palm_.find(2));

  ii.UpdatePalmState(hardware_state[1]);
  EXPECT_TRUE(ii.pointing_.end() != ii.pointing_.find(1));
  EXPECT_TRUE(ii.pending_palm_.end() == ii.pending_palm_.find(1));
  EXPECT_TRUE(ii.palm_.end() == ii.palm_.find(1));
  EXPECT_TRUE(ii.pointing_.end() == ii.pointing_.find(2));
  EXPECT_TRUE(ii.pending_palm_.end() == ii.pending_palm_.find(2));
  EXPECT_TRUE(ii.palm_.end() != ii.palm_.find(2));

  ii.UpdatePalmState(hardware_state[2]);
  EXPECT_TRUE(ii.pointing_.end() != ii.pointing_.find(1));
  EXPECT_TRUE(ii.pending_palm_.end() == ii.pending_palm_.find(1));
  EXPECT_TRUE(ii.palm_.end() == ii.palm_.find(1));
  EXPECT_TRUE(ii.pointing_.end() == ii.pointing_.find(2));
  EXPECT_TRUE(ii.pending_palm_.end() == ii.pending_palm_.find(2));
  EXPECT_TRUE(ii.palm_.end() != ii.palm_.find(2));

  ii.ResetSameFingersState(0);
  EXPECT_TRUE(ii.pointing_.empty());
  EXPECT_TRUE(ii.pending_palm_.empty());
  EXPECT_TRUE(ii.palm_.empty());
}

TEST(ImmediateInterpreterTest, GetGesturingFingersTest) {
  ImmediateInterpreter ii;
  HardwareProperties hwprops = {
    0,  // left edge
    0,  // top edge
    1000,  // right edge
    1000,  // bottom edge
    500,  // pixels/TP width
    500,  // pixels/TP height
    96,  // screen DPI x
    96,  // screen DPI y
    2,  // max fingers
    0,  // t5r2
    0,  // semi-mt
    1  // is button pad
  };
  ii.SetHardwareProperties(hwprops);

  FingerState finger_states[] = {
    // TM, Tm, WM, Wm, Press, Orientation, X, Y, TrID
    {0, 0, 0, 0, 1, 0, 1, 10, 91},
    {0, 0, 0, 0, 1, 0, 2, 5,  92},
    {0, 0, 0, 0, 1, 0, 2, 9,  93},
    {0, 0, 0, 0, 1, 0, 2, 1,  94}
  };
  HardwareState hardware_state[] = {
    // time, buttons, finger count, finger states pointer
    { 200000, 0, 0, NULL },
    { 200001, 0, 1, &finger_states[0] },
    { 200002, 0, 2, &finger_states[0] },
    { 200002, 0, 3, &finger_states[0] },
    { 200002, 0, 4, &finger_states[0] }
  };

  // few pointing fingers
  ii.ResetSameFingersState(0);
  ii.UpdatePalmState(hardware_state[0]);
  EXPECT_TRUE(ii.GetGesturingFingers(hardware_state[0]).empty());

  ii.ResetSameFingersState(0);
  ii.UpdatePalmState(hardware_state[1]);
  set<short, kMaxGesturingFingers> ids =
      ii.GetGesturingFingers(hardware_state[1]);
  EXPECT_EQ(1, ids.size());
  EXPECT_TRUE(ids.end() != ids.find(91));

  ii.ResetSameFingersState(0);
  ii.UpdatePalmState(hardware_state[2]);
  ids = ii.GetGesturingFingers(hardware_state[2]);
  EXPECT_EQ(2, ids.size());
  EXPECT_TRUE(ids.end() != ids.find(91));
  EXPECT_TRUE(ids.end() != ids.find(92));

  ii.ResetSameFingersState(0);
  ii.UpdatePalmState(hardware_state[3]);
  ids = ii.GetGesturingFingers(hardware_state[3]);
  EXPECT_EQ(2, ids.size());
  EXPECT_TRUE(ids.end() != ids.find(92));
  EXPECT_TRUE(ids.end() != ids.find(93));

  ii.ResetSameFingersState(0);
  ii.UpdatePalmState(hardware_state[4]);
  ids = ii.GetGesturingFingers(hardware_state[4]);
  EXPECT_EQ(2, ids.size());
  EXPECT_TRUE(ids.end() != ids.find(92));
  EXPECT_TRUE(ids.end() != ids.find(94));

  // T5R2 test
  hwprops.supports_t5r2 = 1;
  ii.SetHardwareProperties(hwprops);
  ii.ResetSameFingersState(0);
  ii.UpdatePalmState(hardware_state[3]);
  ids = ii.GetGesturingFingers(hardware_state[3]);
  EXPECT_EQ(3, ids.size());
  EXPECT_TRUE(ids.end() != ids.find(91));
  EXPECT_TRUE(ids.end() != ids.find(92));
  EXPECT_TRUE(ids.end() != ids.find(93));
}

}  // namespace gestures
