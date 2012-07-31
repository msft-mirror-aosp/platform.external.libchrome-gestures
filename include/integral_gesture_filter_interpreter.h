// Copyright (c) 2012 The Chromium OS Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <base/memory/scoped_ptr.h>

#include "gestures/include/filter_interpreter.h"
#include "gestures/include/gestures.h"

#ifndef GESTURES_INTEGRAL_GESTURE_FILTER_INTERPRETER_H_
#define GESTURES_INTEGRAL_GESTURE_FILTER_INTERPRETER_H_

namespace gestures {

// This interpreter passes HardwareState unmodified to next_. All gestures
// that pass through, though, are changed to have integral values. Any
// remainder is stored and added to the next gestures. This means that if
// a user is very slowly rolling their finger, many gestures w/ values < 1
// can be accumulated and together create a move of a single pixel.

class IntegralGestureFilterInterpreter : public FilterInterpreter {
 public:
  // Takes ownership of |next|:
  explicit IntegralGestureFilterInterpreter(Interpreter* next);
  virtual ~IntegralGestureFilterInterpreter();

 private:
  virtual Gesture* SyncInterpretImpl(HardwareState* hwstate,
                                 stime_t* timeout);

  virtual Gesture* HandleTimerImpl(stime_t now, stime_t* timeout);

 private:
  void HandleGesture(Gesture** gs);

  float x_move_remainder_, y_move_remainder_;
  float hscroll_remainder_, vscroll_remainder_;
};

}  // namespace gestures

#endif  // GESTURES_INTEGRAL_GESTURE_FILTER_INTERPRETER_H_
