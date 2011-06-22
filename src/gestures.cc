// Copyright (c) 2011 The Chromium OS Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "gestures/include/gestures.h"

#include <sys/time.h>

#include <base/logging.h>

// C API:

static const int kMinSupportedVersion = 1;
static const int kMaxSupportedVersion = 1;

ustime_t UstimeFromTimeval(const struct timeval* tv) {
  return static_cast<ustime_t>(tv->tv_sec) * static_cast<ustime_t>(1000000) +
      static_cast<ustime_t>(tv->tv_usec);
}

GestureInterpreter* NewGestureInterpreterImpl(int version) {
  if (version < kMinSupportedVersion) {
    LOG(ERROR) << "Client too old. It's using version " << version
               << ", but library has min supported version "
               << kMinSupportedVersion;
    return NULL;
  }
  if (version > kMaxSupportedVersion) {
    LOG(ERROR) << "Client too new. It's using version " << version
               << ", but library has max supported version "
               << kMaxSupportedVersion;
    return NULL;
  }
  return new gestures::GestureInterpreter(version);
}

void DeleteGestureInterpreter(GestureInterpreter* obj) {
  delete obj;
}

void GestureInterpreterPushHardwareState(GestureInterpreter* obj,
                                         const struct HardwareState* hwstate) {
  obj->PushHardwareState(hwstate);
}

void GestureInterpreterSetHardwareProperties(
    GestureInterpreter* obj,
    const struct HardwareProperties* hwprops) {
  obj->SetHardwareProps(hwprops);
}

void GestureInterpreterSetCallback(GestureInterpreter* obj,
                                   GestureReadyFunction fn,
                                   void* user_data) {
  obj->set_callback(fn, user_data);
}

void GestureInterpreterSetTapToClickEnabled(GestureInterpreter* obj,
                                            int enabled) {
  obj->set_tap_to_click(enabled != 0);
}

void GestureInterpreterSetMovementSpeed(GestureInterpreter* obj,
                                        int speed) {
  obj->set_move_speed(speed);
}

void GestureInterpreterSetScrollSpeed(GestureInterpreter* obj,
                                      int speed) {
  obj->set_scroll_speed(speed);
}

// C++ API:

GestureInterpreter::GestureInterpreter(int version)
    : callback_(NULL),
      callback_data_(NULL),
      tap_to_click_(false),
      move_speed_(50),
      scroll_speed_(50) {}

GestureInterpreter::~GestureInterpreter() {}

void GestureInterpreter::PushHardwareState(const HardwareState* hwstate) {
  LOG(INFO) << "TODO(adlr): handle input";
}

void GestureInterpreter::SetHardwareProps(const HardwareProperties* hwprops) {
  memcpy(&hw_props_, hwprops, sizeof(hw_props_));
}
