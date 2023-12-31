// Copyright 2011 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef GESTURES_ACTIVITY_REPLAY_H_
#define GESTURES_ACTIVITY_REPLAY_H_

#include <deque>
#include <string>
#include <memory>
#include <set>

#include <json/value.h>

#include "include/activity_log.h"
#include "include/gestures.h"
#include "include/interpreter.h"

// This class can parse a JSON log, as generated by ActivityLog and replay
// it on an interpreter.

namespace gestures {

class PropRegistry;

class ActivityReplay : public GestureConsumer {
 public:
  explicit ActivityReplay(PropRegistry* prop_reg);
  // Returns true on success.
  bool Parse(const std::string& data);
  // An empty set means honor all properties
  bool Parse(const std::string& data, const std::set<std::string>& honor_props);

  // If there is any unexpected behavior, replay continues, but EXPECT_*
  // reports failure, otherwise no failure is reported.
  void Replay(Interpreter* interpreter, MetricsProperties* mprops);

  virtual void ConsumeGesture(const Gesture& gesture);

 private:
  // These return true on success
  bool ParseProperties(const Json::Value& dict,
                       const std::set<std::string>& honor_props);
  bool ParseHardwareProperties(const Json::Value& obj,
                               HardwareProperties* out_props);
  bool ParseEntry(const Json::Value& entry);
  bool ParseHardwareState(const Json::Value& entry);
  bool ParseFingerState(const Json::Value& entry, FingerState* out_fs);
  bool ParseTimerCallback(const Json::Value& entry);
  bool ParseCallbackRequest(const Json::Value& entry);
  bool ParseGesture(const Json::Value& entry);
  bool ParseGestureMove(const Json::Value& entry, Gesture* out_gs);
  bool ParseGestureScroll(const Json::Value& entry, Gesture* out_gs);
  bool ParseGestureSwipe(const Json::Value& entry, Gesture* out_gs);
  bool ParseGestureSwipeLift(const Json::Value& entry, Gesture* out_gs);
  bool ParseGesturePinch(const Json::Value& entry, Gesture* out_gs);
  bool ParseGestureButtonsChange(const Json::Value& entry, Gesture* out_gs);
  bool ParseGestureFling(const Json::Value& entry, Gesture* out_gs);
  bool ParseGestureMetrics(const Json::Value& entry, Gesture* out_gs);
  bool ParsePropChange(const Json::Value& entry);

  bool ReplayPropChange(const ActivityLog::PropChangeEntry& entry);

  ActivityLog log_;
  HardwareProperties hwprops_;
  PropRegistry* prop_reg_;
  std::deque<Gesture> consumed_gestures_;
  std::vector<std::shared_ptr<const std::string> > names_;
};

}  // namespace gestures

#endif  // GESTURES_ACTIVITY_REPLAY_H_
