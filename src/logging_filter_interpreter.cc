// Copyright 2012 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "include/logging_filter_interpreter.h"

#include <errno.h>
#include <fcntl.h>
#include <string>

#include "include/file_util.h"
#include "include/logging.h"

namespace gestures {

LoggingFilterInterpreter::LoggingFilterInterpreter(PropRegistry* prop_reg,
                                                   Interpreter* next,
                                                   Tracer* tracer)
    : FilterInterpreter(prop_reg, next, tracer, true),
      event_debug_logging_enable_(prop_reg,
          "Event Debug Logging Components Enable", 0),
      event_logging_enable_(prop_reg, "Event Logging Enable", false),
      logging_notify_(prop_reg, "Logging Notify", 0),
      logging_reset_(prop_reg, "Logging Reset", 0),
      log_location_(prop_reg, "Log Path",
                    "/var/log/xorg/touchpad_activity_log.txt"),
      integrated_touchpad_(prop_reg, "Integrated Touchpad", false) {
  InitName();
  if (prop_reg && log_.get())
    prop_reg->set_activity_log(log_.get());
  event_debug_logging_enable_.SetDelegate(this);
  IntWasWritten(&event_debug_logging_enable_);
  event_logging_enable_.SetDelegate(this);
  BoolWasWritten(&event_logging_enable_);
  logging_notify_.SetDelegate(this);
  logging_reset_.SetDelegate(this);
}

void LoggingFilterInterpreter::IntWasWritten(IntProperty* prop) {
  if (prop == &logging_notify_)
    Dump(log_location_.val_);
  else if (prop == &logging_reset_)
    Clear();
  else if (prop == &event_debug_logging_enable_) {
    Log("Event Debug Enabled 0x%X", event_debug_logging_enable_.val_);
    SetEventDebugLoggingEnabled(event_debug_logging_enable_.val_);
  }
};

void LoggingFilterInterpreter::BoolWasWritten(BoolProperty* prop) {
  if (prop == &event_logging_enable_) {
    Log("Event logging %s",
        event_logging_enable_.val_ ? "enabled" : "disabled");
    SetEventLoggingEnabled(event_logging_enable_.val_);
  }
}

std::string LoggingFilterInterpreter::EncodeActivityLog() {
  return Encode();
}

void LoggingFilterInterpreter::Dump(const char* filename) {
  std::string data = Encode();
  WriteFile(filename, data.c_str(), data.size());
}
}  // namespace gestures
