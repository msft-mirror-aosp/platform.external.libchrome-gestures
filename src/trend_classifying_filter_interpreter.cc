// Copyright 2013 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "include/trend_classifying_filter_interpreter.h"

#include <cmath>

#include "include/filter_interpreter.h"
#include "include/finger_metrics.h"
#include "include/gestures.h"
#include "include/logging.h"
#include "include/prop_registry.h"
#include "include/tracer.h"
#include "include/util.h"

namespace {

// Constants for multiplication. Used in
// TrendClassifyingFilterInterpreter::ComputeKTVariance (see the header file).
const double k1_18 = 1.0 / 18.0;
const double k2_3 = 2.0 / 3.0;

const int kNumOfSamples = 20;

}  // namespace {}

namespace gestures {

TrendClassifyingFilterInterpreter::TrendClassifyingFilterInterpreter(
    PropRegistry* prop_reg, Interpreter* next, Tracer* tracer)
    : FilterInterpreter(NULL, next, tracer, false),
      trend_classifying_filter_enable_(
          prop_reg, "Trend Classifying Filter Enabled", true),
      second_order_enable_(
          prop_reg, "Trend Classifying 2nd-order Motion Enabled", false),
      min_num_of_samples_(
          prop_reg, "Trend Classifying Min Num of Samples", 6),
      num_of_samples_(
          prop_reg, "Trend Classifying Num of Samples", kNumOfSamples),
      z_threshold_(
          prop_reg, "Trend Classifying Z Threshold", 2.5758293035489004) {
  InitName();
}

void TrendClassifyingFilterInterpreter::SyncInterpretImpl(
    HardwareState* hwstate, stime_t* timeout) {
  if (trend_classifying_filter_enable_.val_)
    UpdateFingerState(*hwstate);
  next_->SyncInterpret(hwstate, timeout);
}

double TrendClassifyingFilterInterpreter::ComputeKTVariance(const int tie_n2,
    const int tie_n3, const size_t n_samples) {
  // Replace divisions with multiplications for better performance
  double var_n = n_samples * (n_samples - 1) * (2 * n_samples + 5) * k1_18;
  double var_t = k2_3 * tie_n3 + tie_n2;
  return var_n - var_t;
}

void TrendClassifyingFilterInterpreter::InterpretTestResult(
    const TrendType trend_type,
    const unsigned flag_increasing,
    const unsigned flag_decreasing,
    unsigned* flags) {
  if (trend_type == TREND_INCREASING)
    *flags |= flag_increasing;
  else if (trend_type == TREND_DECREASING)
    *flags |= flag_decreasing;
}

void TrendClassifyingFilterInterpreter::AddNewStateToBuffer(
    FingerHistory& history, const FingerState& fs) {
  // The history buffer is already full, pop one
  if (history.size() == static_cast<size_t>(num_of_samples_.val_))
    history.pop_front();

  // Push the new finger state to the back of buffer
  auto& current = history.emplace_back(fs);
  if (history.size() == 1)
    return;
  auto& previous_end = history.at(-2);

  current.DxAxis()->val = current.XAxis()->val - previous_end.XAxis()->val;
  current.DyAxis()->val = current.YAxis()->val - previous_end.YAxis()->val;
  // Update the nodes already in the buffer and compute the Kendall score/
  // variance along the way. Complexity is O(|buffer|) per finger.
  int tie_n2[KState::n_axes_] = { 0, 0, 0, 0, 0, 0 };
  int tie_n3[KState::n_axes_] = { 0, 0, 0, 0, 0, 0 };
  for (auto it = history.begin(); it != history.end(); ++it)
    for (size_t i = 0; i < KState::n_axes_; i++)
      if (it != history.begin() || !KState::IsDelta(i)) {
        UpdateKTValuePair(&it->axes_[i], &current.axes_[i],
            &tie_n2[i], &tie_n3[i]);
      }
  size_t n_samples = history.size();
  for (size_t i = 0; i < KState::n_axes_; i++) {
    current.axes_[i].var = ComputeKTVariance(tie_n2[i], tie_n3[i],
        KState::IsDelta(i) ? n_samples - 1 : n_samples);
  }
}

TrendClassifyingFilterInterpreter::TrendType
TrendClassifyingFilterInterpreter::RunKTTest(const KState::KAxis* current,
    const size_t n_samples) {
  // Sample size is too small for a meaningful result
  if (n_samples < static_cast<size_t>(min_num_of_samples_.val_))
    return TREND_NONE;

  // A zero score implies purely random behavior. Need to special-case it
  // because the test might be fooled with a zero variance (e.g. all
  // observations are tied).
  if (!current->score)
    return TREND_NONE;

  // The test conduct the hypothesis test based on the fact that S/sqrt(Var(S))
  // approximately follows the normal distribution. To optimize for speed,
  // we reformulate the expression to drop the sqrt and division operations.
  if (current->score * current->score <
      z_threshold_.val_ * z_threshold_.val_ * current->var) {
    return TREND_NONE;
  }
  return (current->score > 0) ? TREND_INCREASING : TREND_DECREASING;
}

void TrendClassifyingFilterInterpreter::UpdateFingerState(
    const HardwareState& hwstate) {
  RemoveMissingIdsFromMap(&histories_, hwstate);

  FingerState* fs = hwstate.fingers;
  for (short i = 0; i < hwstate.finger_cnt; i++) {
    // Update the map if the contact is new
    if (!MapContainsKey(histories_, fs[i].tracking_id)) {
      histories_[fs[i].tracking_id] = FingerHistory{};
    }
    auto& history = histories_[fs[i].tracking_id];

    // Check if the score demonstrates statistical significance
    AddNewStateToBuffer(history, fs[i]);
    const auto& current = history.back();
    const size_t n_samples = history.size();
    for (size_t idx = 0; idx < KState::n_axes_; idx++)
      if (second_order_enable_.val_ || !KState::IsDelta(idx)) {
        TrendType result = RunKTTest(&current.axes_[idx],
            KState::IsDelta(idx) ? n_samples - 1 : n_samples);
        InterpretTestResult(result, KState::IncFlag(idx),
            KState::DecFlag(idx), &(fs[i].flags));
      }
  }
}

void TrendClassifyingFilterInterpreter::KState::Init() {
  for (size_t i = 0; i < KState::n_axes_; i++)
    axes_[i].Init();
}

void TrendClassifyingFilterInterpreter::KState::Init(const FingerState& fs) {
  Init();
  XAxis()->val = fs.position_x, YAxis()->val = fs.position_y;
  PressureAxis()->val = fs.pressure;
  TouchMajorAxis()->val = fs.touch_major;
}

}
