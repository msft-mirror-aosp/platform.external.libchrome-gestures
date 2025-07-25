// Copyright 2012 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "include/scaling_filter_interpreter.h"

#include <math.h>

#include "include/gestures.h"
#include "include/interpreter.h"
#include "include/logging.h"
#include "include/tracer.h"

namespace gestures {

// Takes ownership of |next|:
ScalingFilterInterpreter::ScalingFilterInterpreter(
    PropRegistry* prop_reg, Interpreter* next, Tracer* tracer,
    GestureInterpreterDeviceClass devclass)
    : FilterInterpreter(nullptr, next, tracer, false),
      tp_x_scale_(1.0),
      tp_y_scale_(1.0),
      tp_x_translate_(0.0),
      tp_y_translate_(0.0),
      screen_x_scale_(1.0),
      screen_y_scale_(1.0),
      orientation_scale_(1.0),
      invert_scrolling_and_swiping_(prop_reg, "Australian Scrolling", false),
      invert_scrolling_only_(prop_reg, "Invert Scrolling", false),
      surface_area_from_pressure_(prop_reg,
                                  "Compute Surface Area from Pressure", true),
      use_touch_size_for_haptic_pad_(
          prop_reg,
          "Compute Surface Area from Touch Size for Haptic Pads",
          false),
      tp_x_bias_(prop_reg, "Touchpad Device Output Bias on X-Axis", 0.0),
      tp_y_bias_(prop_reg, "Touchpad Device Output Bias on Y-Axis", 0.0),
      pressure_scale_(prop_reg, "Pressure Calibration Slope", 1.0),
      pressure_translate_(prop_reg, "Pressure Calibration Offset", 0.0),
      pressure_threshold_(prop_reg, "Pressure Minimum Threshold", 0.0),
      filter_low_pressure_(prop_reg, "Filter Low Pressure", false),
      force_touch_count_to_match_finger_count_(
          prop_reg,
          "Force Touch Count To Match Finger Count",
          false),
      mouse_cpi_(prop_reg, "Mouse CPI", 1000.0),
      device_mouse_(prop_reg, "Device Mouse", IsMouseDevice(devclass)),
      device_pointing_stick_(prop_reg, "Device Pointing Stick",
                             IsPointingStick(devclass)),
      device_touchpad_(prop_reg,
                       "Device Touchpad",
                       IsTouchpadDevice(devclass)) {
  InitName();
}

void ScalingFilterInterpreter::SyncInterpretImpl(HardwareState& hwstate,
                                                     stime_t* timeout) {
  const char name[] = "ScalingFilterInterpreter::SyncInterpretImpl";
  LogHardwareStatePre(name, hwstate);

  ScaleHardwareState(hwstate);

  LogHardwareStatePost(name, hwstate);
  next_->SyncInterpret(hwstate, timeout);
}

// Ignore the finger events with low pressure values especially for the SEMI_MT
// devices such as Synaptics touchpad on Cr-48.
void ScalingFilterInterpreter::FilterLowPressure(HardwareState& hwstate) {
  unsigned short finger_cnt = hwstate.finger_cnt;
  unsigned short touch_cnt = hwstate.touch_cnt;
  float threshold = 0.0;

  // If a button is down, only filter 0 pressure fingers:
  // Pressing fingers might have low pressure, but hovering fingers are
  // reported as fingers with 0 pressure and should still be filtered.
  if (pressure_scale_.val_ > 0.0 && !hwstate.buttons_down) {
    threshold = (pressure_threshold_.val_ - pressure_translate_.val_)
        / pressure_scale_.val_ ;
  }
  for (short i = finger_cnt - 1 ; i >= 0; i--) {
    if (hwstate.fingers[i].pressure <= threshold) {
      if (i != finger_cnt - 1)
        hwstate.fingers[i] = hwstate.fingers[finger_cnt - 1];
      finger_cnt--;
      if (touch_cnt > 0)
        touch_cnt--;
    }
  }
  hwstate.finger_cnt = finger_cnt;
  hwstate.touch_cnt = touch_cnt;
}

void ScalingFilterInterpreter::FilterZeroArea(HardwareState& hwstate) {
  unsigned short finger_cnt = hwstate.finger_cnt;
  unsigned short touch_cnt = hwstate.touch_cnt;
  for (short i = finger_cnt - 1 ; i >= 0; i--) {
    if (hwstate.fingers[i].pressure == 0.0) {
      if (i != finger_cnt - 1)
        hwstate.fingers[i] = hwstate.fingers[finger_cnt - 1];
      finger_cnt--;
      if (touch_cnt > 0)
        touch_cnt--;
    }
  }
  hwstate.finger_cnt = finger_cnt;
  hwstate.touch_cnt = touch_cnt;
}

bool ScalingFilterInterpreter::IsMouseDevice(
    GestureInterpreterDeviceClass devclass) {
  return (devclass == GESTURES_DEVCLASS_MOUSE ||
          devclass == GESTURES_DEVCLASS_MULTITOUCH_MOUSE);
}

bool ScalingFilterInterpreter::IsPointingStick(
    GestureInterpreterDeviceClass devclass) {
  return devclass == GESTURES_DEVCLASS_POINTING_STICK;
}

bool ScalingFilterInterpreter::IsTouchpadDevice(
    GestureInterpreterDeviceClass devclass) {
  return (devclass == GESTURES_DEVCLASS_TOUCHPAD ||
          devclass == GESTURES_DEVCLASS_MULTITOUCH_MOUSE ||
          devclass == GESTURES_DEVCLASS_TOUCHSCREEN);
}

void ScalingFilterInterpreter::ScaleHardwareState(HardwareState& hwstate) {
  if (device_touchpad_.val_)
    ScaleTouchpadHardwareState(hwstate);
  if (device_mouse_.val_ || device_pointing_stick_.val_)
    ScaleMouseHardwareState(hwstate);
}

void ScalingFilterInterpreter::ScaleMouseHardwareState(
    HardwareState& hwstate) {
  hwstate.rel_x = hwstate.rel_x / mouse_cpi_.val_ * 25.4;
  hwstate.rel_y = hwstate.rel_y / mouse_cpi_.val_ * 25.4;
  // TODO(clchiou): Scale rel_wheel and rel_hwheel
}

void ScalingFilterInterpreter::ScaleTouchpadHardwareState(
    HardwareState& hwstate) {
  if (force_touch_count_to_match_finger_count_.val_)
    hwstate.touch_cnt = hwstate.finger_cnt;

  if (surface_area_from_pressure_.val_) {
    // Drop the small fingers, i.e. low pressures.
    if (filter_low_pressure_.val_ || pressure_threshold_.val_ > 0.0)
      FilterLowPressure(hwstate);
  }

  for (short i = 0; i < hwstate.finger_cnt; i++) {
    float cos_2_orit = 0.0, sin_2_orit = 0.0, rx_2 = 0.0, ry_2 = 0.0;
    hwstate.fingers[i].position_x *= tp_x_scale_;
    hwstate.fingers[i].position_x += tp_x_translate_;
    hwstate.fingers[i].position_y *= tp_y_scale_;
    hwstate.fingers[i].position_y += tp_y_translate_;

    // TODO(clchiou): Output orientation is computed on a pixel-unit circle,
    // and it is only equal to the orientation computed on a mm-unit circle
    // when tp_x_scale_ == tp_y_scale_.  Since what we really want is the
    // latter, fix this!
    hwstate.fingers[i].orientation *= orientation_scale_;

    if (hwstate.fingers[i].touch_major || hwstate.fingers[i].touch_minor) {
      cos_2_orit = cosf(hwstate.fingers[i].orientation);
      cos_2_orit *= cos_2_orit;
      sin_2_orit = sinf(hwstate.fingers[i].orientation);
      sin_2_orit *= sin_2_orit;
      rx_2 = tp_x_scale_ * tp_x_scale_;
      ry_2 = tp_y_scale_ * tp_y_scale_;
    }
    if (hwstate.fingers[i].touch_major) {
      float bias = tp_x_bias_.val_ * sin_2_orit + tp_y_bias_.val_ * cos_2_orit;
      hwstate.fingers[i].touch_major =
          fabsf(hwstate.fingers[i].touch_major - bias) *
          sqrtf(rx_2 * sin_2_orit + ry_2 * cos_2_orit);
    }
    if (hwstate.fingers[i].touch_minor) {
      float bias = tp_x_bias_.val_ * cos_2_orit + tp_y_bias_.val_ * sin_2_orit;
      hwstate.fingers[i].touch_minor =
          fabsf(hwstate.fingers[i].touch_minor - bias) *
          sqrtf(rx_2 * cos_2_orit + ry_2 * sin_2_orit);
    }

    // After calibration, touch_major could be smaller than touch_minor.
    // If so, swap them here and update orientation.
    if (orientation_scale_ &&
        hwstate.fingers[i].touch_major < hwstate.fingers[i].touch_minor) {
      std::swap(hwstate.fingers[i].touch_major,
                hwstate.fingers[i].touch_minor);
      if (hwstate.fingers[i].orientation > 0.0)
        hwstate.fingers[i].orientation -= M_PI_2;
      else
        hwstate.fingers[i].orientation += M_PI_2;
    }

    if (surface_area_from_pressure_.val_) {
      hwstate.fingers[i].pressure *= pressure_scale_.val_;
      hwstate.fingers[i].pressure += pressure_translate_.val_;
    } else {
      if (hwstate.fingers[i].touch_major && hwstate.fingers[i].touch_minor)
        hwstate.fingers[i].pressure = M_PI_4 *
            hwstate.fingers[i].touch_major * hwstate.fingers[i].touch_minor;
      else if (hwstate.fingers[i].touch_major)
        hwstate.fingers[i].pressure = M_PI_4 *
            hwstate.fingers[i].touch_major * hwstate.fingers[i].touch_major;
      else
        hwstate.fingers[i].pressure = 0;
    }

    hwstate.fingers[i].pressure = std::max(1.0f, hwstate.fingers[i].pressure);
  }

  if (!surface_area_from_pressure_.val_) {
    FilterZeroArea(hwstate);
  }
}

void ScalingFilterInterpreter::ConsumeGesture(const Gesture& gs) {
  const char name[] = "ScalingFilterInterpreter::ConsumeGesture";
  LogGestureConsume(name, gs);

  Gesture copy = gs;
  switch (copy.type) {
    case kGestureTypeMove: {
      int original_rel_x =
          copy.details.move.ordinal_dx * mouse_cpi_.val_ / 25.4;
      int original_rel_y =
          copy.details.move.ordinal_dy * mouse_cpi_.val_ / 25.4;
      copy.details.move.dx *= screen_x_scale_;
      copy.details.move.dy *= screen_y_scale_;
      copy.details.move.ordinal_dx *= screen_x_scale_;
      copy.details.move.ordinal_dy *= screen_y_scale_;
      // Special case of motion: if a mouse move of 1 device unit
      // (rel_[xy] == 1) would move the cursor on the screen > 1
      // pixel, force it to just one pixel. This prevents low-DPI mice
      // from jumping 2 pixels at a time when doing slow moves.
      // Note, we use 1 / 1.2 = 0.8333 instead of 1 for the number of
      // screen pixels, as external monitors get a 20% distance boost.
      // Mice are most commonly used w/ external displays.
      if (device_mouse_.val_ &&
          ((original_rel_x == 0) != (original_rel_y == 0))) {
        const double kMinPixels = 1.0 / 1.2;
        if (fabs(copy.details.move.dx) > kMinPixels &&
            abs(original_rel_x) == 1) {
          copy.details.move.dx = copy.details.move.ordinal_dx =
              copy.details.move.dx > 0.0 ? kMinPixels : -kMinPixels;
        }
        if (fabs(copy.details.move.dy) > kMinPixels &&
            abs(original_rel_y) == 1) {
          copy.details.move.dy = copy.details.move.ordinal_dy =
              copy.details.move.dy > 0.0 ? kMinPixels : -kMinPixels;
        }
      }
      break;
    }
    case kGestureTypeScroll:
      if (device_touchpad_.val_) {
        copy.details.scroll.dx *= screen_x_scale_;
        copy.details.scroll.dy *= screen_y_scale_;
        copy.details.scroll.ordinal_dx *= screen_x_scale_;
        copy.details.scroll.ordinal_dy *= screen_y_scale_;
      }
      if (!(invert_scrolling_and_swiping_.val_ ||
            invert_scrolling_only_.val_)) {
        copy.details.scroll.dx *= -1;
        copy.details.scroll.dy *= -1;
        copy.details.scroll.ordinal_dx *= -1;
        copy.details.scroll.ordinal_dy *= -1;
      }
      break;
    case kGestureTypeMouseWheel:
      if (!(invert_scrolling_and_swiping_.val_ ||
            invert_scrolling_only_.val_)) {
        copy.details.wheel.dx *= -1;
        copy.details.wheel.dy *= -1;
        copy.details.wheel.tick_120ths_dx *= -1;
        copy.details.wheel.tick_120ths_dy *= -1;
      }
      break;
    case kGestureTypeFling:
      copy.details.fling.vx *= screen_x_scale_;
      copy.details.fling.vy *= screen_y_scale_;
      copy.details.fling.ordinal_vx *= screen_x_scale_;
      copy.details.fling.ordinal_vy *= screen_y_scale_;
      if (!(invert_scrolling_and_swiping_.val_ ||
            invert_scrolling_only_.val_)) {
        copy.details.fling.vx *= -1;
        copy.details.fling.vy *= -1;
        copy.details.fling.ordinal_vx *= -1;
        copy.details.fling.ordinal_vy *= -1;
      }
      break;
    case kGestureTypeSwipe:
      // Scale swipes, as we want them to follow the pointer speed.
      copy.details.swipe.dx *= screen_x_scale_;
      copy.details.swipe.dy *= screen_y_scale_;
      copy.details.swipe.ordinal_dx *= screen_x_scale_;
      copy.details.swipe.ordinal_dy *= screen_y_scale_;
      if (!invert_scrolling_and_swiping_.val_) {
        copy.details.swipe.dy *= -1;
        copy.details.swipe.ordinal_dy *= -1;
      }
      break;
    case kGestureTypeFourFingerSwipe:
      // Scale swipes, as we want them to follow the pointer speed.
      copy.details.four_finger_swipe.dx *= screen_x_scale_;
      copy.details.four_finger_swipe.dy *= screen_y_scale_;
      copy.details.four_finger_swipe.ordinal_dx *= screen_x_scale_;
      copy.details.four_finger_swipe.ordinal_dy *= screen_y_scale_;
      if (!invert_scrolling_and_swiping_.val_) {
        copy.details.four_finger_swipe.dy *= -1;
        copy.details.four_finger_swipe.ordinal_dy *= -1;
      }
      break;
    default:
      break;
  }

  LogGestureProduce(name, copy);
  ProduceGesture(copy);
}

void ScalingFilterInterpreter::Initialize(const HardwareProperties* hwprops,
                                          Metrics* metrics,
                                          MetricsProperties* mprops,
                                          GestureConsumer* consumer) {
  // If the touchpad doesn't specify its resolution in one or both axes, use a
  // reasonable default instead.
  float res_x = hwprops->res_x != 0 ? hwprops->res_x : 32;
  float res_y = hwprops->res_y != 0 ? hwprops->res_y : 32;

  tp_x_scale_ = 1.0 / res_x;
  tp_y_scale_ = 1.0 / res_y;
  tp_x_translate_ = -1.0 * (hwprops->left * tp_x_scale_);
  tp_y_translate_ = -1.0 * (hwprops->top * tp_y_scale_);

  screen_x_scale_ = 133 / 25.4;
  screen_y_scale_ = 133 / 25.4;

  if (hwprops->orientation_maximum)
    orientation_scale_ =
        M_PI / (hwprops->orientation_maximum -
                hwprops->orientation_minimum + 1);
  else
    orientation_scale_ = 0.0;  // no orientation is provided

  float friendly_orientation_minimum;
  float friendly_orientation_maximum;
  if (orientation_scale_) {
    // Transform minimum and maximum values into radians.
    friendly_orientation_minimum =
        orientation_scale_ * hwprops->orientation_minimum;
    friendly_orientation_maximum =
        orientation_scale_ * hwprops->orientation_maximum;
  } else {
    friendly_orientation_minimum = 0.0;
    friendly_orientation_maximum = 0.0;
  }

  // For haptic touchpads, the pressure field is actual force in grams, not an
  // estimate of surface area.
  if (hwprops->is_haptic_pad && use_touch_size_for_haptic_pad_.val_)
    surface_area_from_pressure_.val_ = false;

  // Make fake idealized hardware properties to report to next_.
  friendly_props_ = *hwprops;
  friendly_props_.left = 0.0;
  friendly_props_.top = 0.0;
  friendly_props_.right = (hwprops->right - hwprops->left) * tp_x_scale_;
  friendly_props_.bottom = (hwprops->bottom - hwprops->top) * tp_y_scale_;
  friendly_props_.res_x = 1.0;  // X pixels/mm
  friendly_props_.res_y = 1.0;  // Y pixels/mm
  friendly_props_.orientation_minimum = friendly_orientation_minimum;
  friendly_props_.orientation_maximum = friendly_orientation_maximum;

  // current metrics is no longer valid, pass metrics=nullptr
  FilterInterpreter::Initialize(&friendly_props_, nullptr, mprops, consumer);
}

}  // namespace gestures
