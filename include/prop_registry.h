// Copyright 2011 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef GESTURES_PROP_REGISTRY_H__
#define GESTURES_PROP_REGISTRY_H__

#include <set>
#include <string>

#include <json/value.h>

#include "include/gestures.h"
#include "include/logging.h"

namespace gestures {

class ActivityLog;
class Property;

class PropRegistry {
 public:
  PropRegistry() : prop_provider_(NULL), activity_log_(NULL) {}

  void Register(Property* prop);
  void Unregister(Property* prop);

  void SetPropProvider(GesturesPropProvider* prop_provider, void* data);
  GesturesPropProvider* PropProvider() const { return prop_provider_; }
  void* PropProviderData() const { return prop_provider_data_; }
  const std::set<Property*>& props() const { return props_; }

  void set_activity_log(ActivityLog* activity_log) {
    activity_log_ = activity_log;
  }
  ActivityLog* activity_log() const { return activity_log_; }

 private:
  GesturesPropProvider* prop_provider_;
  void* prop_provider_data_;
  std::set<Property*> props_;
  ActivityLog* activity_log_;
};

class PropertyDelegate;

class Property {
 public:
  Property(PropRegistry* parent, const char* name,
           PropertyDelegate* delegate)
      : gprop_(NULL), parent_(parent), delegate_(delegate), name_(name) {}

  virtual ~Property() {
    if (parent_)
      parent_->Unregister(this);
  }

  void CreateProp();
  virtual void CreatePropImpl() = 0;
  void DestroyProp();

  // b/253585974
  // delegate used to get passed as a constructor parameter but that
  // led to a chance that the delegate was 'this' and setting the
  // delegate to 'this' in the initializer list allowed the property
  // creation to use 'this' before it was initialized.  This could
  // lead to unexpected behavior and if you were lucky to a crash.
  //
  // Now the delegate is always NULL on initilization of the property
  // instance and after the delegate and property are completely
  // created the user should set the delegate in the constructor
  // body. This will allow access to this in a safe manner.
  void SetDelegate(PropertyDelegate* delegate);

  const char* name() { return name_; }
  // Returns a newly allocated Value object
  virtual Json::Value NewValue() const = 0;
  // Returns true on success
  virtual bool SetValue(const Json::Value& value) = 0;

  static GesturesPropBool StaticHandleGesturesPropWillRead(void* data) {
    GesturesPropBool ret =
        reinterpret_cast<Property*>(data)->HandleGesturesPropWillRead();
    return ret;
  }
  // TODO(adlr): pass on will-read notifications
  virtual GesturesPropBool HandleGesturesPropWillRead() { return 0; }
  static void StaticHandleGesturesPropWritten(void* data) {
    reinterpret_cast<Property*>(data)->HandleGesturesPropWritten();
  }
  virtual void HandleGesturesPropWritten() = 0;

 protected:
  GesturesProp* gprop_;
  PropRegistry* parent_;
  PropertyDelegate* delegate_;

 private:
  const char* name_;
};

class BoolProperty : public Property {
 public:
  BoolProperty(PropRegistry* reg, const char* name, GesturesPropBool val,
               PropertyDelegate* delegate = NULL)
      : Property(reg, name, delegate), val_(val) {
    if (parent_)
      parent_->Register(this);
  }
  virtual void CreatePropImpl();
  virtual Json::Value NewValue() const;
  virtual bool SetValue(const Json::Value& value);
  virtual void HandleGesturesPropWritten();

  GesturesPropBool val_;
};

class BoolArrayProperty : public Property {
 public:
  BoolArrayProperty(PropRegistry* reg, const char* name, GesturesPropBool* vals,
                    size_t count, PropertyDelegate* delegate = NULL)
      : Property(reg, name, delegate), vals_(vals), count_(count) {
    if (parent_)
      parent_->Register(this);
  }
  virtual void CreatePropImpl();
  virtual Json::Value NewValue() const;
  virtual bool SetValue(const Json::Value& list);
  virtual void HandleGesturesPropWritten();

  GesturesPropBool* vals_;
  size_t count_;
};

class DoubleProperty : public Property {
 public:
  DoubleProperty(PropRegistry* reg, const char* name, double val,
                 PropertyDelegate* delegate = NULL)
      : Property(reg, name, delegate), val_(val) {
    if (parent_)
      parent_->Register(this);
  }
  virtual void CreatePropImpl();
  virtual Json::Value NewValue() const;
  virtual bool SetValue(const Json::Value& value);
  virtual void HandleGesturesPropWritten();

  double val_;
};

class DoubleArrayProperty : public Property {
 public:
  DoubleArrayProperty(PropRegistry* reg, const char* name, double* vals,
                      size_t count, PropertyDelegate* delegate = NULL)
      : Property(reg, name, delegate), vals_(vals), count_(count) {
    if (parent_)
      parent_->Register(this);
  }
  virtual void CreatePropImpl();
  virtual Json::Value NewValue() const;
  virtual bool SetValue(const Json::Value& list);
  virtual void HandleGesturesPropWritten();

  double* vals_;
  size_t count_;
};

class IntProperty : public Property {
 public:
  IntProperty(PropRegistry* reg, const char* name, int val,
              PropertyDelegate* delegate = NULL)
      : Property(reg, name, delegate), val_(val) {
    if (parent_)
      parent_->Register(this);
  }
  virtual void CreatePropImpl();
  virtual Json::Value NewValue() const;
  virtual bool SetValue(const Json::Value& value);
  virtual void HandleGesturesPropWritten();

  int val_;
};

class IntArrayProperty : public Property {
 public:
  IntArrayProperty(PropRegistry* reg, const char* name, int* vals, size_t count,
                   PropertyDelegate* delegate = NULL)
      : Property(reg, name, delegate), vals_(vals), count_(count) {
    if (parent_)
      parent_->Register(this);
  }
  virtual void CreatePropImpl();
  virtual Json::Value NewValue() const;
  virtual bool SetValue(const Json::Value& list);
  virtual void HandleGesturesPropWritten();

  int* vals_;
  size_t count_;
};

class StringProperty : public Property {
 public:
  StringProperty(PropRegistry* reg, const char* name, const char* val,
                 PropertyDelegate* delegate = NULL)
      : Property(reg, name, delegate), val_(val) {
    if (parent_)
      parent_->Register(this);
  }
  virtual void CreatePropImpl();
  virtual Json::Value NewValue() const;
  virtual bool SetValue(const Json::Value& value);
  virtual void HandleGesturesPropWritten();

  std::string parsed_val_;
  const char* val_;
};

class PropertyDelegate {
 public:
  virtual void BoolWasWritten(BoolProperty* prop) {};
  virtual void BoolArrayWasWritten(BoolArrayProperty* prop) {};
  virtual void DoubleWasWritten(DoubleProperty* prop) {};
  virtual void DoubleArrayWasWritten(DoubleArrayProperty* prop) {};
  virtual void IntWasWritten(IntProperty* prop) {};
  virtual void IntArrayWasWritten(IntArrayProperty* prop) {};
  virtual void StringWasWritten(StringProperty* prop) {};
};

}  // namespace gestures

#endif  // GESTURES_PROP_REGISTRY_H__
