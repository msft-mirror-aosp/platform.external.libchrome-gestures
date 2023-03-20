// Copyright 2012 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <gtest/gtest.h>

#include "include/macros.h"
#include "include/util.h"

namespace gestures {

class UtilTest : public ::testing::Test {};

TEST(UtilTest, DistSqTest) {
  FingerState fs[] = {
    // TM, Tm, WM, Wm, Press, Orientation, X, Y, TrID
    {0, 0, 0, 0, 1, 0, 1, 2, 1, 0},
    {0, 0, 0, 0, 1, 0, 4, 6, 1, 0}
  };
  EXPECT_FLOAT_EQ(DistSq(fs[0], fs[1]), 25);
  EXPECT_FLOAT_EQ(DistSqXY(fs[0], 4, 6), 25);
}

TEST(UtilTest, ListAtTest) {
  const int kMaxElements = 3;
  struct element {
    int x;
  };

  typedef std::list<element> ElemListType;
  typedef std::optional<std::reference_wrapper<element>> OptionalRefElem;

  struct ElemList : public ElemListType {
    OptionalRefElem at(int offset) {
      return ListAt<OptionalRefElem, ElemListType>(*this, offset);
    }
  } list;

  for (auto i = 0; i < kMaxElements; ++i) {
    auto& elem = list.emplace_back();
    elem.x = i;
  }
  EXPECT_EQ(list.at(-1)->get().x, list.at(list.size() - 1)->get().x);

  for (auto i = 0; i < kMaxElements; ++i) {
    for (auto j = 0; j < kMaxElements; ++j) {
      if (i == j) {
        EXPECT_EQ(list.at(i)->get().x, list.at(j)->get().x);
        EXPECT_EQ(&(list.at(i)->get()), &(list.at(j)->get()));
      } else {
        EXPECT_NE(list.at(i)->get().x, list.at(j)->get().x);
        EXPECT_NE(&(list.at(i)->get()), &(list.at(j)->get()));
      }
    }
  }

  int list_count = list.size();
  EXPECT_EQ(list.at(list_count), std::nullopt);
  EXPECT_NE(list.at(list_count - 1), std::nullopt);
  EXPECT_EQ(list.at(-list_count - 1), std::nullopt);
  EXPECT_NE(list.at(-list_count), std::nullopt);
}

}  // namespace gestures
