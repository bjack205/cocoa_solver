//
// Created by Brian Jackson on 7/31/22.
// Copyright (c) 2022 Robotic Exploration Lab. All rights reserved.
//

#include <gtest/gtest.h>
#include <fmt/core.h>

#include "cocoa_types.h"

TEST(BasicTests, Addition) {
  EXPECT_EQ(1, 2 - 1);
}

TEST(UtilsTest, SinglePrecisionCheck) {
  if (cocoa_use_single_precision()) {
    fmt::print("Using Single Precision floats.\n");
    EXPECT_EQ(sizeof(cocoa_float), sizeof(float));
  } else {
    fmt::print("Using Double Precision floats.\n");
    EXPECT_EQ(sizeof(cocoa_float), sizeof(double));
  }
}