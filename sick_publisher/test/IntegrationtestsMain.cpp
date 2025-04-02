// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "system/Environment.hpp"

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);

  std::unique_ptr<sick::Environment> env(sick::Environment::create());
  if (!env->hasEnvVar("CAMERA_AVAILABLE"))
  {
    ::testing::GTEST_FLAG(filter) = "NoCameraAvailable";
  }

  return RUN_ALL_TESTS();
}