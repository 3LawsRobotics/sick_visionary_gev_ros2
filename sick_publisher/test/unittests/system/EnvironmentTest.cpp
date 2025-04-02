// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#include "system/Environment.hpp"

#include <memory>
#include <string>

#include <gtest/gtest.h>

using namespace sick;

TEST(EnvironmentTest, SetEnvironmentVariable)
{
  std::unique_ptr<Environment> env(Environment::create());

  const char foobarUpper[] = "FOOBAR";
  const char foobarLower[] = "foobar";
  EXPECT_FALSE(env->hasEnvVar(foobarUpper));
  EXPECT_TRUE(env->setEnvVar(foobarUpper, foobarLower));
  EXPECT_TRUE(env->hasEnvVar(foobarUpper));

  std::string value;
  EXPECT_TRUE(env->getEnvVar(foobarUpper, &value));
  ASSERT_EQ(value, foobarLower);
}

TEST(EnvironmentTest, HasEnvironmentVariable)
{
  std::unique_ptr<Environment> env(Environment::create());
  ASSERT_TRUE(env->hasEnvVar("PATH"));
}

TEST(EnvironmentTest, UnsetEnvironmentVariable)
{
  std::unique_ptr<Environment> env(Environment::create());

  const char foobarUpper[] = "FOOBAR";
  const char foobarLower[] = "foobar";

  // first set the environment variable
  EXPECT_TRUE(env->setEnvVar(foobarUpper, foobarLower));
  EXPECT_TRUE(env->hasEnvVar(foobarUpper));
  EXPECT_TRUE(env->unsetEnvVar(foobarUpper));
  ASSERT_FALSE(env->hasEnvVar(foobarUpper));
}

TEST(EnvironmentTest, GetReverseEnvironmentVariable)
{
  std::unique_ptr<Environment> env(Environment::create());
  const char foobarUpper[] = "FOOBAR";
  const char foobarLower[] = "foobar";

  // set variable in UPPER case
  EXPECT_TRUE(env->setEnvVar(foobarUpper, foobarLower));

  // now try to get this variable passing the lower case
  std::string envValue;
  EXPECT_TRUE(env->getEnvVar(foobarLower, &envValue));

  ASSERT_EQ(envValue, foobarLower);
  EXPECT_TRUE(env->unsetEnvVar(foobarUpper));

  const char bar[] = "bar";
  // now for the opposite
  EXPECT_TRUE(env->setEnvVar(foobarLower, bar));
  EXPECT_TRUE(env->getEnvVar(foobarUpper, &envValue));

  ASSERT_EQ(envValue, bar);
  EXPECT_TRUE(env->unsetEnvVar(foobarLower));
}