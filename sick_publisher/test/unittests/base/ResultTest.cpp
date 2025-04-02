// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#include <memory>
#include <utility>
#include <vector>

#include <base/Result.hpp>
#include <gtest/gtest.h>

namespace sick
{
namespace
{
int testError = 17;
float testSuccess = 23.0f;
} // namespace

TEST(ResultTest, CreateErrorCaseFromVoid)
{
  Result<void, int> result(std::make_unique<int>(testError));
  EXPECT_TRUE(result.isError());
  EXPECT_FALSE(result.isSuccess());

  auto stored = result.getError();
  EXPECT_EQ(*stored, testError);
}

TEST(ResultTest, CreateErrorCaseFromPointer)
{
  Result<float*, int> result(std::make_unique<int>(testError));
  EXPECT_TRUE(result.isError());
  EXPECT_FALSE(result.isSuccess());

  auto stored = result.getError();
  EXPECT_EQ(*stored, testError);
}

TEST(ResultTest, CreateErrorCaseFromVector)
{
  Result<std::vector<float>, int> result(std::make_unique<int>(testError));
  EXPECT_TRUE(result.isError());
  EXPECT_FALSE(result.isSuccess());

  auto stored = result.getError();
  EXPECT_EQ(*stored, testError);
}

TEST(ResultTest, MoveErrorCaseFromVoid)
{
  Result<void, int> result(std::make_unique<int>(testError));
  Result<void, int> moved_result(std::move(result));

  EXPECT_TRUE(moved_result.isError());
  EXPECT_FALSE(moved_result.isSuccess());

  auto stored = moved_result.getError();
  EXPECT_EQ(*stored, testError);
}

TEST(ResultTest, MoveErrorCaseFromPointer)
{
  Result<float*, int> result(std::make_unique<int>(testError));
  Result<float*, int> moved(std::move(result));

  EXPECT_TRUE(moved.isError());
  EXPECT_FALSE(moved.isSuccess());

  auto stored = moved.getError();
  EXPECT_EQ(*stored, testError);
}

TEST(ResultTest, MoveErrorCaseFromVector)
{
  Result<std::vector<float>, int> result(std::make_unique<int>(testError));
  Result<std::vector<float>, int> moved(std::move(result));

  EXPECT_TRUE(moved.isError());
  EXPECT_FALSE(moved.isSuccess());

  auto stored = moved.getError();
  EXPECT_EQ(*stored, testError);
}

TEST(ResultTest, ReturnErrorCaseFromVoid)
{
  Result<void, int> result = []()
  {
    return std::make_unique<int>(testError);
  }();

  EXPECT_TRUE(result.isError());
  EXPECT_FALSE(result.isSuccess());

  auto stored = result.getError();
  EXPECT_EQ(*stored, testError);
}

TEST(ResultTest, ReturnErrorCaseFromPointer)
{
  Result<float*, int> result = []() -> Result<float*, int>
  {
    return {std::make_unique<int>(testError)};
  }();

  EXPECT_TRUE(result.isError());
  EXPECT_FALSE(result.isSuccess());

  auto stored = result.getError();
  EXPECT_EQ(*stored, testError);
}

TEST(ResultTest, ReturnErrorCaseFromVector)
{
  Result<std::vector<float>, int> result = []() -> Result<std::vector<float>, int>
  {
    return std::make_unique<int>(testError);
  }();

  EXPECT_TRUE(result.isError());
  EXPECT_FALSE(result.isSuccess());

  auto stored = result.getError();
  EXPECT_EQ(*stored, testError);
}

TEST(ResultTest, CreateSuccessCaseFromVoid)
{
  Result<void, int> result;
  EXPECT_TRUE(result.isSuccess());
  EXPECT_FALSE(result.isError());
}

TEST(ResultTest, CreateSuccessCaseFromPointer)
{
  Result<float*, int> result(&testSuccess);
  EXPECT_FALSE(result.isError());
  EXPECT_TRUE(result.isSuccess());

  auto stored = result.getSuccess();
  EXPECT_EQ(*stored, testSuccess);

  // Once the success has been delivered, result has an empty payload and no success/error state_
  EXPECT_FALSE(result.isError());
  EXPECT_FALSE(result.isSuccess());
}

TEST(ResultTest, CreateSuccessCaseFromVector)
{
  Result<std::vector<float>, int> result({1.0f});
  EXPECT_FALSE(result.isError());
  EXPECT_TRUE(result.isSuccess());

  auto stored = result.getSuccess();
  EXPECT_EQ(stored[0], 1.0f);

  EXPECT_FALSE(result.isError());
  EXPECT_FALSE(result.isSuccess());
}

TEST(ResultTest, MoveSuccessCaseFromVoid)
{
  Result<void, int> result;
  Result<void, int> moved(std::move(result));
  EXPECT_TRUE(moved.isSuccess());
  EXPECT_FALSE(moved.isError());
}

TEST(ResultTest, MoveSuccessCaseFromPointer)
{
  Result<float*, int> result(&testSuccess);
  Result<float*, int> moved(std::move(result));
  EXPECT_FALSE(moved.isError());
  EXPECT_TRUE(moved.isSuccess());

  auto stored = moved.getSuccess();
  EXPECT_EQ(*stored, testSuccess);

  EXPECT_FALSE(moved.isError());
  EXPECT_FALSE(moved.isSuccess());
}

TEST(ResultTest, MoveSuccessCaseFromVector)
{
  Result<std::vector<float>, int> result({1.0f});
  Result<std::vector<float>, int> moved(std::move(result));
  EXPECT_FALSE(moved.isError());
  EXPECT_TRUE(moved.isSuccess());

  auto stored = moved.getSuccess();
  EXPECT_EQ(stored[0], 1.0f);

  EXPECT_FALSE(moved.isError());
  EXPECT_FALSE(moved.isSuccess());
}

TEST(ResultTest, ReturnSuccessCaseFromVoid)
{
  Result<void, int> result = []() -> Result<void, int>
  {
    return {};
  }();
  EXPECT_TRUE(result.isSuccess());
  EXPECT_FALSE(result.isError());
}

TEST(ResultTest, ReturnSuccessCaseFromPointer)
{
  Result<float*, int*> result = []() -> Result<float*, int*>
  {
    return {&testSuccess};
  }();
  EXPECT_FALSE(result.isError());
  EXPECT_TRUE(result.isSuccess());

  auto stored = result.getSuccess();
  EXPECT_EQ(*stored, testSuccess);

  EXPECT_FALSE(result.isError());
  EXPECT_FALSE(result.isSuccess());
}

TEST(ResultTest, ReturnSuccessCaseFromVector)
{
  Result<std::vector<float>, int> result = []() -> Result<std::vector<float>, int>
  {
    return {{1.0f}};
  }();

  EXPECT_FALSE(result.isError());
  EXPECT_TRUE(result.isSuccess());

  auto stored = result.getSuccess();
  EXPECT_EQ(stored[0], 1.0f);

  EXPECT_FALSE(result.isError());
  EXPECT_FALSE(result.isSuccess());
}

} // namespace sick