// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#ifndef SICK_PUBLISHER_INCLUDE_BASE_RESULT_HPP
#define SICK_PUBLISHER_INCLUDE_BASE_RESULT_HPP

/// @file Result.hpp
/// @brief The Result type in this class represents the success or failure of an operation, togethr
/// with either the success value or the error itself.
///
/// In the SICK ROS Publisher API there are many (public) functions that do acquire some sort of
/// resource and need to do error checking at the same time. This mandates that the user (client)
/// of this function:
///
///   1. Calls the function
///   2. Checks the status of the previous action
///   3. Does the (optional) error handling or
///   4. Use the acquired resource
///
/// This is cumbersome from a usability perspective and can be avoided by introducing another type
/// in the form of the following class Result<T, E> which follows loosely the notion from Haskell's
/// sum type or Rust's result type where:
///
///     data Result T E = Success T | Error E | Empty
///
/// This type is meant to be used as a return type for all functions that may fail such that Result
/// holds either the required resource (success case), an error object (error case) or an empty
/// case which exists solely for the reason that a Result should/must never be discarded in order
/// to enforce that error handling has to be done. This is of particular interest when a camera
/// is connected and needs proper cleanup.
///
/// Since C++ lacks of a proper sum/result type, this implementation provides templates, or rather
/// template specializations for the following types:
///
///     class Result<void, E>
///     class Result<T*, E>
///     class Result<const T*, E>
///     class Result<T, E> // Fallback definition catching everything else
///
/// @attention
/// Please not that this implementation is part of the public interface for SICK ROS Publisher but
/// can also be used with the private parts of this codebase wherever applicable

#include <cassert>
#include <cstdint>
#include <memory>

#include <base/Compiler.hpp>
#include <type_traits>

namespace sick
{

/// @brief The common interface of the Result<T, E> type
template <typename T, typename E>
class Result;

/// @brief Template specialization for the returning via pointers
template <typename E>
class SICK_NO_DISCARD Result<void, E>
{
public:
  Result() = default;

  Result(std::unique_ptr<E> error)
    : m_error(std::move(error))
  {
  }

  Result(Result<void, E>&& other)
    : m_error(std::move(other.m_error))
  {
  }

  Result<void, E>& operator=(Result<void, E>&& other)
  {
    static_assert(m_error == nullptr);
    m_error = std::move(other.m_error);
    return *this;
  }

  ~Result() { assert(m_error == nullptr); }

  bool isError() const { return m_error != nullptr; }

  bool isSuccess() const { return m_error == nullptr; }

  void getSuccess() {}

  std::unique_ptr<E> getError() { return std::move(m_error); }

private:
  std::unique_ptr<E> m_error;
};

/// @brief Template specialization when both the error and success are pointers.
/// This is implemented as a tagged pointer, where success is 0, so that returning the value is
/// taking the fast path
namespace details
{
/// @brief Utility functions for manipulating the tagged pointer, indicating success or error
enum Payload
{
  Success = 0,
  Error = 1,
  Empty = 2
};

intptr_t createPayload(const void* ptr, Payload type);
Payload getPayload(intptr_t payload);

template <typename T>
static T* getSuccessFromPayload(intptr_t payload)
{
  assert(getPayload(payload) == Success);
  return reinterpret_cast<T*>(payload);
}

template <typename E>
static E* getErrorFromPayload(intptr_t payload)
{
  assert(getPayload(payload) == Error);
  return reinterpret_cast<E*>(payload ^ 1);
}

constexpr static intptr_t g_emptyPayload = Empty;
} // namespace details

template <typename T, typename E>
class SICK_NO_DISCARD Result<T*, E>
{
public:
  static_assert(std::alignment_of<T>::value >= 4, "Result<T*, E*> needs two bits for tagged pointers");
  static_assert(std::alignment_of<E>::value >= 4, "Result<T*, E*> needs two bits for tagged pointers");

  Result(T* success)
    : m_payload(details::createPayload(success, details::Success))
  {
  }

  Result(std::unique_ptr<E> error)
    : m_payload(details::createPayload(error.release(), details::Error))
  {
  }

  // Return a Result<T*, E*> from a Result<TChild*, E*>
  template <typename TChild>
  Result(Result<TChild*, E>&& other)
    : m_payload(other.m_payload)
  {
    other.m_payload = details::g_emptyPayload;
    static_assert(std::is_same<T, TChild>::value || std::is_base_of<T, TChild>::value, "");
  }

  template <typename TChild>
  Result<T*, E>& operator=(Result<TChild*, E>&& other)
  {
    static_assert(m_payload == details::g_emptyPayload);
    static_assert(std::is_same<T, TChild>::value || std::is_base_of<T, TChild>::value, "");
    m_payload = other.m_payload;
    other.m_payload = details::g_emptyPayload;
    return *this;
  }

  ~Result() { assert(m_payload == details::g_emptyPayload && "Either success or error has to be handled explicitly"); }

  bool isError() const { return details::getPayload(m_payload) == details::Error; }

  bool isSuccess() const { return details::getPayload(m_payload) == details::Success; }

  T* getSuccess()
  {
    T* success = details::getSuccessFromPayload<T>(m_payload);
    m_payload = details::g_emptyPayload;
    return success;
  }

  std::unique_ptr<E> getError()
  {
    std::unique_ptr<E> error(details::getErrorFromPayload<E>(m_payload));
    m_payload = details::g_emptyPayload;
    return error;
  }

private:
  template <typename T2, typename E2>
  friend class Result;

  intptr_t m_payload = details::g_emptyPayload;
};

template <typename T, typename E>
class SICK_NO_DISCARD Result<const T*, E>
{
public:
  static_assert(std::alignment_of<T>::value >= 4, "Result<T*, E*> needs two bits for tagged pointers");
  static_assert(std::alignment_of<E>::value >= 4, "Result<T*, E*> needs two bits for tagged pointers");

  Result(const T* success)
    : m_payload(details::createPayload(success, details::Success))
  {
  }

  Result(std::unique_ptr<E> error)
    : m_payload(details::createPayload(error.release(), details::Error))
  {
  }

  Result(Result<const T*, E>&& other) noexcept
    : m_payload(other.m_payload)
  {
    other.m_payload = details::g_emptyPayload;
  }

  Result<const T*, E>& operator=(Result<const T*, E>&& other)
  {
    static_assert(m_payload == details::g_emptyPayload);
    m_payload = other.m_payload;
    other.m_payload = details::g_emptyPayload;
    return *this;
  }

  ~Result() { static_assert(m_payload == details::g_emptyPayload); }

  bool isError() const { return details::getPayload(m_payload) == details::Error; }

  bool isSuccess() const { return details::getPayload(m_payload) == details::Success; }

  const T* getSuccess()
  {
    T* success = details::getSuccessFromPayload<T>(m_payload);
    m_payload = details::g_emptyPayload;
    return success;
  }

  std::unique_ptr<E> getError()
  {
    std::unique_ptr<E> error(details::getErrorFromPayload<E>(m_payload));
    m_payload = details::g_emptyPayload;
    return std::move(error);
  }

private:
  intptr_t m_payload = details::g_emptyPayload;
};

template <typename T, typename E>
class SICK_NO_DISCARD Result
{
public:
  Result(T&& success)
    : m_type(Success)
    , m_success(std::move(success))
  {
  }

  Result(std::unique_ptr<E> error)
    : m_type(Error)
    , m_error(std::move(error))
  {
  }

  Result(Result<T, E>&& other) noexcept
    : m_type(other.m_type)
    , m_error(std::move(other.m_error))
    , m_success(std::move(other.m_success))
  {
    other.m_type = Received;
  }

  Result<T, E>& operator=(Result<T, E>&& other)
  {
    m_type = other.m_type;
    m_error = std::move(other.m_error);
    m_success = std::move(other.m_success);
    other.m_type = Received;
    return *this;
  }

  ~Result() { assert(m_type == Received && "Either success or error has to be handled explicitly"); }

  bool isError() const { return m_type == Error; }

  bool isSuccess() const { return m_type == Success; }

  T&& getSuccess()
  {
    assert(m_type == Success);
    m_type = Received;
    return std::move(m_success);
  }

  std::unique_ptr<E> getError()
  {
    assert(m_type == Error);
    m_type = Received;
    return std::move(m_error);
  }

private:
  enum Payload
  {
    Success = 0,
    Error = 1,
    Received = 2,
  };

  Payload m_type;
  std::unique_ptr<E> m_error;
  T m_success;
};
} // namespace sick

#endif // SICK_PUBLISHER_INCLUDE_BASE_RESULT_HPP
