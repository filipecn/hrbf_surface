/* Copyright (c) 2025, FilipeCN.
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

/// \file   result.h
/// \author FilipeCN (filipedecn@gmail.com)
/// \date   2025-06-07
/// \brief  hrbf_surf return object.

#include <hermes/core/result.h>

#include <sstream>

#pragma once

#define HRBF_SURF_DEBUG

struct HsResult;
namespace hrbf_surf {
template <typename T> using Result = hermes::Result<T, HsResult>;
} // namespace hrbf_surf

struct HsResult {
  enum class Type {
    NoError = 0,    //!< success
    NotFound = 1,   //!< data not found
    ExtError = 2,   //!< third party lib error
    CheckError = 3, //!< a check error ocurred
    IOError = 4,    //!< an io error ocurred
  };

  static HsResult noError() { return {HeError::None, Type::NoError}; }
  static HsResult notFound() { return {HeError::Custom, Type::NotFound}; }
  static HsResult outOfBounds() {
    return {HeError::OutOfBounds, Type::NoError};
  }
  static HsResult error() { return {HeError::Unknown, Type::NoError}; }
  static HsResult extError() { return {HeError::Custom, Type::ExtError}; }
  static HsResult checkError() { return {HeError::Custom, Type::CheckError}; }
  static HsResult inputError() {
    return {HeError::InvalidInput, Type::NoError};
  }
  static HsResult badAllocation() {
    return {HeError::BadAllocation, Type::NoError};
  }
  static HsResult ioError() { return {HeError::Custom, Type::IOError}; }
  static HsResult heError(HeError he) { return {he, Type::NoError}; }

  HsResult() = default;
  HsResult(Type type) : base_type(HeError::Unknown), type(type) {}
  HsResult(HeError base_type, Type type) : base_type(base_type), type(type) {}

  /// \return True if NoError
  operator bool() const { return base_type == HeError::None; }
  template <typename T> operator hrbf_surf::Result<T>() const {
    return hrbf_surf::Result<T>::error(*this);
  }

  HeError base_type{HeError::None};
  Type type{Type::NoError};
};

namespace hermes {

inline std::string to_string(const HsResult &err) {
  std::stringstream ss;
  if (err.base_type != HeError::Custom)
    ss << hermes::to_string(err.base_type);
  if (err.base_type != HeError::None) {
#define VE_ERROR_TYPE_NAME(E)                                                  \
  if (err.type == HsResult::Type::E)                                           \
  ss << " | " << #E
    VE_ERROR_TYPE_NAME(NoError);
    VE_ERROR_TYPE_NAME(NotFound);
    VE_ERROR_TYPE_NAME(ExtError);
    VE_ERROR_TYPE_NAME(CheckError);
    VE_ERROR_TYPE_NAME(IOError);
#undef VE_ERROR_TYPE_NAME
  }
  return ss.str();
}

} // namespace hermes
