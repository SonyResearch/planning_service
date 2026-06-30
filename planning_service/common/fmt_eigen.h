/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file fmt.h
#pragma once

#include <Eigen/Dense>
#include <fmt/format.h>

namespace fmt {
/** Formatter for Eigen types. */
template <typename EigenType>
struct EigenFormatter : formatter<std::string_view> {
  template <typename FormatContext>
  auto format(const EigenType& mat, FormatContext& ctx) const {
    std::stringstream ss;
    ss << mat;
    return formatter<std::string_view>::format(ss.str(), ctx);
  }
};

/** Matrix specification. */
template <typename Scalar, int Rows, int Cols, int Options, int MaxRows,
          int MaxCols>
struct formatter<Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>>
    : EigenFormatter<
          Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>> {};

/** Transpose specification. */
template <typename EigenType>
struct formatter<Eigen::Transpose<EigenType>>
    : EigenFormatter<Eigen::Transpose<EigenType>> {};

/** CwiseBinaryOp specification. */
template <typename BinaryOp, typename Lhs, typename Rhs>
struct formatter<Eigen::CwiseBinaryOp<BinaryOp, Lhs, Rhs>>
    : EigenFormatter<Eigen::CwiseBinaryOp<BinaryOp, Lhs, Rhs>> {};
}  // namespace fmt
