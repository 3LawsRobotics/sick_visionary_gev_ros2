// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#ifndef SICK_PUBLISHER_INCLUDE_SICK_PUBLISHER_CAMERAINTRINSICS_HPP
#define SICK_PUBLISHER_INCLUDE_SICK_PUBLISHER_CAMERAINTRINSICS_HPP

namespace sick
{
/// @brief Simple data structure that holds camera-specific parameters
template <typename T = float>
class CameraIntrinsicsT
{
public:
  T focalLength;
  T principalPointU;
  T principalPointV;
  T aspectRatio;
  T scaleC; ///< Scale factor for depth conversion
  T offset; ///< Offset for depth conversion

  CameraIntrinsicsT()
    : focalLength(T(0))
    , principalPointU(T(0))
    , principalPointV(T(0))
    , aspectRatio(T(1))
    , scaleC(T(1))
    , offset(T(0))
  {
  }
};

/// Define a type alias for the default type
using CameraIntrinsics = CameraIntrinsicsT<>;

} // namespace sick

#endif // CAMERA_INTRINSICS_HPP_