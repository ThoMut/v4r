/**
 * $Id$
 * 
 * Software License Agreement (GNU General Public License)
 *
 *  Copyright (C) 2015:
 *
 *    Johann Prankl, prankl@acin.tuwien.ac.at
 *    Aitor Aldoma, aldoma@acin.tuwien.ac.at
 *
 *      Automation and Control Institute
 *      Vienna University of Technology
 *      Gusshausstraße 25-29
 *      1170 Vienn, Austria
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @author Johann Prankl, Aitor Aldoma
 *
 */

#ifndef KP_REPROJECTION_ERROR_HPP
#define KP_REPROJECTION_ERROR_HPP

#include <ceres/rotation.h>
#include "v4r/common/impl/Vector.hpp"
#include "v4r/keypoints/impl/invPose.hpp"

namespace v4r
{

//template<class T>
//inline void print_values(const std::string &txt, const T &val1, const T &val2, const T &val3, const T &val4)
//{
//    //Do nothing
//}

//template<>
//inline void print_values<double>(const std::string &txt, const double &val1, const double &val2, const double &val3, const double &val4)
//{
//  std::cout<<txt<<" "<<val1<<" "<<val2<<" "<<val3<<" "<<val4<<std::endl;
//}



// Apply camera intrinsics to the normalized point to get image coordinates.
// This applies the radial lens distortion to a point which is in normalized
// camera coordinates (i.e. the principal point is at (0, 0)) to get image
// coordinates in pixels. Templated for use with autodifferentiation.
template <typename T>
inline void applyRadialDistortionCameraIntrinsics(const T &focal_length_x,
                                                  const T &focal_length_y,
                                                  const T &principal_point_x,
                                                  const T &principal_point_y,
                                                  const T &k1,
                                                  const T &k2,
                                                  const T &k3,
                                                  const T &p1,
                                                  const T &p2,
                                                  const T &normalized_x,
                                                  const T &normalized_y,
                                                  T *image_x,
                                                  T *image_y) {
  T x = normalized_x;
  T y = normalized_y;

  // Apply distortion to the normalized points to get (xd, yd).
  T r2 = x*x + y*y;
  T r4 = r2 * r2;
  T r6 = r4 * r2;
  T r_coeff = (T(1) + k1*r2 + k2*r4 + k3*r6);
  T xd = x * r_coeff + T(2)*p1*x*y + p2*(r2 + T(2)*x*x);
  T yd = y * r_coeff + T(2)*p2*x*y + p1*(r2 + T(2)*y*y);

  // Apply focal length and principal point to get the final image coordinates.
  *image_x = focal_length_x * xd + principal_point_x;
  *image_y = focal_length_y * yd + principal_point_y;
}

// Apply camera intrinsics to the normalized point to get image coordinates.
// Templated for use with autodifferentiation.
template <typename T>
inline void applyCameraIntrinsics(const T &focal_length_x,
                                  const T &focal_length_y,
                                  const T &principal_point_x,
                                  const T &principal_point_y,
                                  const T &normalized_x,
                                  const T &normalized_y,
                                  T *image_x,
                                  T *image_y) {
  // Apply focal length and principal point to get the final image coordinates.
  *image_x = focal_length_x * normalized_x + principal_point_x;
  *image_y = focal_length_y * normalized_y + principal_point_y;
}

// Cost functor which computes reprojection error of 3D point X
// on camera defined by angle-axis rotation and it's translation
// (which are in the same block due to optimization reasons).
//
// This functor uses a radial distortion model.
struct RadialDistortionReprojectionError {
  RadialDistortionReprojectionError(const double &_observed_x, const double &_observed_y)
      : observed_x(_observed_x), observed_y(_observed_y) {}

  template <typename T>
  bool operator()(const T* const intrinsics,
                  const T* const R_t,  // Rotation denoted by angle axis
                                       // followed with translation
                  const T* const X,    // Point coordinates 3x1.
                  T* residuals) const {
    // Unpack the intrinsics.
    const T& focal_length_x    = intrinsics[0];
    const T& focal_length_y    = intrinsics[1];
    const T& principal_point_x = intrinsics[2];
    const T& principal_point_y = intrinsics[3];
    const T& k1                = intrinsics[4];
    const T& k2                = intrinsics[5];
    const T& k3                = intrinsics[6];
    const T& p1                = intrinsics[7];
    const T& p2                = intrinsics[8];

    // Compute projective coordinates: x = RX + t.
    T x[3];

    ceres::AngleAxisRotatePoint(R_t, X, x);
    x[0] += R_t[3];
    x[1] += R_t[4];
    x[2] += R_t[5];

    // Compute normalized coordinates: x /= x[2].
    T xn = x[0] / x[2];
    T yn = x[1] / x[2];

    T predicted_x, predicted_y;

    // Apply distortion to the normalized points to get (xd, yd).
    applyRadialDistortionCameraIntrinsics(focal_length_x,
                                          focal_length_y,
                                          principal_point_x,
                                          principal_point_y,
                                          k1, k2, k3,
                                          p1, p2,
                                          xn, yn,
                                          &predicted_x,
                                          &predicted_y);

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - T(observed_x);
    residuals[1] = predicted_y - T(observed_y);

    return true;
  }

  const double observed_x;
  const double observed_y;
};

// Cost functor which computes reprojection error of 3D point X
// on camera defined by angle-axis rotation and it's translation
// (which are in the same block due to optimization reasons).
struct NoDistortionReprojectionError {
  NoDistortionReprojectionError(const double &_observed_x, const double &_observed_y)
      : observed_x(_observed_x), observed_y(_observed_y) {}

  template <typename T>
  bool operator()(const T* const intrinsics,
                  const T* const R_t,  // Rotation denoted by angle axis
                                       // followed with translation
                  const T* const X,    // Point coordinates 3x1.
                  T* residuals) const {
    // Unpack the intrinsics.
    const T& focal_length_x    = intrinsics[0];
    const T& focal_length_y    = intrinsics[1];
    const T& principal_point_x = intrinsics[2];
    const T& principal_point_y = intrinsics[3];

    // Compute projective coordinates: x = RX + t.
    T x[3];

    ceres::AngleAxisRotatePoint(R_t, X, x);
    x[0] += R_t[3];
    x[1] += R_t[4];
    x[2] += R_t[5];

    // Compute normalized coordinates: x /= x[2].
    T xn = x[0] / x[2];
    T yn = x[1] / x[2];

    T predicted_x, predicted_y;

    // Apply distortion to the normalized points to get (xd, yd).
    applyCameraIntrinsics(focal_length_x, focal_length_y, principal_point_x, principal_point_y,
                          xn, yn, &predicted_x, &predicted_y);

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - T(observed_x);
    residuals[1] = predicted_y - T(observed_y);

    return true;
  }

  const double observed_x;
  const double observed_y;
};

// This functor uses a radial distortion model.
struct RadialDistortionReprojectionAndDepthError {
  RadialDistortionReprojectionAndDepthError(const double &_observed_x, const double &_observed_y, const double &_inv_depth, const double &_depth_err_weight)
      : observed_x(_observed_x), observed_y(_observed_y), inv_depth(_inv_depth),
        depth_err_weight(_depth_err_weight) {}

  template <typename T>
  bool operator()(const T* const intrinsics,
                  const T* const R_t,  // Rotation denoted by angle axis
                                       // followed with translation
                  const T* const X,    // Point coordinates 3x1.
                  T* residuals) const {
    // Unpack the intrinsics.
    const T& focal_length_x    = intrinsics[0];
    const T& focal_length_y    = intrinsics[1];
    const T& principal_point_x = intrinsics[2];
    const T& principal_point_y = intrinsics[3];
    const T& k1                = intrinsics[4];
    const T& k2                = intrinsics[5];
    const T& k3                = intrinsics[6];
    const T& p1                = intrinsics[7];
    const T& p2                = intrinsics[8];

    // Compute projective coordinates: x = RX + t.
    T x[3];

    ceres::AngleAxisRotatePoint(R_t, X, x);
    x[0] += R_t[3];
    x[1] += R_t[4];
    x[2] += R_t[5];

    // Compute normalized coordinates: x /= x[2].
    T xn = x[0] / x[2];
    T yn = x[1] / x[2];

    T predicted_x, predicted_y;

    // Apply distortion to the normalized points to get (xd, yd).
    applyRadialDistortionCameraIntrinsics(focal_length_x,
                                          focal_length_y,
                                          principal_point_x,
                                          principal_point_y,
                                          k1, k2, k3,
                                          p1, p2,
                                          xn, yn,
                                          &predicted_x,
                                          &predicted_y);

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - T(observed_x);
    residuals[1] = predicted_y - T(observed_y);
    residuals[2] = T(depth_err_weight)*(T(1.)/x[2] - T(inv_depth));

    return true;
  }

  const double observed_x;
  const double observed_y;
  const double inv_depth;
  const double depth_err_weight;
};


// Cost functor which computes reprojection and a RGBD-depth error of 3D point X
// on camera defined by angle-axis rotation and it's translation
// (which are in the same block due to optimization reasons).
struct NoDistortionReprojectionAndDepthError {
  NoDistortionReprojectionAndDepthError(const double &_observed_x, const double &_observed_y, const double &_inv_depth,
        const double &_depth_err_weight)
      : observed_x(_observed_x), observed_y(_observed_y), inv_depth(_inv_depth),
        depth_err_weight(_depth_err_weight) {}

  template <typename T>
  bool operator()(const T* const intrinsics,
                  const T* const R_t,  // Rotation denoted by angle axis
                                       // followed with translation
                  const T* const X,    // Point coordinates 3x1.
                  T* residuals) const {
    // Unpack the intrinsics.
    const T& focal_length_x    = intrinsics[0];
    const T& focal_length_y    = intrinsics[1];
    const T& principal_point_x = intrinsics[2];
    const T& principal_point_y = intrinsics[3];

    // Compute projective coordinates: x = RX + t.
    T x[3];

    ceres::AngleAxisRotatePoint(R_t, X, x);
    x[0] += R_t[3];
    x[1] += R_t[4];
    x[2] += R_t[5];

    // Compute normalized coordinates: x /= x[2].
    T xn = x[0] / x[2];
    T yn = x[1] / x[2];

    T predicted_x, predicted_y;

    // Apply distortion to the normalized points to get (xd, yd).
    applyCameraIntrinsics(focal_length_x, focal_length_y, principal_point_x, principal_point_y,
                          xn, yn, &predicted_x, &predicted_y);

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - T(observed_x);
    residuals[1] = predicted_y - T(observed_y);
    residuals[2] = T(depth_err_weight)*(T(1.)/x[2] - T(inv_depth));

//    print_values("dx,dy,dd: ",residuals[0],residuals[1],residuals[2],T(0));

    return true;
  }

  const double observed_x;
  const double observed_y;
  const double inv_depth;
  const double depth_err_weight;
};


// Cost functor which computes reprojection and a RGBD-depth error of 3D point X
// on camera defined by angle-axis rotation and it's translation
// (which are in the same block due to optimization reasons).
struct NoDistortionReprojectionAndDepthErrorGlobalPose {
  NoDistortionReprojectionAndDepthErrorGlobalPose(const double &_observed_x, const double &_observed_y, const double &_inv_depth,
        const double &_depth_err_weight)
      : observed_x(_observed_x), observed_y(_observed_y), inv_depth(_inv_depth),
        depth_err_weight(_depth_err_weight) {}

  template <typename T>
  bool operator()(const T* const intrinsics,
                  const T* const Rt0,  // Rotation denoted by angle axis followed with translation (...to keyframe)
                  const T* const Rt1,  // ..to proj. frame
                  const T* const X,    // Point coordinates 3x1. (in keyframe coordinates)
                  T* residuals) const {
    // Unpack the intrinsics.
    const T& focal_length_x    = intrinsics[0];
    const T& focal_length_y    = intrinsics[1];
    const T& principal_point_x = intrinsics[2];
    const T& principal_point_y = intrinsics[3];

    // transform point to global coordinates
    T xg[3];
    T invRt0[6];
    v4r::invPose6(Rt0, &Rt0[3], invRt0, &invRt0[3]);
    ceres::AngleAxisRotatePoint(invRt0, X, xg);
    xg[0] += invRt0[3];
    xg[1] += invRt0[4];
    xg[2] += invRt0[5];

    // Compute projective coordinates: x = RX + t.
    T x[3];

    ceres::AngleAxisRotatePoint(Rt1, xg, x);
    x[0] += Rt1[3];
    x[1] += Rt1[4];
    x[2] += Rt1[5];

    // Compute normalized coordinates: x /= x[2].
    T xn = x[0] / x[2];
    T yn = x[1] / x[2];

    T predicted_x, predicted_y;

    // Apply distortion to the normalized points to get (xd, yd).
    applyCameraIntrinsics(focal_length_x, focal_length_y, principal_point_x, principal_point_y,
                          xn, yn, &predicted_x, &predicted_y);

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - T(observed_x);
    residuals[1] = predicted_y - T(observed_y);
    residuals[2] = T(depth_err_weight)*(T(1.)/x[2] - T(inv_depth));

//    print_values("dx,dy,dd: ",residuals[0],residuals[1],residuals[2],T(0));

    return true;
  }

  const double observed_x;
  const double observed_y;
  const double inv_depth;
  const double depth_err_weight;
};

// This functor uses a radial distortion model.
struct RadialDistortionReprojectionAndDepthErrorGlobalPose {
  RadialDistortionReprojectionAndDepthErrorGlobalPose(const double &_observed_x, const double &_observed_y, const double &_inv_depth, const double &_depth_err_weight)
      : observed_x(_observed_x), observed_y(_observed_y), inv_depth(_inv_depth),
        depth_err_weight(_depth_err_weight) {}

  template <typename T>
  bool operator()(const T* const intrinsics,
                  const T* const Rt0,  // Rotation denoted by angle axis followed with translation (...to keyframe)
                  const T* const Rt1,  // ..to proj. frame
                  const T* const X,    // Point coordinates 3x1.
                  T* residuals) const {
    // Unpack the intrinsics.
    const T& focal_length_x    = intrinsics[0];
    const T& focal_length_y    = intrinsics[1];
    const T& principal_point_x = intrinsics[2];
    const T& principal_point_y = intrinsics[3];
    const T& k1                = intrinsics[4];
    const T& k2                = intrinsics[5];
    const T& k3                = intrinsics[6];
    const T& p1                = intrinsics[7];
    const T& p2                = intrinsics[8];

    // transform point to global coordinates
    T xg[3];
    T invRt0[6];
    v4r::invPose6(Rt0,&Rt0[3], invRt0,&invRt0[3]);
    ceres::AngleAxisRotatePoint(invRt0, X, xg);
    xg[0] += invRt0[3];
    xg[1] += invRt0[4];
    xg[2] += invRt0[5];

    // Compute projective coordinates: x = RX + t.
    T x[3];

    ceres::AngleAxisRotatePoint(Rt1, xg, x);
    x[0] += Rt1[3];
    x[1] += Rt1[4];
    x[2] += Rt1[5];

    // Compute normalized coordinates: x /= x[2].
    T xn = x[0] / x[2];
    T yn = x[1] / x[2];

    T predicted_x, predicted_y;

    // Apply distortion to the normalized points to get (xd, yd).
    applyRadialDistortionCameraIntrinsics(focal_length_x,
                                          focal_length_y,
                                          principal_point_x,
                                          principal_point_y,
                                          k1, k2, k3,
                                          p1, p2,
                                          xn, yn,
                                          &predicted_x,
                                          &predicted_y);

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - T(observed_x);
    residuals[1] = predicted_y - T(observed_y);
    residuals[2] = T(depth_err_weight)*(T(1.)/x[2] - T(inv_depth));

    return true;
  }

  const double observed_x;
  const double observed_y;
  const double inv_depth;
  const double depth_err_weight;
};



}


#endif










