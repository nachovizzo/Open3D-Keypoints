// @file      ISSKeypointDetector.h
// @author    Ignacio Vizzo     [ivizzo@uni-bonn.de]
//
// Copyright (c) 2020 Ignacio Vizzo, all rights reserved
#pragma once

#include <Open3D/Geometry/KDTreeFlann.h>

#include <memory>

#include "Open3D/Geometry/PointCloud.h"

namespace open3d {
namespace keypoints {

/// Function to compute ISS keypoints for a point cloud.
/// Input PointCloud where to extract the keypoints

/// The radius of the spherical neighborhood used to compute the scatter
/// matrix
/// The non maxima suppression radius.
/// The upper bound on the ratio between the second and the first
/// eigenvalue returned by the EVD.
/// The upper bound on the ratio between the third and the second
/// eigenvalue returned by the EVD.
/// Minimum number of neighbors that has to be found while applying the
/// non maxima suppression algorithm.
std::shared_ptr<geometry::PointCloud> ComputeISSKeypoints(
        const geometry::PointCloud& input,
        double salient_radius = 0.0,
        double non_max_radius = 0.0,
        double gamma_21 = 0.975,
        double gamma_32 = 0.975,
        int min_neighbors = 5);

/// Compute the model resolution;
double ComputeResolution(const geometry::PointCloud& cloud,
                         const geometry::KDTreeFlann& kdtree);

}  // namespace keypoints
}  // namespace open3d
