// @file      ISSKeypointDetector.h
// @author    Ignacio Vizzo     [ivizzo@uni-bonn.de]
//
// Copyright (c) 2020 Ignacio Vizzo, all rights reserved
#pragma once

#include <memory>

#include "Open3D/Geometry/PointCloud.h"

namespace open3d {
namespace keypoints {

class ISSKeypointDetector;

/// Function to compute ISS keypoints for a point cloud.
///
/// \param input The Input point cloud.
std::shared_ptr<geometry::PointCloud> ComputeISSKeypoints(
        const geometry::PointCloud &input);

}  // namespace keypoints
}  // namespace open3d
