// @file      ISSKeypointDetector.h
// @author    Ignacio Vizzo     [ivizzo@uni-bonn.de]
//
// Copyright (c) 2020 Ignacio Vizzo, all rights reserved
#pragma once

#include <Open3D/Geometry/KDTreeFlann.h>
#include <Open3D/Geometry/KDTreeSearchParam.h>

#include <memory>

#include "Open3D/Geometry/PointCloud.h"

namespace open3d {
namespace keypoints {

class ISSKeypointDetector {
public:
    /// Function to compute ISS keypoints for a point cloud.
    std::shared_ptr<geometry::PointCloud> ComputeKeypoints(
            const geometry::PointCloud& pcd);

protected:
    /// Helper function to compute the scatter matrix for a a point in the input
    /// pointcloud
    Eigen::Matrix3d ComputeScatterMatrix(const Eigen::Vector3d& p,
                                         const geometry::PointCloud& pcd) const;

    /// Compute the model resolution;
    void ComputeResolution(const geometry::PointCloud& cloud);

public:
    /// KDTree to accelerate nearest neighbour searches
    geometry::KDTreeFlann kdtree_;

    /// The model resolution, to be computed by the method
    double resolution_ = 0.0;
    /// The radius of the spherical neighborhood used to compute the scatter
    /// matrix
    double salient_radius_ = 0.0;
    /// The upper bound on the ratio between the second and the first
    /// eigenvalue returned by the EVD.
    double gamma_21_ = 0.975;
    /// The upper bound on the ratio between the third and the second
    /// eigenvalue returned by the EVD.
    double gamma_32_ = 0.975;
    /// Minimum number of neighbors that has to be found while applying the
    /// non maxima suppression algorithm.
    int min_neighbors_ = 5;
};

}  // namespace keypoints
}  // namespace open3d
