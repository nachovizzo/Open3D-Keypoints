// @file      ISSDetector.h
// @author    Ignacio Vizzo     [ivizzo@uni-bonn.de]
//
// Copyright (c) 2020 Ignacio Vizzo, all rights reserved
#pragma once

#include <Open3D/Geometry/KDTreeFlann.h>

#include <memory>

#include "Open3D/Geometry/PointCloud.h"

namespace open3d {
namespace keypoints {

class ISSDetector {
public:
    explicit ISSDetector(const std::shared_ptr<geometry::PointCloud>& cloud,
                         double salient_radius = 0.0,
                         double non_max_radius = 0.0)
        : cloud_(cloud),
          kdtree_(*cloud),
          salient_radius_(salient_radius),
          non_max_radius_(non_max_radius) {
        if (salient_radius_ == 0.0 || non_max_radius_ == 0.0) {
            const double resolution = ComputeModelResolution();
            salient_radius_ = 6 * resolution;
            non_max_radius_ = 4 * resolution;
        }
    }

    /// Function to compute ISS keypoints for a point cloud.
    std::shared_ptr<geometry::PointCloud> ComputeKeypoints() const;

protected:
    /// Helper function to compute the scatter matrix for a a point in the input
    /// pointcloud
    Eigen::Matrix3d ComputeScatterMatrix(const Eigen::Vector3d& p) const;

    /// Compute the model resolution;
    double ComputeModelResolution() const;

public:
    /// Input PointCloud where to extract the keypoints
    std::shared_ptr<geometry::PointCloud> cloud_;

    /// KDTree to accelerate nearest neighbour searches
    geometry::KDTreeFlann kdtree_;

    /// The radius of the spherical neighborhood used to compute the scatter
    /// matrix
    double salient_radius_ = 0.0;
    /// The non maxima suppression radius.
    double non_max_radius_ = 0.0;
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

std::shared_ptr<geometry::PointCloud> ComputeISSKeypoints(
        const std::shared_ptr<geometry::PointCloud>& input,
        double salient_radius = 0.0,
        double non_max_radius = 0.0);
}  // namespace keypoints
}  // namespace open3d
