// @file      ISSKeypointDetector.cpp
// @author    Ignacio Vizzo     [ivizzo@uni-bonn.de]
//
// Copyright (c) 2020 Ignacio Vizzo, all rights reserved
#include "Open3D/Keypoints/ISSKeypointDetector.h"

#include <Open3D/Geometry/KDTreeFlann.h>
#include <Open3D/Utility/Console.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <cmath>
#include <memory>
#include <vector>

#include "Eigen/src/Core/Matrix.h"
#include "Open3D/Geometry/PointCloud.h"

namespace open3d {
namespace keypoints {

void ISSKeypointDetector::ComputeResolution(const geometry::PointCloud& cloud) {
    std::vector<int> indices(2);
    std::vector<double> distances(2);
#pragma omp parallel for reduction(+ : resolution_)
    for (size_t i = 0; i < cloud.points_.size(); i++) {
        if (kdtree_.SearchKNN(cloud.points_[i], 2, indices, distances) != 0) {
            resolution_ += std::sqrt(distances[1]);
        }
    }
    resolution_ /= cloud.points_.size();
}

Eigen::Matrix3d ISSKeypointDetector::ComputeScatterMatrix(
        const Eigen::Vector3d& p, const geometry::PointCloud& pcd) const {
    std::vector<int> indices;
    std::vector<double> dist;
    int nb_neighbors = kdtree_.SearchRadius(p, salient_radius_, indices, dist);
    if (nb_neighbors < min_neighbors_) {
        return {};
    }

    // sample mean vector
    Eigen::Vector3d up = Eigen::Vector3d::Zero();
    for (const auto& n_idx : indices) {
        up += pcd.points_[n_idx];
    }

    // Compute the scatter matrix
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto& n_idx : indices) {
        const auto& n_point = pcd.points_[n_idx];
        cov += (n_point - up) * (n_point - up).transpose();
    }
    return cov;
}

std::shared_ptr<geometry::PointCloud> ISSKeypointDetector::ComputeKeypoints(
        const geometry::PointCloud& pcd) {
    if (!pcd.HasNormals()) {
        utility::LogError("[ComputeKeypoints] pcd has no normals");
    }

    kdtree_.SetGeometry(pcd);
    ComputeResolution(pcd);
    // TODO: 6 and 4 are magic numbers
    salient_radius_ = 8 * resolution_;
    const double non_max_radius = 4 * resolution_;

    const auto& points = pcd.points_;
    std::vector<double> third_eigen_values(points.size());
#pragma omp parallel for shared(third_eigen_values)
    for (size_t i = 0; i < points.size(); i++) {
        Eigen::Matrix3d cov = ComputeScatterMatrix(points[i], pcd);
        if (cov.isZero()) {
            continue;
        }

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
        const double& e1c = solver.eigenvalues()[2];
        const double& e2c = solver.eigenvalues()[1];
        const double& e3c = solver.eigenvalues()[0];

        if ((e2c / e1c) < gamma_21_ && e3c / e2c < gamma_32_) {
            third_eigen_values[i] = e3c;
        }
    }

    // TODO: Extract this from here
    std::vector<Eigen::Vector3d> keypoints;
    keypoints.reserve(points.size());
#pragma omp parallel for shared(keypoints)
    for (size_t i = 0; i < points.size(); i++) {
        if (third_eigen_values[i] > 0.0) {
            std::vector<int> nn_indices;
            std::vector<double> dist;
            int nb_neighbors = kdtree_.SearchRadius(points[i], non_max_radius,
                                                    nn_indices, dist);

            if (nb_neighbors >= min_neighbors_) {
                bool is_max = true;
                for (const auto& n_idx : nn_indices) {
                    if (third_eigen_values[i] < third_eigen_values[n_idx]) {
                        is_max = false;
                    }
                }
                if (is_max) {
                    keypoints.emplace_back(points[i]);
                }
            }
        }
    }

    return std::make_shared<geometry::PointCloud>(keypoints);
}

}  // namespace keypoints
}  // namespace open3d
