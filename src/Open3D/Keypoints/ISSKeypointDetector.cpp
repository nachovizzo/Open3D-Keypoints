// @file      ISSKeypointDetector.cpp
// @author    Ignacio Vizzo     [ivizzo@uni-bonn.de]
//
// Copyright (c) 2020 Ignacio Vizzo, all rights reserved
#include "Open3D/Keypoints/ISSKeypointDetector.h"

#include <Open3D/Geometry/KDTreeFlann.h>
#include <Open3D/Utility/Console.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <memory>
#include <vector>

#include "Eigen/src/Core/Matrix.h"
#include "Open3D/Geometry/PointCloud.h"

namespace open3d {
namespace keypoints {

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
    up /= nb_neighbors;

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

    const auto& points = pcd.points_;
    std::vector<double> third_eigen_values(points.size());
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

    std::vector<bool> feat_max(points.size());
    for (size_t i = 0; i < points.size(); i++) {
        if (third_eigen_values[i] > 0.0) {
            std::vector<int> nn_indices;
            std::vector<double> dist;
            int nb_neighbors = kdtree_.SearchRadius(points[i], salient_radius_,
                                                    nn_indices, dist);

            if (nb_neighbors >= min_neighbors_) {
                bool is_max = true;
                for (int j = 0; j < nb_neighbors; j++) {
                    if (third_eigen_values[i] <
                        third_eigen_values[nn_indices[j]]) {
                        is_max = false;
                    }
                }
                if (is_max) {
                    feat_max[i] = true;
                }
            }
        }
    }

    std::vector<Eigen::Vector3d> keypoints(points.size());
    for (size_t i = 0; i < points.size(); i++) {
        if (feat_max[i]) {
            keypoints.emplace_back(points[i]);
        }
    }

    return std::make_shared<geometry::PointCloud>(keypoints);
}

}  // namespace keypoints
}  // namespace open3d
