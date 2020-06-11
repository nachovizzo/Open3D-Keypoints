// @file      ISSKeypoint.cpp
// @author    Ignacio Vizzo     [ivizzo@uni-bonn.de]
//
// Copyright (c) 2020 Ignacio Vizzo, all rights reserved

#include <Open3D/IO/ClassIO/PointCloudIO.h>
#include <Open3D/Keypoints/ISSKeypointDetector.h>
#include <Open3D/Open3D.h>

#include <Eigen/Core>
#include <iostream>
#include <memory>
#include <string>

int main(int argc, char *argv[]) {
    using namespace open3d;

    utility::SetVerbosityLevel(utility::VerbosityLevel::Debug);
    if (argc < 2) {
        utility::LogInfo("Open3D {}", OPEN3D_VERSION);
        utility::LogInfo("Usage:");
        utility::LogInfo("\t> {} [filename]\n", argv[0]);
        return 0;
    }

    const std::string filename(argv[1]);
    auto cloud_ptr = std::make_shared<geometry::PointCloud>();
    if (!io::ReadPointCloud(filename, *cloud_ptr)) {
        utility::LogWarning("Failed to read {}\n\n", filename);
        return 1;
    }

    cloud_ptr->EstimateNormals();
    keypoints::ISSKeypointDetector detector;
    auto iss_keypoints = detector.ComputeKeypoints(*cloud_ptr);
    utility::LogInfo("Detected {} keypoints", iss_keypoints->points_.size());

    // Visualize the results
    cloud_ptr->PaintUniformColor(Eigen::Vector3d(0.5, 0.5, 0.5));
    iss_keypoints->PaintUniformColor(Eigen::Vector3d(1.0, 0.75, 0.0));
    visualization::DrawGeometries({iss_keypoints}, "ISS", 1600, 900);

    return 0;
}
