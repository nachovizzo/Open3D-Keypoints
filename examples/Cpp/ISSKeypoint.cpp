// @file      ISSKeypoint.cpp
// @author    Ignacio Vizzo     [ivizzo@uni-bonn.de]
//
// Copyright (c) 2020 Ignacio Vizzo, all rights reserved

#include <Open3D/Keypoints/ISSKeypointDetector.h>
#include <Open3D/Open3D.h>

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

    utility::LogInfo("Successfully read {}\n", filename);
    cloud_ptr->NormalizeNormals();

    auto iss_keypoints = keypoints::ComputeISSKeypoints(*cloud_ptr);
    visualization::DrawGeometries({iss_keypoints}, "PointCloud", 1600, 900);

    return 0;
}
