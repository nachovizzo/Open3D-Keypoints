// @file      ISSKeypoint.cpp
// @author    Ignacio Vizzo     [ivizzo@uni-bonn.de]
//
// Copyright (c) 2020 Ignacio Vizzo, all rights reserved

#include <Open3D/Keypoints/ISSKeypointDetector.h>
#include <Open3D/Open3D.h>

#include <Eigen/Core>
#include <cstdlib>
#include <memory>
#include <string>

int main(int argc, char *argv[]) {
    using namespace open3d;

    utility::SetVerbosityLevel(utility::VerbosityLevel::Debug);
    if (argc < 3) {
        utility::LogInfo("Open3D {}", OPEN3D_VERSION);
        utility::LogInfo("Usage:");
        utility::LogInfo("\t> {} [mesh|pointcloud] [filename] ...\n", argv[0]);
        return 0;
    }

    const std::string option(argv[1]);
    const std::string filename(argv[2]);
    auto cloud = std::make_shared<geometry::PointCloud>();
    auto mesh = std::make_shared<geometry::TriangleMesh>();
    if (option == "mesh") {
        if (!io::ReadTriangleMesh(filename, *mesh)) {
            utility::LogWarning("Failed to read {}", filename);
            return 1;
        }
        cloud = mesh->SamplePointsUniformly(mesh->vertices_.size());
    } else if (option == "pointcloud") {
        if (!io::ReadPointCloud(filename, *cloud)) {
            utility::LogWarning("Failed to read {}\n\n", filename);
            return 1;
        }
    } else {
        utility::LogError("Options {} not supported\n", option);
    }

    if (argc == 3) {
        utility::LogInfo("Using default parameters");
    } else {
        double salient_radius = std::strtod(argv[3], 0);
        double non_max_radius = std::strtod(argv[4], 0);
        utility::LogDebug("salient_radius = {}, non_max_radius = {}",
                          salient_radius, non_max_radius);
    }

    // Compute the ISS Keypoints
    auto iss_keypoints = std::make_shared<geometry::PointCloud>();
    {
        utility::ScopeTimer timer("ISS estimation");
        iss_keypoints = keypoints::ComputeISSKeypoints(cloud);
        utility::LogInfo("Detected {} keypoints",
                         iss_keypoints->points_.size());
    }
    // Visualize the results
    cloud->PaintUniformColor(Eigen::Vector3d(0.5, 0.5, 0.5));
    iss_keypoints->PaintUniformColor(Eigen::Vector3d(1.0, 0.75, 0.0));
    if (option == "mesh") {
        mesh->PaintUniformColor(Eigen::Vector3d(0.5, 0.5, 0.5));
        mesh->ComputeVertexNormals();
        mesh->ComputeTriangleNormals();
        visualization::DrawGeometries({mesh, iss_keypoints}, "ISS", 1600, 900);
    } else {
        visualization::DrawGeometries({iss_keypoints}, "ISS", 1600, 900);
    }

    return 0;
}
