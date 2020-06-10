// @file      iss_detector.cpp
// @author    Ignacio Vizzo     [ivizzo@uni-bonn.de]
//
// Copyright (c) 2020 Ignacio Vizzo, all rights reserved
#include "Open3D/Geometry/PointCloud.h"
#include "Open3D/Keypoints/ISSKeypointDetector.h"
#include "open3d_pybind/docstring.h"
#include "open3d_pybind/keypoints/keypoints.h"

namespace open3d {

void pybind_iss_detector(py::module &m) {
    m.def("compute_iss_keypoints", &keypoints::ComputeISSKeypoints,
          "Function to compute ISS keypoints for a point cloud", "input"_a);
    docstring::FunctionDocInject(m, "compute_iss_keypoints",
                                 {{"input", "The Input point cloud."}});
}

}  // namespace open3d
