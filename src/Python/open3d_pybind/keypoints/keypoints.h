// @file      keypoints.h
// @author    Ignacio Vizzo     [ivizzo@uni-bonn.de]
//
// Copyright (c) 2020 Ignacio Vizzo, all rights reserved
#pragma once

#include "open3d_pybind/open3d_pybind.h"

namespace open3d {

void pybind_keypoints(py::module &m);
void pybind_iss_detector(py::module &m);

}  // namespace open3d
