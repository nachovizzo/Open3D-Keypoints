#!/usr/bin/env python3
# @file      iss_detector.py
# @author    Ignacio Vizzo     [ivizzo@uni-bonn.de]
#
# Copyright (c) 2020 Ignacio Vizzo, all rights reserved
import open3d_keypoints as o3d


def main():
    bunny = o3d.io.read_triangle_mesh('../TestData/Bunny.ply')
    bunny_cloud = bunny.sample_points_uniformly(len(bunny.vertices))
    iss_keypoints = o3d.keypoints.compute_iss_keypoints(bunny_cloud)
    o3d.visualization.draw_geometries([iss_keypoints, bunny_cloud])


if __name__ == "__main__":
    main()
