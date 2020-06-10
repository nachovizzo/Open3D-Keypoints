add_executable(ISSKeypoint ISSKeypoint.cpp)
target_link_libraries(ISSKeypoint Keypoints ${Open3D_LIBRARIES})
