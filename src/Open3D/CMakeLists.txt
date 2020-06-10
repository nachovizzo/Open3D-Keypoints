add_subdirectory(Keypoints)
add_library(${CMAKE_PROJECT_NAME} $<TARGET_OBJECTS:Keypoints>)
