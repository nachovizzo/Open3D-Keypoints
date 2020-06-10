#include "Open3D/Keypoints/ISSKeypointDetector.h"

#include <memory>

#include "Open3D/Geometry/PointCloud.h"

namespace open3d {
namespace keypoints {
std::shared_ptr<geometry::PointCloud> ComputeISSKeypoints(
        const geometry::PointCloud &input) {
    return std::make_shared<geometry::PointCloud>(input);
}
}  // namespace keypoints
}  // namespace open3d
