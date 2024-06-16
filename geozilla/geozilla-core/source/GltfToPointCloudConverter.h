#pragma once

#include "Loader/IGeoModelLoader.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace gz::core
{

class GltfToPointCloudConverter
{
public:
    using Points = pcl::PointCloud<pcl::PointXYZRGB>;

    static void Convert(const IGeoModelLoader::GeoModel& model, Points& points);
};

} // namespace gz::core
