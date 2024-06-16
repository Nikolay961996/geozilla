#include "GeozillaCore.h"

#include <Logger/ConsoleLogger.h>
#include <Loader/GeoModelLoader.h>

#include <CesiumGltf/Model.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace
{

const char* ConvertToRawMemory(const std::string& data)
{
    const auto size = data.size();
    auto* buffer = new char[size + 1];
    buffer[size] = '\0';
    std::copy(std::cbegin(data), std::cend(data), buffer);
    return buffer;
}

} // namespace

const char* GenerateGeoJson(const char* path)
{
    auto loader = gz::core::GeoModelLoader();
#ifdef _DEBUG
    auto logger = std::make_shared<gz::core::ConsoleLogger>();
    loader.SetLogger(logger);
#endif
    loader.Load(path);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->push_back(pcl::PointXYZ(1.0f, 2.0f, 3.0f));

    std::string result = "{}";
    return ConvertToRawMemory(result);
}

void FreeBuffer(const char* buffer)
{
    delete[] buffer;
}
