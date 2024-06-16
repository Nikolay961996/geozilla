#include "GeozillaCore.h"
#include "GltfToPointCloudConverter.h"

#include <Logger/ConsoleLogger.h>
#include <Loader/GeoModelLoader.h>
#include <ZoneSplitter/ZoneSplitter.h>

#include <CesiumGltf/Model.h>

#include <algorithm>

namespace
{

std::vector<gz::core::IGeoModelLoader::GeoModel> LoadGeoModels(const char* path)
{
    if (!path)
        return {};

    auto loader = gz::core::GeoModelLoader();
#ifdef _DEBUG
    loader.SetLogger(std::make_shared<gz::core::ConsoleLogger>());
#endif
    return loader.Load(path);
}

gz::core::GltfToPointCloudConverter::Points::Ptr ConvertToPointCloud(const std::vector<gz::core::IGeoModelLoader::GeoModel>& models)
{
    using namespace gz::core;

    auto pointCloud = std::make_shared<GltfToPointCloudConverter::Points>();

    std::for_each(std::begin(models), std::end(models), [&pc = *pointCloud](const auto& model)
    {
        gz::core::GltfToPointCloudConverter::Convert(model, pc);
    });

    return pointCloud;
}

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
    auto models = LoadGeoModels(path);
    auto pointCloud = ConvertToPointCloud(models);

    auto splitter = ZoneSplitter();
    std::string result = splitter.SplitToZones(pointCloud);
    return ConvertToRawMemory(result);
}

void FreeBuffer(const char* buffer)
{
    delete[] buffer;
}
