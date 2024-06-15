#pragma once

#include <memory>
#include <vector>
#include <filesystem>

namespace CesiumGltf
{

struct Model;

} // CesiumGltf

namespace gz::core
{

struct ILogger;

struct IGeoModelLoader
{
    using GeoModel = CesiumGltf::Model;

    virtual ~IGeoModelLoader() = default;

    virtual void SetLogger(std::shared_ptr<ILogger> logger) = 0;
    virtual bool IsFileSupported(const std::filesystem::path& path) const = 0;
    virtual std::vector<GeoModel> Load(const std::filesystem::path& path) = 0;
};

} // namespace gz::core
