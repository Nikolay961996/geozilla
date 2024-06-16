#pragma once

#include "GeoModelLoaderBase.h"

namespace gz::core
{

class GeoModelLoader
    : public GeoModelLoaderBase
{
public:
    GeoModelLoader();
    GeoModelLoader(std::vector<std::shared_ptr<IGeoModelLoader>> supportedLoaders);

    bool IsFileSupported(const std::filesystem::path& path) const override;
    std::vector<GeoModel> Load(const std::filesystem::path& path) override;

protected:
    std::shared_ptr<IGeoModelLoader> FindLoader(const std::filesystem::path& path) const;

private:
    std::vector<std::shared_ptr<IGeoModelLoader>> m_loaders;
};

} // namespace gz::core
