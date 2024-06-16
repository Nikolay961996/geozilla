#include "GeoModelLoader.h"

#include "B3dmLoader.h"

#include <CesiumGltf/Model.h>

#include <cassert>
#include <sstream>
#include <algorithm>

namespace gz::core
{

GeoModelLoader::GeoModelLoader()
    : m_loaders{ std::make_shared<B3dmLoader>() }
{
}

GeoModelLoader::GeoModelLoader(std::vector<std::shared_ptr<IGeoModelLoader>> supportedLoaders)
    : m_loaders(std::move(supportedLoaders))
{
}

bool GeoModelLoader::IsFileSupported(const std::filesystem::path& path) const
{
    auto loader = FindLoader(path);
    return loader != nullptr;
}

std::vector<IGeoModelLoader::GeoModel> GeoModelLoader::Load(const std::filesystem::path& path)
{
    auto loader = FindLoader(path);
    if (loader)
    {
        loader->SetLogger(GetLogger());

        try
        {
            return loader->Load(path);
        }
        catch (...)
        {
        }
    }

    std::ostringstream oss;
    oss << "Failed to load the file: " << path << std::endl;
    Log(oss.str());

    return {};
}

std::shared_ptr<IGeoModelLoader> GeoModelLoader::FindLoader(const std::filesystem::path& path) const
{
    auto it = std::find_if(std::begin(m_loaders), std::end(m_loaders), [&path](const std::shared_ptr<IGeoModelLoader>& loader)
    {
        assert(loader);
        return loader->IsFileSupported(path);
    });

    return it == std::end(m_loaders) ? nullptr : *it;
}

} // namespace gz::core
