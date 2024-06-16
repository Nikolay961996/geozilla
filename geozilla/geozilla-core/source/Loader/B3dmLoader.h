#pragma once

#include "GeoModelLoaderBase.h"

namespace gz::core
{

class B3dmLoader final
    : public GeoModelLoaderBase
{
public:
    bool IsFileSupported(const std::filesystem::path& path) const override;
    std::vector<GeoModel> Load(const std::filesystem::path& path) override;
};

} // namespace gz::core
