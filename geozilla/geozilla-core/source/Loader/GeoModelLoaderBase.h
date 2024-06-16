#pragma once

#include "IGeoModelLoader.h"

#include <string>
#include <cstddef>

namespace gz::core
{

class GeoModelLoaderBase
    : public IGeoModelLoader
{
public:
    void SetLogger(std::shared_ptr<ILogger> logger) override;

protected:
    std::shared_ptr<ILogger> GetLogger() const;
    void Log(const std::string& message);

    std::vector<std::byte> ReadFile(const std::filesystem::path& path);

private:
    std::shared_ptr<ILogger> m_logger;
};

} // namespace gz::core
