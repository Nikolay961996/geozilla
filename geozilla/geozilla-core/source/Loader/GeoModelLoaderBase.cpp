#pragma once

#include "GeoModelLoaderBase.h"

#include "Logger/ILogger.h"

#include <fstream>

namespace gz::core
{

void GeoModelLoaderBase::SetLogger(std::shared_ptr<ILogger> logger)
{
    m_logger = std::move(logger);
}

std::shared_ptr<ILogger> GeoModelLoaderBase::GetLogger() const
{
    return m_logger;
}

void GeoModelLoaderBase::Log(const std::string& message)
{
    if (m_logger)
    {
        m_logger->Log(message);
    }
}

std::vector<std::byte> GeoModelLoaderBase::ReadFile(const std::filesystem::path& path)
{
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<std::byte> buffer(static_cast<size_t>(size));
    file.read(reinterpret_cast<char*>(buffer.data()), size);

    return buffer;
}

} // namespace gz::core
