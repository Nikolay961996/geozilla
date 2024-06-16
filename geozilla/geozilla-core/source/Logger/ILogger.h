#pragma once

#include <string>

namespace gz::core
{

struct ILogger
{
    virtual ~ILogger() = default;

    virtual void Log(const std::string& message) = 0;
};

} // namespace gz::core
