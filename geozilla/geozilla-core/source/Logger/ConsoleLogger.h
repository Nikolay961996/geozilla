#pragma once

#include "ILogger.h"

namespace gz::core
{

class ConsoleLogger final
    : public ILogger
{
public:
    void Log(const std::string& message) override;
};

} // namespace gz::core
