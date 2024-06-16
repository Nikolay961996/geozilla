#pragma once

#include "ConsoleLogger.h"

#include <iostream>

namespace gz::core
{

void ConsoleLogger::Log(const std::string& message)
{
    std::cout << message;
}

} // namespace gz::core
