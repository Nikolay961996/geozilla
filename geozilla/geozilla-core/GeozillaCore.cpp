#include "Pch.h"

#include "GeozillaCore.h"

#include <string>

namespace
{

const char* ConvertToRawMemory(const std::string& data)
{
    const auto size = data.size();
    auto* buffer = new char[size + 1];
    buffer[size] = '\0';
    std::copy(std::cbegin(data), std::cend(data), buffer);
    return buffer;
}

} // namespace

const char* GenerateGeoJson(const char* pathToB3dm)
{
    std::string result = "{}";
    return ConvertToRawMemory(result);
}

void FreeBuffer(const char* buffer)
{
    delete[] buffer;
}
