#include "GeozillaCore.h"

#include <Cesium3DTilesReader/TilesetReader.h>

#include <string>
#include <vector>
#include <cstddef>
#include <fstream>
#include <filesystem>

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

std::vector<std::byte> ReadFile(const std::filesystem::path& fileName)
{
    std::ifstream file(fileName, std::ios::binary | std::ios::ate);
    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<std::byte> buffer(static_cast<size_t>(size));
    file.read(reinterpret_cast<char*>(buffer.data()), size);

    return buffer;
}

} // namespace

const char* GenerateGeoJson(const char* path)
{
    //auto data = ReadFile("F:/personal/Hackaton/Resources/FGM_HACKATON/tileset_hacaton.json");
    //auto* tilesetReader = new Cesium3DTilesReader::TilesetReader();
    //auto tileResult = tilesetReader->readFromJson(data);
    //if (!tileResult.value)
    //    return nullptr;

    std::string result = "{}";
    return ConvertToRawMemory(result);
}

void FreeBuffer(const char* buffer)
{
    delete[] buffer;
}
