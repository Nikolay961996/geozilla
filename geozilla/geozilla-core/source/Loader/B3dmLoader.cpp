#pragma once

#include "B3dmLoader.h"

#include <CesiumGltf/Model.h>
#include <Cesium3DTilesContent/GltfConverters.h>
#include <Cesium3DTilesContent/B3dmToGltfConverter.h>

namespace
{

class SimpleTaskProcessor
    : public CesiumAsync::ITaskProcessor
{
public:
    virtual void startTask(std::function<void()> f) override
    {
        f();
    }
};

class DummyAssetAccessor final
    : public CesiumAsync::IAssetAccessor
{
public:
    CesiumAsync::Future<std::shared_ptr<CesiumAsync::IAssetRequest>> get(
        const CesiumAsync::AsyncSystem& asyncSystem,
        const std::string& url,
        const std::vector<THeader>& headers) override
    {
        return asyncSystem.createFuture<std::shared_ptr<CesiumAsync::IAssetRequest>>([](const auto& promise)
        {
            promise.reject(std::runtime_error("get operation is not supported"));
        });
    }

    CesiumAsync::Future<std::shared_ptr<CesiumAsync::IAssetRequest>> request(
        const CesiumAsync::AsyncSystem& asyncSystem,
        const std::string& verb,
        const std::string& url,
        const std::vector<THeader>& headers,
        const gsl::span<const std::byte>& contentPayload) override
    {
        return asyncSystem.createFuture<std::shared_ptr<CesiumAsync::IAssetRequest>>([](const auto& promise)
        {
            promise.reject(std::runtime_error("request operation is not supported"));
        });
    }

    void tick() noexcept override
    {
    }
};

Cesium3DTilesContent::AssetFetcher MakeAssetFetcher(const std::string& baseUrl)
{
    static auto asyncSystem = CesiumAsync::AsyncSystem(std::make_shared<SimpleTaskProcessor>());
    auto fileAccessor = std::make_shared<DummyAssetAccessor>();
    auto requestHeaders = std::vector<CesiumAsync::IAssetAccessor::THeader>();
    return Cesium3DTilesContent::AssetFetcher(
        asyncSystem,
        fileAccessor,
        baseUrl,
        glm::dmat4(1.0),
        requestHeaders
    );
}

} // namespace

namespace gz::core
{

bool B3dmLoader::IsFileSupported(const std::filesystem::path& path) const
{
    return path.extension().compare(".b3dm") == 0;
}

std::vector<IGeoModelLoader::GeoModel> B3dmLoader::Load(const std::filesystem::path& path)
{
    auto content = ReadFile(path);
    auto options = CesiumGltfReader::GltfReaderOptions();
    auto assetFetcher = MakeAssetFetcher("");
    auto future = Cesium3DTilesContent::B3dmToGltfConverter::convert(content, options, assetFetcher);
    auto result = future.wait();

    auto models = std::vector<IGeoModelLoader::GeoModel>();
    if (result.model.has_value())
    {
        models.emplace_back(std::move(result.model.value()));
    }
    else
    {
        throw std::runtime_error("Failed to load B3DM file");
    }
    return models;
}

} // namespace gz::core
