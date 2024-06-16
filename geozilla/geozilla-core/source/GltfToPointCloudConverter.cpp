#include "GltfToPointCloudConverter.h"

#include <CesiumGltf/Model.h>

#include <cassert>

namespace gz::core
{

void GltfToPointCloudConverter::Convert(const IGeoModelLoader::GeoModel& model, Points& points)
{
    using CesiumGltf::Model;
    using CesiumGltf::Node;
    using CesiumGltf::Mesh;
    using CesiumGltf::MeshPrimitive;

    auto primitiveCallback = [&points](const Model& gltf, const Node& node, const Mesh& mesh, const MeshPrimitive& primitive, const glm::dmat4& transform)
    {
        assert(primitive.mode == MeshPrimitive::Mode::TRIANGLES);

        auto positionIt = primitive.attributes.find("POSITION");
        if (positionIt == std::end(primitive.attributes))
            return;

        auto accessorIndex = positionIt->second;
        const auto& accessor = gltf.accessors[accessorIndex];
        assert(accessor.type == CesiumGltf::AccessorSpec::Type::VEC3);

        const auto& bufferView = gltf.bufferViews[accessor.bufferView];
        const auto& buffer = gltf.buffers[bufferView.buffer];
        const auto* positions = reinterpret_cast<const float*>(buffer.cesium.data.data() + bufferView.byteOffset + accessor.byteOffset);

        for (size_t i = 0; i < accessor.count; ++i)
        {
            auto x = positions[3 * i + 0];
            auto y = positions[3 * i + 1];
            auto z = positions[3 * i + 2];
            points.emplace_back(x, y, z, 1.0f, 1.0f, 1.0f);
        }
    };

    for (size_t i = 0; i < model.scenes.size(); ++i)
    {
        model.forEachPrimitiveInScene(static_cast<int32_t>(i), primitiveCallback);
    }
}

} // namespace gz::core
