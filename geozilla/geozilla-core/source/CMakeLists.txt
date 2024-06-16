set(PROJECT_NAME geozilla-core)

set(SOURCE_LIST
    # DLL
    DllMain.cpp
    GeozillaCore.h
    GeozillaCore.cpp
    GltfToPointCloudConverter.h
    GltfToPointCloudConverter.cpp

    # ZoneSplitter
    ZoneSplitter/ZoneSplitter.h
    ZoneSplitter/ZoneSplitter.cpp

    # Logger
    Logger/ILogger.h
    Logger/ConsoleLogger.h
    Logger/ConsoleLogger.cpp

    # Loader
    Loader/IGeoModelLoader.h
    Loader/GeoModelLoaderBase.h
    Loader/GeoModelLoaderBase.cpp
    Loader/GeoModelLoader.h
    Loader/GeoModelLoader.cpp
    Loader/B3dmLoader.h
    Loader/B3dmLoader.cpp
)

find_package(PCL REQUIRED)

add_library(${PROJECT_NAME} SHARED ${SOURCE_LIST})

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

target_link_libraries(${PROJECT_NAME} CesiumGltf Cesium3DTilesContent ${PCL_LIBRARIES})

add_definitions(${PCL_DEFINITIONS})

set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_STANDARD 20
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/../geozilla-api/bin/x64/$<CONFIG>/net8.0"
)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${SOURCE_LIST})
