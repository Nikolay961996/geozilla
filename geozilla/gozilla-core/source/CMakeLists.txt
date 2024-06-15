set(PROJECT_NAME geozilla-core)

set(SOURCE_LIST
    # DLL
    DllMain.cpp
    GeozillaCore.h
    GeozillaCore.cpp

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

add_library(${PROJECT_NAME} SHARED ${SOURCE_LIST})

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(${PROJECT_NAME} CesiumGltf Cesium3DTilesContent)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${SOURCE_LIST})

set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 20)
