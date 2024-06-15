#include "ZoneSplitter.h"

std::string ZoneSplitter::SplitToZones(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud) {
    std::vector<Zone> zones = Split(pointCloud);
    for (auto& zone : zones) {
        zone.type = ClassifyZone(zone);
    }
    std::string geojson = CreateGeoJson(zones);

    return geojson;
}

std::vector<Zone> ZoneSplitter::Split(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud) {
    std::vector<Zone> zones;

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);

    int i = 0;
    while (pointCloud->points.size() > 0.3 * pointCloud->points.size()) {
        seg.setInputCloud(pointCloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0) {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        Zone zone;
        zone.type = "undefined"; // Замените на логику классификации типа

        for (const auto& idx : inliers->indices) {
            zone.points.push_back(pointCloud->points[idx]);
        }

        zones.push_back(zone);

        extract.setInputCloud(pointCloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*pointCloud);
        i++;
    }

    return zones;
}

std::string ZoneSplitter::ClassifyZone(const Zone& sector) {
    // TODO: AI detection
    return "grass";
}

std::string ZoneSplitter::CreateGeoJson(const std::vector<Zone>& zones) {
    json geojson;
    geojson["type"] = "FeatureCollection";
    geojson["features"] = json::array();

    for (const auto& zone : zones) {
        json feature;
        feature["type"] = "Feature";
        feature["properties"]["type"] = zone.type;

        json geometry;
        geometry["type"] = "Polygon";
        geometry["coordinates"] = json::array();

        json coordinates = json::array();
        for (const auto& point : zone.points) {
            coordinates.push_back({ point.x, point.y, point.z });
        }
        geometry["coordinates"].push_back(coordinates);

        feature["geometry"] = geometry;
        geojson["features"].push_back(feature);
    }

    std::string result = geojson.dump(4);

    return result;
}