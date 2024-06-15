#pragma once

#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include <Pcl.h>

struct Zone {
    std::string type;
    std::vector<pcl::PointXYZ> points;
};

class ZoneSplitter {
public:
    std::string SplitToZones(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud);

private:
    std::vector<Zone> Split(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud);
    std::string ClassifyZone(const Zone& sector);
    std::string CreateGeoJson(const std::vector<Zone>& zones);
};