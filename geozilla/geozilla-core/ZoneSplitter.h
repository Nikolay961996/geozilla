#pragma once

#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include <Pcl.h>
//#include <opencv2/opencv.hpp>

struct Zone {
    std::string type;
    std::vector<pcl::PointXYZ> points;
};

class ZoneSplitter {
public:
    std::string SplitToZones(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud);

private:
    // pcl
    std::vector<Zone> Split(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud);
    std::string ClassifyZone(const Zone& sector);
    std::string CreateGeoJson(const std::vector<Zone>& zones);

    // open-cv
    //cv::Mat PointCloudToImage(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
};