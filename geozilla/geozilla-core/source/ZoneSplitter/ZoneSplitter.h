#pragma once

#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/opencv.hpp>

typedef pcl::PointXYZRGB PointType;

struct Zone {
    std::string type;
    std::vector<pcl::PointXYZRGB> points;
};

class ZoneSplitter {
public:
    std::string SplitToZones(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud);

private:
    // pcl
    std::vector<Zone> Split(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud);
    std::string ClassifyZone(const Zone& sector);
    std::string CreateGeoJson(const std::vector<Zone>& zones);

    // open-cv
    //cv::Mat PointCloudToImage(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
};
