#include "ZoneSplitter.h"

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>


std::string ZoneSplitter::SplitToZones(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud) {
    std::vector<Zone> zones = Split(pointCloud);
    cv::Mat image = pointCloudToImage(pointCloud);

    for (auto& zone : zones) {
        zone.type = ClassifyZone(zone);
    }
    std::string geojson = CreateGeoJson(zones);

    return geojson;
}

std::vector<Zone> ZoneSplitter::Split(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud) {
    std::vector<Zone> zones;

    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;

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
    cv::Mat image = ZoneToImage(cloud);
    cv::imwrite("output_image.png", image);

    // Вычисление признаков GLCM
    std::vector<double> features = computeGLCMFeatures(image);
    double mean = features[0];
    double variance = features[1];
    double entropy = features[2];
    double contrast = features[3];

    if (entropy < 1.0 && contrast < 0.1) {
        return "road";
    }
    else if (entropy < 1.0 && contrast >= 0.1) {
        return "sidewalk";
    }
    else if (entropy >= 1.0 && variance < 0.5) {
        return "grass";
    }
    else if (variance >= 0.5) {
        return "building";
    }

    return "unknown";
}

std::string ZoneSplitter::CreateGeoJson(const std::vector<Zone>& zones) {
    json geojson;
    geojson["type"] = "FeatureCollection";
    geojson["features"] = json::array();

    for (const auto& sector : sectors) {
        json feature;
        feature["type"] = "Feature";
        feature["properties"]["type"] = sector.type;

        json geometry;
        geometry["type"] = "Polygon";
        geometry["coordinates"] = json::array();

        json coordinates = json::array();
        for (const auto& point : sector.points) {
            coordinates.push_back({ point.x, point.y, point.z });
        }
        geometry["coordinates"].push_back(coordinates);

        feature["geometry"] = geometry;
        geojson["features"].push_back(feature);
    }

    return geojson.dump(4);
}


cv::Mat zoneToImage(Zone& zone) {
    int width = 500;
    int height = 500;
    cv::Mat image(height, width, CV_8UC3, cv::Scalar(0, 0, 0));

    for (const auto& point : zone.points) {
        int x = static_cast<int>((point.x + 10) * 25);
        int y = static_cast<int>((point.y + 10) * 25);
        if (x >= 0 && x < width && y >= 0 && y < height) {
            image.at<cv::Vec3b>(y, x) = cv::Vec3b(point.b, point.g, point.r);
        }
    }

    return image;
}

std::vector<double> computeGLCMFeatures(const cv::Mat& image) {
    cv::Mat grayImage;
    cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);

    cv::Mat glcm = cv::Mat::zeros(256, 256, CV_32F);
    int dx = 1, dy = 0;

    for (int y = 0; y < grayImage.rows - dy; y++) {
        for (int x = 0; x < grayImage.cols - dx; x++) {
            int i = grayImage.at<uchar>(y, x);
            int j = grayImage.at<uchar>(y + dy, x + dx);
            glcm.at<float>(i, j)++;
        }
    }

    glcm /= (grayImage.rows * grayImage.cols);

    std::vector<double> features;

    double mean = 0.0;
    double variance = 0.0;
    double entropy = 0.0;
    double contrast = 0.0;

    for (int i = 0; i < glcm.rows; i++) {
        for (int j = 0; j < glcm.cols; j++) {
            double val = glcm.at<float>(i, j);
            mean += val;
            variance += (i - mean) * (i - mean) * val;
            if (val > 0) {
                entropy -= val * log2(val);
            }
            contrast += (i - j) * (i - j) * val;
        }
    }

    features.push_back(mean);
    features.push_back(variance);
    features.push_back(entropy);
    features.push_back(contrast);

    return features;
}
