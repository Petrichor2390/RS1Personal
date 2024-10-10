#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "yaml-cpp/yaml.h"
#include <chrono>
#include <cmath> 
#include <iostream>
#include <fstream>
#include <vector>
#include <opencv2/opencv.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;
 
/// @brief overlays the generated environment map ontop of the ground truth environment map
class mapOverlay : public rclcpp::Node{
public:
    /// @brief Constructor for node, all code for the mapOverlay runs within the constructor.
    /// Code displays the 2 maps overlayed on one another
    mapOverlay() : Node("mapOverlayNode"){
        auto start = std::chrono::high_resolution_clock::now();

        std::cout << "started mapOverlay script" << std::endl;

        cv::Mat gtImg = cv::imread("/home/charles/roboStu1_ros2_ws/src/sprint3/map/map.pgm", cv::IMREAD_GRAYSCALE);
        cv::Mat mapImg = cv::imread("/home/charles/roboStu1_ros2_ws/src/sprint3/map/my_map.pgm", cv::IMREAD_GRAYSCALE);

        YAML::Node gtImgYAML = YAML::LoadFile("/home/charles/roboStu1_ros2_ws/src/sprint3/map/map.yaml");
        YAML::Node mapImgYAML = YAML::LoadFile("/home/charles/roboStu1_ros2_ws/src/sprint3/map/my_map.yaml");

        auto originGtImg = gtImgYAML["origin"].as<std::vector<double>>();
        auto originMapImg = mapImgYAML["origin"].as<std::vector<double>>();

        double gtImgX = originGtImg[0];
        double gtImgY = originGtImg[1];

        double mapImgX = originMapImg[0];
        double mapImgY = originMapImg[1];

        cv::resize(mapImg, mapImg, cv::Size(), 5, 5);

        cv::resize(mapImg, mapImg, cv::Size(), 0.5, 0.5);
        cv::resize(gtImg, gtImg, cv::Size(), 0.5, 0.5);

        // Trim gtImg based on the offsets
        cv::Rect roi(14, 24, mapImg.cols, mapImg.rows);
        cv::Mat trimmedGtImg = gtImg(roi);

        cv::Mat overlayImg;

        double alpha = 0.5; // Weight for mapImg
        double beta = 1.0 - alpha; // Weight for gtImg

        cv::addWeighted(mapImg, alpha, trimmedGtImg, beta, 0.0, overlayImg);

        //calc computational time
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed = end - start;

        std::cout << "Elapsed time: " << elapsed.count() << " Milli Seconds" << std::endl;

        //show the image
        cv::imshow("Overlay Image", overlayImg);
        cv::waitKey(0);
    }

};
 
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mapOverlay>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}