#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "yaml-cpp/yaml.h"
#include <chrono>
#include <cmath> 
#include <iostream>
#include <algorithm>
#include <fstream>
#include <vector>
#include <opencv2/opencv.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;
 
/// @brief subsribes to odometry and laser scan to find if there is a cylinder around the robot. when it finds a cylinder it displays an image of the generated map with a cylinder overlaid
class objectExclude : public rclcpp::Node{
public:
    /// @brief Constructor for the node, imports required files and initialises variables to starting values
    objectExclude() : Node("objectExcludeNode"){
        std::cout << "started mapFilter script" << std::endl;
 
        //map files
        std::string pgm_fileOriginal = "/home/charles/roboStu1_ros2_ws/src/sprint2/map/my_map.pgm";
        std::string yaml_fileOriginal = "/home/charles/roboStu1_ros2_ws/src/sprint2/map/my_map.yaml";

        //ros subs, service and etc.
        laserSub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&objectExclude::scan_callback, this, std::placeholders::_1));
        odomSub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&objectExclude::odom_callback, this, std::placeholders::_1));

        //inistialising variables
        XposGT_ = 0;
        YposGT_ = 0;
        yawGT_ = 0;
        scanRange_ = 0;
        cylinderX_ = 0;
        cylinderY_ = 0;
        cylinderFound_ = false;        
    }
 
private:
    /// @brief Predicate function used to remove nan values from vector of geometry points
    /// @param point point to be considered
    /// @return false if either x or y is nan, otherwise true
    static bool contains_nan(const geometry_msgs::msg::Point& point) {
        return std::isnan(point.x) || std::isnan(point.y);
    }

    /// @brief callback for the /scan topic recieves the laser scan data from gazebo and processes it into segments (vectors of clustered points in global frame)
    /// @param msg laserscan data from topic
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
        //grab meta data
        auto ranges = msg->ranges;
        float angle_min = msg->angle_min;
        float angle_increment = msg->angle_increment;
        scanRange_ = msg->range_max;

        //find points in local frame
        std::vector<geometry_msgs::msg::Point> local_points;
        for (size_t i = 0; i < ranges.size(); ++i) {
            float range = ranges[i];
            float angle = angle_min + i*angle_increment;

            // Convert to Cartesian coordinates
            float x = range * cos(angle);
            float y = range * sin(angle);

            geometry_msgs::msg::Point point;
            point.x = x;
            point.y = y;
            point.z = 0.0;  // Since it's a 2D scan
            local_points.push_back(point);
        }

        //find points in global frame
        std::vector<geometry_msgs::msg::Point> global_points;
        for (auto& point : local_points) {
            geometry_msgs::msg::Point global_point;

            // Apply transformation to global frame
            global_point.x = cos(yawGT_) * point.x - sin(yawGT_) * point.y + XposGT_;
            global_point.y = sin(yawGT_) * point.x + cos(yawGT_) * point.y + YposGT_;
            global_point.z = point.z;  // Same in 2D
            global_points.push_back(global_point);
        }

        //reomove points that contain nan
        global_points.erase(std::remove_if(global_points.begin(), global_points.end(), contains_nan), global_points.end());

        //number of global points
        // std::cout << "number of global points from scan: " << global_points.size() << std::endl;

        //if there are not enough points to analyse then skip
        if(global_points.size() < 2){
            return;
        }

        //convert to segments
        double segmentTolerance = 0.175; //maximum distance in meteres between consective points to be considered a segment
        std::vector<std::vector<geometry_msgs::msg::Point>> segments;
        std::vector<geometry_msgs::msg::Point> tempSegment;
        tempSegment.push_back(global_points.at(0)); //the first point belongs to the first segment by default
        for(size_t i = 0; i < (global_points.size()-1); i++){
            //distance between current point [i] and the next point
            geometry_msgs::msg::Point p1 = global_points.at(i);
            geometry_msgs::msg::Point p2 = global_points.at(i+1); 
            
            if(distanceBetweenPoints(p1, p2) < segmentTolerance){ //within tolerance add to current segment
                tempSegment.push_back(p2);
            }else{
                //out of tolerance - segment finished add to segments
                segments.push_back(tempSegment);
                tempSegment.clear();
                //add p2 as the first point in the new segment
                tempSegment.push_back(p2);
            }
        }
        segments.push_back(tempSegment); //push back the final segment as this isn't done otherwise
        segments_ = segments;
        findCylinder();
    }

    /// @brief callback for the /odom topic used as ground truth for locating the global positioning of the cylinder
    /// @param msg odometry data from topic
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
        //bring in pos
        XposGT_ = msg->pose.pose.position.x;
        YposGT_ = msg->pose.pose.position.y;

        //calculate yaw
        double x = msg->pose.pose.orientation.x;
        double y = msg->pose.pose.orientation.y;
        double z = msg->pose.pose.orientation.z;
        double w = msg->pose.pose.orientation.w;
        yawGT_ = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    }

    /// @brief generate a set of points in a circle around a given point 
    /// @param center the center of the circle of points to be generated
    /// @param radius the radius of circle of points to be generated
    /// @param num_points number of equally spaced points in the generated circle
    /// @return vector of points
    std::vector<geometry_msgs::msg::Point> generateCirclePoints(geometry_msgs::msg::Point center, double radius, int num_points) {
        std::vector<geometry_msgs::msg::Point> points;
        double angle_increment = 2 * M_PI / num_points; // 360 degrees divided by the number of points

        for (int i = 0; i < num_points; ++i) {
            geometry_msgs::msg::Point point;
            double angle = i * angle_increment;
            point.x = center.x + radius * cos(angle); // x = r * cos(theta)
            point.y = center.y + radius * sin(angle); // y = r * sin(theta)
            point.z = center.z; // keep the z coordinate the same
            points.push_back(point);
        }

        return points;
    }

    /// @brief generates a set of points along a line from the given position to the max range of the laser scanner data received previously
    /// @param Xpos starting x position of the line 
    /// @param Ypos starting y position of the line
    /// @param yaw yaw the line propagates away from the starting point at
    /// @param num_points number of points equally spaced along the line between the start pos specified and the end of the laserscanners range
    /// @return vector of points
    std::vector<geometry_msgs::msg::Point> generatePointsAlongLine(double Xpos, double Ypos, double yaw, int num_points){
        std::vector<geometry_msgs::msg::Point> points;

        // Calculate the step size for each point along the line
        double step_distance = scanRange_ / num_points;

        // Unit vector in the direction of the yaw
        double dx = std::cos(yaw);
        double dy = std::sin(yaw);

        // Generate points along the line
        for (int i = 0; i <= num_points; ++i) {
            geometry_msgs::msg::Point point;
            point.x = Xpos + i*step_distance*dx;
            point.y = Ypos + i*step_distance*dy;
            point.z = 0.0; //2d
            points.push_back(point);
        }

        return points;
    }

    /// @brief calculates the euclidean distance between 2 points
    /// @param p1 point 1 to be considered
    /// @param p2 point 2 to be considered
    /// @return distance
    double distanceBetweenPoints(geometry_msgs::msg::Point p1, geometry_msgs::msg::Point p2){
        return std::hypot(p1.x - p2.x, p1.y - p2.y);
    }

    /// @brief calculates the number of points in a given set of points that are within a given range of a given point
    /// @param p point around which other points are considered to in or out of range
    /// @param pSet set of points being counted as in or out of range of p point
    /// @param radius tolerance between p and pSet
    /// @return number of points within radius distance of p within pSet
    int numberOfNearbyPoints(geometry_msgs::msg::Point p, std::vector<geometry_msgs::msg::Point> pSet, double radius){
        int count = 0;
        for(size_t i = 0; i < pSet.size(); i++){
            if(distanceBetweenPoints(p, pSet.at(i)) < radius){
                count++;
            }
        }
        return count;
    }

    /// @brief prints x and y position of points to terminal
    /// @param pSet points to be printed
    void printPoints(std::vector<geometry_msgs::msg::Point> pSet){
        for(size_t i = 0; i < pSet.size(); i++){
            std::cout << "Xpos: " << pSet.at(i).x << ", Ypos: " << pSet.at(i).y << std::endl;
        }
    }

    /// @brief finds the global yaw of the median point of a set of points (segment)
    /// @param segment set of points considered
    /// @return global yaw
    double yawofSegment(std::vector<geometry_msgs::msg::Point> segment){
        int middleIndex = static_cast<int>(segment.size()/2);

        double dx = segment.at(middleIndex).x - XposGT_;
        double dy = segment.at(middleIndex).y - YposGT_;

        double yaw = atan2(dy, dx);

        return yaw;
    }

    /// @brief calculates if a given segment should be ignored as a possible cylinder based on if the average position of the segment is too far away from the robot
    /// @param segment the segment to ignored or not
    /// @return true for exclude false for include
    bool excludeSegment(std::vector<geometry_msgs::msg::Point> segment){
        bool exclude = false;

        // printPoints(segment);
        //if the segment is too far away on average exclude
        //find the average of all points
        double sumX = 0;
        double sumY = 0;
        for(const auto& point : segment){
            sumX += point.x;
            sumY += point.y;
        }

        geometry_msgs::msg::Point segmentAvgPoint;
        segmentAvgPoint.x = sumX/segment.size();
        segmentAvgPoint.y = sumY/segment.size();
        segmentAvgPoint.z = 0;

        geometry_msgs::msg::Point robotPos;
        robotPos.x = XposGT_;
        robotPos.y = YposGT_;
        robotPos.z = 0;

        //find the distance between robot and segment
        if(distanceBetweenPoints(segmentAvgPoint, robotPos) > 1.5){
            exclude = true;
        }

        return exclude;
    }

    /// @brief calculates based on privately stored data if there is a cylinder within the range of the robot
    void findCylinder(){
        double cylinderRadius = 0.15; //radius of the cylinder being searched for
        int numPointsInCircles = 36*9;
        int numberOfPointsInLine = static_cast<int>(scanRange_/0.005); //point on line every 0.025m
        double linePointSearchRadius = 0.008;
        int cylinderAcceptMatchesTolerance = 60; //the number of points found around a line point to be considered a cylinder
        std::vector<geometry_msgs::msg::Point> cylinderPos;

        // std::cout << "Analysing: " << segments_.size() << " segments" << std::endl;
        
        for(size_t i = 0; i < segments_.size(); i++){
            std::vector<geometry_msgs::msg::Point> lineGenPoints = generatePointsAlongLine(XposGT_, YposGT_, yawofSegment(segments_.at(i)), numberOfPointsInLine);
            //for each segment
            std::vector<geometry_msgs::msg::Point> circleGenPoints;
            for(size_t j = 0; j < segments_.at(i).size(); j++){
                //for each point in a segment
                //generate points in a circle
                std::vector<geometry_msgs::msg::Point> tempCirclePoints = generateCirclePoints(segments_.at(i).at(j), cylinderRadius, numPointsInCircles);
                //append them to circleGenPoints for the segment
                circleGenPoints.insert(circleGenPoints.end(), tempCirclePoints.begin(), tempCirclePoints.end());
            }

            //find the point which has the greatest number of nearby points nearby
            int maxMatch = 0;
            geometry_msgs::msg::Point maxMatchPoint;
            for(size_t k = 0; k < lineGenPoints.size(); k++){
                int match = numberOfNearbyPoints(lineGenPoints.at(k), circleGenPoints, linePointSearchRadius);
                if(match > maxMatch){
                    maxMatch = match;
                    maxMatchPoint = lineGenPoints.at(k);
                }
            }

            // std::cout << "Max match in segment: " << maxMatch << std::endl;

             //cylinder found clause
            if(maxMatch > cylinderAcceptMatchesTolerance){
                if(!excludeSegment(segments_.at(i))){
                    cylinderPos.push_back(maxMatchPoint);
                }
            }
        }
        
        //manage case of no cylinders found and cylinder found
        if(!cylinderFound_){
            printPoints(cylinderPos);
            if(cylinderPos.size() == 0){
                std::cout << "No Cylinders Found" << std::endl;
            }else{
                std::cout << "Located Cylinder" << std::endl;
                cylinderFound_ = true;
                printPoints(cylinderPos);
                cylinderX_ = cylinderPos.at(0).x;
                cylinderY_ = cylinderPos.at(0).y;
                generateImage();
            }
        }
    }

    /// @brief generates a cv::mat image with the cylinder overlayed at the correct position over the generated map of the environment
    void generateImage(){
        //pull in map and YAML
        cv::Mat mapImg = cv::imread("/home/charles/roboStu1_ros2_ws/src/sprint3/map/my_map.pgm", cv::IMREAD_GRAYSCALE);
        YAML::Node mapImgYAML = YAML::LoadFile("/home/charles/roboStu1_ros2_ws/src/sprint3/map/my_map.yaml");
        
        cv::resize(mapImg, mapImg, cv::Size(), 4, 4);

        auto mapOrigin = mapImgYAML["origin"].as<std::vector<double>>();

        std::cout << "mapOriginX: " << mapOrigin.at(0) << ", mapOriginY: " << mapOrigin.at(1) << std::endl;

        double deltaXpos = cylinderX_ - mapOrigin.at(0);
        double deltaYpos = cylinderY_ - mapOrigin.at(1); 

        std::cout << "deltaXpos: " << deltaXpos << ", deltaYpos: " << deltaYpos << std::endl;

        double mPerPixel = 0.0125; //meter per pixel
        int deltaPixelsX = deltaXpos/mPerPixel;
        int deltaPixelsY = -deltaYpos/mPerPixel;

        std::cout << "deltaPixelsX: " << deltaPixelsX << ", deltaPixelsY: " << deltaPixelsY << std::endl;

        int xPixel = 0 + deltaPixelsX;
        int yPixel = mapImg.rows + deltaPixelsY;

        //define cicle
        cv::Point center(xPixel, yPixel); // Example center point
        int radius = 12; //should be 12 for 4x size
        int thickness = -1;          // Thickness of the circle outline (-1 for filled circle)
        cv::Scalar color(0);      // White color for a grayscale image (0-255)

        cv::circle(mapImg, center, radius, color, thickness);
        cv::imshow("Map with Cylinder", mapImg);
        cv::waitKey(0); // Wait for a key press before closing
    }


private:

    /// @brief laserscan subscription variable 
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserSub_;

    /// @brief odometry subscription variable 
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;

    /// @brief ground truth x position of robot
    double XposGT_;

    /// @brief ground truth y position of robot
    double YposGT_;

    /// @brief ground truth yaw of robot
    double yawGT_;

    /// @brief laser scanner max range
    double scanRange_;

    /// @brief cylinder global x position
    double cylinderX_;

    /// @brief cylinder global y position
    double cylinderY_;

    /// @brief ture for if a cylinder has already been found
    bool cylinderFound_;

    /// @brief grouping of points in to clusters of close points for processing
    std::vector<std::vector<geometry_msgs::msg::Point>> segments_;

};
 
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<objectExclude>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}