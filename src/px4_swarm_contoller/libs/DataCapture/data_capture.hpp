#pragma once

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>

namespace datacap {

/**
 * @brief Data Capture class for autonomous systems
 * 
 */
class DataCapture {

    private:
    /**
     * @brief Assigned waypoint or region for a drone
     * 
     */
    Eigen::Vector3f assigned_region;

    /**
     * @brief Navigation points for autonomous movement
     * 
     */
    std::vector<Eigen::Vector3f> waypoints;

    /**
     * @brief Depth data captured by LiDAR
     * 
     */
    std::vector<float> depth_data;

    public:
        /**
         * @brief Construct a new Data Capture object
         * 
         */
        DataCapture();

        /**
         * @brief Destroy the Data Capture object
         * 
         */
        ~DataCapture();

        /**
         * @brief Assign a specific region or waypoint to a drone
         * 
         * @param region 
         */
        void assign_region(const Eigen::Vector3f& region);

        /**
         * @brief Plan autonomous navigation through waypoints
         * 
         * @param new_waypoints 
         */
        void plan_navigation(const std::vector<Eigen::Vector3f>& new_waypoints);

        /**
         * @brief Capture depth data using LiDAR sensors
         * 
         * @return std::vector<float> 
         */
        std::vector<float> capture_depth_data();

        /**
         * @brief Process captured depth data
         * 
         * @param depth_data 
         */
        void process_depth_data(const std::vector<float>& depth_data);

        /**
         * @brief Log data capture and navigation results
         * 
         */
        void log_results() const;
};

}