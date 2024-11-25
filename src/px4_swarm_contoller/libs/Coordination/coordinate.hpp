#pragma once

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

namespace coord {

/**
 * @brief Coordination class for multi-drone operations
 * 
 */
class Coordination {

    private:
    /**
     * @brief Poses of all drones in the map
     * 
     */
    std::vector<Eigen::Matrix4f> drone_poses;

    /**
     * @brief Control commands for drone motion
     * 
     */
    std::vector<Eigen::Vector3f> motion_commands;

    public:
        /**
         * @brief Construct a new Coordination object
         * 
         */
        Coordination();

        /**
         * @brief Destroy the Coordination object
         * 
         */
        ~Coordination();

        /**
         * @brief Assign a pose on the map to a drone
         * 
         * @param drone_id 
         * @param pose 
         */
        void assign_pose(int drone_id, const Eigen::Matrix4f& pose);

        /**
         * @brief Generate motion commands for a drone to avoid overlap
         * 
         * @param drone_id 
         * @return Eigen::Vector3f 
         */
        Eigen::Vector3f generate_motion_command(int drone_id);

        /**
         * @brief Update the position of a drone
         * 
         * @param drone_id 
         * @param new_pose 
         */
        void update_pose(int drone_id, const Eigen::Matrix4f& new_pose);

        /**
         * @brief Ensure area coverage without overlap
         * 
         */
        void ensure_coverage();

        /**
         * @brief Log coordination details
         * 
         */
        void log_details() const;
};

}
