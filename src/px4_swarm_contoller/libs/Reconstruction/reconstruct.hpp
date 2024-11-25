#pragma once

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <open3d/Open3D.h>

namespace reconstruct {

/**
 * @brief Reconstruction class for 3D environment modeling
 * 
 */
class Reconstruction {

    private:
    /**
     * @brief Individual point clouds captured by drones
     * 
     */
    std::vector<open3d::geometry::PointCloud> individual_point_clouds;

    /**
     * @brief Unified point cloud of the environment
     * 
     */
    open3d::geometry::PointCloud unified_point_cloud;

    public:
        /**
         * @brief Construct a new Reconstruction object
         * 
         */
        Reconstruction();

        /**
         * @brief Destroy the Reconstruction object
         * 
         */
        ~Reconstruction();

        /**
         * @brief Add an individual point cloud to the dataset
         * 
         * @param point_cloud 
         */
        void add_point_cloud(const open3d::geometry::PointCloud& point_cloud);

        /**
         * @brief Merge individual point clouds into a unified point cloud
         * 
         * @return open3d::geometry::PointCloud 
         */
        open3d::geometry::PointCloud merge_point_clouds();

        /**
         * @brief Apply sensor fusion to correct discrepancies in datasets
         * 
         */
        void apply_sensor_fusion();

        /**
         * @brief Process the unified point cloud for 3D model generation
         * 
         * @return open3d::geometry::PointCloud 
         */
        open3d::geometry::PointCloud process_unified_point_cloud();

        /**
         * @brief Log reconstruction details
         * 
         */
        void log_details() const;
};

}
