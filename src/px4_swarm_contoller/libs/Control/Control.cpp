/**
 * @file Control.cpp
 * @author Apoorv Thapliyal
 * @brief C++ Source file for the Control class
 * @version 0.1
 * @date 2024-12-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "Control.hpp"

ctrl::Control::Control() : Node("Control"){

    // Initialize the setpoint counter
    offboard_setpoint_counter_ = 0;

    // Initialize the number of drones in the swarm
    num_drones_ = 5;

    std::cout << "Control node initialized" << std::endl;

    // Initalize the publishers for every drone in the swarm
    for(int i=1; i<=num_drones_; i++){

        // String of the offboard control mode topic
        std::string offboard_control_mode_topic = "/px4_" + std::to_string(i) + "/fmu/in/offboard_control_mode";
        
        // String of the trajectory setpoint topic
        std::string trajectory_setpoint_topic = "/px4_" + std::to_string(i) + "/fmu/in/trajectory_setpoint";
        
        // String of the vehicle command topic
        std::string vehicle_command_topic = "/px4_" + std::to_string(i) + "/fmu/in/vehicle_command";

        // Publisher for the offboard control mode
        offboard_control_mode_publishers_.push_back(this->create_publisher<OffboardControlMode>(offboard_control_mode_topic, 10));
        
        // Publisher for the trajectory setpoint
        trajectory_setpoint_publishers_.push_back(this->create_publisher<TrajectorySetpoint>(trajectory_setpoint_topic, 10));
        
        // Publisher for the vehicle command
        vehicle_command_publishers_.push_back(this->create_publisher<VehicleCommand>(vehicle_command_topic, 10));
    }


    // Define a timer callback
    auto timer_callback = [this]() -> void{

        if(offboard_setpoint_counter_ == 10) {
            // Switch to offboard mode and arm the drones
            for(int i=0; i<num_drones_; i++){
                arm(vehicle_command_publishers_[i]);
                offboard_mode(vehicle_command_publishers_[i]);
            }
        }

        // The offboard control mode needs to be paired with a trajectory setpoint
        // Publish setpoint as a 5m altitude hold 
        for(int i=0; i<num_drones_; i++){
            publish_offboard_control_mode(offboard_control_mode_publishers_[i]);
            publish_trajectory_setpoint(trajectory_setpoint_publishers_[i], 0.0, 0.0, -5.0, 0.0);
        }

        // Stop the counter after reaching 11
        if(offboard_setpoint_counter_ < 11) {
            offboard_setpoint_counter_++;
        }
    };

    // Create a timer with a 100ms period
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), timer_callback);
}

void ctrl::Control::publish_offboard_control_mode(rclcpp::Publisher<OffboardControlMode>::SharedPtr pub_){
    OffboardControlMode msg;
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    pub_->publish(msg);
}

void ctrl::Control::publish_trajectory_setpoint(rclcpp::Publisher<TrajectorySetpoint>::SharedPtr pub_, float x, float y, float z, float yaw){
    TrajectorySetpoint msg;
    msg.position = {x, y, z};
    msg.yaw = yaw;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    pub_->publish(msg);
}

void ctrl::Control::arm(rclcpp::Publisher<VehicleCommand>::SharedPtr pub_){
    publish_vehicle_command(pub_, VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

    RCLCPP_INFO(this->get_logger(), "Arm command sent");
}

void ctrl::Control::offboard_mode(rclcpp::Publisher<VehicleCommand>::SharedPtr pub_){
    publish_vehicle_command(pub_, VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);

    RCLCPP_INFO(this->get_logger(), "Offboard mode command sent");
}

void ctrl::Control::publish_vehicle_command(rclcpp::Publisher<VehicleCommand>::SharedPtr pub_, uint16_t command, float param1, float param2){
    VehicleCommand msg;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 0;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    pub_->publish(msg);
}
