/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DSD_MISSIONPROFILEPUBLISHER_H
#define DSD_MISSIONPROFILEPUBLISHER_H

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <dsd_ros_messages/msg/mission_profile.hpp>
#include <dsd_ros_messages/msg/mission_profile_stamped.hpp>

#include <chrono>
#include <dsd_mission_profile/ros_wrapper/PublisherWrapper.h>
#include <vector>


using namespace std::chrono_literals;
/**
 * @brief Mission Profile publisher for publishing mission profile messages
 */
class MissionProfilePublisher : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Mission profile node
     * @param options Optionally pass ROS2 options such as parameters
     */
    MissionProfilePublisher(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

protected:
    /**
     * @brief Declares all node config parameters
     *
     */
    void declare_config_parameters();

    /**
     * @brief Reads all config parameters and converts them in the appropriate data type
     *
     */
    void init_config_parameters();

    /**
     * @brief Periodically send Mission Profile Message
     */
    void mission_profile_periodic_broadcast_callback();

    /**
     * @brief Returns the current time using the rclpp provider by default, can be overriden in a subclass for
     * testability
     */
    virtual rclcpp::Time now();

    /**
     * @brief ROS publisher for publishing the MissionProfileStamped message
     */
    std::shared_ptr<IPublisher<dsd_ros_messages::msg::MissionProfileStamped>> mission_profile_publisher_;
    /**
     * @brief Timer periodically calls mission_profile_periodic_broadcast_callback
     *
     */
    rclcpp::TimerBase::SharedPtr timer_missionprofile_callback_;

    /**
     * @brief ROS config parameter: mission_profile_id to uniquely identify the mission profile
     */
    int mission_profile_id_{};
    /**
     * @brief ROS config parameter: mission_profile_topic on which messages are published
     */
    std::string mission_profile_topic_;
    /**
     * @brief ROS config parameter: map_provider to define the provider of the map data
     */
    std::string map_provider_;
    /**
     * @brief ROS config parameter: track_uuids_ to determine the driving path of the train
     */
    std::vector<std::string> track_uuids_;
    /**
     * @brief Quality of service setting for the publisher
     */
    int dds_qos_{};

    /**
     * @brief Vector of track IDs packed into a ROS message
     */
    std::vector<dsd_ros_messages::msg::ID> IDs_;

    /**
     * @brief Sequence number of the message
     */
    size_t seq_{};
};
#endif // DSD_MISSIONPROFILEPUBLISHER_H