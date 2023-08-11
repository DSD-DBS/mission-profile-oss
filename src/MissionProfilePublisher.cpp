/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#include "dsd_mission_profile/MissionProfilePublisher.h"

#include <dsd_mission_profile/ROSMessages.h>

MissionProfilePublisher::MissionProfilePublisher(const rclcpp::NodeOptions& options)
: Node("mission_profile_publisher", options)
{
    declare_config_parameters();
    init_config_parameters();

    mission_profile_publisher_ = std::make_shared<PublisherWrapper<dsd_ros_messages::msg::MissionProfileStamped>>(
        this->create_publisher<dsd_ros_messages::msg::MissionProfileStamped>(mission_profile_topic_, dds_qos_));

    timer_missionprofile_callback_ = this->create_wall_timer(
        1s, std::bind(&MissionProfilePublisher::mission_profile_periodic_broadcast_callback, this));
}

void MissionProfilePublisher::declare_config_parameters()
{
    this->declare_parameter("mission_profile_id", 1);
    this->declare_parameter("mission_profile_topic", "/mission_profile");
    this->declare_parameter("map_provider", "DB");
    this->declare_parameter("track_uuids", std::vector<std::string>{"101"});
    this->declare_parameter("dds_qos", 3);
}

void MissionProfilePublisher::init_config_parameters()
{
    mission_profile_id_ = this->get_parameter("mission_profile_id").as_int();
    mission_profile_topic_ = this->get_parameter("mission_profile_topic").as_string();
    map_provider_ = this->get_parameter("map_provider").as_string();
    if (map_provider_ == "DB")
    {
        track_uuids_ = this->get_parameter("track_uuids").as_string_array();
        std::transform(begin(track_uuids_), end(track_uuids_), std::back_inserter(IDs_), [](const std::string& uuid) {
            return create_id_message(std::hash<std::string>{}(uuid));
        });
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid map provider!");
    }

    dds_qos_ = this->get_parameter("dds_qos").as_int();

    RCLCPP_INFO(this->get_logger(), "The topic is: %s ", mission_profile_topic_.c_str());
}

void MissionProfilePublisher::mission_profile_periodic_broadcast_callback()
{
    RCLCPP_INFO(this->get_logger(), "Publishing Mission Profile message");
    mission_profile_publisher_->publish(create_message(seq_++, now(), IDs_, mission_profile_id_));
}

rclcpp::Time MissionProfilePublisher::now()
{
    return this->get_clock()->now();
}
