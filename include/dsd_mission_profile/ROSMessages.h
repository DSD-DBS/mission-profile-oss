/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DSD_MISSION_PROFILE_ROSMESSAGES_H
#define DSD_MISSION_PROFILE_ROSMESSAGES_H

#include "rclcpp/time.hpp"

#include <dsd_ros_messages/msg/id.hpp>
#include <dsd_ros_messages/msg/key_value_pair.hpp>
#include <dsd_ros_messages/msg/mission_profile_stamped.hpp>
/**
 * @brief Constructs message of type ID
 * @param id unique identifier
 * @return message of type ID
 */
inline dsd_ros_messages::msg::ID create_id_message(uint64_t id)
{
    dsd_ros_messages::msg::ID id_message;
    id_message.id = id;
    return id_message;
}
/**
 * @brief Constructs message of type key_value_pair
 * @param key text defining key
 * @param value value paired with a key
 * @return message of type KeyValuePair
 */
inline dsd_ros_messages::msg::KeyValuePair create_key_value_pair(const std::string& key, const std::string& value)
{
    dsd_ros_messages::msg::KeyValuePair pair{};
    pair.key = key;
    pair.value = value;
    return pair;
}
/**
 * @brief Constructs the message which is sent periodically by publisher
 * @param seq Sequential number of the message
 * @param timestamp current time of type rclcpp::Time
 * @param track_ids vector that contains messages of type ID
 * @param mission_profile_ID unique number of the mission profile
 * @return message of type MissionProfileStamped
 */
inline dsd_ros_messages::msg::MissionProfileStamped create_message(
    size_t seq, rclcpp::Time timestamp, std::vector<dsd_ros_messages::msg::ID> track_ids, int mission_profile_ID)
{
    dsd_ros_messages::msg::MissionProfileStamped message;
    message.seq = seq;
    message.stamp = timestamp;
    message.mission_profile.track_ids = track_ids;
    dsd_ros_messages::msg::KeyValueMap key_value_map{};
    key_value_map.data = {create_key_value_pair("mission_profile_ID", std::to_string(mission_profile_ID))};
    message.mission_profile.optional_attributes = key_value_map;
    return message;
}

#endif // DSD_MISSION_PROFILE_ROSMESSAGES_H
