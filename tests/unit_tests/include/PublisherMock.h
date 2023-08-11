/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DSD_RAIL_HORIZON_PUBLISHER_MOCK_H
#define DSD_RAIL_HORIZON_PUBLISHER_MOCK_H

#include <gmock/gmock.h>

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <dsd_mission_profile/ros_wrapper/PublisherWrapper.h>


template <typename MessageType>
class PublisherMock : public IPublisher<MessageType>
{
public:
    MOCK_METHOD(void, publish, ( const MessageType& ), (const, override));
    MOCK_METHOD(const char*, get_topic_name, (), (const, override));
    MOCK_METHOD(rclcpp::QoS, get_actual_qos, (), (const, override));
};

#endif // DSD_RAIL_HORIZON_PUBLISHER_MOCK_H
