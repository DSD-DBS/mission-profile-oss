/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#include "dsd_mission_profile/MissionProfilePublisher.h"
#include "dsd_mission_profile/ROSMessages.h"
#include "GTestHelper.h"
#include "PublisherMock.h"
#include "rclcpp/parameter.hpp"
#include "rclcpp/rclcpp.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using ::testing::Eq;

// Expose protected members for testing purposes
class MissionProfilePublisherExposed : public MissionProfilePublisher
{
public:
    using MissionProfilePublisher::mission_profile_periodic_broadcast_callback;
    using MissionProfilePublisher::MissionProfilePublisher;

    using MissionProfilePublisher::mission_profile_publisher_;

    /** Replace now() implementation for testability **/
    rclcpp::Time now() override
    {
        return rclcpp::Time{};
    }
};

class MissionProfilePublisherTest : public ::testing::Test
{
protected:
    MissionProfilePublisherTest()
    {
        rclcpp::init(0, nullptr);
    }

    ~MissionProfilePublisherTest() override
    {
        rclcpp::shutdown();
    }

    std::shared_ptr<MissionProfilePublisherExposed> getDefaultMissionProfilePublisher()
    {
        return std::make_shared<MissionProfilePublisherExposed>();
    }

    std::shared_ptr<MissionProfilePublisherExposed> getOverridenMissionProfilePublisher()
    {
        rclcpp::NodeOptions nodeOptions{};
        nodeOptions.append_parameter_override("mission_profile_topic", overridenTopic);
        nodeOptions.append_parameter_override("mission_profile_ID", static_cast<int>(overridenMissionProfileID));
        nodeOptions.append_parameter_override("map_provider", overridenMapProvider);
        nodeOptions.append_parameter_override("track_uuids", overridenTrackUUIds);
        nodeOptions.append_parameter_override("dds_qos", static_cast<int>(overridenDdsQos));

        return std::make_shared<MissionProfilePublisherExposed>(nodeOptions);
    }

    std::string overridenTopic = "/test_topic";
    uint64_t overridenMissionProfileID = 1;
    std::string overridenMapProvider = "DB";
    std::vector<std::string> overridenTrackUUIds = std::vector<std::string>{"1", "10", "42", "103", "1372"};
    std::size_t overridenDdsQos = 2;
};

TEST_F(MissionProfilePublisherTest, defaultTopicIsMissionProfile)
{
    // Arrange
    auto missionProfilePublisher = getDefaultMissionProfilePublisher();

    // Act
    std::string actualTopicName = missionProfilePublisher->mission_profile_publisher_->get_topic_name();

    // Assert
    EXPECT_THAT("/mission_profile", Eq(actualTopicName));
}

TEST_F(MissionProfilePublisherTest, topicIsOverriden)
{
    // Arrange
    auto missionProfilePublisher = getOverridenMissionProfilePublisher();

    // Act
    std::string actualTopicName = missionProfilePublisher->mission_profile_publisher_->get_topic_name();

    // Assert
    EXPECT_THAT(overridenTopic, Eq(actualTopicName));
}

TEST_F(MissionProfilePublisherTest, defaultQualityOfService)
{
    // Arrange
    auto missionProfilePublisher = getDefaultMissionProfilePublisher();

    // Act
    auto actualQos = missionProfilePublisher->mission_profile_publisher_->get_actual_qos();

    // Assert
    auto depth = rclcpp::QoS{3}.get_rmw_qos_profile().depth;
    auto actualDepth = actualQos.get_rmw_qos_profile().depth;
    EXPECT_THAT(depth, Eq(actualDepth));
}

TEST_F(MissionProfilePublisherTest, qualityOfServiceIsOverriden)
{
    // Arrange
    auto missionProfilePublisher = getOverridenMissionProfilePublisher();

    // Act
    auto actualQos = missionProfilePublisher->mission_profile_publisher_->get_actual_qos();

    // Assert
    auto depth = rclcpp::QoS{overridenDdsQos}.get_rmw_qos_profile().depth;
    auto actualDepth = actualQos.get_rmw_qos_profile().depth;
    EXPECT_THAT(depth, Eq(actualDepth));
}

TEST_F(MissionProfilePublisherTest, messageIsPublishedOnCallback)
{
    // Arrange
    auto missionProfilePublisher = getOverridenMissionProfilePublisher();
    auto publisherMock = std::make_shared<PublisherMock<dsd_ros_messages::msg::MissionProfileStamped>>();
    missionProfilePublisher->mission_profile_publisher_ = publisherMock;

    // Expect
    std::vector<dsd_ros_messages::msg::ID> IDs{};
    std::transform(begin(overridenTrackUUIds), end(overridenTrackUUIds), std::back_inserter(IDs),
        [](const std::string& uuid) {
        return create_id_message(std::hash<std::string>{}(uuid));
    });
    auto message = create_message(0, missionProfilePublisher->now(), IDs, overridenMissionProfileID);
    EXPECT_CALL(*publisherMock, publish(message));

    // Act
    missionProfilePublisher->mission_profile_periodic_broadcast_callback();
}

TEST_F(MissionProfilePublisherTest, messageWithoutTrackIDsIsPublishedOnCallback)
{
    // Arrange
    overridenMapProvider = "INVALID_PROVIDER";
    auto missionProfilePublisher = getOverridenMissionProfilePublisher();
    auto publisherMock = std::make_shared<PublisherMock<dsd_ros_messages::msg::MissionProfileStamped>>();
    missionProfilePublisher->mission_profile_publisher_ = publisherMock;

    // Expect
    std::vector<dsd_ros_messages::msg::ID> IDs{};
    auto message = create_message(0, missionProfilePublisher->now(), IDs, overridenMissionProfileID);
    EXPECT_CALL(*publisherMock, publish(message));

    // Act
    missionProfilePublisher->mission_profile_periodic_broadcast_callback();
}