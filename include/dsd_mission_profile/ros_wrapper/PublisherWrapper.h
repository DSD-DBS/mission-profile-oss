/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DSD_MISSION_PROFILE_WRAPPER_H
#define DSD_MISSION_PROFILE_WRAPPER_H

#include <rclcpp/rclcpp.hpp>

/**
 * @brief Interface of publisher
 */
template <typename MessageType>
class IPublisher
{
public:
    /**
     * @brief Sends a message to the topic for the publisher
     * @param message ROS message of any type
     */
    virtual void publish(const MessageType& message) const = 0;
    /**
     * @brief Get topic to which the publisher sends messages
     * @return Topic name
     */
    [[nodiscard]] virtual const char* get_topic_name() const = 0;
    /**
     * @brief Get the actual QoS settings
     * @return Actual QoS Settings
     */
    [[nodiscard]] virtual rclcpp::QoS get_actual_qos() const = 0;
};

/**
 * @brief Publisher wrapper that implements the methods from IPublisher
 */
template <typename MessageType>
class PublisherWrapper : public IPublisher<MessageType>
{
public:
    /**
     * @brief Constructor for publisher
     * @param publisher Mutable pointer to a ros publisher for publishing certain message type
     */
    PublisherWrapper(typename rclcpp::Publisher<MessageType>::SharedPtr publisher) : publisher_{publisher} {}

    /**
     * @brief Sends a message to the topic for the publisher
     * @param message ROS message of any type
     */
    void publish(const MessageType& message) const override
    {
        publisher_->publish(message);
    }

    /**
     * @brief Get topic to which the publisher sends messages
     * @return Topic name
     */
    [[nodiscard]] const char* get_topic_name() const override
    {
        return publisher_->get_topic_name();
    }

    /**
     * @brief Get the actual QoS settings
     * @return Actual QoS Settings
     */
    [[nodiscard]] rclcpp::QoS get_actual_qos() const override
    {
        return publisher_->get_actual_qos();
    }

private:
    /**
     * @brief The ros publisher for publishing any message type to a given topic
     */
    typename rclcpp::Publisher<MessageType>::SharedPtr publisher_;
};

#endif // DSD_MISSION_PROFILE_WRAPPER_H
