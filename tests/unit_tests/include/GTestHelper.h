/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#include <dsd_ros_messages/msg/mission_profile_stamped.hpp>

#include <ostream>

namespace dsd_ros_messages
{
namespace msg
{

inline std::ostream& operator<<(std::ostream& os, const MissionProfile& mission_profile)
{
    return os << to_yaml(mission_profile);
}

inline std::ostream& operator<<(std::ostream& os, const MissionProfileStamped& mission_profile_stamped)
{
    return os << to_yaml(mission_profile_stamped);
}

} // namespace msg
} // namespace dsd_ros_messages