/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#ifndef IGNITION_GAZEBO_SYSTEMS_MOVEENTITY_HH_
#define IGNITION_GAZEBO_SYSTEMS_MOVEENTITY_HH_

#include <memory>
#include <optional>

#include <ignition/msgs/twist.pb.h>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/System.hh"

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration
  class MoveEntityPrivate;

  /// \brief This system applies a force to the first axis of a specified joint.
  class IGNITION_GAZEBO_VISIBLE MoveEntity
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate
  {
    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(
                const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;

    /// \brief Ignition communication node.
    public: transport::Node node;

    /// \brief Entity
    public: std::vector<Entity> entities;

    /// \brief Commanded twist
    public: std::map<Entity, std::optional<ignition::msgs::Twist>> twistCmds;

    /// \brief Commanded Pose
    public: std::map<Entity, std::optional<ignition::msgs::Pose>> poseCmds;

    /// \brief mutex to protect twistCmd and poseCmd
    public: std::mutex cmdMutex;

    /// \brief Model interface
    public: Model model{kNullEntity};
  };
  }
}
}
}

#endif
