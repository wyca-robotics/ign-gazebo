/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#include <ignition/msgs/vector3d.pb.h>

#include <ignition/math/Vector3.hh>
#include <ignition/plugin/RegisterMore.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/LinearVelocity.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/Model.hh"

#include "Move3d.hh"

using namespace ignition;

using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::Move3dPrivate
{
  /// \brief Callback for velocity subscription
  /// \param[in] _msg Velocity message
  public: void OnLinearVel(const ignition::msgs::Vector3d &_msg);

  /// \brief Ignition communication node.
  public: transport::Node node;

  /// \brief Current velocity command
  public: std::optional<math::Vector3d> linearVelCmd;

  /// \brief Model interface
  public: Model model{kNullEntity};
};

//////////////////////////////////////////////////
Move3d::Move3d() : dataPtr(std::make_unique<Move3dPrivate>())
{
}

//////////////////////////////////////////////////
void Move3d::Configure(
    const Entity &_entity, const std::shared_ptr<const sdf::Element> &,
    EntityComponentManager &_ecm, EventManager &)
{
  this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    ignerr << "Move3d plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  std::string topic{"/model/" + this->dataPtr->model.Name(_ecm) +
                    "/linear_vel"};
  this->dataPtr->node.Subscribe(topic, &Move3dPrivate::OnLinearVel,
                                this->dataPtr.get());

  ignmsg << "Move3D subscribing to linear velocity messages on [" << topic
         << "]" << std::endl;
}

//////////////////////////////////////////////////
void Move3d::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  if (this->dataPtr->model.Entity() == kNullEntity)
    return;

  // Nothing left to do if paused.
  if (_info.paused)
    return;

  // No new command
  if (!this->dataPtr->linearVelCmd.has_value())
    return;

  // update the next position of the model based on the commanded velocity
  auto linVelocity =
      _ecm.Component<components::LinearVelocity>(this->dataPtr->model.Entity());

  if (linVelocity != nullptr)
  {
    *linVelocity = components::LinearVelocity(*this->dataPtr->linearVelCmd);
  }
  else
  {
    _ecm.CreateComponent(
        this->dataPtr->model.Entity(),
        components::LinearVelocity(*this->dataPtr->linearVelCmd));
  }
  // clear the command so that we only update the component when there's a new
  // command.
  this->dataPtr->linearVelCmd.reset();
}

//////////////////////////////////////////////////
void Move3dPrivate::OnLinearVel(const msgs::Vector3d &_msg)
{
  igndbg << "Command: " << _msg.x() << " " << _msg.y() << " " << _msg.z()
         << "\n";
  this->linearVelCmd = msgs::Convert(_msg);
}

IGNITION_ADD_PLUGIN(ignition::gazebo::systems::Move3d,
                    ignition::gazebo::System,
                    Move3d::ISystemConfigure,
                    Move3d::ISystemPreUpdate)
