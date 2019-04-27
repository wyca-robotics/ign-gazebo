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
#include <ignition/plugin/Register.hh>

#include "ignition/gazebo/components/AngularVelocityCmd.hh"
#include "ignition/gazebo/components/LinearVelocityCmd.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/PoseCmd.hh"
#include "ignition/gazebo/components/Name.hh"

#include "MoveEntity.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

//////////////////////////////////////////////////
void MoveEntity::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &,
    EntityComponentManager &_ecm,
    EventManager &)
{
  this->model = Model(_entity);
  this->entities.push_back(_entity);

  if (!this->model.Valid(_ecm))
  {
    ignerr << "MoveEntity plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  auto onCmdPose = [this](Entity _e)
  {
    auto cb = [this, _e](const msgs::Pose &_msg)
    {
      std::lock_guard<std::mutex> lock(this->cmdMutex);
      this->poseCmds[_e] = _msg;
    };
    return std::function<void(const msgs::Pose &_msg)>(cb);
  };

  auto onCmdTwist = [this](Entity _e)
  {
    auto cb = [this, _e](const msgs::Twist &_msg)
    {
      std::lock_guard<std::mutex> lock(this->cmdMutex);
      this->twistCmds[_e] = _msg;
    };
    return std::function<void(const msgs::Twist &_msg)>(cb);
  };

  // Subscribe to model commands
  // TODO(addisu) Add '/world'
  std::string twistTopic{"/model/" + this->model.Name(_ecm) + "/cmd_vel"};
  auto modelTwistCb = onCmdTwist(_entity);
  this->node.Subscribe(twistTopic, modelTwistCb);

  ignmsg << "MoveEntity subscribing to Twist messages on [" << twistTopic << "]"
         << std::endl;

  std::string poseTopic{"/model/" + this->model.Name(_ecm) + "/cmd_pose"};
  auto modelPoseCb = onCmdPose(_entity);
  this->node.Subscribe(poseTopic, modelPoseCb);

  ignmsg << "MoveEntity subscribing to Twist messages on [" << poseTopic << "]"
         << std::endl;

  // Subscribe to link commands
  for (Entity linkEntity :
       _ecm.ChildrenByComponents(_entity, components::Link()))
  {
    this->entities.push_back(linkEntity);
    auto name = _ecm.Component<components::Name>(linkEntity)->Data();

    std::string linkTwistTopic{"/model/" + this->model.Name(_ecm) + "/link/" +
                               name + "/cmd_vel"};
    auto linkTwistCb = onCmdTwist(linkEntity);
    this->node.Subscribe(linkTwistTopic, linkTwistCb);
    ignmsg << "MoveEntity subscribing to Twist messages on [" << linkTwistTopic
           << "]" << std::endl;

    std::string linkPoseTopic{"/model/" + this->model.Name(_ecm) + "/link/" +
                              name + "/cmd_pose"};
    auto linkPoseCb = onCmdPose(linkEntity);
    this->node.Subscribe(linkPoseTopic, linkPoseCb);
    ignmsg << "MoveEntity subscribing to Twist messages on [" << linkPoseTopic
           << "]" << std::endl;
  }
}

//////////////////////////////////////////////////
void MoveEntity::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  // Nothing left to do if paused.
  if (_info.paused)
    return;

  std::lock_guard<std::mutex> lock(this->cmdMutex);

  for (const auto &entity : this->entities)
  {
    auto &poseCmd = this->poseCmds[entity];
    auto &twistCmd = this->twistCmds[entity];

    auto linVelCmdComp =
        _ecm.Component<components::WorldLinearVelocityCmd>(entity);
    auto angVelCmdComp =
        _ecm.Component<components::WorldAngularVelocityCmd>(entity);
    auto poseCmdComp =
        _ecm.Component<components::WorldPoseCmd>(entity);

    if (poseCmd)
    {
      // Pose
      if (poseCmdComp == nullptr)
      {
        _ecm.CreateComponent(entity,
                             components::WorldPoseCmd(msgs::Convert(*poseCmd)));
      }
      else
      {
        poseCmdComp->Data() = msgs::Convert(*poseCmd);
      }
      poseCmd.reset();
    }
    else if (twistCmd)
    {
      // Twist
      if (linVelCmdComp == nullptr)
      {
        _ecm.CreateComponent(entity, components::WorldLinearVelocityCmd(
                                    msgs::Convert(twistCmd->linear())));
      }
      else
      {
        linVelCmdComp->Data() = msgs::Convert(twistCmd->linear());
      }

      if (angVelCmdComp == nullptr)
      {
        _ecm.CreateComponent(entity, components::WorldAngularVelocityCmd(
                                    msgs::Convert(twistCmd->angular())));
      }
      else
      {
        angVelCmdComp->Data() = msgs::Convert(twistCmd->angular());
      }

      twistCmd.reset();
    }
  }
}

IGNITION_ADD_PLUGIN(MoveEntity,
                    ignition::gazebo::System,
                    MoveEntity::ISystemConfigure,
                    MoveEntity::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(MoveEntity, "ignition::gazebo::systems::MoveEntity")
