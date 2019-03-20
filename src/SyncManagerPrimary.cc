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

#include "ignition/common/Profiler.hh"
#include "ignition/gazebo/EntityComponentManager.hh"

#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Performer.hh"
#include "ignition/gazebo/components/PerformerAffinity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Static.hh"

#include "SyncManagerPrimary.hh"
#include "SimulationRunner.hh"

#include "network/NetworkManagerPrimary.hh"
#include "network/components/PerformerActive.hh"

#include "msgs/performer_affinity.pb.h"

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
SyncManagerPrimary::SyncManagerPrimary(SimulationRunner *_runner)
  : SyncManager(_runner)
{
  if (!this->runner->networkMgr)
  {
    ignerr << "Cannot start distributed simulation. " <<
      "Server was given --distributed flag, " <<
      "but invalid configuration detected." << std::endl;
    return;
  }

  this->posePub = this->node.Advertise<ignition::msgs::Pose_V>("pose_update");
  this->node.Subscribe("pose_update", &SyncManagerPrimary::OnPose, this);
}

/////////////////////////////////////////////////
void SyncManagerPrimary::DistributePerformers()
{
  auto mgr = dynamic_cast<NetworkManagerPrimary *>(
      this->runner->networkMgr.get());

  auto &secondaries = mgr->Secondaries();
  auto secondaryIt = secondaries.begin();
  auto &ecm = this->runner->entityCompMgr;

  msgs::PerformerAffinities msg;

  this->runner->entityCompMgr.Each<components::Performer,
                                   components::ParentEntity>(
    [&](const Entity &_entity,
        const components::Performer *,
        const components::ParentEntity *_parent) -> bool
    {
      auto pid = _parent->Data();
      auto parentName =
        this->runner->entityCompMgr.Component<components::Name>(pid);

      auto affinityMsg = msg.add_affinity();
      affinityMsg->mutable_entity()->set_name(parentName->Data());
      affinityMsg->mutable_entity()->set_id(_entity);
      affinityMsg->set_secondary_prefix(secondaryIt->second->prefix);

      auto isStatic = ecm.Component<components::Static>(pid);
      *isStatic = components::Static(false);

      auto isActive = ecm.Component<components::PerformerActive>(_entity);
      *isActive = components::PerformerActive(true);

      this->runner->entityCompMgr.CreateComponent(_entity,
          components::PerformerAffinity(secondaryIt->second->prefix));

      secondaryIt++;
      if (secondaryIt == secondaries.end())
      {
        secondaryIt = secondaries.begin();
      }

      return true;
    });

  for (auto &secondary : secondaries)
  {
    bool result;
    msgs::PerformerAffinities resp;

    std::string topic {secondary.second->prefix + "/affinity"};
    unsigned int timeout = 5000;

    bool executed = this->node.Request(topic, msg, timeout, resp, result);
    if (executed)
    {
      if (!result)
      {
        ignwarn << "Failed to set performer affinities for " <<
          secondary.second->prefix << " (service call failed)" <<
          std::endl;
      }
    }
    else
    {
      ignwarn << "Failed to set performer affinities for " <<
        secondary.second->prefix << " (service call timed out)" <<
        std::endl;
    }
  }
}

/////////////////////////////////////////////////
void SyncManagerPrimary::OnPose(const ignition::msgs::Pose_V & _msg)
{
  std::lock_guard<std::mutex> lock(this->poseMutex);
  this->poseMsgs.push_back(_msg);
}

/////////////////////////////////////////////////
bool SyncManagerPrimary::Sync()
{
  IGN_PROFILE("SyncManagerPrimary::Sync");

  // TODO(mjcarroll) this is where more advanced serialization/sync will go.
  auto &ecm = this->runner->entityCompMgr;

  std::lock_guard<std::mutex> lock(this->poseMutex);
  for (const auto &msg : this->poseMsgs)
  {
    for (int ii = 0; ii < msg.pose_size(); ++ii)
    {
      const auto &poseMsg = msg.pose(ii);
      auto pid = ecm.Component<components::ParentEntity>(poseMsg.id());
      auto pose = ecm.Component<components::Pose>(pid->Data());
      auto newPose = ignition::msgs::Convert(poseMsg);
      *pose = components::Pose(newPose);
    }
  }
  this->poseMsgs.clear();

  return true;
}
