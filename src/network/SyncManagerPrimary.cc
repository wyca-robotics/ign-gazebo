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
#include "ignition/gazebo/components/Static.hh"

#include "components/PerformerActive.hh"
#include "NetworkManagerPrimary.hh"
#include "SyncManagerPrimary.hh"

#include "msgs/performer_affinity.pb.h"

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
SyncManagerPrimary::SyncManagerPrimary(EntityComponentManager &_ecm,
    NetworkManager *_networkManager)
  : SyncManager(_ecm, _networkManager)
{
  this->node.Subscribe("state_update", &SyncManagerPrimary::OnState, this);
}

/////////////////////////////////////////////////
void SyncManagerPrimary::Initialize()
{
  auto mgrPrimary = dynamic_cast<NetworkManagerPrimary *>(
      this->networkManager);
  auto secondaryIt = mgrPrimary->Secondaries().begin();

  private_msgs::PerformerAffinities msg;

  this->ecm->Each<components::Performer, components::ParentEntity>(
    [&](const Entity &_entity,
        const components::Performer *,
        const components::ParentEntity *_parent) -> bool
    {
      auto pid = _parent->Data();
      auto parentName =
        this->ecm->Component<components::Name>(pid);

      auto affinityMsg = msg.add_affinity();
      affinityMsg->mutable_entity()->set_name(parentName->Data());
      affinityMsg->mutable_entity()->set_id(_entity);
      affinityMsg->set_secondary_prefix(secondaryIt->second->prefix);

      // TODO(louise) No need to set to static on primary because it doesn't
      // have physics anyway
      auto isStatic = this->ecm->Component<components::Static>(pid);
      *isStatic = components::Static(false);

      auto isActive = this->ecm->Component<components::PerformerActive>(_entity);
      *isActive = components::PerformerActive(true);

      this->ecm->CreateComponent(_entity,
          components::PerformerAffinity(secondaryIt->second->prefix));

      secondaryIt++;
      if (secondaryIt == mgrPrimary->Secondaries().end())
      {
        secondaryIt = mgrPrimary->Secondaries().begin();
      }

      return true;
    });

  for (auto &secondary : mgrPrimary->Secondaries())
  {
    bool result;
    private_msgs::PerformerAffinities resp;

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

  this->initialized = true;
}

/////////////////////////////////////////////////
void SyncManagerPrimary::OnState(const ignition::msgs::SerializedState & _msg)
{
  std::lock_guard<std::mutex> lock(this->msgMutex);
  this->stateMsgs.push_back(_msg);
}

/////////////////////////////////////////////////
bool SyncManagerPrimary::Sync()
{
  IGN_PROFILE("SyncManagerPrimary::Sync");

  std::lock_guard<std::mutex> lock(this->msgMutex);
  for (const auto &msg : this->stateMsgs)
  {
    this->ecm->SetState(msg);
  }
  this->stateMsgs.clear();

  return true;
}
