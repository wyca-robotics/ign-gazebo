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
#include "ignition/gazebo/Events.hh"
#include "ignition/gazebo/EntityComponentManager.hh"

#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/PerformerAffinity.hh"
#include "ignition/gazebo/components/Static.hh"

#include "components/PerformerActive.hh"
#include "NetworkManagerSecondary.hh"
#include "SyncManagerSecondary.hh"

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
SyncManagerSecondary::SyncManagerSecondary(EntityComponentManager &_ecm,
    NetworkManager *_networkManager) : SyncManager(_ecm, _networkManager)
{
  std::string topic {_networkManager->Namespace() + "/affinity"};

  this->node.Advertise(topic, &SyncManagerSecondary::AffinityService, this);

  this->statePub = this->node.Advertise<ignition::msgs::SerializedState>(
      "state_update");

  igndbg << "Secondary [" << this->networkManager->Namespace()
         << "] waiting for affinity assignment." << std::endl;
}

/////////////////////////////////////////////////
SyncManagerSecondary::~SyncManagerSecondary() = default;

/////////////////////////////////////////////////
bool SyncManagerSecondary::AffinityService(
    const private_msgs::PerformerAffinities &_req,
    private_msgs::PerformerAffinities &)
{
  for (int i = 0; i < _req.affinity_size(); ++i)
  {
    const auto &affinityMsg = _req.affinity(i);
    const auto &entityId = affinityMsg.entity().id();

    auto pid = this->ecm->Component<components::ParentEntity>(entityId);

    this->ecm->CreateComponent(entityId,
      components::PerformerAffinity(affinityMsg.secondary_prefix()));

    // TODO(louise) Remove instead of setting static, and then
    // PerformerActive is not needed
    auto isStatic = this->ecm->Component<components::Static>(pid->Data());
    auto isActive = this->ecm->Component<components::PerformerActive>(entityId);

    if (affinityMsg.secondary_prefix() == this->networkManager->Namespace())
    {
      this->performers.insert(entityId);
      *isStatic = components::Static(false);
      *isActive = components::PerformerActive(true);
      igndbg << "Secondary [" << this->networkManager->Namespace()
             << "] assigned affinity to performer [" << entityId << "]."
             << std::endl;
    }
    else
    {
      *isStatic = components::Static(true);
      *isActive = components::PerformerActive(false);
    }
  }

  this->initialized = true;
  return true;
}

/////////////////////////////////////////////////
bool SyncManagerSecondary::Sync()
{
  IGN_PROFILE("SyncManagerSecondary::Sync");

  // Get all the performer's models
  std::unordered_set<Entity> models;
  for (const auto &perf : this->performers)
  {
    models.insert(this->ecm->Component<components::ParentEntity>(perf)->Data());
  }

  auto msg = this->ecm->State(models);
  this->statePub.Publish(msg);

  return true;
}
