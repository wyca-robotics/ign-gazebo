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

#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Performer.hh"
#include "ignition/gazebo/components/PerformerAffinity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Static.hh"
#include "ignition/gazebo/components/World.hh"

#include "SyncManagerSecondary.hh"
#include "SimulationRunner.hh"

#include "network/NetworkManagerPrimary.hh"
#include "network/NetworkManagerSecondary.hh"
#include "network/components/PerformerActive.hh"

#include "msgs/performer_affinity.pb.h"

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
SyncManagerSecondary::SyncManagerSecondary(SimulationRunner *_runner)
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
}

/////////////////////////////////////////////////
void SyncManagerSecondary::DistributePerformers()
{
  auto mgr = dynamic_cast<NetworkManagerSecondary *>(
      this->runner->networkMgr.get());
  auto &ecm = this->runner->entityCompMgr;
  std::string topic {mgr->Namespace() + "/affinity"};
  bool received = false;

  std::function<bool(const msgs::PerformerAffinities &,
                     msgs::PerformerAffinities &)> fcn =
    [&received, &mgr, this, &ecm](const msgs::PerformerAffinities &_req,
      msgs::PerformerAffinities &/*_resp*/) -> bool
    {
      for (int ii = 0; ii < _req.affinity_size(); ++ii)
      {
        const auto &affinityMsg = _req.affinity(ii);
        const auto &entityId = affinityMsg.entity().id();

        auto pid = ecm.Component<components::ParentEntity>(entityId);

        ecm.CreateComponent(entityId,
          components::PerformerAffinity(affinityMsg.secondary_prefix()));

        auto isStatic = ecm.Component<components::Static>(pid->Data());
        auto isActive = ecm.Component<components::PerformerActive>(entityId);

        if (affinityMsg.secondary_prefix() == mgr->Namespace())
        {
          this->performers.push_back(entityId);
          *isStatic = components::Static(false);
          *isActive = components::PerformerActive(true);
          igndbg << "Secondary [" << mgr->Namespace()
                 << "] assigned affinity to performer [" << entityId << "]."
                 << std::endl;
        }
        else
        {
          *isStatic = components::Static(true);
          *isActive = components::PerformerActive(false);
        }
      }
      received = true;
      return true;
    };

  this->node.Advertise(topic, fcn);

  igndbg << "Secondary [" << mgr->Namespace()
         << "] waiting for affinity assignment." << std::endl;
  while (!received && !this->runner->stopReceived)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

/////////////////////////////////////////////////
bool SyncManagerSecondary::Sync()
{
  IGN_PROFILE("SyncManagerSecondary::Sync");

  // TODO(mjcarroll) this is where more advanced serialization/sync will go.
  auto &ecm = this->runner->entityCompMgr;

  ignition::msgs::Pose_V msg;

  for (const auto &entity : this->performers)
  {
    auto pid = ecm.Component<components::ParentEntity>(entity);
    auto pose = ecm.Component<components::Pose>(pid->Data());
    auto poseMsg = msg.add_pose();
    ignition::msgs::Set(poseMsg, pose->Data());
    poseMsg->set_id(entity);
  }
  this->posePub.Publish(msg);

  return true;
}
