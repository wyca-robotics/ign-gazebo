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

#include "msgs/peer_control.pb.h"

#include <algorithm>
#include <string>

#include <ignition/common/Console.hh>
#include <ignition/common/Util.hh>
#include <ignition/common/Profiler.hh>

#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/PerformerAffinity.hh"
#include "ignition/gazebo/components/Static.hh"
#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/Events.hh"

#include "NetworkManagerPrivate.hh"
#include "NetworkManagerSecondary.hh"
#include "PeerTracker.hh"
#include "components/PerformerActive.hh"

using namespace ignition;
using namespace gazebo;

//////////////////////////////////////////////////
NetworkManagerSecondary::NetworkManagerSecondary(
    std::function<void(UpdateInfo &_info)> _stepFunction,
    EntityComponentManager &_ecm,
    EventManager *_eventMgr,
    const NetworkConfig &_config, const NodeOptions &_options):
  NetworkManager(_stepFunction, _ecm, _eventMgr, _config, _options),
  node(_options)
{
  std::string controlService{this->Namespace() + "/control"};
  if (!this->node.Advertise(controlService, &NetworkManagerSecondary::OnControl,
      this))
  {
    ignerr << "Error advertising PeerControl service [" << controlService
           << "]" << std::endl;
  }
  else
  {
    igndbg << "Advertised PeerControl service on [" << controlService << "]"
      << std::endl;
  }

  std::string stepService{this->Namespace() + "/step"};
  if (!this->node.Advertise(stepService, &NetworkManagerSecondary::StepService,
      this))
  {
    ignerr << "Error advertising Step service [" << stepService
           << "]" << std::endl;
  }
  else
  {
    igndbg << "Advertised Step service on [" << stepService << "]"
      << std::endl;
  }

  auto eventMgr = this->dataPtr->eventMgr;
  if (eventMgr)
  {
    // Set a flag when the executable is stopping to cleanly exit.
    this->stoppingConn = eventMgr->Connect<events::Stop>(
        [this]()
    {
      this->stopReceived = true;
    });

    this->dataPtr->peerRemovedConn = eventMgr->Connect<PeerRemoved>(
        [this](PeerInfo _info)
    {
          if (_info.role == NetworkRole::SimulationPrimary)
          {
            ignmsg << "Primary removed, stopping simulation" << std::endl;
            this->dataPtr->eventMgr->Emit<events::Stop>();
          }
    });

    this->dataPtr->peerStaleConn = eventMgr->Connect<PeerStale>(
        [this](PeerInfo _info)
    {
          if (_info.role == NetworkRole::SimulationPrimary)
          {
            ignerr << "Primary went stale, stopping simulation" << std::endl;
            this->dataPtr->eventMgr->Emit<events::Stop>();
          }
    });
  }
}

//////////////////////////////////////////////////
bool NetworkManagerSecondary::Ready() const
{
  // The detected number of peers in the "Primary" role must be 1
  auto primaries = this->dataPtr->tracker->NumPrimary();
  return (primaries == 1);
}

//////////////////////////////////////////////////
void NetworkManagerSecondary::Handshake()
{
  while (!this->enableSim && !this->stopReceived)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

//////////////////////////////////////////////////
std::string NetworkManagerSecondary::Namespace() const
{
  return this->dataPtr->peerInfo.id.substr(0, 8);
}

//////////////////////////////////////////////////
bool NetworkManagerSecondary::OnControl(const private_msgs::PeerControl &_req,
                                        private_msgs::PeerControl& _resp)
{
  this->enableSim = _req.enable_sim();
  _resp.set_enable_sim(this->enableSim);
  return true;
}

/////////////////////////////////////////////////
bool NetworkManagerSecondary::StepService(
    const private_msgs::SimulationStep &_req,
    msgs::SerializedState &_res)
{
  IGN_PROFILE("NetworkManagerSecondary::StepService");

  // Wait for previous step to complete first
//  {
//    std::unique_lock<std::mutex> lock(this->stepMutex);
//    if (this->stepComplete)
//    {
//      ignerr << "Step complete before starting" << std::endl;
//    }
//  }

  // Throttle the number of step messages going to the debug output.
  if (!_req.paused() && _req.iteration() % 1000 == 0)
  {
    igndbg << "Network iterations: " << _req.iteration()
           << std::endl;
  }

  // Update affinities
  for (int i = 0; i < _req.affinity_size(); ++i)
  {
    const auto &affinityMsg = _req.affinity(i);
    const auto &entityId = affinityMsg.entity().id();

    auto pid = this->dataPtr->ecm->Component<components::ParentEntity>(entityId);

    this->dataPtr->ecm->CreateComponent(entityId,
      components::PerformerAffinity(affinityMsg.secondary_prefix()));

    // TODO(louise) Remove instead of setting static, and then
    // PerformerActive is not needed
    auto isStatic =
        this->dataPtr->ecm->Component<components::Static>(pid->Data());
    auto isActive =
        this->dataPtr->ecm->Component<components::PerformerActive>(entityId);

    if (affinityMsg.secondary_prefix() == this->Namespace())
    {
      this->performers.insert(entityId);
      *isStatic = components::Static(false);
      *isActive = components::PerformerActive(true);
      igndbg << "Secondary [" << this->Namespace()
             << "] assigned affinity to performer [" << entityId << "]."
             << std::endl;
    }
    else
    {
      *isStatic = components::Static(true);
      *isActive = components::PerformerActive(false);
    }
  }

  // Update info
  UpdateInfo info;
  info.iterations = _req.iteration();
  info.paused = _req.paused();
  info.dt = std::chrono::steady_clock::duration(
      std::chrono::nanoseconds(_req.stepsize()));
  info.simTime = std::chrono::steady_clock::duration(
      std::chrono::seconds(_req.simtime().sec()) +
      std::chrono::nanoseconds(_req.simtime().nsec()));

  // Update state with all the performer's models
  std::unordered_set<Entity> models;
  for (const auto &perf : this->performers)
  {
    models.insert(
        this->dataPtr->ecm->Component<components::ParentEntity>(perf)->Data());
  }
  _res = this->dataPtr->ecm->State(models);

  // Step runner
  this->dataPtr->stepFunction(info);

  // Finish step
  {
    std::unique_lock<std::mutex> lock(this->stepMutex);
    this->stepComplete = false;
    lock.unlock();
    this->stepCv.notify_all();
  }

  return true;
}

//////////////////////////////////////////////////
bool NetworkManagerSecondary::Step(UpdateInfo &)
{
  IGN_PROFILE("NetworkManagerSecondary::Step");
  if (!this->enableSim || this->stopReceived)
  {
    return false;
  }

  std::unique_lock<std::mutex> lock(this->stepMutex);
  this->stepComplete = false;
  auto status = this->stepCv.wait_for(lock, std::chrono::seconds(5), [this]()
  {
    return this->stepComplete;
  });

  if (!status)
  {
    ignerr << "Timed out waiting for step to complete." << std::endl;
  }

  return status;
}

