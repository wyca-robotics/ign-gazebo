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
#include "msgs/simulation_step.pb.h"

#include <algorithm>
#include <string>

#include <ignition/common/Console.hh>
#include <ignition/common/Util.hh>
#include <ignition/common/Profiler.hh>

#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Performer.hh"
#include "ignition/gazebo/components/PerformerAffinity.hh"
#include "ignition/gazebo/components/Static.hh"
#include "ignition/gazebo/Conversions.hh"
#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/Events.hh"

#include "NetworkManagerPrimary.hh"
#include "NetworkManagerPrivate.hh"
#include "PeerTracker.hh"
#include "components/PerformerActive.hh"

using namespace ignition;
using namespace gazebo;

//////////////////////////////////////////////////
NetworkManagerPrimary::NetworkManagerPrimary(
    std::function<void(UpdateInfo &_info)> _stepFunction,
    EntityComponentManager &_ecm, EventManager *_eventMgr,
    const NetworkConfig &_config, const NodeOptions &_options):
  NetworkManager(_stepFunction, _ecm, _eventMgr, _config, _options),
  node(_options)
{
  auto eventMgr = this->dataPtr->eventMgr;
  if (eventMgr)
  {
    this->dataPtr->peerRemovedConn = eventMgr->Connect<PeerRemoved>(
        [this](PeerInfo _info)
        {
          if (_info.role == NetworkRole::SimulationSecondary)
          {
            ignerr << "Secondary removed, stopping simulation" << std::endl;
            this->dataPtr->eventMgr->Emit<events::Stop>();
          }
        });

    this->dataPtr->peerStaleConn = eventMgr->Connect<PeerStale>(
        [this](PeerInfo _info)
        {
          if (_info.role == NetworkRole::SimulationSecondary)
          {
            ignerr << "Secondary went stale, stopping simulation" << std::endl;
            this->dataPtr->eventMgr->Emit<events::Stop>();
          }
        });
  }
  else
  {
    ignwarn << "NetworkManager started without EventManager. "
      << "Distributed environment may not terminate correctly" << std::endl;
  }
}

//////////////////////////////////////////////////
void NetworkManagerPrimary::Handshake()
{
  auto peers = this->dataPtr->tracker->SecondaryPeers();
  for (const auto &peer : peers)
  {
    private_msgs::PeerControl req, resp;
    req.set_enable_sim(true);

    auto sc = std::make_unique<SecondaryControl>();
    sc->id = peer;
    sc->prefix = peer.substr(0, 8);

    bool result;
    std::string topic {sc->prefix + "/control"};
    unsigned int timeout = 5000;

    igndbg << "Registering secondary [" << topic << "]" << std::endl;
    bool executed = this->node.Request(topic, req, timeout, resp, result);

    if (executed)
    {
      if (result)
      {
        igndbg << "Peer initialized [" << sc->prefix << "]" << std::endl;
        sc->ready = true;
      }
      else
      {
        ignerr << "Peer service call failed [" << sc->prefix << "]"
          << std::endl;
      }
    }
    else
    {
      ignerr << "Peer service call timed out [" << sc->prefix << "], waited "
             << timeout << " ms" << std::endl;
    }

    this->secondaries[sc->prefix] = std::move(sc);
  }
}

//////////////////////////////////////////////////
bool NetworkManagerPrimary::Ready() const
{
  // The detected number of peers in the "Secondary" role must match
  // the number exepected (set via configuration of environment).
  auto nSecondary = this->dataPtr->tracker->NumSecondary();
  return (nSecondary == this->dataPtr->config.numSecondariesExpected);
}

//////////////////////////////////////////////////
bool NetworkManagerPrimary::Step(UpdateInfo &_info)
{
  IGN_PROFILE("NetworkManagerPrimary::Step");
  // Check all secondaries have been registered
  bool ready = true;
  for (const auto &secondary : this->secondaries)
  {
    ready &= secondary.second->ready;
  }

  if (!ready)
  {
    ignerr << "Trying to step network primary before all peers are ready."
           << std::endl;
    return false;
  }

  private_msgs::SimulationStep step;

  // Step time
  step.set_iteration(_info.iterations);
  step.set_paused(_info.paused);
  step.mutable_simtime()->CopyFrom(convert<msgs::Time>(_info.simTime));

  auto stepSizeSecNsec = math::durationToSecNsec(_info.dt);
  step.set_stepsize(stepSizeSecNsec.second);

  // Affinities that changed this step
  auto secondaryIt = this->secondaries.begin();

  // TODO(louise) Asign affinities according to level changes instead of
  // round-robin
  static bool tmpShortCut{false};
  if (!tmpShortCut)
  {
  tmpShortCut = true;

  // Go through performers and assign affinities
  this->dataPtr->ecm->Each<components::Performer, components::ParentEntity>(
    [&](const Entity &_entity,
        const components::Performer *,
        const components::ParentEntity *_parent) -> bool
    {
      auto pid = _parent->Data();
      auto parentName = this->dataPtr->ecm->Component<components::Name>(pid);
      if (!parentName)
      {
        ignerr << "Internal error: entity [" << _entity
               << "]'s parent missing name." << std::endl;
        return true;
      }

      auto affinityMsg = step.add_affinity();
      affinityMsg->mutable_entity()->set_name(parentName->Data());
      affinityMsg->mutable_entity()->set_id(_entity);
      affinityMsg->set_secondary_prefix(secondaryIt->second->prefix);

      auto isActive =
          this->dataPtr->ecm->Component<components::PerformerActive>(_entity);
      *isActive = components::PerformerActive(true);

      // TODO(louise) Set affinity according to levels, not round-robin
      this->dataPtr->ecm->CreateComponent(_entity,
          components::PerformerAffinity(secondaryIt->second->prefix));

      secondaryIt++;
      if (secondaryIt == this->secondaries.end())
      {
        secondaryIt = this->secondaries.begin();
      }

      return true;
    });
  }

  // TODO(louise): send performer states for new affinities

  // Send step to all secondaries in parallel
  this->secondaryStates.clear();
  for (const auto &secondary : this->secondaries)
  {
    this->node.Request(secondary.second->prefix + "/step", step,
        &NetworkManagerPrimary::OnStepResponse, this);
  }

  // Block until all secondaries are done
  {
    IGN_PROFILE("Waiting for secondaries");
    while (this->secondaryStates.size() < this->secondaries.size())
    {
      std::this_thread::sleep_for(std::chrono::nanoseconds(1));
    }
  }

  // Update primary state with states received from secondaries
  {
    IGN_PROFILE("Updating primary state");
    for (const auto &msg : this->secondaryStates)
    {
      this->dataPtr->ecm->SetState(msg);
    }
    this->secondaryStates.clear();
  }

  // Step all systems
  this->dataPtr->stepFunction(_info);

  return true;
}

//////////////////////////////////////////////////
std::string NetworkManagerPrimary::Namespace() const
{
  return "";
}

//////////////////////////////////////////////////
std::map<std::string, SecondaryControl::Ptr>
    &NetworkManagerPrimary::Secondaries()
{
  return this->secondaries;
}

//////////////////////////////////////////////////
void NetworkManagerPrimary::OnStepResponse(
    const msgs::SerializedState &_res, const bool _result)
{
  if (_result)
    this->secondaryStates.push_back(_res);
}

