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
#include "NetworkManagerPrimary.hh"

#include <algorithm>
#include <string>

#include "ignition/common/Console.hh"
#include "ignition/common/Util.hh"
#include "ignition/gazebo/Conversions.hh"
#include "ignition/gazebo/Events.hh"

#include "msgs/peer_control.pb.h"
#include "msgs/simulation_step.pb.h"

#include "NetworkManagerPrivate.hh"
#include "PeerTracker.hh"

using namespace ignition;
using namespace gazebo;

//////////////////////////////////////////////////
NetworkManagerPrimary::NetworkManagerPrimary(
    std::function<void(UpdateInfo &_info)> _stepFunction,
    EventManager *_eventMgr,
    const NetworkConfig &_config, const NodeOptions &_options):
  NetworkManager(_stepFunction, _eventMgr, _config, _options),
  node(_options)
{
  this->simStepPub = this->node.Advertise<private_msgs::SimulationStep>("step");

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
void NetworkManagerPrimary::Initialize()
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

    igndbg << "Attempting to register secondary [" << topic << "]" << std::endl;
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
        igndbg << "Peer service call failed [" << sc->prefix << "]"
          << std::endl;
      }
    }
    else
    {
      igndbg << "Peer service call timed out [" << sc->prefix << "]"
        << std::endl;
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
std::string NetworkManagerPrimary::Namespace() const
{
  return "";
}

//////////////////////////////////////////////////
std::map<std::string, SecondaryControl::Ptr>&
NetworkManagerPrimary::Secondaries()
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

//////////////////////////////////////////////////
bool NetworkManagerPrimary::Step(UpdateInfo &_info, EntityComponentManager &_ecm)
{
  // Check all secondaries have been registered
  bool ready = true;
  for (const auto &secondary : this->secondaries)
  {
    ready &= secondary.second->ready;
  }

  if (!ready)
    return false;

  // Send time + affinities + new performer states to secondaries
  private_msgs::SimulationStep step;
  step.set_iteration(_info.iterations);
  step.set_paused(_info.paused);
  step.mutable_simtime()->CopyFrom(convert<msgs::Time>(_info.simTime));

  auto stepSizeSecNsec = math::durationToSecNsec(_info.dt);
  step.set_stepsize(stepSizeSecNsec.second);

  // TODO: populate affinities
  // TODO: populate new performer states

  this->secondaryStates.clear();
  for (const auto &secondary : this->secondaries)
  {
    this->node.Request(secondary.second->prefix + "/step", step,
        &NetworkManagerPrimary::OnStepResponse, this);
  }

  // Block until all secondaries are done
  while (this->secondaryStates.size() < this->secondaries.size())
  {
    std::this_thread::sleep_for(std::chrono::nanoseconds(1));
  }

  // TODO: Assemble state updates received from secondaries

  // Update global state and let level manager recalculate affinities
  this->dataPtr->stepFunction(_info);

  return true;
}

