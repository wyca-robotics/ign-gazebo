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

#include "NetworkSecondary.hh"

#include <ignition/common/Uuid.hh>
#include <ignition/transport/Node.hh>

#include "ignition/plugin/RegisterMore.hh"

#include "msgs/client.pb.h"

using namespace ignition::gazebo::systems;

/////////////////////////////////////////////////
NetworkSecondary::NetworkSecondary():
  node(std::make_shared<ignition::transport::Node>())
{
}

/////////////////////////////////////////////////
NetworkSecondary::~NetworkSecondary()
{
}

/////////////////////////////////////////////////
void NetworkSecondary::Configure(
    const std::shared_ptr<const sdf::Element> &/*_sdf*/,
    EntityComponentManager &/*_ecm*/,
    EventManager &/*_eventMgr*/)
{
  this->RegisterWithPrimary();
}

/////////////////////////////////////////////////
bool NetworkSecondary::RegisterWithPrimary()
{
  msgs::ConnectionRequest req;

  req.set_secondary_uuid(this->uuid.String());

  msgs::ConnectionResponse resp;
  bool result;
  unsigned int timeout = 1000;
  bool executed = this->node->Request("/register", req, timeout, resp,
                                      result);

  if (executed)
  {
    if (result)
    {
      igndbg << "UUID [" << this->uuid.String() << "]: Registration success"
             << std::endl;
      this->isRegistered = true;
      return true;
    }
    else
    {
      igndbg << "UUID [" << this->uuid.String() << "]: Registration failure"
             << std::endl;
      this->isRegistered = false;
      return false;
    }
  }
  else
  {
      igndbg << "UUID [" << this->uuid.String() << "]: Registration timeout"
             << std::endl;
    this->isRegistered = false;
    return false;
  }
}

IGNITION_ADD_PLUGIN(ignition::gazebo::systems::NetworkSecondary,
                    ignition::gazebo::System,
                    NetworkSecondary::ISystemConfigure)
