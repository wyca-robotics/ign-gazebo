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

#include "ClientManager.hh"

#include <ignition/common/Console.hh>
#include <ignition/plugin/RegisterMore.hh>

#include <chrono>

using namespace ignition::gazebo::systems;

/////////////////////////////////////////////////
ClientManager::ClientManager(const NodePtr &_node,
                             size_t _expected_num_clients):
  node(_node),
  expected_num_clients(_expected_num_clients)
{
  node->Advertise("/register", &ClientManager::registerClient, this);
  node->Advertise("/unregister", &ClientManager::unregisterClient, this);
}

/////////////////////////////////////////////////
ClientManager::~ClientManager()
{
}

/////////////////////////////////////////////////
bool ClientManager::Ready() {
  std::unique_lock<std::mutex> lock(client_mutex);
  return this->clients.size() == this->expected_num_clients;
}

/////////////////////////////////////////////////
bool ClientManager::registerClient(const msgs::ConnectionRequest &_req,
                                   msgs::ConnectionResponse &/*_resp*/)
{
  ClientInfo info;
  info.uuid = _req.secondary_uuid();

  std::unique_lock<std::mutex> lock(client_mutex);
  if (clients.find(info.uuid) != clients.end()) {
    igndbg << "Attempt to register network secondary twice." << std::endl;
    return false;
  }

  clients[info.uuid] = info;

  return true;
}

/////////////////////////////////////////////////
bool ClientManager::unregisterClient(const msgs::ConnectionRequest &_req,
                                     msgs::ConnectionResponse &/*_resp*/)
{
  std::unique_lock<std::mutex> lock(client_mutex);

  auto it = clients.find(_req.secondary_uuid());

  if (it == clients.end()) {
    igndbg << "Attempt to unregister unknown network secondary." << std::endl;
    return false;
  }

  clients.erase(it);

  return true;
}

