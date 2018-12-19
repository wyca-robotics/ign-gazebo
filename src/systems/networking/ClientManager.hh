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

#ifndef IGNITION_GAZEBO_SYSTEMS_CLIENTMANAGER_HH_
#define IGNITION_GAZEBO_SYSTEMS_CLIENTMANAGER_HH_

#include <list>
#include <map>
#include <memory>
#include <string>

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>

#include <ignition/transport/Node.hh>

#include "msgs/client.pb.h"

namespace ignition
{
namespace gazebo
{
namespace systems
{
  // Inline bracket to help doxygen filtering.
  inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    /// \brief Structure to hold all relevant Secondary information.
    struct IGNITION_GAZEBO_VISIBLE ClientInfo
    {
      /// \brief Network Secondary Unique Identifier
      std::string uuid;
    };

    /// \brief Class to manage network secondary registration/deregistration
    /// The ClientManager is used by the NetworkPrimary to track the connection
    /// state and simulation status of connected instances running with the
    /// NetworkSecondary plugin.
    class IGNITION_GAZEBO_VISIBLE ClientManager
    {
      using NodePtr = std::shared_ptr<ignition::transport::Node>;

      /// \brief Constructor
      /// \param[in] _node Instance of an ignition transport Node to use.
      /// \param[in] _expected_num_clients Expected number of clients.
      public: ClientManager(const NodePtr &_node,
                            size_t _expected_num_clients);

      /// \brief Destructor
      public: ~ClientManager();

      /// \brief Check if manager has all expected clients.
      /// \returns True if ready.
      public: bool Ready();

      /// \brief Callback for secondary registration request.
      /// \param[in] _req Registration request.
      /// \param[in] _resp Response.
      /// \returns True if successfully handled.
      private: bool RegisterClient(const msgs::ConnectionRequest &_req,
                                   msgs::ConnectionResponse &_resp);

      /// \brief Callback for secondary deregistration request
      /// \param[in] _req Unregistration request.
      /// \param[in] _resp Response.
      /// \returns True if successfully handled.
      private: bool UnregisterClient(const msgs::ConnectionRequest &_req,
                                     msgs::ConnectionResponse &_resp);

      /// \brief Transport Node.
      private: NodePtr node;

      /// \brief Expected number of network secondaries to join
      private: size_t expected_num_clients;

      /// \breif Clients mutex
      private: std::mutex clientMutex;

      /// \brief Registered client information.
      private: std::map<std::string, ClientInfo> clients;
    };
  }  // namespace IGNITION_GAZEBO_VERSION_NAMESPACE
}  // namespace systems
}  // namespace gazebo
}  // namespace ignition

#endif  // IGNITION_GAZEBO_SYSTEMS_CLIENTMANAGER_HH_

