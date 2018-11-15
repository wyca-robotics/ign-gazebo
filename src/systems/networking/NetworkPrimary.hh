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
#ifndef IGNITION_GAZEBO_SYSTEMS_NETWORKPRIMARY_HH_
#define IGNITION_GAZEBO_SYSTEMS_NETWORKPRIMARY_HH_

#include <memory>

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>
#include <ignition/gazebo/System.hh>

#include <ignition/transport/Node.hh>

#include "ClientManager.hh"

namespace ignition
{
namespace gazebo
{
namespace systems
{
  // Inline bracket to help doxygen filtering.
  inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    /// \brief System Plugin for primary distributed simulation instance.
    class IGNITION_GAZEBO_VISIBLE NetworkPrimary:
      public System,
      public ISystemConfigure
    {
      /// \brief Constructor
      public: explicit NetworkPrimary();

      /// \brief Destructor
      public: virtual ~NetworkPrimary();

      // Documentation inherited.
      public: void Configure(const sdf::ElementPtr &_sdf,
                             EntityComponentManager &_ecm,
                             EventManager &_eventMgr) override;

      /// \brief Run the plugin's background threads
      public: void Run();

      /// \brief Stop the plugin's background threads
      public: void Stop();

      /// \brief Get whether the plugin's background threads are running.
      /// \return True if background threads are executing.
      public: bool Running() const;

      /// \brief Wait for all secondaries to be present before continuing
      private: void WaitForClients();

      /// \brief Main execution loop of the network primary plugin.
      private: void WorkLoop();

      /// \brief Thread responsible for managing secondary clients.
      private: std::unique_ptr<std::thread> clientThread;

      /// \brief Thread responsible for main plugin execution loop
      private: std::unique_ptr<std::thread> workerThread;

      private: EventManager *eventMgr;

      /// \brief used to indicate that run has been called and that the
      /// background threads are executing.
      private: std::atomic<bool> running;

      /// \brief Transport Node.
      private: std::shared_ptr<ignition::transport::Node> node;

      /// \brief Client Manager.
      private: std::unique_ptr<ClientManager> manager;
    };
  }  // namespace IGNITION_GAZEBO_VERSION_NAMESPACE
}  // namespace systems
}  // namespace gazebo
}  // namespace ignition

#endif  // IGNITION_GAZEBO_SYSTEMS_NETWORKPRIMARY_HH_
