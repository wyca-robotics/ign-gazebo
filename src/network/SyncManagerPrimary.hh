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

#ifndef IGNITION_GAZEBO_SYNCMANAGERPRIMARY_HH
#define IGNITION_GAZEBO_SYNCMANAGERPRIMARY_HH

#include <vector>

#include "ignition/gazebo/config.hh"
#include "ignition/transport/Node.hh"
#include "SyncManager.hh"

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    //
    /// \brief Used to manage syncronization between simulation primary and
    /// simulation secondaries.
    class IGNITION_GAZEBO_VISIBLE SyncManagerPrimary : public SyncManager
    {
      /// \brief Constructor
      /// \param[in] _runner A pointer to the simulationrunner that owns this
      public: explicit SyncManagerPrimary(EntityComponentManager &_ecm,
          NetworkManager *_networkManager);

      /// \brief Distribute performer affinity to the secondaries in the
      /// distributed simulation environment.
      public: void Initialize() override;

      /// \brief Syncronize state between primary and secondary
      /// EntityComponentManagers
      public: bool Sync() override;

      /// \brief Callback for when state syncronization is received.
      /// \param[in] _msg Message with incoming state updates.
      private: void OnState(const ignition::msgs::SerializedState &_msg);

      /// \brief Ignition transport communication node
      private: ignition::transport::Node node;

      /// \brief Mutex to protect collection of incoming state messages
      private: std::mutex msgMutex;

      /// \brief Collection of incoming state update messages
      private: std::vector<ignition::msgs::SerializedState> stateMsgs;
    };
    }
  }
}
#endif  // IGNITION_GAZEBO_SYNCMANAGER_HH
