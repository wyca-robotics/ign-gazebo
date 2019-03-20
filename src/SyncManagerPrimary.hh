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
    // forward declaration
    class SimulationRunner;

    /// \brief Used to manage syncronization between simulation primary and
    /// simulation secondaries.
    class IGNITION_GAZEBO_VISIBLE SyncManagerPrimary : public SyncManager
    {
      /// \brief Constructor
      /// \param[in] _runner A pointer to the simulationrunner that owns this
      public: explicit SyncManagerPrimary(SimulationRunner *_runner);

      /// \brief Distribute performer affinity to the secondaries in the
      /// distributed simulation environment.
      public: void DistributePerformers() override;

      /// \brief Syncronize state between primary and secondary
      /// EntityComponentManagers
      public: bool Sync() override;

      /// \brief Callback for when pose syncronization is received.
      /// \param[in] _msg Message with vector of incoming pose updates
      /// \TODO(mjcarroll) to be replaced with ECM sync.
      private: void OnPose(const ignition::msgs::Pose_V &_msg);

      /// \brief Ignition transport communication node
      private: ignition::transport::Node node;

      /// \brief Publisher for managed perfomers
      /// \TODO(mjcarroll) - Update this to utilize ECM sync
      private: ignition::transport::Node::Publisher posePub;

      /// \brief Mutex to protect collection of incoming pose messages
      private: std::mutex poseMutex;

      /// \brief Collection of incoming pose update messages
      private: std::vector<ignition::msgs::Pose_V> poseMsgs;
    };
    }
  }
}
#endif  // IGNITION_GAZEBO_SYNCMANAGER_HH
