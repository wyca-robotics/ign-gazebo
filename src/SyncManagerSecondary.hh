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

#ifndef IGNITION_GAZEBO_SYNCMANAGERSECONDARY_HH
#define IGNITION_GAZEBO_SYNCMANAGERSECONDARY_HH

#include <string>
#include <unordered_map>
#include <vector>

#include "ignition/gazebo/config.hh"
#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/Types.hh"
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
    class SyncManagerSecondary : public SyncManager
    {
      /// \brief Constructor
      /// \param[in] _runner A pointer to the simulationrunner that owns this
      public: explicit SyncManagerSecondary(SimulationRunner *_runner);

      /// \brief Distribute performer affinity to the secondaries in the
      /// distributed simulation environment.
      public: void DistributePerformers() override;

      /// \brief Syncronize state between primary and secondary
      /// EntityComponentManagers
      public: bool Sync() override;

      /// \brief Ignition transport communication node
      private: ignition::transport::Node node;

      /// \brief Publisher for managed perfomers
      private: ignition::transport::Node::Publisher statePub;

      /// \brief Collection of performers associated with a secondary
      private: std::unordered_set<Entity> performers;
    };
    }
  }
}
#endif  // IGNITION_GAZEBO_SYNCMANAGER_HH
