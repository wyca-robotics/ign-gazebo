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

#ifndef IGNITION_GAZEBO_SYNCMANAGER_HH
#define IGNITION_GAZEBO_SYNCMANAGER_HH

#include "ignition/gazebo/config.hh"
#include "ignition/gazebo/EntityComponentManager.hh"

#include "NetworkManager.hh"

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {

    /// \brief Used to manage syncronization between simulation primary and
    /// simulation secondaries.
    ///
    /// Utilizes the concept of "performers" introduced in the level manager.
    /// Each performer has an affinity, which is the mapping between the
    /// performer and the distributed simulation secondary. The distributed
    /// simulation primary does not have any performers of itself, but
    /// purely manages the distribution of performers to the secondaries.
    ///
    /// ## Primary - Secondary communication
    ///
    /// * The primary notifies secondaries of affinity changes using each
    /// secondary's `/<namespace>/affinity` service.
    ///
    /// * Each secondary sends the state update of its performers via the
    /// /state_update topic.
    ///
    /// ## Components
    ///
    /// The SyncManager will also attach components to the performer entity to
    /// manage the distribution:
    /// * PerformerAffinity: notes the secondary that each performer is
    /// associated with.
    /// * PerformerActive: marks whether the performer is active on this
    /// secondary - so it isn't really transferable simulation state.
    class SyncManager
    {
      /// \brief Constructor
      /// \param[in] _ecm The entity-component manager.
      /// \param[in] _networkManager Pointer to network manager.
      public: explicit SyncManager(EntityComponentManager &_ecm,
          NetworkManager *_networkManager);

      /// \brief Distribute performer affinity to the secondaries in the
      /// distributed simulation environment.
      public: virtual void Initialize() = 0;

      /// \brief Syncronize state between primary and secondary
      /// EntityComponentManagers
      public: virtual bool Sync() = 0;

      /// \brief Distribute performer affinity to the secondaries in the
      /// distributed simulation environment.
      public: bool Initialized() const;

      /// \brief Pointer to entity component manager.
      protected: EntityComponentManager *ecm{nullptr};

      /// \brief Pointer to network manager.
      protected: NetworkManager *networkManager{nullptr};

      /// \brief Whether it's been initialized.
      protected: bool initialized{false};
    };
    }
  }
}
#endif  // IGNITION_GAZEBO_SYNCMANAGER_HH
