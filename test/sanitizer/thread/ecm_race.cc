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

#include <chrono>
#include <gtest/gtest.h>

#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/World.hh"

using namespace ignition;
using namespace gazebo;

// Function that simulates the *update phase of systems where they can mutate 
// entities and their components
void SystemUpdate(EntityComponentManager& _ecm)
{
  using namespace std::chrono_literals;
  std::vector<EntityId> entities;
  // Create entities
  for (int i = 0; i < 100; ++i)
  {
    EntityId entity = _ecm.CreateEntity();
    _ecm.CreateComponent(entity, components::World());
    _ecm.CreateComponent(entity, components::Name("world_name"));
    entities.push_back(entity);
    std::this_thread::sleep_for(50ms);
  }
  std::this_thread::sleep_for(200ms);
  for (const auto entity : entities)
  {
    _ecm.RequestEraseEntity(entity);
  }
}

void SystemEach(EntityComponentManager& _ecm)
{
  _ecm.Each<components::World, components::Name>(
      [&](const EntityId,
          components::World *,
          components::Name * /*_name*/) -> bool
      {
        // TODO(addisu) component access is not thread safe.
        // *_name = components::Name("new_name");
        return true;
      });
}

class EntityCompMgrTest : public gazebo::EntityComponentManager
{
  public: void ProcessEntityErasures()
  {
    this->ProcessEraseEntityRequests();
  }
};

int main()
{
  const int iters = 1;
  const int nThreads = 5;

  EntityCompMgrTest  ecm;
  std::vector<std::thread> threads;

  for (int count = 0; count < iters; ++count)
  {
    for (int i = 0; i < nThreads; ++i)
    {
      threads.emplace_back(SystemUpdate, std::ref(ecm));
      threads.emplace_back(SystemEach, std::ref(ecm));
    }
    for (auto &thread : threads)
    {
      thread.join();
    }
    ecm.ProcessEntityErasures();
    threads.clear();
  }
}
