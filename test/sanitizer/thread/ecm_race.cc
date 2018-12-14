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

#include <random>
#include <chrono>
#include <gtest/gtest.h>

#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/World.hh"

using namespace ignition;
using namespace gazebo;

//////////////////////////////////////////////////
/// \breif Helper function to sleep a random amount of time
/// \param[in] _max Maximum number of seconds to sleep
void SleepRandom(double _max)
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0, _max);
  const double toSleep = dis(gen);
  std::this_thread::sleep_for(std::chrono::duration<double>(toSleep));
}

//////////////////////////////////////////////////
// Function that simulates the *update phase of systems where they can mutate
// entities and their components
void SystemUpdate(EntityComponentManager& _ecm)
{
  using namespace std::chrono_literals;
  std::vector<EntityId> entities;
  std::vector<std::pair<EntityId, ComponentKey>> components;

  // sleep a random amount before starting
  SleepRandom(0.2);

  // Create entities
  for (int i = 0; i < 100; ++i)
  {
    EntityId entity = _ecm.CreateEntity();
    _ecm.CreateComponent(entity, components::World());
    auto compKey = _ecm.CreateComponent(entity, components::Name("world_name"));
    entities.push_back(entity);
    components.push_back(std::make_pair(entity,compKey));
    // sleep a small random amount
    SleepRandom(0.05);
  }

  SleepRandom(0.1);
  for (const auto [entity, compKey] : components)
  {
    _ecm.RemoveComponent(entity, compKey);
  }
  // sleep a random amount before erasing
  SleepRandom(0.1);
  for (const auto entity : entities)
  {
    _ecm.RequestEraseEntity(entity);
  }
}

//////////////////////////////////////////////////
void SystemEach(EntityComponentManager& _ecm)
{
  _ecm.Each<components::World, components::Name>(
      [&](const EntityId _entity,
          const components::World *,
          const components::Name * /* _name */) -> bool
      {
        _ecm.SetComponent(_entity, components::Name("new_name"));
        _ecm.RequestSetComponent(_entity, components::Name("another_name"));
        return true;
      });
}

//////////////////////////////////////////////////
/// A proxy class for calling protected methods in EntityComponentManager
class EntityCompMgrTest : public gazebo::EntityComponentManager
{
  public: void ProcessAllRequests()
  {
    this->ProcessEraseEntityRequests();
  }
};

int main()
{
  const int iters = 2;
  const int nThreads = 20;

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
    ecm.ProcessAllRequests();
    threads.clear();
  }
}
