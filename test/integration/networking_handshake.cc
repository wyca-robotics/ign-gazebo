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
#include <gtest/gtest.h>
#include <chrono>
#include <condition_variable>

#include <ignition/transport/Node.hh>

#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/test_config.hh"  // NOLINT(build/include)

using namespace ignition;
using namespace gazebo;
using namespace std::chrono_literals;

class SystemFixture : public ::testing::Test
{
  protected: void SetUp() override
  {
    // Augment the system plugin path.  In SetUp to avoid test order issues.
    setenv("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
      (std::string(PROJECT_BINARY_PATH) + "/lib").c_str(), 1);
  }
};

/////////////////////////////////////////////////
// Get the current paused state from the world stats message
void TestPaused(bool _paused, const std::string &_worldName)
{
  std::condition_variable condition;
  std::mutex mutex;
  transport::Node node;
  bool paused = !_paused;

  std::function<void(const ignition::msgs::WorldStatistics &)> cb =
      [&](const ignition::msgs::WorldStatistics &_msg)
  {
    std::unique_lock<std::mutex> lock(mutex);
    paused = _msg.paused();
    condition.notify_all();
  };

  std::unique_lock<std::mutex> lock(mutex);
  node.Subscribe("/world/" + _worldName + "/stats", cb);
  auto status = condition.wait_for(lock, 1s);
  ASSERT_TRUE(status == std::cv_status::no_timeout);
  EXPECT_EQ(_paused, paused);
}

/////////////////////////////////////////////////
TEST_F(SystemFixture, NetworkingHandshake)
{
  // Configure the network primary to expect two clients.
  ServerConfig primaryConfig;
  primaryConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/distributed/primary.sdf");

  Server primary(primaryConfig);
  primary.SetUpdatePeriod(1us);
  EXPECT_FALSE(primary.Running());

  // Expecting only networking.
  EXPECT_EQ(1u, *primary.SystemCount());

  primary.Run(false);
  TestPaused(true, "distributed_primary");

  // Once all secondaries are added, we expect the simulation to begin.
  ServerConfig secondaryConfig;
  secondaryConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/distributed/secondary.sdf");

  Server secondary1(secondaryConfig);
  secondary1.SetUpdatePeriod(1us);
  EXPECT_FALSE(secondary1.Running());
  // Expecting networking and physics
  EXPECT_EQ(2u, *secondary1.SystemCount());
  secondary1.Run(false);
  TestPaused(true, "distributed_primary");

  Server secondary2(secondaryConfig);
  secondary2.SetUpdatePeriod(1us);
  EXPECT_FALSE(secondary2.Running());

  // Expecting networking and physics
  EXPECT_EQ(2u, *secondary2.SystemCount());
  secondary2.Run(false);
  TestPaused(false, "distributed_primary");
}
