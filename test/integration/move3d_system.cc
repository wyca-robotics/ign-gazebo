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
#include <ignition/common/Console.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/test_config.hh"

#include "plugins/MockSystem.hh"

using namespace ignition;
using namespace gazebo;

/// \brief Test Move3dTest system
class Move3dTest : public ::testing::Test
{
  // Documentation inherited
  protected: void SetUp() override
  {
    ignition::common::Console::SetVerbosity(4);
    setenv("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
           (std::string(PROJECT_BINARY_PATH) + "/lib").c_str(), 1);
  }
};

class Relay
{
  public: Relay()
  {
    auto plugin = loader.LoadPlugin("libMockSystem.so",
                                "ignition::gazebo::MockSystem",
                                nullptr);
    EXPECT_TRUE(plugin.has_value());

    this->systemPtr = plugin.value();

    this->mockSystem =
        dynamic_cast<MockSystem *>(systemPtr->QueryInterface<System>());
    EXPECT_NE(nullptr, this->mockSystem);
  }

  public: Relay &OnPreUpdate(MockSystem::CallbackType cb)
  {
    this->mockSystem->preUpdateCallback = cb;
    return *this;
  }

  public: Relay &OnUpdate(MockSystem::CallbackType cb)
  {
    this->mockSystem->updateCallback = cb;
    return *this;
  }

  public: Relay &OnPostUpdate(MockSystem::CallbackTypeConst cb)
  {
    this->mockSystem->postUpdateCallback = cb;
    return *this;
  }

  public: SystemPluginPtr systemPtr;

  private: SystemLoader loader;
  private: MockSystem *mockSystem;
};

/////////////////////////////////////////////////
TEST_F(Move3dTest, PublishCmd)
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/move3d.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  const std::string modelName = "sphere";
  const ignition::math::Vector3d sphereVelocity{1.0, 0, 0};

  // Create a system that sets the link velocity and records the last pose of
  // the sphere
  Relay testSystem;
  std::vector<math::Pose3d> poses;
  testSystem.OnPostUpdate([&](const gazebo::UpdateInfo &,
                              const gazebo::EntityComponentManager &_ecm)
      {
        _ecm.Each<components::Model, components::Name, components::Pose>(
            [&](const ignition::gazebo::EntityId &,
                const components::Model *,
                const components::Name *_name,
                const components::Pose *_pose) -> bool
            {
              if (_name->Data() == modelName)
              {
                poses.push_back(_pose->Data());
              }
              return true;
            });
      });

  server.AddSystem(testSystem.systemPtr);

  // Run server and check that the sphere's x position didn't change
  const size_t iters = 1000;
  server.Run(true, iters, false);

  EXPECT_EQ(iters, poses.size());
  EXPECT_DOUBLE_EQ(poses.front().Pos().X(), poses.back().Pos().X());

  // Publish command and check that the sphere moved in the commanded direction.
  transport::Node node;
  auto pub = node.Advertise<msgs::Vector3d>("/sphere/move3d/linear_vel");

  msgs::Vector3d msg = msgs::Convert(sphereVelocity);
  pub.Publish(msg);
  server.Run(true, iters, false);

  EXPECT_EQ(2 * iters, poses.size());

  // calculate expected position in the x axis based on the sphere's
  // constant velocity
  const double dt = 0.001;
  const double expectedSphereXDist = sphereVelocity.X() * dt * iters;
  EXPECT_NEAR(expectedSphereXDist,
              poses.back().Pos().X() - poses.front().Pos().X(), 5e-6);
}
