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
#include <ignition/msgs/pose_v.pb.h>

#include <string>

#include <ignition/common/Console.hh>
#include <ignition/common/Filesystem.hh>
#include <ignition/transport/Node.hh>
#include <ignition/transport/log/Log.hh>
#include <ignition/transport/log/Batch.hh>
#include <ignition/transport/log/MsgIter.hh>
#include <ignition/transport/log/QualifiedTime.hh>
#include <ignition/math/Pose3.hh>

#include <sdf/Root.hh>
#include <sdf/World.hh>
#include <sdf/Element.hh>

#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/ServerConfig.hh"
#include "ignition/gazebo/test_config.hh"

using namespace ignition;
using namespace gazebo;

//////////////////////////////////////////////////
class LogSystemTest : public ::testing::Test
{
  /// \brief Clear and get a path to store logs during the test
  /// \return The destination path
  public: std::string InitializeLogPath()
  {
    std::string logDest = common::joinPaths(PROJECT_BINARY_PATH, "test",
      "test_log");
    if (common::exists(logDest))
    {
      common::removeAll(logDest);
    }
    return logDest;
  }

  // Documentation inherited
  protected: void SetUp() override
  {
    common::Console::SetVerbosity(4);
    setenv("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
           (std::string(PROJECT_BINARY_PATH) + "/lib").c_str(), 1);
  }
};

/////////////////////////////////////////////////
// This test checks that a file is created by log recorder
TEST_F(LogSystemTest, CreateLogFile)
{
  // Configure to use binary path as log destination
  auto logDest = this->InitializeLogPath();

  // Configure server to load falling world with LogRecorder plugin
  ServerConfig::PluginInfo pluginInfo;
  pluginInfo.SetEntityName("default");
  pluginInfo.SetEntityType("world");
  pluginInfo.SetFilename("libignition-gazebo-log-system.so");
  pluginInfo.SetName("ignition::gazebo::systems::LogRecord");

  auto pluginElem = std::make_shared<sdf::Element>();
  pluginElem->SetName("plugin");
  pluginElem->AddAttribute("name", "string",
      "ignition::gazebo::systems::LogRecord", true);
  pluginElem->AddAttribute("filename", "string", "libignition-gazebo-log-system.so",
      true);

  auto pathElem = std::make_shared<sdf::Element>();
  pluginElem->InsertElement(pathElem);
  pathElem->SetName("path");
  pathElem->AddValue("string", logDest, "1");

  pluginInfo.SetSdf(pluginElem);

  // Fill server configuration
  ServerConfig serverConfig;
  serverConfig.AddPlugin(pluginInfo);
  serverConfig.SetSdfFile(common::joinPaths(
      std::string(PROJECT_SOURCE_PATH), "test", "worlds", "falling.sdf"));

  // Start server
  Server server(serverConfig);

  // Run for a few seconds to record different states
  server.Run(true, 3000, false);

  // Verify file is created
  EXPECT_TRUE(common::exists(common::joinPaths(logDest, "state.tlog")));

  common::removeAll(logDest);
}

/////////////////////////////////////////////////
// This test checks that state is played back correctly
TEST_F(LogSystemTest, StatePlayback)
{
  auto logPath = common::joinPaths(std::string(PROJECT_SOURCE_PATH), "test",
      "media", "log_falling_serialized_state");

  // Configure server to load falling world with LogRecorder plugin
  ServerConfig::PluginInfo pluginInfo;
  pluginInfo.SetEntityName("default");
  pluginInfo.SetEntityType("world");
  pluginInfo.SetFilename("libignition-gazebo-log-system.so");
  pluginInfo.SetName("ignition::gazebo::systems::LogPlayback");

  auto pluginElem = std::make_shared<sdf::Element>();
  pluginElem->SetName("plugin");
  pluginElem->AddAttribute("name", "string",
      "ignition::gazebo::systems::LogPlayback", true);
  pluginElem->AddAttribute("filename", "string",
      "libignition-gazebo-log-system.so", true);

  auto pathElem = std::make_shared<sdf::Element>();
  pluginElem->InsertElement(pathElem);
  pathElem->SetName("path");
  pathElem->AddValue("string", logPath, "1");

  pluginInfo.SetSdf(pluginElem);

  // Fill server configuration
  ServerConfig serverConfig;
  serverConfig.AddPlugin(pluginInfo);
  serverConfig.SetSdfFile(common::joinPaths(
      std::string(PROJECT_SOURCE_PATH), "test", "worlds", "blank.sdf"));

  // Start server
  Server server(serverConfig);

  // TODO(louise) Instead of subscribing to pose messages, do a simple check of
  // pose components to see that the sphere is falling

  // Callback function for entities played back
//  auto msgCb = [&](const msgs::Pose_V &_msg) -> void
//  {
//    server.SetPaused(true);
//
//    // Look for recorded topics with current sim time
//    std::chrono::nanoseconds end =
//      std::chrono::seconds(_msg.header().stamp().sec()) +
//      std::chrono::nanoseconds(_msg.header().stamp().nsec());
//    auto timeRange = transport::log::QualifiedTimeRange(begin, end);
//
//    // Access selective recorded messages in .tlog file
//    transport::log::TopicList topicList(logPoseTopic);
//    transport::log::Batch batch = log.QueryMessages(topicList);
//    transport::log::MsgIter iter = batch.begin();
//    // If no messages
//    if (iter == batch.end())
//      return;
//
//    // Skip until last timestamp in range, closest to current time
//    msgs::Pose_V posevMsg;
//    for (; iter != batch.end(); ++iter)
//    {
//      // Convert recorded binary bytes in string into a ign-msgs msg
//      posevMsg.ParseFromString(iter->Data());
//    }
//
//    // Maps entity to pose recorded
//    // Key: entity. Value: pose
//    std::map <Entity, msgs::Pose> idToPose;
//    // Loop through all recorded poses, update map
//    for (int i = 0; i < posevMsg.pose_size(); ++i)
//    {
//      msgs::Pose pose = posevMsg.pose(i);
//      idToPose.insert_or_assign(pose.id(), pose);
//    }
//
//    // Loop through all played poses and compare to recorded ones
//    //std::cerr << _msg.pose_size() << std::endl;
//    for (int i = 0; i < _msg.pose_size(); ++i)
//    {
//      math::Pose3d posePlayed = msgs::Convert(_msg.pose(i));
//      math::Pose3d poseRecorded = msgs::Convert(idToPose.at(_msg.pose(i).id()));
//
//      /*
//      double dist = sqrt(pow(posePlayed.Pos().X() - poseRecorded.Pos().X(), 2) +
//        pow(posePlayed.Pos().Y() - poseRecorded.Pos().Y(), 2) +
//        pow(posePlayed.Pos().Z() - poseRecorded.Pos().Z(), 2));
//
//      if (dist >= 0.3)
//      {
//        std::cerr << _msg.pose(i).name() << std::endl;
//        std::cerr << posePlayed << std::endl;
//        std::cerr << poseRecorded << std::endl;
//      }
//
//      // Allow small tolerance to difference between recorded and played back
//      EXPECT_LT(dist, 0.3);
//      */
//
//
//      auto diff = posePlayed - poseRecorded;
//      std::cerr << diff << std::endl;
//
//      EXPECT_EQ(diff, math::Pose3d());
//
//      /*
//      EXPECT_NEAR(abs(diff.Pos().X()), 0.1);
//      EXPECT_NEAR(abs(diff.Pos().Y()), 0.1);
//      EXPECT_NEAR(abs(diff.Pos().Z()), 0.1);
//
//      EXPECT_NEAR(abs(diff.Rot().W()), 0.1);
//      EXPECT_NEAR(abs(diff.Rot().X()), 0.1);
//      EXPECT_NEAR(abs(diff.Rot().Y()), 0.1);
//      EXPECT_NEAR(abs(diff.Rot().Z()), 0.1);
//      */
//    }
//
//    // Update begin time range for next time step
//    begin = std::chrono::nanoseconds(end);
//
//    server.SetPaused(false);
//  };

  // Run for a few seconds to play back different poses
  server.Run(true, 3000, false);
}
