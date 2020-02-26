/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
#include <mutex>
#include <map>
#include <iostream>
#include <condition_variable>

#include <ignition/transport.hh>
#include <ignition/math.hh>
#include <ignition/msgs.hh>
#include <ignition/common/Time.hh>
#include <ignition/transport/log/Playback.hh>

// Usage:
//
// 1. Run the subt world using the `path_tracer.ign` launch file along with
// an IGN_PARTITION name of PATH_TRACER. For example
//
//     $ IGN_PARTITION=PATH_TRACER ign launch -v 4 path_tracer.ign worldName:=urban_circuit_01
//
// 2. Run the program by passing in the location of a log file. For example:
//
//     $ ./subt_marker /data/logs/state.tlog

// Node that will display the visual markers.
ignition::transport::Node *markerNode;
int markerId = 0;
std::mutex mutex;
std::condition_variable cv;

// The colors used to represent each robot.
std::vector<ignition::math::Color> colors =
{
  {1.0, 0.0, 0.0},
  {0.0, 1.0, 0.0},
  {1.0, 0.0, 1.0},
  {0.0, 1.0, 1.0},
  {1.0, 1.0, 0.0},
  {0.0, 0.0, 1.0},
  {1.0, 1.0, 1.0}
};

// Mapping of robot name to color
std::map<std::string, ignition::math::Color> robots;

// Last pose of a robot. This is used to reduce the number of markers.
std::map<std::string, ignition::math::Pose3d> prevPose;

// PoseData contains all pose data for a single pose message callback.
class PoseData
{
  public: std::map<std::string, std::vector<ignition::math::Pose3d>> poses;
};

// All of the pose data.
std::vector<PoseData> poses;

//////////////////////////////////////////////////
// Helper function that spawns a visual marker.
void spawnMarker(ignition::math::Color &_color,
    const ignition::math::Vector3d &_pos,
    ignition::msgs::Marker::Type _type)
{
  // Create the marker message
  ignition::msgs::Marker markerMsg;
  ignition::msgs::Material matMsg;
  markerMsg.set_ns("default");
  markerMsg.set_id(markerId++);
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  //markerMsg.set_type(ignition::msgs::Marker::SPHERE);
  markerMsg.set_type(_type);
  markerMsg.set_visibility(ignition::msgs::Marker::GUI);

  // Set color
  markerMsg.mutable_material()->mutable_ambient()->set_r(_color.R());
  markerMsg.mutable_material()->mutable_ambient()->set_g(_color.G());
  markerMsg.mutable_material()->mutable_ambient()->set_b(_color.B());
  markerMsg.mutable_material()->mutable_ambient()->set_a(_color.A());
  markerMsg.mutable_material()->mutable_diffuse()->set_r(_color.R());
  markerMsg.mutable_material()->mutable_diffuse()->set_g(_color.G());
  markerMsg.mutable_material()->mutable_diffuse()->set_b(_color.B());
  markerMsg.mutable_material()->mutable_diffuse()->set_a(_color.A());
  markerMsg.mutable_material()->mutable_emissive()->set_r(_color.R());
  markerMsg.mutable_material()->mutable_emissive()->set_g(_color.G());
  markerMsg.mutable_material()->mutable_emissive()->set_b(_color.B());
  markerMsg.mutable_material()->mutable_emissive()->set_a(_color.A());

  ignition::msgs::Set(markerMsg.mutable_scale(),
      ignition::math::Vector3d(0.05, 0.05, 0.05));

  // The rest of this function adds different shapes and/or modifies shapes.
  // Read the terminal statements to figure out what each node.Request
  // call accomplishes.
  ignition::msgs::Set(markerMsg.mutable_pose(),
      ignition::math::Pose3d(_pos.X(), _pos.Y(), _pos.Z(), 0, 0, 0));
  markerNode->Request("/marker", markerMsg);
}

//////////////////////////////////////////////////
// This callback is triggered on every pose message in the log file.
void cb(const ignition::msgs::Pose_V &_msg)
{
  PoseData data;

  // Process each pose in the message.
  for (int i = 0; i < _msg.pose_size(); ++i)
  {
    // Only conder robots.
    std::string name = _msg.pose(i).name();
    if (name.find("_wheel") != std::string::npos ||
        name.find("rotor_") != std::string::npos || name == "base_link") {
      continue;
    }

    ignition::math::Pose3d pose = ignition::msgs::Convert(_msg.pose(i));

    if (robots.find(name) == robots.end())
    {
      robots[name] = colors[robots.size()];
      prevPose[name] = pose;
    }

    // Filter poses.
    if (prevPose[name].Pos().Distance(pose.Pos()) > 1.0)
    {
      data.poses[name].push_back(pose);
      prevPose[name] = pose;
    }
  }

  // Store data.
  if (!data.poses.empty())
    poses.push_back(data);
}

//////////////////////////////////////////////////
// Playback a log file.
void playback(char **_argv)
{
  std::unique_lock<std::mutex> lock(mutex);
  ignition::transport::log::Playback player(_argv[1]);
  bool valid = false;
  ignition::transport::log::PlaybackHandlePtr handle;

  // Playback all topics
  const int64_t addTopicResult = player.AddTopic(std::regex(".*"));
  if (addTopicResult == 0)
    std::cerr << "No topics to play back\n";
  else if (addTopicResult < 0)
    std::cerr << "Failed to advertise topics: " << addTopicResult << "\n";
  else
  {
    // Begin playback
    handle = player.Start(std::chrono::seconds(5), false);
    if (!handle)
    {
      std::cerr << "Failed to start playback\n";
      return;
    }
    valid = true;
  }
  cv.notify_all();
  lock.unlock();

  // Wait until the player stops on its own
  if (valid)
  {
    std::cerr << "Playing all messages in the log file\n";
    handle->WaitUntilFinished();
  }
}


//////////////////////////////////////////////////
// Display the poses.
void displayPoses()
{
  for (const PoseData &d : poses)
  {
    for (std::map<std::string,
        std::vector<ignition::math::Pose3d>>::const_iterator
        iter = d.poses.begin(); iter != d.poses.end(); iter++)
    {
      for (const ignition::math::Pose3d &p : iter->second)
      {
        spawnMarker(robots[iter->first],
            p.Pos() + ignition::math::Vector3d(0, 0, 0.5),
            ignition::msgs::Marker::SPHERE);
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  /*
  std::unique_lock<std::mutex> lock(mutex);
  std::thread playbackThread(playback, _argv);

  cv.wait(lock);
  */
  ignition::transport::Node node;

  ignition::transport::NodeOptions opts;
  opts.SetPartition("PATH_PLAYBACK");
  markerNode = new ignition::transport::Node(opts);
/*
  bool subscribed = false;
  for (int i = 0; i < 5 && !subscribed; ++i)
  {
    std::vector<std::string> topics;
    node.TopicList(topics);

    // Subscribe to the first /dynamic_pose/info topic
    for (auto const &topic : topics)
    {
      if (topic.find("/dynamic_pose/info") != std::string::npos)
      {
        // Subscribe to a topic by registering a callback.
        if (!node.Subscribe(topic, cb))
        {
          std::cerr << "Error subscribing to topic ["
            << topic << "]" << std::endl;
          return -1;
        }
        subscribed = true;
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  playbackThread.join();
  */

  // displayPoses();
  spawnMarker(colors[2],
      ignition::math::Vector3d( -89.05, -1.138369, -7.08),
      ignition::msgs::Marker::SPHERE);

  spawnMarker(colors[1],
      ignition::math::Vector3d(-88.9868, -1.18523, -7.133),
      ignition::msgs::Marker::BOX);
}
