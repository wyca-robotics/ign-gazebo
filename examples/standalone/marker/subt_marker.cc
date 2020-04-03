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

std::vector<std::string> teams =
{
  "Coordinated_Robotics",
  "MTRI",
  "sodium-24-robotics",
  "CYNET-ai",
  "robotika",
  "Sophisticated_Engineering_UG",
  "Kankanwadi",
  "Scientific_Systems_Company"
};

// PoseData contains all pose data for a single pose message callback.
class PoseData
{
  public: std::map<std::string, std::vector<ignition::math::Pose3d>> poses;
};

class Processor
{
  public: Processor(const std::string &_path)
  {
    ignition::transport::NodeOptions opts;
    opts.SetPartition("PATH_PLAYBACK");
    this->markerNode = new ignition::transport::Node(opts);
    this->GetArtifactPoses();

    std::unique_lock<std::mutex> lock(this->mutex);
    std::thread playbackThread(std::bind(&Processor::Playback, this, _path));

    this->cv.wait(lock);
    ignition::transport::Node node;

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
          if (!node.Subscribe(topic, &Processor::Cb, this))
          {
            std::cerr << "Error subscribing to topic ["
              << topic << "]" << std::endl;
            return;
          }
          subscribed = true;
          break;
        }
      }
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    playbackThread.join();

    this->DisplayPoses();
    this->DisplayArtifacts();

    std::ifstream in(_path + "/subt.log");
    std::string line, prevLine;
    std::string key = "reported_pos[";
    int lineNum = 0;
    while (std::getline(in, line))
    {
      if (line.find("modified_score ") != std::string::npos)
      {
        ignition::math::Vector3d reportedPos;
        int idx = prevLine.find(key);
        if (idx != std::string::npos)
        {
          int endIdx = prevLine.rfind("]");
          std::string pos = prevLine.substr(idx + key.size(),
              endIdx - (idx + key.size()));

          std::stringstream stream;
          stream << pos;
          stream >> reportedPos;

          if (line.find("modified_score 1") != std::string::npos)
          {
            this->SpawnMarker(colors[1], reportedPos,
                ignition::msgs::Marker::SPHERE,
                ignition::math::Vector3d(8, 8, 8));
          }
          else
          {
            this->SpawnMarker(colors[0], reportedPos,
                ignition::msgs::Marker::BOX,
                ignition::math::Vector3d(8, 8, 8));
          }
        }
      }

      prevLine = line;
      lineNum++;
    }
  }

  public: ~Processor()
  {
    ignition::msgs::Marker markerMsg;
    markerMsg.set_ns("default");
    markerMsg.set_action(ignition::msgs::Marker::DELETE_ALL);
    markerNode->Request("/marker", markerMsg);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    delete this->markerNode;
    this->markerNode = nullptr;
  }

  //////////////////////////////////////////////////
  // Playback a log file.
  public: void Playback(std::string _path)
  {
    std::unique_lock<std::mutex> lock(this->mutex);
    ignition::transport::log::Playback player(_path + "/state.tlog");
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

  /////////////////////////////////////////////////
  public: void GetArtifactPoses()
  {
    bool subscribed = false;
    for (int i = 0; i < 5 && !subscribed; ++i)
    {
      std::vector<std::string> topics;
      this->markerNode->TopicList(topics);

      for (auto const &topic : topics)
      {
        if (topic.find("/pose/info") != std::string::npos)
        {
          this->markerNode->Subscribe(topic, &Processor::ArtifactCb, this);
        }
      }
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }

  /////////////////////////////////////////////////
  public: void ArtifactCb(const ignition::msgs::Pose_V &_msg)
  {
    PoseData data;

    // Process each pose in the message.
    for (int i = 0; i < _msg.pose_size(); ++i)
    {
      // Only consider artifacts.
      std::string name = _msg.pose(i).name();
      if (name.find("rescue") == 0 ||
          name.find("backpack") == 0 ||
          name.find("vent") == 0 ||
          name.find("gas") == 0 ||
          name.find("drill") == 0 ||
          name.find("extinguisher") == 0 ||
          name.find("phone") == 0)
      {
        ignition::math::Pose3d pose = ignition::msgs::Convert(_msg.pose(i));
        this->artifacts[name] = pose;
      }
    }
  }

  //////////////////////////////////////////////////
  // Display the poses.
  public: void DisplayPoses()
  {
    for (const PoseData &d : this->poses)
    {
      for (std::map<std::string,
          std::vector<ignition::math::Pose3d>>::const_iterator
          iter = d.poses.begin(); iter != d.poses.end(); iter++)
      {
        for (const ignition::math::Pose3d &p : iter->second)
        {
          this->SpawnMarker(this->robots[iter->first],
              p.Pos() + ignition::math::Vector3d(0, 0, 0.5),
              ignition::msgs::Marker::SPHERE,
              ignition::math::Vector3d(1, 1, 1));
        }
      }
      //std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  /////////////////////////////////////////////////
  public: void DisplayArtifacts()
  {
    for (const auto &[name, pose] : this->artifacts)
    {
      this->SpawnMarker(colors[2], pose.Pos(), ignition::msgs::Marker::SPHERE,
          ignition::math::Vector3d(8, 8, 8));
    }
  }

  //////////////////////////////////////////////////
  // Helper function that spawns a visual marker.
  public: void SpawnMarker(ignition::math::Color &_color,
    const ignition::math::Vector3d &_pos,
    ignition::msgs::Marker::Type _type,
    const ignition::math::Vector3d &_scale)
  {
    // Create the marker message
    ignition::msgs::Marker markerMsg;
    ignition::msgs::Material matMsg;
    markerMsg.set_ns("default");
    markerMsg.set_id(markerId++);
    markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
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

    ignition::msgs::Set(markerMsg.mutable_scale(), _scale);

    // The rest of this function adds different shapes and/or modifies shapes.
    // Read the terminal statements to figure out what each node.Request
    // call accomplishes.
    ignition::msgs::Set(markerMsg.mutable_pose(),
        ignition::math::Pose3d(_pos.X(), _pos.Y(), _pos.Z(), 0, 0, 0));
    this->markerNode->Request("/marker", markerMsg);
  }

  //////////////////////////////////////////////////
  // This callback is triggered on every pose message in the log file.
  public: void Cb(const ignition::msgs::Pose_V &_msg)
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

      if (this->robots.find(name) == this->robots.end())
      {
        this->robots[name] = this->colors[this->robots.size()+3];
        this->prevPose[name] = pose;
      }

      // Filter poses.
      if (this->prevPose[name].Pos().Distance(pose.Pos()) > 1.0)
      {
        data.poses[name].push_back(pose);
        this->prevPose[name] = pose;
      }
    }

    // Store data.
    if (!data.poses.empty())
      this->poses.push_back(data);
  }

  // Mapping of robot name to color
  private: std::map<std::string, ignition::math::Color> robots;

  // Last pose of a robot. This is used to reduce the number of markers.
  private: std::map<std::string, ignition::math::Pose3d> prevPose;

  private:std::map<std::string, ignition::math::Pose3d> artifacts;

  private: int markerId = 0;

  // Node that will display the visual markers.
  private: ignition::transport::Node *markerNode;

  private: std::mutex mutex;
  private: std::condition_variable cv;

  // The colors used to represent each robot.
  private: std::vector<ignition::math::Color> colors =
  {
    // Bad Report locations
    {1.0, 0.0, 0.0, 0.1},

    // Good Artifact locations
    {0.0, 1.0, 0.0, 0.1},

    // Artifact locations.
    {0.0, 1.0, 1.0, 0.1},

    // Robot colors
    {153/255.0,0,1},
    {173/255.0,51/255.0,1},
    {194/255.0,102/255.0,1},
    {214/255.0,153/255.0,1},
    {235/255.0,204/255.0,1}
  };

  // All of the pose data.
  private: std::vector<PoseData> poses;
};

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  std::string path = _argv[1];
  std::string index = _argv[2];

  for (const std::string &team : teams)
  {
    std::string teamPath = path + "/" + team + "/" + index;

    std::ifstream test((teamPath + "/score.yml").c_str());
    if (test.good())
    {
      std::cout << "# Processing: " << teamPath << std::endl;

      Processor p(teamPath);

      char pass;
      std::cout << "Next\n";
      std::cin >> pass;
    }
    else
      std::cout << "Skipping[" << teamPath << "]\n";
  }
  return 0;
/*
  std::unique_lock<std::mutex> lock(mutex);
  std::thread playbackThread(playback, path);

  cv.wait(lock);
  ignition::transport::Node node;

  ignition::transport::NodeOptions opts;
  opts.SetPartition("PATH_PLAYBACK");
  markerNode = new ignition::transport::Node(opts);

  getArtifactPoses();

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


  displayPoses();
  displayArtifacts();

  std::ifstream in(path + "/subt.log");
  std::string line;
  std::string key = "reported_pos[";
  while (std::getline(in, line))
  {
    int idx = line.find(key);
    if (idx != std::string::npos)
    {
      int endIdx = line.rfind("]");
      std::string pos = line.substr(idx + key.size(),
          endIdx - (idx + key.size()));

      ignition::math::Vector3d reportedPos;
      std::stringstream stream;
      stream << pos;
      stream >> reportedPos;
      spawnMarker(colors[0], reportedPos, ignition::msgs::Marker::SPHERE,
          ignition::math::Vector3d(8, 8, 8));
    }
  }

  ignition::common::Time::Sleep(ignition::common::Time(4));

  ignition::msgs::Marker markerMsg;
  markerMsg.set_ns("default");
  markerMsg.set_action(ignition::msgs::Marker::DELETE_ALL);
  markerNode->Request("/marker", markerMsg);
  */
}
