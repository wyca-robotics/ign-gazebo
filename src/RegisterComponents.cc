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

#include <ignition/gazebo/components/Factory.hh>

#include <ignition/gazebo/components/Altimeter.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/Camera.hh>
#include <ignition/gazebo/components/CanonicalLink.hh>
#include <ignition/gazebo/components/ChildLinkName.hh>
#include <ignition/gazebo/components/Collision.hh>
#include <ignition/gazebo/components/Component.hh>
#include <ignition/gazebo/components/ContactSensorData.hh>
#include <ignition/gazebo/components/ContactSensor.hh>
#include <ignition/gazebo/components/DepthCamera.hh>
#include <ignition/gazebo/components/Geometry.hh>
#include <ignition/gazebo/components/GpuLidar.hh>
#include <ignition/gazebo/components/Gravity.hh>
#include <ignition/gazebo/components/Imu.hh>
#include <ignition/gazebo/components/Inertial.hh>
#include <ignition/gazebo/components/JointAxis.hh>
#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/JointType.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/LevelBuffer.hh>
#include <ignition/gazebo/components/LevelEntityNames.hh>
#include <ignition/gazebo/components/Level.hh>
#include <ignition/gazebo/components/Light.hh>
#include <ignition/gazebo/components/LinearAcceleration.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/LogicalCamera.hh>
#include <ignition/gazebo/components/MagneticField.hh>
#include <ignition/gazebo/components/Magnetometer.hh>
#include <ignition/gazebo/components/Material.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/ParentLinkName.hh>
#include <ignition/gazebo/components/Performer.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Sensor.hh>
#include <ignition/gazebo/components/Static.hh>
#include <ignition/gazebo/components/ThreadPitch.hh>
#include <ignition/gazebo/components/Visual.hh>
#include <ignition/gazebo/components/World.hh>

using namespace ignition;
using namespace gazebo;
using namespace components;

IGN_GAZEBO_REGISTER_COMPONENT(
  "ign_gazebo_components.Altimeter", Altimeter)
IGN_GAZEBO_REGISTER_COMPONENT(
  "ign_gazebo_components.AngularVelocity", AngularVelocity)
IGN_GAZEBO_REGISTER_COMPONENT(
  "ign_gazebo_components.WorldAngularVelocity", WorldAngularVelocity)
IGN_GAZEBO_REGISTER_COMPONENT(
  "ign_gazebo_components.Camera", Camera)
IGN_GAZEBO_REGISTER_COMPONENT(
  "ign_gazebo_components.CanonicalLink", CanonicalLink)
IGN_GAZEBO_REGISTER_COMPONENT(
  "ign_gazebo_components.ChildLinkName", ChildLinkName)
IGN_GAZEBO_REGISTER_COMPONENT(
  "ign_gazebo_components.Collision", Collision)
IGN_GAZEBO_REGISTER_COMPONENT(
  "ign_gazebo_components.ContactSensor", ContactSensor)
IGN_GAZEBO_REGISTER_COMPONENT(
  "ign_gazebo_components.ContactSensorData", ContactSensorData)
IGN_GAZEBO_REGISTER_COMPONENT(
  "ign_gazebo_components.DepthCamera", DepthCamera)
IGN_GAZEBO_REGISTER_COMPONENT(
  "ign_gazebo_components.Geometry", Geometry)
IGN_GAZEBO_REGISTER_COMPONENT(
  "ign_gazebo_components.GpuLidar", GpuLidar)
IGN_GAZEBO_REGISTER_COMPONENT(
  "ign_gazebo_components.Gravity", Gravity)
IGN_GAZEBO_REGISTER_COMPONENT(
  "ign_gazebo_components.Imu", Imu)
IGN_GAZEBO_REGISTER_COMPONENT(
  "ign_gazebo_components.Inertial", Inertial)
IGN_GAZEBO_REGISTER_COMPONENT(
  "ign_gazebo_components.Joint", Joint)
IGN_GAZEBO_REGISTER_COMPONENT(
  "ign_gazebo_components.JointAxis", JointAxis)
IGN_GAZEBO_REGISTER_COMPONENT(
  "ign_gazebo_components.JointAxis2", JointAxis2)
IGN_GAZEBO_REGISTER_COMPONENT(
  "ign_gazebo_components.JointType", JointType)
IGN_GAZEBO_REGISTER_COMPONENT(
  "ign_gazebo_components.JointVelocity", JointVelocity)
IGN_GAZEBO_REGISTER_COMPONENT(
  "ign_gazebo_components.JointVelocity2", JointVelocity2)
IGN_GAZEBO_REGISTER_COMPONENT(
  "ign_gazebo_components.Level", Level)
IGN_GAZEBO_REGISTER_COMPONENT(
  "ign_gazebo_components.DefaultLevel", DefaultLevel)
IGN_GAZEBO_REGISTER_COMPONENT(
  "ign_gazebo_components.LevelBuffer", LevelBuffer)
IGN_GAZEBO_REGISTER_COMPONENT(
  "ign_gazebo_components.LevelEntityNames", LevelEntityNames)
IGN_GAZEBO_REGISTER_COMPONENT(
  "ign_gazebo_components.Light", Light)
IGN_GAZEBO_REGISTER_COMPONENT(
  "ign_gazebo_components.LinearAcceleration", LinearAcceleration)
IGN_GAZEBO_REGISTER_COMPONENT(
  "ign_gazebo_components.WorldLinearAcceleration", WorldLinearAcceleration)
IGN_GAZEBO_REGISTER_COMPONENT(
  "ign_gazebo_components.LinearVelocity", LinearVelocity)
IGN_GAZEBO_REGISTER_COMPONENT(
  "ign_gazebo_components.WorldLinearVelocity", WorldLinearVelocity)
IGN_GAZEBO_REGISTER_COMPONENT(
  "ign_gazebo_components.Link", Link)
IGN_GAZEBO_REGISTER_COMPONENT(
  "ign_gazebo_components.LogicalCamera", LogicalCamera)
IGN_GAZEBO_REGISTER_COMPONENT(
  "ign_gazebo_components.MagneticField", MagneticField)
IGN_GAZEBO_REGISTER_COMPONENT(
  "ign_gazebo_components.Magnetometer", Magnetometer)
IGN_GAZEBO_REGISTER_COMPONENT(
  "ign_gazebo_components.Material", Material)
IGN_GAZEBO_REGISTER_COMPONENT(
  "ign_gazebo_components.Model", Model)
IGN_GAZEBO_REGISTER_COMPONENT(
  "ign_gazebo_components.Name", Name)
IGN_GAZEBO_REGISTER_COMPONENT(
  "ign_gazebo_components.ParentEntity", ParentEntity)
IGN_GAZEBO_REGISTER_COMPONENT(
  "ign_gazebo_components.ParentLinkName", ParentLinkName)
IGN_GAZEBO_REGISTER_COMPONENT(
  "ign_gazebo_components.Performer", Performer)
IGN_GAZEBO_REGISTER_COMPONENT(
  "ign_gazebo_components.Pose", Pose)
IGN_GAZEBO_REGISTER_COMPONENT(
  "ign_gazebo_components.WorldPose", WorldPose)
IGN_GAZEBO_REGISTER_COMPONENT(
  "ign_gazebo_components.Sensor", Sensor)
IGN_GAZEBO_REGISTER_COMPONENT(
  "ign_gazebo_components.Static", Static)
IGN_GAZEBO_REGISTER_COMPONENT(
  "ign_gazebo_components.ThreadPitch", ThreadPitch)
IGN_GAZEBO_REGISTER_COMPONENT(
  "ign_gazebo_components.Visual", Visual)
IGN_GAZEBO_REGISTER_COMPONENT(
  "ign_gazebo_components.World", World)
