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
#ifndef IGNITION_GAZEBO_SYSTEMS_IMU_HH_
#define IGNITION_GAZEBO_SYSTEMS_IMU_HH_

#include <memory>
#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>
#include <ignition/gazebo/System.hh>

namespace ignition
{
namespace gazebo
{
namespace systems
{
  // Inline bracket to help doxygen filtering.
  inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
  // Forward declarations.
  class ImuPrivate;
  class ImuSensor;

  /// \class Imu Imu.hh ignition/gazebo/systems/Imu.hh
  /// \brief An imu sensor that reports vertical position and velocity
  /// readings over ign transport
  class IGNITION_GAZEBO_VISIBLE Imu:
    public System,
    public ISystemPreUpdate,
    public ISystemPostUpdate
  {
    /// \brief Constructor
    public: explicit Imu();

    /// \brief Destructor
    public: virtual ~Imu();

    /// Documentation inherited
    public: void PreUpdate(const UpdateInfo &_info,
                           EntityComponentManager &_ecm) override final;

    /// Documentation inherited
    public: void PostUpdate(const UpdateInfo &_info,
                            const EntityComponentManager &_ecm) override final;

    // /// \brief Returns the angular velocity in the IMU sensor local frame.
    // /// \param[in] _noiseFree True if the returned measurement should
    // /// not use noise.
    // /// \return Angular velocity.
    // public: ignition::math::Vector3d AngularVelocity(
    //             const bool _noiseFree = false) const;
    //
    // /// \brief Returns the imu linear acceleration in the IMU sensor
    // /// local frame
    // /// \param[in] _noiseFree True if the returned measurement should
    // /// not use noise.
    // /// \return Linear acceleration.
    // public: ignition::math::Vector3d LinearAcceleration(
    //             const bool _noiseFree = false) const;
    //
    // /// \brief get orientation of the IMU relative to a reference pose
    // /// Initially, the reference pose is the boot up pose of the IMU,
    // /// but user can call either SetReferencePose to define current
    // /// pose as the reference frame, or call SetWorldToReferencePose
    // /// to define transform from world frame to reference frame.
    // /// \return returns the orientation quaternion of the IMU relative to
    // /// the imu reference pose.
    // public: ignition::math::Quaterniond Orientation() const;
    //
    // /// \brief Sets the current IMU pose as the reference NED pose,
    // /// i.e. X axis of the IMU is aligned with North,
    // ///      Y axis of the IMU is aligned with East,
    // ///      Z axis of the IMU is aligned with Downward (gravity) direction.
    // public: void SetReferencePose();
    //
    // // Documentation inherited.
    // public: virtual bool IsActive() const;

    /// \brief Sets the rotation transform from world frame to IMU's
    /// reference frame.
    /// For example, if this IMU works with respect to NED frame, then
    /// call this function with the transform that transforms world frame
    /// to NED frame. Subsequently, ImuSensor::Orientation will return
    /// identity transform if the IMU is aligned with the NED frame.
    /// \param _orientation rotation from world frame to imu reference frame.
    public: void SetWorldToReferenceOrientation(
      const ignition::math::Quaterniond &_orientation);

    /// \brief Private data pointer.
    private: std::unique_ptr<ImuPrivate> dataPtr;
  };
  }
}
}
}
#endif
