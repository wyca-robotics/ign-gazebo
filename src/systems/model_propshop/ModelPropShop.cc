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

#include <array>

#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include <ignition/msgs/entity_factory.pb.h>
#include <ignition/msgs/entity.pb.h>

#include "ignition/gazebo/components/Camera.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Sensor.hh"
#include "ignition/gazebo/components/World.hh"

#include <sdf/Element.hh>

#include "ModelPropShop.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::ModelPropShopPrivate
{
  public: Entity cameraEntity;

  public: std::string objectName;

  public: std::string outputFolder;

  public: ignition::transport::Node transportNode;

  public: const int timeout = 5000;

  public: std::string poseService;

  public: std::string removeService;

  public: double cameraFrameInterval;

  public: int currentView = 0;

  public: double startingTime;

  public: bool init = false;

  public: bool done = false;

  public: std::array<ignition::math::Pose3d, 5> views;

  public: ModelPropShopPrivate();
};

//////////////////////////////////////////////////
ModelPropShopPrivate::ModelPropShopPrivate()
{
  ignition::math::Pose3d pose; 
  // Perspective
  // TODO bounding box might help here?
  pose.Pos().Set(0,0,0);
  pose.Rot().Euler(IGN_DTOR(0), IGN_DTOR(30), IGN_DTOR(0));
  views[0] = pose;
  // Top
  pose.Rot().Euler(0, IGN_DTOR(90), 0);
  views[1] = pose;
  // Front
  pose.Rot().Euler(0, 0, 0);
  views[2] = pose;
  // Side
  pose.Rot().Euler(0, 0, IGN_DTOR(-90));
  views[3] = pose;
  // Back
  pose.Rot().Euler(0, 0, IGN_DTOR(180));
  views[4] = pose;
}

//////////////////////////////////////////////////
ModelPropShop::ModelPropShop()
    : System(), dataPtr(std::make_unique<ModelPropShopPrivate>())
{
}

//////////////////////////////////////////////////
void ModelPropShop::Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm, EventManager &)
{
  this->dataPtr->cameraEntity = _ecm.ParentEntity(_entity);
  // Make sure attached model has a camera component
  if (!_ecm.EntityHasComponentType(_entity, components::Camera().TypeId()))
  {
    ignerr << "No camera component found" << std::endl;
  }
  this->dataPtr->cameraFrameInterval = 
    1.0 / _ecm.Component<components::Camera>(_entity)->Data().UpdateRate();
  // Spawn the model
  ignition::msgs::EntityFactory req;
  std::string model_uri = _sdf->Get<std::string>("model_uri");
  size_t sep = model_uri.find_last_of("\\/");
  // Get the model name (useful for fuel URLs)
  if (sep != std::string::npos)
    this->dataPtr->objectName = model_uri.substr(sep + 1, model_uri.size() - 1);
  else
    this->dataPtr->objectName = model_uri;
  req.set_sdf_filename(model_uri);
  ignition::math::Pose3d pose {0,0,0,0,0,0};
  ignition::msgs::Set(req.mutable_pose(), pose);

  Entity worldEntity = _ecm.EntityByComponents(components::World());
  std::string worldName = _ecm.Component<components::Name>(worldEntity)->Data();

  std::string spawnService {"/world/" + worldName + "/create"};
  
  this->dataPtr->poseService = "/world/" + worldName + "/set_pose";
  this->dataPtr->removeService = "/world/" + worldName + "/remove";

  // Spawn the model
  ignition::msgs::Boolean rep;
  bool result;
  bool executed = this->dataPtr->transportNode.Request(spawnService, req, this->dataPtr->timeout, rep, result);
  if (executed && result && rep.data())
    ignmsg << "Entity creation successful" << std::endl;
  else
    ignerr << "Entity creation failed" << std::endl;
}

//////////////////////////////////////////////////
void ModelPropShop::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
  if (_info.paused || this->dataPtr->done == true)
    return;
  // TODO Get bounding box for object, unimplemented for now
  // TODO center in z based on bounding box height
  double curSimTime = std::chrono::duration_cast<std::chrono::nanoseconds>
    (_info.simTime).count() * 1e-9;
  if (this->dataPtr->init == false)
  {
    this->dataPtr->startingTime = curSimTime;
    this->dataPtr->init = true;
  }

  double dt = curSimTime - this->dataPtr->startingTime;

  // Represents the time at which a new frame will be triggered, deadline halfway between frames
  double nextFrameDeadline = this->dataPtr->cameraFrameInterval * (this->dataPtr->currentView - 0.5);
  // TODO arbitrary value?
  if (dt > nextFrameDeadline)
  {
    ignition::math::Pose3d cameraPose = this->dataPtr->views[this->dataPtr->currentView];
    ignition::msgs::Pose req {ignition::msgs::Convert(cameraPose)};
    req.set_name(this->dataPtr->objectName);
    ignition::msgs::Boolean rep;
    bool result;
    bool executed = this->dataPtr->transportNode.Request(this->dataPtr->poseService,
        req, this->dataPtr->timeout, rep, result);
    if (executed && result && rep.data())
      ignmsg << "Pose service call successful" << std::endl;
    else
      ignerr << "Pose service call failed" << std::endl;
    if (this->dataPtr->currentView < this->dataPtr->views.size() - 1)
      ++this->dataPtr->currentView;
    else
    {
      // Remove the camera
      ignition::msgs::Entity req;
      // TODO remove hardcoded
      req.set_type(ignition::msgs::Entity::MODEL);
      req.set_name("camera");
      ignition::msgs::Boolean rep;
      bool result;
      bool executed = this->dataPtr->transportNode.Request(this->dataPtr->removeService,
          req, this->dataPtr->timeout, rep, result);
      if (executed && result && rep.data())
        this->dataPtr->done = true;
    }
    // Will segfault past 5, which is a convenient way to close the simulation
  }
}

IGNITION_ADD_PLUGIN(ModelPropShop,
                    ignition::gazebo::System,
                    ModelPropShop::ISystemConfigure,
                    ModelPropShop::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(ModelPropShop, "ignition::gazebo::systems::ModelPropShop")
