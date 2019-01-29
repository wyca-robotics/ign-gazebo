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
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/Events.hh"

#include "ignition/gazebo/components/Altimeter.hh"
#include "ignition/gazebo/components/Camera.hh"
#include "ignition/gazebo/components/CanonicalLink.hh"
#include "ignition/gazebo/components/Collision.hh"
#include "ignition/gazebo/components/ChildLinkName.hh"
#include "ignition/gazebo/components/Geometry.hh"
#include "ignition/gazebo/components/Inertial.hh"
#include "ignition/gazebo/components/Joint.hh"
#include "ignition/gazebo/components/JointAxis.hh"
#include "ignition/gazebo/components/JointType.hh"
#include "ignition/gazebo/components/Light.hh"
#include "ignition/gazebo/components/LinearVelocity.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Material.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentLinkName.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Sensor.hh"
#include "ignition/gazebo/components/Static.hh"
#include "ignition/gazebo/components/ThreadPitch.hh"
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/components/World.hh"

#include <map>
#include <set>
#include <vector>

#include "ignition/common/Profiler.hh"

using namespace ignition;
using namespace gazebo;

class ignition::gazebo::EntityComponentManagerPrivate
{
  /// \brief Recursively insert an entity and all its descendants into a given
  /// set.
  /// \param[in] _entity Entity to be inserted.
  /// \param[in, out] _set Set to be filled.
  public: void InsertEntityRecursive(Entity _entity,
      std::set<Entity> &_set);

  /// \brief Map of component storage classes. The key is a component
  /// type id, and the value is a pointer to the component storage.
  public: std::map<ComponentTypeId,
          std::unique_ptr<ComponentStorageBase>> components;

  /// \brief A graph holding all entities, arranged according to their
  /// parenting.
  public: EntityGraph entities;

  /// \brief Entities that have just been created
  public: std::set<Entity> newlyCreatedEntities;

  /// \brief Entities that need to be erased.
  public: std::set<Entity> toEraseEntities;

  /// \brief Flag that indicates if all entities should be erased.
  public: bool eraseAllEntities{false};

  /// \brief The set of components that each entity has
  public: std::map<Entity, std::vector<ComponentKey>> entityComponents;

  /// \brief A mutex to protect newly created entityes.
  public: std::mutex entityCreatedMutex;

  /// \brief A mutex to protect entity erase.
  public: std::mutex entityEraseMutex;

  /// \brief The set of all views.
  public: mutable std::map<detail::ComponentTypeKey, detail::View> views;

  /// \brief Keep track of entities already used to ensure uniqueness.
  public: uint64_t entityCount{0};
};

//////////////////////////////////////////////////
EntityComponentManager::EntityComponentManager()
  : dataPtr(new EntityComponentManagerPrivate)
{
}

//////////////////////////////////////////////////
EntityComponentManager::~EntityComponentManager() = default;

//////////////////////////////////////////////////
size_t EntityComponentManager::EntityCount() const
{
  return this->dataPtr->entities.Vertices().size();
}

/////////////////////////////////////////////////
Entity EntityComponentManager::CreateEntity()
{
  Entity entity = this->dataPtr->entityCount++;

  if (entity == std::numeric_limits<int64_t>::max())
  {
    ignwarn << "Reached maximum number of entities [" << entity << "]"
            << std::endl;
    return entity;
  }

  this->dataPtr->entities.AddVertex(std::to_string(entity), entity, entity);

  // Add entity to the list of newly created entities
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->entityCreatedMutex);
    this->dataPtr->newlyCreatedEntities.insert(entity);
  }

  return entity;
}

/////////////////////////////////////////////////
void EntityComponentManager::ClearNewlyCreatedEntities()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->entityCreatedMutex);
  this->dataPtr->newlyCreatedEntities.clear();
  for (auto &view : this->dataPtr->views)
  {
    view.second.ClearNewEntities();
  }
}

/////////////////////////////////////////////////
void EntityComponentManagerPrivate::InsertEntityRecursive(Entity _entity,
    std::set<Entity> &_set)
{
  for (const auto &vertex : this->entities.AdjacentsFrom(_entity))
  {
    this->InsertEntityRecursive(vertex.first, _set);
  }
  _set.insert(_entity);
}

/////////////////////////////////////////////////
void EntityComponentManager::RequestEraseEntity(Entity _entity, bool _recursive)
{
  // Leave children parentless
  if (!_recursive)
  {
    auto childEntities = this->ChildrenByComponents(_entity,
        components::ParentEntity(_entity));
    for (const auto childEntity : childEntities)
    {
      this->RemoveComponent<components::ParentEntity>(childEntity);
    }
  }

  {
    std::lock_guard<std::mutex> lock(this->dataPtr->entityEraseMutex);
    if (!_recursive)
    {
      this->dataPtr->toEraseEntities.insert(_entity);
    }
    else
    {
      this->dataPtr->InsertEntityRecursive(_entity,
          this->dataPtr->toEraseEntities);
    }
  }

  this->UpdateViews(_entity);
}

/////////////////////////////////////////////////
void EntityComponentManager::RequestEraseEntities()
{
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->entityEraseMutex);
    this->dataPtr->eraseAllEntities = true;
  }
  this->RebuildViews();
}

/////////////////////////////////////////////////
void EntityComponentManager::ProcessEraseEntityRequests()
{
  IGN_PROFILE("EntityComponentManager::ProcessEraseEntityRequests");
  std::lock_guard<std::mutex> lock(this->dataPtr->entityEraseMutex);
  // Short-cut if erasing all entities
  if (this->dataPtr->eraseAllEntities)
  {
    IGN_PROFILE("EraseAll");
    this->dataPtr->eraseAllEntities = false;
    this->dataPtr->entities = EntityGraph();
    this->dataPtr->entityComponents.clear();
    this->dataPtr->toEraseEntities.clear();

    for (std::pair<const ComponentTypeId,
        std::unique_ptr<ComponentStorageBase>> &comp: this->dataPtr->components)
    {
      comp.second->RemoveAll();
    }

    // All views are now invalid.
    this->dataPtr->views.clear();
  }
  else
  {
    IGN_PROFILE("Erase");
    // Otherwise iterate through the list of entities to erase.
    for (const Entity entity : this->dataPtr->toEraseEntities)
    {
      // Make sure the entity exists and is not erased.
      if (!this->HasEntity(entity))
        continue;

      // Remove from graph
      this->dataPtr->entities.RemoveVertex(entity);

      // Remove the components, if any.
      if (this->dataPtr->entityComponents.find(entity) !=
          this->dataPtr->entityComponents.end())
      {
        for (const ComponentKey &key :
            this->dataPtr->entityComponents.at(entity))
        {
          this->dataPtr->components.at(key.first)->Remove(key.second);
        }

        // Remove the entry in the entityComponent map
        this->dataPtr->entityComponents.erase(entity);
      }

      // Remove the entity from views.
      for (auto &view : this->dataPtr->views)
      {
        view.second.EraseEntity(entity, view.first);
      }
    }
    // Clear the set of entities to erase.
    this->dataPtr->toEraseEntities.clear();
  }
}

/////////////////////////////////////////////////
bool EntityComponentManager::RemoveComponent(
    const Entity _entity, const ComponentTypeId &_typeId)
{
  auto componentId = this->EntityComponentIdFromType(_entity, _typeId);
  ComponentKey key{_typeId, componentId};
  return this->RemoveComponent(_entity, key);
}

/////////////////////////////////////////////////
bool EntityComponentManager::RemoveComponent(
    const Entity _entity, const ComponentKey &_key)
{
  // Make sure the entity exists and has the component.
  if (!this->EntityHasComponent(_entity, _key))
    return false;

  auto entityComponentIter = std::find(
      this->dataPtr->entityComponents[_entity].begin(),
      this->dataPtr->entityComponents[_entity].end(), _key);

  this->dataPtr->components.at(_key.first)->Remove(_key.second);
  this->dataPtr->entityComponents[_entity].erase(entityComponentIter);

  this->UpdateViews(_entity);
  return true;
}

/////////////////////////////////////////////////
bool EntityComponentManager::EntityHasComponent(const Entity _entity,
    const ComponentKey &_key) const
{
  return this->HasEntity(_entity) &&
    std::find(this->dataPtr->entityComponents[_entity].begin(),
        this->dataPtr->entityComponents[_entity].end(), _key) !=
    this->dataPtr->entityComponents[_entity].end();
}

/////////////////////////////////////////////////
bool EntityComponentManager::EntityHasComponentType(const Entity _entity,
    const ComponentTypeId &_typeId) const
{
  if (!this->HasEntity(_entity))
    return false;

  auto iter = this->dataPtr->entityComponents.find(_entity);

  if (iter == this->dataPtr->entityComponents.end())
    return false;

  return std::find_if(iter->second.begin(), iter->second.end(),
      [&] (const ComponentKey &_key)
      {
        return _key.first == _typeId;
      }) != iter->second.end();
}

/////////////////////////////////////////////////
bool EntityComponentManager::IsNewEntity(const Entity _entity) const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->entityCreatedMutex);
  return this->dataPtr->newlyCreatedEntities.find(_entity) !=
         this->dataPtr->newlyCreatedEntities.end();
}

/////////////////////////////////////////////////
bool EntityComponentManager::IsMarkedForErasure(const Entity _entity) const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->entityEraseMutex);
  if (this->dataPtr->eraseAllEntities)
  {
    return true;
  }
  return this->dataPtr->toEraseEntities.find(_entity) !=
         this->dataPtr->toEraseEntities.end();
}

/////////////////////////////////////////////////
bool EntityComponentManager::HasEntity(const Entity _entity) const
{
  auto vertex = this->dataPtr->entities.VertexFromId(_entity);
  return vertex.Id() != math::graph::kNullId;
}

/////////////////////////////////////////////////
Entity EntityComponentManager::ParentEntity(const Entity _entity) const
{
  auto parents = this->Entities().AdjacentsTo(_entity);
  if (parents.empty())
    return kNullEntity;

  // TODO(louise) Do we want to support multiple parents?
  return parents.begin()->first;
}

/////////////////////////////////////////////////
bool EntityComponentManager::SetParentEntity(const Entity _child,
    const Entity _parent)
{
  // Remove current parent(s)
  auto parents = this->Entities().AdjacentsTo(_child);
  for (const auto &parent : parents)
  {
    auto edge = this->dataPtr->entities.EdgeFromVertices(parent.first, _child);
    this->dataPtr->entities.RemoveEdge(edge);
  }

  // Leave parent-less
  if (_parent == kNullEntity)
  {
    return true;
  }

  // Add edge
  auto edge = this->dataPtr->entities.AddEdge({_parent, _child}, true);
  return (math::graph::kNullId != edge.Id());
}

/////////////////////////////////////////////////
ComponentKey EntityComponentManager::CreateComponentImplementation(
    const Entity _entity, const ComponentTypeId _componentTypeId,
    const void *_data)
{
  // Instantiate the new component.
  std::pair<ComponentId, bool> componentIdPair =
    this->dataPtr->components[_componentTypeId]->Create(_data);

  ComponentKey componentKey{_componentTypeId, componentIdPair.first};

  this->dataPtr->entityComponents[_entity].push_back(componentKey);

  if (componentIdPair.second)
    this->RebuildViews();
  else
    this->UpdateViews(_entity);

  return componentKey;
}


/////////////////////////////////////////////////
bool EntityComponentManager::EntityMatches(Entity _entity,
    const std::set<ComponentTypeId> &_types) const
{
  auto iter = this->dataPtr->entityComponents.find(_entity);
  if (iter == this->dataPtr->entityComponents.end())
    return false;

  // \todo(nkoenig) The performance of this could be improved. Ideally we
  // wouldn't need two loops to confirm that an entity matches a set of
  // types. It might be possible to create bitmask for component sets.
  // Fixing this might not be high priority, unless we expect frequent
  // creation of entities and/or queries.
  for (const ComponentTypeId &type : _types)
  {
    bool found = false;
    for (const ComponentKey &comp : iter->second)
    {
      if (comp.first == type)
      {
        found = true;
        break;
      }
    }
    if (!found)
      return false;
  }

  return true;
}

/////////////////////////////////////////////////
ComponentId EntityComponentManager::EntityComponentIdFromType(
    const Entity _entity, const ComponentTypeId _type) const
{
  auto ecIter = this->dataPtr->entityComponents.find(_entity);

  if (ecIter == this->dataPtr->entityComponents.end())
    return -1;

  auto iter =
    std::find_if(ecIter->second.begin(), ecIter->second.end(),
      [&] (const ComponentKey &_key)
  {
    return _key.first == _type;
  });

  if (iter != ecIter->second.end())
    return iter->second;

  return -1;
}

/////////////////////////////////////////////////
const void *EntityComponentManager::ComponentImplementation(
    const Entity _entity, const ComponentTypeId _type) const
{
  auto ecIter = this->dataPtr->entityComponents.find(_entity);

  if (ecIter == this->dataPtr->entityComponents.end())
    return nullptr;

  auto iter = std::find_if(ecIter->second.begin(), ecIter->second.end(),
      [&] (const ComponentKey &_key)
  {
    return _key.first == _type;
  });

  if (iter != ecIter->second.end())
    return this->dataPtr->components.at(iter->first)->Component(iter->second);

  return nullptr;
}

/////////////////////////////////////////////////
void *EntityComponentManager::ComponentImplementation(
    const Entity _entity, const ComponentTypeId _type)
{
  auto ecIter = this->dataPtr->entityComponents.find(_entity);

  if (ecIter == this->dataPtr->entityComponents.end())
    return nullptr;

  auto iter =
    std::find_if(ecIter->second.begin(), ecIter->second.end(),
        [&] (const ComponentKey &_key)
  {
    return _key.first == _type;
  });

  if (iter != ecIter->second.end())
    return this->dataPtr->components.at(iter->first)->Component(iter->second);

  return nullptr;
}

/////////////////////////////////////////////////
const void *EntityComponentManager::ComponentImplementation(
    const ComponentKey &_key) const
{
  if (this->dataPtr->components.find(_key.first) !=
      this->dataPtr->components.end())
  {
    return this->dataPtr->components.at(_key.first)->Component(_key.second);
  }
  return nullptr;
}

/////////////////////////////////////////////////
void *EntityComponentManager::ComponentImplementation(const ComponentKey &_key)
{
  if (this->dataPtr->components.find(_key.first) !=
      this->dataPtr->components.end())
  {
    return this->dataPtr->components.at(_key.first)->Component(_key.second);
  }
  return nullptr;
}

/////////////////////////////////////////////////
bool EntityComponentManager::HasComponentType(
    const ComponentTypeId _typeId) const
{
  return this->dataPtr->components.find(_typeId) !=
    this->dataPtr->components.end();
}

/////////////////////////////////////////////////
void EntityComponentManager::RegisterComponentType(
    const ComponentTypeId _typeId,
    ComponentStorageBase *_type)
{
  igndbg << "Register new component type " << _typeId << ".\n";
  this->dataPtr->components[_typeId].reset(_type);
}

/////////////////////////////////////////////////
void *EntityComponentManager::First(const ComponentTypeId _componentTypeId)
{
  auto iter = this->dataPtr->components.find(_componentTypeId);
  if (iter != this->dataPtr->components.end())
  {
    return iter->second->First();
  }
  return nullptr;
}

//////////////////////////////////////////////////
const EntityGraph &EntityComponentManager::Entities() const
{
  return this->dataPtr->entities;
}

//////////////////////////////////////////////////
bool EntityComponentManager::FindView(const std::set<ComponentTypeId> &_types,
    std::map<detail::ComponentTypeKey, detail::View>::iterator &_iter) const
{
  _iter = this->dataPtr->views.find(_types);
  return _iter != this->dataPtr->views.end();
}

//////////////////////////////////////////////////
std::map<detail::ComponentTypeKey, detail::View>::iterator
    EntityComponentManager::AddView(const std::set<ComponentTypeId> &_types,
    detail::View &&_view) const
{
  // If the view already exists, then the map will return the iterator to
  // the location that prevented the insertion.
  return this->dataPtr->views.insert(
      std::make_pair(_types, std::move(_view))).first;
}

//////////////////////////////////////////////////
void EntityComponentManager::UpdateViews(const Entity _entity)
{
  IGN_PROFILE("EntityComponentManager::UpdateViews");
  for (auto &view : this->dataPtr->views)
  {
    // Add/update the entity if it matches the view.
    if (this->EntityMatches(_entity, view.first))
    {
      view.second.AddEntity(_entity, this->IsNewEntity(_entity));
      // If there is a request to delete this entity, update the view as
      // well
      if (this->IsMarkedForErasure(_entity))
      {
        view.second.AddEntityToErased(_entity);
      }
      for (const ComponentTypeId &compTypeId : view.first)
      {
        view.second.AddComponent(_entity, compTypeId,
            this->EntityComponentIdFromType(_entity, compTypeId));
      }
    }
    else
    {
      view.second.EraseEntity(_entity, view.first);
    }
  }
}

//////////////////////////////////////////////////
void EntityComponentManager::RebuildViews()
{
  IGN_PROFILE("EntityComponentManager::RebuildViews");
  for (auto &view : this->dataPtr->views)
  {
    view.second.entities.clear();
    view.second.components.clear();
    // Add all the entities that match the component types to the
    // view.
    for (const auto &vertex : this->dataPtr->entities.Vertices())
    {
      Entity entity = vertex.first;
      if (this->EntityMatches(entity, view.first))
      {
        view.second.AddEntity(entity, this->IsNewEntity(entity));
        // If there is a request to delete this entity, update the view as
        // well
        if (this->IsMarkedForErasure(entity))
        {
          view.second.AddEntityToErased(entity);
        }
        // Store pointers to all the components. This recursively adds
        // all the ComponentTypeTs that belong to the entity to the view.
        for (const ComponentTypeId &compTypeId : view.first)
        {
          view.second.AddComponent(entity, compTypeId,
              this->EntityComponentIdFromType(
                entity, compTypeId));
        }
      }
    }
  }
}

//////////////////////////////////////////////////
void EntityComponentManager::SetParent(Entity _child, Entity _parent)
{
  // TODO(louise) Figure out a way to avoid duplication while keeping all
  // state in components and also keeping a convenient graph in the ECM
  this->SetParentEntity(_child, _parent);
  this->CreateComponent(_child, components::ParentEntity(_parent));
}

//////////////////////////////////////////////////
Entity EntityComponentManager::CreateEntities(const sdf::World *_world,
    EventManager &_eventMgr)
{
  IGN_PROFILE("EntityComponentManager::CreateEntities(sdf::World)");

  // World entity
  Entity worldEntity = this->CreateEntity();

  // World components
  this->CreateComponent(worldEntity, components::World());
  this->CreateComponent(worldEntity,
      components::Name(_world->Name()));

  // Models
  for (uint64_t modelIndex = 0; modelIndex < _world->ModelCount();
      ++modelIndex)
  {
    auto model = _world->ModelByIndex(modelIndex);
    auto modelEntity = this->CreateEntities(model, _eventMgr);

    this->SetParent(modelEntity, worldEntity);
  }

  // Lights
  for (uint64_t lightIndex = 0; lightIndex < _world->LightCount();
      ++lightIndex)
  {
    auto light = _world->LightByIndex(lightIndex);
    auto lightEntity = this->CreateEntities(light);

    this->SetParent(lightEntity, worldEntity);
  }

  _eventMgr.Emit<events::LoadPlugins>(worldEntity, _world->Element());

  return worldEntity;
}

//////////////////////////////////////////////////
Entity EntityComponentManager::CreateEntities(const sdf::Model *_model,
    EventManager &_eventMgr)
{
  IGN_PROFILE("EntityComponentManager::CreateEntities(sdf::Model)");

  // Entity
  Entity modelEntity = this->CreateEntity();

  // Components
  this->CreateComponent(modelEntity, components::Model());
  this->CreateComponent(modelEntity,
      components::Pose(_model->Pose()));
  this->CreateComponent(modelEntity,
      components::Name(_model->Name()));
  this->CreateComponent(modelEntity,
      components::Static(_model->Static()));

  // NOTE: Pose components of links, visuals, and collisions are expressed in
  // the parent frame until we get frames working.

  // Links
  for (uint64_t linkIndex = 0; linkIndex < _model->LinkCount();
      ++linkIndex)
  {
    auto link = _model->LinkByIndex(linkIndex);
    auto linkEntity = this->CreateEntities(link);

    this->SetParent(linkEntity, modelEntity);
    if (linkIndex == 0)
    {
      this->CreateComponent(linkEntity, components::CanonicalLink());
    }
  }

  // Joints
  for (uint64_t jointIndex = 0; jointIndex < _model->JointCount();
      ++jointIndex)
  {
    auto joint = _model->JointByIndex(jointIndex);
    auto jointEntity = this->CreateEntities(joint);

    this->SetParent(jointEntity, modelEntity);
  }

  // Model plugins
  _eventMgr.Emit<events::LoadPlugins>(modelEntity, _model->Element());

  return modelEntity;
}

//////////////////////////////////////////////////
Entity EntityComponentManager::CreateEntities(const sdf::Light *_light)
{
  IGN_PROFILE("EntityComponentManager::CreateEntities(sdf::Light)");

  // Entity
  Entity lightEntity = this->CreateEntity();

  // Components
  this->CreateComponent(lightEntity, components::Light(*_light));
  this->CreateComponent(lightEntity,
      components::Pose(_light->Pose()));
  this->CreateComponent(lightEntity,
      components::Name(_light->Name()));

  return lightEntity;
}

//////////////////////////////////////////////////
Entity EntityComponentManager::CreateEntities(const sdf::Link *_link)
{
  IGN_PROFILE("EntityComponentManager::CreateEntities(sdf::Link)");

  // Entity
  Entity linkEntity = this->CreateEntity();

  // Components
  this->CreateComponent(linkEntity, components::Link());
  this->CreateComponent(linkEntity,
      components::Pose(_link->Pose()));
  this->CreateComponent(linkEntity,
      components::Name(_link->Name()));
  this->CreateComponent(linkEntity,
      components::Inertial(_link->Inertial()));

  // Visuals
  for (uint64_t visualIndex = 0; visualIndex < _link->VisualCount();
      ++visualIndex)
  {
    auto visual = _link->VisualByIndex(visualIndex);
    auto visualEntity = this->CreateEntities(visual);

    this->SetParent(visualEntity, linkEntity);
  }

  // Collisions
  for (uint64_t collisionIndex = 0; collisionIndex < _link->CollisionCount();
      ++collisionIndex)
  {
    auto collision = _link->CollisionByIndex(collisionIndex);
    auto collisionEntity = this->CreateEntities(collision);

    this->SetParent(collisionEntity, linkEntity);
  }

  // Lights
  for (uint64_t lightIndex = 0; lightIndex < _link->LightCount();
      ++lightIndex)
  {
    auto light = _link->LightByIndex(lightIndex);
    auto lightEntity = this->CreateEntities(light);

    this->SetParent(lightEntity, linkEntity);
  }

  // Sensors
  for (uint64_t sensorIndex = 0; sensorIndex < _link->SensorCount();
      ++sensorIndex)
  {
    auto sensor = _link->SensorByIndex(sensorIndex);
    auto sensorEntity = this->CreateEntities(sensor);

    this->SetParent(sensorEntity, linkEntity);
  }

  return linkEntity;
}

//////////////////////////////////////////////////
Entity EntityComponentManager::CreateEntities(const sdf::Joint *_joint)
{
  IGN_PROFILE("EntityComponentManager::CreateEntities(sdf::Joint)");

  // Entity
  Entity jointEntity = this->CreateEntity();

  // Components
  this->CreateComponent(jointEntity,
      components::Joint());
  this->CreateComponent(jointEntity,
      components::JointType(_joint->Type()));

  if (_joint->Axis(0))
  {
    this->CreateComponent(jointEntity,
        components::JointAxis(*_joint->Axis(0)));
  }

  if (_joint->Axis(1))
  {
    this->CreateComponent(jointEntity,
        components::JointAxis2(*_joint->Axis(1)));
  }

  this->CreateComponent(jointEntity,
      components::Pose(_joint->Pose()));
  this->CreateComponent(jointEntity ,
      components::Name(_joint->Name()));
  this->CreateComponent(jointEntity ,
      components::ThreadPitch(_joint->ThreadPitch()));
  this->CreateComponent(jointEntity,
      components::ParentLinkName(_joint->ParentLinkName()));
  this->CreateComponent(jointEntity,
      components::ChildLinkName(_joint->ChildLinkName()));

  return jointEntity;
}

//////////////////////////////////////////////////
Entity EntityComponentManager::CreateEntities(const sdf::Visual *_visual)
{
  IGN_PROFILE("EntityComponentManager::CreateEntities(sdf::Visual)");

  // Entity
  Entity visualEntity = this->CreateEntity();

  // Components
  this->CreateComponent(visualEntity, components::Visual());
  this->CreateComponent(visualEntity,
      components::Pose(_visual->Pose()));
  this->CreateComponent(visualEntity,
      components::Name(_visual->Name()));

  if (_visual->Geom())
  {
    this->CreateComponent(visualEntity,
        components::Geometry(*_visual->Geom()));
  }

  // \todo(louise) Populate with default material if undefined
  if (_visual->Material())
  {
    this->CreateComponent(visualEntity,
        components::Material(*_visual->Material()));
  }

  return visualEntity;
}

//////////////////////////////////////////////////
Entity EntityComponentManager::CreateEntities(const sdf::Collision *_collision)
{
  IGN_PROFILE("EntityComponentManager::CreateEntities(sdf::Collision)");

  // Entity
  Entity collisionEntity = this->CreateEntity();

  // Components
  this->CreateComponent(collisionEntity,
      components::Collision());
  this->CreateComponent(collisionEntity,
      components::Pose(_collision->Pose()));
  this->CreateComponent(collisionEntity,
      components::Name(_collision->Name()));

  if (_collision->Geom())
  {
    this->CreateComponent(collisionEntity,
        components::Geometry(*_collision->Geom()));
  }

  return collisionEntity;
}

//////////////////////////////////////////////////
Entity EntityComponentManager::CreateEntities(const sdf::Sensor *_sensor)
{
  IGN_PROFILE("EntityComponentManager::CreateEntities(sdf::Sensor)");

  // Entity
  Entity sensorEntity = this->CreateEntity();

  // Components
  this->CreateComponent(sensorEntity,
      components::Sensor());
  this->CreateComponent(sensorEntity,
      components::Pose(_sensor->Pose()));
  this->CreateComponent(sensorEntity,
      components::Name(_sensor->Name()));

  if (_sensor->Type() == sdf::SensorType::CAMERA)
  {
    auto elem = _sensor->Element();

    this->CreateComponent(sensorEntity,
        components::Camera(elem));
  }
  else if (_sensor->Type() == sdf::SensorType::ALTIMETER)
  {
     auto elem = _sensor->Element();

    this->CreateComponent(sensorEntity,
        components::Altimeter(elem));

    // create components to be filled by physics
    this->CreateComponent(sensorEntity,
        components::WorldPose(math::Pose3d::Zero));
    this->CreateComponent(sensorEntity,
        components::WorldLinearVelocity(math::Vector3d::Zero));
  }
  else
  {
    ignwarn << "Sensor type [" << static_cast<int>(_sensor->Type())
            << "] not supported yet." << std::endl;
  }

  return sensorEntity;
}
