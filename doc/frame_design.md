# Design document for Frames of Reference

## Disk Representation

### SDF

** Current Design **

SDF has a frame of reference mechanism that is very flexible at the expense
of efficiency. SDF's frame semantics was designed to handle cases where
frames are represented relative to parent entities, as a kinematic chain, or
a combination there of. A major assumption is that SDF is not used for
high-performance computation. Instead, SDF is a mechanism for data exchange.
A user of SDF should transform frame information into their preferred memory
or wire representation.

** Proposal(s) **


## Memory Representation

Goals:

1. Efficiency.
1. Cross-library usability. Ideally ign-\* or other library/app, could use a common API in ign-math or ign-common.
1. If a tree structure is not used, then we should have a mechanism to
   transform frames into a tree structure. This will be useful for
   visualization purposes (graphically show me the frame graph), writing
   back out to SDF, and sharing with other systems that may expect a tree
   structure. 

** Current Design **

Currently, [ignition::gazebo::components::Pose](https://bitbucket.org/ignitionrobotics/ign-gazebo/src/e38b48e51c74124e070cfc236cfd290941d04a1d/include/ignition/gazebo/components/Pose.hh?at=default&fileviewer=file-view-default) holds an `ignition::math::Pose3d` for an entity, but its frame of reference is not clear from the component itself. For now, the physics and rendering systems have been assuming the SDF spec:

Entity | Pose frame
----------- | ---------
Model | Parent entity (world or parent model)
Link | Parent model
Collision | Parent link
Visual | Parent link
Joint (not on `ign-gazebo` yet) | Child link

** Proposal(s) **

## Wire Representation

Goals:

1. Low-cost conversion of memory representation to wire representation.
1. As small (in terms of bytes) a representation as possible.

** Current Design **

We send pose information in a protobuf message over ignition transport. This
won't change. What could change is the message itself.

** Proposal(s) **
