/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_MechanicNode_H
#define OpenFDM_MechanicNode_H

#include <string>
#include "LeafNode.h"
#include "MechanicLink.h"

namespace OpenFDM {

// Ok, types of mechanic nodes:
// Joints, mostly two ports need ordered execution
// RigidBody, no own logic, just moving ports from parent to children.
//   should not show up in computations at all
// Interact, forces internal, external, mass, sensor, no ordering required,
//   could execute parallel

// For contexts:
// JointContext has internal data for the AB algorithm
// No RigidBody context at all
// Interact, something similar we have now for the mechanic context??

// Current inheritence tree:
// Leaf?- Model ...
//      |
//      |- MechanicNode?-- RigidBody
//                      |
//                      |- Interact  -- Mass
//                      |            |- Sensor
//                      |            |- Force
//                      |
//                      |- Joint ------ RootJoint
//                                   |- CartesianJoint -- RevoluteJoint
//                                                     | ...

// TODO:
// * Remove AbstractNodeContext ...
// * allocate contexts later ...

class DiscreteTask;
class PortValueList;
class Task;
class MechanicContext;

class MechanicNode : public LeafNode {
  OPENFDM_OBJECT(MechanicNode, LeafNode);
public:
  MechanicNode(const std::string& name);
  virtual ~MechanicNode();

  virtual void accept(NodeVisitor& visitor);
  virtual void accept(ConstNodeVisitor& visitor) const;

  virtual AbstractNodeContext* newNodeContext() const;
  virtual MechanicContext* newMechanicContext() const = 0;

protected:
  MechanicLink newMechanicLink(const std::string& name)
  { return MechanicLink(this, name); }
};

} // namespace OpenFDM

#endif
