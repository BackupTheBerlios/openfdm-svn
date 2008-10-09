/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_ConstNodeVisitor_H
#define OpenFDM_ConstNodeVisitor_H

#include "Assert.h"
#include "Node.h"
#include "Referenced.h"

namespace OpenFDM {

class Node;
class Group;
class GroupInput;
class GroupOutput;
class GroupMechanicLink;
class LibraryNode;
class LeafNode;
class Model;
class Output;
class MechanicNode;
class RigidBody;
class RootJoint;
class Interact;

class ConstNodeVisitor : public Referenced {
public:
  virtual ~ConstNodeVisitor();

  virtual void apply(const Node&);

  virtual void apply(const Group&);
  virtual void apply(const GroupInput&);
  virtual void apply(const GroupOutput&);
  virtual void apply(const GroupMechanicLink&);

  virtual void apply(const LibraryNode&);

  virtual void apply(const LeafNode&);
  virtual void apply(const Model&);
  virtual void apply(const Output&);

  virtual void apply(const MechanicNode&);

  virtual void apply(const RigidBody&);

  virtual void apply(const Interact&);
  virtual void apply(const RootJoint&);

  const NodePath& getNodePath() const { return mNodePath; }

  template<typename T>
  void handleNodePathAndApply(const T* node)
  {
    OpenFDMAssert(node);
    mNodePath.push_back(node);
    apply(*node);
    mNodePath.pop_back();
  }
private:
  NodePath mNodePath;
};

} // namespace OpenFDM

#endif
