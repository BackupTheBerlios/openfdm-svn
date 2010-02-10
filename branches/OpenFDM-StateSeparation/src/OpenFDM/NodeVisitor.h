/* -*-c++-*- OpenFDM - Copyright (C) 2007-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_NodeVisitor_H
#define OpenFDM_NodeVisitor_H

#include "Assert.h"
#include "Node.h"

namespace OpenFDM {

class Node;
class Group;
class GroupInterfaceNode;
class LibraryNode;
class LeafNode;
class AbstractModel;
class Input;
class Joint;
class Output;
class MechanicNode;
class RigidBody;
class RootJoint;
class Interact;

class Port;
class NumericPort;
class MechanicLink;

class NodeVisitor {
public:
  virtual ~NodeVisitor();

  virtual void apply(Node&);

  virtual void apply(Group&);
  virtual void apply(GroupInterfaceNode&);

  virtual void apply(LibraryNode&);

  virtual void apply(LeafNode&);
  virtual void apply(AbstractModel&);
  virtual void apply(Input&);
  virtual void apply(Output&);

  virtual void apply(MechanicNode&);

  virtual void apply(RigidBody&);

  virtual void apply(Joint&);
  virtual void apply(RootJoint&);

  virtual void apply(Interact&);

  virtual void apply(const Port&);
  virtual void apply(const NumericPort&);
  virtual void apply(const MechanicLink&);

  const NodePath& getNodePath() const { return mNodePath; }

  template<typename T>
  void handleNodePathAndApply(T* node)
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
