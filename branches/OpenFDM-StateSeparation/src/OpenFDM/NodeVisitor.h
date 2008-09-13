/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_NodeVisitor_H
#define OpenFDM_NodeVisitor_H

namespace OpenFDM {

class Node;
class Group;
class GroupAcceptorNode;
class GroupProviderNode;
class LibraryNode;
class LeafNode;
class Model;
class Output;
class MechanicNode;
class RigidBody;
class RootJoint;
class Interact;

class NodeVisitor {
public:
  virtual ~NodeVisitor();

  virtual void apply(Node&);

  virtual void apply(Group&);
  virtual void apply(GroupAcceptorNode&);
  virtual void apply(GroupProviderNode&);

  virtual void apply(LibraryNode&);

  virtual void apply(LeafNode&);
  virtual void apply(Model&);
  virtual void apply(Output&);

  virtual void apply(MechanicNode&);

  virtual void apply(RigidBody&);

  virtual void apply(Interact&);
  virtual void apply(RootJoint&);
};

} // namespace OpenFDM

#endif