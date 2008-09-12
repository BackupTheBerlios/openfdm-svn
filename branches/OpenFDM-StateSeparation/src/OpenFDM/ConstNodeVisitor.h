/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_ConstNodeVisitor_H
#define OpenFDM_ConstNodeVisitor_H

namespace OpenFDM {

class Node;
class Group;
class GroupAcceptorNode;
class GroupProviderNode;
class LibraryNode;
class LeafNode;
class Model;
class MechanicNode;
class RigidBody;
class RootJoint;
class Interact;

class ConstNodeVisitor {
public:
  virtual ~ConstNodeVisitor();

  virtual void apply(const Node&);

  virtual void apply(const Group&);
  virtual void apply(const GroupAcceptorNode&);
  virtual void apply(const GroupProviderNode&);

  virtual void apply(const LibraryNode&);

  virtual void apply(const LeafNode&);
  virtual void apply(const Model&);

  virtual void apply(const MechanicNode&);

  virtual void apply(const RigidBody&);

  virtual void apply(const Interact&);
  virtual void apply(const RootJoint&);
};

} // namespace OpenFDM

#endif
