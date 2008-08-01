/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_NodeVisitor_H
#define OpenFDM_NodeVisitor_H

namespace OpenFDM {

class Node;
class GroupAcceptorNode;
class GroupProviderNode;
class LeafNode;
class Model;
class Body;
class Group;

class NodeVisitor {
public:
  virtual ~NodeVisitor();

  virtual void apply(Node& node)
  { }
  virtual void apply(GroupAcceptorNode& node)
  { apply((Node&)node); }
  virtual void apply(GroupProviderNode& node)
  { apply((Node&)node); }
  virtual void apply(LeafNode& node)
  { apply((Node&)node); }
  virtual void apply(Model& node)
  { apply((LeafNode&)node); }
  virtual void apply(Body& node)
  { apply((LeafNode&)node); }
  virtual void apply(Group& node)
  { apply((Node&)node); }
};

} // namespace OpenFDM

#endif
