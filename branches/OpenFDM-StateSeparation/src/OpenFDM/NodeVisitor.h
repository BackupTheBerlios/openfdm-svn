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
class Group;

class NodeVisitor {
public:
  virtual ~NodeVisitor();

  virtual void apply(Node&);
  virtual void apply(Group&);
  virtual void apply(GroupAcceptorNode&);
  virtual void apply(GroupProviderNode&);
  virtual void apply(LeafNode&);
  virtual void apply(Model&);
};

} // namespace OpenFDM

#endif
