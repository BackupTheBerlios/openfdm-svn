/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Group_H
#define OpenFDM_Group_H

#include <string>
#include <vector>
#include "Connect.h"
#include "Node.h"
#include "Port.h"
#include "SharedPtr.h"

namespace OpenFDM {

class Group : public Node {
  OPENFDM_OBJECT(Group, Node);
public:
  Group(const std::string& name);
  virtual ~Group();

  /// Methods for the visitors.
  virtual void accept(NodeVisitor& visitor);
  virtual void accept(ConstNodeVisitor& visitor) const;

  /// Traverse this groups children with a visitor
  void traverse(NodeVisitor& visitor);
  void traverse(ConstNodeVisitor& visitor) const;

  /// Add a new child. Returns the number of this child within the group
  /// on success else ~0u is returned.
  unsigned addChild(Node* node);
  /// Remove the given child.
  void removeChild(const Node* node);
  void removeChild(unsigned i);
  /// Returns the number of children
  unsigned getNumChildren() const;
  /// Get child at index i.
  Node* getChild(unsigned i);
  /// Get child at index i.
  const Node* getChild(unsigned i) const;
  /// Get child number of the given node. If the node is not contained in
  /// the group ~0u is returned.
  unsigned getChildIndex(const Node* node) const;


  /// Connect api
  unsigned getNumConnects() const;
  Connect* getConnect(unsigned i);
  const Connect* getConnect(unsigned i) const;
  void removeConnect(unsigned i);
  void removeConnect(const Connect* connect);
  unsigned getConnectIndex(const Connect* connect) const;
  
  /// Create a new connect and connect the given ports with it
  /// FIXME, this is currently the only way to get a Connect into a group ...
  Connect* connect(const Port* port0, const Port* port1);

private:
  typedef std::vector<SharedPtr<Connect> > ConnectList;
  ConnectList mConnectList;

  typedef std::vector<SharedPtr<Node> > ChildList;
  ChildList mChildList;
};

} // namespace OpenFDM

#endif
