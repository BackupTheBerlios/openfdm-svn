/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Group_H
#define OpenFDM_Group_H

#include <string>
#include <vector>
#include <sstream>
#include "Node.h"
#include "PortId.h"
#include "PortInfo.h"
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

  /// Add a new child. Returns the number of this child wthin the group
  /// on success else ~0u is returned.
  unsigned addChild(const SharedPtr<Node>& node);
  /// Remove the given child. Returns the true on success.
  bool removeChild(const Node* node);
  /// Returns the number of children
  unsigned getNumChildren() const;
  /// Get child at index i.
  SharedPtr<Node> getChild(unsigned i);
  /// Get child at index i.
  SharedPtr<const Node> getChild(unsigned i) const;
  /// Get child number of the given node. If the node is not contained in
  /// the group ~0u is returned.
  unsigned getChildNumber(const Node* node) const;

  bool isChildPort(const PortInfo* portInfo) const
  {
    if (!portInfo)
      return false;
    SharedPtr<const Node> node = portInfo->getNode();
    if (!node)
      return false;
    if (!node->isChildOf(this))
      return false;
    return true;
  }
  
  bool connect(const PortInfo* port0, const PortInfo* port1)
  {
    // Make sure the models belong to this group
    if (!isChildPort(port0))
      return false;
    if (!isChildPort(port1))
      return false;

    // Just a crude first time check if this will work in principle.
    if (!port0->canConnect(*port1))
      return false;

    SharedPtr<Connect> connect = new Connect;
    connect->mPortInfo0 = port0;
    connect->mPortInfo1 = port1;
    _connectList.push_back(connect);

    return true;
  }

  unsigned getNumConnects() const
  { return _connectList.size(); }

  unsigned getConnectNodeIndex0(unsigned i) const
  {
    if (getNumConnects() <= i)
      return ~0u;
    return getChildNumber(_connectList[i]->mPortInfo0.lock()->getNode());
  }
  unsigned getConnectNodeIndex1(unsigned i) const
  {
    if (getNumConnects() <= i)
      return ~0u;
    return getChildNumber(_connectList[i]->mPortInfo1.lock()->getNode());
  }

  SharedPtr<const PortInfo>
  getConnectPortInfo0(unsigned i) const
  {
    if (getNumConnects() <= i)
      return 0;
    return _connectList[i]->mPortInfo0.lock();
  }
  SharedPtr<const PortInfo>
  getConnectPortInfo1(unsigned i) const
  {
    if (getNumConnects() <= i)
      return 0;
    return _connectList[i]->mPortInfo1.lock();
  }

private:

  struct Connect : public Referenced {
    WeakPtr<const PortInfo> mPortInfo0;
    WeakPtr<const PortInfo> mPortInfo1;

    // Where the line in the gui will be ...??
    // std::list<Vector2> _positions;
  };

  typedef std::vector<SharedPtr<Connect> > ConnectList;
  ConnectList _connectList;

  typedef std::vector<SharedPtr<Node> > ChildList;
  ChildList _childList;
};

} // namespace OpenFDM

#endif
