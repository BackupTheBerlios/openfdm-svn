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
  typedef const Node* NodeId;

  Group(const std::string& name);
  virtual ~Group();

  virtual void accept(NodeVisitor& visitor);
  virtual void accept(ConstNodeVisitor& visitor) const;

  void traverse(NodeVisitor& visitor);
  void traverse(ConstNodeVisitor& visitor) const;

  NodeId addChild(const SharedPtr<Node>& node);
  unsigned getNumChildren() const;
  SharedPtr<Node> getChild(unsigned i);
  SharedPtr<const Node> getChild(unsigned i) const;
  unsigned getChildNumber(const Node* node) const;

  bool connect(const NodeId& nodeId0, const std::string& portName0,
               const NodeId& nodeId1, const std::string& portName1)
    OpenFDM_DEPRECATED
  {
    return connect(nodeId0->getPort(portName0), nodeId1->getPort(portName1));
  }
  bool connect(const NodeId& nodeId0, unsigned portNum0,
               const NodeId& nodeId1, unsigned portNum1)
    OpenFDM_DEPRECATED
  {
    return connect(nodeId0->getPort(portNum0), nodeId1->getPort(portNum1));
  }

  bool connect(const PortInfo* port0, const PortInfo* port1)
  {
    // Make sure the models belong to this group
    if (!port0)
      return false;
    SharedPtr<const Node> child0 = port0->getNode();
    if (!child0)
      return false;
    if (!child0->isChildOf(this))
      return false;

    if (!port1)
      return false;
    SharedPtr<const Node> child1 = port1->getNode();
    if (!child1)
      return false;
    if (!child1->isChildOf(this))
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

  struct Connect : public WeakReferenced {
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
