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
  class NodeId;

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

  bool connect(const NodeId& nodeId0, const std::string& portName0,
               const NodeId& nodeId1, const std::string& portName1)
  { return connect(nodeId0, nodeId0.getPortId(portName0),
                   nodeId1, nodeId1.getPortId(portName1)); }
  bool connect(const NodeId& nodeId0, unsigned portNum0,
               const NodeId& nodeId1, unsigned portNum1)
  { return connect(nodeId0, nodeId0.getPortId(portNum0),
                   nodeId1, nodeId1.getPortId(portNum1)); }

  bool connect(const NodeId& nodeId0, const PortId& portId0,
               const NodeId& nodeId1, const PortId& portId1)
  {
    // Make sure the models belong to this group
    SharedPtr<Node> child0 = getChild(nodeId0);
    if (!child0)
      return false;
    SharedPtr<Node> child1 = getChild(nodeId1);
    if (!child1)
      return false;

    SharedPtr<const PortInfo> port0 = child0->getPort(portId0);
    if (!port0)
      return false;
    SharedPtr<const PortInfo> port1 = child1->getPort(portId1);
    if (!port1)
      return false;

    // Just a crude first time check if this will work in principle.
    if (!port0->canConnect(*port1))
      return false;

    SharedPtr<Connect> connect = new Connect;
    connect->_portId0 = portId0;
    connect->_nodeId0 = nodeId0;
    connect->_portId1 = portId1;
    connect->_nodeId1 = nodeId1;
    _connectList.push_back(connect);

    return true;
  }


  unsigned getNumConnects() const
  { return _connectList.size(); }

  unsigned getConnectNodeIndex0(unsigned i) const
  {
    if (getNumConnects() <= i)
      return ~0u;
    return getChildNumber(_connectList[i]->_nodeId0);
  }
  unsigned getConnectNodeIndex1(unsigned i) const
  {
    if (getNumConnects() <= i)
      return ~0u;
    return getChildNumber(_connectList[i]->_nodeId1);
  }

  SharedPtr<const PortInfo>
  getConnectPortInfo0(unsigned i) const
  {
    if (getNumConnects() <= i)
      return 0;
    SharedPtr<const Node> node = getChild(_connectList[i]->_nodeId0);
    return node->getPort(_connectList[i]->_portId0);
  }
  SharedPtr<const PortInfo>
  getConnectPortInfo1(unsigned i) const
  {
    if (getNumConnects() <= i)
      return 0;
    SharedPtr<const Node> node = getChild(_connectList[i]->_nodeId1);
    return node->getPort(_connectList[i]->_portId1);
  }

private:
  unsigned getChildNumber(const NodeId& nodeId) const;
  SharedPtr<Node> getChild(const NodeId& nodeId);
  SharedPtr<const Node> getChild(const NodeId& nodeId) const;
  class Child;
public:
  class NodeId {
    // FIXME a node ID has an associated name and that is unique. That should
    // be the blocks name where it can be referenced. May be the NodeId should
    // just contain that string??
    // A serialized group can refere these names.
    // May be the same should happen with portid's???
    //
    // Remove the name from the Object.
    // Store Connects as seperate objects
  public:
    NodeId() {}
    PortId getPortId(unsigned i) const
    {
      SharedPtr<Child> child = _child.lock();
      if (!child)
        return PortId();
      SharedPtr<Node> node = child->node;
      if (!node)
        return PortId();
      return node->getPortId(i);
    }
    PortId getPortId(const std::string& name) const
    {
      SharedPtr<Child> child = _child.lock();
      if (!child)
        return PortId();
      SharedPtr<Node> node = child->node;
      if (!node)
        return PortId();
      return node->getPortId(name);
    }

  private:
    friend class Group;
    NodeId(const SharedPtr<Child>& child) : _child(child) {}
    WeakPtr<Child> _child;
  };

private:

  struct Child : public WeakReferenced {
    Child(Group* _group, Node* _node) :
      group(_group), node(_node)
    { }
    WeakPtr<Group> group;
    SharedPtr<Node> node;
    // name extension to make name uniqe?
  };

  struct Connect : public WeakReferenced {
    NodeId _nodeId0;
    PortId _portId0;

    NodeId _nodeId1;
    PortId _portId1;

    // Where the line in the gui will be ...??
    // std::list<Vector2> _positions;
  };

  typedef std::vector<SharedPtr<Connect> > ConnectList;
  ConnectList _connectList;

  typedef std::vector<SharedPtr<Child> > ChildList;
  ChildList _childList;
};

} // namespace OpenFDM

#endif
