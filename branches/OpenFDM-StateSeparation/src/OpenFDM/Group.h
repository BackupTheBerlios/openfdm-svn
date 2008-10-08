/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Group_H
#define OpenFDM_Group_H

#include <string>
#include <vector>
#include <sstream>
#include "AcceptorPortInfo.h"
#include "ConstNodeVisitor.h"
#include "Node.h"
#include "NodeVisitor.h"
#include "Object.h"
#include "PortId.h"
#include "PortInfo.h"
#include "ProviderPortInfo.h"
#include "SharedPtr.h"

namespace OpenFDM {

/// Port structure:
/// InputPort (NumericPortValue, size constraint?)
/// OutputPort (NumericPortValue, size constraint?)
/// MechanicLink (MechanicPortValue ...)

class ProxyAcceptorPortInfo;
class ProxyProviderPortInfo;

class GroupAcceptorNode : public Node {
public:
  GroupAcceptorNode(const std::string& name = std::string());
  virtual void accept(NodeVisitor& visitor)
  { visitor.apply(*this); }
  virtual void accept(ConstNodeVisitor& visitor) const
  { visitor.apply(*this); }
// private:
  SharedPtr<ProxyProviderPortInfo> _groupInternalPort;
//   WeakPtr<ProxyAcceptorPortInfo> _groupExternalPort;
};

class GroupProviderNode : public Node {
public:
  GroupProviderNode(const std::string& name = std::string());
  virtual void accept(NodeVisitor& visitor)
  { visitor.apply(*this); }
  virtual void accept(ConstNodeVisitor& visitor) const
  { visitor.apply(*this); }
// private:
  SharedPtr<ProxyAcceptorPortInfo> _groupInternalPort;
//   WeakPtr<ProxyProviderPortInfo> _groupExternalPort;
//   MatrixInputPort mInputPort;
};

class ProxyAcceptorPortInfo : public AcceptorPortInfo {
public:
  ProxyAcceptorPortInfo(Node* node, const std::string& name = std::string()) :
    AcceptorPortInfo(node, name) {}
  SharedPtr<GroupAcceptorNode> mGroupPort;
  virtual const ProxyAcceptorPortInfo* toProxyAcceptorPortInfo() const
  { return this; }

  // FIXME
  virtual bool acceptPortValue(const PortValue* portValue) const
  { return true; }

};

class ProxyProviderPortInfo : public ProviderPortInfo {
public:
  ProxyProviderPortInfo(Node* node, const std::string& name = std::string()) :
    ProviderPortInfo(node, name) {}
  virtual PortValue* newValueImplementation() const
  { return 0; } //FIXME
  virtual const ProxyProviderPortInfo* toProxyProviderPortInfo() const
  { return this; }
  // FIXME
  virtual bool acceptPortValue(const PortValue* portValue) const
  { return true; }
  SharedPtr<GroupProviderNode> mGroupPort;
};

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
  unsigned getGroupPortNodeIndex(const PortId& portId) const
  {
    SharedPtr<const PortInfo> port = getPort(portId);
    if (!port)
      return ~0u;

    SharedPtr<Node> node;
    const ProxyProviderPortInfo* proxyProviderPort = port->toProxyProviderPortInfo();
    const ProxyAcceptorPortInfo* proxyAcceptorPort = port->toProxyAcceptorPortInfo();
    if (proxyProviderPort) {
      node = proxyProviderPort->mGroupPort;
    } else if (proxyAcceptorPort) {
      node = proxyAcceptorPort->mGroupPort;
    } else
      return ~0u;

    for (unsigned i = 0; i < _childList.size(); ++i) {
      if (_childList[i]->node == node)
        return i;
    }
    return ~0u;
  }

  // add a new group port to the group
  NodeId addAcceptorPort()
  {
    GroupAcceptorNode *groupAcceptorNode = new GroupAcceptorNode;
    NodeId nodeId = addChild(groupAcceptorNode);
    ProxyAcceptorPortInfo* proxyPort = new ProxyAcceptorPortInfo(this, "ProxyPort");
    proxyPort->mGroupPort = groupAcceptorNode;
    return nodeId;
  }
  NodeId addProviderPort()
  {
    GroupProviderNode *groupProviderNode = new GroupProviderNode;
    NodeId nodeId = addChild(groupProviderNode);
    ProxyProviderPortInfo* proxyPort = new ProxyProviderPortInfo(this, "ProxyPort");
    proxyPort->mGroupPort = groupProviderNode;
    return nodeId;
  }

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
    if (!getChild(nodeId0))
      return false;
    if (!getChild(nodeId1))
      return false;

    SharedPtr<const PortInfo> port0 = nodeId0.getPortPtr(portId0);
    if (!port0)
      return false;
    SharedPtr<const PortInfo> port1 = nodeId1.getPortPtr(portId1);
    if (!port1)
      return false;

    // Just a crude first time check if this will work in principle.
    if (!port0->canConnect(*port1))
      return false;

    if (port0->toProviderPortInfo() && port1->toAcceptorPortInfo()) {
      SharedPtr<Connect> connect = new Connect;
      if (!connect->setProvider(nodeId0, portId0))
        return false;
      if (!connect->addAcceptor(nodeId1, portId1))
        return false;
      _connectList.push_back(connect);
      return true;
    } else if (port1->toProviderPortInfo() && port0->toAcceptorPortInfo()) {
      SharedPtr<Connect> connect = new Connect;
      if (!connect->setProvider(nodeId1, portId1))
        return false;
      if (!connect->addAcceptor(nodeId0, portId0))
        return false;
      _connectList.push_back(connect);
      return true;
    } else {
      return false;
    }
  }


  unsigned getNumConnects() const
  { return _connectList.size(); }

  unsigned getConnectAcceptorNodeIndex(unsigned i) const
  {
    if (getNumConnects() <= i)
      return ~0u;
    NodeId nodeId = _connectList[i]->_acceptorNodeId;
    return getChildNumber(nodeId);
  }
  unsigned getConnectProviderNodeIndex(unsigned i) const
  {
    if (getNumConnects() <= i)
      return ~0u;
    NodeId nodeId = _connectList[i]->_providerNodeId;
    return getChildNumber(nodeId);
  }

  SharedPtr<const AcceptorPortInfo>
  getConnectAcceptorPortInfo(unsigned i) const
  {
    if (getNumConnects() <= i)
      return 0;
    return _connectList[i]->_acceptorPort.lock();
  }
  SharedPtr<const ProviderPortInfo>
  getConnectProviderPortInfo(unsigned i) const
  {
    if (getNumConnects() <= i)
      return 0;
    return _connectList[i]->_providerPort.lock();
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

    const PortInfo* getPortPtr(const PortId& portId) const // FIXME??
    {
      SharedPtr<Child> child = _child.lock();
      if (!child)
        return 0;
      SharedPtr<Node> node = child->node;
      if (!node)
        return 0;
      return node->getPort(portId);
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
  };

  struct Connect : public WeakReferenced {
    bool setProvider(const NodeId& node, const PortId& portId)
    {
      SharedPtr<const PortInfo> port = node.getPortPtr(portId);
      if (!port)
        return false;
      const ProviderPortInfo* providerPort = port->toProviderPortInfo();
      if (!providerPort)
        return false;
      _providerNodeId = node;
      _providerPort = providerPort;
      return true;
    }
    bool addAcceptor(const NodeId& node, const PortId& portId)
    {
      SharedPtr<const PortInfo> port = node.getPortPtr(portId);
      if (!port)
        return false;
      const AcceptorPortInfo* acceptorPort = port->toAcceptorPortInfo();
      if (!acceptorPort)
        return false;
      _acceptorNodeId = node;
      _acceptorPort = acceptorPort;
      return true;
    }

    NodeId _providerNodeId;
    WeakPtr<const ProviderPortInfo> _providerPort;

    NodeId _acceptorNodeId;
    WeakPtr<const AcceptorPortInfo> _acceptorPort;

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
