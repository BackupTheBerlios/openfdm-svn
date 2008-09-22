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
  virtual ProxyAcceptorPortInfo* toProxyAcceptorPortInfo()
  { return this; }
  virtual const ProxyAcceptorPortInfo* toProxyAcceptorPortInfo() const
  { return this; }
};

class ProxyProviderPortInfo : public ProviderPortInfo {
public:
  ProxyProviderPortInfo(Node* node, const std::string& name = std::string()) :
    ProviderPortInfo(node, name) {}
  virtual PortValue* newValueImplementation() const
  { return 0; } //FIXME
  virtual ProxyProviderPortInfo* toProxyProviderPortInfo()
  { return this; }
  virtual const ProxyProviderPortInfo* toProxyProviderPortInfo() const
  { return this; }
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

  NodeId addChild(const SharedPtr<Node>& node);
  unsigned getNumChildren() const;
  NodeId getNodeId(unsigned i) const;
  unsigned getChildNumber(const NodeId& nodeId) const;
  SharedPtr<Node> getChild(unsigned i);
  SharedPtr<const Node> getChild(unsigned i) const;
  SharedPtr<Node> getChild(const NodeId& nodeId);
  SharedPtr<const Node> getChild(const NodeId& nodeId) const;

  NodeId getGroupAcceptorNode(const PortId& portId) const
  {
    SharedPtr<const PortInfo> port = getPort(portId);
    if (!port)
      return NodeId();
    const ProxyAcceptorPortInfo* proxyAcceptorPort = port->toProxyAcceptorPortInfo();
    if (!proxyAcceptorPort)
      return NodeId();

    SharedPtr<GroupAcceptorNode> groupPort = proxyAcceptorPort->mGroupPort;
    ChildList::const_iterator i;
    for (i = _childList.begin(); i != _childList.end(); ++i) {
      if ((*i)->node == groupPort)
        return NodeId(*i);
    }
    return NodeId();
  }
  NodeId getGroupProviderNode(const PortId& portId) const
  {
    SharedPtr<const PortInfo> port = getPort(portId);
    if (!port)
      return NodeId();
    const ProxyProviderPortInfo* proxyProviderPort = port->toProxyProviderPortInfo();
    if (!proxyProviderPort)
      return NodeId();

    SharedPtr<GroupProviderNode> groupPort = proxyProviderPort->mGroupPort;
    ChildList::const_iterator i;
    for (i = _childList.begin(); i != _childList.end(); ++i) {
      if ((*i)->node == groupPort)
        return NodeId(*i);
    }
    return NodeId();
  }
  NodeId getGroupPortNode(const PortId& portId) const
  {
    NodeId nodeId = getGroupProviderNode(portId);
    if (getChild(nodeId)) // FIXME!!
      return nodeId;
    return getGroupAcceptorNode(portId);
  }
  unsigned getGroupPortNodeIndex(const PortId& portId) const
  {
    return getChildNumber(getGroupPortNode(portId));
  }

  PortId getGroupPort(const NodeId& nodeId) const
  {
    // FIXME horrible algorithm
    unsigned numPorts = getNumPorts();
    for (unsigned i = 0; i < numPorts; ++i) {
      NodeId thisId = getGroupPortNode(getPortId(i));
      if (getChildNumber(nodeId) == getChildNumber(thisId))
        return getPortId(i);
    }
    return PortId();
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
  
  NodeId getConnectAcceptorNodeId(unsigned i) const
  {
    if (getNumConnects() <= i)
      return NodeId();
    return _connectList[i]->_acceptorNodeId;
  }
  unsigned getConnectAcceptorNodeIndex(unsigned i) const
  {
    return getChildNumber(getConnectAcceptorNodeId(i));
  }
  NodeId getConnectProviderNodeId(unsigned i) const
  {
    if (getNumConnects() <= i)
      return NodeId();
    return _connectList[i]->_providerNodeId;
  }
  unsigned getConnectProviderNodeIndex(unsigned i) const
  {
    return getChildNumber(getConnectProviderNodeId(i));
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

  PortId getConnectAcceptorPortId(unsigned i) const
  { return PortId(SharedPtr<const PortInfo>(getConnectAcceptorPortInfo(i))); }
  PortId getConnectProviderPortId(unsigned i) const
  { return PortId(SharedPtr<const PortInfo>(getConnectProviderPortInfo(i))); }

private:
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

    std::string getId() const
    {
      SharedPtr<Child> child = _child.lock();
      if (!child)
        return std::string();
      return _child.lock()->identifier;
    }

    // FIXME, do I need ???
    bool operator<(const NodeId& nodeId) const
    { return _child < nodeId._child; }

  private:
    friend class Group;
    NodeId(const SharedPtr<Child>& child) : _child(child) {}
    WeakPtr<Child> _child;
  };

private:

  struct Child : public WeakReferenced {
    Child(Group* _group, Node* _node, std::string& id) :
      group(_group), node(_node), identifier(id)
    { }
    WeakPtr<Group> group;
    SharedPtr<Node> node;
    // what happens if the nodes name changes ?? FIXME
    std::string identifier;
  };

  std::string getUniqueIdentifier(const std::string& name) const
  {
    if (isUniqueIdentifier(name))
      return name;
    unsigned counter = 0;
    std::string identifier;
    do {
      std::stringstream ss;
      ss << name << '(' << ++counter << ')';
      identifier = ss.str();
    } while (!isUniqueIdentifier(identifier));
    OpenFDMAssert(!identifier.empty());
    return identifier;
  }

  // Tells true if the identifier is not yet used
  bool isUniqueIdentifier(const std::string& identifier) const
  {
    for (ChildList::const_iterator i = _childList.begin();
         i != _childList.end(); ++i) {
      if ((*i)->identifier == identifier)
        return false;
    }
    return true;
  }

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
      _providerPortId = portId;
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
      _acceptorPortId = portId;
      _acceptorPort = acceptorPort;
      return true;
    }

    NodeId _providerNodeId;
    PortId _providerPortId;
    WeakPtr<const ProviderPortInfo> _providerPort;

    NodeId _acceptorNodeId;
    PortId _acceptorPortId;
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
