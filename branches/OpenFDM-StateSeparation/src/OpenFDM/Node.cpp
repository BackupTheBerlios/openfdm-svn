/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "Node.h"

#include "ConstNodeVisitor.h"
#include "Group.h"
#include "NodeVisitor.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Node, Object)
  END_OPENFDM_OBJECT_DEF

Node::Node(const std::string& name) :
  Object(name),
  mSampleTime(SampleTime::getInherited())
{
}

Node::~Node()
{
}

void
Node::accept(NodeVisitor& visitor)
{
  visitor.handleNodePathAndApply(this);
}

void
Node::accept(ConstNodeVisitor& visitor) const
{
  visitor.handleNodePathAndApply(this);
}

void
Node::ascend(NodeVisitor& visitor)
{
  ParentList::iterator i;
  for (i = mParentList.begin(); i != mParentList.end(); ++i) {
    SharedPtr<Node> parent = i->lock();
    if (!parent)
      continue;
    parent->accept(visitor);
  }
}

void
Node::ascend(ConstNodeVisitor& visitor) const
{
  ParentList::const_iterator i;
  for (i = mParentList.begin(); i != mParentList.end(); ++i) {
    SharedPtr<const Node> parent = i->lock();
    if (!parent)
      continue;
    parent->accept(visitor);
  }
}

void
Node::traversePorts(NodeVisitor& visitor) const
{
  PortList::const_iterator i;
  for (i = mPortList.begin(); i != mPortList.end(); ++i)
    (*i)->accept(visitor);
}

void
Node::traversePorts(ConstNodeVisitor& visitor) const
{
  PortList::const_iterator i;
  for (i = mPortList.begin(); i != mPortList.end(); ++i)
    (*i)->accept(visitor);
}

WeakPtr<const Node>
Node::getParent(unsigned i) const
{
  if (mParentList.size() <= i)
    return 0;
  return mParentList[i];
}

WeakPtr<Node>
Node::getParent(unsigned i)
{
  if (mParentList.size() <= i)
    return 0;
  return mParentList[i];
}

bool
Node::isChildOf(const Group* group) const
{
  ParentList::const_iterator i;
  for (i = mParentList.begin(); i != mParentList.end(); ++i) {
    SharedPtr<const Node> node = i->lock();
    if (node == group)
      return true;
  }
  return false;
}

SharedPtr<const PortInfo>
Node::getPort(const PortId& portId) const
{
  return portId._port.lock();
}

SharedPtr<const PortInfo>
Node::getPort(unsigned index) const
{
  if (mPortList.size() <= index)
    return 0;
  return mPortList[index];
}

SharedPtr<const PortInfo>
Node::getPort(const std::string& name) const
{
  return getPort(getPortId(name));
}

unsigned
Node::getNumPorts() const
{
  return mPortList.size();
}

PortId
Node::getPortId(unsigned index) const
{
  if (mPortList.size() <= index)
    return PortId();
  return PortId(mPortList[index]);
}

PortId
Node::getPortId(const std::string& name) const
{
  PortList::const_iterator i;
  for (i = mPortList.begin(); i != mPortList.end(); ++i) {
    if (name == (*i)->getName())
      return PortId(*i);
  }
  return PortId();
}

unsigned
Node::getPortIndex(const PortId& portId) const
{
  SharedPtr<const PortInfo> port = portId._port.lock();
  if (!port)
    return ~0u;
  PortList::const_iterator i;
  i = std::find(mPortList.begin(), mPortList.end(), portId._port.lock());
  if (i == mPortList.end())
    return ~0u;
  return port->getIndex();
}

bool
Node::checkPort(const PortId& portId) const
{
  PortList::const_iterator i;
  i = std::find(mPortList.begin(), mPortList.end(), portId._port.lock());
  return i != mPortList.end();
}

bool
Node::addParent(Node* parent)
{
  if (!parent)
    return false;
  ParentList::const_iterator i;
  for (i = mParentList.begin(); i != mParentList.end(); ++i) {
    SharedPtr<const Node> lockedParent = i->lock();
    if (parent == lockedParent)
      return false;
  }
  mParentList.push_back(parent);
  return true;
}

void
Node::removeParent(Node* parent)
{
  ParentList::iterator i;
  for (i = mParentList.begin(); i != mParentList.end();) {
    SharedPtr<const Node> lockedParent = i->lock();
    if (parent == lockedParent)
      i = mParentList.erase(i);
    else
      ++i;
  }
}

class Node::NodePathListCollectVisitor : public ConstNodeVisitor {
public:
  virtual void apply(const Node& node)
  {
    if (node.getNumParents()) {
      node.ascend(*this);
    } else {
      mNodePathList.push_back(getNodePath());
    }
  }

  NodePathList mNodePathList;
};

NodePathList
Node::getNodePathList() const
{
  NodePathListCollectVisitor visitor;
  return visitor.mNodePathList;
}

void
Node::addPort(PortInfo* port)
{
  if (!port)
    return;
  port->setIndex(mPortList.size());
  mPortList.push_back(port);
}

void
Node::removePort(PortInfo* port)
{
  PortList::iterator i;
  i = std::find(mPortList.begin(), mPortList.end(), port);
  if (i == mPortList.end())
    return;
  unsigned index = port->getIndex();
  port->invalidateIndex();
  i = mPortList.erase(i);
  for (; i != mPortList.end(); ++i)
    (*i)->setIndex(index++);
}

} // namespace OpenFDM
