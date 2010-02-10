/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#include "Node.h"

#include "ConstNodeVisitor.h"
#include "Group.h"
#include "LogStream.h"
#include "NodeVisitor.h"
#include "TypeInfo.h"
#include "Variant.h"

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
Node::isChildOf(const Node* node) const
{
  if (!node)
    return false;
  ParentList::const_iterator i;
  for (i = mParentList.begin(); i != mParentList.end(); ++i) {
    SharedPtr<const Node> parentNode = i->lock();
    if (node == parentNode)
      return true;
  }
  return false;
}

unsigned
Node::getNumPorts() const
{
  return mPortList.size();
}

SharedPtr<const Port>
Node::getPort(unsigned index) const
{
  if (mPortList.size() <= index)
    return 0;
  return mPortList[index];
}

SharedPtr<const Port>
Node::getPort(const std::string& name) const
{
  PortList::const_iterator i;
  for (i = mPortList.begin(); i != mPortList.end(); ++i) {
    if (name == (*i)->getName())
      return *i;
   }
  return 0;
}

unsigned
Node::getPortIndex(const Port* port) const
{
  PortList::const_iterator i;
  for (i = mPortList.begin(); i != mPortList.end(); ++i) {
    if (port == i->get())
      return std::distance(mPortList.begin(), i);
  }
  return ~0u;
}

bool
Node::checkPort(const Port* port) const
{
  PortList::const_iterator i;
  for (i = mPortList.begin(); i != mPortList.end(); ++i) {
    if (port == i->get())
      return true;
  }
  return false;
}

class Node::CycleCheckVisitor : public ConstNodeVisitor {
public:
  CycleCheckVisitor(const Node* node) :
    mNode(node), mDetectedCycle(false)
  { }
  virtual void apply(const Node& node)
  {
    if (mNode == &node)
      mDetectedCycle = true;
    else
      node.ascend(*this);
  }
  const Node* mNode;
  bool mDetectedCycle;
};

bool
Node::addParent(Node* parent)
{
  if (!parent)
    return false;
  ParentList::const_iterator i;
  for (i = mParentList.begin(); i != mParentList.end(); ++i) {
    SharedPtr<const Node> lockedParent = i->lock();
    if (parent == lockedParent) {
      Log(Model, Info) << "Cannot add model \"" << getName()
                       << "\" a second time to the parent \""
                       << lockedParent->getName() << "\"" << std::endl;
      return false;
    }
  }

  CycleCheckVisitor visitor(this);
  visitor.apply(*parent);
  if (visitor.mDetectedCycle) {
    Log(Model, Info) << "Cannot add model \"" << getName()
                     << "\" to parent \"" << parent->getName()
                     << "\". Node cannot be parent of itself." << std::endl;
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
      std::reverse(mNodePathList.back().begin(), mNodePathList.back().end());
    }
  }

  NodePathList mNodePathList;
};

NodePathList
Node::getNodePathList() const
{
  NodePathListCollectVisitor visitor;
  visitor.apply(*this);
  return visitor.mNodePathList;
}

void
Node::addPort(Port* port)
{
  if (!port)
    return;
  port->setIndex(mPortList.size());
  mPortList.push_back(port);
}

void
Node::removePort(Port* port)
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
