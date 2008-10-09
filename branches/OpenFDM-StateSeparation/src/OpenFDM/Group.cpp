/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "Group.h"
#include "ConstNodeVisitor.h"
#include "NodeVisitor.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Group, Node)
  END_OPENFDM_OBJECT_DEF

Group::Group(const std::string& name) :
  Node(name)
{
}

Group::~Group()
{
}

void
Group::accept(NodeVisitor& visitor)
{
  visitor.handleNodePathAndApply(this);
}

void
Group::accept(ConstNodeVisitor& visitor) const
{
  visitor.handleNodePathAndApply(this);
}

void Group::traverse(NodeVisitor& visitor)
{
  ChildList::const_iterator i;
  for (i = _childList.begin(); i != _childList.end(); ++i)
    (*i)->node->accept(visitor);
}

void Group::traverse(ConstNodeVisitor& visitor) const
{
  ChildList::const_iterator i;
  for (i = _childList.begin(); i != _childList.end(); ++i)
    (*i)->node->accept(visitor);
}

Group::NodeId
Group::addChild(const SharedPtr<Node>& node)
{
  if (!node)
    return NodeId();
  if (!node->addParent(this))
    return NodeId();
  _childList.push_back(new Child(this, node));
  return NodeId(_childList.back());
}

unsigned
Group::getNumChildren() const
{
  return _childList.size();
}

unsigned
Group::getChildNumber(const NodeId& nodeId) const
{
  SharedPtr<Child> child = nodeId._child.lock();
  if (!child)
    return ~0u;
  if (child->group.lock() != this)
    return ~0u;
  ChildList::const_iterator i;
  i = std::find(_childList.begin(), _childList.end(), child);
  if (i == _childList.end())
    return ~0u;
  return std::distance(_childList.begin(), i);
}

SharedPtr<Node>
Group::getChild(unsigned i)
{
  if (_childList.size() <= i)
    return 0;
  return _childList[i]->node;
}

SharedPtr<const Node>
Group::getChild(unsigned i) const
{
  if (_childList.size() <= i)
    return 0;
  return _childList[i]->node;
}

SharedPtr<Node>
Group::getChild(const NodeId& nodeId)
{
  SharedPtr<Child> child = nodeId._child.lock();
  if (!child)
    return 0;
  // Check if it belongs to this current group
  if (child->group.lock() != this)
    return 0;
  return child->node;
}

SharedPtr<const Node>
Group::getChild(const NodeId& nodeId) const
{
  SharedPtr<Child> child = nodeId._child.lock();
  if (!child)
    return 0;
  // Check if it belongs to this current group
  SharedPtr<Group> group = child->group.lock();
  if (group != this)
    return 0;
  return child->node;
}

} // namespace OpenFDM
