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
    (*i)->accept(visitor);
}

void Group::traverse(ConstNodeVisitor& visitor) const
{
  ChildList::const_iterator i;
  for (i = _childList.begin(); i != _childList.end(); ++i)
    (*i)->accept(visitor);
}

unsigned
Group::addChild(const SharedPtr<Node>& node)
{
  if (!node)
    return ~0u;
  if (!node->addParent(this))
    return ~0u;
  _childList.push_back(node);
  return _childList.size() - 1;
}

bool
Group::removeChild(const Node* node)
{
  ChildList::iterator i;
  i = std::find(_childList.begin(), _childList.end(), node);
  if (i == _childList.end())
    return false;
  _childList.erase(i);
  return true;
}

unsigned
Group::getNumChildren() const
{
  return _childList.size();
}

unsigned
Group::getChildNumber(const Node* node) const
{
  ChildList::const_iterator i;
  i = std::find(_childList.begin(), _childList.end(), node);
  if (i == _childList.end())
    return ~0u;
  return std::distance(_childList.begin(), i);
}

SharedPtr<Node>
Group::getChild(unsigned i)
{
  if (_childList.size() <= i)
    return 0;
  return _childList[i];
}

SharedPtr<const Node>
Group::getChild(unsigned i) const
{
  if (_childList.size() <= i)
    return 0;
  return _childList[i];
}

} // namespace OpenFDM
