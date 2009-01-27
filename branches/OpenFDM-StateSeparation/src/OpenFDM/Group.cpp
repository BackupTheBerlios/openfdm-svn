/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
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
  for (i = mChildList.begin(); i != mChildList.end(); ++i)
    (*i)->accept(visitor);
}

void Group::traverse(ConstNodeVisitor& visitor) const
{
  ChildList::const_iterator i;
  for (i = mChildList.begin(); i != mChildList.end(); ++i)
    (*i)->accept(visitor);
}

unsigned
Group::addChild(Node* node)
{
  if (!node)
    return ~0u;
  if (!node->addParent(this))
    return ~0u;
  mChildList.push_back(node);
  return mChildList.size() - 1;
}

void
Group::removeChild(const Node* node)
{
  ChildList::iterator i;
  i = std::find(mChildList.begin(), mChildList.end(), node);
  if (i == mChildList.end())
    return;
  mChildList.erase(i);
}

void
Group::removeChild(unsigned index)
{
  if (mChildList.size() <= index)
    return;
  ChildList::iterator i = mChildList.begin();
  std::advance(i, index);
  mChildList.erase(i);
}

unsigned
Group::getNumChildren() const
{
  return mChildList.size();
}

unsigned
Group::getChildIndex(const Node* node) const
{
  ChildList::const_iterator i;
  i = std::find(mChildList.begin(), mChildList.end(), node);
  if (i == mChildList.end())
    return ~0u;
  return std::distance(mChildList.begin(), i);
}

Node*
Group::getChild(unsigned i)
{
  if (mChildList.size() <= i)
    return 0;
  return mChildList[i];
}

const Node*
Group::getChild(unsigned i) const
{
  if (mChildList.size() <= i)
    return 0;
  return mChildList[i];
}

unsigned
Group::getNumConnects() const
{
  return mConnectList.size();
}

Connect*
Group::getConnect(unsigned i)
{
  if (mConnectList.size() <= i)
    return 0;
  return mConnectList[i];
}

const Connect*
Group::getConnect(unsigned i) const
{
  if (mConnectList.size() <= i)
    return 0;
  return mConnectList[i];
}

void
Group::removeConnect(unsigned index)
{
  if (mConnectList.size() <= index)
    return;
  ConnectList::iterator i = mConnectList.begin();
  std::advance(i, index);
  mConnectList.erase(i);
}

void
Group::removeConnect(const Connect* connect)
{
  ConnectList::iterator i;
  i = std::find(mConnectList.begin(), mConnectList.end(), connect);
  if (i == mConnectList.end())
    return;
  mConnectList.erase(i);
}

unsigned
Group::getConnectIndex(const Connect* connect) const
{
  ConnectList::const_iterator i;
  i = std::find(mConnectList.begin(), mConnectList.end(), connect);
  if (i == mConnectList.end())
    return ~0u;
  return std::distance(mConnectList.begin(), i);
}

Connect*
Group::connect(const Port* port0, const Port* port1)
{
  SharedPtr<Connect> connect = new Connect(this);
  if (!connect->setPort0(port0))
    return 0;
  if (!connect->setPort1(port1))
    return 0;
  mConnectList.push_back(connect);
  return connect.get();
}

} // namespace OpenFDM
