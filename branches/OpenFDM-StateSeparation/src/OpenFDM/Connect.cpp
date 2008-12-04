/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "Connect.h"
#include "Group.h"

namespace OpenFDM {

Connect::Connect(const Group* group) :
  mGroup(group)
{
}

Connect::~Connect()
{
}

SharedPtr<const Group>
Connect::getGroup() const
{
  return mGroup.lock();
}

SharedPtr<const PortInfo>
Connect::getPortInfo0() const
{
  return mPortInfo0.lock();
}

bool
Connect::setPortInfo0(const PortInfo* portInfo0)
{
  SharedPtr<const PortInfo> portInfo1 = mPortInfo1.lock();
  if (!isCompatible(portInfo0, portInfo1))
    return false;
  mPortInfo0 = portInfo0;
  return true;
}

SharedPtr<const PortInfo>
Connect::getPortInfo1() const
{
  return mPortInfo1.lock();
}

bool
Connect::setPortInfo1(const PortInfo* portInfo1)
{
  SharedPtr<const PortInfo> portInfo0 = mPortInfo0.lock();
  if (!isCompatible(portInfo0, portInfo1))
    return false;
  mPortInfo1 = portInfo1;
  return true;
}

bool
Connect::isCompatible(const PortInfo* portInfo0, const PortInfo* portInfo1) const
{
  if (!portInfo0)
    return true;
  if (!portInfo1)
    return true;
  if (!isInGroup(*portInfo0))
    return false;
  if (!isInGroup(*portInfo1))
    return false;
  // Just a crude first time check if this will work in principle.
  return portInfo0->canConnect(*portInfo1);
}

bool
Connect::isInGroup(const PortInfo& portInfo) const
{
  SharedPtr<const Node> node = portInfo.getNode();
  if (!node)
    return false;
  SharedPtr<const Group> group = getGroup();
  if (!group)
    return false;
  if (!node->isChildOf(group))
    return false;
  return true;
}

} // namespace OpenFDM
