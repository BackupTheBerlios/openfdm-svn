/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "Connect.h"
#include "Group.h"
#include "LogStream.h"

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

SharedPtr<const Port>
Connect::getPort0() const
{
  return mPort0.lock();
}

bool
Connect::setPort0(const Port* portInfo0)
{
  SharedPtr<const Port> portInfo1 = mPort1.lock();
  if (!isCompatible(portInfo0, portInfo1)) {
    Log(Model, Warning) << "Trying to connect incompatible ports" << std::endl;
    return false;
  }
  mPort0 = portInfo0;
  return true;
}

SharedPtr<const Port>
Connect::getPort1() const
{
  return mPort1.lock();
}

bool
Connect::setPort1(const Port* portInfo1)
{
  SharedPtr<const Port> portInfo0 = mPort0.lock();
  if (!isCompatible(portInfo0, portInfo1)) {
    Log(Model, Warning) << "Trying to connect incompatible ports" << std::endl;
    return false;
  }
  mPort1 = portInfo1;
  return true;
}

bool
Connect::isCompatible(const Port* portInfo0, const Port* portInfo1) const
{
  if (!portInfo0)
    return true;
  if (!portInfo1)
    return true;
  if (!isInGroup(*portInfo0)) {
    Log(Model, Warning) << "Port is in different group than model" << std::endl;
    return false;
  }
  if (!isInGroup(*portInfo1)) {
    Log(Model, Warning) << "Port is in different group than model" << std::endl;
    return false;
  }
  // Just a crude first time check if this will work in principle.
  return portInfo0->canConnect(*portInfo1);
}

bool
Connect::isInGroup(const Port& portInfo) const
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
