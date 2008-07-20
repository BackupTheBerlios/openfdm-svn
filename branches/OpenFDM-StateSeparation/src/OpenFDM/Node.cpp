/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "Node.h"
#include "NodeVisitor.h"

namespace OpenFDM {

Node::Node(const std::string& name) :
  Object(name)
{
}

Node::~Node()
{
}

void
Node::accept(NodeVisitor& visitor)
{
  visitor.apply(*this);
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
