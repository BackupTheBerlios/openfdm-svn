/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#include "AcceptorPortInfo.h"

namespace OpenFDM {

AcceptorPortInfo::AcceptorPortInfo(Node* node, const std::string& name) :
  PortInfo(node, name)
{
}

AcceptorPortInfo::~AcceptorPortInfo()
{
}

const AcceptorPortInfo*
AcceptorPortInfo::toAcceptorPortInfo() const
{
  return this;
}

} // namespace OpenFDM
