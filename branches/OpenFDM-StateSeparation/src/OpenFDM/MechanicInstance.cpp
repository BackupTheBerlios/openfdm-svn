/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#include "MechanicInstance.h"

namespace OpenFDM {

MechanicInstance::MechanicInstance(const NodePath& nodePath,
                                   const MechanicNode* mechanicNode) :
  AbstractNodeInstance(nodePath),
  mMechanicContext(new MechanicContext(mechanicNode))
{
}

MechanicInstance::~MechanicInstance()
{
}

bool
MechanicInstance::isConnectedTo(const MechanicInstance& mechanicInstance) const
{
  return mMechanicContext->isConnectedTo(*mechanicInstance.mMechanicContext);
}

MechanicContext&
MechanicInstance::getNodeContext()
{
  return *mMechanicContext;
}

const MechanicContext&
MechanicInstance::getNodeContext() const
{
  return *mMechanicContext;
}

} // namespace OpenFDM
