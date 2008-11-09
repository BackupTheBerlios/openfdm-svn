/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#include "MechanicInstance.h"

namespace OpenFDM {

MechanicInstance::MechanicInstance(const NodePath& nodePath,
                                   const SampleTime& sampleTime,
                                   const MechanicNode* mechanicNode) :
  AbstractNodeInstance(nodePath, sampleTime),
  mMechanicContext(mechanicNode->newMechanicContext())
{
}

MechanicInstance::~MechanicInstance()
{
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
