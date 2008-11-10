/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#include "AbstractNodeInstance.h"

#include "Assert.h"
#include "Node.h"
#include "SharedPtr.h"
#include "WeakReferenced.h"

namespace OpenFDM {

AbstractNodeInstance::AbstractNodeInstance(const SampleTime& sampleTime) :
  mSampleTime(sampleTime)
{
}

AbstractNodeInstance::~AbstractNodeInstance()
{
}

} // namespace OpenFDM
