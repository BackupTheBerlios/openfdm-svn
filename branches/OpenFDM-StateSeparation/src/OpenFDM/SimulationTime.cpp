/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "SimulationTime.h"

#include "Task.h"
#include "TypeInfo.h"
#include "Variant.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(SimulationTime, Model)
  END_OPENFDM_OBJECT_DEF

SimulationTime::SimulationTime(const std::string& name) :
  Model(name),
  mOutputPort(newRealOutputPort("output"))
{
}

SimulationTime::~SimulationTime(void)
{
}

void
SimulationTime::output(const Task& task, const DiscreteStateValueVector&,
                       const ContinousStateValueVector&,
                       PortValueList& portValues) const
{
  portValues[mOutputPort] = task.getTime();
}

} // namespace OpenFDM
