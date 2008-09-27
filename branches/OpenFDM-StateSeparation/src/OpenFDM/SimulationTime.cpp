/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "SimulationTime.h"
#include "ModelVisitor.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(SimulationTime, Model)
  END_OPENFDM_OBJECT_DEF

SimulationTime::SimulationTime(const std::string& name) :
  Model(name)
{
  setNumOutputPorts(1);
  setOutputPort(0, "output", this, &SimulationTime::getOutputValue);
}

SimulationTime::~SimulationTime(void)
{
}

void
SimulationTime::accept(ModelVisitor& visitor)
{
  visitor.handleNodePathAndApply(*this);
}

bool
SimulationTime::init()
{
  return Model::init();
}

void
SimulationTime::output(const TaskInfo& taskInfo)
{
  mOutputValue = taskInfo.getTime();
}

const real_type&
SimulationTime::getOutputValue(void) const
{
  return mOutputValue;
}

} // namespace OpenFDM
