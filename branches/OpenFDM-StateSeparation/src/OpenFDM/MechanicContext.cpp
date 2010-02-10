/* -*-c++-*- OpenFDM - Copyright (C) 2007-2010 Mathias Froehlich 
 *
 */

#include "MechanicContext.h"

namespace OpenFDM {

MechanicContext::MechanicContext(const Environment* environment) :
  mEnvironment(environment)
{
  OpenFDMAssert(mEnvironment);
}

MechanicContext::~MechanicContext()
{
}

void
MechanicContext::initDesignPosition()
{
}

void
MechanicContext::initVelocities(const /*Init*/Task& task)
{
  init(task);
  velocities(task);
}

void
MechanicContext::init(const /*Init*/Task& task)
{
}

void
MechanicContext::velocities(const Task& task)
{
}

void
MechanicContext::articulation(const Task& task)
{
}

void
MechanicContext::accelerations(const Task& task)
{
}

void
MechanicContext::derivative(const Task&)
{
}

void
MechanicContext::update(const DiscreteTask& discreteTask)
{
}

} // namespace OpenFDM
