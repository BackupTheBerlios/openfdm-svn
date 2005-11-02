/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "Model.h"
#include "Vector.h"
#include "LineForce.h"

namespace OpenFDM {

LineForce::LineForce(const std::string& name) :
  Model(name)
{
}

LineForce::~LineForce(void)
{
}

void
LineForce::computeForce(real_type position, real_type vel)
{
  // Set the position and velocity
  mPosition = position;
  mVel = vel;
  // call the output function
  TaskInfo taskInfo;
  taskInfo.addSampleTime(SampleTime::Continous);
  output(taskInfo);
}

} // namespace OpenFDM
