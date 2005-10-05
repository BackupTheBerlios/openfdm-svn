/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include <string>
#include <vector>

#include "Assert.h"
#include "Object.h"
#include "Property.h"
#include "Vector.h"
#include "Model.h"
#include "DeadBand.h"

namespace OpenFDM {

DeadBand::DeadBand(const std::string& name) : Model(name)
{
  setDirectFeedThrough(true);
  
  setNumInputPorts(1);
  setInputPortName(0, "input");
  
  setNumOutputPorts(1);
  setOutputPort(0, "output", Property(this, &DeadBand::getOutput));
  
  addProperty("width", Property(this, &DeadBand::getWidth, &DeadBand::setWidth));
}

DeadBand::~DeadBand(void)
{
}
  
bool
DeadBand::init(void)
{
  OpenFDMAssert(getInputPort(0).isValid());
  return getInputPort(0).isValid();
}

void
DeadBand::output(void)
{
  OpenFDMAssert(getInputPort(0).isValid());
  
  mOutput = getInputPort(0).getValue().toReal();
  if (mOutput < -mWidth)
    mOutput += mWidth;
  else if (mWidth < mOutput)
    mOutput -= mWidth;
  else
    mOutput = 0;
}

const real_type&
DeadBand::getWidth(void) const
{
  return mWidth;
}

void
DeadBand::setWidth(const real_type& width)
{
  mWidth = width;
}

real_type
DeadBand::getOutput(void) const
{
  return mOutput;
}

} // namespace OpenFDM
