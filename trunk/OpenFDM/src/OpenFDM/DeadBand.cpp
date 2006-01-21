/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
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

BEGIN_OPENFDM_OBJECT_DEF(DeadBand)
  END_OPENFDM_OBJECT_DEF

DeadBand::DeadBand(const std::string& name) : Model(name)
{
  setDirectFeedThrough(true);
  
  setNumInputPorts(1);
  setInputPortName(0, "input");
  
  setNumOutputPorts(1);
  setOutputPort(0, "output", this, &DeadBand::getOutput);
  
  addStoredProperty("width", Property(this, &DeadBand::getWidth, &DeadBand::setWidth));
}

DeadBand::~DeadBand(void)
{
}
  
bool
DeadBand::init(void)
{
  mInputPort = getInputPort(0)->toRealPortHandle();
  if (!mInputPort.isConnected()) {
    Log(Model, Error) << "Initialization of DeadBand model \"" << getName()
                      << "\" failed: Input port \"" << getInputPortName(0)
                      << "\" is not connected!" << endl;
    return false;
  }

  return true;
}

void
DeadBand::output(const TaskInfo&)
{
  OpenFDMAssert(mInputPort.isConnected());
  
  mOutput = mInputPort.getRealValue();
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

const real_type&
DeadBand::getOutput(void) const
{
  return mOutput;
}

} // namespace OpenFDM
