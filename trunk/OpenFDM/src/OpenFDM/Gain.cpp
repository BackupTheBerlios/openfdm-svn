/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "Gain.h"

#include <string>
#include <vector>

#include "Assert.h"
#include "Object.h"
#include "Vector.h"
#include "Model.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Gain, Model)
  DEF_OPENFDM_PROPERTY(Real, Gain, Serialized)
  END_OPENFDM_OBJECT_DEF

Gain::Gain(const std::string& name) :
  Model(name),
  mGain(1)
{
  setDirectFeedThrough(true);
  
  setNumInputPorts(1);
  setInputPortName(0, "input");
  
  setNumOutputPorts(1);
  setOutputPort(0, "output", this, &Gain::getOutput);
}

Gain::~Gain(void)
{
}
  
bool
Gain::init(void)
{
  mInputPort = getInputPort(0)->toMatrixPortHandle();
  if (!mInputPort.isConnected()) {
    Log(Model, Error) << "Initialization of Gain model \"" << getName()
                      << "\" failed: Input port \"" << getInputPortName(0)
                      << "\" is not connected!" << endl;
    return false;
  }
  mOutput.resize(mInputPort.getMatrixValue());

  return Model::init();
}

void Gain::output(const TaskInfo&)
{
  OpenFDMAssert(mInputPort.isConnected());
  mOutput = mInputPort.getMatrixValue();
  mOutput *= mGain;
}

const real_type&
Gain::getGain(void) const
{
  return mGain;
}

void
Gain::setGain(const real_type& gain)
{
  mGain = gain;
}

const Matrix&
Gain::getOutput(void) const
{
  return mOutput;
}

} // namespace OpenFDM
