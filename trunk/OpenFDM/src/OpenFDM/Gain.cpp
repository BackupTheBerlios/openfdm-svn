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
#include "Gain.h"

namespace OpenFDM {

Gain::Gain(const std::string& name) :
  Model(name)
{
  setDirectFeedThrough(true);
  
  setNumInputPorts(1);
  setInputPortName(0, "input");
  
  setNumOutputPorts(1);
  setOutputPort(0, "output", Property(this, &Gain::getOutput));
  addProperty("output", Property(this, &Gain::getOutput));
  
  addProperty("gain", Property(this, &Gain::getGain, &Gain::setGain));
}

Gain::~Gain(void)
{
}
  
bool
Gain::init(void)
{
  OpenFDMAssert(getInputPort(0).isValid());
  
  // Make sure it is invalid if sizes do not match.
  mOutput.resize(getInputPort(0).getValue().toMatrix());

  return true;
}

void Gain::output(const TaskInfo&)
{
  OpenFDMAssert(getInputPort(0).isValid());
  mOutput = getInputPort(0).getValue().toMatrix();
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
