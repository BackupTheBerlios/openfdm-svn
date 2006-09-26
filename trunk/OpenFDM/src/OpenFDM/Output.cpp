/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "Output.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Output, Model)
  DEF_OPENFDM_PROPERTY(Real, OutputGain, Serialized)
  DEF_OPENFDM_PROPERTY(String, OutputName, Serialized)
  END_OPENFDM_OBJECT_DEF

Output::Output(const std::string& name) :
  Model(name),
  mOutputGain(1)
{
  setDirectFeedThrough(true);
  setNumInputPorts(1);
  setInputPortName(0, "input");
}

Output::~Output(void)
{
}

bool
Output::init(void)
{
  mInputPort = getInputPort(0)->toRealPortHandle();
  if (!mInputPort.isConnected()) {
    Log(Model, Error) << "Initialization of Output model \"" << getName()
                      << "\" failed: Input port \"" << getInputPortName(0)
                      << "\" is not connected!" << endl;
    return false;
  }

  return Model::init();
}

const Output*
Output::toOutput(void) const
{
  return this;
}

Output*
Output::toOutput(void)
{
  return this;
}

void
Output::output(const TaskInfo&)
{
  mValue = mInputPort.getRealValue();
}

const real_type&
Output::getValue(void) const
{
  return mValue;
}

const real_type&
Output::getOutputGain(void) const
{
  return mOutputGain;
}

void
Output::setOutputGain(const real_type& outputGain)
{
  mOutputGain = outputGain;
}

const std::string&
Output::getOutputName(void) const
{
  return mOutputName;
}

void
Output::setOutputName(const std::string& outputName)
{
  mOutputName = outputName;
}

} // namespace OpenFDM
