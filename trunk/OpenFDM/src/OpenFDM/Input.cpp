/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "Input.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Input, Model)
  DEF_OPENFDM_PROPERTY(Real, InputGain, Serialized)
  DEF_OPENFDM_PROPERTY(String, InputName, Serialized)
  END_OPENFDM_OBJECT_DEF

Input::Input(const std::string& name) :
  Model(name),
  mInputGain(1)
{
  setNumOutputPorts(1);
  setOutputPort(0, "output", this, &Input::getOutputValue);
}

Input::~Input(void)
{
}

const Input*
Input::toInput(void) const
{
  return this;
}

Input*
Input::toInput(void)
{
  return this;
}

bool
Input::init(void)
{
  return Model::init();
}

void
Input::output(const TaskInfo&)
{
  mOutputValue = mInputValue*mInputGain;
}

const real_type&
Input::getInputValue(void) const
{
  return mInputValue;
}

void
Input::setInputValue(const real_type& value)
{
  mInputValue = value;
}

const real_type&
Input::getInputGain(void) const
{
  return mInputGain;
}

void
Input::setInputGain(const real_type& inputGain)
{
  mInputGain = inputGain;
}

const std::string&
Input::getInputName(void) const
{
  return mInputName;
}

void
Input::setInputName(const std::string& inputName)
{
  mInputName = inputName;
}

const real_type&
Input::getOutputValue(void) const
{
  return mOutputValue;
}





/// FIXME
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
  RealPortHandle rh = getInputPort(0)->toRealPortHandle();
  mValue = rh.getRealValue();
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
