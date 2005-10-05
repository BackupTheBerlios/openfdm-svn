/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "Input.h"

namespace OpenFDM {

Input::Input(const std::string& name) :
  Model(name),
  mInputGain(1)
{
  setNumOutputPorts(1);
  setOutputPort(0, "output", Property(this, &Input::getOutputValue));

  addProperty("inputValue",
              Property(this, &Input::getInputValue, &Input::setInputValue));
  addProperty("inputGain",
              Property(this, &Input::getInputGain, &Input::setInputGain));
  addProperty("inputName",
              Property(this, &Input::getInputName, &Input::setInputName));
}

Input::~Input(void)
{
}

bool
Input::init(void)
{
  return true;
}

void
Input::output(void)
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









Output::Output(const std::string& name) :
  Model(name),
  mOutputGain(1)
{
  setNumInputPorts(1);
  setInputPortName(0, "input");

  addProperty("value", Property(this, &Output::getValue));
  addProperty("outputGain",
              Property(this, &Output::getOutputGain, &Output::setOutputGain));
  addProperty("outputName",
              Property(this, &Output::getOutputName, &Output::setOutputName));
}

Output::~Output(void)
{
}

bool
Output::init(void)
{
  return true;
}

void
Output::output(void)
{
  mValue = mOutputGain*getInputPort(0).getValue().toReal();
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
