/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "Input.h"
#include "ModelVisitor.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Input, Model)
  DEF_OPENFDM_PROPERTY(Real, InputGain, Serialized)
  DEF_OPENFDM_PROPERTY(String, InputName, Serialized)
  END_OPENFDM_OBJECT_DEF

Input::Callback::~Callback()
{
}

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

void
Input::accept(ModelVisitor& visitor)
{
  visitor.apply(*this);
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
  if (!mCallback) {
    Log(Model, Error) << "Initialization of Input model \"" << getName()
                      << "\" failed: Input Callback not set!" << endl;
    return false;
  }

  return Model::init();
}

void
Input::output(const TaskInfo&)
{
  mOutputValue = mInputGain*mCallback->getValue();
}

Input::Callback*
Input::getCallback(void) const
{
  return mCallback;
}

void
Input::setCallback(Input::Callback* callback)
{
  mCallback = callback;
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

} // namespace OpenFDM
