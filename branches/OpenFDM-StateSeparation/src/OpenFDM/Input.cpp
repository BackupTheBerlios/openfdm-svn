/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "Input.h"

#include "ConstNodeVisitor.h"
#include "NodeVisitor.h"
#include "PortValueList.h"

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
  mOutputPort(newRealOutputPort("output")),
  mInputGain(1)
{
}

Input::~Input(void)
{
}

void
Input::accept(NodeVisitor& visitor)
{
  visitor.handleNodePathAndApply(this);
}

void
Input::accept(ConstNodeVisitor& visitor) const
{
  visitor.handleNodePathAndApply(this);
}

void
Input::output(const Task& ,const DiscreteStateValueVector&,
              const ContinousStateValueVector&,
              PortValueList& portValues) const
{
  portValues[mOutputPort] = mInputGain*mCallback->getValue();
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

} // namespace OpenFDM
