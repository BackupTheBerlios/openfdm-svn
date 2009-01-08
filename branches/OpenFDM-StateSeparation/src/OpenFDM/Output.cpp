/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "Output.h"

#include "ConstNodeVisitor.h"
#include "NodeVisitor.h"
#include "PortValueList.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Output, Model)
  DEF_OPENFDM_PROPERTY(Real, OutputGain, Serialized)
  DEF_OPENFDM_PROPERTY(String, OutputName, Serialized)
  END_OPENFDM_OBJECT_DEF

Output::Callback::~Callback()
{
}

Output::Output(const std::string& name, Output::Callback* callback) :
  Model(name),
  mInputPort(newRealInputPort("input", true)),
  mCallback(callback),
  mOutputGain(1)
{
}

Output::~Output(void)
{
}

void
Output::accept(NodeVisitor& visitor)
{
  visitor.handleNodePathAndApply(this);
}

void
Output::accept(ConstNodeVisitor& visitor) const
{
  visitor.handleNodePathAndApply(this);
}

void
Output::output(const Task&,const DiscreteStateValueVector&,
               const ContinousStateValueVector&,
               PortValueList& portValues) const
{
  if (!mCallback)
    return;
  mCallback->setValue(mOutputGain*portValues[mInputPort]);
}

Output::Callback*
Output::getCallback(void) const
{
  return mCallback;
}

void
Output::setCallback(Output::Callback* callback)
{
  mCallback = callback;
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
