/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#include "JSBSimActuator.h"

#include <OpenFDM/DiscreteIntegrator.h>
#include <OpenFDM/Group.h>
#include <OpenFDM/Product.h>
#include <OpenFDM/Summer.h>
#include <OpenFDM/TimeDerivative.h>

namespace OpenFDM {

JSBSimActuator::JSBSimActuator(const std::string& name) :
  JSBSimFCSComponent(name)
{
  // The error input of the JSBSim Actuator controller
  mGroupInput = new GroupInput("Input");
  getGroup()->addChild(mGroupInput);

  // That single output port is this one
  getGroup()->connect(getInternalOutputPort(),
                      mGroupInput->getPort("output"));
}

JSBSimActuator::~JSBSimActuator(void)
{
}

const Port*
JSBSimActuator::getInputPort()
{
  return getGroup()->getPort(mGroupInput->getExternalPortIndex());
}

} //namespace OpenFDM
