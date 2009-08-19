/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "JSBSimSwitch.h"

#include <OpenFDM/DiscreteIntegrator.h>
#include <OpenFDM/Group.h>
#include <OpenFDM/Product.h>
#include <OpenFDM/Summer.h>
#include <OpenFDM/TimeDerivative.h>

namespace OpenFDM {

JSBSimSwitch::JSBSimSwitch(const std::string& name) :
  JSBSimFCSComponent(name)
{
  // Currently a dummy implementation
  ConstModel* constModel = new ConstModel("Constant", 1);
  getGroup()->addChild(constModel);

  getGroup()->connect(getInternalOutputPort(),
                      constModel->getPort("output"));
}

JSBSimSwitch::~JSBSimSwitch(void)
{
}

} //namespace OpenFDM
