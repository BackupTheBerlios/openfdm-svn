/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_JSBSimActuator_H
#define OpenFDM_JSBSimActuator_H

#include "JSBSimFCSComponent.h"
#include <OpenFDM/Connect.h>
#include <OpenFDM/ConstModel.h>
#include <OpenFDM/GroupInput.h>
#include <OpenFDM/Product.h>
#include <OpenFDM/SharedPtr.h>

namespace OpenFDM {

/// Just a small container mapping the JSBSim Actuator parameters to
/// the OpenFDM models.
class JSBSimActuator : public JSBSimFCSComponent {
public:
  JSBSimActuator(const std::string& name);
  virtual ~JSBSimActuator(void);

  const Port* getInputPort();

private:
  SharedPtr<GroupInput> mGroupInput;
};

} //namespace OpenFDM

#endif
