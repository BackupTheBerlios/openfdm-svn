/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_JSBSimSwitch_H
#define OpenFDM_JSBSimSwitch_H

#include "JSBSimFCSComponent.h"
#include <OpenFDM/Connect.h>
#include <OpenFDM/ConstModel.h>
#include <OpenFDM/GroupInput.h>
#include <OpenFDM/Product.h>
#include <OpenFDM/SharedPtr.h>

namespace OpenFDM {

/// Just a small container mapping the JSBSim Switch parameters to
/// the OpenFDM models.
class JSBSimSwitch : public JSBSimFCSComponent {
public:
  JSBSimSwitch(const std::string& name);
  virtual ~JSBSimSwitch(void);
};

} //namespace OpenFDM

#endif
