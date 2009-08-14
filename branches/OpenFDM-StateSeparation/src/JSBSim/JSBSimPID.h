/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_JSBSimPID_H
#define OpenFDM_JSBSimPID_H

#include "JSBSimFCSComponent.h"
#include <OpenFDM/Connect.h>
#include <OpenFDM/ConstModel.h>
#include <OpenFDM/GroupInput.h>
#include <OpenFDM/Product.h>
#include <OpenFDM/SharedPtr.h>

namespace OpenFDM {

/// Just a small container mapping the JSBSim PID parameters to
/// the OpenFDM models.
class JSBSimPID : public JSBSimFCSComponent {
public:
  JSBSimPID(const std::string& name);
  virtual ~JSBSimPID(void);

  void setKI(real_type ki);
  const Port* getKIPort();

  void setKP(real_type kp);
  const Port* getKPPort();

  void setKD(real_type kd);
  const Port* getKDPort();

private:
  SharedPtr<ConstModel> mKIConstant;
  SharedPtr<GroupInput> mKIGroupInput;
  SharedPtr<Connect> mKIConnect;
  SharedPtr<Product> mKIProduct;

  SharedPtr<ConstModel> mKPConstant;
  SharedPtr<GroupInput> mKPGroupInput;
  SharedPtr<Connect> mKPConnect;
  SharedPtr<Product> mKPProduct;

  SharedPtr<ConstModel> mKDConstant;
  SharedPtr<GroupInput> mKDGroupInput;
  SharedPtr<Connect> mKDConnect;
  SharedPtr<Product> mKDProduct;
};

} //namespace OpenFDM

#endif
