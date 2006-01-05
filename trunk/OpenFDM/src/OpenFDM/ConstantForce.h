/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_ConstantForce_H
#define OpenFDM_ConstantForce_H

#include "Assert.h"
#include "Object.h"
#include "Vector.h"
#include "Frame.h"
#include "Force.h"

namespace OpenFDM {

class ConstantForce
  : public ExternalForce {
public:
  ConstantForce(const std::string& name,
                const Vector6& force = Vector6::zeros());
  virtual ~ConstantForce(void);

  using ExternalForce::setForce;
};

} // namespace OpenFDM

#endif
