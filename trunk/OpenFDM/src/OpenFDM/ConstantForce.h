/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
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

  void setPosition(const Vector3& p)
  { mPosition = p; }
  const Vector3& getPosition(void) const
  { return mPosition; }

  void setOrientation(const Quaternion& o)
  { mOrientation = o; }
  const Rotation& getOrientation(void) const
  { return mOrientation; }

  void setForce(const Vector6& force)
  { mForce = force; }

protected:
  /**
   */
  virtual void computeForce(void);

private:
  Vector3 mPosition;
  Rotation mOrientation;
  Vector6 mForce;
};

} // namespace OpenFDM

#endif
