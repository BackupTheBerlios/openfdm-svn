/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_DirectForce_H
#define OpenFDM_DirectForce_H

#include "Assert.h"
#include "Object.h"
#include "Vector.h"
#include "Frame.h"
#include "Force.h"

namespace OpenFDM {

class DirectForce
  : public ExternalForce {
public:
  DirectForce(const std::string& name,
              const Vector6& direction = Vector6::unit(4));
  virtual ~DirectForce(void);

  void setPosition(const Vector3& p);
  const Vector3& getPosition(void) const;

  void setOrientation(const Quaternion& o);
  const Quaternion& getOrientation(void) const;
  const Rotation& getRotation(void) const;

  void setDirection(const Vector6& direction);
  const Vector6& getDirection(void) const;

  real_type getMagnitude(void) const;
  const Vector6& getForce(void) const;

  virtual bool init(void);
  virtual void output(const TaskInfo&);

protected:
  /**
   */
  virtual void computeForce(void);

private:
  Vector3 mPosition;
  Rotation mOrientation;
  Vector6 mDirection;
  Vector6 mForce;
  real_type mMagnitude;
};

} // namespace OpenFDM

#endif
