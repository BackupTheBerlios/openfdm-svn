/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
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

  void setDirection(const Vector6& direction);
  const Vector6& getDirection(void) const;

  real_type getMagnitude(void) const;

  virtual bool init(void);
  virtual void output(const TaskInfo&);

private:
  Vector6 mDirection;
  real_type mMagnitude;
};

} // namespace OpenFDM

#endif
