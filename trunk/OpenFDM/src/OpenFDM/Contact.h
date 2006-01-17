/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Contact_H
#define OpenFDM_Contact_H

#include "Assert.h"
#include "Object.h"
#include "Vector.h"
#include "Frame.h"
#include "Force.h"
#include "Ground.h"
#include "Environment.h"

namespace OpenFDM {

class Contact : public ExternalForce {
  OPENFDM_OBJECT(Contact, ExternalForce);
public:
  Contact(const std::string& name);
  virtual ~Contact(void);

  virtual bool init(void);
  virtual void output(const TaskInfo&);

  // Compute the plane normal force.
  virtual real_type
  computeNormalForce(real_type compressLen, real_type compressVel) const;

  // Compute the friction force.
  virtual Vector3
  computeFrictionForce(real_type normForce, const Vector3& vel,
                       const Vector3& groundNormal, real_type friction) const;

private:
  void getGround(real_type t);

  bool mEnabled;
  GroundValues mGroundVal;
  SharedPtr<Environment> mEnvironment;
};

} // namespace OpenFDM

#endif
