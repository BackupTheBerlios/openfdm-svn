/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Contact_H
#define OpenFDM_Contact_H

#include "SingleLinkInteract.h"

namespace OpenFDM {

class Contact : public SingleLinkInteract {
  OPENFDM_OBJECT(Contact, SingleLinkInteract);
  class Context;
public:
  Contact(const std::string& name);
  virtual ~Contact(void);

  virtual MechanicContext*
  newMechanicContext(const Environment*, PortValueList&) const;

  // Compute the plane normal force.
  virtual real_type
  computeNormalForce(real_type compressLen, real_type compressVel) const;

  // Compute the friction force.
  virtual Vector3
  computeFrictionForce(real_type normForce, const Vector3& vel,
                       const Vector3& groundNormal, real_type friction) const;
};

} // namespace OpenFDM

#endif
