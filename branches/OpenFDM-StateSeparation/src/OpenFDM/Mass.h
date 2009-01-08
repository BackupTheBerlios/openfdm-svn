/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Mass_H
#define OpenFDM_Mass_H

#include "SingleLinkInteract.h"
#include "Inertia.h"

namespace OpenFDM {

class Mass : public SingleLinkInteract {
  OPENFDM_OBJECT(Mass, SingleLinkInteract);
  class Context;
public:
  Mass(const std::string& name, const real_type& mass = 0,
       const InertiaMatrix& inertia = InertiaMatrix(0, 0, 0, 0, 0, 0),
       const Vector3& position = Vector3::zeros());
  virtual ~Mass(void);

  virtual MechanicContext*
  newMechanicContext(const Environment*, PortValueList&) const;

  const InertiaMatrix& getInertia(void) const;
  void setInertia(const InertiaMatrix& inertia);

  /** Set the local spatial inertia.
      @param mass mass in kg.
      Sets the spatial inertia of the current rigid body to a simple
      point mass of the mass given in the argument.
      The rotational inertia matrix is set to zero in this case.
      Be careful with such simple point masses in case of rotational motion.
      The spatial inertia matrix of such a body is singular in this case.
      With such a singular matrix there is no way to simulate rotational
      motion.
   */
  const real_type& getMass() const;
  void setMass(const real_type& mass);

private:
  /// The paremeters that can be set from outside
  real_type mMass;
  InertiaMatrix mInertia;
};

} // namespace OpenFDM

#endif
