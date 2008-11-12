/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Mass_H
#define OpenFDM_Mass_H

#include "Interact.h"
#include "Inertia.h"

namespace OpenFDM {

class Mass : public Interact {
  OPENFDM_OBJECT(Mass, Interact);
public:
  Mass(const std::string& name, const real_type& mass = 0,
       const InertiaMatrix& inertia = InertiaMatrix(0, 0, 0, 0, 0, 0),
       const Vector3& position = Vector3::zeros());
  virtual ~Mass(void);

  virtual void initDesignPosition(PortValueList&) const;
  virtual void articulation(const Task&, const ContinousStateValueVector&,
                            PortValueList&, Matrix&) const;

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

  /** Set a position offset for the inertia given.
   */
  const Vector3& getPosition(void) const;
  void setPosition(const Vector3& position);

private:
  MechanicLink mMechanicLink;

  /// The paremeters that can be set from outside
  real_type mMass;
  InertiaMatrix mInertia;
  Vector3 mPosition;
};

} // namespace OpenFDM

#endif
