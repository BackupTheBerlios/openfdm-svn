/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Mass_H
#define OpenFDM_Mass_H

#include "Assert.h"
#include "MultiBodyModel.h"
#include "Inertia.h"

namespace OpenFDM {

class Visitor;
class ConstVisitor;

class Mass
  : public MultiBodyModel {
  OpenFDM_NodeImplementation(1);
public:
  Mass(const SpatialInertia& inertia = SpatialInertia(0),
       const std::string& name = std::string());
  virtual ~Mass(void);

  virtual void accept(Visitor& visitor);
  virtual void accept(ConstVisitor& visitor) const;

  virtual Mass* toMass(void);
  virtual const Mass* toMass(void) const;

  const SpatialInertia& getInertia(void) const
  { return _inertia; }

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
  void setInertia(real_type mass);

  /** Set the local spatial inertia.
      @param mass mass in kg.
      @param inertia inertia matrix in kg*m^2.
      Sets the spatial inertia of the current rigid body to a simple
      point mass of the mass and inertia given in the argument.
   */
  void setInertia(real_type mass, const InertiaMatrix& inertia);

  /** Set the local spatial inertia.
   */
  void setInertia(const SpatialInertia& I);

  // More efficient getter. ??
  void contributeInertia(SpatialInertia& I) const;

private:
  SpatialInertia _inertia;
};

} // namespace OpenFDM

#endif
