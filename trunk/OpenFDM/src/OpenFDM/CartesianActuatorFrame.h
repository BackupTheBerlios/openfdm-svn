/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_CartesianActuatorFrame_H
#define OpenFDM_CartesianActuatorFrame_H

#include "Assert.h"
#include "Vector.h"
#include "Matrix.h"
#include "Inertia.h"
#include "Frame.h"

namespace OpenFDM {

template<unsigned n>
class CartesianActuatorFrame :
  public Frame {
public:
  typedef LinAlg::Vector<real_type,n> VectorN;
  typedef LinAlg::Matrix<real_type,6,n> Matrix6N;

  CartesianActuatorFrame(const std::string& name) :
    Frame(name),
    mRelVelDot(Vector6::zeros())
  { }
  virtual ~CartesianActuatorFrame(void)
  { }

  /// The interface routine for the Frame,
  /// returns the relative velocity derivative of this frame
  virtual const Vector6& getRelVelDot(void) const
  { return mRelVelDot; }

  /// Compute the articulated force and inertia past inboard to that joint
  bool jointArticulation(SpatialInertia& artI, Vector6& artF,
                         const Vector6& outF, const SpatialInertia& outI)
  {
    // The formulas conform to Roy Featherstones book eqn (7.36), (7.37)

    // Compute the articulated force and inertia.
    // This Since there is no projection step with the joint axis, it is clear
    // that this is just a rigid connection ...
    artF = outF + outI*(getHdot() + mRelVelDot);
    artI = outI;

    return true;
  }

protected:
  void setRelVelDot(const Vector6& relVelDot)
  {
    mRelVelDot = relVelDot;
    setAccelDirty();
  }
  void setLinearRelVelDot(const Vector3& relVelDot)
  {
    mRelVelDot.setLinear(relVelDot);
    setAccelDirty();
  }
  void setAngularRelVelDot(const Vector3& relVelDot)
  {
    mRelVelDot.setAngular(relVelDot);
    setAccelDirty();
  }

private:
  /// The derivative of the frame spatial velocity
  Vector6 mRelVelDot;
};

} // namespace OpenFDM

#endif
