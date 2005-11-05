/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_RigidBody_H
#define OpenFDM_RigidBody_H

#include <iosfwd>
#include <list>
#include <string>

#include "Assert.h"
#include "Object.h"
#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "Inertia.h"
#include "Frame.h"

namespace OpenFDM {

// Rename to Body???
class RigidBody :
    public Frame {
public:
  // HMM ... FIXME
  friend class Joint;
  friend class FreeJoint;

  /** Constructor.
   */
  RigidBody(const std::string& name = std::string());
  /** Destructor.
   */
  virtual ~RigidBody(void);

  virtual RigidBody* toRigidBody(void);
  virtual const RigidBody* toRigidBody(void) const;

  virtual void accept(Visitor& visitor);
  virtual void accept(ConstVisitor& visitor) const;

// protected:
  /** Compute articulated values outboard including this body.
      That function is part of the articulated body algorithm.
      It is used to compute the articulated force and articulated inertia
      of the tree of rigid bodies mounted below the current one.
      These articulated values (inertia and force) is then stored in the
      rigid body class and can be queried for computation of the inboard
      articulated values as well as for the computations of the relative
      accelerations in the next step.
      @see const SpatialInertia& getArtInertia(void) const
      @see const Vector6& getArtForce(void) const
   */
  void computeArtValues(void);

  void computeAccel(void);

  /** Get the outboard articulated inertia.
      Returns a reference to the outboard articulated inertia.
      The returned inertia matrix is computed prevously by a call to
      @see computeArtValues(void).
   */
  const SpatialInertia& getArtInertia(void) const
  { return mArtInertia; }

  /** Get the outboard articulated force.
      Returns a reference to the outboard articulated force.
      The returned force vector is computed prevously by a call to
      @see computeArtValues(void).
   */
  const Vector6& getArtForce(void) const
  { return mArtForce; }

  /** HMM
   */
  Vector6 getPAlpha(void) const
  { return mArtForce + mArtInertia*getHdot(); }

  /**
   */
  void contributeForce(const Vector6& force)
  { mArtForce += force; }

  /**
   */
  void contributeInertia(const SpatialInertia& inertia)
  { mArtInertia += inertia; }

private:
  /** Outboard articulated inertia.
   */
  SpatialInertia mArtInertia;

  /** Outboard articulated force.
   */
  Vector6 mArtForce;
};

} // namespace OpenFDM

#endif
