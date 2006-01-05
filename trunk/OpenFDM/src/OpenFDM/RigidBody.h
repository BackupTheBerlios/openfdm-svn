/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_RigidBody_H
#define OpenFDM_RigidBody_H

#include <string>
#include <vector>

#include "Object.h"
#include "Vector.h"
#include "Inertia.h"
#include "Frame.h"

namespace OpenFDM {

class Interact;
class MultiBodySystem;

// Rename to Body???
class RigidBody :
    public Object {
public:
  /// Constructor
  RigidBody(const std::string& name);
  /// Destructor
  virtual ~RigidBody(void);


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

  /// Apply a force to that rigid body
  void applyForce(const Vector6& force)
  { mArtForce -= force; }

  /// Contribute articulated force as used in the articulated body algorithm
  void contributeForce(const Vector6& force)
  { mArtForce += force; }

  /// Contribute articulated inertia as used in the articulated body algorithm
  void contributeInertia(const SpatialInertia& inertia)
  { mArtInertia += inertia; }

  /// Contribute the articulated inertia and force of a local Mass object
  void contributeLocalInertia(const SpatialInertia& inertia)
  {
    Vector6 iv = getFrame()->getSpVel();
    Vector6 Jiv = inertia*iv;
    mArtForce += Vector6(cross(iv.getAngular(), Jiv.getAngular()) +
                         cross(iv.getLinear(), Jiv.getLinear()),
                         cross(iv.getAngular(), Jiv.getLinear()));
    mArtInertia += inertia;
  }

  /// Introduce an interface routine
  void setFrame(Frame* frame)
  {
    // take over all children
    frame->reparentChildren(mFrame);
    mFrame = frame;
  }
  Frame* getFrame(void)
  { return mFrame; }
  const Frame* getFrame(void) const
  { return mFrame; }

  void setParentMultiBodySystem(MultiBodySystem* multiBodySystem);
  MultiBodySystem* getParentMultiBodySystem(void);

  bool setInboardJoint(Joint* joint);
  bool addInteract(Interact* interact);
  bool removeInteract(Interact* interact);

private:
  /// Outboard articulated inertia
  SpatialInertia mArtInertia;

  /// Outboard articulated force
  Vector6 mArtForce;

  /// Frame attached to this rigid body
  SharedPtr<Frame> mFrame;

  /// One of our interacts is special ...
  /// FIXME Make that dynamic later, for now we need to know the tree root at
  /// MultiBodySystem build time
  SharedPtr<Joint> mInboardJoint;
  /// All Interacts attached to this RigidBody
  typedef std::vector<SharedPtr<Interact> > InteractList;
  InteractList mInteracts;

  WeakPtr<MultiBodySystem> mParentMultiBodySystem;

  // HMM ... FIXME
  friend class Interact;
};

} // namespace OpenFDM

#endif
