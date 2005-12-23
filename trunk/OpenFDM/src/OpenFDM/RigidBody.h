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

class Interact;

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
  virtual void traverse(Visitor& visitor);
  virtual void traverse(ConstVisitor& visitor) const;


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


  /** Introduce an interface routine
   */
  Frame* getFrame(void)
  { return this; }
  const Frame* getFrame(void) const
  { return this; }

  /// FIXME remove
  virtual bool addInteract2(Interact* child, unsigned parentNum = 0);

private:
  void addInteract(Interact* interact);
  bool removeInteract(Interact* interact);

  /** Outboard articulated inertia.
   */
  SpatialInertia mArtInertia;

  /** Outboard articulated force.
   */
  Vector6 mArtForce;

  /// Frame attached to this rigid body
//   SharedPtr<Frame> mFrame;

  typedef std::vector<SharedPtr<Interact> > InteractList;
  InteractList mInteracts;

  /// FIXME: is interact too???
  typedef std::vector<SharedPtr<Mass> > MassList;
  MassList mMasses;

  friend class Interact;
};

class Interact :
    public Model {
public:
  Interact(const std::string& name, unsigned numParents) :
    Model(name), mParents(numParents) { }
  virtual ~Interact(void) { }


  /// Double dispatch helper for the multibody system visitor
  virtual void accept(Visitor& visitor);
  /// Double dispatch helper for the multibody system visitor
  virtual void traverse(Visitor& visitor)
  { }
  /// Double dispatch helper for the multibody system visitor
  virtual void accept(ConstVisitor& visitor) const;
  /// Double dispatch helper for the multibody system visitor
  virtual void traverse(ConstVisitor& visitor) const
  { }


  bool attachTo(RigidBody* rigidBody)
  {
    if (!rigidBody) {
      Log(MultiBody,Error) << "Got 0 RigidBody pointer to attach to." << endl;
      return false;
    }
    ParentList::iterator it;
    for (it = mParents.begin(); it != mParents.end(); ++it) {
      if (!(*it)) {
        (*it) = rigidBody;
        (*it)->addInteract(this);
        return true;
      }
    }

    Log(MultiBody,Error) << "Cannot attach Interact \"" << getName()
                         << "\" to RigidBody \"" << rigidBody->getName()
                         << "\": Already attached to 2 Rigid bodies."
                         << endl;
    return false;
  }
  bool detachFrom(RigidBody* rigidBody)
  {
    if (!rigidBody) {
      Log(MultiBody,Error) << "Got 0 RigidBody pointer to attach to." << endl;
      return false;
    }
    ParentList::iterator it;
    for (it = mParents.begin(); it != mParents.end(); ++it) {
      if ((*it) == rigidBody) {
        (*it)->removeInteract(this);
        (*it) = 0;
        return true;
      }
    }

    Log(MultiBody,Error) << "Cannot detatach Interact \"" << getName()
                         << "\" from RigidBody \"" << rigidBody->getName()
                         << "\": Interact is not attached to that RigidBody."
                         << endl;
    return false;
  }

  virtual void interactWith(RigidBody* rigidBody) = 0;

  /// FIXME remove
  const Frame* getParentFrame(unsigned id = 0) const
  {
    OpenFDMAssert(id < mParents.size() && mParents[id]);
    return mParents[id]->getFrame();
  }
  /// FIXME remove
  Frame* getParentFrame(unsigned id = 0)
  {
    OpenFDMAssert(id < mParents.size() && mParents[id]);
    return mParents[id]->getFrame();
  }

private:
  typedef std::vector<WeakPtr<RigidBody> > ParentList;
  ParentList mParents;
};

class Joint2 :
    public Interact {
public:
  Joint2(const std::string& name) : Interact(name, 2) { }
  virtual ~Joint2(void) { }

};

} // namespace OpenFDM

#endif
