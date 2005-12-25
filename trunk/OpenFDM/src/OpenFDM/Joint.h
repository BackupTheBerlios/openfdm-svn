/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Joint_H
#define OpenFDM_Joint_H

#include "Assert.h"
#include "Object.h"
#include "Frame.h"
#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "Inertia.h"
#include "Interact.h"
#include "Frame.h"
#include "LogStream.h"

namespace OpenFDM {

class Joint
  : public Interact {
public:
  Joint(const std::string& name)
    : Interact(name, 2)
  {}

  virtual bool isArticulatedJoint(void) const
  {
    const Frame* parent0 = getParentFrame(0);
    if (!parent0)
      return false;
    const Frame* parent1 = getParentFrame(1);
    if (!parent1)
      return false;
    if (parent1->isParentFrame(parent0))
      return true;
    if (parent0->isParentFrame(parent1))
      return true;
    return false;
  }

  virtual Frame* getInboardGroup(void)
  {
    Frame* parent0 = getParentFrame(0);
    if (!parent0)
      return 0;
    Frame* parent1 = getParentFrame(1);
    if (!parent1)
      return 0;
    if (parent1->isParentFrame(parent0))
      return parent0;
    if (parent0->isParentFrame(parent1))
      return parent1;
    return 0;
  }
  virtual const Frame* getInboardGroup(void) const
  {
    const Frame* parent0 = getParentFrame(0);
    if (!parent0)
      return 0;
    const Frame* parent1 = getParentFrame(1);
    if (!parent1)
      return 0;
    if (parent1->isParentFrame(parent0))
      return parent0;
    if (parent0->isParentFrame(parent1))
      return parent1;
    return 0;
  }
  virtual Frame* getOutboardGroup(void)
  {
    Frame* parent0 = getParentFrame(0);
    if (!parent0)
      return 0;
    Frame* parent1 = getParentFrame(1);
    if (!parent1)
      return 0;
    if (parent1->isParentFrame(parent0))
      return parent1;
    if (parent0->isParentFrame(parent1))
      return parent0;
    return 0;
  }
  virtual const Frame* getOutboardGroup(void) const
  {
    const Frame* parent0 = getParentFrame(0);
    if (!parent0)
      return 0;
    const Frame* parent1 = getParentFrame(1);
    if (!parent1)
      return 0;
    if (parent1->isParentFrame(parent0))
      return parent1;
    if (parent0->isParentFrame(parent1))
      return parent0;
    return 0;
  }

  virtual void interactWith(RigidBody* rigidBody)
  {
    // HMmMm
    if (!isArticulatedJoint())
      return;

    if (rigidBody->getFrame() != getInboardGroup())
      return;

    SpatialInertia artI = SpatialInertia::zeros();
    Vector6 artF = Vector6::zeros();
    contributeArticulation(artI, artF);
    rigidBody->contributeForce(artF);
    rigidBody->contributeInertia(artI);
  }

  bool contributeArticulation(SpatialInertia& artI, Vector6& artF)
  {
    RigidBody* outboardBody = getOutboardGroup()->toRigidBody();
    if (!outboardBody)
      return false;

    Log(ArtBody, Debug) << "Contributing articulation from \""
                        << outboardBody->getName() << "\" through joint \""
                        << getName() << "\"" << endl;

    // We need the articulated inertia and force from the outboard body.
    SpatialInertia I = outboardBody->getArtInertia();
    Vector6 F = outboardBody->getArtForce();

    Log(ArtBody, Debug3) << "Outboard Articulated values: Force:\n"
                         << trans(F) << "\nInertia\n" << I << endl;

    // Apply the joint degrees of freedom to that.
    // If there was an error, (something was singular ???)
    // just ignore that part. FIXME, ist this ok????
    if (!jointArticulation(I, F))
      return false;

    Log(ArtBody, Debug3) << "Outboard Articulated values past joint "
                         << "projection: Force:\n" << trans(F)
                         << "\nInertia\n" << I << endl;

    // Contribute the transformed values to the parent.
    artI += outboardBody->inertiaToParent(I);
    artF += outboardBody->forceToParent(F);

    return true;
  }

  // Joint slot ...
  // FIXME: pure virtual
  virtual bool jointArticulation(SpatialInertia& artI, Vector6& artF) = 0;
  virtual Vector6 computeRelAccel(const SpatialInertia& artI,
                                  const Vector6& artF) = 0;

  // FIXME: just for compatibility
  Vector6 getHdot()
  {
    RigidBody* outboardBody = getOutboardGroup()->toRigidBody();
    if (!outboardBody)
      return Vector6::zeros();
    return outboardBody->getHdot();
  }

  //???
  bool updateAccels(void)
  {
    RigidBody* outboardBody = getOutboardGroup()->toRigidBody();
    if (!outboardBody)
      return false;

    // Set the local acceleration
    setOutboardRelAccel(computeRelAccel(outboardBody->getArtInertia(),
                                        outboardBody->getArtForce()));
    
    return true;
  }


  Quaternion mOrientation[2];
  Vector3 mPosition[2];

protected:
  void setOutboardState(const Vector3& pos, const Quaternion& orient,
                        const Vector6& vel)
  {
    RigidBody* outboardBody = getOutboardGroup()->toRigidBody();
    if (!outboardBody)
      return;

    outboardBody->disableAccel();
    outboardBody->setPosition(pos);
    outboardBody->setOrientation(orient);
    outboardBody->setRelVel(vel);
  }

  void setOutboardPosition(const Vector3& pos)
  {
    RigidBody* outboardBody = getOutboardGroup()->toRigidBody();
    if (!outboardBody)
      return;

    outboardBody->disableAccel();
    outboardBody->setPosition(pos);
  }
  void setOutboardOrientation(const Quaternion& orient)
  {
    RigidBody* outboardBody = getOutboardGroup()->toRigidBody();
    if (!outboardBody)
      return;

    outboardBody->disableAccel();
    outboardBody->setOrientation(orient);
  }
  void setOutboardRelVel(const Vector6& vel)
  {
    RigidBody* outboardBody = getOutboardGroup()->toRigidBody();
    if (!outboardBody)
      return;

    outboardBody->disableAccel();
    outboardBody->setRelVel(vel);
  }
  void setOutboardRelAccel(const Vector6& accel)
  {
    RigidBody* outboardBody = getOutboardGroup()->toRigidBody();
    if (!outboardBody)
      return;

    outboardBody->enableAccel();
    outboardBody->setRelAccel(accel);
  }
};

} // namespace OpenFDM

#endif
