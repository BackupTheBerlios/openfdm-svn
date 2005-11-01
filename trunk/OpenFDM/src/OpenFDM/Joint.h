/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Joint_H
#define OpenFDM_Joint_H

#include "Assert.h"
#include "Object.h"
#include "MultiBodyModel.h"
#include "Frame.h"
#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "Inertia.h"
#include "Visitor.h"
#include "ConstVisitor.h"
#include "Frame.h"
#include "LogStream.h"

namespace OpenFDM {

class Joint
  : public MultiBodyModel {
  OpenFDM_NodeImplementation(2);
public:
  Joint(const std::string& name)
    : MultiBodyModel(name)
  {}

  virtual void accept(Visitor& visitor)
  { visitor.apply(*this); }
  virtual void accept(ConstVisitor& visitor) const
  { visitor.apply(*this); }

  bool isArticulatedJoint(void) const
  {
    if (!getParentFrame(0))
      return false;
    if (!getParentFrame(1))
      return false;
    if (getParentFrame(1)->isParentFrame(getParentFrame(0)))
      return true;
    if (getParentFrame(0)->isParentFrame(getParentFrame(1)))
      return true;
    return false;
  }

  Frame* getInboardGroup(void)
  {
    if (!getParentFrame(0))
      return 0;
    if (!getParentFrame(1))
      return 0;
    if (getParentFrame(1)->isParentFrame(getParentFrame(0)))
      return getParentFrame(0);
    if (getParentFrame(0)->isParentFrame(getParentFrame(1)))
      return getParentFrame(1);
    return 0;
  }
  const Frame* getInboardGroup(void) const
  {
    if (!getParentFrame(0))
      return 0;
    if (!getParentFrame(1))
      return 0;
    if (getParentFrame(1)->isParentFrame(getParentFrame(0)))
      return getParentFrame(0);
    if (getParentFrame(0)->isParentFrame(getParentFrame(1)))
      return getParentFrame(1);
    return 0;
  }
  Frame* getOutboardGroup(void)
  {
    if (!getParentFrame(0))
      return 0;
    if (!getParentFrame(1))
      return 0;
    if (getParentFrame(1)->isParentFrame(getParentFrame(0)))
      return getParentFrame(1);
    if (getParentFrame(0)->isParentFrame(getParentFrame(1)))
      return getParentFrame(0);
    return 0;
  }
  const Frame* getOutboardGroup(void) const
  {
    if (!getParentFrame(0))
      return 0;
    if (!getParentFrame(1))
      return 0;
    if (getParentFrame(1)->isParentFrame(getParentFrame(0)))
      return getParentFrame(1);
    if (getParentFrame(0)->isParentFrame(getParentFrame(1)))
      return getParentFrame(0);
    return 0;
  }

  // Cast functions.
  virtual Joint* toJoint(void)
  { return this; }
  virtual const Joint* toJoint(void) const
  { return this; }

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
