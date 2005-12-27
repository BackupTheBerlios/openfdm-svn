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

  /// FIXME: joint's should be lockable, which means trylock == true and
  /// velocity small enough - keep position ...

  RigidBody* getOutboardBody(void)
  { return getParentRigidBody(0); }
  RigidBody* getInboardBody(void)
  { return getParentRigidBody(1); }

  virtual void interactWith(RigidBody* rigidBody)
  {
    // HMmMm
    if (rigidBody != getInboardBody())
      return;

    getOutboardBody()->computeArtValues();
    SpatialInertia artI = SpatialInertia::zeros();
    Vector6 artF = Vector6::zeros();
    contributeArticulation(artI, artF);
    rigidBody->contributeForce(artF);
    rigidBody->contributeInertia(artI);
  }

  bool contributeArticulation(SpatialInertia& artI, Vector6& artF)
  {
    RigidBody* outboardBody = getOutboardBody();
    if (!outboardBody)
      return false;

    Frame* frame = outboardBody->getFrame();

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
    artI += frame->inertiaToParent(I);
    artF += frame->forceToParent(F);

    return true;
  }

  // Joint slot ...
  // FIXME: pure virtual
  virtual bool jointArticulation(SpatialInertia& artI, Vector6& artF) = 0;
  virtual Vector6 computeRelAccel(const SpatialInertia& artI,
                                  const Vector6& artF) = 0;

  //???
  bool updateAccels(RigidBody* rigidBody)
  {
    RigidBody* outboardBody = getOutboardBody();
    if (!outboardBody)
      return false;

    if (outboardBody == rigidBody)
      return false;

    // Set the local acceleration
    Vector6 accel = computeRelAccel(outboardBody->getArtInertia(),
                                    outboardBody->getArtForce());
    
    Frame* frame0 = outboardBody->getFrame();
    if (!frame0)
      return false;
    FreeFrame* frame = dynamic_cast<FreeFrame*>(frame0);
    if (!frame)
      return false;

    frame->enableAccel();
    frame->setRelAccel(accel);

    outboardBody->computeAccel();

    return true;
  }
};

} // namespace OpenFDM

#endif
