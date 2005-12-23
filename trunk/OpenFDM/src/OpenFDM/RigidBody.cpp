/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "Assert.h"
#include "LogStream.h"
#include "Object.h"
#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "Inertia.h"
#include "Frame.h"
#include "RigidBody.h"
#include "Force.h"
#include "Mass.h"
#include "Joint.h"

namespace OpenFDM {

RigidBody::RigidBody(const std::string& name) :
  Frame(name)
{
}

RigidBody::~RigidBody(void)
{
}

RigidBody*
RigidBody::toRigidBody(void)
{
  return this;
}

const RigidBody*
RigidBody::toRigidBody(void) const
{
  return this;
}

void
RigidBody::accept(Visitor& visitor)
{
  visitor.apply(*this);
}

void
RigidBody::accept(ConstVisitor& visitor) const
{
  visitor.apply(*this);
}

void
RigidBody::traverse(Visitor& visitor)
{
  InteractList::iterator it;
  for (it = mInteracts.begin(); it != mInteracts.end(); ++it)
    (*it)->accept(visitor);
  Frame::traverse(visitor);
}

void
RigidBody::traverse(ConstVisitor& visitor) const
{
  InteractList::const_iterator it;
  for (it = mInteracts.begin(); it != mInteracts.end(); ++it)
    (*it)->accept(visitor);
  Frame::traverse(visitor);
}

void
RigidBody::computeArtValues(void)
{
  Log(ArtBody, Debug) << "Entry of computeArtValues of \"" << getName()
                      << "\"" << endl;

  // At first this is the inertia matrix of the current body.
  mArtInertia = SpatialInertia::zeros();
  mArtForce = Vector6::zeros();

  // Now collect all articulated forces and all articulated inertias.
  unsigned n = getNumMultiBodyModels();
  for (unsigned i = 0; i < n; ++i) {
    Joint* joint = getMultiBodyModel(i)->toJoint();
    if (joint) {
      Log(ArtBody, Debug) << "Processing joint \"" << joint->getName()
                          << "\" to body \"" << getName() << "\"" << endl;
      // Check if this is an articulated joint and if we are the parent.
      // BTW: here the recursion happens ...
      if (joint->isArticulatedJoint() && this == joint->getInboardGroup())
        joint->contributeArticulation(mArtInertia, mArtForce);
      else
        // Return some contraint force for now...
        ;
    }
  }

  // Collect all articulated forces and inertias
  InteractList::iterator it;
  for (it = mInteracts.begin(); it != mInteracts.end(); ++it)
    (*it)->interactWith(this);
  
  Log(ArtBody, Debug3) << "On exit of computeArtValues of \"" << getName()
                       << "\" Force is:\n" << trans(mArtForce)
                       << "\nInertia:\n" << mArtInertia << endl;
}

void
RigidBody::computeAccel(void)
{
  Log(ArtBody, Debug) << "Entry of computeAccel of \"" << getName()
                      << "\"" << endl;

  unsigned n = getNumMultiBodyModels();
  for (unsigned i = 0; i < n; ++i) {
    Joint* joint = getMultiBodyModel(i)->toJoint();
    if (joint) {
      Log(ArtBody, Debug) << "Processing joint \"" << joint->getName()
                          << "\" to body \"" << getName()
                          << "\" for acceleration update" << endl;
      // Check if this is an articulated joint and if we are the parent.
      if (joint->isArticulatedJoint() && this == joint->getOutboardGroup())
        joint->updateAccels();
    }
  }
  
  Log(ArtBody, Debug3) << "On exit of computeAccel of \"" << getName()
                       << "\"" << endl;
}

void
RigidBody::addInteract(Interact* interact)
{
  mInteracts.push_back(interact);
}

bool
RigidBody::removeInteract(Interact* interact)
{
  InteractList::iterator it;
  for (it = mInteracts.begin(); it != mInteracts.end(); ++it) {
    if ((*it) == interact) {
      mInteracts.erase(it);
      return true;
    }
  }
  return false;
}

bool
RigidBody::addInteract2(Interact* child, unsigned parentNum)
{
  child->attachTo(this);
}

} // namespace OpenFDM
