/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

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
#include "ModelVisitor.h"
#include "Joint.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Joint, Interact)
  END_OPENFDM_OBJECT_DEF

Joint::Joint(const std::string& name)
  : Interact(name, 2)
{
}

Joint::~Joint(void)
{
}

void
Joint::accept(ModelVisitor& visitor)
{
  visitor.apply(*this);
}

const Joint*
Joint::toJoint(void) const
{
  return this;
}

Joint*
Joint::toJoint(void)
{
  return this;
}

void
Joint::output(const TaskInfo& taskInfo)
{
  RigidBody* outboardBody = getOutboardBody();
  if (!outboardBody)
    return;
  
  Log(ArtBody, Debug) << "Preparing Body \""
                      << outboardBody->getName() << "\" through joint \""
                      << getName() << "\"" << endl;
  
  outboardBody->computeArtValues();
}

void
Joint::interactWith(RigidBody* rigidBody)
{
  // HMmMm
  if (rigidBody != getInboardBody())
    return;
  
  RigidBody* outboardBody = getOutboardBody();
  if (!outboardBody)
    return;
  
  Log(ArtBody, Debug) << "Contributing articulation from \""
                      << outboardBody->getName() << "\" through joint \""
                      << getName() << "\"" << endl;
  
  // We need the articulated inertia and force from the outboard body.
  SpatialInertia I;
  Vector6 F;
  
  // Apply the joint degrees of freedom to that.
  // If there was an error, (something was singular ???)
  // just ignore that part. FIXME, ist this ok????
  jointArticulation(I, F, outboardBody->getArtInertia(),
                    outboardBody->getArtForce());
  
  Log(ArtBody, Debug3) << "Outboard Articulated values past joint "
                       << "projection: Force:\n" << trans(F)
                       << "\nInertia\n" << I << endl;
  
  // Contribute the transformed values to the parent.
  if (!rigidBody)
    return;
  Frame* frame = outboardBody->getFrame();
  rigidBody->contributeInertia(frame->inertiaToParent(I));
  rigidBody->contributeForce(frame->forceToParent(F));
}

} // namespace OpenFDM
