/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "Joint.h"

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

void
Joint::output(const TaskInfo& taskInfo)
{
  RigidBody* outboardBody = getOutboardBody();
  if (!outboardBody)
    return;
  
  Log(ArtBody, Debug) << "Preparing Body \""
                      << outboardBody->getName() << "\" through joint \""
                      << getName() << "\"" << endl;

  // We need the articulated inertia and force from the outboard body.
  // Apply the joint degrees of freedom to that.
  // If there was an error, (something was singular ???)
  // just ignore that part. FIXME, ist this ok????
  jointArticulation(mInboardInertia, mInboardForce,
                    outboardBody->getArtInertia(),
                    outboardBody->getArtForce());

  Log(ArtBody, Debug3) << "Outboard Articulated values past joint "
                       << "projection: Force:\n" << trans(mInboardForce)
                       << "\nInertia\n" << mInboardInertia << endl;
}

bool
Joint::dependsDirectOn(Model* model)
{
  if (Interact::dependsDirectOn(model))
    return true;
  
  RigidBody* outboardBody = getOutboardBody();
  if (!outboardBody)
    return false;
  
  if (model == outboardBody)
    return true;
  
  Interact* interact = model->toInteract();
  if (!interact)
    return false;

  return interact != this && interact->isChildOf(outboardBody);
}

void
Joint::interactWith(RigidBody* rigidBody)
{
  // HMmMm
  if (!rigidBody)
    return;
  if (rigidBody != getInboardBody())
    return;
  
  RigidBody* outboardBody = getOutboardBody();
  if (!outboardBody)
    return;
  
  // Contribute the transformed values to the parent.
  Frame* frame = outboardBody->getFrame();
  rigidBody->contributeInertia(frame->inertiaToParent(mInboardInertia));
  rigidBody->contributeForce(frame->forceToParent(mInboardForce));
}

} // namespace OpenFDM
