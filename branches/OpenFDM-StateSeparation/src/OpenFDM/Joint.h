/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
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

#include "MechanicContext.h"

namespace OpenFDM {

class JointContext;

// May be each joint can be a root joint.
// A joint with one link is a root joint?

// It would be nice if each joint is lockable anyway
// May be then unify all that joint stuff in a common framework here??

// design position consitency:
// Each joint has a design positon property.
// This is for a root joint the position of the joint coorinate system of the
// mechanical system simulation and this is of limited external use.
// For a usual joint, this is an invariant point in the joint.
class Joint : public MechanicNode {
  OPENFDM_OBJECT(Joint, MechanicNode);
public:
  Joint(const std::string& name);
  virtual ~Joint(void);

  virtual void accept(NodeVisitor& visitor);
  virtual void accept(ConstNodeVisitor& visitor) const;

  virtual MechanicContext*
  newMechanicContext(const Environment* environment,
                     const MechanicLink* parentLink,
                     const MechanicLink* childLink,
                     PortValueList& portValueList) const;

  virtual JointContext*
  newJointContext(const Environment* environment,
                  MechanicLinkValue* parentLinkValue,
                  MechanicLinkValue* childLinkValue,
                  PortValueList& portValueList) const = 0;


  /** The design position of the joint.
   */
  const Vector3& getPosition() const;
  void setPosition(const Vector3& position);

private:
  Vector3 mPosition;
};

} // namespace OpenFDM

#endif
