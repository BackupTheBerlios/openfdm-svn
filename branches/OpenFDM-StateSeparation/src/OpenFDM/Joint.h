/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
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

// May be each joint can be a root joint.
// A joint with one link is a root joint?
// The given position is then meant to be in the root coordinate system?
// That is: if connected to a parent, it is the designPosition wrt root joint
// If not connected, in the root coordinate system of the simulation?
// It would be nice if each joint is lockable anyway
// May be then unify all that joint stuff in a common framework here??
class Joint : public MechanicNode {
  OPENFDM_OBJECT(Joint, MechanicNode);
public:
  Joint(const std::string& name);
  virtual ~Joint(void);

  virtual void accept(NodeVisitor& visitor);
  virtual void accept(ConstNodeVisitor& visitor) const;

  virtual MechanicContext*
  newMechanicContext(const MechanicLinkInfo* parentLink,
                     const MechanicLinkInfo* childLink,
                     PortValueList& portValues) const = 0;
};

} // namespace OpenFDM

#endif
