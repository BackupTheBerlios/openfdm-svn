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

class Joint : public MechanicNode {
  OPENFDM_OBJECT(Joint, MechanicNode);
public:
  Joint(const std::string& name);
  virtual ~Joint(void);

  virtual void accept(NodeVisitor& visitor);
  virtual void accept(ConstNodeVisitor& visitor) const;

  virtual MechanicContext* newMechanicContext(PortValueList& portValues) const = 0;
};

} // namespace OpenFDM

#endif
