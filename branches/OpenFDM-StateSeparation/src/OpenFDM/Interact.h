/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Interact_H
#define OpenFDM_Interact_H

#include <string>
#include "MechanicInteractPort.h"
#include "MechanicNode.h"

namespace OpenFDM {

class Interact : public MechanicNode {
  OPENFDM_OBJECT(Interact, MechanicNode);
public:
  Interact(const std::string& name);
  virtual ~Interact();

  virtual void accept(NodeVisitor& visitor);
  virtual void accept(ConstNodeVisitor& visitor) const;

protected:
  MechanicInteractPort newMechanicInteractPort(const std::string& name)
  { return MechanicInteractPort(this, name); }
};

} // namespace OpenFDM

#endif
