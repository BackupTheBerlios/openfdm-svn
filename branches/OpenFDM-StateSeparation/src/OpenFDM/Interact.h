/* -*-c++-*- OpenFDM - Copyright (C) 2007-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Interact_H
#define OpenFDM_Interact_H

#include <string>
#include "MechanicNode.h"

namespace OpenFDM {

class ConstNodeVisitor;
class Environment;
class MechanicContext;
class NodeVisitor;
class PortValueList;

class Interact : public MechanicNode {
  OPENFDM_OBJECT(Interact, MechanicNode);
public:
  Interact(const std::string& name);
  virtual ~Interact();

  virtual void accept(NodeVisitor& visitor);
  virtual void accept(ConstNodeVisitor& visitor) const;

  virtual MechanicContext*
  newMechanicContext(const Environment*, PortValueList&) const = 0;
};

} // namespace OpenFDM

#endif
