/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Interact_H
#define OpenFDM_Interact_H

#include <string>
#include "MechanicNode.h"

namespace OpenFDM {

class Interact : public MechanicNode {
  OPENFDM_OBJECT(Interact, MechanicNode);
public:
  Interact(const std::string& name);
  virtual ~Interact();

  virtual void accept(NodeVisitor& visitor);
  virtual void accept(ConstNodeVisitor& visitor) const;

  virtual MechanicContext* newMechanicContext(PortValueList& portValues) const;

  virtual void initDesignPosition(PortValueList&) const = 0;
  virtual void velocity(const Task&, const ContinousStateValueVector&,
                        PortValueList&) const
  { }
  virtual void articulation(const Task&, const ContinousStateValueVector&,
                            PortValueList&, Matrix&) const
  { }
  virtual void acceleration(const Task&, const ContinousStateValueVector&,
                            PortValueList&, const Matrix&, Vector&) const
  { }
  virtual void derivative(const DiscreteStateValueVector&,
                          const ContinousStateValueVector&,
                          const PortValueList&, const Vector&,
                          ContinousStateValueVector&) const
  { }

private:
  class Context;
};

} // namespace OpenFDM

#endif
