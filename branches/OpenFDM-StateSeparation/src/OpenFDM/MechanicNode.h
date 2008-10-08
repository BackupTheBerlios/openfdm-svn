/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_MechanicNode_H
#define OpenFDM_MechanicNode_H

#include <string>
#include "LeafNode.h"
#include "MechanicLink.h"

namespace OpenFDM {

class PortValueList;
class DiscreteTask;

class MechanicNode : public LeafNode {
  OPENFDM_OBJECT(MechanicNode, LeafNode);
public:
  MechanicNode(const std::string& name);
  virtual ~MechanicNode();

  virtual void accept(NodeVisitor& visitor);
  virtual void accept(ConstNodeVisitor& visitor) const;

  virtual void velocity(const ContinousStateValueVector&,
                        PortValueList&) const
  { }
  virtual void articulation(const ContinousStateValueVector&,
                            PortValueList&) const
  { }
  virtual void derivative(const DiscreteStateValueVector&,
                          const ContinousStateValueVector&,
                          const PortValueList&,
                          ContinousStateValueVector&) const
  { }
  virtual void update(const DiscreteTask&, DiscreteStateValueVector&,
                      const ContinousStateValueVector&,
                      const PortValueList&) const
  { }
protected:
  MechanicLink newMechanicLink(const std::string& name)
  { return MechanicLink(this, name); }
};

} // namespace OpenFDM

#endif
