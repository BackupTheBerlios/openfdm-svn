/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_MechanicNode_H
#define OpenFDM_MechanicNode_H

#include <string>
#include "LeafNode.h"
#include "MechanicLink.h"

namespace OpenFDM {

class DiscreteTask;
class PortValueList;
class Task;
class MechanicContext;
class FrameData;

class MechanicNode : public LeafNode {
  OPENFDM_OBJECT(MechanicNode, LeafNode);
public:
  MechanicNode(const std::string& name);
  virtual ~MechanicNode();

  virtual void accept(NodeVisitor& visitor);
  virtual void accept(ConstNodeVisitor& visitor) const;

  virtual AbstractNodeContext* newNodeContext() const;
  virtual MechanicContext* newMechanicContext() const;

  virtual void velocity(const Task&, const ContinousStateValueVector&,
                        PortValueList&, FrameData&) const
  { }
  virtual void articulation(const Task&, const ContinousStateValueVector&,
                            PortValueList&, FrameData&) const
  { }
  // hmm, may be this should be output???
  virtual void acceleration(const Task&, const ContinousStateValueVector&,
                            PortValueList&, FrameData&) const
  { }
  virtual void derivative(const DiscreteStateValueVector&,
                          const ContinousStateValueVector&,
                          const PortValueList&, FrameData&,
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
