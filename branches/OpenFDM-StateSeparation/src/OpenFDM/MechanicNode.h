/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_MechanicNode_H
#define OpenFDM_MechanicNode_H

#include <string>
#include "LeafNode.h"

namespace OpenFDM {

class PortValueList;

class MechanicNode : public LeafNode {
  OPENFDM_OBJECT(MechanicNode, LeafNode);
public:
  MechanicNode(const std::string& name);
  virtual ~MechanicNode();

  virtual void accept(NodeVisitor& visitor);

  virtual void velocity(const ContinousStateValueVector&,
                        PortValueList&) const
  { }
  virtual void articulation(const ContinousStateValueVector&,
                            PortValueList&) const
  { }
  virtual void derivative(const ContinousStateValueVector&,
                          const PortValueList&,
                          ContinousStateValueVector&) const
  { }
};

} // namespace OpenFDM

#endif
