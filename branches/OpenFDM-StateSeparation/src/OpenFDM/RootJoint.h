/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_RootJoint_H
#define OpenFDM_RootJoint_H

#include <string>
#include "Joint.h"

namespace OpenFDM {

class RootJoint : public Joint {
  OPENFDM_OBJECT(RootJoint, Joint);
public:
  RootJoint(const std::string& name);
  virtual ~RootJoint();

  virtual MechanicContext*
  newMechanicContext(const Environment* environment,
                     const MechanicLinkInfo* parentLink,
                     const MechanicLinkInfo* childLink,
                     PortValueList& portValueList) const;

  virtual void accept(NodeVisitor& visitor);
  virtual void accept(ConstNodeVisitor& visitor) const;

  virtual void init(const Task&,DiscreteStateValueVector&,
                    ContinousStateValueVector&, const PortValueList&) const
  { }
  virtual void initDesignPosition(PortValueList&) const = 0;

  virtual void velocity(const Task&, const Environment& environment,
                        const ContinousStateValueVector&,
                        PortValueList&) const = 0;
  virtual void articulation(const Task&, const Environment& environment,
                            const ContinousStateValueVector&,
                            PortValueList&) const = 0;
  virtual void acceleration(const Task&, const Environment& environment,
                            const ContinousStateValueVector&,
                            PortValueList&) const = 0;
  virtual void derivative(const Environment& environment,
                          const DiscreteStateValueVector&,
                          const ContinousStateValueVector&,
                          const PortValueList& portValues,
                          ContinousStateValueVector&) const
  {}
private:
  class Context;
};

} // namespace OpenFDM

#endif
