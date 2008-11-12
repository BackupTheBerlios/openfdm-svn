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
  newMechanicContext(const MechanicLinkInfo* parentLink,
                     const MechanicLinkInfo* childLink,
                     PortValueList& portValueList) const;

  virtual void accept(NodeVisitor& visitor);
  virtual void accept(ConstNodeVisitor& visitor) const;

  const Vector3& getAngularBaseVelocity() const
  { return mAngularBaseVelocity; }
  void setAngularBaseVelocity(const Vector3& angularBaseVelocity)
  { mAngularBaseVelocity = angularBaseVelocity; }

  virtual void velocity(const Task&, const ContinousStateValueVector&,
                        PortValueList&) const = 0;
  virtual void articulation(const Task&, const ContinousStateValueVector&,
                            PortValueList&) const = 0;
  virtual void acceleration(const Task&, const ContinousStateValueVector&,
                            PortValueList&) const = 0;
  virtual void derivative(const DiscreteStateValueVector&,
                          const ContinousStateValueVector&,
                          const PortValueList& portValues,
                          ContinousStateValueVector&) const
  {}
private:
  class Context;

  Vector3 mAngularBaseVelocity;
};

} // namespace OpenFDM

#endif
