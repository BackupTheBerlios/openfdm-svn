/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_MobileRootJoint_H
#define OpenFDM_MobileRootJoint_H

#include "Assert.h"
#include "Object.h"
#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "Inertia.h"
#include "RootJoint.h"

namespace OpenFDM {

class MobileRootJoint : public RootJoint {
  OPENFDM_OBJECT(MobileRootJoint, RootJoint);
public:
  MobileRootJoint(const std::string& name);
  virtual ~MobileRootJoint();

  virtual void velocity(const Task&, const ContinousStateValueVector& states,
                        PortValueList& portValues) const;
  virtual void articulation(const Task&, const ContinousStateValueVector&,
                            PortValueList& portValues) const;
  virtual void acceleration(const Task&, const ContinousStateValueVector&,
                            PortValueList& portValues) const;
  virtual void derivative(const DiscreteStateValueVector&,
                          const ContinousStateValueVector&,
                          const PortValueList& portValues,
                          ContinousStateValueVector&) const;
private:
  MechanicLink mMechanicLink;

  SharedPtr<MatrixStateInfo> mPositionStateInfo;
  SharedPtr<MatrixStateInfo> mOrientationStateInfo;
  SharedPtr<MatrixStateInfo> mVelocityStateInfo;
};

} // namespace OpenFDM

#endif
