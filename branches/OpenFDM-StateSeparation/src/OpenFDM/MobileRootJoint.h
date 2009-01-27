/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_MobileRootJoint_H
#define OpenFDM_MobileRootJoint_H

#include "Assert.h"
#include "Object.h"
#include "Vector.h"
#include "Matrix.h"
#include "MatrixStateInfo.h"
#include "Quaternion.h"
#include "Inertia.h"
#include "RootJoint.h"

namespace OpenFDM {

class MobileRootJoint : public RootJoint {
  OPENFDM_OBJECT(MobileRootJoint, RootJoint);
public:
  MobileRootJoint(const std::string& name);
  virtual ~MobileRootJoint();

  virtual void init(const Task&, DiscreteStateValueVector&,
                    ContinousStateValueVector&,
                    const PortValueList&) const;
  virtual void initDesignPosition(PortValueList&) const;
  virtual void velocity(const Task&, const Environment& environment,
                        const ContinousStateValueVector& states,
                        PortValueList& portValues) const;
  virtual void articulation(const Task&, const Environment& environment,
                            const ContinousStateValueVector&,
                            PortValueList& portValues) const;
  virtual void acceleration(const Task&, const Environment& environment,
                            const ContinousStateValueVector&,
                            PortValueList& portValues) const;
  virtual void derivative(const Environment& environment,
                          const DiscreteStateValueVector&,
                          const ContinousStateValueVector&,
                          const PortValueList& portValues,
                          ContinousStateValueVector&) const;
private:
  MechanicLink_ mMechanicLink;

  SharedPtr<Vector3StateInfo> mPositionStateInfo;
  SharedPtr<Vector4StateInfo> mOrientationStateInfo;
  SharedPtr<Vector6StateInfo> mVelocityStateInfo;
};

} // namespace OpenFDM

#endif
