/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
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
  virtual void velocity(const Task&, const ContinousStateValueVector& states,
                        PortValueList& portValues, FrameData&) const;
  virtual void articulation(const Task&, const ContinousStateValueVector&,
                            PortValueList& portValues, FrameData&) const;
  virtual void acceleration(const Task&, const ContinousStateValueVector&,
                            PortValueList& portValues, FrameData&) const;
  virtual void derivative(const DiscreteStateValueVector&,
                          const ContinousStateValueVector&,
                          const PortValueList& portValues, FrameData&,
                          ContinousStateValueVector&) const;
private:
  MechanicLink mMechanicLink;

  SharedPtr<Vector3StateInfo> mPositionStateInfo;
  SharedPtr<Vector4StateInfo> mOrientationStateInfo;
  SharedPtr<Vector6StateInfo> mVelocityStateInfo;
};

} // namespace OpenFDM

#endif
