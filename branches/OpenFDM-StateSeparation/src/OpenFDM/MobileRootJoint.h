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

  const Vector3& getInitialPosition() const;
  void setInitialPosition(const Vector3& initialPosition);

  const Quaternion& getInitialOrientation() const;
  void setInitialOrientation(const Quaternion& initialOrientation);

  const Vector3& getInitialLinearVelocity() const;
  void setInitialLinearVelocity(const Vector3& initialLinearVelocity);

  const Vector3& getInitialAngularVelocity() const;
  void setInitialAngularVelocity(const Vector3& initialAngularVelocity);

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
  SharedPtr<MechanicLink> mMechanicLink;

  SharedPtr<Vector3StateInfo> mPositionStateInfo;
  SharedPtr<Vector4StateInfo> mOrientationStateInfo;
  SharedPtr<Vector6StateInfo> mVelocityStateInfo;

  Vector3 mInitialPosition;
  Quaternion mInitialOrientation;
  Vector3 mInitialLinearVelocity;
  Vector3 mInitialAngularVelocity;
};

} // namespace OpenFDM

#endif
