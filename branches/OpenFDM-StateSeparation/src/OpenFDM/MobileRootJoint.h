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

  virtual JointContext*
  newJointContext(const Environment* environment,
                  MechanicLinkValue* parentLink,
                  MechanicLinkValue* childLink,
                  PortValueList& portValueList) const;

  const Vector3& getInitialPosition() const;
  void setInitialPosition(const Vector3& initialPosition);

  const Quaternion& getInitialOrientation() const;
  void setInitialOrientation(const Quaternion& initialOrientation);

  const Vector3& getInitialLinearVelocity() const;
  void setInitialLinearVelocity(const Vector3& initialLinearVelocity);

  const Vector3& getInitialAngularVelocity() const;
  void setInitialAngularVelocity(const Vector3& initialAngularVelocity);

  void init(const Task&, ContinousStateValueVector&) const;
  void velocity(const Task&, const Environment& environment,
                const ContinousStateValueVector& states, ChildLink&) const;
  void acceleration(const Task&, const Environment& environment,
                    const ContinousStateValueVector&, ChildLink&) const;
  void derivative(const Environment& environment,
                  const DiscreteStateValueVector&,
                  const ContinousStateValueVector&,
                  const ChildLink& childLink,
                  ContinousStateValueVector&) const;
private:
  class Context;

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
