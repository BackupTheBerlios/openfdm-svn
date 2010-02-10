/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_RotationalJoint_H
#define OpenFDM_RotationalJoint_H

#include "Assert.h"
#include "Object.h"
#include "Vector.h"
#include "Matrix.h"
#include "MatrixStateInfo.h"
#include "Quaternion.h"
#include "Inertia.h"
#include "Joint.h"
#include "MatrixInputPort.h"
#include "MatrixOutputPort.h"
#include "ContinousStateValueVector.h"
#include "PortValueList.h"
#include "MechanicContext.h"
#include "CartesianJoint.h"

namespace OpenFDM {

class RotationalJoint : public CartesianJoint<3> {
  OPENFDM_OBJECT(RotationalJoint, Joint);
public:
  RotationalJoint(const std::string& name);
  virtual ~RotationalJoint(void);

  const Quaternion& getInitialOrientation() const;
  void setInitialOrientation(const Quaternion& initialOrientation);

  const Vector3& getInitialVelocity() const;
  void setInitialVelocity(const Vector3& initialVelocity);

  void setEnableExternalForce(bool enable);
  bool getEnableExternalForce() const;

protected:

  virtual void init(const Task&, DiscreteStateValueVector&,
                    ContinousStateValueVector& continousState,
                    const PortValueList&) const;
  virtual Matrix6N getJointMatrix() const;

  virtual void velocity(const Task& task, Context& context,
                        const ContinousStateValueVector& states,
                        PortValueList& portValues) const;
  virtual void articulation(const Task& task, Context& context,
                            const ContinousStateValueVector& states,
                            PortValueList& portValues) const;
  virtual void acceleration(const Task& task, Context& context,
                            const ContinousStateValueVector& states,
                            PortValueList& portValues) const;
  virtual void derivative(const Task& task, Context& context,
                          const ContinousStateValueVector&,
                          const PortValueList& portValues,
                          ContinousStateValueVector&) const;

private:
  MatrixInputPort mForcePort;
  MatrixOutputPort mOrientationPort;
  MatrixOutputPort mVelocityPort;

  SharedPtr<Vector4StateInfo> mPositionStateInfo;
  SharedPtr<Vector3StateInfo> mVelocityStateInfo;

  Quaternion mInitialOrientation;
  Vector3 mInitialVelocity;
};

} // namespace OpenFDM

#endif
