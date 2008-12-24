/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_UniversalJoint_H
#define OpenFDM_UniversalJoint_H

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

class UniversalJoint : public CartesianJoint<2> {
  OPENFDM_OBJECT(UniversalJoint, Joint);
public:
  UniversalJoint(const std::string& name);
  virtual ~UniversalJoint(void);

  /** Sets the joint axis where this joint is allowed to rotate around.
   */
  const Vector3& getAxis() const;
  void setAxis(const Vector3& axis);

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

  SharedPtr<Vector3StateInfo> mPositionStateInfo;
  SharedPtr<Vector2StateInfo> mVelocityStateInfo;

  Vector3 mAxis;
  Quaternion mOrientation;
};

} // namespace OpenFDM

#endif
