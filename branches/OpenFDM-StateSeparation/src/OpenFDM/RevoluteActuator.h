/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_RevoluteActuator_H
#define OpenFDM_RevoluteActuator_H

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

class RevoluteActuator : public CartesianJoint<1> {
  OPENFDM_OBJECT(RevoluteActuator, Joint);
public:
  RevoluteActuator(const std::string& name);
  virtual ~RevoluteActuator(void);

  /** Sets the joint axis where this joint is allowed to rotate around.
   */
  const Vector3& getAxis() const;
  void setAxis(const Vector3& axis);

  const real_type& getInitialPosition() const;
  void setInitialPosition(const real_type& initialPosition);

  const real_type& getInitialVelocity() const;
  void setInitialVelocity(const real_type& initialVelocity);

  const bool& getVelocityControl(void) const;
  void setVelocityControl(const bool& velocityControl);

  const real_type& getMaxVel(void) const;
  void setMaxVel(const real_type& maxVel);

  const real_type& getVelGain(void) const;
  void setVelGain(const real_type& velGain);

  const real_type& getVelDotGain(void) const;
  void setVelDotGain(const real_type& velDotGain);

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
  MatrixInputPort mInputPort;
  MatrixOutputPort mPositionPort;
  MatrixOutputPort mVelocityPort;

  SharedPtr<Vector1StateInfo> mPositionStateInfo;
  SharedPtr<Vector1StateInfo> mVelocityStateInfo;

  Vector3 mAxis;
  real_type mInitialPosition;
  real_type mInitialVelocity;
  bool mVelocityControl;
  real_type mVelGain;
  real_type mVelDotGain;
  real_type mMaxVel;
};

} // namespace OpenFDM

#endif
