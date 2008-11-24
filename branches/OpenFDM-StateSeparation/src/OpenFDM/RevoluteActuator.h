/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
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

  const Vector3& getPosition() const;
  void setPosition(const Vector3& position);

  const bool& getVelocityControl(void) const
  { return mVelocityControl; }
  void setVelocityControl(const bool& velocityControl)
  { mVelocityControl = velocityControl; }

  const real_type& getMaxVel(void) const
  { return mMaxVel; }
  void setMaxVel(const real_type& maxVel)
  { mMaxVel = maxVel; }

  const real_type& getVelGain(void) const
  { return mVelGain; }
  void setVelGain(const real_type& velGain)
  { mVelGain = velGain; }

  const real_type& getVelDotGain(void) const
  { return mVelDotGain; }
  void setVelDotGain(const real_type& velDotGain)
  { mVelDotGain = velDotGain; }

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
  bool mVelocityControl;
  real_type mVelGain;
  real_type mVelDotGain;
  real_type mMaxVel;
};

} // namespace OpenFDM

#endif
