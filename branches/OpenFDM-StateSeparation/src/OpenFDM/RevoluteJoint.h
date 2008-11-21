/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_RevoluteJoint_H
#define OpenFDM_RevoluteJoint_H

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

class RevoluteJoint : public CartesianJoint<1> {
  OPENFDM_OBJECT(RevoluteJoint, Joint);
public:
  RevoluteJoint(const std::string& name);
  virtual ~RevoluteJoint(void);

  /** Sets the joint axis where this joint is allowed to rotate around.
   */
  const Vector3& getAxis() const;
  void setAxis(const Vector3& axis);

  const Vector3& getPosition() const;
  void setPosition(const Vector3& position);

  void setEnableExternalForce(bool enable);
  bool getEnableExternalForce() const;

protected:

  virtual void initDesignPosition(const MechanicLinkValue& parentLink,
                                  MechanicLinkValue& childLink) const;
  virtual void init(const Task&, DiscreteStateValueVector&,
                    ContinousStateValueVector& continousState,
                    const PortValueList&) const;

  virtual void velocity(const MechanicLinkValue& parentLink,
                        MechanicLinkValue& childLink,
                        const ContinousStateValueVector& states,
                        PortValueList& portValues) const;
  virtual void articulation(MechanicLinkValue& parentLink,
                            const MechanicLinkValue& childLink,
                            const ContinousStateValueVector& states,
                            PortValueList& portValues,
                            MatrixFactorsNN& hIh, Vector6& pAlpha) const;
  virtual void acceleration(const MechanicLinkValue& parentLink,
                            MechanicLinkValue& childLink,
                            const ContinousStateValueVector& states,
                            PortValueList& portValues,
                            const MatrixFactorsNN& hIh, const Vector6& pAlpha,
                            VectorN& velDot) const;

  virtual void derivative(const DiscreteStateValueVector&,
                          const ContinousStateValueVector&,
                          const PortValueList& portValues,
                          const VectorN& velDot,
                          ContinousStateValueVector&) const;

  using CartesianJoint<1>::velocity;
  using CartesianJoint<1>::articulation;
  using CartesianJoint<1>::acceleration;

private:
  MatrixInputPort mForcePort;
  MatrixOutputPort mPositionPort;
  MatrixOutputPort mVelocityPort;

  SharedPtr<Vector1StateInfo> mPositionStateInfo;
  SharedPtr<Vector1StateInfo> mVelocityStateInfo;

  Vector3 mAxis;
  Vector3 mPosition;
};

} // namespace OpenFDM

#endif
