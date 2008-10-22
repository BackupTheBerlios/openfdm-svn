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

namespace OpenFDM {

class RevoluteJoint : public Joint {
  OPENFDM_OBJECT(RevoluteJoint, Joint);
public:
  RevoluteJoint(const std::string& name);
  virtual ~RevoluteJoint(void);

  /** Sets the joint axis where this joint is allowed to rotate around.
   */
  const Vector3& getAxis() const;
  void setAxis(const Vector3& axis);

  /** Set the position of the joint.
   */
//   void setPosition(const Vector3& position);

  /** Sets the zero orientation of the joint.
   */
//   void setOrientation(const Quaternion& orientation);

protected:

  enum { n = 1 };
  typedef LinAlg::Vector<real_type,n> VectorN;
  typedef LinAlg::Matrix<real_type,6,n> Matrix6N;
  typedef LinAlg::Matrix<real_type,n,n> MatrixNN;
  typedef LinAlg::MatrixFactors<real_type,n,n,LinAlg::LUTag> MatrixFactorsNN;

  virtual void init(const Task&, DiscreteStateValueVector&,
                    ContinousStateValueVector& continousState,
                    const PortValueList&) const;

  virtual void velocity(const MechanicLinkValue& parentLink,
                        MechanicLinkValue& childLink,
                        const ContinousStateValueVector& states,
                        PortValueList& portValues,
                        FrameData& frameData) const;
  virtual void articulation(MechanicLinkValue& parentLink,
                            const MechanicLinkValue& childLink,
                            const ContinousStateValueVector& states,
                            PortValueList& portValues,
                            FrameData& frameData) const;
  virtual void acceleration(const MechanicLinkValue& parentLink,
                            MechanicLinkValue& childLink,
                            const ContinousStateValueVector& states,
                            PortValueList& portValues,
                            FrameData& frameData) const;

  virtual void derivative(const DiscreteStateValueVector&,
                          const ContinousStateValueVector&,
                          const PortValueList& portValues, FrameData&,
                          ContinousStateValueVector&) const;

private:
  MatrixInputPort mForcePort;
  MatrixOutputPort mPositionPort;
  MatrixOutputPort mVelocityPort;

  SharedPtr<Vector1StateInfo> mPositionStateInfo;
  SharedPtr<Vector1StateInfo> mVelocityStateInfo;

  Vector3 mAxis;

  Matrix6N mJointMatrix;
};

} // namespace OpenFDM

#endif
