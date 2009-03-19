/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Actuator_H
#define OpenFDM_Actuator_H

#include "Assert.h"
#include "Types.h"
#include "Object.h"
#include "Model.h"
#include "RealStateInfo.h"

namespace OpenFDM {

// FIXME: implement!

class Actuator :
    public Model {
public:
  Actuator(const std::string& name);
  virtual ~Actuator();

  virtual void init(const Task&, DiscreteStateValueVector& discreteState,
                    ContinousStateValueVector& continousState,
                    const PortValueList& portValueList) const;
  virtual void output(const Task&,const DiscreteStateValueVector&,
                      const ContinousStateValueVector& continousState,
                      PortValueList& portValues) const;
  virtual void derivative(const DiscreteStateValueVector&,
                          const ContinousStateValueVector& state,
                          const PortValueList& portValues,
                          ContinousStateValueVector& deriv) const;

  void setMinPos(const real_type& m)
  { mMinDeflection = m; }
  const real_type& getMinPos(void) const
  { return mMinDeflection; }

  void setMaxPos(const real_type& m)
  { mMaxDeflection = m; }
  const real_type& getMaxPos(void) const
  { return mMaxDeflection; }

  void setMaxRate(const real_type& rate)
  { mMaxRate = rate; }
  const real_type& getMaxRate(void) const
  { return mMaxRate; }

  void setEigenFreq(const real_type& f)
  { mEigenFreq = f; }
  real_type getEigenFreq(void) const
  { return mEigenFreq; }

  void setDampingRatio(const real_type& damp)
  { mDampingRatio = damp; }
  const real_type& getDampingRatio(void) const
  { return mDampingRatio; }

private:
  RealInputPort mInputPort;
  RealOutputPort mPositionPort;
  RealOutputPort mVelocityPort;

  SharedPtr<Vector1StateInfo> mPositionStateInfo;
  SharedPtr<Vector1StateInfo> mVelocityStateInfo;

  real_type mInitialPosition;
  real_type mInitialVelocity;
  real_type mMinDeflection;
  real_type mMaxDeflection;
  real_type mMaxRate;
  real_type mEigenFreq;
  real_type mDampingRatio;
};

} // namespace OpenFDM

#endif
