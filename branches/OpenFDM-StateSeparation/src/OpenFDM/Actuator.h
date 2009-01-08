/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Actuator_H
#define OpenFDM_Actuator_H

#include "Assert.h"
#include "Types.h"
#include "Object.h"
#include "Model.h"

namespace OpenFDM {

// FIXME: implement!

class Actuator :
    public Model {
public:
  Actuator(const std::string& name) : Model(name)
  {
    mVel = 0;

    mMinDeflection = -Limits<real_type>::max();
    mMaxDeflection = Limits<real_type>::max();
    mMaxRate = Limits<real_type>::max();
    mOmega = 1.0/150;
    mDampingRatio = 0.7;
  }
  virtual ~Actuator() {}

  virtual void update(real_type dt)
  {
    real_type input; /* FIXME = mInput->getValue();*/

    // Apply the actuator limits.
    input = min(input, mMaxDeflection);
    input = max(input, mMinDeflection);

    // Current output
    real_type output = mOutput->getValue();

    // Build the error
    real_type error = input - output;
    
    // Apply some rate limit
    error = min(error, 2*mDampingRatio*mMaxRate/mOmega);
    error = max(error, -2*mDampingRatio*mMaxRate/mOmega);

    // compute acceleration
    real_type accel = error*mOmega*mOmega - 2*mOmega*mDampingRatio;

    // Compute the actuator velocity
    mVel += dt*accel;

    // Compute the actuator position.
    output += dt*mVel;

    // Apply the actuator limits.
    output = min(output, mMaxDeflection);
    output = max(output, mMinDeflection);
    
    // Set the output value
    mOutput->setValue(output);
  }

  void setMinPos(real_type m)
  { mMinDeflection = m; }
  real_type getMinPos(void) const
  { return mMinDeflection; }

  void setMaxPos(real_type m)
  { mMaxDeflection = m; }
  real_type getMaxPos(void) const
  { return mMaxDeflection; }

  void setMaxRate(real_type rate)
  { mMaxRate = rate; }
  real_type getMaxRate(void) const
  { return mMaxRate; }

  void setEigenFreq(real_type f)
  { mOmega = 1/f; }
  real_type getEigenFreq(void) const
  { return 1/mOmega; }

  void setDampingRatio(real_type damp)
  { mDampingRatio = damp; }
  real_type getDampingRatio(void) const
  { return mDampingRatio; }

private:
  real_type mVel;

  real_type mMinDeflection;
  real_type mMaxDeflection;
  real_type mMaxRate;
  real_type mOmega;
  real_type mDampingRatio;
};

} // namespace OpenFDM

#endif
