/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "DoPri5.h"

#include "LogStream.h"
#include "Object.h"
#include "Vector.h"
#include "ODESolver.h"

namespace OpenFDM {

DoPri5::DoPri5(void)
{
}

DoPri5::~DoPri5(void)
{
}

bool
DoPri5::integrate(real_type toTEnd)
{
  real_type h = 0;
  while (!reached(toTEnd)) {
    real_type t = getTime();
    h = maxStepsize(toTEnd);

    // Compute the inner stages k1, ..., k7
    evalFunction(t, mState, k1);

    y2 = mState + (h*a21)*k1;
    evalFunction(t+c2*h, y2, k2);
    
    y3 = mState + (h*a31)*k1 + (h*a32)*k2;
    evalFunction(t+c3*h, y3, k3);
    
    y4 = mState + (h*a41)*k1 + (h*a42)*k2 + (h*a43)*k3;
    evalFunction(t+c4*h, y4, k4);
    
    y5 = mState + (h*a51)*k1 + (h*a52)*k2 + (h*a53)*k3 + (h*a54)*k4;
    evalFunction(t+c5*h, y5, k5);
    
    y6 = mState + (h*a61)*k1 + (h*a62)*k2 + (h*a63)*k3
      + (h*a64)*k4 + (h*a65)*k5;
    evalFunction(t+h, y6, k6);
    
    y7 = mState + (h*a71)*k1 + (h*a73)*k3 + (h*a74)*k4
      + (h*a75)*k5 + (h*a76)*k6;
    evalFunction(t+h, y7, k7);
    
    // The error estimate. Is the error to a 4-th order embedded scheme.
    err = (h*e1)*k1 + (h*e3)*k3 + (h*e4)*k4 + (h*e5)*k5
      + (h*e6)*k6 + (h*e7)*k7;

    real_type atol = 1e-10;
    real_type rtol = 1e-14;
    real_type en = scaledErr(y7, err, atol, rtol);

    if (1 < en)
      Log(TimeStep, Warning) << "DOPRI5: error too big: " << en << std::endl;
    else
      Log(TimeStep, Info) << "DOPRI5: local error: " << en << std::endl;

    // Need to save that here
    mRCont[0] = mState;

    // We do unconditionally accept all steps, y7 is the new mState value.
    mState = y7;
    mTime += h;
  }

  // Dense output ...
  mRCont[1] = mState - mRCont[0];
  mRCont[2] = h*k1 - mRCont[1];
  mRCont[3] = (-h)*k2 + mRCont[1] - mRCont[2];
  mRCont[4] = (h*d1)*k1 + (h*d3)*k3 + (h*d4)*k4 + (h*d5)*k5
    + (h*d6)*k6 + (h*d7)*k2;

  return true;
}

bool
DoPri5::denseOutput(real_type t, Vector& out)
{
  if (t < mTime - mStepsize || mTime < t) {
    Log(TimeStep, Error) << "Request for dense output at t = " << t
                         << " out of range [" << mTime - mStepsize
                         << "," << mTime << "]" << std::endl;
//     return false;
  }

  /// Compute dense output. That is
  real_type theta = (t - (mTime - mStepsize))/mStepsize;
  real_type theta1 = 1 - theta;
  out = mRCont[0] + theta*(mRCont[1] + theta1*(mRCont[2] + theta*(mRCont[3] + theta1*mRCont[4])));
  return true;
}

// The values of the Runge-Kutta tables
const real_type DoPri5::a21 = real_type(2.0)/real_type(10.0);
const real_type DoPri5::a31 = real_type(3.0)/real_type(40.0);
const real_type DoPri5::a32 = real_type(9.0)/real_type(40.0);
const real_type DoPri5::a41 = real_type(44.0)/real_type(45.0);
const real_type DoPri5::a42 = real_type(-56.0)/real_type(15.0);
const real_type DoPri5::a43 = real_type(32.0)/real_type(9.0);
const real_type DoPri5::a51 = real_type(19372.0)/real_type(6561.0);
const real_type DoPri5::a52 = real_type(-25360.0)/real_type(2187.0);
const real_type DoPri5::a53 = real_type(64448.0)/real_type(6561.0);
const real_type DoPri5::a54 = real_type(-212.0)/real_type(729.0);
const real_type DoPri5::a61 = real_type(9017.0)/real_type(3168.0);
const real_type DoPri5::a62 = real_type(-355.0)/real_type(33.0);
const real_type DoPri5::a63 = real_type(46732.0)/real_type(5247.0);
const real_type DoPri5::a64 = real_type(49.0)/real_type(176.0);
const real_type DoPri5::a65 = real_type(-5103.0)/real_type(18656.0);
const real_type DoPri5::a71 = real_type(35.0)/real_type(384.0);
const real_type DoPri5::a73 = real_type(500.0)/real_type(1113.0);
const real_type DoPri5::a74 = real_type(125.0)/real_type(192.0);
const real_type DoPri5::a75 = real_type(-2187.0)/real_type(6784.0);
const real_type DoPri5::a76 = real_type(11.0)/real_type(84.0);

const real_type DoPri5::c2 = real_type(2.0)/real_type(10.0);
const real_type DoPri5::c3 = real_type(3.0)/real_type(10.0);
const real_type DoPri5::c4 = real_type(8.0)/real_type(10.0);
const real_type DoPri5::c5 = real_type(8.0)/real_type(9.0);

const real_type DoPri5::d1 = real_type(-12715105075.0)/real_type(11282082432.0);
const real_type DoPri5::d3 = real_type(87487479700.0)/real_type(32700410799.0);
const real_type DoPri5::d4 = real_type(-10690763975.0)/real_type(1880347072.0);
const real_type DoPri5::d5 = real_type(701980252875.0)/real_type(199316789632.0);
const real_type DoPri5::d6 = real_type(-1453857185.0)/real_type(822651844.0);
const real_type DoPri5::d7 = real_type(69997945.0L)/real_type(29380423.0);

const real_type DoPri5::e1 = real_type(71.0)/real_type(57600.0);
const real_type DoPri5::e3 = real_type(-71.0)/real_type(16695.0);
const real_type DoPri5::e4 = real_type(71.0)/real_type(1920.0);
const real_type DoPri5::e5 = real_type(-17253.0)/real_type(339200.0);
const real_type DoPri5::e6 = real_type(22.0)/real_type(525.0);
const real_type DoPri5::e7 = real_type(-1.0)/real_type(40.0);

} // namespace OpenFDM
