/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
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

    if (1.0 < en)
      Log(TimeStep, Warning) << "DOPRI5: error too big: " << en << endl;
    else
      Log(TimeStep, Info) << "DOPRI5: local error: " << en << endl;

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
                         << "," << mTime << "]" << endl;
//     return false;
  }

  /// Compute dense output. That is
  real_type theta = (t - (mTime - mStepsize))/mStepsize;
  real_type theta1 = 1 - theta;
  out = mRCont[0] + theta*(mRCont[1] + theta1*(mRCont[2] + theta*(mRCont[3] + theta1*mRCont[4])));
  return true;
}

// The values of the Runge-Kutta tables
const real_type DoPri5::a21 = 2.0/10.0;
const real_type DoPri5::a31 = 3.0/40.0;
const real_type DoPri5::a32 = 9.0/40.0;
const real_type DoPri5::a41 = 44.0/45.0;
const real_type DoPri5::a42 = -56.0/15.0;
const real_type DoPri5::a43 = 32.0/9.0;
const real_type DoPri5::a51 = 19372.0/6561.0;
const real_type DoPri5::a52 = -25360.0/2187.0;
const real_type DoPri5::a53 = 64448.0/6561.0;
const real_type DoPri5::a54 = -212.0/729.0;
const real_type DoPri5::a61 = 9017.0/3168.0;
const real_type DoPri5::a62 = -355.0/33.0;
const real_type DoPri5::a63 = 46732.0/5247.0;
const real_type DoPri5::a64 = 49.0/176.0;
const real_type DoPri5::a65 = -5103.0/18656.0;
const real_type DoPri5::a71 = 35.0/384.0;
const real_type DoPri5::a73 = 500.0/1113.0;
const real_type DoPri5::a74 = 125.0/192.0;
const real_type DoPri5::a75 = -2187.0/6784.0;
const real_type DoPri5::a76 = 11.0/84.0;

const real_type DoPri5::c2 = 2.0/10.0;
const real_type DoPri5::c3 = 3.0/10.0;
const real_type DoPri5::c4 = 8.0/10.0;
const real_type DoPri5::c5 = 8.0/9.0;

const real_type DoPri5::d1 = -12715105075.0/11282082432.0;
const real_type DoPri5::d3 = 87487479700.0/32700410799.0;
const real_type DoPri5::d4 = -10690763975.0/1880347072.0;
const real_type DoPri5::d5 = 701980252875.0/199316789632.0;
const real_type DoPri5::d6 = -1453857185.0/822651844.0;
const real_type DoPri5::d7 = 69997945.0/29380423.0;

const real_type DoPri5::e1 = 71.0/57600.0;
const real_type DoPri5::e3 = -71.0/16695.0;
const real_type DoPri5::e4 = 71.0/1920.0;
const real_type DoPri5::e5 = -17253.0/339200.0;
const real_type DoPri5::e6 = 22.0/525.0;
const real_type DoPri5::e7 = -1.0/40.0;

} // namespace OpenFDM
