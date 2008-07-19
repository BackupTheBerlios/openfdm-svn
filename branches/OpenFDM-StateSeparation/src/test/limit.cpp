/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include <iostream>
#include <fstream>
#include <OpenFDM/Types.h>
#include <OpenFDM/Limits.h>
#include <OpenFDM/Math.h>

using namespace OpenFDM;

static real_type estimateEps()
{
  // Ok, totally useless outputs, but makes sure that the floating point
  // registers get dumped out into memory. This actually works around that
  // totally boring i387 fpu ...
  std::ofstream dummy("/dev/null");

  real_type eps = 1;
  dummy << "Compute Epsilon" << std::endl;
  while (1 + eps != 1) {
    eps /= 2;
    dummy << eps << ", ";
  }
  eps *= 2;
  return eps;
}

static real_type estimateMin(const real_type& eps)
{
  // Ok, totally useless outputs, but makes sure that the floating point
  // registers get dumped out into memory. This actually works around that
  // totally boring i387 fpu ...
  std::ofstream dummy("/dev/null");

  real_type minTry = eps;
  real_type min = eps;
  while (minTry != 0) {
    min = minTry;
    minTry = ldexp(minTry, -128*128);
    dummy << minTry << ", ";
  }
  minTry = min;
  while (minTry != 0) {
    min = minTry;
    minTry = ldexp(minTry, -128);
    dummy << minTry << ", ";
  }
  minTry = min;
  while (minTry != 0) {
    min = minTry;
    minTry /= 2;
    dummy << minTry << ", ";
  }
  return min;
}

static real_type estimateMax(const real_type& eps)
{
  // Ok, totally useless outputs, but makes sure that the floating point
  // registers get dumped out into memory. This actually works around that
  // totally boring i387 fpu ...
  std::ofstream dummy("/dev/null");

  real_type maxTry = 1 - eps;
  real_type max = 1 - eps;
  while (isfinite(maxTry)) {
    max = maxTry;
    maxTry = ldexp(maxTry, 128*128);
    dummy << maxTry << ", ";
  }
  maxTry = max;
  while (isfinite(maxTry)) {
    max = maxTry;
    maxTry = ldexp(maxTry, 128);
    dummy << maxTry << ", ";
  }
  maxTry = max;
  while (isfinite(maxTry)) {
    max = maxTry;
    maxTry *= 2;
    dummy << maxTry << ", ";
  }
  return max;
}

int
main(int argc, char *argv[])
{
  real_type eps = estimateEps();
  std::cout << "Computed Eps = " << eps << std::endl;
  real_type min = estimateMin(eps);
  std::cout << "Computed Min = " << min << std::endl;
  real_type max = estimateMax(eps);
  std::cout << "Computed Max = " << max << std::endl;

  std::cout << "Epsilon = " << Limits<real_type>::epsilon() << std::endl;
  std::cout << "Min = " << Limits<real_type>::min() << std::endl;
  std::cout << "Safe Min = " << Limits<real_type>::safe_min() << std::endl;
  std::cout << "Max = " << Limits<real_type>::max() << std::endl;
  std::cout << "Round Error = " << Limits<real_type>::round_error() << std::endl;
  std::cout << "Infinity = " << Limits<real_type>::infinity() << std::endl;
  std::cout << "Quiet NaN = " << Limits<real_type>::quiet_NaN() << std::endl;
  std::cout << "Signaling NaN = " << Limits<real_type>::signaling_NaN() << std::endl;
  std::cout << "Denormlized Min = " << Limits<real_type>::denorm_min() << std::endl;

  if (eps != Limits<real_type>::epsilon())
    return EXIT_FAILURE;
  if (min != Limits<real_type>::min())
    return EXIT_FAILURE;
  if (max != Limits<real_type>::max())
    return EXIT_FAILURE;

  return EXIT_SUCCESS;
}
