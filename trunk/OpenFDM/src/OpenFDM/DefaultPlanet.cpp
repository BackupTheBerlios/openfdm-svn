/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "Types.h"
#include "Object.h"
#include "Units.h"
#include "Vector.h"
#include "Quaternion.h"
#include "Planet.h"
#include "DefaultPlanet.h"

namespace OpenFDM {

DefaultPlanet::DefaultPlanet(void)
  : mRotationRate(2.0*pi/(60.0*60.0*24.0))
{
  // values taken from simgear
  setAequatorialRadius(6378137.0);
  setFlattening(298.257223563);

  // Major semi axis or Aequatorial Radius is called a
  // Minor semi axis or polar radius is called b.
  // The the flattening f is defined by f = (a-b)/a.
  // For our earth these parameters are.
  //  a = 6378137.0
  //  b = 6356752.3142
  //  f = 1/298.257223563
}

DefaultPlanet::~DefaultPlanet(void)
{
}

real_type
DefaultPlanet::getAequatorialRadius(void) const
{
  return a;
}

void
DefaultPlanet::setAequatorialRadius(real_type r)
{
  a = r;
  ra2 = 1/(a*a);
}

void
DefaultPlanet::setFlattening(real_type flat)
{
  real_type squash = 1 - 1/flat;
  e2 = fabs(1 - squash*squash);
  e = sqrt(e2);
  e4 = e2*e2;
}

Geodetic
DefaultPlanet::toGeod(const Vector3& cart) const
{
  // according to
  // H. Vermeille,
  // Direct transformation from geocentric to geodetic ccordinates,
  // Journal of Geodesy (2002) 76:451-454
  real_type X = cart(1);
  real_type Y = cart(2);
  real_type Z = cart(3);
  real_type XXpYY = X*X+Y*Y;
  real_type sqrtXXpYY = sqrt(XXpYY);
  real_type p = XXpYY*ra2;
  real_type q = Z*Z*(1-e*e)*ra2;
  real_type r = 1.0/6.0*(p+q-e4);
  real_type s = e4*p*q/(4.0*r*r*r);
  real_type t = pow(1.0+s+sqrt(s*(2.0+s)), 1.0/3.0);
  real_type u = r*(1.0+t+1.0/t);
  real_type v = sqrt(u*u+e4*q);
  real_type w = e2*(u+v-q)/(2.0*v);
  real_type k = sqrt(u+v+w*w)-w;
  real_type D = k*sqrtXXpYY/(k+e2);
  real_type lambda = 2.0*atan2(Y, X+sqrtXXpYY);
  real_type sqrtDDpZZ = sqrt(D*D+Z*Z);
  real_type phi = 2.0*atan2(Z, D+sqrtDDpZZ);
  real_type h = (k+e2-1)*sqrtDDpZZ/k;
  return Geodetic(phi, lambda, h);
}

Vector3
DefaultPlanet::toCart(const Geodetic& geod) const
{
  // according to
  // H. Vermeille,
  // Direct transformation from geocentric to geodetic ccordinates,
  // Journal of Geodesy (2002) 76:451-454
  real_type lambda = geod.longitude;
  real_type phi = geod.latitude;
  real_type h = geod.altitude;
  real_type sphi = sin(phi);
  real_type n = a/sqrt(1.0-e2*sphi*sphi);
  real_type cphi = cos(phi);
  real_type slambda = sin(lambda);
  real_type clambda = cos(lambda);
  real_type X = (h+n)*cphi*clambda;
  real_type Y = (h+n)*cphi*slambda;
  real_type Z = (h+n-e2*n)*sphi;
  return Vector3(X, Y, Z);
}

} // namespace OpenFDM
