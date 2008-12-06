/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "WGS84Planet.h"

#include "Environment.h"
#include "Types.h"
#include "Unit.h"
#include "Vector.h"
#include "Quaternion.h"

namespace OpenFDM {

WGS84Planet::WGS84Planet(void)
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

WGS84Planet::~WGS84Planet(void)
{
}

real_type
WGS84Planet::getAequatorialRadius(void) const
{
  return a;
}

void
WGS84Planet::setAequatorialRadius(real_type r)
{
  a = r;
  ra2 = 1/(a*a);
}

void
WGS84Planet::setFlattening(real_type flat)
{
  real_type squash = 1 - 1/flat;
  e2 = fabs(1 - squash*squash);
  e = sqrt(e2);
  e4 = e2*e2;
}

Plane
WGS84Planet::getHorizont(const Vector3& position) const
{
  Geodetic geodetic = toGeod(position);
  geodetic.altitude = 0;
  Vector down = getGeodHLOrientation(geodetic).backTransform(Vector3::unit(2));
  return Plane(down, toCart(geodetic));
}

Vector3
WGS84Planet::getGravityAcceleration(const Vector3& cart) const
{
  // FIXME, this is not WGS84!!!
  Geodetic geodetic = toGeod(cart);
  Vector down = getGeodHLOrientation(geodetic).backTransform(Vector3::unit(2));
  real_type dist2 = dot(cart, cart);
  return (-5.9742e24*gravity_constant/(dist2))*down;
}

Vector3
WGS84Planet::getAngularVelocity(const real_type& t) const
{
  return Vector3(0, 0, pi2/(24*60*60));
}

Vector6
WGS84Planet::getAcceleration(const real_type& t) const
{
  return Vector6::zeros();
}

Geodetic
WGS84Planet::toGeod(const Vector3& cart) const
{
  // according to
  // H. Vermeille,
  // Direct transformation from geocentric to geodetic ccordinates,
  // Journal of Geodesy (2002) 76:451-454
  real_type X = cart(0);
  real_type Y = cart(1);
  real_type Z = cart(2);
  real_type XXpYY = X*X+Y*Y;
  real_type sqrtXXpYY = sqrt(XXpYY);
  real_type p = XXpYY*ra2;
  real_type q = Z*Z*(1-e*e)*ra2;
  real_type r = 1.0/6.0*(p+q-e4);
  real_type s = e4*p*q/(4.0*r*r*r);
  real_type t = pow(1.0+s+sqrt(s*(2.0+s)), real_type(1)/3);
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
WGS84Planet::toCart(const Geodetic& geod) const
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

Quaternion
WGS84Planet::getGeodHLOrientation(const Geodetic& pos) const
{
  return Quaternion::fromLonLat(pos.longitude, pos.latitude);
}

Quaternion
WGS84Planet::getGeodHLOrientation(const Vector3& pos) const
{
  return getGeodHLOrientation(toGeod(pos));
}

std::ostream&
operator<<(std::ostream& os, const Geodetic& geod)
{
  return os << "[ lon = " << rad2deg*geod.longitude
            << ", lat = " << rad2deg*geod.latitude
            << ", alt = " << geod.altitude
            << " ]";
}

} // namespace OpenFDM
