/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "EllipticPlanet.h"

#include "Environment.h"
#include "Types.h"
#include "Unit.h"
#include "Vector.h"
#include "Quaternion.h"

namespace OpenFDM {

EllipticPlanet::EllipticPlanet(void)
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

EllipticPlanet::~EllipticPlanet(void)
{
}

real_type
EllipticPlanet::getAequatorialRadius(void) const
{
  return a;
}

void
EllipticPlanet::setAequatorialRadius(real_type r)
{
  a = r;
  ra2 = 1/(a*a);
}

void
EllipticPlanet::setFlattening(real_type flat)
{
  real_type squash = 1 - 1/flat;
  e2 = fabs(1 - squash*squash);
  e = sqrt(e2);
  e4 = e2*e2;
}

Plane
EllipticPlanet::getHorizont(const Vector3& position) const
{
  Geodetic geodetic = toGeod(position);
  geodetic.altitude = 0;
  Vector down = getGeodHLOrientation(geodetic).backTransform(Vector3::unit(2));
  return Plane(down, toCart(geodetic));
}

Vector3
EllipticPlanet::getGravityAcceleration(const Vector3& cart) const
{
  real_type dist = norm(cart);
  return (-5.9742e24*gravity_constant/(dist*dist*dist))*cart;
}

Vector3
EllipticPlanet::getAngularVelocity(const real_type& t) const
{
  return Vector3(0, 0, pi2/(24*60*60));
}

Vector6
EllipticPlanet::getAcceleration(const real_type& t) const
{
  return Vector6::zeros();
}

Geodetic
EllipticPlanet::toGeod(const Vector3& cart) const
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
EllipticPlanet::toCart(const Geodetic& geod) const
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

Geocentric
EllipticPlanet::toGeoc(const Vector3& cart) const
{
  real_type lon = (cart(0) == 0 && cart(1) == 0)
    ? real_type(0) : atan2(cart(1), cart(0));
  real_type nxy = sqrt(cart(0)*cart(0)+cart(1)*cart(1));
  real_type lat = (nxy == 0 && cart(2) == 0)
    ? real_type(0) : atan2(cart(2), nxy);
  return Geocentric(lat, lon, norm(cart));
}

Vector3
EllipticPlanet::toCart(const Geocentric& geoc) const
{
  real_type slat = sin(geoc.latitude);
  real_type clat = cos(geoc.latitude);
  real_type slon = sin(geoc.longitude);
  real_type clon = cos(geoc.longitude);
  return geoc.radius*Vector3( clat*clon, clat*slon, slat );
}

Geocentric
EllipticPlanet::toGeoc(const Geodetic& geod) const
{
  return toGeoc(toCart(geod));
}

Geodetic
EllipticPlanet::toGeod(const Geocentric& geoc) const
{
  return toGeod(toCart(geoc));
}

Quaternion
EllipticPlanet::getGeodHLOrientation(const Geodetic& pos) const
{
  return Quaternion::fromLonLat(pos.longitude, pos.latitude);
}

Quaternion
EllipticPlanet::getGeodHLOrientation(const Vector3& pos) const
{
  return getGeodHLOrientation(toGeod(pos));
}

Quaternion
EllipticPlanet::getGeodHLOrientation(const Geocentric& pos) const
{
  return getGeodHLOrientation(toCart(pos));
}

Quaternion
EllipticPlanet::getGeocHLOrientation(const Geodetic& pos) const
{
  return getGeocHLOrientation(toCart(pos));
}

Quaternion
EllipticPlanet::getGeocHLOrientation(const Vector3& pos) const
{
  return getGeocHLOrientation(toGeoc(pos));
}

Quaternion
EllipticPlanet::getGeocHLOrientation(const Geocentric& pos) const
{
  return Quaternion::fromLonLat(pos.longitude, pos.latitude);
}

Vector3
EllipticPlanet::getGoecHLRate(const Geocentric& pos, const Vector3& ecVel) const
{
  Quaternion hlOrientation = getGeocHLOrientation(pos);
  Vector3 hlVel = hlOrientation.transform(ecVel);
  Vector3 hlRate = Vector3(hlVel(1), -hlVel(0), -hlVel(1)*tan(pos.latitude));
  return hlOrientation.backTransform((1/pos.radius)*hlRate);
}

Vector3
EllipticPlanet::getGoecHLRate(const Vector3& pos, const Vector3& ecVel) const
{
  return getGoecHLRate(toGeoc(pos), ecVel);
}

Vector3
EllipticPlanet::getGoecHLRate(const Geodetic& pos, const Vector3& ecVel) const
{
  return getGoecHLRate(toCart(pos), ecVel);
}

std::ostream&
operator<<(std::ostream& os, const Geodetic& geod)
{
  return os << "[ lon = " << rad2deg*geod.longitude
            << ", lat = " << rad2deg*geod.latitude
            << ", alt = " << geod.altitude
            << " ]";
}

std::ostream&
operator<<(std::ostream& os, const Geocentric& geoc)
{
  return os << "[ lon = " << rad2deg*geoc.longitude
            << ", lat = " << rad2deg*geoc.latitude
            << ", rad = " << geoc.radius
            << " ]";
}

} // namespace OpenFDM
