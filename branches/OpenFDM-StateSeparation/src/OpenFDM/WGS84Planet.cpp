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

/// Implements the WGS84 ellipsoid as described in
///
/// [1] World Geodetic System 1984,
///     Its Definition and Relationships with Local Geodetic Systems
/// [2] H Moritz - Journal of Geodesy, 1980
///
/// The gravity model is so far the easiest one only valid for low altitudes
/// FIXME .. implement taylor series and the exact one ...


WGS84Planet::WGS84Planet(void)
{
  // values taken from simgear
  setAequatorialRadius(6378137.0);
  setFlattening(1/real_type(298.257223563));
  setGM(3986004.418e8);
  setOmega(7292115e-11);

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

const real_type&
WGS84Planet::getAequatorialRadius(void) const
{
  return a;
}

void
WGS84Planet::setAequatorialRadius(const real_type& r)
{
  a = r;
  a2 = a*a;
  ra2 = 1/a2;
}

const real_type&
WGS84Planet::getFlattening() const
{
  return f;
}

void
WGS84Planet::setFlattening(const real_type& flat)
{
  f = flat;
  real_type squash = 1 - f;
  e2 = fabs(1 - squash*squash);
  e = sqrt(e2);
  e4 = e2*e2;
}

const real_type&
WGS84Planet::getGM() const
{
  return GM;
}

void
WGS84Planet::setGM(const real_type& gm)
{
  GM = gm;
}

const real_type&
WGS84Planet::getOmega() const
{
  return mOmega;
}

void
WGS84Planet::setOmega(const real_type& omega)
{
  mOmega = omega;
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
WGS84Planet::getGravityAcceleration(const Vector3& position) const
{
  Geodetic geodetic = toGeod(position);

  real_type b = a - f*a;

  real_type m = mOmega*mOmega*a2*b/GM;

  // According to [2]
  real_type ePrime = e/sqrt(1 - e2);
  real_type q0 = 0.5*((1 + 3/(ePrime*ePrime))*atan(ePrime) - 3/ePrime);
  real_type q0Prime = 3*(1 + 1/(ePrime*ePrime))*(1 - 1/ePrime*atan(ePrime)) - 1;

  real_type gammaC = GM/(a*b)*(1 - m - m/6*(ePrime*q0Prime/q0));
  real_type gammaP = GM/a2*(1 + m/3*ePrime*q0Prime/q0);

  // According to [1]
  real_type k = b*gammaP/(a*gammaC) - 1;

  real_type sLat = sin(geodetic.latitude);
  real_type sLat2 = sLat*sLat;
  real_type gamma = gammaC*(1 + k*sLat2)/sqrt(1 - e2*sLat2);

  real_type h = geodetic.altitude;

  real_type gammaH = gamma*(1 - 2/a*(1 + f + m - 2*f*sLat2)*h + (3/a2)*h*h);

  /// FIXME This is not the whole story, The direction also varies ...
  Vector down = getGeodHLOrientation(geodetic).backTransform(Vector3::unit(2));

  /// FIXME
  /// Hmm, this includes the centrifugal force also,
  /// we need to get rid of that again since we handle the effect of the
  /// rotationg reference frame in an other way.
  return gammaH*down;
}

Vector3
WGS84Planet::getAngularVelocity(const real_type& t) const
{
  return Vector3(0, 0, mOmega);
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
