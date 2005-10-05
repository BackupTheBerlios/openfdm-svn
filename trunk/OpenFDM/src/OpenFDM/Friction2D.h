/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Friction2D_H
#define OpenFDM_Friction2D_H

#include "Assert.h"
#include "Model.h"
#include "Vector.h"
#include "Quaternion.h"

namespace OpenFDM {

class Friction2D :
    public Model {
public:
  Friction2D(const std::string& name) : Model(name) {}
  virtual ~Friction2D(void) {}

  

  /** Friction force computation.
   * Computes and returns the friction force dependent of the velocity.
   * The velocity is already projected in the plane. But the velocity is not
   * available in a 2d vector.
   * This form is sufficient for simple viscosous friction computations.
   * This function should be reimplemented for such simple friction models.
   * For friction models operating in real 2d, this function computes a 2d
   * velocity vector and calls the aprioriate function with 2d vectors.
   */
  virtual Vector3 getFrictionForce(const Vector3& planeNormal,
                                   const Vector3& vel, real_type normalForce,
                                   real_type surfaceCoef)
  {
    // Compute transform rotating the surface normal onto the z axis.
    // Note that this implementation provides quaternion which is a steady
    // function of the plane normal here.
    Rotation r = Quaternion::fromRotateTo(planeNormal, Vector3::unit(3));

    // Transform the velocity into the x/y plane.
    Vector3 v = r.transform(vel);

    // Compute the 2d friction force.
    Vector2 f = getFrictionForce(Vector2(v(1), v(2)), normalForce,
                                 surfaceCoef);

    // Return the friction force in the original coordinate directions.
    return r.backTransform(Vector3(f(1), f(2), 0));
  }

  /** Friction force computation.
   * The real 2d Version for the force computation.
   */
  virtual Vector2 getFrictionForce(const Vector2& vel, real_type normalForce,
                                   real_type surfaceCoef)
  {
    real_type frictionCoef = 0.8;
    real_type nVel = norm(vel);
    if (1 < nVel)
      return (-surfaceCoef*frictionCoef*normalForce/nVel)*vel;
    else
      return (-surfaceCoef*frictionCoef*normalForce)*vel;
  }
};

class WheelFriction :
    public Model {
public:
  WheelFriction(const std::string& name) : Model(name) {}
  virtual ~WheelFriction(void) {}

  /** Friction force computation.
   */
  virtual Vector2 getFrictionForce(const Vector2& vel, real_type omegaR, real_type normalForce, real_type surfaceCoef)
  {
    real_type frictionCoef = 0.8;
    real_type nVel = norm(vel);
    if (1 < nVel)
      return (-surfaceCoef*frictionCoef*normalForce/nVel)*vel;
    else
      return (-surfaceCoef*frictionCoef*normalForce)*vel;
  }
};

} // namespace OpenFDM

#endif
