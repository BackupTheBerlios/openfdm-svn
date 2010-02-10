/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_DynamicPressure_H
#define OpenFDM_DynamicPressure_H

#include <string>

#include "Types.h"
#include "Model.h"

namespace OpenFDM {

class DynamicPressure : public Model {
  OPENFDM_OBJECT(DynamicPressure, Model);
public:
  DynamicPressure(const std::string& name);
  virtual ~DynamicPressure(void);

  virtual void output(const Task&, const DiscreteStateValueVector&,
                      const ContinousStateValueVector&,
                      PortValueList& portValues) const;

  const Vector3& getDirection() const
  { return mDirection; }
  void setDirection(const Vector3& direction)
  { mDirection = direction; }

private:
  /// The input port which must provide the fluid velocity vector
  MatrixInputPort mVelocityPort;
  /// The input port which must provide the sensed density
  RealInputPort mDensityPort;
  /// The resulting dynamic pressure
  RealOutputPort mDynamicPressurePort;

  /// The direction vector for the dynamic pressure.
  /// If set to zero, the dynamic pressure into the wind direction is measured
  Vector3 mDirection;
};

} // namespace OpenFDM

#endif
